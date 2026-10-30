/* Lane C reference runtime: feeds recorded frames + IMU into the ORIGINAL reference runtime SDK through public browser
 * APIs (MediaStreamTrackGenerator, untrusted DeviceMotion/DeviceOrientation events) and records the public
 * XrController output per callback.  Adapted from scripts/dev/references/xr_reference/experiment-runtime.js
 * (moving arm only) with: lazy frame fetch (long sequences), pre/post timing modules around XrController,
 * reality.* event listeners and world-point id churn.  No SDK bytes or Wasm are modified here. */
(async () => {
  const state = window.__probe;
  const fixture = await (await fetch('/fixture.json')).json();
  const {width, height} = fixture.config;
  const frameCount = fixture.frames.length;
  const prefetchAhead = 48;
  const cache = new Map();
  const load = index => {
    if (index >= frameCount || cache.has(index)) return;
    cache.set(index, fetch('/gray/' + index).then(r => r.arrayBuffer()).then(b => new Uint8Array(b)));
  };
  const source = document.createElement('canvas');
  source.width = width; source.height = height;
  const ctx = source.getContext('2d', {willReadFrequently: true});
  const image = ctx.createImageData(width, height);
  const draw = gray => {
    for (let p = 0; p < gray.length; p++) {
      image.data[4 * p] = gray[p]; image.data[4 * p + 1] = gray[p]; image.data[4 * p + 2] = gray[p]; image.data[4 * p + 3] = 255;
    }
    ctx.putImageData(image, 0, 0);
  };
  for (let i = 0; i < prefetchAhead; i++) load(i);
  const gray0 = await cache.get(0);
  draw(gray0);
  let writer = null, stream, track;
  const ownedTracks = [];
  const createStream = () => {
    track = new MediaStreamTrackGenerator({kind: 'video'});
    writer = track.writable.getWriter();
    writer.closed.catch(error => state.events.push({name: 'writerClosed', at: performance.now(), message: String(error)}));
    stream = new MediaStream([track]);
    ownedTracks.push(track);
    const settings = track.getSettings.bind(track);
    Object.defineProperty(track, 'getSettings', {value: () => ({...settings(), width, height, frameRate: fixture.nominalFps, facingMode: 'environment'})});
  };
  createStream();
  navigator.mediaDevices.getUserMedia = async constraints => {
    state.cameraRequests.push({constraints, at: performance.now(), previousTrackState: track.readyState});
    if (track.readyState === 'ended') createStream();
    return stream;
  };
  const timers = new Set();
  let epoch = null;
  const schedule = (fn, ms) => { const id = setTimeout(() => { timers.delete(id); fn(); }, ms); timers.add(id); };
  // Opt-in (architecture.md §6.5, WP5 exception N3): only scenarios with scale 'absolute' receive
  // DERIVED-LINACC-OS-EQUIVALENT linear acceleration instead of acceleration: null; responsive scenarios are unchanged.
  // Same arithmetic and operation order as slam_refact/tools/harness/derive_linear_accel.py; the dispatch summary is
  // recorded as the events.json entry 'derivedLinearAcceleration' for comparison with the harness file.
  const derivedLinAcc = fixture.scale === 'absolute' ? {dispatched: 0, sum: [0, 0, 0], first: null, last: null, bootstrap: null} : null;
  const linearAccelerationOf = packet => {
    const [w, x, y, z] = packet.quaternionEarthFromDevice, a = packet.deviceAcceleration;
    const r20 = 2 * (x * z - y * w), r21 = 2 * (y * z + x * w), r22 = 1 - 2 * (x * x + y * y);
    return [a[0] - 9.80665 * r20, a[1] - 9.80665 * r21, a[2] - 9.80665 * r22];
  };
  const emit = (packet, initial = false) => {
    const deviceA = {x: packet.deviceAcceleration[0], y: packet.deviceAcceleration[1], z: packet.deviceAcceleration[2]};
    const timeStamp = initial ? performance.now() : epoch + packet.sourceTimestamp * 1000;
    let acceleration = null;
    if (derivedLinAcc) {
      const v = linearAccelerationOf(packet);
      acceleration = {x: v[0], y: v[1], z: v[2]};
      if (initial) derivedLinAcc.bootstrap ??= v;
      else {
        derivedLinAcc.first ??= v; derivedLinAcc.last = v; derivedLinAcc.dispatched++;
        for (let k = 0; k < 3; k++) derivedLinAcc.sum[k] += v[k];
      }
    }
    const event = new DeviceMotionEvent('devicemotion', {acceleration, accelerationIncludingGravity: deviceA, rotationRate: packet.rotationRate, interval: packet.intervalMs});
    Object.defineProperty(event, 'timeStamp', {value: timeStamp});
    window.dispatchEvent(event);
    const orientation = new DeviceOrientationEvent('deviceorientation', packet.orientation);
    Object.defineProperty(orientation, 'timeStamp', {value: timeStamp});
    window.dispatchEvent(orientation);
    if (!initial) state.imuDispatched++;
  };
  const submit = async (gray, timestampUs) => {
    if (track.readyState === 'ended') return false;
    draw(gray);
    const frame = new VideoFrame(source, {timestamp: timestampUs});
    try { await writer.write(frame); } finally { frame.close(); }
    return true;
  };
  let finishPromise = null;
  const finish = () => {
    if (finishPromise) return finishPromise;
    finishPromise = (async () => {
      if (derivedLinAcc) state.events.push({name: 'derivedLinearAcceleration', label: 'DERIVED-LINACC-OS-EQUIVALENT', ...derivedLinAcc});
      for (const id of timers) clearTimeout(id);
      timers.clear(); clearInterval(bootstrap);
      try {
        try { XR8.stop(); } catch (error) { state.errors.push({kind: 'stop', message: String(error)}); }
        try { await writer.close(); } catch (error) { state.events.push({name: 'writerCloseAfterStop', at: performance.now(), message: String(error)}); }
      } finally {
        ownedTracks.forEach(t => t.stop());
        state.ownedResourcesClosed = ownedTracks.every(t => t.readyState === 'ended');
        state.finishedWallMs = performance.now();
        state.done = true;
      }
    })();
    return finishPromise;
  };
  window.__finishProbe = finish;
  const begin = () => {
    if (state.started) return;
    state.started = true; clearInterval(bootstrap);
    epoch = performance.now() + 200;  // small lead so the first frame is not late
    state.epochWallMs = epoch;
    window.__refBegin?.();
    const publish = async index => {
      if (index >= frameCount) { schedule(finish, 1000); return; }
      for (let k = 1; k <= prefetchAhead; k++) load(index + k);
      const gray = await cache.get(index);
      cache.delete(index);
      const timestampUs = Math.round(epoch * 1000) + Math.round(fixture.frames[index].physicalTimestamp * 1e6);
      const publication = [index, timestampUs, performance.now(), null];
      state.frames.push(publication);
      const ok = await submit(gray, timestampUs);
      publication[3] = ok ? performance.now() : -1;
      const next = index + 1;
      if (next < frameCount) schedule(() => publish(next), Math.max(0, epoch + fixture.frames[next].physicalTimestamp * 1000 - performance.now()));
      else schedule(finish, 1000);
    };
    const sensor = index => {
      let next = index;
      const due = (performance.now() - epoch) / 1000;
      while (next < fixture.sensorPackets.length && fixture.sensorPackets[next].sourceTimestamp <= due + 1e-6) { emit(fixture.sensorPackets[next]); next++; }
      if (next < fixture.sensorPackets.length) schedule(() => sensor(next), Math.max(0, epoch + fixture.sensorPackets[next].sourceTimestamp * 1000 - performance.now()));
    };
    schedule(() => { publish(0); sensor(0); }, Math.max(0, epoch - performance.now()));
  };
  // Bootstrap (as in experiment-runtime.js): frame 0 pixels + first IMU packet every 16 ms until onStart, so the
  // SDK sees a live camera.  Their videoTime precedes the run epoch and they are excluded from the trace.
  const bootstrap = setInterval(() => {
    void submit(gray0, Math.round(performance.now() * 1000)).catch(() => {});
    emit(fixture.sensorPackets[0], true);
  }, 16);
  const loaded = () => {
    state.sdkVersion = XR8.version();
    state.deviceEstimate = XR8.XrDevice.deviceEstimate();
    state.requestedConfiguration = {disableWorldTracking: false, enableWorldPoints: true, scale: fixture.scale};
    XR8.XrController.configure(state.requestedConfiguration);
    state.renderCameraConfiguration = {cam: {pixelRectWidth: width, pixelRectHeight: height, nearClipPlane: 0.01, farClipPlane: 1000}, origin: {x: 0, y: 2, z: 0}, facing: {w: 1, x: 0, y: 0, z: 0}};
    XR8.XrController.updateCameraProjectionMatrix(state.renderCameraConfiguration);
    const cycle = {gpu0: 0, gpu1: 0, cpu0: 0, cpu1: 0};
    let lastIntrinsics = null, lastIds = new Set(), freshCount = 0;
    const realityEvents = ['reality.trackingstatus', 'reality.cameraconfigured', 'reality.imageloading', 'reality.imagescanning'];
    XR8.addCameraPipelineModules([
      XR8.GlTextureRenderer.pipelineModule(),
      {name: 'ref-pre', onProcessGpu: () => { cycle.gpu0 = performance.now(); }, onProcessCpu: () => { cycle.cpu0 = performance.now(); }},
      XR8.XrController.pipelineModule(),
      {
        name: 'ref-post',
        listeners: realityEvents.map(event => ({event, process: args => state.events.push({name: event, at: performance.now(), detail: JSON.parse(JSON.stringify(args.detail ?? null))})})),
        onProcessGpu: () => { cycle.gpu1 = performance.now(); },
        onProcessCpu: () => { cycle.cpu1 = performance.now(); },
        onStart: args => {
          const gl = args.GLctx; let backend = null;
          if (gl?.getParameter) { const ext = gl.getExtension('WEBGL_debug_renderer_info'); backend = {version: gl.getParameter(gl.VERSION), unmaskedRenderer: ext ? gl.getParameter(ext.UNMASKED_RENDERER_WEBGL) : null}; }
          state.events.push({name: 'onStart', at: performance.now(), videoWidth: args.videoWidth, videoHeight: args.videoHeight, canvasWidth: args.canvasWidth, canvasHeight: args.canvasHeight, orientation: args.orientation, backend});
          begin();
        },
        onCameraStatusChange: args => state.events.push({name: 'cameraStatus', at: performance.now(), status: args.status, error: args.error ? String(args.error) : null}),
        onException: error => state.errors.push({kind: 'pipeline', message: String(error), stack: error?.stack}),
        onUpdate: ({frameStartResult: f, processCpuResult: p}) => {
          const now = performance.now();
          const r = p.reality;
          if (!r) { state.rows.push([now, f.videoTime, f.repeatFrame ? 1 : 0, null]); return; }
          const intr = Array.from(r.intrinsics || []);
          if (!lastIntrinsics || intr.some((v, i) => v !== lastIntrinsics[i])) {
            lastIntrinsics = intr;
            state.intrinsicsChanges.push({at: now, videoTime: f.videoTime, textureWidth: f.textureWidth, textureHeight: f.textureHeight, orientation: f.orientation, intrinsics: intr});
          }
          let added = 0, removed = 0, conf = null;
          const wp = r.worldPoints || [];
          if (!f.repeatFrame) {
            const ids = new Set();
            let cs = 0;
            for (const pt of wp) { ids.add(pt.id); cs += pt.confidence; if (!lastIds.has(pt.id)) added++; }
            for (const id of lastIds) if (!ids.has(id)) removed++;
            lastIds = ids;
            conf = wp.length ? cs / wp.length : null;
            freshCount++;
            if (fixture.worldPointSampleEvery > 0 && freshCount % fixture.worldPointSampleEvery === 1) {
              state.worldPointSamples.push({videoTime: f.videoTime, points: wp.map(pt => [pt.id, +pt.confidence.toFixed(4), +pt.position.x.toFixed(5), +pt.position.y.toFixed(5), +pt.position.z.toFixed(5)])});
            }
          }
          const pos = r.position || {}, rot = r.rotation || {};
          state.rows.push([now, f.videoTime, f.repeatFrame ? 1 : 0, r.trackingStatus, r.trackingReason,
            pos.x, pos.y, pos.z, rot.w, rot.x, rot.y, rot.z, wp.length, added, removed, conf,
            cycle.gpu1 - cycle.gpu0, cycle.cpu1 - cycle.cpu0, f.frameTime, f.textureWidth, f.textureHeight, f.orientation,
            window.__memProbe ? window.__memProbe.seq : null]);  // memprobe.js: last processStagedFrame seq (null without --memprobe)
        },
      },
    ]);
    state.ready = true;
    document.getElementById('start').onclick = () => {
      state.runOptions = {allowedDevices: XR8.XrConfig.device().MOBILE, cameraConfig: {direction: XR8.XrConfig.camera().BACK}};
      try { XR8.run({canvas: document.getElementById('view'), ...state.runOptions, verbose: true}); }
      catch (error) { state.errors.push({kind: 'run', message: String(error), stack: error.stack}); }
    };
  };
  if (window.XR8) loaded(); else window.addEventListener('xrloaded', loaded, {once: true});
})();
