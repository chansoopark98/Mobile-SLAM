/* Fake camera + synthetic sensors for the US-010 sample smoke (slam_refact/web-chunk/tests/run-sample-smoke.mjs injects this with
 * page.addInitScript, so the sample pages need no test code for it).
 *
 * Same mechanism and the same recorded input as slam_refact/tools/re/runtime/page-runtime.js (MediaStreamTrackGenerator frames with their
 * physical timestamps, untrusted DeviceMotion / DeviceOrientation events, a bootstrap of frame 0 + the first sensor packet until XR8 has started),
 * but it installs no pipeline modules of its own except two timing markers, because the page under test builds its own pipeline:
 *   - `sample-input-hook` is registered first (on xrloaded): its onStart starts the timed playback, its onProcessGpu / onProcessCpu record the
 *     start of the XR8 loop cycle (window.__input.cycle) that the probe module (registered last) closes.
 * The run is over when every frame was published (window.__input.done after 1 s).
 */
(() => {
  if (!/^https?:$/.test(location.protocol)) return;  // about:blank and friends: nothing to fake
  const input = window.__input = { started: false, done: false, frames: [], imuDispatched: 0, cameraRequests: [], errors: [], events: [], cycle: { gpu0: 0, cpu0: 0 } };
  let fixture = null;
  const ready = (async () => {
    fixture = await (await fetch('/input/fixture.json')).json();
    input.fixtureFrames = fixture.frames.length;
  })();
  const frameCount = () => fixture.frames.length;
  const prefetchAhead = 48;
  const cache = new Map();
  const load = (index) => {
    if (index >= frameCount() || cache.has(index)) return;
    cache.set(index, fetch('/input/gray/' + index).then((r) => r.arrayBuffer()).then((b) => new Uint8Array(b)));
  };
  let source, ctx, image, writer, stream, track, gray0, width, height;
  const ownedTracks = [];
  const timers = new Set();
  let epoch = null;
  const schedule = (fn, ms) => { const id = setTimeout(() => { timers.delete(id); fn(); }, ms); timers.add(id); };
  const draw = (gray) => {
    for (let p = 0; p < gray.length; p++) { image.data[4 * p] = gray[p]; image.data[4 * p + 1] = gray[p]; image.data[4 * p + 2] = gray[p]; image.data[4 * p + 3] = 255; }
    ctx.putImageData(image, 0, 0);
  };
  const createStream = () => {
    track = new MediaStreamTrackGenerator({ kind: 'video' });
    writer = track.writable.getWriter();
    writer.closed.catch((error) => input.events.push({ name: 'writerClosed', message: String(error) }));
    stream = new MediaStream([track]);
    ownedTracks.push(track);
    const settings = track.getSettings.bind(track);
    Object.defineProperty(track, 'getSettings', { value: () => ({ ...settings(), width, height, frameRate: fixture.nominalFps, facingMode: 'environment' }) });
  };
  const emit = (packet, initial = false) => {
    const deviceA = { x: packet.deviceAcceleration[0], y: packet.deviceAcceleration[1], z: packet.deviceAcceleration[2] };
    const timeStamp = initial ? performance.now() : epoch + packet.sourceTimestamp * 1000;
    const event = new DeviceMotionEvent('devicemotion', { acceleration: null, accelerationIncludingGravity: deviceA, rotationRate: packet.rotationRate, interval: packet.intervalMs });
    Object.defineProperty(event, 'timeStamp', { value: timeStamp });
    window.dispatchEvent(event);
    const orientation = new DeviceOrientationEvent('deviceorientation', packet.orientation);
    Object.defineProperty(orientation, 'timeStamp', { value: timeStamp });
    window.dispatchEvent(orientation);
    if (!initial) input.imuDispatched++;
  };
  const submit = async (gray, timestampUs) => {
    if (track.readyState === 'ended') return false;
    draw(gray);
    const frame = new VideoFrame(source, { timestamp: timestampUs });
    try { await writer.write(frame); } finally { frame.close(); }
    return true;
  };
  let bootstrap = null, finishPromise = null;
  const finish = () => {
    if (finishPromise) return finishPromise;
    finishPromise = (async () => {
      for (const id of timers) clearTimeout(id);
      timers.clear(); clearInterval(bootstrap);
      input.finishedAtMs = performance.now();
      input.done = true;
    })();
    return finishPromise;
  };
  window.__finishInput = async () => {
    await finish();
    try { window.XR8 && XR8.stop(); } catch (error) { input.errors.push({ kind: 'stop', message: String(error) }); }
    try { await writer.close(); } catch (error) { input.events.push({ name: 'writerCloseAfterStop', message: String(error) }); }
    ownedTracks.forEach((t) => t.stop());
    input.ownedResourcesClosed = ownedTracks.every((t) => t.readyState === 'ended');
  };
  const begin = () => {
    if (input.started) return;
    input.started = true; clearInterval(bootstrap);
    epoch = performance.now() + 200;
    input.epochWallMs = epoch;
    const publish = async (index) => {
      if (index >= frameCount()) { schedule(finish, 1000); return; }
      for (let k = 1; k <= prefetchAhead; k++) load(index + k);
      const gray = await cache.get(index);
      cache.delete(index);
      const timestampUs = Math.round(epoch * 1000) + Math.round(fixture.frames[index].physicalTimestamp * 1e6);
      const publication = [index, timestampUs, performance.now(), null];
      input.frames.push(publication);
      const ok = await submit(gray, timestampUs);
      publication[3] = ok ? performance.now() : -1;
      const next = index + 1;
      if (next < frameCount()) schedule(() => publish(next), Math.max(0, epoch + fixture.frames[next].physicalTimestamp * 1000 - performance.now()));
      else schedule(finish, 1000);
    };
    const sensor = (index) => {
      let next = index;
      const due = (performance.now() - epoch) / 1000;
      while (next < fixture.sensorPackets.length && fixture.sensorPackets[next].sourceTimestamp <= due + 1e-6) { emit(fixture.sensorPackets[next]); next++; }
      if (next < fixture.sensorPackets.length) schedule(() => sensor(next), Math.max(0, epoch + fixture.sensorPackets[next].sourceTimestamp * 1000 - performance.now()));
    };
    schedule(() => { publish(0); sensor(0); }, Math.max(0, epoch - performance.now()));
  };

  let setupPromise = null;
  const setup = () => setupPromise ??= (async () => {
    await ready;
    width = fixture.config.width; height = fixture.config.height;
    source = document.createElement('canvas');
    source.width = width; source.height = height;
    ctx = source.getContext('2d', { willReadFrequently: true });
    image = ctx.createImageData(width, height);
    for (let i = 0; i < prefetchAhead; i++) load(i);
    gray0 = await cache.get(0);
    draw(gray0);
    createStream();
    bootstrap = setInterval(() => {
      void submit(gray0, Math.round(performance.now() * 1000)).catch(() => {});
      emit(fixture.sensorPackets[0], true);
    }, 16);
  })();
  navigator.mediaDevices.getUserMedia = async (constraints) => {
    await setup();
    input.cameraRequests.push({ constraints: JSON.parse(JSON.stringify(constraints)), at: performance.now(), previousTrackState: track.readyState });
    if (track.readyState === 'ended') createStream();
    return stream;
  };

  const hookModule = () => ({
    name: 'sample-input-hook',
    onStart: () => begin(),
    onProcessGpu: () => { input.cycle.gpu0 = performance.now(); },
    onProcessCpu: () => { input.cycle.cpu0 = performance.now(); },
  });
  const install = () => XR8.addCameraPipelineModule(hookModule());
  if (window.XR8) install(); else window.addEventListener('xrloaded', install, { once: true });
})();
