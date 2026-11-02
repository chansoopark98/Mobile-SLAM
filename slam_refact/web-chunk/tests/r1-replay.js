// R1 deterministic replay page (identity mode of run-engine-worker-ab.mjs). The recorded input (flip6-s2 frames + synthetic DeviceMotion packets of
// /input/fixture.json) is played through the REAL xr-slam.js chunk in a real Chromium (real WebGL2 pyramid, SwiftShader), with
//   - a virtual clock: performance.now() is set by this script before every IMU packet and frame, so the receive times, stage times and frame
//     times the chunk hands to the engine are identical in every run (the live path stamps them with the wall clock, which is why live runs are
//     not bit-identical even main vs main);
//   - a fake XR8 (the callbacks are called directly, in the order XR8 calls them: IMU events, onProcessGpu, onProcessCpu);
//   - in Worker mode, a wait after each frame until the Worker has answered it, so the answer order is the frame order (the answers are keyed
//     by the engine's source frame index, so the one-callback delay of the Worker mode does not matter for the comparison).
// Output: window.__r1replay = {done, error, answers: [{src, pos, rot, K, status, reason, wp}], errors, engineWorker, ...}.
const params = new URLSearchParams(location.search);
const out = window.__r1replay = { done: false, error: null, answers: [], errors: [], frames: 0, engineWorker: null, sessions: null };
const EPOCH_MS = 1000;
const virtual = { now: EPOCH_MS };
Object.defineProperty(performance, 'now', { configurable: true, value: () => virtual.now });
const sleep0 = () => new Promise((resolve) => setTimeout(resolve, 0));
const until = async (condition, what, limit = 20000) => {
  for (let i = 0; i < limit; i++) { if (condition()) return; await sleep0(); }
  throw new Error('timeout waiting for ' + what);
};
try {
  const fixture = await (await fetch('/input/fixture.json')).json();
  const frames = Math.min(fixture.frames.length, Number(params.get('frames') ?? fixture.frames.length));
  const W = fixture.config.width, H = fixture.config.height;
  window._c8 = { checkAllZeroes: 0 };
  window.XR8 = { isPaused: () => false,
    XrDevice: { deviceEstimate: () => ({ manufacturer: 'Google', model: 'Pixel 7' }), isDeviceBrowserCompatible: () => true },
    XrConfig: { camera: () => ({ FRONT: 'front', BACK: 'back' }), device: () => ({ MOBILE: 'mobile', ANY: 'any' }) },
    XrPermissions: { permissions: () => ({ CAMERA: 'camera', DEVICE_MOTION: 'devicemotion', DEVICE_ORIENTATION: 'deviceorientation' }) },
    drawTexForComputeTex: (t) => t };
  const mod = await import('/engine/xr-slam.js');
  const ctrl = await mod.XrControllerFactory({ paused: false }, Promise.resolve({}), () => () => {}, (e) => out.errors.push(String(e && e.message || e)));
  const diag = window.__slamRefactChunk;
  // the GL context is created after the engine is loaded (a context made before the 256 MB engine memory was lost in headless Chromium)
  const canvas = document.createElement('canvas');
  canvas.width = 16; canvas.height = 16;
  const gl = canvas.getContext('webgl2', { antialias: false });
  if (!gl) throw new Error('no WebGL2');
  const pm = ctrl.pipelineModule();
  ctrl.configure({ enableWorldPoints: true });
  ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: W, pixelRectHeight: H, nearClipPlane: 0.01, farClipPlane: 1000 } });
  pm.onBeforeSessionInitialize({ sessionAttributes: { fillsCameraTexture: true, controlsCamera: false } });
  pm.onStart({ computeCtx: gl, canvasWidth: W, canvasHeight: H });
  const framework = { dispatchEvent: () => {} };
  pm.onSessionAttach({ videoWidth: W, videoHeight: H, canvasWidth: W, canvasHeight: H, rotation: 0, computeCtx: gl, framework, GLctx: gl,
    sessionAttributes: { fillsCameraTexture: true } });
  await until(() => diag.stats.sessionsCreated >= 1, 'the session');

  const texture = gl.createTexture();
  gl.bindTexture(gl.TEXTURE_2D, texture);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.LINEAR);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  const rgba = new Uint8Array(W * H * 4);
  const upload = (gray) => {
    for (let p = 0; p < gray.length; p++) { const g = gray[p]; rgba[4 * p] = g; rgba[4 * p + 1] = g; rgba[4 * p + 2] = g; rgba[4 * p + 3] = 255; }
    gl.bindTexture(gl.TEXTURE_2D, texture);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, W, H, 0, gl.RGBA, gl.UNSIGNED_BYTE, rgba);
  };
  const emit = (packet) => {
    const event = new DeviceMotionEvent('devicemotion', { acceleration: null,
      accelerationIncludingGravity: { x: packet.deviceAcceleration[0], y: packet.deviceAcceleration[1], z: packet.deviceAcceleration[2] },
      rotationRate: packet.rotationRate, interval: packet.intervalMs });
    Object.defineProperty(event, 'timeStamp', { value: virtual.now });
    window.dispatchEvent(event);
    const orientation = new DeviceOrientationEvent('deviceorientation', packet.orientation);
    Object.defineProperty(orientation, 'timeStamp', { value: virtual.now });
    window.dispatchEvent(orientation);
  };
  const record = (r) => {
    const st = diag.sourceTiming;
    if (!r || !st || !st.fresh || !st.hasPose) return;
    out.answers.push({ src: st.sourceFrameIndex, pos: [r.position.x, r.position.y, r.position.z], rot: [r.rotation.w, r.rotation.x, r.rotation.y, r.rotation.z],
      K: r.intrinsics, status: r.trackingStatus, reason: r.trackingReason,
      wp: r.worldPoints.map((w) => [w.id, w.confidence, w.position.x, w.position.y, w.position.z]) });
  };
  const ew = () => diag.engineWorker && diag.engineWorker.active ? diag.engineWorker.stats : null;
  const settle = async () => { const s = ew(); if (s) await until(() => s.resultsReceived >= s.framesSubmitted, 'the Worker answer'); };
  const fetchGray = (index) => fetch('/input/gray/' + index).then((r) => r.arrayBuffer()).then((b) => new Uint8Array(b));

  let next = fetchGray(0), packetIndex = 0;
  for (let k = 0; k < frames; k++) {
    const gray = await next;
    if (k + 1 < frames) next = fetchGray(k + 1);
    const t = fixture.frames[k].physicalTimestamp;
    while (packetIndex < fixture.sensorPackets.length && fixture.sensorPackets[packetIndex].sourceTimestamp <= t) {
      virtual.now = EPOCH_MS + fixture.sensorPackets[packetIndex].sourceTimestamp * 1000;
      emit(fixture.sensorPackets[packetIndex++]);
    }
    virtual.now = EPOCH_MS + t * 1000;
    upload(gray);
    const f = { computeTexture: texture, cameraTexture: texture, textureWidth: W, textureHeight: H, orientation: 0, repeatFrame: false,
      videoTime: 20 + t, frameTime: virtual.now / 1000 };
    pm.onProcessGpu({ framework, frameStartResult: f });
    record(pm.onProcessCpu({ framework, frameStartResult: f }));
    out.frames++;
    await settle();
  }
  for (let i = 0; i < 6; i++) {  // repeated callbacks take the results still on their way (Worker mode)
    virtual.now += 33;
    record(pm.onProcessCpu({ framework, frameStartResult: { computeTexture: texture, cameraTexture: texture, textureWidth: W, textureHeight: H, orientation: 0,
      repeatFrame: true, videoTime: 20 + fixture.frames[frames - 1].physicalTimestamp, frameTime: virtual.now / 1000 } }));
    await settle();
  }
  out.engineWorker = diag.engineWorker ? { active: diag.engineWorker.active, fallback: diag.engineWorker.fallback,
    framesSubmitted: diag.engineWorker.stats?.framesSubmitted ?? null, resultsReceived: diag.engineWorker.stats?.resultsReceived ?? null,
    backpressureSkips: diag.engineWorker.stats?.backpressureSkips ?? null } : null;
  out.engine = diag.engine ? { wasmSha256: diag.engine.wasmSha256, matchesManifest: diag.engine.matchesManifest } : null;
  out.chunkStats = { processedFrames: diag.stats.processedFrames, freshResponses: diag.stats.freshResponses, droppedStaged: diag.stats.droppedStaged, processErrors: diag.stats.processErrors };
  out.warnings = diag.warnings;
} catch (error) {
  out.error = String(error && error.stack || error);
}
out.done = true;
