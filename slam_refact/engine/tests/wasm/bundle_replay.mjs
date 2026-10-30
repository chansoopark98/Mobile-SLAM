// Replay bundle (slam_refact/tools/harness/prepare_inputs.py output) for the WASM engine, in the
// call order of tools/harness/replay/replay_main.cc: bootstrap stage first (output discarded), then
// for every input frame push each sensor row with receive_time_ns <= frame time and process the
// frame unless it is drop-masked. No node APIs: shared by the node tests and the web smoke page
// (slam_refact/web-smoke). Bundle I/O for node is in bundle_io.mjs.

export function engineConfig(config) {
  return {
    estimateScale: config.estimate_scale === '1', disableVio: config.disable_vio === '1',
    captureWidth: Number(config.capture_width), captureHeight: Number(config.capture_height),
    rotationDeg: Number(config.rotation_deg), manufacturer: config.manufacturer, model: config.model,
    pyramidModel: config.pyramid_model, origin: config.origin.split(' ').map(Number),
  };
}

function push(glue, s) {
  glue.pushDeviceMotion(s.t, s.aig[0], s.aig[1], s.aig[2], s.lin !== null, ...(s.lin ?? [0, 0, 0]),
    s.rate[0], s.rate[1], s.rate[2], s.intervalMs);
  glue.pushDeviceOrientation(s.t, s.orient[0], s.orient[1], s.orient[2]);
}

// Runs the bootstrap plus the first `maxFrames` input frames; returns one row per input frame
// ({i, fresh, pose|null}). loadImage(frame) -> {width, height, pixels}.
export function replay(Module, bundle, maxFrames, loadImage) {
  const glue = new Module.SlamRefactGlue(engineConfig(bundle.config));
  let next = 0;
  while (next < bundle.sensors.length && bundle.sensors[next].bootstrap) push(glue, bundle.sensors[next++]);
  const boot = bundle.frames[0];
  const bootImg = loadImage(boot);
  glue.processFrame(boot.timeNs, boot.videoTimeNs, bootImg.pixels, bootImg.width, bootImg.height);
  const rows = [];
  for (const f of bundle.frames.slice(1, 1 + maxFrames)) {
    while (next < bundle.sensors.length && bundle.sensors[next].t <= f.timeNs) push(glue, bundle.sensors[next++]);
    if (!f.process) { rows.push({ i: f.index, fresh: false, pose: null }); continue; }
    const img = loadImage(f);
    const t0 = performance.now();
    const pose = glue.processFrame(f.timeNs, f.videoTimeNs, img.pixels, img.width, img.height);
    rows.push({ i: f.index, fresh: true, wallMs: performance.now() - t0, pose });
  }
  glue.delete();
  return rows;
}

// Rotation angle (deg) between unit quaternions and position distance, per frame.
export function poseDiff(a, b) {
  const dot = Math.abs(a.rotationWxyz.reduce((s, v, k) => s + v * b.rotationWxyz[k], 0));
  const rot = 2 * Math.acos(Math.min(1, dot)) * 180 / Math.PI;
  const pos = Math.hypot(...a.position.map((v, k) => v - b.position[k]));
  return { rot, pos };
}
