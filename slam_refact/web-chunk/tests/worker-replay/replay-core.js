// Shared by the main-thread replay (replay-page.js) and the Worker replay (engine-worker.js): the engine-side work of one camera frame, in the
// order the recorded chunk run called the glue. The same function runs in both places, so the two modes do identical engine work.
export function buildGroups(ops) {
  const groups = [];
  let cur = [];
  for (const op of ops) {
    cur.push(op);
    if (op.name === 'processFramePyramid') { groups.push({ ops: cur, atlas: op.atlas, timeNanos: op.args[0], videoNs: op.args[1] }); cur = []; }
  }
  return groups;
}

// atlas: Uint8Array of the frame's pyramid. Returns what a response needs: pose arrays and the flat world points.
// timing (optional object): `hitMs` accumulates the time of the xrHitTest calls (the hitTest of the live chunk runs on the same thread as the engine).
export function runGroup(glue, group, atlas, timing = null) {
  let result = null;
  for (const op of group.ops) {
    if (op.name === 'xrHitTest' && timing) {
      const t = performance.now();
      glue.xrHitTest(...op.args);
      timing.hitMs = (timing.hitMs ?? 0) + (performance.now() - t);
    } else if (op.name === 'processFramePyramid') {
      const [timeNanos, videoNs, , size, rows] = op.args;
      const pose = glue.processFramePyramid(timeNanos, videoNs, atlas, size, rows);
      result = { position: Array.from(pose.position), rotationWxyz: Array.from(pose.rotationWxyz), intrinsics: Array.from(pose.intrinsics), repeated: pose.repeated,
        videoFrameIndex: pose.videoFrameIndex };
    } else {
      glue[op.name](...op.args);
    }
  }
  if (result) result.world = Array.from(glue.getWorldPoints());
  return result;
}

// Cost model of hitTest(FEATURE_POINT) on the main thread when the engine lives in a Worker (spec 06 section 2.1 operation structure, not bit-identical):
// every world point goes to camera coordinates, its tangent-plane distance to the ray is weighted with a Gaussian, and the weighted mean depth is returned.
// m = {position: [x, y, z], rotationWxyz: [w, x, y, z], intrinsics: [16], world: flat [id, confidence, x, y, z] * N}.
export function mirrorHit(m, x, y) {
  const [w, qx, qy, qz] = m.rotationWxyz, p = m.position, K = m.intrinsics, a = m.world;
  // rows of R^T = columns of R
  const r00 = 1 - 2 * (qy * qy + qz * qz), r01 = 2 * (qx * qy - qz * w), r02 = 2 * (qx * qz + qy * w);
  const r10 = 2 * (qx * qy + qz * w), r11 = 1 - 2 * (qx * qx + qz * qz), r12 = 2 * (qy * qz - qx * w);
  const r20 = 2 * (qx * qz - qy * w), r21 = 2 * (qy * qz + qx * w), r22 = 1 - 2 * (qx * qx + qy * qy);
  const tx = ((x - 0.5) * 2 - K[8]) / (K[0] || 1), ty = ((0.5 - y) * 2 - K[9]) / (K[5] || 1);
  let sw = 0, sz = 0;
  for (let i = 0; i + 4 < a.length; i += 5) {
    const dx = a[i + 2] - p[0], dy = a[i + 3] - p[1], dz = a[i + 4] - p[2];
    const vx = r00 * dx + r10 * dy + r20 * dz, vy = r01 * dx + r11 * dy + r21 * dz, vz = r02 * dx + r12 * dy + r22 * dz;
    const depth = -vz;
    if (!depth) continue;
    const du = vx / depth - tx, dv = vy / depth - ty;
    const wgt = Math.exp(-0.5 * (du * du + dv * dv) / 0.0004);
    sw += wgt; sz += wgt * depth;
  }
  return sw > 0 ? sz / sw : 0;
}
