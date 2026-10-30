/* Test instrumentation of the slam_refact XR8 samples (US-010). It is inert unless the page URL has `?probe=1`.
 *
 * What it records (window.__xrSample, read by slam_refact/web-chunk/tests/run-sample-smoke.mjs):
 *  - one row per pipeline `onUpdate` that carries a `reality` result: the reality pose / intrinsics and the camera the sample renders with
 *    (position, quaternion, projectionMatrix) and the errors between them, the number of world points, the cycle times of the XR8 loop;
 *  - one hitTest call per update at the screen centre (FEATURE_POINT and ESTIMATED_SURFACE) and every hitTest call the sample itself makes;
 *  - the cube: when it becomes visible, where, and its world position afterwards;
 *  - requestAnimationFrame intervals and long tasks.
 * `?mutate=camera-lag` and `?mutate=hit-shape` inject a fault (negative controls of the smoke, see
 * build/slam-refact-full-20261019/preregistration/us010-samples.v1.md section 2); without them the probe changes nothing the sample does.
 * The sample's own code (cube, tap, hitTest call) does not depend on this file. Classic script (no module), loaded before the sample. */
(function () {
  'use strict';
  // The pose comparison of criteria S7-S9: pose = the Object3D XR8 positions, projection = the camera XR8 gives its projection matrix to,
  // pos / rot / K = the reality result. rotation: q_rel = conj(q_camera) * q_reality, angle = 2 atan2(|v|, |w|).
  const poseErrors = (pose, projection, pos, rot, K) => {
    const cp = pose.position, cq = pose.quaternion, P = projection.projectionMatrix.elements;
    const posErr = Math.hypot(cp.x - pos.x, cp.y - pos.y, cp.z - pos.z);
    const rw = cq.w * rot.w + cq.x * rot.x + cq.y * rot.y + cq.z * rot.z;
    const rx = cq.w * rot.x - cq.x * rot.w - cq.y * rot.z + cq.z * rot.y;
    const ry = cq.w * rot.y + cq.x * rot.z - cq.y * rot.w - cq.z * rot.x;
    const rz = cq.w * rot.z - cq.x * rot.y + cq.y * rot.x - cq.z * rot.w;
    const rotErr = 2 * Math.atan2(Math.hypot(rx, ry, rz), Math.abs(rw));
    let projErr = 0;
    for (let i = 0; i < 16; i++) projErr = Math.max(projErr, Math.abs(P[i] - K[i]));
    return { posErr, rotErr, projErr };
  };
  const params = new URLSearchParams(location.search);
  if (!params.has('probe')) { window.XrSampleProbe = { enabled: false, createModule: () => null, poseErrors }; return; }
  const mutate = params.get('mutate') || '';
  const probeHits = params.get('hits') !== '0';  // ?hits=0: no per-update hitTest by the probe (timing runs that size the probe's own cost)
  const now = () => performance.now();
  const state = window.__xrSample = {
    version: 1, mutate, kind: null, ready: false, errors: [], realityUpdates: 0, nullUpdates: 0, updates: [],
    hits: [], appHitCalls: [], rafTimes: [], longtasks: [], firstHitUpdate: null, firstWp: null, firstReal: null, firstRealHit: null, cubeTransitions: 0, probeHits, tapDue: false, lastTapUpdate: -1000, taps: 0, screenshotDue: false,
    placement: null, post: { updates: 0, maxCubeDrift: 0, maxCamDisp: 0 }, cycle: [],
  };
  window.addEventListener('error', (e) => state.errors.push({ kind: 'window', message: e.message, stack: e.error && e.error.stack }));
  window.addEventListener('unhandledrejection', (e) => state.errors.push({ kind: 'promise', message: String(e.reason), stack: e.reason && e.reason.stack }));

  // requestAnimationFrame cadence and long tasks (desktop headless; the page runs the XR8 loop on these frames).
  const rafTick = (t) => { state.rafTimes.push(t); requestAnimationFrame(rafTick); };
  requestAnimationFrame(rafTick);
  try {
    new PerformanceObserver((list) => { for (const e of list.getEntries()) state.longtasks.push({ start: e.startTime, duration: e.duration }); })
      .observe({ entryTypes: ['longtask'] });
  } catch (error) { state.errors.push({ kind: 'longtask-observer', message: String(error) }); }

  // Every hitTest call that does not come from the probe's own per-update call is a call of the sample (tap handler).
  let inProbeCall = false;
  const plain = (hits) => JSON.parse(JSON.stringify(hits));
  const installHitWrap = () => {
    const ctl = window.XR8 && window.XR8.XrController;
    if (!ctl || ctl.hitTest.__probeWrapped) return;
    const original = ctl.hitTest;
    const wrapped = function (x, y, types) {
      const result = original.call(ctl, x, y, types);
      if (mutate === 'hit-shape') for (const h of result) delete h.distance;
      if (!inProbeCall) state.appHitCalls.push({ update: state.realityUpdates, x, y, types: types ?? null, result: plain(result) });
      return result;
    };
    Object.assign(wrapped, original);
    wrapped.__probeWrapped = true;
    ctl.hitTest = wrapped;
  };
  if (window.XR8) installHitWrap(); else window.addEventListener('xrloaded', installHitWrap, { once: true });

  const probeHit = (types) => {
    inProbeCall = true;
    try { return window.XR8.XrController.hitTest(0.5, 0.5, types); } finally { inProbeCall = false; }
  };
  const finite = (...xs) => xs.every((v) => typeof v === 'number' && Number.isFinite(v));
  const worldPosition = (obj) => { obj.updateWorldMatrix(true, false); const e = obj.matrixWorld.elements; return [e[12], e[13], e[14]]; };

  // Pipeline module appended after the sample's modules. opts: kind ('threejs' | 'aframe'), getCamera() -> {pose: Object3D that XR8 positions,
  // projection: camera whose projectionMatrix XR8 sets}, getCube() -> Object3D | null, skipUpdates (updates the framework does not apply a pose to).
  window.XrSampleProbe = {
    enabled: true,
    state,
    poseErrors,
    createModule(opts) {
      state.kind = opts.kind;
      const skip = opts.skipUpdates | 0;
      let prev = null, cubeSeen = false, cubeWasVisible = false, placedCube = null, camAtPlace = null;
      return {
        name: 'xrsampleprobe',
        onStart: () => { state.objects = { camera: !!opts.getCamera().pose, cube: !!opts.getCube() }; state.ready = true; },
        onException: (error) => state.errors.push({ kind: 'pipeline', message: String(error), stack: error && error.stack }),
        onProcessGpu: () => { state.lastGpu1 = now(); },
        onProcessCpu: () => { state.lastCpu1 = now(); },
        onUpdate: ({ frameStartResult: f, processCpuResult: p }) => {
          const r = p.reality;
          if (!r) { state.nullUpdates++; return; }
          const n = ++state.realityUpdates;
          const camObj = opts.getCamera();
          const { pose, projection } = camObj;
          const pos = r.position, rot = r.rotation, K = r.intrinsics;
          if (mutate === 'camera-lag' && prev) {
            pose.position.set(prev.pos.x, prev.pos.y, prev.pos.z);
            pose.quaternion.set(prev.rot.x, prev.rot.y, prev.rot.z, prev.rot.w);
          }
          const cp = pose.position, cq = pose.quaternion;
          const { posErr, rotErr, projErr } = poseErrors(pose, projection, pos, rot, K);
          const cyc = window.__input && window.__input.cycle;
          const row = {
            n, wall: now(), videoTime: f.videoTime, repeat: f.repeatFrame ? 1 : 0, status: r.trackingStatus, reason: r.trackingReason,
            nwp: (r.worldPoints || []).length, nreal: (r.worldPoints || []).filter((w) => !(w.id === 0 && w.confidence === 0)).length, pos: [pos.x, pos.y, pos.z], rot: [rot.w, rot.x, rot.y, rot.z],
            cam: [cp.x, cp.y, cp.z], camq: [cq.w, cq.x, cq.y, cq.z], posErr, rotErr, projErr, paired: n > skip ? 1 : 0,
            xrGpuMs: cyc && state.lastGpu1 ? state.lastGpu1 - cyc.gpu0 : null, xrCpuMs: cyc && state.lastCpu1 ? state.lastCpu1 - cyc.cpu0 : null,
          };
          if (camObj.world) row.camWorld = worldPosition(camObj.world);
          state.updates.push(row);
          if (row.nreal >= 1 && state.firstReal === null) {
            const st = window.__slamRefactChunk && window.__slamRefactChunk.stats;
            state.firstReal = { n, wall: row.wall, chunkProcessed: st ? st.processedFrames : null, chunkStaged: st ? st.stagedFrames : null };
          }
          if (row.nwp >= 1 && state.firstWp === null) {
            const st = window.__slamRefactChunk && window.__slamRefactChunk.stats;
            state.firstWp = { n, wall: row.wall, chunkProcessed: st ? st.processedFrames : null, chunkStaged: st ? st.stagedFrames : null };
          }

          // hitTest at the screen centre, every update (structure and availability of the world-effects API). v3: nreal = world points that are
          // not padding (id 0 with confidence 0); an update with nwp >= 1 and nreal = 0 is padding-only (hit distance 0 = the camera position).
          if (probeHits) {
            const fp = probeHit(['FEATURE_POINT']), es = probeHit(['ESTIMATED_SURFACE']);
            const h = fp[0];
            const hit = { n, nwp: row.nwp, nreal: row.nreal, len: fp.length, esLen: es.length, keys: h ? Object.keys(h).sort().join(',') : null,
              type: h ? h.type : null, posFinite: h ? !!h.position && finite(h.position.x, h.position.y, h.position.z) : null,
              rotFinite: h ? !!h.rotation && finite(h.rotation.x, h.rotation.y, h.rotation.z, h.rotation.w) : null,
              distance: h ? h.distance : null, distFinite: h ? finite(h.distance) && h.distance >= 0 : null,
              camDist: h && h.position && finite(h.position.x) ? Math.hypot(h.position.x - pos.x, h.position.y - pos.y, h.position.z - pos.z) : null,
              distDiff: h && h.position && finite(h.distance, h.position.x) ? Math.abs(h.distance - Math.hypot(h.position.x - pos.x, h.position.y - pos.y, h.position.z - pos.z)) : null };
            if (n <= 3 || n % 200 === 0) hit.raw = plain(fp);
            state.hits.push(hit);
            if (fp.length === 1 && row.nreal >= 1 && h.distance > 0 && state.firstRealHit === null) state.firstRealHit = n;
            if (state.firstHitUpdate === null && fp.length === 1) state.firstHitUpdate = n;
            // v3 tap rule (preregistration us010-samples.v3.md 1.5): 60 updates after the first real hit, at an update whose centre hit is a real estimate
            // (distance > 0), 15 updates after the last tap.
            if (state.firstRealHit !== null && n >= state.firstRealHit + 60 && !state.tapDue && !cubeSeen && n >= state.lastTapUpdate + 15
              && fp.length === 1 && h.distance > 0) state.tapDue = true;
          }

          // the cube: placement and world anchoring
          const cube = opts.getCube();
          if (cube) {
            const v = !!cube.visible;
            if (v && !cubeWasVisible) state.cubeTransitions++;
            cubeWasVisible = v;
          }
          if (cube && cube.visible && !cubeSeen) {
            cubeSeen = true; placedCube = cube;
            const lastApp = state.appHitCalls[state.appHitCalls.length - 1] || null;
            state.placement = { update: n, cubePosition: [cube.position.x, cube.position.y, cube.position.z], cubeWorld: worldPosition(cube),
              lastAppHit: lastApp, appHitCalls: state.appHitCalls.length };
            camAtPlace = [cp.x, cp.y, cp.z];
          }
          if (placedCube) {
            const w = worldPosition(placedCube), w0 = state.placement.cubeWorld;
            state.post.updates++;
            state.post.maxCubeDrift = Math.max(state.post.maxCubeDrift, Math.hypot(w[0] - w0[0], w[1] - w0[1], w[2] - w0[2]));
            state.post.maxCamDisp = Math.max(state.post.maxCamDisp, Math.hypot(cp.x - camAtPlace[0], cp.y - camAtPlace[1], cp.z - camAtPlace[2]));
            if (n >= state.placement.update + 3) state.screenshotDue = true;
          }
          prev = { pos: { x: pos.x, y: pos.y, z: pos.z }, rot: { x: rot.x, y: rot.y, z: rot.z, w: rot.w } };
        },
      };
    },
  };
})();
