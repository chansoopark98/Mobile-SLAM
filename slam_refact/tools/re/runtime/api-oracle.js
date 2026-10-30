/* XrController API oracle hook (full-parity PRD US-002/003/005, slam_refact/docs/spec/06-xr-controller-api.md).
 * Installed with page.addInitScript before page-runtime.js. On xrloaded (before page-runtime's own listener) it
 *   - merges `configure` options into page-runtime's XrController.configure call,
 *   - re-sends updateCameraProjectionMatrix with the extra `cam` / `origin` / `facing` after page-runtime's call,
 *   - appends a pipeline module whose onUpdate records, for fresh (non-repeat) frames, reality.lighting and, every
 *     `hitEvery` fresh frames after `hitWarmup`, XrController.hitTest for a fixed screen grid and type sets together
 *     with the same callback's reality.{position, rotation, intrinsics, worldPoints} (a self-consistent pair).
 * FULL-7 (RM-3b, review M2): option `ab` adds a second recording kind. On every `ab.every`-th fresh callback after
 * `ab.warmup` it runs, INSIDE that one onUpdate, a sequence of coordinate-system steps (updateCameraProjectionMatrix
 * with origin / facing incl. pitch and roll, configure of mirroredDisplay / leftHandedAxes, scale changes through
 * origin.y), calls XrController.hitTest for a screen point grid after each step, and finally restores the base
 * configuration. All steps see the same internal tracker state (camera, feature set), so hit_B and hit_A differ only
 * by the coordinate system: this is what makes the recording discriminating (a similarity transform of the whole state
 * leaves the FEATURE_POINT computation unchanged, so a single-configuration recording cannot tell a wrong
 * origin / facing / scale model from the right one). Records window.__apiOracle.ab.
 * Read-only with respect to the SDK: no SDK bytes, exports or memory are changed. Results: window.__apiOracle. */
window.__apiOracleInit = options => {
  const o = {configure: {}, cam: null, origin: null, facing: null, hitEvery: 20, hitWarmup: 15,
    points: [[0.5, 0.5], [0.25, 0.3], [0.75, 0.7], [0.1, 0.9], [0.9, 0.1]],
    typeSets: [[], ['FEATURE_POINT'], ['ESTIMATED_SURFACE'], ['DETECTED_SURFACE'], ['ESTIMATED_SURFACE', 'DETECTED_SURFACE']],
    ...options};
  const state = window.__apiOracle = {options: o, installed: false, error: null, configureCalls: [], projectionCalls: [],
    lighting: [], hits: [], ab: [], fresh: 0};
  const num = v => (Object.is(v, -0) ? '-0' : v);  // keep the sign of zero through JSON
  const vec = p => (p ? {x: num(p.x), y: num(p.y), z: num(p.z)} : null);
  const quat = q => (q ? {w: num(q.w), x: num(q.x), y: num(q.y), z: num(q.z)} : null);
  // ---- FULL-7 A/B sequence definition (the exact arguments are recorded with every sample) ----
  const quatMul = (a, b) => ({w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z, x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
    y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x, z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w});
  const axisQuat = (deg, axis) => { const h = deg * Math.PI / 360; const q = {w: Math.cos(h), x: 0, y: 0, z: 0}; q[axis] = Math.sin(h); return q; };
  const unit = q => { const n = Math.hypot(q.w, q.x, q.y, q.z); return {w: q.w / n, x: q.x / n, y: q.y / n, z: q.z / n}; };
  const abBase = {origin: {x: 0, y: 2, z: 0}, facing: {w: 1, x: 0, y: 0, z: 0}};
  const upd = (origin, facing) => ({fn: 'update', args: {origin, facing}});
  const cfg = args => ({fn: 'configure', args});
  const pitchYawRoll = unit(quatMul(quatMul(axisQuat(40, 'y'), axisQuat(25, 'x')), axisQuat(20, 'z')));
  const defaultSteps = [
    {id: 'base0', calls: []},
    {id: 'null-identity', calls: [upd(abBase.origin, abBase.facing)]},
    {id: 'origin-xz', calls: [upd({x: 0.7, y: 2, z: -1.3}, abBase.facing)]},
    {id: 'scale-3.5', calls: [upd({x: 0.7, y: 3.5, z: -1.3}, abBase.facing)]},
    {id: 'yaw40', calls: [upd(abBase.origin, axisQuat(40, 'y'))]},
    {id: 'pitch25', calls: [upd(abBase.origin, axisQuat(25, 'x'))]},
    {id: 'roll20', calls: [upd(abBase.origin, axisQuat(20, 'z'))]},
    {id: 'yaw40-pitch25-roll20', calls: [upd(abBase.origin, pitchYawRoll)]},
    {id: 'mirror', calls: [upd(abBase.origin, abBase.facing), cfg({mirroredDisplay: true})]},
    {id: 'lefthanded', calls: [cfg({mirroredDisplay: false, leftHandedAxes: true})]},
    {id: 'lh-mirror-origin-yaw', calls: [cfg({mirroredDisplay: true, leftHandedAxes: true}),
      upd({x: -0.4, y: 1.2, z: 0.9}, unit(quatMul(axisQuat(-65, 'y'), axisQuat(15, 'x'))))]},
    {id: 'restore', calls: [cfg({mirroredDisplay: false, leftHandedAxes: false}), upd(abBase.origin, abBase.facing)]},
  ];
  // Every point has its mirror partner (1 - x, y) in the list (the centre is its own partner): the mirror steps query
  // x' = 1 - x of the internal ray, so the base-configuration hit at the partner is the internal hit of a mirror query.
  const abPoints = [[0.5, 0.5], [0.3, 0.4], [0.7, 0.4], [0.2, 0.75], [0.8, 0.75], [0.8, 0.25], [0.2, 0.25], [0.1, 0.9], [0.9, 0.9]];
  window.addEventListener('xrloaded', () => {
    try {
      const xc = XR8.XrController;
      const configure = xc.configure.bind(xc);
      const update = xc.updateCameraProjectionMatrix.bind(xc);
      xc.configure = args => {
        const merged = {...(args || {}), ...o.configure};
        state.configureCalls.push(merged);
        return configure(merged);
      };
      xc.updateCameraProjectionMatrix = args => {
        update(args);
        state.projectionCalls.push(args);
        if (o.cam || o.origin || o.facing) {
          const extra = {};
          if (o.cam) extra.cam = o.cam;
          if (o.origin) extra.origin = o.origin;
          if (o.facing) extra.facing = o.facing;
          update(extra);
          state.projectionCalls.push(extra);
        }
      };
      const add = XR8.addCameraPipelineModules.bind(XR8);
      XR8.addCameraPipelineModules = modules => add([...modules, {
        name: 'api-oracle',
        onUpdate: ({frameStartResult: f, processCpuResult: p}) => {
          const r = p && p.reality;
          if (!r || f.repeatFrame) return;
          state.fresh++;
          state.lighting.push({videoTime: f.videoTime, fresh: state.fresh,
            lighting: r.lighting ? {exposure: num(r.lighting.exposure), temperature: num(r.lighting.temperature)} : null,
            status: r.trackingStatus, reason: r.trackingReason});
          if (o.ab && state.fresh >= (o.ab.warmup ?? 15) && (state.fresh - (o.ab.warmup ?? 15)) % (o.ab.every ?? 20) === 0) {
            const wpAb = (r.worldPoints || []).map(pt => [pt.id, pt.confidence, num(pt.position.x), num(pt.position.y), num(pt.position.z)]);
            const rec = {videoTime: f.videoTime, fresh: state.fresh, status: r.trackingStatus, reason: r.trackingReason,
              position: vec(r.position), rotation: quat(r.rotation), intrinsics: Array.from(r.intrinsics || []), worldPoints: wpAb, steps: []};
            for (const step of (o.ab.steps || defaultSteps)) {
              for (const c of step.calls) { if (c.fn === 'update') update(c.args); else configure(c.args); }
              const queries = [];
              for (const [x, y] of (o.ab.points || abPoints)) {
                const hits = XR8.XrController.hitTest(x, y, []).map(h => ({type: h.type, position: vec(h.position), rotation: quat(h.rotation), distance: num(h.distance)}));
                queries.push({x, y, hits});
              }
              rec.steps.push({id: step.id, calls: step.calls, queries});
            }
            state.ab.push(rec);
          }
          if (state.fresh < o.hitWarmup || (state.fresh - o.hitWarmup) % o.hitEvery !== 0) return;
          const wp = (r.worldPoints || []).map(pt => [pt.id, pt.confidence, num(pt.position.x), num(pt.position.y), num(pt.position.z)]);
          const queries = [];
          for (const [x, y] of o.points) {
            for (const types of o.typeSets) {
              const hits = XR8.XrController.hitTest(x, y, types).map(h => ({type: h.type, position: vec(h.position),
                rotation: quat(h.rotation), distance: num(h.distance)}));
              queries.push({x, y, types, hits});
            }
          }
          state.hits.push({videoTime: f.videoTime, fresh: state.fresh, status: r.trackingStatus, reason: r.trackingReason,
            position: vec(r.position), rotation: quat(r.rotation), intrinsics: Array.from(r.intrinsics || []), worldPoints: wp,
            queries});
        },
      }]);
      state.installed = true;
    } catch (error) {
      state.error = String(error && error.stack || error);
    }
  }, {once: true});
};
