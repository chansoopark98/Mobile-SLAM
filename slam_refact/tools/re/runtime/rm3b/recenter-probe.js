/* G6 / RM-3b criterion 5 (full-parity plan, g-remaining-plan-2026-10-25.md section 2.3, G6): original-SDK recenter probe.
 * Installed with page.addInitScript before page-runtime.js (the same mechanism as api-oracle.js). On xrloaded it appends a
 * pipeline module whose onUpdate (a) records every callback's pose (callback sequence number, videoTime, input frame index,
 * repeatFrame, trackingStatus/Reason, position, rotation) and (b) executes the scripted XrController calls
 * (recenter / configure / updateCameraProjectionMatrix) INSIDE the onUpdate of the first non-repeat callback whose input frame
 * index is >= the action's atFrame (so the call happens between two processStagedFrame calls, like any app-side call).
 * Read-only with respect to the SDK: no SDK bytes, exports or memory are changed; it only calls public XrController methods.
 * Options: {actions: [{id, atFrame, calls: [{fn: 'recenter'} | {fn: 'configure', args} | {fn: 'update', args}]}]}.
 * Results: window.__recenterProbe = {options, installed, error, callbacks, actions}. */
window.__recenterProbeInit = options => {
  const o = {actions: [], ...options};
  const state = window.__recenterProbe = {options: o, installed: false, error: null, callbacks: [], actions: []};
  const num = v => (Object.is(v, -0) ? '-0' : v);  // keep the sign of zero through JSON
  const pending = o.actions.slice().sort((a, b) => a.atFrame - b.atFrame);
  let scanned = 0;
  const byUs = new Map();
  const frameOf = videoTime => {  // page-runtime pushes [index, videoTimeUs, ...] before it submits the frame
    const frames = (window.__probe && window.__probe.frames) || [];
    for (; scanned < frames.length; scanned++) byUs.set(frames[scanned][1], frames[scanned][0]);
    const us = Math.round(videoTime * 1e6);
    return byUs.get(us) ?? byUs.get(us - 1) ?? byUs.get(us + 1) ?? null;
  };
  window.addEventListener('xrloaded', () => {
    try {
      const add = XR8.addCameraPipelineModules.bind(XR8);
      XR8.addCameraPipelineModules = modules => add([...modules, {
        name: 'rm3b-recenter-probe',
        onUpdate: ({frameStartResult: f, processCpuResult: p}) => {
          const r = p && p.reality;
          if (!r) return;
          const frameIndex = frameOf(f.videoTime);
          const seq = state.callbacks.length;
          state.callbacks.push({seq, videoTime: f.videoTime, frameIndex, repeat: f.repeatFrame ? 1 : 0, status: r.trackingStatus, reason: r.trackingReason,
            p: r.position ? [num(r.position.x), num(r.position.y), num(r.position.z)] : null,
            q_wxyz: r.rotation ? [num(r.rotation.w), num(r.rotation.x), num(r.rotation.y), num(r.rotation.z)] : null});
          if (f.repeatFrame || frameIndex == null || !pending.length || frameIndex < pending[0].atFrame) return;
          const action = pending.shift();
          const done = [];
          for (const c of action.calls) {
            const rec = {fn: c.fn, args: c.args ?? null, ok: true, error: null};
            try {
              if (c.fn === 'recenter') XR8.XrController.recenter();
              else if (c.fn === 'configure') XR8.XrController.configure(c.args);
              else if (c.fn === 'update') XR8.XrController.updateCameraProjectionMatrix(c.args);
              else throw new Error('unknown fn ' + c.fn);
            } catch (error) { rec.ok = false; rec.error = String(error && error.message || error); }
            done.push(rec);
          }
          state.actions.push({id: action.id, atFrameRequested: action.atFrame, atFrameActual: frameIndex, afterCallbackSeq: seq, calls: done});
        },
      }]);
      state.installed = true;
    } catch (error) {
      state.error = String(error && error.stack || error);
    }
  }, {once: true});
};
