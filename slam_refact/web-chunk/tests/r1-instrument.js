/* R1 harness page instrumentation (injected with page.addInitScript by run-engine-worker-ab.mjs, perf mode; the sample runs WITHOUT ?probe=1).
 *
 * Passive: no polling, no requestAnimationFrame loop, no periodic timer. Per camera frame it does small pushes only:
 *   - `r1-capture` (registered at xrloaded, before the sample's modules): onProcessGpu entry of a camera frame = "capture callback entry"
 *     (one record per non-repeat frame callback).
 *   - `r1-apply` (registered right before XR8.run, after every sample module): onUpdate entry (the pose is applied: every earlier module's
 *     onUpdate has run), its onRender exit, and the render-callback count.
 * Frame identity: the callback's videoTime (seconds) -> the fixture frame whose timestamp (microseconds) it is (window.__input.frames).
 * Source frame of a pose: window.__slamRefactChunk.resultSeqs (the chunk's list of the frame ordinals whose results it handled; the <seq> of the
 * `sr-frame-done:<seq>` marks, SLAM_REFACT_CHUNK_OPTIONS.frameDoneMarks, set by the harness in both arms), drained by each update.
 * The window is defined by those marks (node side); the only timer here is ONE one-shot, 3500 ms after the input is over and long after the window,
 * that tells the harness to stop the trace.
 */
(() => {
  if (!/^https?:$/.test(location.protocol)) return;
  const r1 = window.__r1 = { ready: false, captures: [], renderCallbacks: [], applied: [], renders: [], rows: [], longtasks: [], finished: false, errors: [], timerSet: false };
  const tsIndex = new Map();
  let seen = 0;
  const indexOf = (seconds) => {
    const frames = window.__input && window.__input.frames;
    if (!frames || !Number.isFinite(seconds)) return -1;
    for (; seen < frames.length; seen++) tsIndex.set(frames[seen][1], frames[seen][0]);
    const index = tsIndex.get(Math.round(seconds * 1e6));
    return index === undefined ? -1 : index;
  };
  try {
    new PerformanceObserver((list) => { for (const e of list.getEntries()) r1.longtasks.push({ start: e.startTime, duration: e.duration }); })
      .observe({ type: 'longtask', buffered: true });
  } catch (error) { r1.errors.push('longtask observer: ' + error); }

  const early = () => ({
    name: 'r1-capture',
    onProcessGpu: ({ frameStartResult: f }) => {
      if (f.repeatFrame) return;
      // every non-repeat frame callback, in order (index -1: a frame the page fed that is not a recorded frame)
      const index = indexOf(f.videoTime);
      r1.captures.push([index, performance.now()]);
      // the chunk numbers this frame (its sr-frame-start / sr-frame-done seq) before it does any work: recorded frame k -> k + 1 (1, 2, 3, ...),
      // frames the page fed outside the recording -> 1e6 + n, so the recorded frames' seq are unique
      const diag = window.__slamRefactChunk;
      if (diag && typeof diag.setFrameOrdinal === 'function') diag.setFrameOrdinal(index >= 0 ? index + 1 : 1000000 + (r1.bootFrames = (r1.bootFrames || 0) + 1));
    },
  });

  const late = () => ({
    name: 'r1-apply',
    onStart: () => { r1.ready = true; },
    onUpdate: ({ frameStartResult: f, processCpuResult: p }) => {
      const t = performance.now();
      const repeat = !!f.repeatFrame;
      const callbackIndex = repeat ? -1 : indexOf(f.videoTime);
      if (callbackIndex >= 0) r1.renderCallbacks.push([callbackIndex, t]);
      const reality = p && p.reality;
      const diag = window.__slamRefactChunk;
      // every result the chunk handled since the last update (fresh or repeated pose; several when the Worker had a backlog) is consumed by this update
      if (diag && diag.resultSeqs) for (const seq of diag.resultSeqs.splice(0)) if (seq < 1000000) r1.applied.push([seq - 1, t, callbackIndex, repeat ? 1 : 0, seq]);
      if (reality && !repeat) {
        const wp = reality.worldPoints || [];
        r1.rows.push({ n: r1.rows.length + 1, wall: t, videoTime: f.videoTime, repeat: 0, status: reality.trackingStatus, reason: reality.trackingReason,
          nwp: wp.length, nreal: wp.filter((w) => !(w.id === 0 && w.confidence === 0)).length,
          pos: [reality.position.x, reality.position.y, reality.position.z], rot: [reality.rotation.w, reality.rotation.x, reality.rotation.y, reality.rotation.z] });
      }
      if (!r1.timerSet && window.__input && window.__input.done) {
        r1.timerSet = true;
        setTimeout(() => { r1.finished = true; if (typeof window.__r1Signal === 'function') window.__r1Signal(); }, 3500);  // the trace keeps 3000 ms after the window end (report-only task sum)
      }
    },
    // the last module's render callback returns after every other module's render callbacks; its exit is the earliest legal time of a done mark
    onRender: () => { r1.renders.push([performance.now()]); },
  });

  const install = () => {
    XR8.addCameraPipelineModule(early());
    const run = XR8.run.bind(XR8);
    XR8.run = (...args) => { XR8.addCameraPipelineModule(late()); return run(...args); };
  };
  if (window.XR8) install(); else window.addEventListener('xrloaded', install, { once: true });
})();
