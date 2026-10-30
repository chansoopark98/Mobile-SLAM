/* US-010 execution-model experiment, capture side (v3: atlases are kept in memory during the run and posted afterwards) (injected with page.addInitScript by tests/run-exec-model.mjs --phase capture).
 * SLAM_REFACT_CHUNK_OPTIONS.createGlue is the chunk's test hook: it returns a Proxy of the real engine glue that records every call
 * (name + arguments) and posts every atlas (the pyramid bytes of processFramePyramid) to the harness. The recorded input is replayed by
 * replay-page.js in the main thread and in a Worker. A run with this hook is not a timing run. */
(() => {
  const cap = window.__capture = { config: null, ops: [], atlases: 0, pending: 0, errors: [], engineVersion: null, buffers: [] };
  const SKIP = new Set(['delete', 'pyramidLayout', 'getFramesUntilRecenter']);
  const plain = (o, keys) => (o ? Object.fromEntries(keys.map((k) => [k, o[k]])) : o);
  const motion = (e) => ({ acceleration: plain(e.acceleration, ['x', 'y', 'z']), accelerationIncludingGravity: plain(e.accelerationIncludingGravity, ['x', 'y', 'z']),
    rotationRate: plain(e.rotationRate, ['alpha', 'beta', 'gamma']), interval: e.interval });
  const post = (url, body) => { cap.pending++; return fetch(url, { method: 'POST', body }).then((r) => { if (!r.ok) cap.errors.push(url + ' ' + r.status); }, (e) => cap.errors.push(String(e))).finally(() => { cap.pending--; }); };
  window.SLAM_REFACT_CHUNK_OPTIONS = {
    createGlue(Module, config) {
      cap.config = JSON.parse(JSON.stringify(config));
      const glue = new Module.SlamRefactGlue(config);
      return new Proxy(glue, {
        get(target, prop) {
          const value = Reflect.get(target, prop);
          if (typeof value !== 'function' || typeof prop !== 'string' || SKIP.has(prop)) return typeof value === 'function' ? value.bind(target) : value;
          return (...args) => {
            if (prop === 'processFramePyramid') {
              const k = cap.atlases++;
              cap.ops.push({ name: prop, args: [args[0], args[1], null, args[3], args[4]], atlas: k });
              cap.buffers[k] = args[2].slice();  // copy in memory: the chunk may reuse its buffer; nothing is sent during the run (v3: the v2 per-frame POST slowed the run 3.5x)
            } else if (prop === 'pushDeviceMotionEvent') {
              cap.ops.push({ name: prop, args: [args[0], motion(args[1])] });
            } else {
              cap.ops.push({ name: prop, args: JSON.parse(JSON.stringify(args)) });
            }
            return value.apply(target, args);
          };
        },
      });
    },
  };
  cap.flush = async () => {
    await post('/__capture/ops', JSON.stringify({ config: cap.config, ops: cap.ops }));
    for (let k = 0; k < cap.buffers.length; k++) { await post(`/__capture/atlas/${k}`, cap.buffers[k]); cap.buffers[k] = null; }
    while (cap.pending) await new Promise((r) => setTimeout(r, 20));
  };
})();
