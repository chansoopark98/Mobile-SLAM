// US-010 execution-model experiment, replay side: plays the recorded engine input (capture-hook.js) back at the recorded camera cadence and
// measures the main thread. mode=sync: the engine runs on the main thread (as the chunk does); mode=worker: the engine runs in a module Worker
// and the main thread only transfers the atlas and receives the answer; mode=idle: the same schedule and rAF probe with no engine work (the page's
// own jitter). Results: window.__replay (read by tests/run-exec-model.mjs). No XR8, no GL: engine and scheduling only.
import { loadEngine } from '/engine/engine-loader.js';
import { buildGroups, runGroup, mirrorHit } from './replay-core.js';

const mode = new URLSearchParams(location.search).get('mode');
const out = window.__replay = { mode, done: false, error: null, rafTimes: [], longtasks: [], groups: [], engineLoadMs: null, startedAt: null, finishedAt: null };
try {
  const { config, ops } = await (await fetch('/__replay/ops.json')).json();
  const groups = buildGroups(ops);
  // worker mode: hitTest calls stay on the main thread (mirror); the Worker gets the groups without them
  const hitOps = groups.map((g) => g.ops.filter((o) => o.name === 'xrHitTest'));
  const workerGroups = groups.map((g) => ({ ...g, ops: g.ops.filter((o) => o.name !== 'xrHitTest') }));
  let mirror = null;
  // non-padding world points (padding = id 0 with confidence 0): the v3 initialization criterion
  const nrealOf = (w) => { let n = 0; for (let i = 0; i + 1 < w.length; i += 5) if (!(w[i] === 0 && w[i + 1] === 0)) n++; return n; };
  out.groupCount = groups.length;
  const atlasBytes = groups.map(() => null);
  const fetchAtlas = (i) => (atlasBytes[i] ??= fetch(`/__replay/atlas/${groups[i].atlas}`).then((r) => r.arrayBuffer()));
  for (let i = 0; i < Math.min(8, groups.length); i++) fetchAtlas(i);

  let glue = null, worker = null;
  if (mode === 'sync') {
    const t0 = performance.now();
    const { Module, loaded } = await loadEngine(new URL('/engine/slam-refact-engine/', location.origin), { print: () => {}, printErr: () => {} });
    glue = new Module.SlamRefactGlue(config);
    out.engineLoadMs = performance.now() - t0; out.matchesManifest = loaded.matchesManifest; out.wasmSha256 = loaded.wasmSha256;
  } else if (mode === 'worker') {
    worker = new Worker('/__replay/engine-worker.js', { type: 'module' });
    const ready = new Promise((resolve, reject) => { worker.onmessage = (e) => resolve(e.data); worker.onerror = (e) => reject(new Error(e.message)); });
    worker.postMessage({ type: 'init', config });
    const r = await ready;
    out.engineLoadMs = r.loadMs; out.matchesManifest = r.matchesManifest; out.wasmSha256 = r.wasmSha256;
  } else if (mode !== 'idle') throw new Error('mode');

  const rafTick = (t) => { out.rafTimes.push(t); requestAnimationFrame(rafTick); };
  requestAnimationFrame(rafTick);
  try { new PerformanceObserver((l) => { for (const e of l.getEntries()) out.longtasks.push({ start: e.startTime, duration: e.duration }); }).observe({ entryTypes: ['longtask'] }); } catch { /* optional */ }

  const pending = new Map();
  if (worker) {
    worker.onmessage = (e) => {
      const h0 = performance.now();
      const m = e.data;
      const transit = performance.timeOrigin + h0 - m.workerSentEpoch;
      const rec = pending.get(m.i);
      pending.delete(m.i);
      rec.recvAt = h0; rec.rttMs = h0 - rec.sentAt; rec.workerMs = m.engineMs; rec.transitMs = transit; rec.worldValues = m.result ? m.result.world.length : null;
      if (m.result) mirror = m.result;   // mirror construction: keep the latest pose and world points for the main-thread hitTest
      rec.handlerMs = performance.now() - h0;
      if (m.result) rec.nreal = nrealOf(m.result.world);
    };
  }

  const t0 = groups[0].videoNs;
  const start = performance.now() + 300;
  out.startedAt = start;
  await new Promise((resolve) => {
    const step = async (i) => {
      if (i >= groups.length) { resolve(); return; }
      const buf = await fetchAtlas(i);
      for (let k = 1; k <= 8; k++) if (i + k < groups.length) fetchAtlas(i + k);
      const rec = { i, scheduledAt: start + (groups[i].videoNs - t0) / 1e6 };
      out.groups.push(rec);
      const wait = rec.scheduledAt - performance.now();
      if (wait > 0) await new Promise((r) => setTimeout(r, wait));
      rec.runAt = performance.now();
      if (mode === 'sync') {
        const a = performance.now();
        const timing = {};
        const res = runGroup(glue, groups[i], new Uint8Array(buf), timing);
        rec.syncMs = performance.now() - a; rec.hitMs = timing.hitMs ?? 0; rec.engineMs = rec.syncMs - rec.hitMs; rec.hitCalls = hitOps[i].length; rec.worldValues = res ? res.world.length : null; rec.nreal = res ? nrealOf(res.world) : null;
      } else if (mode === 'worker') {
        const a = performance.now();
        const fresh = new ArrayBuffer(buf.byteLength);   // the readback must target a fresh transferable buffer in this design
        const b = performance.now();
        worker.postMessage({ type: 'frame', i, group: workerGroups[i], atlas: buf }, [buf]);
        const c = performance.now();
        rec.allocMs = b - a; rec.postMs = c - b; rec.sentAt = c; void fresh;
        if (mirror) { const m0 = performance.now(); for (const op of hitOps[i]) mirrorHit(mirror, op.args[0], op.args[1]); rec.mirrorHitMs = performance.now() - m0; } else rec.mirrorHitMs = 0;
        rec.hitCalls = hitOps[i].length;
        pending.set(i, rec);
      }
      atlasBytes[i] = null;
      setTimeout(() => step(i + 1), 0);
    };
    step(0);
  });
  if (worker) await new Promise((resolve) => { const id = setInterval(() => { if (!pending.size) { clearInterval(id); resolve(); } }, 20); });
  await new Promise((r) => setTimeout(r, 300));
  out.finishedAt = performance.now();
  if (worker) worker.terminate();
} catch (error) {
  out.error = String(error && error.stack || error);
}
out.done = true;
