// The engine in a module Worker (US-010 execution-model experiment). Loads the same engine bytes with the chunk's engine-loader, builds the glue
// with the recorded config and runs one recorded camera frame per message.
import { loadEngine } from '/engine/engine-loader.js';
import { runGroup } from './replay-core.js';

let glue = null;
self.onmessage = async (event) => {
  const m = event.data;
  if (m.type === 'init') {
    const t0 = performance.now();
    const { Module, loaded } = await loadEngine(new URL('/engine/slam-refact-engine/', self.location.origin), { print: () => {}, printErr: () => {} });
    glue = new Module.SlamRefactGlue(m.config);
    self.postMessage({ type: 'ready', loadMs: performance.now() - t0, matchesManifest: loaded.matchesManifest, wasmSha256: loaded.wasmSha256 });
    return;
  }
  const t0 = performance.now();
  const result = runGroup(glue, m.group, new Uint8Array(m.atlas));
  const engineMs = performance.now() - t0;
  self.postMessage({ type: 'result', i: m.i, engineMs, workerSentEpoch: performance.timeOrigin + performance.now(), result });
};
