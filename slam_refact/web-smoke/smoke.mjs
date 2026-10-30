// Page side of the web smoke (US-012): runs a short TUM clip through the WASM GlueRunner in the
// browser and publishes the result on window.__smokeResult for run-smoke.mjs.
import { replay } from '/shared/bundle_replay.mjs';

const log = (s) => { document.getElementById('log').textContent = s; };

async function main() {
  const clip = await (await fetch('/clip/clip.json')).json();
  const images = await Promise.all(clip.bundle.frames.map(async (f) => {
    const buf = new Uint8Array(await (await fetch(`/clip/img/${f.imageIndex}`)).arrayBuffer());
    return { width: clip.width, height: clip.height, pixels: buf };
  }));
  const Module = await globalThis.SlamRefactEngine();
  const t0 = performance.now();
  const rows = replay(Module, clip.bundle, clip.maxFrames, (f) => images[f.imageIndex]);
  const totalMs = performance.now() - t0;
  window.__smokeResult = { ok: true, userAgent: navigator.userAgent, totalMs, rows };
  log(`done ${rows.length} frames`);
}

main().catch((e) => {
  window.__smokeResult = { ok: false, error: String(e && e.stack || e) };
  log(String(e));
});
