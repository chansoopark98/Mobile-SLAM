// US-010 execution-model ablation (REPORT, not registered in us010-samples.v2.md section 3; added afterwards to locate where the rAF slowdown of the
// real chunk comes from): the same run as run-sample-smoke.mjs --timing with the engine call removed through the chunk's own test hook
// (SLAM_REFACT_CHUNK_OPTIONS.createGlue returns the real glue except that processFramePyramid, after the first 3 real calls, returns a copy of the third
// pose with advancing frame indices; the GL pyramid draw, the PBO readback, three.js and everything else stay). No chunk file is modified.
//   node slam_refact/web-chunk/tests/run-sample-ablation.mjs --ablation noengine --sample threejs --label <label> [other run-sample-smoke options]
import { chromium } from '../../../scripts/dev/node_modules/playwright/index.mjs';

const argv = process.argv.slice(2);
const i = argv.indexOf('--ablation');
if (i < 0 || argv[i + 1] !== 'noengine') throw new Error('--ablation noengine');
argv.splice(i, 2);
const script = `window.SLAM_REFACT_CHUNK_OPTIONS = { createGlue(Module, config) {
  const glue = new Module.SlamRefactGlue(config); let calls = 0, cached = null;
  return new Proxy(glue, { get(t, p) {
    const v = Reflect.get(t, p);
    if (typeof v !== 'function') return v;
    if (p === 'processFramePyramid') return (...a) => {
      calls++;
      if (calls <= 3) { cached = v.apply(t, a); return cached; }
      const k = calls - 3;
      return { ...cached, videoFrameIndex: cached.videoFrameIndex + k, sourceFrameIndex: cached.sourceFrameIndex + k, repeated: false };
    };
    return v.bind(t);
  } });
} };`;
const launch = chromium.launch.bind(chromium);
chromium.launch = async (options) => {
  const browser = await launch(options);
  const newContext = browser.newContext.bind(browser);
  browser.newContext = async (o) => { const context = await newContext(o); await context.addInitScript(script); return context; };
  return browser;
};
const { main } = await import('./run-sample-smoke.mjs');
await main(argv);
