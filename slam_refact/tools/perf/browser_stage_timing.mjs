#!/usr/bin/env node
// Timing of the deployed XR chunk (web/slam-refact-xr/engine/*) + deployed WASM engine in headless Chromium (Playwright) on the recorded flip6-s2 input of the
// US-010 sample smoke (real-time playback). DESKTOP numbers on SwiftShader (software GL) - NOT a phone's. Serial: one browser at a time.
// Reuses the sample page/fixture/server of slam_refact/web-chunk (read only) but serves the DEPLOYED chunk files, not the working tree.
//   node slam_refact/tools/perf/browser_stage_timing.mjs --label <[a-z0-9-]+> --out-root <dir under build/> [--max-frames 300] [--sample threejs]
//        [--chunk-dir web/slam-refact-xr/engine] [--engine-dir web/slam-refact-xr/engine/slam-refact-engine] [--browser playwright|<path>]
// Output: <out-root>/<label>/{meta.json,state.json} (never overwritten).
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';
import { chromium } from '../../../scripts/dev/node_modules/playwright/index.mjs';
import { startSampleServer } from '../../web-chunk/samples/serve.mjs';
import { loadFixture, UA } from '../../web-chunk/tests/sample-fixture.mjs';
import { timingSummary } from '../../web-chunk/tests/sample-timing.mjs';

const repo = path.resolve(import.meta.dirname, '../../..');
const sha = (b) => createHash('sha256').update(b).digest('hex');
const { values: a } = parseArgs({ options: { label: { type: 'string' }, 'out-root': { type: 'string' }, 'max-frames': { type: 'string', default: '300' },
  sample: { type: 'string', default: 'threejs' }, 'chunk-dir': { type: 'string', default: 'web/slam-refact-xr/engine' },
  'engine-dir': { type: 'string', default: 'web/slam-refact-xr/engine/slam-refact-engine' }, browser: { type: 'string', default: 'playwright' },
  scenario: { type: 'string', default: 'flip6-s2' } } });
if (!/^[a-z0-9-]+$/.test(a.label || '')) throw new Error('--label required');
const outRoot = path.resolve(repo, a['out-root']);
if (!outRoot.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--out-root must be under build/');
const out = path.join(outRoot, a.label);
await fs.mkdir(outRoot, { recursive: true });
await fs.mkdir(out);
const chunkDir = path.resolve(repo, a['chunk-dir']), engineDir = path.resolve(repo, a['engine-dir']);
const data = await loadFixture(a.scenario, Number(a['max-frames']));
const report = { schema: 'xr-perf-browser-timing-v1', label: a.label, argv: process.argv, startedAt: new Date().toISOString(), node: process.version,
  loadAvgAtStart: os.loadavg(), note: 'desktop headless Chromium + SwiftShader, recorded flip6-s2 frames, NOT a phone', chunkDir: a['chunk-dir'], engineDir: a['engine-dir'],
  served: {}, input: { scenario: a.scenario, frames: data.frames.length, fixtureSha256: data.fixtureSha256, nominalFps: data.nominalFps }, pageErrors: [], blockedExternal: [], console: [], requestFailures: [] };
for (const f of await fs.readdir(chunkDir)) if (f.endsWith('.js')) report.served[f] = sha(await fs.readFile(path.join(chunkDir, f))).slice(0, 16);
for (const f of ['slam_refact_engine.wasm', 'slam_refact_engine.js']) report.served['engine/' + f] = sha(await fs.readFile(path.join(engineDir, f))).slice(0, 16);
let server, browser;
try {
  const inputScript = await fs.readFile(path.join(repo, 'slam_refact/web-chunk/tests/sample-input.js'), 'utf8');
  const extra = async (p) => {
    if (p === '/input/fixture.json') return { body: data.fixtureBytes, type: 'application/json' };
    const m = /^\/input\/gray\/(\d+)$/.exec(p);
    return m ? { body: await data.gray(Number(m[1])), type: 'application/octet-stream' } : null;
  };
  server = await startSampleServer({ dirs: { chunk: chunkDir, engine: engineDir }, extra });
  const origin = server.origin;
  const launch = { headless: true, args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] };
  if (a.browser !== 'playwright') launch.executablePath = a.browser;
  browser = await chromium.launch(launch);
  report.browserVersion = browser.version();
  const context = await browser.newContext({ viewport: { width: data.width, height: data.height }, permissions: ['camera', 'microphone'], isMobile: true, hasTouch: true,
    deviceScaleFactor: 1, userAgent: UA[data.ua] });
  const page = await context.newPage();
  await page.addInitScript(inputScript);
  await page.route('**/*', (route) => {
    const u = route.request().url();
    if (!u.startsWith(origin + '/') && !u.startsWith('data:') && !u.startsWith('blob:')) { report.blockedExternal.push(u); return route.abort('blockedbyclient'); }
    return route.continue();
  });
  page.on('pageerror', (e) => report.pageErrors.push({ message: e.message }));
  page.on('console', (m) => { if (report.console.length < 100 && m.type() !== 'debug') report.console.push({ type: m.type(), text: m.text().slice(0, 300) }); });
  page.on('requestfailed', (r) => report.requestFailures.push({ url: r.url().slice(0, 200), failure: r.failure()?.errorText }));
  await page.goto(`${origin}/samples/${a.sample}/index.html?probe=1`);
  await page.waitForFunction(() => window.__xrSample?.ready === true, null, { timeout: 90000 });
  const total = Math.round(data.frames.at(-1).t * 1000) + 120000;
  report.runCompleted = await page.waitForFunction(() => window.__input.done, null, { timeout: total, polling: 500 }).then(() => true, () => false);
  const sample = await page.evaluate(() => window.__xrSample);
  const input = await page.evaluate(() => window.__input);
  const chunkDiag = await page.evaluate(() => window.__slamRefactChunk ?? null);
  await fs.writeFile(path.join(out, 'state.json'), JSON.stringify({ sample, input, chunkDiag }) + '\n');
  report.timing = timingSummary({ sample, input, chunkDiag });
  report.loadAvgAtEnd = os.loadavg();
  report.status = report.pageErrors.length ? 'COMPLETED_WITH_ERRORS' : 'COMPLETED';
} catch (error) {
  report.status = 'FAILED'; report.error = { message: error.message, stack: error.stack };
} finally {
  if (browser) await browser.close();
  if (server) await server.close();
  report.finishedAt = new Date().toISOString();
  await fs.writeFile(path.join(out, 'meta.json'), JSON.stringify(report, null, 1) + '\n');
}
console.log(JSON.stringify({ label: a.label, status: report.status, error: report.error?.message, out: path.relative(repo, out) }));
