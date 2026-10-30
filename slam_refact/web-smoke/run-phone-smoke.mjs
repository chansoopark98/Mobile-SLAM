#!/usr/bin/env node
// Headless smoke of the user phone page web-chunk/samples/phone/ (desktop Chromium with mobile emulation, NOT a phone).
// The camera and the motion sensors are the RECORDED flip6-s2 frames / synthetic DeviceMotion of web-chunk/tests/sample-input.js: a synthetic
// input, never a device result. One-off 127.0.0.1 server (web-chunk/samples/serve.mjs), external requests blocked and counted, nothing under web/.
//
//   node slam_refact/web-smoke/run-phone-smoke.mjs --label <[a-z0-9-]+> [--max-frames 300] [--engine-dir <dir under build/>]
// Output (never overwritten): build/slam-refact-phone-20261022/<label>/{result.json,metrics-download.json,*.png}
import fs from 'node:fs/promises';
import path from 'node:path';
import zlib from 'node:zlib';
import { parseArgs } from 'node:util';
import { chromium } from '../../scripts/dev/node_modules/playwright/index.mjs';
import { startSampleServer, DEFAULT_ENGINE_DIR } from '../web-chunk/samples/serve.mjs';
import { loadFixture, UA } from '../web-chunk/tests/sample-fixture.mjs';

const repo = path.resolve(import.meta.dirname, '../..');
const { values } = parseArgs({ options: { label: { type: 'string' }, 'max-frames': { type: 'string', default: '300' }, 'engine-dir': { type: 'string', default: DEFAULT_ENGINE_DIR } } });
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (a label is never reused)');
const engineDir = path.resolve(repo, values['engine-dir']);
if (!engineDir.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--engine-dir must be under build/');
const out = path.join(repo, 'build/slam-refact-phone-20261022', values.label);
await fs.mkdir(path.dirname(out), { recursive: true });
await fs.mkdir(out);

// A tiny valid PNG (gray noise) for the file-picker check; built here so the smoke needs no image dependency.
const crcTable = Array.from({ length: 256 }, (_, n) => { let c = n; for (let k = 0; k < 8; k++) c = c & 1 ? 0xedb88320 ^ (c >>> 1) : c >>> 1; return c >>> 0; });
const crc = (b) => { let c = 0xffffffff; for (const x of b) c = crcTable[(c ^ x) & 255] ^ (c >>> 8); return (c ^ 0xffffffff) >>> 0; };
const chunk = (type, data) => { const t = Buffer.concat([Buffer.from(type), data]); const l = Buffer.alloc(4); l.writeUInt32BE(data.length); const c = Buffer.alloc(4); c.writeUInt32BE(crc(t)); return Buffer.concat([l, t, c]); };
const makePng = (w, h) => {
  const raw = Buffer.alloc((w + 1) * h); let s = 12345;
  for (let y = 0; y < h; y++) for (let x = 0; x < w; x++) { s = (s * 1103515245 + 12345) >>> 0; raw[y * (w + 1) + 1 + x] = s >>> 24; }
  const ihdr = Buffer.alloc(13); ihdr.writeUInt32BE(w, 0); ihdr.writeUInt32BE(h, 4); ihdr[8] = 8; ihdr[9] = 0;
  return Buffer.concat([Buffer.from([137, 80, 78, 71, 13, 10, 26, 10]), chunk('IHDR', ihdr), chunk('IDAT', zlib.deflateSync(raw)), chunk('IEND', Buffer.alloc(0))]);
};

const checks = {};
const check = (id, pass, detail) => { checks[id] = { pass: !!pass, detail }; };
const report = { schema: 'slam-refact-phone-smoke-v1', label: values.label, argv: process.argv, startedAt: new Date().toISOString(), node: process.version,
  inputKind: 'SYNTHETIC/RECORDED desktop input (flip6-s2 frames + synthetic DeviceMotion); not a phone, not a device result',
  requests: [], reqUrls: [], offOrigin: [], blockedExternal: [], nonGet: [], console: [], pageErrors: [], requestFailures: [], downloads: [] };
let server, browser;
try {
  const data = await loadFixture('flip6-s2', Number(values['max-frames']));
  report.input = { frames: data.frames.length, fixtureSha256: data.fixtureSha256 };
  const inputScript = await fs.readFile(path.join(repo, 'slam_refact/web-chunk/tests/sample-input.js'), 'utf8');
  const extra = async (p) => {
    if (p === '/input/fixture.json') return { body: data.fixtureBytes, type: 'application/json' };
    const m = /^\/input\/gray\/(\d+)$/.exec(p);
    return m ? { body: await data.gray(Number(m[1])), type: 'application/octet-stream' } : null;
  };
  server = await startSampleServer({ dirs: { engine: engineDir }, extra });
  const origin = server.origin;
  browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
    args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream'] });
  report.browserVersion = browser.version();
  const context = await browser.newContext({ viewport: { width: data.width, height: data.height }, permissions: ['camera'], isMobile: true, hasTouch: true, deviceScaleFactor: 1, userAgent: UA[data.ua], acceptDownloads: true });
  const page = await context.newPage();
  await page.addInitScript(inputScript);
  // Spy installed after the fake input: counts camera requests and motion-listener registrations made by the page.
  await page.addInitScript(() => {
    const spy = window.__spy = { gum: 0, sensorListeners: 0, storageWrites: 0 };
    const gum = navigator.mediaDevices.getUserMedia.bind(navigator.mediaDevices);
    navigator.mediaDevices.getUserMedia = (...a) => { spy.gum++; return gum(...a); };
    const add = window.addEventListener.bind(window);
    window.addEventListener = (type, ...r) => { if (type === 'devicemotion' || type === 'deviceorientation') spy.sensorListeners++; return add(type, ...r); };
    for (const k of ['setItem']) { const o = Storage.prototype[k]; Storage.prototype[k] = function (...a) { spy.storageWrites++; return o.apply(this, a); }; }
  });
  await page.route('**/*', (route) => {
    const u = route.request().url();
    if (!u.startsWith(origin + '/') && !u.startsWith('data:') && !u.startsWith('blob:')) { report.blockedExternal.push(u); return route.abort('blockedbyclient'); }
    return route.continue();
  });
  page.on('request', (r) => { report.reqUrls.push(r.url().replace(origin, '')); if (r.method() !== 'GET') report.nonGet.push(`${r.method()} ${r.url()}`); });
  page.on('response', (r) => { const u = r.url(); report.requests.push({ url: u.replace(origin, ''), status: r.status() }); if (!u.startsWith(origin + '/') && !u.startsWith('data:') && !u.startsWith('blob:')) report.offOrigin.push(u); });
  page.on('requestfailed', (r) => report.requestFailures.push({ url: r.url(), failure: r.failure() }));
  page.on('console', (m) => { if (report.console.length < 400) report.console.push({ phase: report.phase, type: m.type(), text: m.text().slice(0, 400) }); });
  page.on('pageerror', (e) => report.pageErrors.push({ phase: report.phase, message: e.message }));
  page.on('download', (d) => report.downloads.push({ name: d.suggestedFilename(), at: Date.now() }));

  // --- 1. load, before Start
  report.phase = 'load';
  await page.goto(`${origin}/samples/phone/index.html`);
  await page.waitForFunction(() => !document.getElementById('start').disabled, null, { timeout: 60000 });
  await page.waitForTimeout(1500);  // would catch a camera request made right after load
  const pre = await page.evaluate(() => ({ spy: window.__spy, cameraRequests: window.__input.cameraRequests.length, phase: window.__phoneApp.getState().phase,
    startClicked: window.__phoneApp.getState().startClicked, startScreenHidden: document.getElementById('start-screen').hidden, hudHidden: document.getElementById('hud').hidden,
    xr8: !!window.XR8, tracks: window.__input.started }));
  report.pre = pre;
  check('P1-no-camera-before-start', pre.spy.gum === 0 && pre.cameraRequests === 0 && !pre.startClicked, JSON.stringify(pre));
  check('P2-no-sensor-listener-before-start', pre.spy.sensorListeners === 0, `sensorListeners=${pre.spy.sensorListeners}`);
  const preReq = report.reqUrls.filter((u) => u.startsWith('/engine/'));
  check('P2b-engine-not-loaded-before-start', !pre.xr8 && preReq.length === 0, `window.XR8=${pre.xr8} engineRequests=${JSON.stringify(preReq)}`);
  check('P3-start-visible-hud-hidden', !pre.startScreenHidden && pre.hudHidden && pre.phase === 'idle', `phase=${pre.phase}`);
  await page.screenshot({ path: path.join(out, 'pre-start.png') });

  // --- 2. Start (a user tap), the status panel follows the engine
  report.phase = 'run';
  await page.touchscreen.tap(...await page.evaluate(() => { const r = document.getElementById('start').getBoundingClientRect(); return [r.x + r.width / 2, r.y + r.height / 2]; }));
  await page.waitForFunction(() => window.__phoneApp.getState().phase === 'running', null, { timeout: 60000 });
  const afterStart = await page.evaluate(() => ({ gum: window.__spy.gum, sensorListeners: window.__spy.sensorListeners }));
  check('P4-camera-requested-after-start', afterStart.gum >= 1 && afterStart.sensorListeners > 0 /* >0: positive control of P2 */, JSON.stringify(afterStart));
  await page.waitForFunction(() => window.__phoneApp.getState().status !== 'UNSPECIFIED' || window.__input.done, null, { timeout: 120000, polling: 200 });
  const normalWait = await page.waitForFunction(() => window.__phoneApp.getState().status === 'NORMAL' || window.__input.done, null, { timeout: 120000, polling: 200 }).then(() => true, () => false);
  const panel = await page.evaluate(() => ({ status: document.getElementById('h-status').textContent, reason: document.getElementById('h-reason').textContent, points: document.getElementById('h-points').textContent,
    ups: document.getElementById('h-ups').textContent, orient: document.getElementById('h-orient').textContent, video: document.getElementById('h-video').textContent, st: window.__phoneApp.getState() }));
  report.panel = panel;
  check('P5-status-panel-updates', panel.status !== '-' && panel.status.length > 0 && /portrait/.test(panel.orient) && /\d+×\d+/.test(panel.video), JSON.stringify({ status: panel.status, reason: panel.reason, points: panel.points, orient: panel.orient, video: panel.video }));
  check('P6-reached-NORMAL', normalWait && panel.st.status === 'NORMAL', `status=${panel.st.status} reason=${panel.st.reason} worldPoints=${panel.st.worldPoints} (synthetic input)`);
  await page.screenshot({ path: path.join(out, 'running.png') });

  // --- 3. tap -> hit test -> object (up to 6 attempts, spaced so the engine gets new frames)
  const taps = [];
  for (let i = 0; i < 6; i++) {
    await page.touchscreen.tap(data.width / 2, data.height / 2);
    await page.waitForTimeout(700);
    const s = await page.evaluate(() => ({ n: window.__phoneApp.getState().objectCount, msg: document.getElementById('h-msg').textContent }));
    taps.push(s);
    if (s.n > 0) break;
  }
  report.taps = taps;
  check('P7-tap-places-object', taps.at(-1).n > 0, JSON.stringify(taps.at(-1)));
  await page.screenshot({ path: path.join(out, 'placed.png') });

  // --- 4. menu: clear objects; opt-in metrics download (no download before the click)
  report.phase = 'menu';
  check('P8-no-download-before-click', report.downloads.length === 0, `downloads=${report.downloads.length}`);
  await page.click('#panel summary');
  await page.click('#b-clear');
  check('P9-clear-objects', (await page.evaluate(() => window.__phoneApp.getState().objectCount)) === 0, 'objectCount after clear');
  const [dl] = await Promise.all([page.waitForEvent('download'), page.click('#b-metrics')]);
  const dlPath = path.join(out, 'metrics-download.json');
  await dl.saveAs(dlPath);
  const mText = await fs.readFile(dlPath, 'utf8');
  const m = JSON.parse(mText);
  const forbidden = /"(image|pixels|frame|imu|gyro|accel|rotationRate|devicemotion|samples)"\s*:/i;
  check('P10-metrics-json-opt-in', report.downloads.length === 1 && m.schema === 'slam-refact-phone-metrics-v1' && m.updates > 0 && !forbidden.test(mText) && mText.length < 200000,
    `bytes=${mText.length} updates=${m.updates} timeToFirstNormalMs=${m.timeToFirstNormalMs} placed=${m.placed} hits=${m.hitTests}/${m.hitMisses} miss`);
  report.metricsSummary = { updates: m.updates, timeToFirstNormalMs: m.timeToFirstNormalMs, updateIntervalMs: m.updateIntervalMs, worldPoints: m.worldPoints, transitions: m.transitions.length, hitTests: m.hitTests };

  // --- 5. image target loader (page-side checks; the engine's acceptance of a synthetic target is not claimed)
  report.phase = 'targets';
  const bundled = await page.evaluate(() => [...document.querySelectorAll('#demo-targets input[type=checkbox]')].map((b) => b.dataset.name));
  report.bundledNames = bundled;
  for (const name of bundled) await page.click(`#demo-targets input[data-name="${name}"]`);
  await page.waitForTimeout(800);
  const bt = await page.evaluate(() => ({ st: window.__phoneApp.getState(), boxes: [...document.querySelectorAll('#demo-targets input')].map((b) => [b.dataset.name, b.checked]), err: document.getElementById('errors').textContent }));
  check('P13b-bundled-targets-load', bundled.length > 0 && bt.st.targets.filter((t) => t.source === 'demo').length === bundled.length && bt.boxes.every(([, on]) => on),
    `bundled=${JSON.stringify(bundled)} listed=${JSON.stringify(bt.st.targets.map((t) => t.name))} (page accepts; engine acceptance of curved targets is separate, see chunk errors in result.console)`);
  const planar = Buffer.from(JSON.stringify({ name: 'smoke-planar', type: 'PLANAR', metadata: null }));
  await page.setInputFiles('#f-json', { name: 'smoke-planar.json', mimeType: 'application/json', buffer: planar });
  await page.setInputFiles('#f-image', { name: 'smoke.png', mimeType: 'image/png', buffer: makePng(64, 64) });
  await page.click('#b-add-target');
  await page.waitForTimeout(500);
  const t1 = await page.evaluate(() => window.__phoneApp.getState());
  check('P11-picked-target-listed', t1.targets.some((t) => t.name === 'smoke-planar') && t1.targets.find((t) => t.name === 'smoke-planar').source === 'file', JSON.stringify(t1.targets));
  const curvy = Buffer.from(JSON.stringify({ name: 'smoke-bad', type: 'SPHERE' }));
  await page.setInputFiles('#f-json', { name: 'smoke-bad.json', mimeType: 'application/json', buffer: curvy });
  await page.setInputFiles('#f-image', { name: 'smoke.png', mimeType: 'image/png', buffer: makePng(64, 64) });
  await page.click('#b-add-target');
  await page.waitForTimeout(300);
  const t2 = await page.evaluate(() => ({ st: window.__phoneApp.getState(), err: document.getElementById('errors').textContent }));
  check('P12-unsupported-target-rejected-visibly', t2.st.targets.length === t1.targets.length && /지원하지 않습니다/.test(t2.err), t2.err.split('\n').slice(-1)[0]);
  const manifestRes = report.requests.find((r) => r.url === '/samples/phone/targets/manifest.json');
  check('P13-bundled-manifest-read', manifestRes && manifestRes.status === 200, JSON.stringify(manifestRes));
  report.targetsAfter = t2.st.targets;

  // --- 6. rotate the viewport: the page only shows values (what the chunk does is the chunk's business)
  report.phase = 'landscape';
  const errorsBeforeRotate = report.pageErrors.length;
  await page.setViewportSize({ width: data.height, height: data.width });
  await page.waitForTimeout(1500);
  const land = await page.evaluate(() => ({ orient: document.getElementById('h-orient').textContent, status: document.getElementById('h-status').textContent, msg: document.getElementById('h-msg').textContent,
    video: document.getElementById('h-video').textContent, errors: document.getElementById('errors').textContent.split('\n').slice(-3) }));
  report.landscape = land;
  check('P14-landscape-value-shown', /landscape/.test(land.orient), land.orient);
  await page.screenshot({ path: path.join(out, 'landscape.png') });
  report.landscapePageErrors = report.pageErrors.length - errorsBeforeRotate;

  // --- 7. nothing stored
  report.phase = 'storage';
  const storage = await page.evaluate(async () => ({ local: localStorage.length, session: sessionStorage.length, cookie: document.cookie, writes: window.__spy.storageWrites,
    idb: (await indexedDB.databases()).map((d) => d.name), caches: await caches.keys(), opfs: await (async () => { try { const root = await navigator.storage.getDirectory(); let n = 0; for await (const _ of root.keys()) n++; return n; } catch (e) { return String(e); } })() }));
  report.storage = storage;
  check('P15-nothing-stored', storage.local === 0 && storage.session === 0 && storage.cookie === '' && storage.writes === 0 && storage.idb.length === 0 && storage.caches.length === 0 && storage.opfs === 0, JSON.stringify(storage));

  try { await page.evaluate(() => window.__finishInput?.()); } catch (e) { report.cleanupError = String(e); }
  const core = (c) => c.phase !== 'targets' && c.phase !== 'landscape';
  const consoleErrors = report.console.filter((c) => c.type === 'error');
  report.consoleErrors = consoleErrors;
  check('P16-no-console-or-page-errors-portrait', consoleErrors.filter(core).length === 0 && report.pageErrors.filter(core).length === 0,
    `core phases (load/run/menu/storage): console.error=${consoleErrors.filter(core).length} pageErrors=${report.pageErrors.filter(core).length}; targets-phase console.error=${consoleErrors.filter((c) => c.phase === 'targets').length} (engine rejects the synthetic, non-engine-valid target JSON; reported, not hidden); landscape-phase console.error=${consoleErrors.filter((c) => c.phase === 'landscape').length} pageErrors=${report.pageErrors.filter((c) => c.phase === 'landscape').length}`);
  check('P17-no-external-or-nonget', report.offOrigin.length === 0 && report.blockedExternal.length === 0 && report.nonGet.length === 0 && report.requestFailures.length === 0,
    `offOrigin=${report.offOrigin.length} blocked=${report.blockedExternal.length} nonGet=${report.nonGet.length} failed=${report.requestFailures.length} requests=${report.requests.length}`);
  report.status = 'COMPLETED';
} catch (error) {
  report.status = 'FAILED'; report.error = { message: error.message, stack: error.stack };
} finally {
  if (browser) await browser.close();
  if (server) await server.close();
  report.finishedAt = new Date().toISOString();
  report.checks = checks;
  report.failed = Object.entries(checks).filter(([, c]) => !c.pass).map(([k]) => k);
  report.pass = report.status === 'COMPLETED' && report.failed.length === 0;
  await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(report, null, 1) + '\n');
}
console.log(JSON.stringify({ label: values.label, status: report.status, pass: report.pass, failed: report.failed, out: path.relative(repo, out), error: report.error?.message,
  checks: Object.fromEntries(Object.entries(checks).map(([k, c]) => [k, c.pass])) }, null, 1));
process.exitCode = report.pass ? 0 : 1;
