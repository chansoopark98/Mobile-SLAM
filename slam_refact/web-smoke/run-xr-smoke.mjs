#!/usr/bin/env node
// Headless smoke of the deployed-layout XR entry page (slam-refact-xr.html + slam-refact-xr/**), desktop Chromium with mobile emulation (NOT a phone).
//
// Server: web/server.js itself (createServer: real TLS certificate, headers, allowlist), started IN THIS PROCESS on 127.0.0.1 with a free port and
// webRoot = --root (a staging dir made by slam_refact/tools/deploy_xr.py, or web/ after a promotion). It is closed at the end. Chromium reaches it as
// https://dev.serdic.com:<port> through --host-resolver-rules (no DNS, no outside connection). Anything else is blocked and counted.
// Camera and motion sensors are the RECORDED flip6-s2 frames / synthetic DeviceMotion of web-chunk/tests/sample-input.js (a synthetic input, never a
// device result); the harness serves them with page.route (/input/*), they are not files of the server under test.
//
//   node slam_refact/web-smoke/run-xr-smoke.mjs --label <[a-z0-9-]+> --root <staging web root> [--max-frames 300]
// Output (never overwritten): build/slam-refact-w-20261023/smoke/<label>/{result.json,*.png}
import fs from 'node:fs/promises';
import { createRequire } from 'node:module';
import https from 'node:https';
import path from 'node:path';
import crypto from 'node:crypto';
import { parseArgs } from 'node:util';
import { chromium } from '../../scripts/dev/node_modules/playwright/index.mjs';
import { loadFixture, UA } from '../web-chunk/tests/sample-fixture.mjs';
import { CURRENT_INTEGRITY_PATH, verifyXrTreeIntegrity } from './xr-tree-integrity.mjs';
import { openDetailsIfClosed } from './xr-smoke-ui.mjs';

const repo = path.resolve(import.meta.dirname, '../..');
const { values } = parseArgs({ options: { label: { type: 'string' }, root: { type: 'string' }, 'max-frames': { type: 'string', default: '300' } } });
if (!/^[a-z0-9-]+$/.test(values.label || '')) throw new Error('--label [a-z0-9-]+ required (a label is never reused)');
if (!values.root) throw new Error('--root <web root to serve> required');
const root = path.resolve(repo, values.root);
const out = path.join(repo, 'build/slam-refact-w-20261023/smoke', values.label);
await fs.mkdir(path.dirname(out), { recursive: true });
await fs.mkdir(out);
const HOST = 'dev.serdic.com';

const checks = {};
const check = (id, pass, detail) => { checks[id] = { pass: !!pass, detail }; };
const report = { schema: 'slam-refact-xr-smoke-v1', label: values.label, argv: process.argv, startedAt: new Date().toISOString(), node: process.version, root: path.relative(repo, root),
  inputKind: 'SYNTHETIC/RECORDED desktop input (flip6-s2 frames + synthetic DeviceMotion); not a phone, not a device result', serverPid: process.pid,
  pages: {}, tree: {}, probes: [] };
const sha = (b) => crypto.createHash('sha256').update(b).digest('hex');
const walk = async (dir, base = dir) => (await Promise.all((await fs.readdir(dir, { withFileTypes: true })).map((e) => (e.isDirectory() ? walk(path.join(dir, e.name), base) : [path.relative(base, path.join(dir, e.name))])))).flat();

const nodeGet = (port, p, method = 'GET') => new Promise((resolve, reject) => {
  const req = https.request({ host: '127.0.0.1', port, path: p, method, servername: HOST, headers: { Host: `${HOST}:${port}` } }, (res) => {
    const chunks = []; res.on('data', (c) => chunks.push(c)); res.on('end', () => resolve({ status: res.statusCode, headers: res.headers, body: Buffer.concat(chunks) }));
  });
  req.on('error', reject); req.end();
});

let server, browser;
try {
  const require = createRequire(import.meta.url);
  const { createServer } = require(path.join(repo, 'web/server.js'));
  server = createServer({ webRoot: root });
  await new Promise((resolve, reject) => { server.once('error', reject); server.listen(0, '127.0.0.1', resolve); });
  const port = server.address().port;
  const origin = `https://${HOST}:${port}`;
  report.server = { port, origin, tls: 'web/server.js createServer (real certificate)' };

  // ---- 1. every file is served with equal sha256; current integrity or staged deployment metadata binds the whole tree
  const files = (await walk(root)).filter((f) => f === 'slam-refact-xr.html' || f.startsWith('slam-refact-xr/')).sort();
  const treeMiss = [];
  const contentFiles = [];
  const digest = crypto.createHash('sha256');
  for (const f of files) {
    const r = await nodeGet(port, '/' + f);
    const bytes = await fs.readFile(path.join(root, f));
    if (r.status !== 200 || sha(r.body) !== sha(bytes)) treeMiss.push({ f, status: r.status });
    if (f !== 'slam-refact-xr/deploy.json') {
      digest.update(f + '\0' + sha(bytes) + '\n');
      contentFiles.push({ path: f, bytes: bytes.length, sha256: sha(bytes) });
    }
  }
  const deploy = JSON.parse(await fs.readFile(path.join(root, 'slam-refact-xr/deploy.json'), 'utf8'));
  report.tree = { files: files.length, bundleSha256: digest.digest('hex'), deployBundleSha256: deploy.bundle_sha256, recordKind: deploy.record_kind ?? 'deployment-receipt', engineWasm: deploy.engine?.wasm_sha256 };
  check('T1-every-staged-file-served-with-equal-sha', files.length > 0 && treeMiss.length === 0, `files=${files.length} miss=${JSON.stringify(treeMiss)}`);
  const integrityPath = root === path.join(repo, 'web') ? CURRENT_INTEGRITY_PATH : 'slam-refact-xr/deploy.json';
  try {
    // Only the canonical project web tree uses this fixed, checked-in runtime inventory.
    // Staging roots must match their own artifact index; metadata cannot redirect reads.
    const currentIntegrity = root === path.join(repo, 'web')
      ? JSON.parse(await fs.readFile(path.join(repo, CURRENT_INTEGRITY_PATH), 'utf8')) : undefined;
    report.tree.integrity = verifyXrTreeIntegrity({ files: contentFiles, deploy, currentIntegrity });
    check('T2-staged-tree-matches-deploy-json-digest', true, `verified ${report.tree.integrity.metadataPath}: ${report.tree.integrity.bundleSha256}`);
  } catch (error) {
    report.tree.integrity = { verified: false, metadataPath: integrityPath, error: error.message };
    check('T2-staged-tree-matches-deploy-json-digest', false, error.message);
  }
  const probes = [['/server.js', 403], ['/package.json', 403], ['/slam-refact-xr/engine/other.js', 403], ['/slam-refact-xr/engine/xr.js.map', 403], ['/slam-refact-xr/samples/phone/targets/MANIFEST.sha256', 403],
    ['/slam-refact-xr/samples/phone/index.html', 404], ['/slam-refact-xr/%2e%2e/server.js', 403], ['/slam-refact-xr/engine/%2e%2e/%2e%2e/server.js', 403], ['/slam-refact-xr/samples/phone/.omc/x.json', 403]];
  for (const [p, want] of probes) { const r = await nodeGet(port, p); report.probes.push({ p, status: r.status, want }); }
  const alias = await nodeGet(port, '/slam-refact-xr?x=1');
  report.probes.push({ p: '/slam-refact-xr?x=1', status: alias.status, location: alias.headers.location, want: 302 });
  const post = await new Promise((resolve, reject) => { const req = https.request({ host: '127.0.0.1', port, path: '/slam-refact-xr.html', method: 'POST', servername: HOST, headers: { Host: `${HOST}:${port}` } }, (res) => { res.resume(); res.on('end', () => resolve(res.statusCode)); }); req.on('error', reject); req.end(); });
  report.probes.push({ p: 'POST /slam-refact-xr.html', status: post, want: 405 });
  check('T3-hostile-and-unlisted-paths-denied-alias-and-method', report.probes.filter((x) => x.p !== '/slam-refact-xr?x=1' && x.p !== 'POST /slam-refact-xr.html').every((x) => x.status === x.want)
    && alias.status === 302 && alias.headers.location === '/slam-refact-xr.html?x=1' && post === 405, JSON.stringify(report.probes.map((x) => `${x.p}=${x.status}`)));
  const hdr = (await nodeGet(port, '/slam-refact-xr.html')).headers;
  check('T4-security-headers', hdr['cross-origin-embedder-policy'] === 'credentialless' && hdr['cross-origin-opener-policy'] === 'same-origin' && /camera=\(self\)/.test(hdr['permissions-policy']) && /no-store/.test(hdr['cache-control']), JSON.stringify({ coep: hdr['cross-origin-embedder-policy'], coop: hdr['cross-origin-opener-policy'], pp: hdr['permissions-policy'], cc: hdr['cache-control'] }));

  // ---- 2. browser
  const data = await loadFixture('flip6-s2', Number(values['max-frames']));
  report.input = { frames: data.frames.length, fixtureSha256: data.fixtureSha256 };
  const inputScript = await fs.readFile(path.join(repo, 'slam_refact/web-chunk/tests/sample-input.js'), 'utf8');
  browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
    args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', `--host-resolver-rules=MAP ${HOST} 127.0.0.1`] });
  report.browserVersion = browser.version();

  // A fresh context per page: fake input + spies, only the origin allowed, everything counted.
  const openPage = async (name) => {
    const rec = { name, requests: [], reqUrls: [], inputRequests: 0, offOrigin: [], blockedExternal: [], nonGet: [], console: [], pageErrors: [], requestFailures: [], downloads: [], phase: 'load' };
    const context = await browser.newContext({ viewport: { width: data.width, height: data.height }, permissions: ['camera'], isMobile: true, hasTouch: true, deviceScaleFactor: 1, userAgent: UA[data.ua], acceptDownloads: true });
    const page = await context.newPage();
    await page.addInitScript(inputScript);
    await page.addInitScript(() => {
      const spy = window.__spy = { gum: 0, sensorListeners: 0, storageWrites: 0 };
      const gum = navigator.mediaDevices.getUserMedia.bind(navigator.mediaDevices);
      navigator.mediaDevices.getUserMedia = (...a) => { spy.gum++; return gum(...a); };
      const add = window.addEventListener.bind(window);
      window.addEventListener = (type, ...r) => { if (type === 'devicemotion' || type === 'deviceorientation') spy.sensorListeners++; return add(type, ...r); };
      const o = Storage.prototype.setItem; Storage.prototype.setItem = function (...a) { spy.storageWrites++; return o.apply(this, a); };
    });
    await page.route('**/*', async (route) => {
      const u = route.request().url();
      if (u.startsWith(origin + '/input/')) {
        rec.inputRequests++;
        const p = new URL(u).pathname;
        if (p === '/input/fixture.json') return route.fulfill({ status: 200, contentType: 'application/json', body: data.fixtureBytes });
        const m = /^\/input\/gray\/(\d+)$/.exec(p);
        if (m) return route.fulfill({ status: 200, contentType: 'application/octet-stream', body: await data.gray(Number(m[1])) });
        return route.fulfill({ status: 404, body: 'no' });
      }
      if (!u.startsWith(origin + '/') && !u.startsWith('data:') && !u.startsWith('blob:')) { rec.blockedExternal.push(u); return route.abort('blockedbyclient'); }
      return route.continue();
    });
    page.on('request', (r) => { rec.reqUrls.push(r.url().replace(origin, '')); if (r.method() !== 'GET') rec.nonGet.push(`${r.method()} ${r.url()}`); });
    page.on('response', (r) => { const u = r.url(); rec.requests.push({ url: u.replace(origin, ''), status: r.status() }); if (!u.startsWith(origin + '/') && !u.startsWith('data:') && !u.startsWith('blob:')) rec.offOrigin.push(u); });
    page.on('requestfailed', (r) => rec.requestFailures.push({ url: r.url(), failure: r.failure() }));
    page.on('console', (m) => { if (rec.console.length < 300) rec.console.push({ phase: rec.phase, type: m.type(), text: m.text().slice(0, 400) }); });
    page.on('pageerror', (e) => rec.pageErrors.push({ phase: rec.phase, message: e.message }));
    page.on('download', (d) => rec.downloads.push({ name: d.suggestedFilename() }));
    report.pages[name] = rec;
    return { page, context, rec };
  };
  const finishInput = async (page) => { try { await page.evaluate(() => window.__finishInput?.()); } catch (e) { /* page already gone */ } };
  const netChecks = (id, rec, allow404 = []) => {
    const bad = rec.requests.filter((r) => r.status >= 400 && !allow404.includes(r.url));
    check(`${id}-no-external-nonget-failed-or-error-status`, rec.offOrigin.length === 0 && rec.blockedExternal.length === 0 && rec.nonGet.length === 0 && rec.requestFailures.length === 0 && bad.length === 0,
      `offOrigin=${rec.offOrigin.length} blocked=${rec.blockedExternal.length} nonGet=${rec.nonGet.length} failed=${rec.requestFailures.length} status>=400=${JSON.stringify(bad)} requests=${rec.requests.length} (+${rec.inputRequests} harness /input/*)`);
  };

  // ---- 2a. entry page (= phone app) via the typed alias
  {
    const { page, context, rec } = await openPage('entry');
    await page.goto(`${origin}/slam-refact-xr`);
    const finalUrl = page.url();
    await page.waitForFunction(() => !document.getElementById('start').disabled, null, { timeout: 60000 });
    await page.waitForTimeout(1500);
    const pre = await page.evaluate(() => ({ spy: window.__spy, cameraRequests: window.__input.cameraRequests.length, phase: window.__phoneApp.getState().phase, startClicked: window.__phoneApp.getState().startClicked,
      startScreenHidden: document.getElementById('start-screen').hidden, hudHidden: document.getElementById('hud').hidden, xr8: !!window.XR8, isSecureContext, coi: crossOriginIsolated,
      links: [...document.querySelectorAll('#xr-links a')].map((a) => a.getAttribute('href')), baseURI: document.baseURI }));
    rec.pre = pre;
    check('E0-alias-lands-on-entry-page-secure-context', /\/slam-refact-xr\.html$/.test(finalUrl) && pre.isSecureContext, `url=${finalUrl} isSecureContext=${pre.isSecureContext} crossOriginIsolated=${pre.coi} base=${pre.baseURI}`);
    check('E1-no-camera-before-start', pre.spy.gum === 0 && pre.cameraRequests === 0 && !pre.startClicked, JSON.stringify({ gum: pre.spy.gum, cameraRequests: pre.cameraRequests, startClicked: pre.startClicked }));
    check('E2-no-sensor-listener-no-engine-before-start', pre.spy.sensorListeners === 0 && !pre.xr8 && rec.reqUrls.filter((u) => u.startsWith('/slam-refact-xr/engine/')).length === 0,
      `sensorListeners=${pre.spy.sensorListeners} XR8=${pre.xr8} engineRequests=${JSON.stringify(rec.reqUrls.filter((u) => u.startsWith('/slam-refact-xr/engine/')))}`);
    check('E3-start-visible-hud-hidden', !pre.startScreenHidden && pre.hudHidden && pre.phase === 'idle', `phase=${pre.phase}`);
    const resolvedLinks = [];
    for (const href of pre.links) resolvedLinks.push(await page.evaluate((h) => new URL(h, document.baseURI).pathname, href));
    const linkStatus = [];
    for (const p of resolvedLinks) linkStatus.push({ p, status: (await nodeGet(port, p)).status });
    check('E4-sample-links-resolve-and-are-served', pre.links.length >= 6 && linkStatus.every((l) => l.status === 200) && ['threejs', 'aframe', 'demo-cylinder-sheet', 'demo-flat-sheet', 'deploy.json'].every((k) => resolvedLinks.some((p) => p.includes(k))), JSON.stringify(linkStatus));
    await page.screenshot({ path: path.join(out, 'entry-pre-start.png') });

    rec.phase = 'run';
    await page.touchscreen.tap(...await page.evaluate(() => { const r = document.getElementById('start').getBoundingClientRect(); return [r.x + r.width / 2, r.y + r.height / 2]; }));
    await page.waitForFunction(() => window.__phoneApp.getState().phase === 'running', null, { timeout: 90000 });
    const after = await page.evaluate(() => ({ gum: window.__spy.gum, sensorListeners: window.__spy.sensorListeners }));
    check('E5-camera-and-sensors-only-after-start', after.gum >= 1 && after.sensorListeners > 0, JSON.stringify(after));
    const normal = await page.waitForFunction(() => window.__phoneApp.getState().status === 'NORMAL' || window.__input.done, null, { timeout: 120000, polling: 200 }).then(() => true, () => false);
    const st = await page.evaluate(() => ({ s: window.__phoneApp.getState(), status: document.getElementById('h-status').textContent, video: document.getElementById('h-video').textContent }));
    rec.state = st.s;
    const served = Object.fromEntries(['xr.js', 'xr-slam.js', 'engine-loader.js', 'slam-refact-engine/manifest.json', 'slam-refact-engine/slam_refact_engine.js', 'slam-refact-engine/slam_refact_engine.wasm']
      .map((f) => [f, rec.requests.find((r) => r.url === `/slam-refact-xr/engine/${f}`)?.status ?? null]));
    check('E6-chunk-and-engine-loaded-from-deployed-layout', Object.values(served).every((s) => s === 200), JSON.stringify(served));
    check('E7-reached-NORMAL', normal && st.s.status === 'NORMAL', `status=${st.s.status} reason=${st.s.reason} worldPoints=${st.s.worldPoints} video=${st.video} (synthetic input; NOT a device result)`);
    await page.screenshot({ path: path.join(out, 'entry-running.png') });
    // try1 tapped 6 times at 0.7 s from the first NORMAL while worldPoints was still 0 and failed; from try2 on the harness taps only while worldPoints > 0,
    // every 0.5 s, until an object exists or the recorded input is exhausted (a tap before any world point can only report "no surface").
    const taps = [];
    for (;;) {
      const s = await page.evaluate(() => ({ n: window.__phoneApp.getState().objectCount, wp: window.__phoneApp.getState().worldPoints, done: window.__input.done, msg: document.getElementById('h-msg').textContent }));
      if (s.n > 0 || s.done || taps.length >= 200) { taps.push({ ...s, final: true }); break; }
      if (s.wp > 0) { await page.touchscreen.tap(data.width / 2, data.height / 2); taps.push(s); }
      await page.waitForTimeout(500);
    }
    rec.taps = { count: taps.length, first: taps[0], last: taps.at(-1) };
    check('E8-tap-places-object', taps.at(-1).n > 0, `${JSON.stringify(rec.taps.last)} tapsAfterWorldPoints=${taps.length - 1} (package B reported this check as FLAKY: replay timing/load dependent)`);
    rec.phase = 'targets';
    await page.locator('#panel > summary').click();
    await openDetailsIfClosed(page.locator('#demo-targets').locator('xpath=ancestor::details[1]'));
    const names = await page.evaluate(() => [...document.querySelectorAll('#demo-targets input[type=checkbox]')].map((b) => b.dataset.name));
    for (const n of names) { await page.click(`#demo-targets input[data-name="${n}"]`); await page.waitForTimeout(500); }
    await page.waitForTimeout(800);
    const tg = await page.evaluate(() => ({ st: window.__phoneApp.getState(), boxes: [...document.querySelectorAll('#demo-targets input')].map((b) => [b.dataset.name, b.checked]), err: document.getElementById('errors').textContent }));
    rec.targets = { names, listed: tg.st.targets.map((t) => t.name), errorText: tg.err, chunkWarnings: tg.st.chunkWarnings };
    check('E9-bundled-targets-offered-and-loaded-by-page', names.includes('demo-flat') && names.includes('demo-cylinder') && tg.st.targets.length === names.length, `names=${JSON.stringify(names)} listed=${JSON.stringify(rec.targets.listed)}`);
    const targetRes = rec.requests.filter((r) => r.url.includes('/samples/phone/targets/')).map((r) => `${r.url.split('/').pop()}=${r.status}`);
    check('E10-target-files-served', names.length > 0 && targetRes.length >= 1 + 2 * names.length && targetRes.every((x) => x.endsWith('=200')), JSON.stringify(targetRes));
    rec.curvyObservation = `engine/chunk answer to the bundled targets (page error area): ${JSON.stringify(tg.err)}; chunkWarnings=${JSON.stringify(tg.st.chunkWarnings)} (reported, not a pass criterion: found/lost of a target needs a printed target in the camera view, UNVERIFIED)`;
    await page.screenshot({ path: path.join(out, 'entry-targets.png') });
    rec.phase = 'storage';
    const storage = await page.evaluate(async () => ({ local: localStorage.length, session: sessionStorage.length, cookie: document.cookie, writes: window.__spy.storageWrites, idb: (await indexedDB.databases()).map((d) => d.name), caches: await caches.keys() }));
    rec.storage = storage;
    check('E11-nothing-stored', storage.local === 0 && storage.session === 0 && storage.cookie === '' && storage.writes === 0 && storage.idb.length === 0 && storage.caches.length === 0, JSON.stringify(storage));
    await finishInput(page);
    const core = rec.console.filter((c) => c.type === 'error' && c.phase !== 'targets');
    const coreErr = rec.pageErrors.filter((c) => c.phase !== 'targets');
    check('E12-no-console-or-page-errors-outside-targets-phase', core.length === 0 && coreErr.length === 0, `console.error=${core.length} pageErrors=${coreErr.length} ${JSON.stringify(core.slice(0, 3))}; targets-phase console.error=${rec.console.filter((c) => c.type === 'error' && c.phase === 'targets').length} pageErrors=${rec.pageErrors.filter((c) => c.phase === 'targets').length}`);
    netChecks('E13', rec);
    await context.close();
  }

  // ---- 2b. sample pages behind the start gate
  for (const sample of ['threejs', 'aframe']) {
    const { page, context, rec } = await openPage(sample);
    const id = sample === 'threejs' ? 'S' : 'A';
    await page.goto(`${origin}/slam-refact-xr/samples/${sample}/index.html?probe=1`);
    await page.waitForSelector('#xr-gate-start', { timeout: 60000 });
    await page.waitForTimeout(2500);  // xr.js loads eagerly here; a camera request or listener would show by now
    const pre = await page.evaluate(() => ({ spy: window.__spy, cameraRequests: window.__input.cameraRequests.length, xr8: !!window.XR8, gate: window.__xrGate.isReleased(), isSecureContext }));
    rec.pre = pre;
    check(`${id}1-${sample}-no-camera-no-sensor-before-tap`, pre.spy.gum === 0 && pre.cameraRequests === 0 && pre.spy.sensorListeners === 0 && !pre.gate && pre.isSecureContext, JSON.stringify({ gum: pre.spy.gum, cameraRequests: pre.cameraRequests, sensorListeners: pre.spy.sensorListeners, xr8Loaded: pre.xr8, gateReleased: pre.gate }));
    await page.screenshot({ path: path.join(out, `${sample}-gate.png`) });
    rec.phase = 'run';
    const box = await page.locator('#xr-gate-start').boundingBox();
    await page.touchscreen.tap(box.x + box.width / 2, box.y + box.height / 2);
    const ok = await page.waitForFunction(() => window.__xrSample && window.__xrSample.realityUpdates >= 20, null, { timeout: 120000, polling: 250 }).then(() => true, () => false);
    const post = await page.evaluate(() => ({ spy: window.__spy, gate: window.__xrGate.isReleased(), updates: window.__xrSample && window.__xrSample.realityUpdates, errors: window.__xrSample && window.__xrSample.errors, wp: window.__xrSample && window.__xrSample.firstWp }));
    rec.post = post;
    check(`${id}2-${sample}-runs-after-tap`, ok && post.gate && post.spy.gum >= 1 && post.spy.sensorListeners > 0 && post.errors.length === 0, JSON.stringify({ realityUpdates: post.updates, gum: post.spy.gum, sensorListeners: post.spy.sensorListeners, errors: post.errors, firstWp: post.wp }));
    const engineServed = ['xr.js', 'xr-slam.js', 'slam-refact-engine/slam_refact_engine.wasm'].map((f) => rec.requests.find((r) => r.url === `/slam-refact-xr/engine/${f}`)?.status ?? null);
    check(`${id}3-${sample}-chunk-and-engine-served-from-deployed-layout`, engineServed.every((s) => s === 200), JSON.stringify(engineServed));
    await page.screenshot({ path: path.join(out, `${sample}-running.png`) });
    await finishInput(page);
    const errs = rec.console.filter((c) => c.type === 'error');
    check(`${id}4-${sample}-no-console-or-page-errors`, errs.length === 0 && rec.pageErrors.length === 0, `console.error=${errs.length} ${JSON.stringify(errs.slice(0, 3))} pageErrors=${rec.pageErrors.length} ${JSON.stringify(rec.pageErrors.slice(0, 2))}`);
    netChecks(`${id}5`, rec);
    await context.close();
  }
  report.status = 'COMPLETED';
} catch (error) {
  report.status = 'FAILED'; report.error = { message: error.message, stack: error.stack };
} finally {
  if (browser) await browser.close();
  if (server) await new Promise((resolve) => { server.close(resolve); server.closeAllConnections?.(); });
  report.serverClosed = !!server && !server.listening;
  report.finishedAt = new Date().toISOString();
  report.checks = checks;
  report.failed = Object.entries(checks).filter(([, c]) => !c.pass).map(([k]) => k);
  report.pass = report.status === 'COMPLETED' && report.failed.length === 0;
  await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(report, null, 1) + '\n');
}
console.log(JSON.stringify({ label: values.label, status: report.status, pass: report.pass, failed: report.failed, out: path.relative(repo, out), error: report.error?.message,
  checks: Object.fromEntries(Object.entries(checks).map(([k, c]) => [k, c.pass])) }, null, 1));
process.exitCode = report.pass ? 0 : 1;
