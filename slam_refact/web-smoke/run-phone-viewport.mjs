#!/usr/bin/env node
// Reproducible viewport and interaction regression for the staged phone UI.
// Desktop Chromium emulation with recorded frames and synthetic sensors; not a phone or tracking-accuracy test.
// node slam_refact/web-smoke/run-phone-viewport.mjs --label <unique-label> --root <staged-web-root>
import fs from 'node:fs/promises';
import path from 'node:path';
import { createRequire } from 'node:module';
import { parseArgs } from 'node:util';
import { chromium } from '../../scripts/dev/node_modules/playwright/index.mjs';
import { loadFixture, UA } from '../web-chunk/tests/sample-fixture.mjs';

const repo = path.resolve(import.meta.dirname, '../..');
const { values } = parseArgs({ options: { label: { type: 'string' }, root: { type: 'string' } } });
if (!/^[a-z0-9-]+$/.test(values.label || '') || !values.root) throw new Error('--label <unique lowercase label> and --root <staged web root> are required');
const out = path.join(repo, 'build/slam-refact-w-20261023/viewport', values.label);
await fs.mkdir(path.dirname(out), { recursive: true });
await fs.mkdir(out); // never overwrite evidence
const report = { label: values.label, root: values.root, input: 'recorded Flip6 frames + synthetic sensors; desktop emulation only', scenarios: [], checks: {} };
const check = (name, pass, detail) => { report.checks[name] = { pass: !!pass, detail }; };
let server, browser;
try {
  const { createServer } = createRequire(import.meta.url)(path.join(repo, 'web/server.js'));
  server = createServer({ webRoot: path.resolve(repo, values.root) });
  await new Promise((resolve, reject) => { server.once('error', reject); server.listen(0, '127.0.0.1', resolve); });
  const origin = `https://dev.serdic.com:${server.address().port}`;
  const fixture = await loadFixture('flip6-s2', 300);
  const inputScript = await fs.readFile(path.join(repo, 'slam_refact/web-chunk/tests/sample-input.js'), 'utf8');
  report.fixtureSha256 = fixture.fixtureSha256;
  browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
    args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--host-resolver-rules=MAP dev.serdic.com 127.0.0.1'] });
  report.browserVersion = browser.version();
  for (const [name, width, height] of [['portrait', 390, 844], ['short', 360, 640], ['landscape', 844, 390]]) {
    const context = await browser.newContext({ viewport: { width, height }, permissions: ['camera'], isMobile: true, hasTouch: true, deviceScaleFactor: 2, userAgent: UA.pixel7 });
    const page = await context.newPage();
    const errors = [];
    page.on('pageerror', (error) => errors.push(error.message));
    await page.addInitScript(inputScript);
    await page.addInitScript(() => {
      window.__viewportSpy = { tracks: [], sensorListeners: 0 };
      const add = window.addEventListener.bind(window);
      window.addEventListener = (type, ...args) => { if (type === 'devicemotion' || type === 'deviceorientation') window.__viewportSpy.sensorListeners++; return add(type, ...args); };
      const gum = navigator.mediaDevices.getUserMedia.bind(navigator.mediaDevices);
      navigator.mediaDevices.getUserMedia = async (...args) => { const stream = await gum(...args); window.__viewportSpy.tracks.push(...stream.getTracks()); return stream; };
    });
    await page.route('**/*', async (route) => {
      const url = route.request().url();
      if (url === `${origin}/input/fixture.json`) return route.fulfill({ status: 200, contentType: 'application/json', body: fixture.fixtureBytes });
      const gray = /^\/input\/gray\/(\d+)$/.exec(new URL(url).pathname);
      if (url.startsWith(origin + '/') && gray) return route.fulfill({ status: 200, contentType: 'application/octet-stream', body: await fixture.gray(Number(gray[1])) });
      if (!url.startsWith(origin + '/') && !url.startsWith('blob:') && !url.startsWith('data:')) return route.abort();
      return route.continue();
    });
    await page.goto(`${origin}/slam-refact-xr`);
    await page.waitForFunction(() => window.__phoneApp && !document.querySelector('#start').disabled);
    const pre = await page.evaluate(() => ({ phase: __phoneApp.getState().phase, cameraRequests: __input.cameraRequests.length, sensorListeners: __viewportSpy.sensorListeners, xr8: !!window.XR8, tabIndex: document.querySelector('#camerafeed').tabIndex }));
    check(`${name}-no-permissions-before-start`, pre.phase === 'idle' && pre.cameraRequests === 0 && pre.sensorListeners === 0 && !pre.xr8, pre);
    check(`${name}-canvas-focus-follows-session`, pre.tabIndex === -1, pre.tabIndex);
    await page.screenshot({ path: path.join(out, `${name}-before.png`) });
    await page.locator('#start').click();
    await page.waitForFunction(() => __phoneApp.getState().phase === 'running', null, { timeout: 90000 });
    await page.waitForTimeout(2000);
    const view = await page.evaluate(() => {
      const canvas = document.querySelector('#camerafeed'), r = canvas.getBoundingClientRect();
      return { viewport: [innerWidth, innerHeight], dpr: devicePixelRatio, rect: [r.left, r.top, r.width, r.height], buffer: [canvas.width, canvas.height],
        center: document.elementFromPoint(innerWidth / 2, innerHeight / 2)?.id, cameraRequests: __input.cameraRequests.length,
        liveTracks: __viewportSpy.tracks.filter((track) => track.readyState === 'live').length, tabIndex: canvas.tabIndex, state: __phoneApp.getState() };
    });
    const near = (a, b) => Math.abs(a - b) <= 1;
    check(`${name}-canvas-fills-viewport`, near(view.rect[0], 0) && near(view.rect[1], 0) && near(view.rect[2], width) && near(view.rect[3], height), view);
    check(`${name}-camera-live-and-center-accessible`, view.cameraRequests > 0 && view.liveTracks > 0 && view.center === 'camerafeed', { cameraRequests: view.cameraRequests, liveTracks: view.liveTracks, center: view.center });
    await page.screenshot({ path: path.join(out, `${name}-running.png`) });
    await page.evaluate(() => {
      window.__hitCoordinates = [];
      const original = XR8.XrController.hitTest.bind(XR8.XrController);
      XR8.XrController.hitTest = (x, y, ...args) => { window.__hitCoordinates.push([x, y]); return original(x, y, ...args); };
    });
    await page.touchscreen.tap(width / 2, height / 2);
    const hit = await page.evaluate(() => window.__hitCoordinates);
    check(`${name}-center-tap-normalized`, hit.length === 1 && Math.abs(hit[0][0] - .5) < .01 && Math.abs(hit[0][1] - .5) < .01, hit);
    await page.locator('#camerafeed').focus();
    await page.keyboard.press('Enter');
    const keyboardHits = await page.evaluate(() => window.__hitCoordinates);
    const keyboardHit = keyboardHits.at(-1);
    check(`${name}-keyboard-center`, view.tabIndex === 0 && keyboardHits.length === 2 && Math.abs(keyboardHit[0] - .5) < .01 && Math.abs(keyboardHit[1] - .5) < .01, keyboardHits);
    await page.locator('#panel > summary').click();
    await page.locator('.advanced > summary').first().click();
    const expanded = await page.locator('.advanced').first().evaluate((element) => element.open);
    check(`${name}-target-controls-expand`, expanded, expanded);
    await page.locator('#b-stop').click();
    await page.waitForTimeout(300);
    const stop = await page.evaluate(() => ({ phase: __phoneApp.getState().phase, tracks: __viewportSpy.tracks.map((track) => track.readyState), restartVisible: !document.querySelector('#b-restart').hidden, tabIndex: document.querySelector('#camerafeed').tabIndex }));
    check(`${name}-stop-releases-camera`, stop.phase === 'stopped' && stop.tracks.length > 0 && stop.tracks.every((state) => state === 'ended') && stop.restartVisible && stop.tabIndex === -1, stop);
    check(`${name}-no-page-errors`, errors.length === 0, errors);
    report.scenarios.push({ name, pre, view, hit, stop, errors });
    try { await page.evaluate(() => window.__finishInput?.()); } catch { /* page already closed */ }
    await context.close();
  }
  report.status = 'COMPLETED';
} catch (error) {
  report.status = 'FAILED'; report.error = { message: error.message, stack: error.stack };
} finally {
  if (browser) await browser.close();
  if (server) await new Promise((resolve) => { server.close(resolve); server.closeAllConnections?.(); });
  report.serverClosed = !!server && !server.listening;
  report.failed = Object.entries(report.checks).filter(([, value]) => !value.pass).map(([name]) => name);
  await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(report, null, 2) + '\n');
  console.log(JSON.stringify({ status: report.status, failed: report.failed, error: report.error, out }, null, 2));
  if (report.status !== 'COMPLETED' || report.failed.length) process.exitCode = 1;
}
