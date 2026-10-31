// Screenshots of the slam_refact phone page at a phone viewport (Playwright Chromium mobile emulation, fake camera). Dev tool only.
//   node scripts/dev/phone-ux-shots.mjs --label after [--samples-dir <dir containing phone/ and common/>] [--out build/xr-ux-20261031] [--width 360 --height 744]
//     [--browser chromium|webkit] [--ua android|ios]
// An ephemeral 127.0.0.1 server serves the page (nothing is started on a fixed port); the camera and sensors are the recorded flip6-s2 input of
// slam_refact/web-chunk/tests/sample-input.js (same fake the sample smoke uses; xr.js needs sensor events to start a session on a desktop). Tracking on the
// fake camera is not a SLAM result: the HUD states after the first one are set through the page's own hud/guidance modules, and the file names say so.
import fs from 'node:fs/promises';
import path from 'node:path';
import { parseArgs } from 'node:util';
import { startSampleServer } from '../../slam_refact/web-chunk/samples/serve.mjs';
import { loadFixture } from '../../slam_refact/web-chunk/tests/sample-fixture.mjs';

const { values } = parseArgs({ options: { label: { type: 'string', default: 'after' }, 'samples-dir': { type: 'string' }, out: { type: 'string', default: 'build/xr-ux-20261031' },
  width: { type: 'string', default: '360' }, height: { type: 'string', default: '744' }, browser: { type: 'string', default: 'chromium' }, ua: { type: 'string', default: 'android' }, scale: { type: 'string', default: '2' } } });
const repo = path.resolve(import.meta.dirname, '../..');
const out = path.resolve(repo, values.out);
await fs.mkdir(out, { recursive: true });
const { chromium, webkit } = await import('./node_modules/playwright/index.mjs');
const UA = {
  android: 'Mozilla/5.0 (Linux; Android 14; SM-F741N) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Mobile Safari/537.36',
  ios: 'Mozilla/5.0 (iPhone; CPU iPhone OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) CriOS/126.0.6478.153 Mobile/15E148 Safari/604.1',
};
const dirs = values['samples-dir'] ? { samples: path.resolve(values['samples-dir']) } : {};
const data = await loadFixture('flip6-s2', 300);
const inputScript = await fs.readFile(path.join(repo, 'slam_refact/web-chunk/tests/sample-input.js'), 'utf8');
const extra = async (pathname) => {
  if (pathname === '/input/fixture.json') return { body: data.fixtureBytes, type: 'application/json' };
  const m = /^\/input\/gray\/(\d+)$/.exec(pathname);
  return m ? { body: await data.gray(Number(m[1])), type: 'application/octet-stream' } : null;
};
const srv = await startSampleServer({ dirs, extra });
const notes = { label: values.label, origin: srv.origin, samplesDir: values['samples-dir'] || 'repo', ua: values.ua, browser: values.browser, shots: [], console: [], pageErrors: [] };
let browser;
try {
  const isWebkit = values.browser === 'webkit';
  browser = await (isWebkit ? webkit : chromium).launch(isWebkit ? { headless: true } : { executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
    args: ['--enable-unsafe-swiftshader', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream', '--autoplay-policy=no-user-gesture-required'] });
  notes.browserVersion = browser.version();
  const context = await browser.newContext({ viewport: { width: +values.width, height: +values.height }, deviceScaleFactor: +values.scale, isMobile: true, hasTouch: true, userAgent: UA[values.ua],
    ...(isWebkit ? {} : { permissions: ['camera'] }) });
  const page = await context.newPage();
  if (!isWebkit) await page.addInitScript(inputScript);
  // iOS emulation: the permission functions exist only on iOS; record their order relative to the engine script request.
  if (values.ua === 'ios') await page.addInitScript(() => {
    window.__order = [];
    const mk = (n) => ({ requestPermission: () => { window.__order.push(n); return Promise.resolve('granted'); } });
    window.DeviceMotionEvent = Object.assign(window.DeviceMotionEvent || function () {}, mk('motion'));
    window.DeviceOrientationEvent = Object.assign(window.DeviceOrientationEvent || function () {}, mk('orientation'));
  });
  page.on('console', (m) => { if (['error', 'warning'].includes(m.type()) && notes.console.length < 40) notes.console.push(`${m.type()}: ${m.text().slice(0, 240)}`); });
  page.on('pageerror', (e) => notes.pageErrors.push(e.message));
  const shot = async (name, about) => { const f = `${values.label}-${name}.png`; await page.screenshot({ path: path.join(out, f) }); notes.shots.push({ file: f, about }); };
  await page.goto(`${srv.origin}/samples/phone/index.html`);
  await page.waitForTimeout(1600);  // hero animation finished
  await shot('1-start', 'start screen');
  const hasHud = await page.evaluate(() => !!window.__phoneApp?.hud);
  await page.click('#start');
  await page.waitForTimeout(500);
  await shot('2-starting', 'right after the Start tap (steps / engine loading)');
  const running = await page.waitForFunction(() => window.__phoneApp.getState().phase === 'running', null, { timeout: 40000 }).then(() => true, () => false);
  notes.reachedRunning = running;
  if (values.ua === 'ios') notes.permissionOrder = await page.evaluate(() => window.__order);
  await page.waitForTimeout(1500);
  await shot('3-hud-natural', `HUD as the engine reports it on the fake camera (reachedRunning=${running}); state ${JSON.stringify(await page.evaluate(() => { const s = window.__phoneApp.getState(); return { phase: s.phase, status: s.status, reason: s.reason, points: s.worldPoints }; }))}`);
  if (hasHud && running) {
    // The page's own onUpdate keeps rendering the real state every frame; freeze that so a forced state stays on screen for the screenshot.
    await page.evaluate(() => { const h = window.__phoneApp.hud; window.__realRender = h.render.bind(h); h.render = () => {}; });
    const force = (g) => page.evaluate(async (g) => { const m = await import('/samples/phone/guidance.js'); window.__realRender(m.trackingGuide({ phase: 'running', ...g })); }, g);
    await force({ status: 'LIMITED', reason: 'INITIALIZING' }); await shot('4-hud-init-forced', 'forced guidance state: initializing (fake)');
    await force({ status: 'LIMITED', reason: 'EXCESSIVE_MOTION', points: 12 }); await shot('5-hud-fast-forced', 'forced guidance state: excessive motion (fake)');
    await force({ status: 'LIMITED', reason: 'INSUFFICIENT_FEATURES', points: 6, notTrackingMs: 15000 }); await shot('6-hud-features-forced', 'forced guidance state: insufficient features, slow (fake)');
    await force({ status: 'NORMAL', objectCount: 2 });
    await page.evaluate(() => { const h = window.__phoneApp.hud; h.setCount(2); h.tapRing(180, 330, true); h.flash({ tone: 'ok', title: '물체를 놓았어요 (2개)', body: '다시 탭하면 하나 더 놓아요.' }, 60000); });
    await page.waitForTimeout(120); await shot('7-hud-placed-forced', 'forced: tracking ok, object placed, tap ring (fake)');
    await page.evaluate(() => { const h = window.__phoneApp.hud; h.tapRing(120, 420, false); });
    await page.waitForTimeout(150); await shot('8-hud-miss-forced', 'forced: tap miss ring (fake)');
    await page.evaluate(() => { document.getElementById('panel').open = true; }); await page.waitForTimeout(150); await shot('9-panel', 'tools sheet open');
    await page.evaluate(() => { document.getElementById('panel').open = false; window.__phoneApp.hud.showRecovery({ title: '카메라를 허용해 주세요', body: '주소창 왼쪽의 자물쇠 아이콘 > 권한 > 카메라를 허용으로 바꾼 뒤 다시 시도해 주세요.' }, 'NotAllowedError: Permission denied'); });
    await page.waitForTimeout(150); await shot('10-recovery', 'camera-denied recovery sheet (forced)');
  }
} finally {
  if (browser) await browser.close();
  await srv.close();
}
await fs.writeFile(path.join(out, `${values.label}-notes.json`), JSON.stringify(notes, null, 1));
console.log(JSON.stringify({ shots: notes.shots.map((s) => s.file), reachedRunning: notes.reachedRunning, permissionOrder: notes.permissionOrder, pageErrors: notes.pageErrors, console: notes.console.slice(0, 8) }, null, 1));
