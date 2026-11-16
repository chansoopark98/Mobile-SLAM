#!/usr/bin/env node
// Headless check of a STAGED bundle in three modes (default = no engine option, ?keepmap=1 = K1, ?keepmap=fast = K1F; K1/K1F are experimental
// opt-ins REJECTED by the 2026-11-16 gate (build/xr-commercial-20261116/verdict-K1-final.json, verdict-K1F-final.json), never a default): desktop
// Chromium with mobile emulation and the recorded flip6-s2 frames + synthetic DeviceMotion of tests/sample-input.js. NOT a phone, not a device
// result. Modelled on build/xr-tracker-diag-20261116/tools/run-diag-page-check.mjs (same server, fake input and Chromium launch).
//   node slam_refact/web-chunk/tests/run-product-page-check.mjs --root <staged web root> --out <new dir> [--max-frames 240] [--normal-timeout-s 240]
// Exit code 0 only when, in every mode, NORMAL is reached with 0 page errors, and the engine read-back (window.__phoneApp.getEngineInfo()) shows:
// default: no engineConfig sent, accuracy.enabled false and every k1* flag false; keepmap=1: the K1 flags true, not ignored, k1FastExtend false;
// keepmap=fast: every K1F flag true and not ignored.
import crypto from 'node:crypto';
import fs from 'node:fs/promises';
import { createRequire } from 'node:module';
import path from 'node:path';
import { pathToFileURL } from 'node:url';
import { parseArgs } from 'node:util';
const repo = path.resolve(import.meta.dirname, '../../..');
const { chromium } = await import(path.join(repo, 'scripts/dev/node_modules/playwright/index.mjs'));
const { loadFixture, UA } = await import(path.join(repo, 'slam_refact/web-chunk/tests/sample-fixture.mjs'));
const { EXPERIMENTAL_ENGINE_CONFIGS } = await import(pathToFileURL(path.join(repo, 'slam_refact/web-chunk/samples/phone/product-config.js')).href);
const { values } = parseArgs({ options: { root: { type: 'string' }, out: { type: 'string' }, 'max-frames': { type: 'string', default: '240' }, 'normal-timeout-s': { type: 'string', default: '240' } } });
if (!values.root || !values.out) { console.error('usage: run-product-page-check.mjs --root <staged web root> --out <new dir>'); process.exit(2); }
const root = path.resolve(repo, values.root), out = path.resolve(repo, values.out);
await fs.mkdir(out);
const sha256 = async (file) => crypto.createHash('sha256').update(await fs.readFile(file)).digest('hex');
const deploy = JSON.parse(await fs.readFile(path.join(root, 'slam-refact-xr/deploy.json'), 'utf8'));
const HOST = 'dev.serdic.com';
const require = createRequire(import.meta.url);
const { createServer } = require(path.join(repo, 'web/server.js'));
const server = createServer({ webRoot: root });
await new Promise((resolve, reject) => { server.once('error', reject); server.listen(0, '127.0.0.1', resolve); });
const origin = `https://${HOST}:${server.address().port}`;
const data = await loadFixture('flip6-s2', Number(values['max-frames']));
const inputScript = await fs.readFile(path.join(repo, 'slam_refact/web-chunk/tests/sample-input.js'), 'utf8');
const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
  args: ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', `--host-resolver-rules=MAP ${HOST} 127.0.0.1`] });
const result = {
  inputKind: 'SYNTHETIC/RECORDED desktop input (flip6-s2 frames + synthetic DeviceMotion); not a phone, not a device result',
  root: path.relative(repo, root), frames: data.frames.length, browser: browser.version(),
  engineWasmSha256: await sha256(path.join(root, 'slam-refact-xr/engine/slam-refact-engine/slam_refact_engine.wasm')),
  bundleSha256: deploy.bundle_sha256 ?? null, experimentalConfigs: EXPERIMENTAL_ENGINE_CONFIGS, modes: {}, checks: [], pass: false,
};
const check = (mode, name, ok, detail) => result.checks.push({ mode, name, ok: !!ok, detail: detail ?? null });
try {
  for (const [mode, query] of [['default', ''], ['K1', '?keepmap=1'], ['K1F', '?keepmap=fast']]) {
    const context = await browser.newContext({ viewport: { width: data.width, height: data.height }, permissions: ['camera'], isMobile: true, hasTouch: true, deviceScaleFactor: 1, userAgent: UA[data.ua] });
    const page = await context.newPage();
    const rec = { query, pageErrors: [], consoleErrors: [], blocked: [], reachedNormal: false };
    await page.addInitScript(inputScript);
    await page.route('**/*', async (route) => {
      const u = route.request().url();
      if (u.startsWith(origin + '/input/')) {
        const p = new URL(u).pathname;
        if (p === '/input/fixture.json') return route.fulfill({ status: 200, contentType: 'application/json', body: data.fixtureBytes });
        const m = /^\/input\/gray\/(\d+)$/.exec(p);
        return m ? route.fulfill({ status: 200, contentType: 'application/octet-stream', body: await data.gray(Number(m[1])) }) : route.fulfill({ status: 404, body: 'no' });
      }
      if (!u.startsWith(origin + '/') && !u.startsWith('data:') && !u.startsWith('blob:')) { rec.blocked.push(u); return route.abort('blockedbyclient'); }
      return route.continue();
    });
    page.on('pageerror', (e) => rec.pageErrors.push(e.message));
    page.on('console', (m) => { if (m.type() === 'error' && rec.consoleErrors.length < 50) rec.consoleErrors.push(m.text().slice(0, 300)); });
    try {
      await page.goto(`${origin}/slam-refact-xr.html${query}`);
      await page.waitForFunction(() => !document.getElementById('start').disabled, null, { timeout: 60000 });
      await page.touchscreen.tap(...await page.evaluate(() => { const r = document.getElementById('start').getBoundingClientRect(); return [r.x + r.width / 2, r.y + r.height / 2]; }));
      await page.waitForFunction(() => window.__phoneApp.getState().phase === 'running', null, { timeout: 90000 });
      await page.waitForFunction(() => window.__phoneApp.getState().status === 'NORMAL', null, { timeout: Number(values['normal-timeout-s']) * 1000, polling: 250 });
      rec.reachedNormal = true;
    } catch (e) { rec.failure = String(e && e.message || e).split('\n')[0]; }
    rec.final = await page.evaluate(() => {
      const app = window.__phoneApp, chunk = window.__slamRefactChunk;
      const st = app && app.getState();
      return { state: st && { phase: st.phase, status: st.status, reason: st.reason, worldPoints: st.worldPoints, errors: st.errors, mapLost: st.mapLost },
        engineInfo: app && app.getEngineInfo && app.getEngineInfo(),
        sentEngineConfig: chunk && chunk.session ? { engineConfig: chunk.session.engineConfig ?? null, glueConfigAccuracy: chunk.session.glueConfig ? chunk.session.glueConfig.accuracy ?? null : null } : null,
        effectiveAccuracy: chunk && chunk.session && chunk.session.configurationReadback && chunk.session.configurationReadback.effectiveConfig ? chunk.session.configurationReadback.effectiveConfig.accuracy ?? null : null,
        warnings: chunk ? chunk.warnings : null };
    }).catch((e) => ({ unavailable: String(e && e.message || e) }));
    result.modes[mode] = rec;
    await context.close();

    const info = rec.final.engineInfo || {}, rb = info.engineReadback || {}, eff = rb.available ? rec.final.effectiveAccuracy : null;
    check(mode, 'reaches NORMAL', rec.reachedNormal, rec.failure ?? rec.final.state?.status);
    check(mode, '0 page errors', rec.pageErrors.length === 0 && (rec.final.state?.errors || []).length === 0, [...rec.pageErrors, ...(rec.final.state?.errors || [])].join(' | ') || null);
    check(mode, `engineMode '${mode}'`, info.engineMode === mode, info.engineMode);
    if (mode === 'default') {
      check(mode, 'no engineConfig sent', rec.final.sentEngineConfig && rec.final.sentEngineConfig.engineConfig === null && rec.final.sentEngineConfig.glueConfigAccuracy === null, JSON.stringify(rec.final.sentEngineConfig));
      check(mode, 'read-back accuracy off (enabled false, every k1* false)', rb.available === true && eff && eff.enabled === false && Object.entries(eff).filter(([k]) => /^k1/.test(k)).every(([, v]) => v === false), JSON.stringify(eff));
    } else {
      const sent = Object.values(EXPERIMENTAL_ENGINE_CONFIGS).find((c) => c.name === mode).accuracy;
      check(mode, 'sent accuracy is the registered configuration', JSON.stringify(rec.final.sentEngineConfig?.glueConfigAccuracy) === JSON.stringify(sent), JSON.stringify(rec.final.sentEngineConfig));
      check(mode, 'read-back available', rb.available === true, rb.reason ?? null);
      for (const k of Object.keys(sent)) {
        check(mode, `flag ${k} reads back true, not ignored`, eff && eff[k] === true && !(rb.ignoredKeys || []).includes(`accuracy.${k}`), `effective=${eff ? eff[k] : null} ignored=${(rb.ignoredKeys || []).includes(`accuracy.${k}`)}`);
      }
      if (!('k1FastExtend' in sent)) check(mode, 'k1FastExtend reads back false', eff && eff.k1FastExtend === false, `effective=${eff ? eff.k1FastExtend : null}`);
      check(mode, 'allProductFlagsActive', rb.allProductFlagsActive === true, JSON.stringify(rb.flags ?? null));
    }
  }
} finally { await browser.close(); await new Promise((r) => server.close(r)); }
result.pass = result.checks.every((c) => c.ok);
await fs.writeFile(path.join(out, 'result.json'), JSON.stringify(result, null, 1));
console.log(`root ${result.root}\nbundle ${result.bundleSha256}\nwasm   ${result.engineWasmSha256}`);
for (const c of result.checks) console.log(`${c.ok ? 'PASS' : 'FAIL'}  ${c.mode.padEnd(7)} ${c.name}${c.ok ? '' : `  [${c.detail}]`}`);
console.log(`result: ${result.pass ? 'PASS' : 'FAIL'} -> ${path.relative(repo, path.join(out, 'result.json'))}`);
process.exit(result.pass ? 0 : 1);
