/** Real WebGL layout regression with injected poses; no sensor/SLAM accuracy claim. */
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';

const root = process.cwd();
const require = createRequire(path.join(root, 'scripts/dev/package.json'));
const { chromium } = require('playwright');
const baseline = process.argv.includes('--record-baseline');
const origin = process.env.MOBILE_RENDERER_TEST_ORIGIN || 'https://dev.serdic.com:7002';
const output = path.resolve(process.env.MOBILE_RENDERER_TEST_OUTPUT || 'build/trajectory-rendering');
const label = baseline ? 'before' : 'after';
await fs.mkdir(output, { recursive: true });
const fingerprints = {};
for (const filename of ['web/js/renderer.js', 'web/js/app.js', 'web/js/test-tumvi-app.js', 'web/index.html']) {
    fingerprints[filename] = createHash('sha256').update(await fs.readFile(filename)).digest('hex');
}
const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome',
    headless: true, args: ['--enable-unsafe-swiftshader', '--use-gl=angle', '--use-angle=swiftshader'] });
const report = { createdAt: new Date().toISOString(), origin, browser: browser.version(),
    playwright: require('playwright/package.json').version, fingerprints, scenarios: [], failures: [],
    scope: 'Desktop Chrome WebGL with synthetic poses; physical phone IMU, initialization and trajectory accuracy unverified.' };

async function snapshot(page, name, dpr) {
    await page.evaluate(async () => {
        for (let i = 0; i < 3; i++) await new Promise(requestAnimationFrame);
    });
    const state = await page.evaluate(() => {
        const renderer = (window.__mobileSLAM || window.__tumviReplay).app.renderer;
        const canvas = renderer.renderer.domElement;
        const gl = renderer.renderer.getContext();
        renderer.render();
        function greenPixels() {
            const pixels = new Uint8Array(gl.drawingBufferWidth * gl.drawingBufferHeight * 4);
            gl.readPixels(0, 0, gl.drawingBufferWidth, gl.drawingBufferHeight, gl.RGBA, gl.UNSIGNED_BYTE, pixels);
            let count = 0;
            for (let i = 0; i < pixels.length; i += 4) {
                if (pixels[i] < 35 && pixels[i + 1] > 200 && pixels[i + 2] > 85 && pixels[i + 2] < 190) count++;
            }
            return count;
        }
        const visibleGreen = greenPixels();
        renderer.trajectoryLine.visible = false;
        renderer.render();
        const hiddenGreen = greenPixels();
        renderer.trajectoryLine.visible = true;
        renderer.render();
        const logical = { x: 0, y: 0, set(x, y) { this.x = x; this.y = y; return this; } };
        renderer.renderer.getSize(logical);
        const rect = canvas.getBoundingClientRect();
        const container = canvas.parentElement.getBoundingClientRect();
        return { logical: [logical.x, logical.y], canvas: [canvas.width, canvas.height],
            buffer: [gl.drawingBufferWidth, gl.drawingBufferHeight], viewport: Array.from(gl.getParameter(gl.VIEWPORT)),
            css: { width: rect.width, height: rect.height, bottom: rect.bottom },
            parent: { width: container.width, height: container.height, bottom: container.bottom },
            inline: [canvas.style.width, canvas.style.height], dpr: renderer.renderer.getPixelRatio(),
            innerHeight, aspect: renderer.camera.aspect, trajectoryPoints: renderer.trajectoryGeometry.drawRange.count,
            visibleGreen, hiddenGreen, trajectoryPixels: visibleGreen - hiddenGreen };
    });
    report.scenarios.push({ name, requestedDpr: dpr, ...state });
    const check = (condition, message) => { if (!condition) report.failures.push(`${name}/DPR${dpr}: ${message}`); };
    check(state.buffer.every((value, index) => value === Math.floor(state.logical[index] * state.dpr)), 'drawing buffer does not match logical size × DPR');
    check(state.viewport[2] === state.buffer[0] && state.viewport[3] === state.buffer[1], 'GL viewport does not match drawing buffer');
    check(Math.abs(state.logical[0] - state.css.width) <= 1 && Math.abs(state.logical[1] - state.css.height) <= 1, 'renderer logical size does not match CSS');
    check(Math.abs(state.css.width - state.parent.width) <= 1 && Math.abs(state.css.height - state.parent.height) <= 1, 'canvas does not fill current parent');
    check(state.css.bottom <= state.innerHeight + 1 && state.css.height > 0, 'canvas overflows or is hidden below viewport');
    check(state.inline.every(value => value === ''), 'renderer fixed the canvas inline CSS size');
    check(Math.abs(state.aspect - state.logical[0] / state.logical[1]) < 1e-8, 'camera aspect does not match renderer');
    check(state.trajectoryPoints === 16 && state.trajectoryPixels > 10, 'injected trajectory does not produce visible green pixels');
    await page.screenshot({ path: path.join(output, `${label}-dpr${dpr}-${name}.png`) });
}

async function injectTrajectory(page) {
    await page.evaluate(() => {
        const renderer = (window.__mobileSLAM || window.__tumviReplay).app.renderer;
        renderer.followCamera = false;
        for (let i = 0; i < 16; i++) {
            const a = i * Math.PI * 1.5 / 15;
            renderer.updateCameraPose([1,0,0,Math.cos(a), 0,1,0,Math.sin(a), 0,0,1,0.5, 0,0,0,1]);
        }
    });
}

try {
    for (const dpr of [1, 3]) {
        const context = await browser.newContext({ viewport: { width: 412, height: 915 }, deviceScaleFactor: dpr,
            isMobile: true, hasTouch: true });
        const page = await context.newPage();
        const errors = [];
        page.on('pageerror', error => errors.push(error.message));
        const response = await page.goto(origin + '/');
        if (origin.startsWith('https:')) assert.ok((await response.securityDetails())?.protocol?.startsWith('TLS'));
        await page.waitForFunction(() => window.__mobileSLAM?.app?.renderer, null, { timeout: 30000 });
        await injectTrajectory(page);
        await snapshot(page, 'initial', dpr);
        if (!baseline) {
            const guards = await page.evaluate(() => {
                const renderer = window.__mobileSLAM.app.renderer;
                const canvas = renderer.renderer.domElement;
                const before = [canvas.width, canvas.height, renderer.camera.aspect];
                let writes = 0;
                const original = renderer.renderer.setSize;
                renderer.renderer.setSize = function (...args) { writes++; return original.apply(this, args); };
                renderer.resize(canvas.clientWidth, canvas.clientHeight);
                renderer.resize(0, canvas.clientHeight);
                renderer.resize(canvas.clientWidth, 0);
                renderer.resize(NaN, Infinity);
                renderer.renderer.setSize = original;
                return { before, after: [canvas.width, canvas.height, renderer.camera.aspect], writes };
            });
            assert.deepEqual(guards.after, guards.before, 'Hidden/invalid size must retain usable renderer state');
            assert.equal(guards.writes, 0, 'Repeated or invalid resize must not reset the canvas');
        }
        await page.setViewportSize({ width: 412, height: 800 });
        await snapshot(page, 'height-resize', dpr);
        await page.evaluate(() => { document.getElementById('status').textContent = 'Initializing — IMU active / imu_missing_past / heuristic_unverified. Synthetic layout regression with longer status text; no sensor or accuracy assertion.'; });
        await snapshot(page, 'status-wrap', dpr);
        await page.setViewportSize({ width: 915, height: 412 });
        await snapshot(page, 'landscape', dpr);
        if (errors.length) report.failures.push(...errors.map(error => `DPR${dpr}: ${error}`));
        await context.close();
        if (!baseline) {
            const replayContext = await browser.newContext({ viewport: { width: 1280, height: 800 }, deviceScaleFactor: dpr });
            const replay = await replayContext.newPage();
            replay.on('pageerror', error => report.failures.push(`TUM/DPR${dpr}: ${error.message}`));
            await replay.goto(origin + '/test-tumvi.html');
            await replay.waitForFunction(() => window.__tumviReplay?.app?.renderer, null, { timeout: 30000 });
            await injectTrajectory(replay);
            await snapshot(replay, 'tum-factory', dpr);
            await replay.setViewportSize({ width: 1000, height: 650 });
            await snapshot(replay, 'tum-resize', dpr);
            await replay.evaluate(() => { document.getElementById('status').textContent = 'Layout fixture '.repeat(35); });
            await snapshot(replay, 'tum-status-wrap', dpr);
            await replayContext.close();
        }
    }
} finally {
    await browser.close();
    await fs.writeFile(path.join(output, `${label}.json`), JSON.stringify(report, null, 2) + '\n');
}
console.log(JSON.stringify({ report: path.join(output, `${label}.json`), scenarios: report.scenarios.length,
    failures: report.failures }, null, 2));
if (!baseline) assert.equal(report.failures.length, 0, 'Renderer resize regressions; see evidence JSON');
