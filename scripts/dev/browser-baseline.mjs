/** Read-only browser/WASM audit. Never starts web/server.js or rebuilds artifacts. */
import http from 'node:http';
import path from 'node:path';
import fs from 'node:fs/promises';
import { createHash } from 'node:crypto';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';
import { chromium } from 'playwright';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const output = path.resolve(process.env.BROWSER_AUDIT_OUTPUT || path.join(root, 'build/audit-browser'));
const frames = Number(process.env.BROWSER_AUDIT_FRAMES || 60);
if (!Number.isInteger(frames) || frames < 1 || frames > 1000) throw new Error('BROWSER_AUDIT_FRAMES must be 1..1000');
const replayFrames = Number(process.env.BROWSER_AUDIT_REPLAY_FRAMES || 0);
if (!Number.isInteger(replayFrames) || replayFrames < 0 || replayFrames > 5000) throw new Error('BROWSER_AUDIT_REPLAY_FRAMES must be 0..5000');
const replayTimeout = Number(process.env.BROWSER_AUDIT_REPLAY_TIMEOUT_MS || 60000);
if (!Number.isInteger(replayTimeout) || replayTimeout < 1000 || replayTimeout > 600000) throw new Error('BROWSER_AUDIT_REPLAY_TIMEOUT_MS must be 1000..600000');
const wasmVariant = process.env.BROWSER_AUDIT_WASM || 'active';
if (!['active', 'fresh'].includes(wasmVariant)) throw new Error('BROWSER_AUDIT_WASM must be active or fresh');
const wasmPath = wasmVariant === 'fresh' ? '/__audit-wasm__/vio_engine.js' : '/vio_engine.js';
const artifacts = ['web/vio_engine.js', 'web/js/vio_engine_worker.js', 'wasm/build/vio_engine.js', 'wasm/build/vio_engine_worker.js', 'wasm/dist/vio_engine.js', 'web/js/app.js', 'web/js/vio-wrapper.js', 'web/js/vio-worker.js', 'web/js/test-tumvi-app.js', 'src/vio_engine.cpp', 'src/backend/estimator.cpp', 'wasm/vio_bindings.cpp', 'wasm/CMakeLists.txt'];
if (wasmVariant === 'fresh') artifacts.push('build/audit-wasm/vio_engine.js');
const fingerprints = {};
for (const file of artifacts) {
    const data = await fs.readFile(path.join(root, file));
    fingerprints[file] = { bytes: data.length, sha256: createHash('sha256').update(data).digest('hex') };
}
const requests = [];
const mime = { '.html': 'text/html', '.js': 'application/javascript', '.mjs': 'application/javascript', '.csv': 'text/csv', '.png': 'image/png', '.wasm': 'application/wasm' };
const server = http.createServer(async (req, res) => {
    const url = new URL(req.url, 'http://localhost');
    res.setHeader('Cross-Origin-Opener-Policy', 'same-origin');
    res.setHeader('Cross-Origin-Embedder-Policy', 'credentialless');
    res.setHeader('Cache-Control', 'no-store');
    if (req.method === 'POST' && url.pathname === '/log') { req.resume(); res.end('audit: remote logs discarded'); return; }
    if (url.pathname === '/__audit__') { res.setHeader('Content-Type', 'text/html'); res.end('<!doctype html><title>WASM baseline</title>'); return; }
    if (url.pathname === '/favicon.ico') { res.writeHead(204).end(); return; }
    let decoded;
    try { decoded = decodeURIComponent(url.pathname); } catch { res.writeHead(400).end(); return; }
    const base = path.join(root, decoded.startsWith('/datasets/') ? 'assets/datasets' : decoded.startsWith('/__audit-wasm__/') ? 'build/audit-wasm' : 'web');
    const relative = decoded.startsWith('/datasets/') ? decoded.slice('/datasets/'.length) : decoded.startsWith('/__audit-wasm__/') ? decoded.slice('/__audit-wasm__/'.length) : decoded === '/' ? 'index.html' : decoded.slice(1);
    const file = path.resolve(base, relative);
    if (!file.startsWith(base + path.sep) || decoded.includes('\0')) { res.writeHead(403).end(); return; }
    try {
        const content = await fs.readFile(file);
        res.setHeader('Content-Type', mime[path.extname(file)] || 'application/octet-stream');
        requests.push({ path: url.pathname, status: 200 });
        res.end(content);
    } catch (error) {
        const status = error.code === 'ENOENT' ? 404 : 500;
        requests.push({ path: url.pathname, status });
        res.writeHead(status).end('Not Found');
    }
});
await new Promise(resolve => server.listen(0, '127.0.0.1', resolve));
const baseUrl = `http://127.0.0.1:${server.address().port}`;
let browser;
const report = { createdAt: new Date().toISOString(), framesRequested: frames, wasmVariant, node: process.version, playwright: createRequire(import.meta.url)('playwright/package.json').version, fingerprints, requests, limits: ['Desktop headless browser; no real phone sensors, thermal endurance, or independent trajectory accuracy evidence.', 'Synthetic replay does not establish VIO initialization, convergence, tracking accuracy, or production latency.'] };
try {
    browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args: ['--enable-unsafe-swiftshader'] });
    report.browser = browser.version();
    const context = await browser.newContext({ ignoreHTTPSErrors: true });
    const page = await context.newPage();
    const consoleLog = [];
    page.on('console', message => consoleLog.push({ type: message.type(), text: message.text() }));
    page.on('pageerror', error => consoleLog.push({ type: 'pageerror', text: error.message }));
    await page.goto(baseUrl + '/__audit__');
    report.synthetic = await page.evaluate(async ({ frames, wasmPath }) => {
        const { VIOWrapper } = await import('/js/vio-wrapper.js');
        const vio = new VIOWrapper();
        const bounded = (promise, ms = 15000) => Promise.race([promise, new Promise((_, reject) => setTimeout(() => reject(new Error('audit operation timed out')), ms))]);
        const result = { crossOriginIsolated, secureContext: isSecureContext, frameResults: [], parameterResults: {} };
        const initStart = performance.now();
        await bounded(vio.load(wasmPath));
        result.loadMs = performance.now() - initStart;
        result.configured = await bounded(vio.configure({ width: 320, height: 240, fx: 260, fy: 260, cx: 160, cy: 120, modelType: 2, acc_n: 0.08, acc_w: 0.004, gyr_n: 0.004, gyr_w: 0.0002, g_norm: 9.81 }));
        for (const [name, args] of [['setMobileParams', [0.04, 8, 150]], ['setFThreshold', [1]], ['setTrackingParams', [21, 3, 20, 0]], ['setPnPParams', [true, 3]]]) result.parameterResults[name] = await bounded(vio[name](...args));
        let tickCount = 0;
        const timer = setInterval(() => tickCount++, 5);
        for (let frame = 0; frame < frames; frame++) {
            const timestamp = 1 + frame * 0.05;
            const imu = new Float64Array(10 * 7);
            for (let i = 0; i < 10; i++) imu.set([timestamp - 0.045 + i * 0.005, 0, 0, 9.81, 0, 0, 0], i * 7);
            vio.sendIMU(imu, 10);
            const image = new Uint8Array(320 * 240);
            for (let y = 0; y < 240; y++) for (let x = 0; x < 320; x++) image[y * 320 + x] = (((Math.floor((x + frame * 2) / 20) + Math.floor(y / 20)) % 2) * 200 + (x + frame * 2) % 256) / 2;
            const t0 = performance.now();
            const accepted = vio.sendFrame(image, timestamp);
            const busyDrop = frame === 0 ? !vio.sendFrame(image, timestamp) : null;
            await bounded(vio.waitForFree(10000));
            const r = vio.getLatestResult();
            result.frameResults.push({ frame, accepted, busyDrop, roundTripMs: performance.now() - t0, initialized: r?.initialized, status: r?.statusCode, features: r?.featureCount, imuCount: r?.imuCount, hasPose: !!r?.pose, finitePose: r?.pose ? Array.from(r.pose).every(Number.isFinite) : null, timestampPresent: r ? 'timestamp' in r : false });
        }
        clearInterval(timer);
        result.mainThreadTicks = tickCount;
        vio.sendFrame(new Uint8Array(4), 1 + frames * 0.05);
        await bounded(vio.waitForFree(10000));
        result.invalidImageStatus = vio.getLatestResult()?.statusCode;
        vio.sendFrame(new Uint8Array(320 * 240), 4 + frames * 0.05);
        await bounded(vio.waitForFree(10000));
        result.frameGapStatus = vio.getLatestResult()?.statusCode;
        result.mapPointCount = vio.getMapPoints().count;
        vio.dispose();
        const classic = new VIOWrapper();
        try { await bounded(classic.load('/js/vio_engine_worker.js')); result.classicWorkerBuild = { loaded: true }; }
        catch (error) { result.classicWorkerBuild = { loaded: false, error: error.message }; }
        finally { classic.dispose(); }
        return result;
    }, { frames, wasmPath });
    const times = report.synthetic.frameResults.map(row => row.roundTripMs).sort((a, b) => a - b);
    const percentile = p => times[Math.ceil(times.length * p) - 1];
    report.synthetic.roundTripMs = { p50: percentile(0.5), p95: percentile(0.95), p99: percentile(0.99), max: times.at(-1) };
    report.synthetic.initializedFrames = report.synthetic.frameResults.filter(row => row.initialized).length;
    report.synthetic.poseFrames = report.synthetic.frameResults.filter(row => row.hasPose).length;
    report.synthetic.coverage = { load: true, configure: report.synthetic.configured, parameterMethods: Object.values(report.synthetic.parameterResults).every(Boolean), completeFrames: report.synthetic.frameResults.length === frames, initializedTracking: report.synthetic.initializedFrames > 0, finitePoseOutputs: report.synthetic.poseFrames > 0 && report.synthetic.frameResults.filter(row => row.hasPose).every(row => row.finitePose) };
    report.synthetic.console = consoleLog.splice(0);
    report.pages = [];
    for (const pathname of ['/', '/test-tumvi.html']) {
        await page.goto(baseUrl + pathname + (wasmVariant === 'fresh' ? '?wasm=__audit-wasm__/vio_engine.js' : ''));
        let ready = false;
        try { await page.waitForFunction(() => /WASM loaded/.test(document.getElementById('status')?.textContent) && !document.getElementById('btn-start')?.disabled, null, { timeout: 15000 }); ready = true; } catch {}
        const record = { pathname, ready, status: await page.locator('#status').textContent() };
        if (pathname === '/test-tumvi.html' && ready) {
            await page.evaluate(async () => {
                const { VIOWrapper } = await import('/js/vio-wrapper.js');
                const send = VIOWrapper.prototype.sendFrame;
                const receive = VIOWrapper.prototype._handleWorkerMessage;
                window.__auditFrames = [];
                window.__auditSettings = {};
                for (const name of ['configure', 'setMobileParams', 'setFThreshold', 'setTrackingParams', 'setPnPParams']) {
                    const method = VIOWrapper.prototype[name];
                    VIOWrapper.prototype[name] = function (...args) {
                        window.__auditSettings[name] = { args };
                        return method.apply(this, args).then(success => { window.__auditSettings[name].success = success; return success; });
                    };
                }
                VIOWrapper.prototype.sendFrame = function (image, timestamp) {
                    const accepted = send.call(this, image, timestamp);
                    if (accepted) this.__auditPending = { timestamp, sentAt: performance.now() };
                    return accepted;
                };
                VIOWrapper.prototype._handleWorkerMessage = function (msg) {
                    if (msg.type === 'result' && this.__auditPending) {
                        const data = msg.data || {};
                        const receivedAt = performance.now();
                        window.__auditFrames.push({ timestamp: this.__auditPending.timestamp, sentAtMs: this.__auditPending.sentAt, receivedAtMs: receivedAt, roundTripMs: receivedAt - this.__auditPending.sentAt, status: data.statusCode, initialized: data.initialized, features: data.featureCount, imuCount: data.imuCount, pose: data.pose ? Array.from(data.pose) : null });
                        this.__auditPending = null;
                    }
                    return receive.call(this, msg);
                };
            });
            if (replayFrames > 0) await page.locator('#speed-select').selectOption('-1');
            await page.locator('#btn-start').click();
            try { await page.waitForFunction(() => /Error|failed|FAILED|Step mode|Processing/.test(document.getElementById('status').textContent), null, { timeout: 15000 }); } catch {}
            record.afterStart = await page.locator('#status').textContent();
            record.replaySettings = await page.evaluate(() => window.__auditSettings);
            if (replayFrames > 0 && /Processing/.test(record.afterStart)) {
                try { await page.waitForFunction(limit => window.__auditFrames.length >= limit || /Error|Complete/.test(document.getElementById('status').textContent), replayFrames, { timeout: replayTimeout }); }
                catch { record.replayTimedOut = true; }
                await page.locator('#btn-pause').click();
                record.replayFrames = await page.evaluate(() => window.__auditFrames);
                const rows = record.replayFrames;
                const durations = rows.map(row => row.roundTripMs).sort((a, b) => a - b);
                const percentile = p => durations[Math.ceil(durations.length * p) - 1] ?? null;
                const statusCounts = {};
                for (const row of rows) statusCounts[row.status] = (statusCounts[row.status] || 0) + 1;
                record.replaySummary = { count: rows.length, initializedFrames: rows.filter(row => row.initialized).length, poseFrames: rows.filter(row => row.pose).length, firstPoseFrame: rows.findIndex(row => row.pose), nonfinitePoses: rows.filter(row => row.pose?.some(value => !Number.isFinite(value))).length, statusCounts, roundTripMs: { p50: percentile(0.5), p95: percentile(0.95), p99: percentile(0.99), max: durations.at(-1) ?? null }, datasetDurationS: rows.length > 1 ? rows.at(-1).timestamp - rows[0].timestamp : 0, wallDurationS: rows.length ? (rows.at(-1).receivedAtMs - rows[0].sentAtMs) / 1000 : 0, limits: 'Submitted-frame roundtrip only; excludes image decode, camera capture latency, playback wait, and physical sensor clocks. Max-speed serial replay is not live pacing.' };
            }
        }
        record.console = consoleLog.splice(0);
        report.pages.push(record);
    }
} catch (error) {
    report.error = { message: error.message, stack: error.stack };
    process.exitCode = 1;
} finally {
    if (browser) await browser.close();
    await new Promise(resolve => server.close(resolve));
    await fs.mkdir(output, { recursive: true });
    await fs.writeFile(path.join(output, 'browser-baseline.json'), JSON.stringify(report, null, 2) + '\n');
    console.log(JSON.stringify({ output: path.join(output, 'browser-baseline.json'), browser: report.browser, synthetic: report.synthetic && { ...report.synthetic, frameResults: undefined, console: undefined }, pages: report.pages?.map(({ console, replayFrames, ...page }) => page), error: report.error?.message }, null, 2));
}
