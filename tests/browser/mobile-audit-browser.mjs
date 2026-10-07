import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';
import https from 'node:https';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '../..');
const { chromium } = createRequire(path.join(root, 'scripts/dev/package.json'))('playwright');
const origin = process.env.MOBILE_AUDIT_TEST_ORIGIN || 'https://127.0.0.1:8766';
const output = path.join(root, 'build/dev-mobile-audit');
const probe = (method, pathname, body = '') => new Promise((resolve, reject) => {
    const url = new URL(origin);
    const request = https.request({ hostname: url.hostname, port: url.port || 443, path: pathname, method, rejectUnauthorized: false }, response => { response.resume(); resolve(response.statusCode); });
    request.on('error', reject);
    request.end(body);
});
await fs.mkdir(output, { recursive: true });
const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true, args: ['--enable-unsafe-swiftshader', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] });
const evidence = { scope: 'Desktop fake camera and mocked sensors, not physical phone accuracy evidence', assertions: [] };
try {
    const context = await browser.newContext({ ignoreHTTPSErrors: true, acceptDownloads: true, viewport: { width: 412, height: 915 } });
    await context.addInitScript(() => {
        Object.defineProperty(navigator, 'userActivation', { configurable: true, value: { isActive: false, hasBeenActive: true } });
        Object.defineProperty(window, 'DeviceMotionEvent', { configurable: true, value: class extends Event {
            constructor(type, options) { super(type); Object.assign(this, options); }
            static requestPermission() { if (window.__mockPermissionMode === 'sync_error') throw new DOMException('mock sync inactive gesture', 'NotAllowedError'); if (window.__mockPermissionMode === 'error') return Promise.reject(new DOMException('mock inactive gesture', 'NotAllowedError')); return Promise.resolve(window.__mockPermissionMode === 'denied' ? 'denied' : 'granted'); }
        } });
        class MockSensor extends EventTarget {
            constructor(options, gyro = false) { super(); this.options = options; this.gyro = gyro; this.timestamp = null; this.x = 0; this.y = gyro ? 0 : 9.81; this.z = 0; }
            start() { this.timer = setInterval(() => { this.timestamp = performance.now() - (this.gyro ? 3 : 5); this.dispatchEvent(new Event('reading')); }, 16); }
            stop() { clearInterval(this.timer); }
        }
        Object.defineProperty(window, 'Accelerometer', { configurable: true, value: class extends MockSensor { constructor(options) { super(options, false); } } });
        Object.defineProperty(window, 'Gyroscope', { configurable: true, value: class extends MockSensor { constructor(options) { super(options, true); } } });
        window.__mockMotionTimer = setInterval(() => window.dispatchEvent(new DeviceMotionEvent('devicemotion', { accelerationIncludingGravity: { x: 0, y: 9.81, z: 0 }, acceleration: { x: 0, y: 0, z: 0 }, rotationRate: { alpha: 0, beta: 0, gamma: 0 }, interval: 16 })), 16);
    });
    const page = await context.newPage();
    const errors = [];
    const requests = [];
    page.on('pageerror', error => errors.push(error.message));
    page.on('request', request => requests.push({ method: request.method(), url: request.url() }));
    await page.goto(origin + '/audit-mobile.html');
    await page.locator('#seconds').fill('10');
    await page.locator('#open').click();
    await page.waitForFunction(() => window.__mobileAudit?.ready, null, { timeout: 20000 });
    await page.locator('#record').click();
    const frame = page.frames().find(frame => new URL(frame.url()).pathname === '/index.html');
    assert.ok(frame, 'Existing live index iframe loaded');
    await frame.locator('#btn-start').click();
    await page.waitForFunction(() => (window.__mobileAudit?.summary()?.completed || 0) >= 12, null, { timeout: 12000 });
    const permissionChecks = await frame.evaluate(async () => {
        const { IMU } = await import('/js/imu.js?v=11');
        window.__mockPermissionMode = 'denied';
        const denied = await new IMU().requestPermission();
        window.__mockPermissionMode = 'error';
        const error = await new IMU().requestPermission();
        window.__mockPermissionMode = 'sync_error';
        const synchronousError = await new IMU().requestPermission();
        return { denied, error, synchronousError };
    });
    assert.deepEqual(permissionChecks, { denied: false, error: false, synchronousError: false });
    await page.locator('#stop').click();
    const downloadPromise = page.waitForEvent('download');
    await page.locator('#export').click();
    const download = await downloadPromise;
    const target = path.join(output, 'mobile-audit-mock.json');
    await download.saveAs(target);
    const report = JSON.parse(await fs.readFile(target, 'utf8'));
    assert.equal(report.schema, 'mobile-slam-observation-v1');
    assert.equal(report.stopReason, 'manual');
    assert.equal(report.metadata.secureContext, true);
    assert.ok(Number.isFinite(report.metadata.clocks.parentPerformanceTimeOriginMs));
    assert.ok(Number.isFinite(report.metadata.clocks.appPerformanceTimeOriginMs));
    assert.match(report.metadata.clocks.recorderAtMs, /Parent/);
    assert.match(report.metadata.clocks.hookTimestamps, /iframe/);
    assert.ok(report.summary.completed >= 12);
    assert.equal(report.summary.missingOutputTimestamp, report.summary.completed);
    assert.ok(report.metadata.blockedRemoteLogWrites > 0);
    assert.equal(requests.filter(request => !['GET', 'HEAD'].includes(request.method)).length, 0);
    assert.equal(errors.length, 0, errors.join('; '));
    assert.ok(report.samples.raw_motion.length > 0);
    assert.ok(report.samples.raw_generic.length > 0);
    assert.ok(report.samples.device_imu[0].accelSensorTimestampMs !== null);
    assert.ok(report.samples.transformed_vio_imu.some(row => Math.abs(row.values[2] - 9.81) < 1e-9));
    assert.ok(report.samples.camera_config.some(row => row.method === 'initialize' && row.crop === 'none' && row.output[0] === 480 && row.output[1] === 640));
    assert.ok(report.samples.vio_config.some(row => row.method === 'configure'));
    assert.ok(report.samples.permission.some(row => row.phase === 'result' && row.granted));
    assert.ok(report.samples.permission_native.some(row => row.phase === 'request' && row.userActivationActive === false));
    assert.ok(report.samples.permission_native.some(row => row.permission === 'denied'));
    assert.ok(report.samples.permission_native.some(row => row.phase === 'error' && row.name === 'NotAllowedError'));
    assert.ok(report.samples.permission_native.some(row => row.phase === 'error' && row.synchronous === true));
    assert.equal(report.metadata.moduleUrls.wrapper, origin + '/js/vio-wrapper.js?v=11');
    assert.ok(report.totalSamples <= report.limits.maxTotal);
    for (const rows of Object.values(report.samples)) assert.ok(rows.length <= report.limits.maxSamplesPerChannel);
    for (const [file, fingerprint] of Object.entries(report.metadata.manifest)) {
        const filename = file.split('?')[0].slice(1);
        const bytes = await fs.readFile(path.join(root, 'web', filename));
        assert.equal(fingerprint.sha256, createHash('sha256').update(bytes).digest('hex'));
    }
    const count = report.totalSamples;
    await page.waitForTimeout(150);
    assert.equal((await page.evaluate(() => window.__mobileAudit.export())).totalSamples, count);
    evidence.assertions = ['unchanged existing app module identity/hashes', 'HTTPS secure context', 'manual existing camera Start', 'fake-camera capture/crop/config and mocked raw/device/transformed sensor observations', 'missing output timestamp explicitly retained', 'bounded local JSON download', 'beacon blocked/no browser writes', 'stop prevents subsequent rows', 'no pageerror'];
    evidence.summary = report.summary;
    evidence.observerOverhead = report.metadata.observerOverhead;
    evidence.browser = browser.version();
    evidence.report = target;
    evidence.requests = requests;
    await page.screenshot({ path: path.join(output, 'mobile-audit-mock.png'), fullPage: true });
    const postStatus = await probe('POST', '/log', 'do not persist this probe');
    assert.equal(postStatus, 405);
    evidence.serverWriteProbe = { method: 'POST', path: '/log', status: postStatus, scope: 'Audit server rejects writes; no product server execution' };
    evidence.privatePathProbes = [];
    for (const method of ['GET', 'HEAD']) for (const pathname of ['/.certs/key.pem', '/%2ecerts/key.pem', '/%2Ecerts%2fkey.pem', '/%2e%2e/secret.js']) {
        const status = await probe(method, pathname);
        assert.equal(status, 403);
        evidence.privatePathProbes.push({ method, path: pathname, status });
    }
} catch (error) {
    evidence.error = error.message;
    process.exitCode = 1;
} finally {
    await browser.close();
    await fs.writeFile(path.join(output, 'mobile-audit-browser-evidence.json'), JSON.stringify(evidence, null, 2) + '\n');
    console.log(JSON.stringify({ ...evidence, requests: undefined }, null, 2));
}
