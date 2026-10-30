import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';

const { chromium } = createRequire(path.resolve('scripts/dev/package.json'))('playwright');
const origin = process.env.MOBILE_CLOCK_ORIGIN || 'https://dev.serdic.com:7002';
const output = path.resolve(process.env.MOBILE_CLOCK_OUTPUT || 'build/mobile-clock');
await fs.mkdir(output, { recursive: true });
const report = { scope: 'Desktop Chrome, fake camera, mocked sensor clocks, real Worker/WASM; physical phone and tracking accuracy unverified', scenarios: [] };
const browser = await chromium.launch({ executablePath: '/usr/bin/google-chrome', headless: true,
    args: ['--enable-unsafe-swiftshader', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] });
try {
    report.browser = browser.version();
    report.sources = {};
    for (const file of ['web/js/app.js', 'web/js/imu.js', 'web/js/vio-worker.js', 'web/vio_engine.js']) {
        report.sources[file] = createHash('sha256').update(await fs.readFile(file)).digest('hex');
    }
    for (const offsetMs of [0, 3_600_000]) {
        const context = await browser.newContext({ viewport: { width: 412, height: 915 }, deviceScaleFactor: 3, isMobile: true });
        await context.addInitScript(offset => {
            const query = navigator.permissions.query.bind(navigator.permissions);
            navigator.permissions.query = descriptor => ['accelerometer', 'gyroscope'].includes(descriptor.name)
                ? Promise.resolve({ state: 'granted' }) : query(descriptor);
            class Sensor extends EventTarget {
                constructor(_options, gyro = false) { super(); this.x = 0; this.y = gyro ? 0 : 9.81; this.z = 0; this.timestamp = null; }
                start() { this.timer = setInterval(() => { this.timestamp = performance.now() - 5 + offset; this.dispatchEvent(new Event('reading')); }, 16); }
                stop() { clearInterval(this.timer); }
            }
            Object.defineProperty(window, 'Accelerometer', { configurable: true, value: class extends Sensor {} });
            Object.defineProperty(window, 'Gyroscope', { configurable: true, value: class extends Sensor { constructor(options) { super(options, true); } } });
            window.__clockMotionTimer = setInterval(() => {
                const event = new Event('devicemotion');
                Object.defineProperties(event, {
                    timeStamp: { value: performance.now() - 5 },
                    accelerationIncludingGravity: { value: { x: 0, y: 9.81, z: 0 } },
                    acceleration: { value: { x: 0, y: 0, z: 0 } },
                    rotationRate: { value: { beta: 0, gamma: 0, alpha: 0 } },
                });
                window.dispatchEvent(event);
            }, 16);
        }, offsetMs);
        const page = await context.newPage();
        const errors = [];
        page.on('pageerror', error => errors.push(error.message));
        await page.goto(origin);
        await page.locator('#btn-start').click();
        await page.waitForFunction(() => window.__mobileSLAM?.app.vio.getMetrics().framesCompleted >= 8, null, { timeout: 15000 });
        const sample = () => {
            const app = window.__mobileSLAM.app, result = app.vio.getLatestResult();
            return { nowS: performance.now() / 1000, status: document.querySelector('#status').textContent,
                sensor: app.imu.getSensorType(), imu: app.imu.getDiagnostics(), transport: app.vio.getMetrics(),
                result: { reason: result?.reason, epoch: result?.engineEpoch, imuCount: result?.imuCount,
                    inputTimestamp: result?.inputTimestamp, imuEndpointTimestamp: result?.imuEndpointTimestamp,
                    poseFresh: result?.poseFresh, featureCount: result?.featureCount } };
        };
        const first = await page.evaluate(sample);
        await page.waitForFunction(count => window.__mobileSLAM.app.vio.getMetrics().framesCompleted >= count + 5, first.transport.framesCompleted);
        const second = await page.evaluate(sample);
        report.scenarios.push({ offsetMs, first, second, errors });
        await page.evaluate(() => { clearInterval(window.__clockMotionTimer); window.__mobileSLAM.app.stop(); });
        await context.close();
    }
    for (const scenario of report.scenarios) {
        assert.deepEqual(scenario.errors, []);
        assert.equal(scenario.second.sensor, scenario.offsetMs ? 'device_motion' : 'generic_sensor', 'Invalid future sensor clock must recover using DeviceMotion');
        assert.ok(scenario.first.result.imuEndpointTimestamp >= 0, 'Real engine must receive IMU at/before image time');
        assert.ok(scenario.second.result.imuEndpointTimestamp > scenario.first.result.imuEndpointTimestamp, 'Real engine IMU endpoint must advance');
        assert.notEqual(scenario.second.result.reason, 'imu_missing_past');
        assert.equal(scenario.second.result.epoch, scenario.first.result.epoch, 'Clock recovery must not repeatedly reset engine');
        assert.ok(scenario.second.imu.lastSample.timestampS <= scenario.second.nowS + 0.001);
    }
    report.passed = true;
} catch (error) {
    report.passed = false; report.failure = error.stack; process.exitCode = 1;
} finally {
    await fs.writeFile(path.join(output, 'report.json'), JSON.stringify(report, null, 2) + '\n');
    await browser.close();
}
console.log(JSON.stringify({ passed: report.passed, scenarios: report.scenarios.map(row => ({ offsetMs: row.offsetMs, sensor: row.second.sensor, reason: row.second.result.reason, endpoint: row.second.result.imuEndpointTimestamp })), output, failure: report.failure }));
