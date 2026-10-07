import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import path from 'node:path';
import { createRequire } from 'node:module';
import { createHash } from 'node:crypto';

const root = process.cwd();
const { chromium } = createRequire(path.join(root, 'scripts/dev/package.json'))('playwright');
const origin = process.env.MOBILE_SOURCE_TEST_ORIGIN || 'http://127.0.0.1:8767';
const output = path.resolve(process.env.MOBILE_SOURCE_TEST_OUTPUT || path.join(root, 'build/refactor-evidence/mobile'));
const realProfile = process.env.MOBILE_SOURCE_REAL_PROFILE === '1';
if (realProfile && !origin.startsWith('https://')) throw new Error('Real profile verification requires strict HTTPS');
const publicDirectory = path.join(root, 'web/public');
const profilePath = realProfile ? `/public/mobile-slam-camera-contract-${process.pid}.json` : '/public/fixture.json';
let temporaryProfile = null;
let createdPublicDirectory = false;
function profileFixture(orientationType) {
    return { schema: 'mobile-slam-camera-profile-v1', pixelFrame: 'drawImage', orientationType,
        provenance: 'analytic fixture only; no phone calibration', width: 640,height: 480,fx: 400,fy: 410,cx: 220,cy: 330,
        modelType: 2,distortion: [0.02,0.01,0.003,0.004],r_ic: [1,0,0,0,0,-1,0,1,0],t_ic: [0.03,0.04,0.05] };
}
await fs.mkdir(output, { recursive: true });
const browser = await chromium.launch({ executablePath: process.env.BROWSER_EXECUTABLE || '/usr/bin/google-chrome', headless: true,
    args: ['--enable-unsafe-swiftshader', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream'] });
const report = { scope: 'Actual browser fake-camera / mocked API contracts; physical Android/iOS, calibration, field GT and thermal unverified', origin, browser: browser.version(), realProfileEnabled: realProfile, scenarios: [] };
try {
    const scenarios = [{ ios: false, permission: 'granted' },{ ios: true, permission: 'granted' },{ ios: true, permission: 'denied' },{ ios: true, permission: 'error' },{ ios: false, permission: 'granted', profile: true }];
    for (const scenario of scenarios.filter(row => !process.env.MOBILE_SOURCE_PROFILE_ONLY || row.profile)) {
        const context = await browser.newContext({ viewport: { width: 412, height: 915 }, userAgent: scenario.ios ? 'iPhone Mobile Safari fixture' : undefined });
        await context.addInitScript(({ ios, permission }) => {
            window.__permissionCalls = [];
            Object.defineProperty(window, 'DeviceMotionEvent', { configurable: true, value: class extends Event {
                constructor(type, values = {}) { super(type); Object.assign(this, values); Object.defineProperty(this, 'timeStamp', { value: performance.now() - 8 }); }
                static requestPermission() {
                    window.__permissionCalls.push({ activation: navigator.userActivation.isActive, atMs: performance.now() });
                    return permission === 'error' ? Promise.reject(new DOMException('fixture denied', 'NotAllowedError')) : Promise.resolve(permission);
                }
            } });
            class Sensor extends EventTarget {
                constructor(options, gyro = false) { super(); this.options = options; this.gyro = gyro; this.timestamp = null; this.x = 0; this.y = gyro ? 0 : 9.81; this.z = 0; }
                start() { this.timer = setInterval(() => { this.timestamp = performance.now() - (this.gyro ? 7 : 5); this.dispatchEvent(new Event('reading')); }, 16); }
                stop() { clearInterval(this.timer); }
            }
            Object.defineProperty(window, 'Accelerometer', { configurable: true, value: class extends Sensor {} });
            Object.defineProperty(window, 'Gyroscope', { configurable: true, value: class extends Sensor { constructor(options) { super(options, true); } } });
            window.__motionTimer = setInterval(() => window.dispatchEvent(new DeviceMotionEvent('devicemotion', {
                accelerationIncludingGravity: { x: 0, y: ios ? -9.81 : 9.81, z: 0 }, acceleration: { x: 0,y: 0,z: 0 },
                rotationRate: { alpha: 0,beta: 0,gamma: 0 }, interval: 16 })), 16);
        }, scenario);
        const page = await context.newPage(); const errors = [];
        let profileOrientation = 'portrait-primary';
        page.on('pageerror', error => errors.push(error.message));
        if (scenario.profile && !realProfile) await context.route(url => url.pathname === profilePath,
            route => route.fulfill({ contentType: 'application/json', body: JSON.stringify(profileFixture(profileOrientation)) }));
        const pageResponse = await page.goto(origin + '/audit-mobile.html' + (scenario.profile ? `?config=mobile_highend&rotate=ccw&crop=landscape_4_3&cameraProfile=${encodeURIComponent(profilePath)}` : ''));
        const security = await pageResponse.securityDetails();
        await page.locator('#seconds').fill('10');
        await page.locator('#open').click();
        await page.waitForFunction(() => window.__mobileAudit?.ready, null, { timeout: 20000 });
        await page.locator('#record').click();
        const frame = page.frames().find(frame => new URL(frame.url()).pathname === '/index.html');
        assert.ok(frame);
        profileOrientation = await frame.evaluate(() => screen.orientation.type);
        let profileHTTP = null;
        if (scenario.profile && realProfile) {
            createdPublicDirectory = !(await fs.stat(publicDirectory).catch(error => { if (error.code === 'ENOENT') return null; throw error; }));
            await fs.mkdir(publicDirectory, { recursive: true });
            assert.equal((await fs.lstat(publicDirectory)).isSymbolicLink(), false, 'Temporary public fixture must not write through a symlink');
            const fixture = profileFixture(profileOrientation);
            const bytes = JSON.stringify(fixture) + '\n';
            const filename = path.join(publicDirectory, path.basename(profilePath));
            const fixtureFile = await fs.open(filename, 'wx', 0o644);
            temporaryProfile = filename;
            try { await fixtureFile.writeFile(bytes); } finally { await fixtureFile.close(); }
            profileHTTP = await frame.evaluate(async pathname => {
                const get = await fetch(pathname);
                const bytes = await get.arrayBuffer();
                const head = await fetch(pathname, { method: 'HEAD' });
                return { pathname, getStatus: get.status, headStatus: head.status, headBytes: (await head.arrayBuffer()).byteLength,
                    sha256: Array.from(new Uint8Array(await crypto.subtle.digest('SHA-256', bytes)), value => value.toString(16).padStart(2, '0')).join(''),
                    json: JSON.parse(new TextDecoder().decode(bytes)), headers: Object.fromEntries(get.headers), secureContext: isSecureContext };
            }, profilePath);
            assert.equal(profileHTTP.getStatus, 200); assert.equal(profileHTTP.headStatus, 200); assert.equal(profileHTTP.headBytes, 0);
            assert.equal(profileHTTP.sha256, createHash('sha256').update(bytes).digest('hex'));
            assert.deepEqual(profileHTTP.json, fixture);
            assert.match(profileHTTP.headers['content-type'], /application\/json/); assert.match(profileHTTP.headers['cache-control'], /no-store/);
            assert.equal(profileHTTP.headers['cross-origin-opener-policy'], 'same-origin');
            assert.equal(profileHTTP.headers['cross-origin-embedder-policy'], 'credentialless');
            assert.equal(profileHTTP.secureContext, true);
            if (origin.startsWith('https:')) assert.ok(security?.protocol?.startsWith('TLS'), 'Actual HTTPS uses default strict browser certificate validation');
            report.profileHTTP = { ...profileHTTP, security, strictCertificateValidation: true,
                scope: 'Real temporary same-origin synthetic profile; no route interception and no physical phone calibration claim' };
            report.serverHealth = await frame.evaluate(async () => {
                const response = await fetch('/__health__');
                if (!response.ok) throw new Error(`Actual server health HTTP ${response.status}`);
                return response.json();
            });
            assert.equal(report.serverHealth.schema, 'mobile-slam-https-health-v1');
        }
        await frame.locator('#btn-start').click();
        try { await page.waitForFunction(() => (window.__mobileAudit?.summary()?.completed || 0) >= 3, null, { timeout: 8000 }); }
        catch (error) { report.failedScenario = { ...scenario, errors, frame: await frame.evaluate(() => ({ status: document.getElementById('status')?.textContent, diagnostics: window.__mobileSLAM?.diagnostics(), orientation: screen.orientation.type })) }; throw error; }
        const diagnostics = await frame.evaluate(() => ({ ...window.__mobileSLAM.diagnostics(), permissionCalls: window.__permissionCalls,
            status: document.getElementById('status').textContent, frameText: document.getElementById('frame-count').textContent,
            overflowX: document.documentElement.scrollWidth > innerWidth, statusBox: document.getElementById('status').getBoundingClientRect().toJSON(), viewportHeight: innerHeight }));
        assert.equal(diagnostics.permissionCalls.length, 1);
        assert.equal(diagnostics.permissionCalls[0].activation, true, 'native iOS adapter invoked in actual Start activation');
        assert.ok(diagnostics.permissionCalls[0].atMs < diagnostics.imu.permission.resolvedAtMs);
        assert.equal(diagnostics.imu.permission.state, scenario.permission);
        assert.equal(diagnostics.calibration.status, scenario.profile ? 'supplied_calibration_unverified' : 'heuristic_unverified');
        assert.ok(diagnostics.transport.framesCompleted >= 3);
        assert.ok(diagnostics.transport.framesSubmitted >= diagnostics.transport.framesCompleted);
        if (scenario.permission === 'granted') assert.ok(diagnostics.imu.emitted > 10);
        else { assert.equal(diagnostics.imu.emitted, 0); assert.match(diagnostics.status, /permission (denied|error)/); }
        if (scenario.permission === 'granted') {
            await frame.evaluate(() => window.dispatchEvent(new Event('blur')));
            const before = await frame.evaluate(() => window.__mobileSLAM.diagnostics().transport.clientEpoch);
            await frame.evaluate(() => window.dispatchEvent(new Event('focus')));
            const after = await frame.evaluate(() => window.__mobileSLAM.diagnostics().transport.clientEpoch);
            assert.ok(after > before, 'focus resume resets estimator session after input gap');
        }
        if (!scenario.ios && !scenario.profile) {
            await frame.waitForFunction(() => !!window.__mobileSLAM.app.vio.getLatestResult(), null, { timeout: 3000 });
            const duplicate = await frame.evaluate(() => {
                const vio = window.__mobileSLAM.app.vio;
                const result = vio.getLatestResult();
                if (!result) return false;
                const completed = vio.getMetrics().framesCompleted;
                vio._handleWorkerMessage({ type: 'result', success: true, requestId: result.requestId, clientEpoch: result.clientEpoch,
                    sequence: result.sequence, data: result });
                if (vio.getMetrics().framesCompleted !== completed) throw new Error('Duplicate reply changed frame count');
                return true;
            });
            // Focus reset can clear the cache before a new result; when a result
            // exists, a duplicated reply remains excluded from recorder counts.
            report.duplicateReplyInjected = duplicate;
        }
        assert.equal(diagnostics.overflowX, false);
        assert.ok(diagnostics.statusBox.bottom <= diagnostics.viewportHeight, 'permission and calibration status stays visible');
        const tag = `${scenario.ios ? 'ios' : 'android'}-${scenario.permission}${scenario.profile ? '-profile' : ''}`;
        await frame.locator('body').screenshot({ path: path.join(output, `${tag}.png`) });
        await page.locator('#stop').click();
        const observation = await page.evaluate(() => window.__mobileAudit.export());
        assert.equal(observation.summary.staleResultsIgnored, !scenario.ios && !scenario.profile && report.duplicateReplyInjected ? 1 : 0);
        assert.ok(observation.samples.results.every(row => row.association.includes('requestId')));
        if (scenario.permission === 'granted') {
            const rows = observation.samples.device_imu;
            assert.ok(rows.length > 10);
            assert.ok(rows.every(row => row.callbackArrivalMs - row.timestampS * 1000 >= 5), 'source readings retain delayed callback offset');
            assert.ok(observation.samples.transformed_vio_imu.some(row => Math.abs(row.values[2] - 9.81) < 1e-9));
        }
        if (scenario.profile) {
            const params = observation.samples.vio_config.find(row => row.method === 'configure').args[0];
            assert.deepEqual([params.width,params.height], [322,240]);
            assert.ok(Math.abs(params.fx - 410 * 322 / 480) < 1e-12);
            assert.ok(Math.abs(params.fy - 400 * 240 / 360) < 1e-12);
            assert.ok(Math.abs(params.cx - (330.5 * 322 / 480 - 0.5)) < 1e-12);
            assert.ok(Math.abs(params.cy - (279.5 * 240 / 360 - 0.5)) < 1e-12);
            assert.deepEqual(params.r_ic, [0,-1,0,0,0,-1,1,0,0]);
            assert.deepEqual(params.t_ic, [0.03,0.04,0.05]);
            assert.deepEqual([params.k2,params.k3,params.k4,params.k5], [0.02,0.01,-0.004,0.003]);
        }
        assert.deepEqual(errors, []);
        report.scenarios.push({ ...scenario, diagnostics, observationSummary: observation.summary, profileHTTP,
            assertions: 'permission activation, source clock, fixed axes, count, observer envelope, pause epoch, pageerror' });
        await fs.writeFile(path.join(output, `${tag}-observation.json`), JSON.stringify(observation, null, 2));
        await context.close();
    }
    const context = await browser.newContext({ viewport: { width: 412,height: 915 } });
    const page = await context.newPage();
    await page.goto(origin + '/test-tumvi.html?frames=5&speed=-1');
    await page.waitForFunction(() => window.__tumviReplay && !document.getElementById('btn-start').disabled, null, { timeout: 15000 });
    const rotations = await page.evaluate(async () => {
        const { Camera } = await import('/js/camera.js?v=11');
        const source = document.createElement('canvas'); source.width = 2; source.height = 3;
        const ctx = source.getContext('2d'); const rgba = ctx.createImageData(2,3);
        [10,20,30,40,50,60].forEach((value,i) => { rgba.data.set([value,value,value,255],i*4); }); ctx.putImageData(rgba,0,0);
        const out = {};
        for (const rotation of ['none','cw','ccw','half']) {
            const camera = new Camera(); camera.video = source; camera._nativeWidth = 2; camera._nativeHeight = 3;
            camera._rotateMode = rotation; camera._cropMode = 'none';
            camera.width = rotation === 'cw' || rotation === 'ccw' ? 3 : 2; camera.height = camera.width === 3 ? 2 : 3;
            camera.canvas = document.createElement('canvas'); camera.canvas.width = camera.width; camera.canvas.height = camera.height; camera.ctx = camera.canvas.getContext('2d');
            out[rotation] = Array.from(camera._captureGrayscaleCPU());
        }
        return out;
    });
    assert.deepEqual(rotations, { none: [10,20,30,40,50,60],cw: [50,30,10,60,40,20],ccw: [20,40,60,10,30,50],half: [60,50,40,30,20,10] });
    report.pixelRotationGolden = rotations;
    for (const viewport of [{ width: 412,height: 915 },{ width: 1280,height: 720 }]) {
        await page.setViewportSize(viewport);
        const layout = await page.evaluate(() => ({ overflowX: document.documentElement.scrollWidth > innerWidth, bottom: document.getElementById('status').getBoundingClientRect().bottom, height: innerHeight }));
        assert.equal(layout.overflowX, false); assert.ok(layout.bottom <= layout.height);
        await page.screenshot({ path: path.join(output, `replay-${viewport.width}.png`) });
    }
    await context.close();
} catch (error) { report.error = error.stack; process.exitCode = 1; }
finally {
    try { await browser.close(); }
    finally {
        if (temporaryProfile) { await fs.unlink(temporaryProfile); report.temporaryProfileRemoved = true; }
        if (createdPublicDirectory) await fs.rmdir(publicDirectory).catch(error => { if (!['ENOENT', 'ENOTEMPTY'].includes(error.code)) throw error; });
        await fs.writeFile(path.join(output, 'browser-contracts.json'), JSON.stringify(report, null, 2));
        console.log(JSON.stringify(report, null, 2));
    }
}
