import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const sources = Object.fromEntries(await Promise.all(['imu', 'camera', 'orientation', 'app', 'renderer', 'test-tumvi-app'].map(async name => [name, await fs.readFile(new URL(`../../web/js/${name}.js`, import.meta.url), 'utf8')])));
const quiet = { log() {}, warn() {}, error() {} };
function realm({ ios = false, now = 5000 } = {}) {
    const listeners = new Map();
    class Sensor {
        constructor() { this.handlers = {}; this.timestamp = null; }
        addEventListener(name, fn) { this.handlers[name] = fn; }
        start() {} stop() {}
        read(timestamp, values = [1, 2, 3]) { this.timestamp = timestamp; [this.x, this.y, this.z] = values; this.handlers.reading(); }
    }
    const context = vm.createContext({ console: quiet, Float64Array, Uint8Array, URL, URLSearchParams, setTimeout, clearTimeout, setInterval, clearInterval,
        navigator: { userAgent: ios ? 'iPhone' : 'Android', platform: '', maxTouchPoints: 0 },
        performance: { now: () => now, timeOrigin: 1700000000000 },
        window: { location: { search: '', href: 'https://dev.serdic.com:7002/' }, addEventListener: (name, fn) => listeners.set(name, fn), removeEventListener: name => listeners.delete(name) },
        screen: { orientation: { type: 'portrait-primary' } }, Accelerometer: Sensor, Gyroscope: Sensor, DeviceMotionEvent: class {},
        document: { addEventListener() {}, hidden: false }, requestAnimationFrame() {}, HTMLVideoElement: class {}, VIOWrapper: class {}, Camera: class {}, Renderer: class {}, OrientationHandler: class {},
    });
    vm.runInContext(sources.imu.replace(/export /g, '') + '\nglobalThis.IMU = IMU;', context);
    return { context, listeners, imu: new context.IMU() };
}
function appRealm() {
    const fixture = realm();
    vm.runInContext(sources.app.replace(/^import .*$/gm, '').replace(/const app = new App\(\);[\s\S]*$/, '') + '\nglobalThis.App = App;', fixture.context);
    const app = new fixture.context.App();
    return { ...fixture, app };
}

function cameraRealm() {
    const { context } = realm();
    vm.runInContext(sources.camera.replace(/export /g, '') + '\nglobalThis.cameraContract = { validateCameraProfile, transformCameraProfile };', context);
    return context.cameraContract;
}
const profile = { schema: 'mobile-slam-camera-profile-v1', pixelFrame: 'drawImage', orientationType: 'portrait-primary', provenance: 'fixture calibration',
    width: 640, height: 480, fx: 500, fy: 400, cx: 300, cy: 200, modelType: 2, distortion: [0.1, 0.01, 0.02, 0.03],
    r_ic: [1,0,0,0,1,0,0,0,1], t_ic: [0.1,0.2,0.3] };

test('K follows integer pixel rotation then crop and center-preserving rounded resize', () => {
    const { transformCameraProfile } = cameraRealm();
    const cw = transformCameraProfile(profile, { rotation: 'cw', cropX: 10, cropY: 20, cropWidth: 460, cropHeight: 600, width: 230, height: 300 });
    assert.equal(cw.fx, 200); assert.equal(cw.fy, 250);
    assert.equal(cw.cx, 134.25); assert.equal(cw.cy, 139.75);
    assert.deepEqual(Array.from(cw.r_ic), [0,1,0,-1,0,0,0,0,1]);
    assert.deepEqual(Array.from(cw.t_ic), [0.1,0.2,0.3]);
    assert.equal(cw.k4, 0.03); assert.equal(cw.k5, -0.02);
    const expected = { none: [300,200,500,400], ccw: [200,339,400,500], half: [339,279,500,400] };
    for (const [rotation, [cx,cy,fx,fy]] of Object.entries(expected)) {
        const swapped = rotation === 'ccw';
        const out = transformCameraProfile(profile, { rotation, cropX: 0, cropY: 0, cropWidth: swapped ? 480 : 640, cropHeight: swapped ? 640 : 480, width: swapped ? 480 : 640, height: swapped ? 640 : 480 });
        assert.deepEqual([out.cx,out.cy,out.fx,out.fy], [cx,cy,fx,fy]);
    }
    const rounded = transformCameraProfile(profile, { rotation: 'none', cropX: 0, cropY: 0, cropWidth: 640, cropHeight: 480, width: 212, height: 158 });
    assert.ok(Math.abs(rounded.fx - 500 * 212 / 640) < 1e-12);
    assert.ok(Math.abs(rounded.cy - ((200.5) * 158 / 480 - 0.5)) < 1e-12);
});

test('Measured profile rejects improper rotations, missing provenance and malformed distortion', () => {
    const { validateCameraProfile } = cameraRealm();
    assert.doesNotThrow(() => validateCameraProfile(profile));
    assert.throws(() => validateCameraProfile({ ...profile, r_ic: [-1,0,0,0,1,0,0,0,1] }), /rotation/);
    assert.throws(() => validateCameraProfile({ ...profile, provenance: '' }), /provenance/);
    assert.throws(() => validateCameraProfile({ ...profile, distortion: [0,0,NaN,0] }), /distortion/);
    assert.throws(() => validateCameraProfile({ ...profile, fx: 0 }), /intrinsics/);
});

test('Actual grayscale resize uses the same pixel-center convention as calibration', () => {
    const { context } = appRealm();
    const gray = new Uint8Array([0,1,2,10,11,12,20,21,22]);
    assert.deepEqual(Array.from(context.downsampleGray(gray,3,3,2,2)), [3,4,18,19]);
    assert.deepEqual(Array.from(context.downsampleGray(new Uint8Array([0,40,80,120]),2,2,1,1)), [60]);
});

test('Generic source time survives delayed callback and emits once per gyro', () => {
    const { imu } = realm(); imu.start(500);
    imu._accel.read(1000, [0, 9.81, 0]);
    assert.equal(imu.flush().count, 0, 'acceleration alone is not an IMU sample');
    imu._gyro.read(1000, [1, 2, 3]);
    imu._accel.read(1002, [0, 9.81, 0]); imu._gyro.read(1002, [1, 2, 3]);
    const { data, count } = imu.flush();
    assert.equal(count, 2, 'valid 500Hz readings are not dropped by a 4ms floor');
    assert.deepEqual(Array.from(data).filter((_, i) => i % 7 === 0), [1, 1.002]);
});

test('Generic pairing waits for both sources and rejects stale/nonfinite data', () => {
    const { imu } = realm(); imu.start(100);
    imu._gyro.read(1000); assert.equal(imu.flush().count, 0);
    imu._accel.read(1000); assert.equal(imu.flush().count, 1);
    imu._gyro.read(1100); assert.equal(imu.flush().count, 0, '100ms-old accel is not paired');
    imu._accel.read(1100, [null, 1, 2]); assert.equal(imu.flush().count, 0);
});

test('DeviceMotion normalizes epoch event time and preserves axis/sign/radian contract', () => {
    for (const ios of [false, true]) {
        const { imu, listeners, context } = realm({ ios });
        delete context.Accelerometer; delete context.Gyroscope; imu.start();
        listeners.get('devicemotion')({ timeStamp: 1700000001250, accelerationIncludingGravity: { x: 1, y: 2, z: 3 }, rotationRate: { beta: 90, gamma: 180, alpha: -90 } });
        const { data, count } = imu.flush(); assert.equal(count, 1);
        assert.equal(data[0], 1.25);
        assert.deepEqual(Array.from(data.slice(1, 4)), ios ? [-1, -2, -3] : [1, 2, 3]);
        assert.ok(Math.abs(data[4] - Math.PI / 2) < 1e-12);
        assert.ok(Math.abs(data[5] - Math.PI) < 1e-12);
        assert.ok(Math.abs(data[6] + Math.PI / 2) < 1e-12);
    }
});

test('IMU invalid readings and overflow are observable with retained ordered samples', () => {
    const { imu } = realm();
    imu._pushSample(0.1, NaN, 0, 9.81, 0, 0, 0);
    for (let i = 0; i < 600; i++) imu._pushSample(1 + i / 1000, 0, 0, 9.81, 0, 0, 0);
    const rows = imu.flush(); assert.equal(rows.count, 512);
    assert.equal(rows.data[0], 1.088);
    assert.equal(imu.getDiagnostics().drops.overflow, 88);
    assert.equal(imu.getDiagnostics().drops.nonfinite, 1);
});

test('Motion relative event clock remains independent of delayed arrival and null axes do not become zeros', () => {
    const { imu, listeners, context } = realm({ now: 9000 }); delete context.Accelerometer; delete context.Gyroscope; imu.start();
    const sample = { timeStamp: 1234, accelerationIncludingGravity: { x: 0, y: 9.81, z: 0 }, rotationRate: { beta: 0, gamma: 0, alpha: 0 } };
    listeners.get('devicemotion')(sample);
    listeners.get('devicemotion')({ ...sample, timeStamp: 1235, rotationRate: { beta: null, gamma: 0, alpha: 0 } });
    const data = imu.flush(); assert.equal(data.count, 1); assert.equal(data.data[0], 1.234);
    assert.equal(imu.getDiagnostics().lastSample.arrivalMs, 9000);
    assert.equal(imu.getDiagnostics().drops.nonfinite, 1);
});

test('Main adapter preserves fixed body axes for Android/iOS portrait/landscape and actually sends the transformed IMU', () => {
    for (const ios of [false,true]) for (const type of ['portrait-primary','landscape-primary','portrait-secondary','landscape-secondary']) {
        const { app, context } = appRealm(); context.navigator.userAgent = ios ? 'iPhone' : 'Android'; context.screen.orientation.type = type;
        app.imu = new context.IMU(); delete context.Accelerometer; delete context.Gyroscope;
        app.imu._tryDeviceMotion(); app.imu.running = true;
        const sign = ios ? -1 : 1;
        app.imu._motionHandler({ timeStamp: 1000, accelerationIncludingGravity: { x: sign, y: 2*sign, z: 3*sign }, rotationRate: { beta: 90, gamma: 180, alpha: -90 } });
        let sent; app.vio.sendIMU = (data, count) => { sent = { data, count }; return true; };
        assert.equal(app._flushAndSendIMU(), 1);
        assert.deepEqual(Array.from(sent.data.slice(1,4)), [1,-3,2], `${ios ? 'iOS' : 'Android'} ${type}`);
        assert.ok(Math.abs(sent.data[4] - Math.PI/2) < 1e-12);
        assert.ok(Math.abs(sent.data[5] - Math.PI/2) < 1e-12);
        assert.ok(Math.abs(sent.data[6] - Math.PI) < 1e-12);
    }
});

test('Existing camera-to-body orientation tables preserve explicit golden basis vectors', () => {
    const { context } = realm();
    vm.runInContext(sources.orientation.replace(/export /g, '') + '\nglobalThis.Orientation = OrientationHandler;', context);
    const expectedX = { 'portrait-primary': [1,0,0], 'landscape-primary': [0,0,1], 'portrait-secondary': [-1,0,0], 'landscape-secondary': [0,0,-1] };
    for (const [type, x] of Object.entries(expectedX)) {
        const r = context.Orientation.getRICForType(type);
        assert.deepEqual([r[0],r[3],r[6]], x);
        assert.deepEqual([r[2],r[5],r[8]], [0,1,0]);
    }
});

test('Discarding paused samples invalidates pair readiness and preserves explicit drop counts', () => {
    const { imu } = realm(); imu.start(60); imu._accel.read(1000); imu._gyro.read(1000);
    imu.discard('resume'); assert.equal(imu.flush().count, 0); assert.equal(imu.getDiagnostics().drops.discarded, 1);
    imu._gyro.read(1010); assert.equal(imu.flush().count, 0);
    imu._accel.read(1010); assert.equal(imu.flush().count, 1);
});

test('Renderer clears an empty map and hides pose frustum after reset', () => {
    const { context } = realm();
    vm.runInContext(sources.renderer.replace(/^import .*$/gm, '').replace(/export /g, '') + '\nglobalThis.RendererContract = Renderer;', context);
    const draws = []; const geometry = { getAttribute: () => ({}), setDrawRange: (...args) => draws.push(args) };
    const renderer = { pointsGeometry: geometry, trajectoryGeometry: geometry, frustumGroup: { visible: true, matrix: { identity() {} } }, _latestPoseMatrix: {}, _trajCount: 10 };
    context.RendererContract.prototype.updateMapPoints.call(renderer, null, 0);
    assert.deepEqual(draws.at(-1), [0,0]);
    context.RendererContract.prototype.clear.call(renderer);
    assert.equal(renderer.frustumGroup.visible, false); assert.equal(renderer._latestPoseMatrix, null);
});

test('Start invokes native iOS permission inside gesture before orientation/camera awaits', async () => {
    const { app, context } = appRealm();
    const order = [];
    context.DeviceMotionEvent.requestPermission = () => { order.push('permission'); return Promise.resolve('denied'); };
    app.startBtn = { disabled: false };
    app.orientation = { tryLockPortrait: () => { order.push('orientation'); return new Promise(() => {}); } };
    app.start();
    assert.equal(order[0], 'permission');
});

test('Live result consumes a pose only once and clears state on loss/engine epoch', () => {
    const { app } = appRealm();
    const events = [];
    app.renderer = { clear: () => events.push('clear'), updateCameraPose: () => events.push('pose'), updateMapPoints: () => events.push('map') };
    app.vio.getMapPoints = () => ({ points: null, count: 0 });
    const pose = new Float64Array([1,0,0,1,0,1,0,2,0,0,1,3,0,0,0,1]);
    const result = { clientEpoch: 1, engineEpoch: 1, sequence: 1, statusCode: 2, initialized: true, poseFresh: true, poseValid: true, poseTimestamp: 1, pose };
    app._consumeResult(result); app._consumeResult(result);
    assert.equal(events.filter(e => e === 'pose').length, 1);
    app._consumeResult({ ...result, sequence: 2, statusCode: 3, pose: null, poseFresh: false, poseValid: false });
    assert.equal(events.at(-1), 'clear');
    app._consumeResult({ ...result, sequence: 3, engineEpoch: 2 });
    assert.equal(events.filter(e => e === 'clear').length, 2);
});

function replayRealm(options = {}) {
    const { context } = realm(options);
    vm.runInContext(sources['test-tumvi-app'].replace(/^import .*$/gm, '').replace(/const app = new TUMVITestApp\(\);[\s\S]*$/, '') + '\nglobalThis.replayContract = { TUMVITestApp, sliceIMUCursor, replayDeadline };', context);
    return { context, ...context.replayContract };
}

test('Replay cursor delivers each future bracket once and preserves constant sample count', () => {
    const { sliceIMUCursor } = replayRealm();
    const timestamps = new Float64Array([1,1.02,1.04,1.06,1.08]);
    const data = new Float64Array(35); for (let i = 0; i < 5; i++) data[i * 7] = timestamps[i];
    const a = sliceIMUCursor(data, timestamps, 5, 0, 1.01, true);
    const b = sliceIMUCursor(data, timestamps, 5, a.nextCursor, 1.03, true);
    const c = sliceIMUCursor(data, timestamps, 5, b.nextCursor, 1.08, true);
    assert.deepEqual([a.count,b.count,c.count], [2,1,2]);
    assert.equal(b.data[0], 1.04);
    assert.equal(c.nextCursor, 5);
    const tooEarly = sliceIMUCursor(data, timestamps, 5, a.nextCursor, 1.015, true);
    assert.equal(tooEarly.count, 0); assert.equal(tooEarly.nextCursor, a.nextCursor);
});

test('Paced replay deadline excludes previous engine processing time', () => {
    const { replayDeadline } = replayRealm();
    assert.ok(Math.abs(replayDeadline(1000, 10, 10.05, 1) - 1050) < 1e-9);
    assert.ok(Math.abs(replayDeadline(1000, 10, 10.05, 2) - 1025) < 1e-9);
});

test('Replay exact bound has no extra frame after completion; reset cancels pending decoded image', async () => {
    const { TUMVITestApp } = replayRealm();
    const app = new TUMVITestApp(); const calls = [];
    app.ui = { pauseBtn: {}, stepBtn: {}, startBtn: {}, resetBtn: {} };
    app.setStatus = () => {}; app.log = () => {}; app.updateProgress = () => {}; app.updateDiagnostics = () => {}; app.updateRenderer = () => {}; app.displayImagePreview = () => {};
    app.imageList = [{ timestamp_s: 1 },{ timestamp_s: 1.05 }]; app.frameLimit = 1; app.playing = true;
    app.imuData = new Float64Array([0.99,0,0,9.81,0,0,0,1.01,0,0,9.81,0,0,0]); app.imuTimestamps = new Float64Array([0.99,1.01]); app.imuCount = 2;
    app.prefetcher = { get: async () => new Uint8Array(4), clear() {} };
    app.vio = { reset() {}, waitForFree: async () => {}, sendIMU() {}, sendFrame: (_gray, timestamp) => { calls.push(timestamp); return true; }, getLatestResult: () => ({ inputTimestamp: 1, poseTimestamp: null, poseFresh: false, poseValid: false, pose: null }) };
    await app.processNextFrame(); await app.processNextFrame(); assert.deepEqual(calls, [1]);
    app.currentFrame = 0; app.playing = true; let decode;
    app.prefetcher.get = () => new Promise(resolve => { decode = resolve; });
    const pending = app.processNextFrame(); await app.reset(); decode(new Uint8Array(4)); await pending;
    assert.deepEqual(calls, [1]);
});
