import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';
import { limitedPipeline } from './limited-fixture.mjs';
import { loadBrowserSource } from './browser-source-loader.mjs';

const sources = Object.fromEntries(await Promise.all(['imu', 'camera', 'orientation', 'app', 'renderer', 'vio-wrapper', 'test-tumvi-app'].map(async name => [name, await loadBrowserSource(name)])));
const quiet = { log() {}, warn() {}, error() {} };
function realm({ ios = false, now = 5000, search = '' } = {}) {
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
        window: { location: { search, href: 'https://dev.serdic.com:7002/' + search }, addEventListener: (name, fn) => listeners.set(name, fn), removeEventListener: name => listeners.delete(name) },
        screen: { orientation: { type: 'portrait-primary' } }, Accelerometer: Sensor, Gyroscope: Sensor, DeviceMotionEvent: class {},
        document: { addEventListener() {}, hidden: false }, requestAnimationFrame() {}, HTMLVideoElement: class {}, VIOWrapper: class {}, Camera: class {}, Renderer: class {}, OrientationHandler: class {},
    });
    vm.runInContext(sources.imu.replace(/export /g, '') + '\nglobalThis.IMU = IMU;', context);
    return { context, listeners, imu: new context.IMU() };
}
function appRealm(options = {}) {
    const fixture = realm(options);
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
        // W3C rotationRate alpha=x, beta=y, gamma=z; Chrome154 pump passes gyro x/y/z in that order.
        assert.ok(Math.abs(data[4] + Math.PI / 2) < 1e-12);
        assert.ok(Math.abs(data[5] - Math.PI / 2) < 1e-12);
        assert.ok(Math.abs(data[6] - Math.PI) < 1e-12);
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
        // Device [-pi/2,pi/2,pi] -> fixed body [x,-z,y].
        assert.ok(Math.abs(sent.data[4] + Math.PI/2) < 1e-12);
        assert.ok(Math.abs(sent.data[5] + Math.PI) < 1e-12);
        assert.ok(Math.abs(sent.data[6] - Math.PI/2) < 1e-12);
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
    renderer.hideInertialPrediction = () => {}; renderer.hideGroundPlane = () => {};
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
    const { app } = appRealm({ now: 1000 });
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

test('App never renders Lost or unknown status as a normal finite fresh pose', () => {
    for(const statusCode of [3,99]) {
        const {app}=appRealm({ now: 1000 }),events=[];
        app.statusEl={};
        app.renderer={clear:()=>events.push('clear'),updateCameraPose:()=>events.push('pose'),updateMapPoints:()=>events.push('map')};
        app.vio.getMapPoints=()=>({points:null,count:0});
        app._consumeResult({clientEpoch:1,engineEpoch:1,sequence:1,statusCode,initialized:true,inputTimestamp:1,
            poseFresh:true,poseValid:true,poseTimestamp:1,pose:new Float64Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])});
        assert.equal(events.includes('pose'),false);
        assert.equal(events.at(-1),'clear');
        assert.doesNotMatch(app.statusEl.textContent,/· Tracking(?:\s|\/|·)/);
        assert.match(app.statusEl.textContent,statusCode===3?/Lost/:/Unknown/);
    }
});

test('App renders current Limited once with a concise position-quality label and preserves loss/epoch clearing', () => {
    const {app}=appRealm({ now: 1000 }),events=[];app.statusEl={};
    app.renderer={clear:()=>events.push('clear'),updateCameraPose:()=>events.push('pose'),updateMapPoints:()=>events.push('map')};
    app.vio.getMapPoints=()=>({points:null,count:0});
    const row={clientEpoch:1,engineEpoch:1,sequence:1,statusCode:5,trackingState:'tracking_limited',initialized:true,
        inputTimestamp:1,poseTimestamp:1,poseFresh:true,poseValid:true,quality:limitedPipeline().quality,
        reason:'tracking_limited',pose:new Float64Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1])};
    app._consumeResult(row);app._consumeResult(row);
    assert.equal(events.filter(event=>event==='pose').length,1);
    assert.match(app.statusEl.textContent,/Limited/);
    assert.match(app.statusEl.textContent,/position uncertain/i);
    assert.doesNotMatch(app.statusEl.textContent,/Ceres|covariance|confidence|\d+%/i);
    app._consumeResult({...row,sequence:2,poseFresh:false,poseValid:false});assert.equal(events.at(-1),'clear');
    app._consumeResult({...row,sequence:3,engineEpoch:2});assert.equal(events.filter(event=>event==='clear').length,2);
});

test('Pose age uses only its result source timing even after a newer frame was submitted', () => {
    const { app } = appRealm();
    app.statusEl = {};
    app.lastFrameTiming = { source: 'video_presentation', timestampS: 4.9 };
    const row = { clientEpoch: 1, engineEpoch: 1, sequence: 1, statusCode: 2, initialized: true,
        inputTimestamp: 4.8, poseTimestamp: 4.8, poseFresh: true, poseValid: true,
        pose: new Float64Array([1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1]),
        sourceTiming: { source: 'video_capture_time', timestampS: 4.8 } };
    app._consumeResult(row);
    assert.match(app.statusEl.textContent, /pose age 200ms \(video_capture_time\)/);
    assert.doesNotMatch(app.statusEl.textContent, /video_presentation/);
    for (const sourceTiming of [null, undefined]) {
        app._consumeResult({ ...row, sequence: ++row.sequence, sourceTiming });
        assert.match(app.statusEl.textContent, /pose age 200ms \(camera time proxy\)/);
        assert.doesNotMatch(app.statusEl.textContent, /video_presentation|video_capture_time/);
    }
});

test('Future IMU time cannot be labeled active while the engine waits for past samples', () => {
    const { app } = appRealm();
    app.statusEl = {};
    app.imu.running = true;
    app.imu._diagnostics.lastSample = { timestampS: 65 };
    app._consumeResult({ clientEpoch: 1, engineEpoch: 1, sequence: 1,
        statusCode: 1, reason: 'imu_missing_past', initialized: false,
        inputTimestamp: 5, pose: null, poseFresh: false, poseValid: false });
    assert.doesNotMatch(app.statusEl.textContent, /IMU active/);
    assert.match(app.statusEl.textContent, /clock ahead/);
    assert.match(app.statusEl.textContent, /60000ms/);
});

function displayFixture() {
    const { app, context } = appRealm();
    let now = 1000;
    context.performance.now = () => now;
    const events = [], writes = [], metrics = { workerErrors: 0, lastError: null };
    app.updateStatus = message => writes.push(message);
    app.renderer = {
        visible: false, trajectory: [], map: null,
        updateCameraPose(pose) { this.visible = true; this.trajectory.push(pose); events.push('pose'); },
        updateMapPoints(points) { this.map = points; events.push('map'); },
        hideCameraPose() { this.visible = false; events.push('hide'); },
        clear() { this.visible = false; this.trajectory = []; this.map = null; events.push('clear'); },
    };
    app.vio.worker = {};
    app.vio.configured = true;
    app.vio.getMetrics = () => metrics;
    const map = new Float64Array([1, 2, 3]);
    app.vio.getMapPoints = () => ({ points: map, count: 1 });
    const result = Object.freeze({ clientEpoch: 1, engineEpoch: 1, sequence: 1, statusCode: 2,
        initialized: true, poseFresh: true, poseValid: true, poseTimestamp: 1, inputTimestamp: 1,
        sourceTiming: Object.freeze({ source: 'video_capture_time', timestampS: 1 }),
        pose: new Float64Array([1,0,0,1,0,1,0,2,0,0,1,3,0,0,0,1]) });
    return { app, context, result, events, writes, metrics, map, setNow: value => { now = value; } };
}

test('Display age advances for a repeated result and expires immediately after the inclusive 500ms boundary', () => {
    const f = displayFixture();
    for (const [now, age] of [[1000, 0], [1100, 100], [1500, 500]]) {
        f.setNow(now); f.app._consumeResult(f.result);
        assert.equal(f.app.renderer.visible, true);
        assert.match(f.writes.at(-1), new RegExp(`Tracking.*pose age ${age}ms`));
    }
    f.setNow(1501); f.app._consumeResult(f.result);
    assert.equal(f.app.renderer.visible, false);
    assert.match(f.writes.at(-1), /Stale.*pose age 501ms/);
    f.setNow(6000); f.app._consumeResult(f.result);
    assert.match(f.writes.at(-1), /Stale.*pose age 5000ms/);
    assert.equal(f.events.filter(event => event === 'pose').length, 1);
    assert.equal(f.events.filter(event => event === 'map').length, 1);
    assert.equal(f.events.includes('clear'), false);
    assert.deepEqual(f.app.renderer.trajectory, [f.result.pose]);
    assert.equal(f.app.renderer.map, f.map);
    assert.equal(f.app.frameMetrics.completed, 1);
    assert.equal(f.result.poseFresh, true);
    assert.equal(f.result.statusCode, 2);
    assert.equal(f.result.poseTimestamp, 1);
});

test('Repeated result status updates are at most 10Hz, with new result and stale transitions immediate', () => {
    const f = displayFixture();
    f.app.imu.running = true;
    f.app.imu._diagnostics.lastSample = { timestampS: 0.95 };
    f.app._consumeResult(f.result);
    assert.match(f.writes.at(-1), /IMU active/);
    for (let now = 1001; now < 1100; now++) { f.setNow(now); f.app._consumeResult(f.result); }
    assert.equal(f.writes.length, 1);
    f.setNow(1100); f.app._consumeResult(f.result);
    assert.equal(f.writes.length, 2);
    f.setNow(1101); f.app._consumeResult({ ...f.result, sequence: 2 });
    assert.equal(f.writes.length, 3, 'new result bypasses periodic throttle');
    f.setNow(1500); f.app._consumeResult({ ...f.result, sequence: 2 });
    assert.match(f.writes.at(-1), /IMU waiting for data/);
    const beforeExpiry = f.writes.length;
    f.setNow(1501); f.app._consumeResult({ ...f.result, sequence: 2 });
    assert.equal(f.writes.length, beforeExpiry + 1);
    assert.match(f.writes.at(-1), /Stale/);
});

test('A new result with an expired, future, or nonfinite pose time never displays a current pose', () => {
    for (const timestamp of [0, 1.001, NaN]) {
        const f = displayFixture();
        f.app._consumeResult(f.result);
        f.app._consumeResult({ ...f.result, sequence: 2, poseTimestamp: timestamp });
        assert.equal(f.app.renderer.visible, false, `timestamp ${timestamp}`);
        assert.equal(f.events.filter(event => event === 'pose').length, 1);
        assert.doesNotMatch(f.writes.at(-1), /· Tracking(?:\s|\/|·)/);
        assert.match(f.writes.at(-1), /Stale/);
        if (Number.isFinite(timestamp)) {
            assert.equal(f.events.includes('clear'), false);
            assert.equal(f.app.renderer.map, f.map);
        }
    }
});

test('Temporary null and current Worker failure hide only the frustum, then a new result resumes once', () => {
    const f = displayFixture();
    f.app._consumeResult(f.result);
    f.setNow(1001); f.app._consumeResult(null);
    assert.equal(f.app.renderer.visible, false);
    assert.match(f.writes.at(-1), /Waiting/);
    f.metrics.workerErrors = 1; f.metrics.lastError = 'fixture Worker failure';
    f.app.vio.worker = null; f.app.vio.configured = false;
    f.setNow(1002); f.app._consumeResult(null);
    assert.match(f.writes.at(-1), /Worker error.*fixture Worker failure/);
    assert.equal(f.events.includes('clear'), false);
    assert.deepEqual(f.app.renderer.trajectory, [f.result.pose]);
    assert.equal(f.app.renderer.map, f.map);
    f.app.vio.worker = {}; f.app.vio.configured = true;
    const recovered = { ...f.result, sequence: 2 };
    f.setNow(1003); f.app._consumeResult(recovered); f.app._consumeResult(recovered);
    assert.equal(f.app.renderer.visible, true);
    assert.equal(f.events.filter(event => event === 'pose').length, 2);
    assert.equal(f.events.filter(event => event === 'map').length, 2);
    assert.match(f.writes.at(-1), /Tracking/);
    f.app.vio.configured = false;
    f.setNow(1004); f.app._consumeResult(null);
    assert.match(f.writes.at(-1), /Waiting/);
    assert.doesNotMatch(f.writes.at(-1), /Worker error/);
    assert.equal(f.events.includes('clear'), false);
});

test('Missing Worker without a failure metric is Waiting, including after prior status writes', () => {
    const f = displayFixture();
    f.app.vio.worker = null; f.app.vio.configured = false;
    f.app._consumeResult(null);
    assert.match(f.writes.at(-1), /Waiting/);
    assert.doesNotMatch(f.writes.at(-1), /Worker error/);
});

test('Actual Worker failure invalidation is displayed on the next tick and normal reset still clears history', () => {
    const f = displayFixture();
    vm.runInContext(sources['vio-wrapper'].replace(/export /g, '') + '\nglobalThis.WrapperContract = VIOWrapper;', f.context);
    const wrapper = new f.context.WrapperContract();
    let terminated = false;
    wrapper.worker = { terminate() { terminated = true; } };
    wrapper.configured = true;
    wrapper._latestResult = f.result;
    wrapper._latestMapPoints = { points: f.map, count: 1 };
    f.app.vio = wrapper;
    f.app._consumeResult(wrapper.getLatestResult());
    wrapper._workerFailure(new Error('actual fixture failure'));
    assert.equal(terminated, true);
    assert.equal(wrapper.getLatestResult(), null);
    f.setNow(1001); f.app._consumeResult(wrapper.getLatestResult());
    assert.match(f.writes.at(-1), /Worker error.*actual fixture failure/);
    assert.equal(f.app.renderer.visible, false);
    assert.equal(f.events.includes('clear'), false);
    assert.deepEqual(f.app.renderer.trajectory, [f.result.pose]);
    assert.equal(f.app.renderer.map, f.map);
    wrapper.worker = {};
    wrapper._request = () => new Promise(() => {});
    wrapper.configured = true;
    f.app.camera.getVideoElement = () => ({});
    f.app._startVideoFrameCallbacks = () => {};
    f.app.resetVIO();
    assert.equal(f.events.at(-1), 'clear');
    assert.deepEqual(f.app.renderer.trajectory, []);
    assert.equal(f.app.renderer.map, null);
    f.setNow(1002); f.app._consumeResult(wrapper.getLatestResult());
    assert.match(f.writes.at(-1), /Waiting/);
    assert.doesNotMatch(f.writes.at(-1), /Worker error/);
});

test('Nonfinite display age hides the frustum without mutating the finite producer result', () => {
    const f = displayFixture();
    f.app._consumeResult(f.result);
    f.setNow(NaN); f.app._consumeResult(f.result);
    assert.equal(f.app.renderer.visible, false);
    assert.match(f.writes.at(-1), /Stale/);
    assert.equal(f.events.includes('clear'), false);
    assert.deepEqual(f.app.renderer.trajectory, [f.result.pose]);
    assert.equal(f.result.poseTimestamp, 1);
    assert.equal(f.result.poseFresh, true);
});

test('Renderer hides a current pose without discarding historical map, trajectory or transform', () => {
    const { context } = realm();
    vm.runInContext(sources.renderer.replace(/^import .*$/gm, '').replace(/export /g, '') + '\nglobalThis.RendererContract = Renderer;', context);
    const matrix = {}, trajectoryGeometry = {}, pointsGeometry = {};
    const renderer = { frustumGroup: { visible: true, matrix }, _latestPoseMatrix: matrix,
        _trajCount: 12, trajectoryGeometry, pointsGeometry };
    context.RendererContract.prototype.hideCameraPose.call(renderer);
    assert.equal(renderer.frustumGroup.visible, false);
    assert.equal(renderer._latestPoseMatrix, matrix);
    assert.equal(renderer.frustumGroup.matrix, matrix);
    assert.equal(renderer._trajCount, 12);
    assert.equal(renderer.trajectoryGeometry, trajectoryGeometry);
    assert.equal(renderer.pointsGeometry, pointsGeometry);
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

function lowFeatureFixture() {
    const f = displayFixture();
    f.app.lowFeatureEnabled = true;
    f.app.renderer.predictionVisible = false;
    f.app.renderer.plane = null;
    f.app.renderer.updateInertialPrediction = function (pose) { this.predictionVisible = true; this.predictedPose = pose; f.events.push('prediction'); };
    f.app.renderer.hideInertialPrediction = function () { this.predictionVisible = false; };
    f.app.renderer.updateGroundPlane = function (plane) { this.plane = plane; };
    f.app.renderer.hideGroundPlane = function () { this.plane = null; };
    const originalClear = f.app.renderer.clear;
    f.app.renderer.clear = function () { originalClear.call(this); this.predictionVisible = false; this.plane = null; };
    const prediction = { schema: 'mobile-slam-inertial-prediction-v1', mode: 'inertial-only', poseFrame: 'camera', engineEpoch: 1,
        timestamp: 1, anchorTimestamp: .6, maxHorizonSeconds: .5, pose: Array.from(f.result.pose),
        visualSupported: false, metricTranslationSupported: false, positionUncertain: true };
    const plane = { schema: 'mobile-slam-ground-plane-v1', kind: 'horizontal-surface-candidate', semanticClassification: 'unknown',
        state: 'retained', engineEpoch: 1, clientEpoch: 1, sourceTimestamp: .5, sourceSequence: 1,
        normal: [0,0,1], offset: 0, centroid: [0,0,0], corners: [-1,-1,0,1,-1,0,1,1,0,-1,1,0],
        supportCount: 24, sourcePointCount: 30, rms: .01, maxAgeSeconds: 1 };
    const lost = { ...f.result, sequence: 2, statusCode: 3, trackingState: 'lost', reason: 'empty_features',
        poseFresh: false, poseValid: false, poseTimestamp: null, pose: null, engineFrameTimestamp: 1,
        inertialPrediction: prediction, groundPlane: plane };
    return { ...f, prediction, plane, lost };
}

test('App keeps low-feature layers off by default and opts in only with ?low-feature=1', () => {
    for (const [search, expected] of [['', false], ['?low-feature=0', false], ['?low-feature=true', false], ['?low-feature=1', true]]) {
        assert.equal(appRealm({ search }).app.lowFeatureEnabled, expected, search);
    }
    const f = lowFeatureFixture(); f.app.lowFeatureEnabled = false;
    f.app._consumeResult(f.lost);
    assert.equal(f.app.renderer.predictionVisible, false); assert.equal(f.app.renderer.plane, null);
    assert.equal(f.app.lowFeatureDisplay.prediction, false); assert.equal(f.app.lowFeatureDisplay.groundPlane, false);
    assert.doesNotMatch(f.writes.at(-1), /IMU prediction|바닥 후보/);
    assert.match(sources.app, /lowFeature: this\.lowFeatureEnabled/);
});

test('Inertial-only display stays separate from Lost metric pose and never appends trajectory or updates maps', () => {
    const f=lowFeatureFixture();f.app._consumeResult(f.lost);f.app._consumeResult(f.lost);
    assert.equal(f.app.renderer.predictionVisible,true);
    assert.equal(f.app.renderer.visible,false);
    assert.equal(f.events.filter(x=>x==='pose'||x==='map').length,0);
    assert.equal(f.events.filter(x=>x==='prediction').length,1);
    assert.match(f.writes.at(-1),/Lost.*IMU prediction.*position uncertain/);
    assert.equal(f.lost.poseFresh,false);assert.equal(f.lost.pose,null);
    assert.equal(f.app.lowFeatureDisplay.prediction,true);
});

test('Prediction source age100ms and source horizon500ms are inclusive and arrival cannot renew expiry', () => {
    const f=lowFeatureFixture();f.lost.inertialPrediction={...f.prediction,anchorTimestamp:.5};
    f.setNow(1100);f.app._consumeResult(f.lost);assert.equal(f.app.renderer.predictionVisible,true);
    f.setNow(1100.01);f.app._consumeResult({...f.lost,sequence:3});assert.equal(f.app.renderer.predictionVisible,false);
    f.setNow(1000);f.app._consumeResult({...f.lost,sequence:4,inertialPrediction:{...f.prediction,anchorTimestamp:.49999}});
    assert.equal(f.app.renderer.predictionVisible,false);
    f.app._consumeResult({...f.lost,sequence:5,inertialPrediction:{...f.prediction,timestamp:1.001}});
    assert.equal(f.app.renderer.predictionVisible,false);
});

test('Current visual pose takes priority and raw visual recovery replaces prediction without changing pose', () => {
    const f=lowFeatureFixture();f.app._consumeResult(f.lost);assert.equal(f.app.renderer.predictionVisible,true);
    const visual={...f.result,sequence:3,inertialPrediction:f.prediction};f.app._consumeResult(visual);
    assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.visible,true);
    assert.equal(f.events.filter(x=>x==='pose').length,1);assert.equal(f.events.filter(x=>x==='map').length,1);
    assert.equal(f.app.renderer.trajectory[0],visual.pose);
});

test('Ground candidate retained geometry expires independently on RAF without extending sourceTimestamp', () => {
    const f=lowFeatureFixture();f.app._consumeResult(f.lost);assert.equal(f.app.renderer.plane,f.plane);
    assert.match(f.writes.at(-1),/바닥 후보.*분류 미확인.*retained.*24/);
    f.setNow(1500);f.app._consumeResult(f.lost);assert.equal(f.app.renderer.plane,f.plane);
    assert.equal(f.app.renderer.predictionVisible,false);
    f.setNow(1500.01);f.app._consumeResult(f.lost);assert.equal(f.app.renderer.plane,null);
    assert.equal(f.plane.sourceTimestamp,.5);
});

test('Null, failure, old payload epoch and source sequence clear extra displays; old binary is unavailable', () => {
    const f=lowFeatureFixture();f.app._consumeResult(f.lost);f.app._consumeResult(null);
    assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.plane,null);
    f.app._consumeResult({...f.lost,sequence:3,engineEpoch:2});
    assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.plane,null);
    f.app._consumeResult({...f.lost,sequence:4,groundPlane:{...f.plane,sourceSequence:99}});
    assert.equal(f.app.renderer.plane,null);
    f.metrics.workerErrors=1;f.app.vio.worker=null;f.app.vio.configured=false;
    f.app._consumeResult({...f.lost,sequence:5});assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.plane,null);
    f.metrics.workerErrors=0;f.app.vio.worker={};f.app.vio.configured=true;
    f.app._consumeResult({...f.lost,sequence:6,inertialPrediction:null,groundPlane:null});
    assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.plane,null);
});

test('Source deadlines reject a late producer, reset clears all extra layers, and no auxiliary payload means old-binary behavior', () => {
    const f=lowFeatureFixture();
    f.setNow(5000);f.app._consumeResult(f.lost);
    assert.equal(f.app.lowFeatureDisplay.prediction,false);assert.equal(f.app.lowFeatureDisplay.groundPlane,false);
    assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.plane,null);
    f.setNow(1000);f.app._consumeResult({...f.lost,sequence:3});
    assert.equal(f.app.renderer.predictionVisible,true);assert.notEqual(f.app.renderer.plane,null);
    f.app.vio.reset=()=>{};f.app.camera.getVideoElement=()=>({});f.app._startVideoFrameCallbacks=()=>{};
    f.app.resetVIO();
    assert.equal(f.app.lowFeatureDisplay.prediction,false);assert.equal(f.app.lowFeatureDisplay.groundPlane,false);
    assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.plane,null);
    const legacy={...f.lost,sequence:4};delete legacy.inertialPrediction;delete legacy.groundPlane;
    f.app._consumeResult(legacy);assert.match(f.writes.at(-1),/Lost/);assert.doesNotMatch(f.writes.at(-1),/IMU prediction|바닥 후보/);
    assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.plane,null);
});

test('Retained floor rejects foreign client identity and nonfinite source time without changing original payload', () => {
    for(const change of [{clientEpoch:99},{sourceTimestamp:NaN},{sourceTimestamp:1.01}]) {
        const f=lowFeatureFixture();const plane={...f.plane,...change};
        f.app._consumeResult({...f.lost,groundPlane:plane});assert.equal(f.app.renderer.plane,null);
        assert.equal(f.plane.sourceTimestamp,.5);assert.equal(f.plane.clientEpoch,1);
    }
});

test('Blur/visibility/capture lifecycle removes extra layers even when the result loop is paused', () => {
    const f=lowFeatureFixture();let draws=0;f.app.renderer.render=()=>{draws++;};
    f.app._consumeResult(f.lost);assert.equal(f.app.lowFeatureDisplay.prediction,true);
    f.app._clearVideoFrameCapture();
    assert.equal(f.app.lowFeatureDisplay.prediction,false);assert.equal(f.app.lowFeatureDisplay.groundPlane,false);
    assert.equal(f.app.renderer.predictionVisible,false);assert.equal(f.app.renderer.plane,null);assert.equal(draws,1);
});
