import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const cameraSource = await fs.readFile(new URL('../../web/js/camera.js', import.meta.url), 'utf8');
const appSource = await fs.readFile(new URL('../../web/js/app.js', import.meta.url), 'utf8');
const portraitRIC = [1,0,0,0,0,1,0,-1,0];
const info = (type = 'portrait-primary', r_ic = portraitRIC) => ({ type, r_ic });

function fixture({ search = '', width = 640, height = 480, darkEdge = false } = {}) {
    const video = { videoWidth: width, videoHeight: height, readyState: 2, setAttribute() {}, play: async () => {},
        set srcObject(value) { if (value) queueMicrotask(() => this.onloadedmetadata?.()); } };
    const track = { getSettings: () => ({ width, height }), getCapabilities: () => ({}), stop() {} };
    const context = vm.createContext({ URL, URLSearchParams, queueMicrotask, setTimeout, clearTimeout,
        console: { log() {}, warn() {}, error() {} },
        window: { location: { search, href: `http://fixture.invalid/${search}` } },
        navigator: { userAgent: 'Android fixture', mediaDevices: { getUserMedia: async () => ({ getVideoTracks: () => [track], getTracks: () => [track] }) } },
        document: { addEventListener() {}, hidden: false, createElement: type => type === 'video' ? video : {
            getContext: () => ({ drawImage() {}, getImageData() { const data = new Uint8Array(width * height * 4).fill(120);
                if (darkEdge) for (let y = 0; y < height; y++) data.fill(0, (y * width + Math.floor(width * 0.95)) * 4, (y + 1) * width * 4);
                return { data }; } }) } },
    });
    vm.runInContext(cameraSource.replace(/export /g, '') + '\nglobalThis.CameraContract=Camera;', context);
    vm.runInContext(appSource.replace(/^import .*$/gm, '').replace(/const app = new App\(\);[\s\S]*$/, '') + '\nglobalThis.AppContract=App;', context);
    const camera = new context.CameraContract(), app = Object.create(context.AppContract.prototype), calls = [];
    let fThreshold = 2;
    Object.assign(app, { running: true, activeConfig: 'mobile_default', camera, renderer: null,
        _lastConfigParams: { width, height, r_ic: portraitRIC, mobileMotionGuard: true, acc_n: 0.5, acc_w: 0.003, gyr_n: 0.2, gyr_w: 0.0002, g_norm: 9.81 },
        imu: { discard() {} }, _stopIMUFlush() { this.flushPaused = true; }, _startIMUFlush() { this.flushPaused = false; },
        updateStatus() {}, stop() { this.running = false; },
        vio: { reset() {}, configure: async params => { calls.push(['configure', structuredClone(params)]); fThreshold = 1; return true; },
            setMobileParams: async (...args) => { calls.push(['mobile', ...args]); return true; },
            setTrackingParams: async (...args) => { calls.push(['tracking', ...args]); return true; },
            setPnPParams: async (...args) => { calls.push(['pnp', ...args]); return true; },
            setFThreshold: async value => { calls.push(['fThreshold', value]); fThreshold = value; return true; } } });
    return { camera, app, video, calls, context, fThreshold: () => fThreshold };
}

test('Natural video dimensions and rotation do not depend on image brightness', async () => {
    for (const darkEdge of [false, true]) {
        const f = fixture({ darkEdge }); await f.camera.initialize();
        assert.equal(f.camera.getRotateMode(), 'none');
        assert.deepEqual([f.camera.width, f.camera.height], [640, 480]);
        assert.deepEqual([f.camera.getPixelTransform().sourceWidth, f.camera.getPixelTransform().sourceHeight], [640, 480]);
    }
});

test('Explicit none preserves landscape pixels and actual profile source dimensions', async () => {
    const f = fixture({ search: '?rotate=none' }); await f.camera.initialize();
    assert.deepEqual([f.camera.width, f.camera.height], [640, 480]);
    assert.deepEqual([f.camera.getPixelTransform().sourceWidth, f.camera.getPixelTransform().sourceHeight], [640, 480]);
});

test('Every explicit rotation survives video resize while dimensions refresh', async () => {
    for (const rotation of ['none', 'cw', 'ccw', 'half']) {
        const f = fixture({ search: `?rotate=${rotation}` }); await f.camera.initialize();
        f.video.videoWidth = 480; f.video.videoHeight = 640;
        await f.camera.redetectOrientation();
        assert.equal(f.camera.getRotateMode(), rotation);
        assert.deepEqual([f.camera.getPixelTransform().sourceWidth, f.camera.getPixelTransform().sourceHeight], [480, 640]);
        assert.deepEqual([f.camera.width, f.camera.height], ['cw','ccw'].includes(rotation) ? [640,480] : [480,640]);
    }
});

test('Heuristic intrinsics and camera-body rotation follow each actual pixel rotation', async () => {
    const expected = { none: portraitRIC, cw: [0,1,0,0,0,1,1,0,0], ccw: [0,-1,0,0,0,1,-1,0,0], half: [-1,0,0,0,0,1,0,1,0] };
    for (const rotation of Object.keys(expected)) {
        const f = fixture({ search: `?rotate=${rotation}&fx=400` }); await f.camera.initialize();
        await f.app._onOrientationChange(info());
        const params = f.calls.find(call => call[0] === 'configure')[1];
        assert.deepEqual(params.r_ic.map(value => value || 0), expected[rotation]);
        assert.deepEqual([params.fx,params.fy], [400,400]);
        assert.deepEqual([params.cx,params.cy], ['cw','ccw'].includes(rotation) ? [239.5,319.5] : [319.5,239.5]);
        assert.equal(f.app.calibrationProvenance.status, 'heuristic_unverified');
    }
});

test('Orientation reconfiguration reapplies the complete runtime profile including fth', async () => {
    const f = fixture({ search: '?rotate=none&fth=2&pnp=1&freq=4' }); await f.camera.initialize();
    await f.app._onOrientationChange(info());
    assert.equal(f.fThreshold(), 2);
    assert.deepEqual(f.calls.filter(call => call[0] !== 'configure'), [
        ['mobile',0.06,10,120], ['tracking',21,3,30,0], ['pnp',true,4], ['fThreshold',2],
    ]);
});

test('Runtime parameter rejection cannot complete reconfiguration', async () => {
    const f = fixture({ search: '?rotate=none' }); await f.camera.initialize();
    f.app.vio.setTrackingParams = async () => false;
    await assert.rejects(f.app._onOrientationChange(info()), /tracking/i);
});

test('Overlapping orientation updates serialize configuration and retain the latest request', async () => {
    const f = fixture({ search: '?rotate=none' }); await f.camera.initialize();
    let release, active = 0, maximum = 0, count = 0;
    f.app.vio.configure = async params => { active++; maximum = Math.max(maximum,active); f.calls.push(['configure', structuredClone(params)]);
        if (++count === 1) await new Promise(resolve => { release = resolve; }); active--; return true; };
    const first = f.app._onOrientationChange(info());
    await new Promise(resolve => setImmediate(resolve));
    const lastRIC = [0,1,0,0,0,1,1,0,0];
    const second = f.app._onOrientationChange(info('landscape-primary', lastRIC));
    await new Promise(resolve => setImmediate(resolve)); release(); await Promise.all([first,second]);
    assert.equal(maximum,1);
    assert.deepEqual(Array.from(f.app._lastConfigParams.r_ic),lastRIC);
    assert.equal(f.app._reconfiguring,false);
});

test('Stopping during geometry refresh cannot configure or resume IMU afterward', async () => {
    const f = fixture({ search: '?rotate=none' }); await f.camera.initialize();
    let release;
    f.camera.redetectOrientation = () => new Promise(resolve => { release = resolve; });
    const pending = f.app._onOrientationChange(info());
    f.app.stop(); release(); await pending;
    assert.equal(f.calls.length,0);
    assert.equal(f.app.flushPaused,true);
    assert.equal(f.app._reconfiguring,false);
});

test('A reset during camera reconfiguration cannot invalidate the pending configure', async () => {
    const f = fixture({ search: '?rotate=none' }); await f.camera.initialize();
    let complete, rejectConfigure, resetCalls = 0;
    f.app.vio.configure = () => new Promise((resolve,reject) => { complete = resolve; rejectConfigure = reject; });
    f.app.vio.reset = () => { resetCalls++; rejectConfigure(new Error('Worker reset invalidated configuration')); };
    const pending = f.app._onOrientationChange(info());
    await new Promise(resolve => setImmediate(resolve));
    f.app.resetVIO(); complete(true); await pending;
    assert.equal(resetCalls,0, 'The in-progress configure already establishes a new estimator lifecycle');
    assert.equal(f.app.running,true);
});

test('Geometry reconfiguration does not resume IMU while hidden or blur-paused', async () => {
    for (const pause of ['hidden','blur']) {
        const f = fixture({ search: '?rotate=none' }); await f.camera.initialize();
        if (pause === 'hidden') f.context.document.hidden = true;
        else f.app._blurPaused = true;
        await f.app._onOrientationChange(info());
        assert.equal(f.app.flushPaused,true);
        assert.equal(f.app._reconfiguring,false);
    }
});

test('IMU flush cannot start while stopped, reconfiguring, hidden or blur-paused', () => {
    const f = fixture(); let timers = 0;
    f.context.setInterval = () => { timers++; return 42; };
    const start = Object.getPrototypeOf(f.app)._startIMUFlush;
    for (const state of ['stopped','reconfiguring','hidden','blur']) {
        f.app.running = state !== 'stopped'; f.app._reconfiguring = state === 'reconfiguring';
        f.context.document.hidden = state === 'hidden'; f.app._blurPaused = state === 'blur';
        start.call(f.app);
    }
    assert.equal(timers,0);
    f.app.running = true; f.app._reconfiguring = false; f.context.document.hidden = false; f.app._blurPaused = false;
    start.call(f.app); assert.equal(timers,1);
});

test('Wide natural source center crop uses source K before rounded processing resize', async () => {
    const f = fixture({ search: '?rotate=none&fx=400', width: 640, height: 360 }); await f.camera.initialize();
    f.camera.enableLandscapeCrop(); f.app.activeConfig = 'mobile_highend';
    await f.app._onOrientationChange(info());
    const params = f.calls.find(call => call[0] === 'configure')[1];
    assert.deepEqual([params.width,params.height],[322,240]);
    assert.ok(Math.abs(params.fx - 400*322/480) < 1e-12);
    assert.ok(Math.abs(params.fy - 400*240/360) < 1e-12);
    assert.deepEqual([params.cx,params.cy],[160.5,119.5]);
    assert.equal(f.app.calibrationProvenance.transform.cropX,80);
});
