import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../web/js/app.js', import.meta.url), 'utf8');
const cameraSource = await fs.readFile(new URL('../../web/js/camera.js', import.meta.url), 'utf8');
const { processingDimensions } = await import('data:text/javascript;base64,' + Buffer.from(cameraSource).toString('base64'));

function fixture({ callbacks = true, now = 1337516.710 } = {}) {
    let clock = now, pending = null, handle = 0, epoch = 1, captureNull = false, captureThrows = false, captures = 0;
    const scheduled = new Map(), pixels = new Uint8Array([10,20,30,40]), sentPixels = [];
    const sent = [], video = { currentTime: 1335.548207, videoWidth: 2, videoHeight: 2 };
    const geometry = { sourceWidth: 2, sourceHeight: 2, rotation: 'none', cropX: 0, cropY: 0, cropWidth: 2, cropHeight: 2, width: 2, height: 2 };
    if (callbacks) {
        video.requestVideoFrameCallback = callback => { pending = ++handle; scheduled.set(pending,callback); return pending; };
        video.cancelVideoFrameCallback = id => { scheduled.delete(id); };
    }
    const context = vm.createContext({ processingDimensions, URL, URLSearchParams, Uint8Array, Float64Array, console: { log() {}, warn() {} },
        performance: { now: () => clock, timeOrigin: 1700000000000 },
        window: { location: { search: '', href: 'https://fixture.invalid/' } },
        navigator: { userAgent: 'Android fixture' }, document: { hidden: false, addEventListener() {} },
        requestAnimationFrame() {}, VIOWrapper: class {}, Camera: class {}, IMU: class {}, OrientationHandler: class {},
    });
    vm.runInContext(source.replace(/^import .*$/gm, '').replace(/const app = new App\(\);[\s\S]*$/, '') + '\nglobalThis.AppContract = App;', context);
    const app = new context.AppContract();
    app.running = true; app._frameWarmupCount = 5; app._processScale = 1;
    app._captureWidth = app._processWidth = 2; app._captureHeight = app._processHeight = 2;
    app._lastConfigParams = { width: 2, height: 2 };
    app.lastFPSTime = now; app.lastImuUpdateTime = now;
    app.camera = { width: 2, height: 2, getVideoElement: () => video, getPixelTransform: () => ({ ...geometry }),
        getRotateMode: () => geometry.rotation, stop() {}, captureGrayscale: () => {
            captures++; if (captureThrows) throw new Error('fixture readback unavailable'); return captureNull ? null : pixels;
        } };
    app.vio = { sendFrame: (gray, timestamp) => { sent.push(timestamp); sentPixels.push(Array.from(gray)); return true; },
        getLatestResult: () => null, getMetrics: () => ({ clientEpoch: epoch }), reset() { epoch++; }, dispose() {} };
    app.imu = { discard() {}, getRate: () => 60, stop() {} };
    app.orientation = { stopListening() {} };
    app._flushAndSendIMU = () => 1; app.updateIMUOverlay = () => {};
    app._startIMUFlush = () => {}; app._stopIMUFlush = () => {};
    return { app, video, sent, sentPixels, geometry, context, captures: () => captures,
        setPixels: values => pixels.set(values), setNull: value => { captureNull = value; }, setThrows: value => { captureThrows = value; },
        nextEpoch: () => { epoch++; }, currentCallback: () => scheduled.get(pending),
        setNow: value => { clock = value; }, start: () => app._startVideoFrameCallbacks(video),
        deliver: metadata => { assert.ok(scheduled.has(pending), 'Actual video callback must be registered'); const callback = scheduled.get(pending); scheduled.delete(pending); callback(clock, metadata); } };
}

function metadata(presentedFrames = 10, values = {}) {
    return { presentedFrames, mediaTime: 2314658.08417, presentationTime: 1337508.335, captureTime: 1337504.805, ...values };
}

test('Different live media origins preserve valid captureTime and raw mediaTime metadata', () => {
    const f = fixture(); f.start(); f.deliver(metadata()); f.app.processLoop();
    assert.deepEqual(f.sent, [1337.504805]);
    assert.equal(f.app.lastFrameTiming.mediaTimeS, 2314658.08417);
    assert.equal(f.app.lastFrameTiming.source, 'video_capture_time');
    assert.equal(f.app.lastFrameTiming.presentedFrames, 10);
    assert.equal(f.app.lastFrameTiming.association, 'video_frame_callback_snapshot');
    assert.equal(f.app.lastFrameTiming.physicalExposureVerified, false);
});

test('Presented frame ID deduplicates repeated callbacks despite currentTime changes', () => {
    const f = fixture(); f.start(); f.deliver(metadata()); f.app.processLoop();
    f.setNow(1337616.710); f.video.currentTime++;
    f.deliver(metadata(10, { captureTime: 1337604.805 })); f.app.processLoop();
    assert.equal(f.sent.length, 1);
    f.deliver(metadata(11, { captureTime: 1337604.805 })); f.app.processLoop();
    assert.deepEqual(f.sent, [1337.504805, 1337.604805]);
});

test('Increasing callback frame ID is consumed even with constant currentTime', () => {
    const f = fixture(); f.start(); f.deliver(metadata()); f.app.processLoop();
    f.setNow(1337616.710); f.deliver(metadata(11, { captureTime: 1337604.805 })); f.app.processLoop();
    assert.equal(f.sent.length, 2);
});

test('Supported callback API waits for a frame ID instead of assigning arrival to an unassociated frame', () => {
    const f = fixture(); f.start(); f.app.processLoop();
    assert.equal(f.sent.length, 0);
});

test('Past DOM capture time remains valid without inventing a callback-time offset', () => {
    const f = fixture({ now: 5000 }); f.start(); f.deliver(metadata(1, { captureTime: 4900, presentationTime: 4995 })); f.app.processLoop();
    assert.deepEqual(f.sent, [4.9]);
    assert.equal(f.app.lastFrameTiming.callbackArrivalMs, 5000);
    assert.equal(f.app.lastFrameTiming.physicalExposureVerified, false);
});

test('Invalid or future capture time selects a valid presentation time', () => {
    for (const captureTime of [5002, -1, NaN, Infinity, undefined]) {
        const f = fixture({ now: 5000 }); f.start(); f.deliver(metadata(1, { captureTime, presentationTime: 4995 })); f.app.processLoop();
        assert.deepEqual(f.sent, [4.995]);
        assert.equal(f.app.lastFrameTiming.source, 'video_presentation');
        assert.equal(f.app.lastFrameTiming.physicalExposureVerified, false);
    }
});

test('Invalid capture and presentation timestamps fall back to callback arrival', () => {
    const f = fixture({ now: 5000 }); f.start(); f.deliver(metadata(1, { captureTime: 5002, presentationTime: 900000000 })); f.app.processLoop();
    assert.deepEqual(f.sent, [5]);
    assert.equal(f.app.lastFrameTiming.source, 'video_callback_arrival');
    assert.equal(f.app.lastFrameTiming.captureTimeMs, 5002, 'Retain raw rejected timing for diagnosis');
});

test('Reset clears callback metadata and waits for another presented frame', () => {
    const f = fixture(); f.start(); f.deliver(metadata()); f.app.processLoop(); f.app.resetVIO();
    f.setNow(1337616.710); f.video.currentTime++; f.app.processLoop();
    assert.equal(f.sent.length, 1);
    f.deliver(metadata(11, { captureTime: 1337604.805 })); f.app.processLoop();
    assert.equal(f.sent.length, 2);
});

test('API-unsupported path keeps currentTime dedup and canvas arrival provenance', () => {
    const f = fixture({ callbacks: false, now: 5000 }); f.start(); f.app.processLoop();
    assert.deepEqual(f.sent, [5]);
    assert.equal(f.app.lastFrameTiming.source, 'canvas_read_arrival');
    assert.equal(f.app.lastFrameTiming.physicalExposureVerified, false);
    f.setNow(5100); f.app.processLoop(); assert.equal(f.sent.length, 1);
    f.video.currentTime++; f.app.processLoop(); assert.deepEqual(f.sent, [5, 5.1]);
});

test('Frame A callback followed by mutable video B before RAF submits owned A pixels with A time', () => {
    const f = fixture(); f.start(); f.deliver(metadata());
    f.setPixels([50,60,70,80]); f.video.currentTime++;
    f.app.processLoop();
    assert.deepEqual(f.sent,[1337.504805]);
    assert.deepEqual(f.sentPixels,[[10,20,30,40]]);
    assert.equal(f.captures(),1,'RAF must not read the mutable video again');
});

test('Multiple callbacks coalesce to one owned latest packet and consume it once', () => {
    const f = fixture({now:6000}); f.start();
    for (let index=1;index<=64;index++) {
        f.setPixels([index,index,index,index]);
        f.deliver(metadata(index,{captureTime:5000+index,presentationTime:5000+index}));
    }
    f.setPixels([99,99,99,99]); f.app.processLoop(); f.app.processLoop();
    assert.deepEqual(f.sent,[5.064]); assert.deepEqual(f.sentPixels,[[64,64,64,64]]);
    assert.equal(f.app.frameMetrics.snapshotCaptured,64);
    assert.equal(f.app.frameMetrics.snapshotCoalesced,63);
    assert.equal(f.app.frameMetrics.snapshotConsumed,1);
});

for (const failure of ['null','throw']) test(`${failure} capture clears a pending packet without relabeling old pixels`, () => {
    const f=fixture(); f.start(); f.deliver(metadata());
    if(failure==='null')f.setNull(true);else f.setThrows(true);
    f.setNow(1337616.710); f.deliver(metadata(11,{captureTime:1337604.805}));
    f.setNull(false);f.setThrows(false);f.setPixels([80,80,80,80]);f.app.processLoop();
    assert.equal(f.sent.length,0);
    assert.equal(f.app.frameMetrics.snapshotCaptureFailed,1);
    f.deliver(metadata(12,{captureTime:1337604.805}));f.app.processLoop();
    assert.deepEqual(f.sentPixels,[[80,80,80,80]]);
});

test('Reset invalidates an already scheduled callback even on the same video element', () => {
    const f=fixture();f.start();f.deliver(metadata());
    const oldCallback=f.currentCallback(), oldGeneration=f.app.captureGeneration;
    f.app.resetVIO();f.setNow(1337616.710);oldCallback(1337616.710,metadata(11,{captureTime:1337604.805}));f.app.processLoop();
    assert.equal(f.sent.length,0);assert.ok(f.app.captureGeneration>oldGeneration);
    f.deliver(metadata(12,{captureTime:1337604.805}));f.app.processLoop();
    assert.equal(f.sent.length,1);assert.equal(f.app.lastFrameTiming.captureGeneration,f.app.captureGeneration);
});

test('Stop and restart exclude callbacks from the previous session on the same video', () => {
    const f=fixture();f.start();const oldCallback=f.currentCallback();
    f.app.stop();f.app.running=true;f.start();oldCallback(1337516.710,metadata());f.app.processLoop();
    assert.equal(f.sent.length,0);
    f.deliver(metadata(11));f.app.processLoop();assert.equal(f.sent.length,1);
});

test('Capture and consumption reject natural-source resize even when cropped output size is unchanged', () => {
    for(const when of ['capture','consume']) {
        const f=fixture();f.start();
        if(when==='consume')f.deliver(metadata());
        f.video.videoWidth=4;f.video.videoHeight=1;
        if(when==='capture')f.deliver(metadata());
        f.app.processLoop();assert.equal(f.sent.length,0);
        assert.equal(f.app.frameMetrics.snapshotGeometryRejected,1);
    }
});

test('A packet cannot cross a Worker client epoch or calibration object change', () => {
    for(const change of ['epoch','config','rotation']) {
        const f=fixture();f.start();f.deliver(metadata());
        if(change==='epoch')f.nextEpoch();
        else if(change==='config')f.app._lastConfigParams={...f.app._lastConfigParams};
        else f.geometry.rotation='half';
        f.app.processLoop();assert.equal(f.sent.length,0);
    }
});

test('Hidden, blur-paused and reconfiguring callbacks cannot retain a publishable packet', () => {
    for(const pause of ['hidden','blur','reconfigure']) {
        const f=fixture();f.start();f.deliver(metadata());
        if(pause==='hidden')f.context.document.hidden=true;
        else if(pause==='blur')f.app._blurPaused=true;
        else f.app._reconfiguring=true;
        f.deliver(metadata(11));
        f.context.document.hidden=false;f.app._blurPaused=false;f.app._reconfiguring=false;
        f.app.processLoop();assert.equal(f.sent.length,0);
        f.deliver(metadata(12));f.app.processLoop();assert.equal(f.sent.length,1);
    }
});

test('Visibility resume invalidates the old callback session and rearms fresh capture', () => {
    const f=fixture();f.app._startDeviceOrientationListener=()=>{};f.app._stopDeviceOrientationListener=()=>{};
    f.start();f.deliver(metadata());const oldCallback=f.currentCallback(),generation=f.app.captureGeneration;
    f.context.document.hidden=true;f.app._onVisibilityChange();
    f.context.document.hidden=false;f.app._onVisibilityChange();
    oldCallback(1337516.710,metadata(11));f.app.processLoop();
    assert.equal(f.sent.length,0);assert.ok(f.app.captureGeneration>generation);
    f.deliver(metadata(12));f.app.processLoop();assert.equal(f.sent.length,1);
});

test('Async geometry reconfiguration discards old pixels before awaiting and rearms after configure', async () => {
    const f=fixture();f.start();f.deliver(metadata());const oldCallback=f.currentCallback(),generation=f.app.captureGeneration;
    let release;f.app.camera.redetectOrientation=()=>new Promise(resolve=>{release=resolve;});
    f.app._cameraCalibration=()=>({calibration:{width:2,height:2,r_ic:[1,0,0,0,1,0,0,0,1]}});
    f.app._applyRuntimeParameters=async()=>{};
    f.app.vio.configure=async()=>{f.nextEpoch();return true;};
    const reconfigured=f.app._onOrientationChange({type:'portrait-primary'});
    oldCallback(1337516.710,metadata(11));f.app.processLoop();assert.equal(f.sent.length,0);
    release();await reconfigured;f.app.processLoop();assert.equal(f.sent.length,0);
    assert.ok(f.app.captureGeneration>generation);
    f.deliver(metadata(12));f.app.processLoop();assert.equal(f.sent.length,1);
    assert.equal(f.app.lastFrameTiming.clientEpoch,2);
});

test('Existing initial frame interval applies to snapshot consumption', () => {
    const f=fixture({now:5000});f.start();f.deliver(metadata(1,{captureTime:4990}));f.app.processLoop();
    f.setNow(5020);f.deliver(metadata(2,{captureTime:5015}));f.app.processLoop();assert.equal(f.sent.length,1);
    f.setNow(5050);f.app.processLoop();assert.deepEqual(f.sent,[4.99,5.015]);
});
