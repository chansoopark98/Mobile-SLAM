import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import vm from 'node:vm';

const source = await fs.readFile(new URL('../../web/js/camera.js', import.meta.url), 'utf8');
const devices = [
    { kind: 'videoinput', label: 'camera 1, facing front', deviceId: 'fixture-front-one' },
    { kind: 'videoinput', label: 'camera 3, facing front', deviceId: 'fixture-front-three' },
    { kind: 'videoinput', label: 'camera 2, facing back', deviceId: 'fixture-rear-two' },
    { kind: 'videoinput', label: 'camera 0, facing back', deviceId: 'fixture-rear-zero' },
];

function fixture({ search = '', listed = devices, labelsHidden = false, replacementFacing = 'environment',
    replacementId = 'fixture-rear-zero', replacementError = null, enumerationError = null,
    secondEnumerationError = null, replacementEmpty = false, bootstrapError = null } = {}) {
    const events = [], requests = [], captures = [];
    let enumerationCount = 0, permission = !labelsHidden;
    function stream(name, deviceId, facingMode, empty = false) {
        const track = { readyState: 'live', stopped: 0,
            getSettings() { events.push(`${name}:settings`); return { deviceId, facingMode, width: 640, height: 480 }; },
            getCapabilities() { return {}; },
            stop() { this.stopped++; this.readyState = 'ended'; events.push(`${name}:stop`); } };
        return { track, getVideoTracks: () => empty ? [] : [track], getTracks: () => [track] };
    }
    const initial = stream('bootstrap', 'fixture-rear-two', 'environment');
    const replacement = stream('selected', replacementId, replacementFacing, replacementEmpty);
    const context = vm.createContext({ URLSearchParams, console: { log() {}, warn() {} }, setTimeout,
        window: { location: { search: search + (search ? '&' : '?') + 'rotate=none' } },
        navigator: { mediaDevices: {
            async getUserMedia(constraints) {
                requests.push(JSON.parse(JSON.stringify(constraints))); events.push(`request:${requests.length}`);
                if (captures.some(capture => !capture.track.stopped)) throw new Error('fixture exclusive camera hardware busy');
                if (constraints.video.deviceId) {
                    if (replacementError) throw replacementError;
                    captures.push(replacement); return replacement;
                }
                if (bootstrapError) throw bootstrapError;
                permission = true; captures.push(initial); return initial;
            },
            async enumerateDevices() {
                events.push('enumerate'); enumerationCount++;
                if (enumerationError) throw enumerationError;
                if (secondEnumerationError && enumerationCount === 2) throw secondEnumerationError;
                return permission ? listed : devices.map(device => ({ ...device, label: '' }));
            },
        } },
        document: { createElement(type) {
            if (type === 'video') return { readyState: 2, videoWidth: 640, videoHeight: 480,
                setAttribute() {}, play: () => Promise.resolve(),
                set srcObject(value) { if (value) queueMicrotask(() => this.onloadedmetadata()); } };
            return { getContext: () => ({}) };
        } },
    });
    vm.runInContext(source.replace(/export /g, '') + '\nglobalThis.CameraContract = Camera;', context);
    return { camera: new context.CameraContract(), events, requests, initial, replacement, captures };
}

test('Default camera keeps the existing rear capture without enumeration or replacement', async () => {
    const f = fixture(); await f.camera.initialize();
    assert.equal(f.requests.length, 1);
    assert.equal(f.requests[0].video.facingMode.exact, 'environment');
    assert.ok(!f.events.includes('enumerate'));
    assert.equal(f.camera.stream, f.initial);
    f.camera.stop(); assert.equal(f.initial.track.stopped, 1);
});

test('Visible rear label opens its selected camera directly with the original capture constraints', async () => {
    const f = fixture({ search: '?camera=0' }); await f.camera.initialize(640, 480);
    assert.equal(f.requests.length, 1);
    assert.ok(f.events.indexOf('enumerate') < f.events.indexOf('request:1'));
    assert.equal(f.requests[0].video.deviceId.exact, 'fixture-rear-zero');
    assert.deepEqual(f.requests[0].video.width, { ideal: 640 });
    assert.deepEqual(f.requests[0].video.height, { ideal: 480 });
    assert.deepEqual(f.requests[0].video.frameRate, { ideal: 30, min: 20 });
    assert.equal(f.requests[0].video.facingMode.exact, 'environment');
    assert.equal(f.initial.track.stopped, 0, 'No bootstrap was opened');
    assert.equal(f.camera.stream, f.replacement);
    f.camera.stop(); assert.equal(f.replacement.track.stopped, 1);
});

test('Hidden labels use a permission bootstrap that closes before exclusive selected-camera open', async () => {
    const f = fixture({ search: '?camera=0', labelsHidden: true }); await f.camera.initialize();
    assert.equal(f.requests.length, 2);
    assert.equal(f.requests[0].video.deviceId, undefined);
    assert.equal(f.requests[1].video.deviceId.exact, 'fixture-rear-zero');
    assert.ok(f.events.indexOf('bootstrap:stop') < f.events.indexOf('request:2'));
    assert.equal(f.initial.track.stopped, 1); assert.equal(f.replacement.track.stopped, 0);
    assert.equal(f.camera.stream, f.replacement);
    f.camera.stop(); assert.equal(f.replacement.track.stopped, 1);
});

test('Explicit selection rejects missing, front and ambiguous visible labels before capture', async () => {
    const cases = [[], devices.filter(d => d.label.includes('front')),
        [...devices, { kind: 'videoinput', label: 'camera 0, facing back', deviceId: 'fixture-duplicate' }]];
    for (const listed of cases) {
        const f = fixture({ search: '?camera=0', listed });
        await assert.rejects(f.camera.initialize(), /camera|rear|ambiguous/i);
        assert.equal(f.requests.length, 0); assert.equal(f.camera.stream, null);
    }
});

test('Malformed camera selection is rejected before enumeration or capture', async () => {
    for (const value of ['', 'front', '-1', '0.5', '00', '0&camera=2']) {
        const f = fixture({ search: '?camera=' + value });
        await assert.rejects(f.camera.initialize(), /camera|selection/i);
        assert.equal(f.requests.length, 0); assert.ok(!f.events.includes('enumerate'));
    }
});

test('Selected-camera failure closes its permission bootstrap without another-device fallback', async () => {
    for (const labelsHidden of [false, true]) {
        const f = fixture({ search: '?camera=0', labelsHidden, replacementError: new Error('fixture selected camera unavailable') });
        await assert.rejects(f.camera.initialize(), /unavailable/);
        assert.equal(f.requests.length, labelsHidden ? 2 : 1);
        assert.equal(f.initial.track.stopped, labelsHidden ? 1 : 0);
        assert.equal(f.camera.stream, null);
    }
});

test('Unexpected facing, device or empty selected video stream is closed before rejection', async () => {
    for (const options of [{ replacementFacing: 'user' }, { replacementFacing: '' },
        { replacementId: 'fixture-wrong-device' }, { replacementEmpty: true }]) {
        const f = fixture({ search: '?camera=0', ...options });
        await assert.rejects(f.camera.initialize(), /camera|rear|device|video/i);
        assert.equal(f.replacement.track.stopped, 1); assert.equal(f.camera.stream, null);
    }
});

test('Enumeration errors close only an opened bootstrap and leave no capture owned', async () => {
    for (const options of [{ enumerationError: new Error('fixture enumeration failed') },
        { labelsHidden: true, secondEnumerationError: new Error('fixture enumeration failed') }]) {
        const f = fixture({ search: '?camera=0', ...options });
        await assert.rejects(f.camera.initialize(), /enumeration/);
        assert.equal(f.requests.length, options.labelsHidden ? 1 : 0);
        assert.equal(f.initial.track.stopped, options.labelsHidden ? 1 : 0);
        assert.equal(f.camera.stream, null);
    }
});

test('Unavailable requested camera after permission closes the bootstrap', async () => {
    const f = fixture({ search: '?camera=0', labelsHidden: true, listed: devices.filter(d => d.label.includes('front')) });
    await assert.rejects(f.camera.initialize(), /camera|rear/i);
    assert.equal(f.requests.length, 1); assert.equal(f.initial.track.stopped, 1);
    assert.equal(f.camera.stream, null);
});

test('Initialization cannot stop or replace an existing owned capture', async () => {
    const f = fixture({ search: '?camera=0' }); f.camera.stream = f.initial; f.camera.running = true;
    await assert.rejects(f.camera.initialize(), /already|capture|initializ/i);
    assert.equal(f.requests.length, 0); assert.ok(!f.events.includes('enumerate'));
    assert.equal(f.camera.stream, f.initial); assert.equal(f.initial.track.stopped, 0);
});

test('Explicit camera selection preserves caller-supplied camera profile usage', async () => {
    const f = fixture({ search: '?camera=0&cameraProfile=%2Fpublic%2Fcalibration.json' });
    await f.camera.initialize();
    assert.equal(f.requests.length, 1); assert.equal(f.camera.stream, f.replacement);
    f.camera.stop();
});
