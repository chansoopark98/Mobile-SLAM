import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';

const source = await fs.readFile(new URL('../../web/js/capture-worker.js', import.meta.url), 'utf8');
const { CaptureWorker, createCaptureWorkerHandler } = await import('data:text/javascript;base64,' + Buffer.from(source).toString('base64'));

function fixture(options = {}) {
    const messages = [], frames = [], accepted = [], errors = [];
    const worker = { terminated: false, postMessage(message) {
        if (options.transferFailure && message.type === 'frame') throw new Error('transfer failed');
        messages.push(message);
    }, terminate() { this.terminated = true; } };
    const video = { value: 17 };
    const client = new CaptureWorker({
        createWorker: () => worker,
        createFrame: (input, init) => {
            if (options.constructorFailure) throw new Error('constructor failed');
            const frame = { value: input.value, timestamp: init.timestamp, closed: 0,
                close() { this.closed++; } };
            frames.push(frame); options.onConstruct?.(frames); return frame;
        },
        onFrame(packet) { accepted.push(packet); options.onFrame?.(packet, client); return true; },
        onError(error) { errors.push(error.message); },
        timeoutMs: options.timeoutMs ?? 2000,
    });
    const geometry = { sourceWidth: 2, sourceHeight: 2, rotation: 'none', cropX: 0, cropY: 0,
        cropWidth: 2, cropHeight: 2, captureWidth: 2, captureHeight: 2, width: 2, height: 2, clientEpoch: 1 };
    const packet = (seq, generation = 1) => ({ video, configuration: {}, geometry: { ...geometry },
        captureGeneration: generation, timing: { timestampS: seq / 10, presentedFrames: seq, physicalExposureVerified: false } });
    const complete = (message = messages.findLast(value => value.type === 'frame'), change = {}) => {
        message.frame.close();
        worker.onmessage({ data: { type: 'frame', identity: message.identity,
            gray: new Uint8Array(4).fill(message.frame.value).buffer,
            timing: { workerDrawReadMs: 1, workerGrayMs: 0.5 }, closedFrames: frames.filter(frame => frame.closed).length,
            ...change } });
    };
    client.start();
    return { client, video, worker, frames, messages, accepted, errors, packet, complete,
        ready: () => worker.onmessage({ data: { type: 'ready' } }) };
}

test('Unready capture falls back before allocating a VideoFrame; first real result validates the path', async () => {
    const f = fixture();
    assert.equal(f.client.capture(f.video, f.packet(1)), false);
    assert.equal(f.frames.length, 0);
    f.ready(); assert.equal(f.client.capture(f.video, f.packet(2)), true);
    assert.equal(f.client.getMetrics().state, 'probing');
    f.video.value = 99; f.complete();
    assert.deepEqual(Array.from(f.accepted[0].gray), [17,17,17,17]);
    assert.equal(f.accepted[0].timing.timestampS, 0.2);
    assert.equal(f.accepted[0].timing.physicalExposureVerified, false);
    assert.equal(f.client.getMetrics().state, 'active');
    await f.client.stop();
});

test('Only one transferred frame and newest pending frame survive busy capture', async () => {
    const f = fixture(); f.ready(); f.client.capture(f.video, f.packet(1));
    const first = f.messages.findLast(message => message.type === 'frame');
    for (let seq = 2; seq <= 64; seq++) {
        f.video.value = seq; f.client.capture(f.video, f.packet(seq));
        assert.equal(f.client.getMetrics().inFlight, 1);
        assert.equal(f.client.getMetrics().pending, 1);
        assert.ok(f.frames.filter(frame => !frame.closed).length <= 2);
    }
    f.complete(first); f.complete();
    assert.deepEqual(f.accepted.map(packet => packet.timing.presentedFrames), [1,64]);
    assert.deepEqual(Array.from(f.accepted[1].gray), [64,64,64,64]);
    assert.equal(f.client.getMetrics().coalesced, 62);
    assert.equal(f.client.getMetrics().scheduled, 64);
    assert.equal(f.client.getMetrics().posted, 2);
    assert.equal(f.client.getMetrics().dropped, 62);
    assert.ok(f.frames.every(frame => frame.closed === 1));
    await f.client.stop();
});

test('Resource bound holds during construction, before capture returns', async () => {
    let peak = 0;
    const f = fixture({ onConstruct: frames => { peak = Math.max(peak, frames.filter(frame => !frame.closed).length); } });
    f.ready();
    try {
        for (let seq = 1; seq <= 10; seq++) f.client.capture(f.video, f.packet(seq));
        assert.equal(peak, 2, 'Old pending ownership must close before allocating the replacement');
    } finally {
        f.client.cancel(); f.complete(); await f.client.stop();
    }
});

test('Reentrant completion callback cannot dispatch two remote frames or revive older pending input', async () => {
    const f = fixture({ onFrame: (packet, client) => {
        if (packet.timing.presentedFrames === 1) client.capture(f.video, f.packet(3));
    } });
    f.ready(); f.client.capture(f.video,f.packet(1)); f.client.capture(f.video,f.packet(2));
    try {
        f.complete(f.messages.find(message => message.type === 'frame'));
        assert.deepEqual(f.messages.filter(message => message.type === 'frame').map(message => message.identity.presentedFrames), [1,3]);
        assert.equal(f.client.getMetrics().inFlight, 1);
        assert.equal(f.frames[1].closed, 1, 'Reentrant newest frame supersedes the prior pending input');
        f.complete();
        assert.deepEqual(f.accepted.map(packet => packet.timing.presentedFrames), [1,3]);
    } finally { f.client.cancel(); if(f.client.inFlight)f.complete(); await f.client.stop(); }
});

test('Cancel closes pending ownership and rejects old generation while restart remains bounded', async () => {
    const f = fixture(); f.ready(); f.client.capture(f.video, f.packet(1));
    const first = f.messages.findLast(message => message.type === 'frame');
    f.client.capture(f.video, f.packet(2)); f.client.cancel();
    assert.equal(f.frames[1].closed, 1);
    f.client.capture(f.video, f.packet(3,2)); f.complete(first); f.complete();
    assert.deepEqual(f.accepted.map(packet => packet.captureGeneration), [2]);
    assert.ok(f.frames.every(frame => frame.closed === 1));
    assert.equal(f.client.getMetrics().dropped, 2);
    await f.client.stop();
});

test('A stale duplicate cannot finish a different request; mismatched geometry is rejected', async () => {
    const f = fixture(); f.ready(); f.client.capture(f.video, f.packet(1));
    const first = f.messages.findLast(message => message.type === 'frame');
    f.client.capture(f.video, f.packet(2)); f.complete(first);
    f.worker.onmessage({ data: { type: 'frame', identity: first.identity } });
    assert.equal(f.client.getMetrics().inFlight, 1);
    const second = f.messages.findLast(message => message.type === 'frame');
    f.complete(second, { identity: { ...second.identity, geometry: { ...second.identity.geometry, cropX: 1 } } });
    assert.equal(f.accepted.length, 1);
    assert.equal(f.client.getMetrics().state, 'failed');
    assert.equal(f.client.capture(f.video, f.packet(3)), false);
    await f.client.stop();
});

test('Post-transfer error drops that request and pending frame; fallback is for a new callback', async () => {
    const f = fixture(); f.ready(); f.client.capture(f.video, f.packet(1));
    const first = f.messages.findLast(message => message.type === 'frame');
    f.client.capture(f.video, f.packet(2)); first.frame.close();
    f.worker.onmessage({ data: { type: 'error', identity: first.identity, message: 'draw failed', closedFrames: 1 } });
    assert.equal(f.accepted.length, 0); assert.equal(f.frames.length, 2);
    assert.ok(f.frames.every(frame => frame.closed === 1));
    assert.equal(f.client.getMetrics().dropped, 2);
    assert.equal(f.client.capture(f.video, f.packet(3)), false);
    assert.equal(f.frames.length, 2, 'Failure cannot reread a video or manufacture a replacement old frame');
    await f.client.stop();
});

test('Constructor failure is pre-transfer; transfer failure consumes and closes the failed request', async () => {
    const a = fixture({ constructorFailure: true }); a.ready();
    assert.equal(a.client.capture(a.video, a.packet(1)), false);
    assert.equal(a.client.getMetrics().scheduled, 0); await a.client.stop();
    const b = fixture({ transferFailure: true }); b.ready();
    assert.equal(b.client.capture(b.video, b.packet(1)), true);
    assert.equal(b.frames[0].closed, 1);
    assert.equal(b.client.getMetrics().dropped, 1); await b.client.stop();
});

test('Stop drains the owned frame before termination and rejects its result', async () => {
    const f = fixture(); f.ready(); f.client.capture(f.video, f.packet(1));
    const stopped = f.client.stop();
    assert.equal(f.worker.terminated, false);
    f.complete(); await stopped;
    assert.equal(f.worker.terminated, true);
    assert.equal(f.accepted.length, 0);
    assert.ok(f.frames.every(frame => frame.closed === 1));
    assert.equal(f.client.getMetrics().releasedByTermination, 0);
});

test('A hung worker has a bounded stop and reports realm release separately from close ACK', async () => {
    const f = fixture({ timeoutMs: 10 }); f.ready(); f.client.capture(f.video, f.packet(1));
    await f.client.stop();
    assert.equal(f.worker.terminated, true);
    assert.equal(f.client.getMetrics().releasedByTermination, 1);
    assert.equal(f.client.getMetrics().closedRemote, 0);
    assert.equal(f.client.getMetrics().inFlight, 0);
});

test('Stop/start waits for drain and ignores messages from the terminated Worker realm', async () => {
    const workers = [], accepted = [];
    const client = new CaptureWorker({ createWorker: () => {
        const worker = { postMessage() {}, terminate() { this.terminated = true; } };
        workers.push(worker); return worker;
    }, createFrame: (video,init) => ({ timestamp:init.timestamp,close(){} }), onFrame: packet => accepted.push(packet) });
    client.start(); workers[0].onmessage({data:{type:'ready'}});
    client.capture({}, { configuration:{},captureGeneration:1,
        geometry:{cropWidth:2,cropHeight:2,clientEpoch:1},timing:{timestampS:0.1,presentedFrames:1} });
    const oldIdentity = client.inFlight.identity;
    const stopped = client.stop(); client.start();
    assert.equal(workers.length,1,'Restart cannot allocate another Worker before the old owned frame closes');
    workers[0].onmessage({data:{type:'frame',identity:oldIdentity,gray:new Uint8Array(4).buffer,
        timing:{workerDrawReadMs:1,workerGrayMs:1},closedFrames:1}});
    await stopped;
    assert.equal(workers.length,2); assert.equal(workers[0].terminated,true);
    workers[0].onmessage({data:{type:'ready'}});
    workers[0].onerror({message:'late old error'});
    assert.equal(client.getMetrics().state,'starting');
    assert.equal(client.getMetrics().errors,0);
    workers[1].onmessage({data:{type:'ready'}});
    assert.equal(client.getMetrics().state,'ready');
    assert.equal(accepted.length,0); await client.stop();
});

test('Worker closes a transferred frame when draw or timestamp validation fails', () => {
    const replies = [];
    class Canvas {
        getContext() { return { getImageData: () => ({ data: new Uint8ClampedArray(16) }),
            save() {}, restore() {}, translate() {}, rotate() {}, drawImage() { throw new Error('draw failed'); } }; }
    }
    const handler = createCaptureWorkerHandler(message => replies.push(message), Canvas);
    handler({ type: 'init' });
    for (const timestamp of [100000, 999999]) {
        const frame = { timestamp, displayWidth: 2, displayHeight: 2, closed: 0, close() { this.closed++; } };
        handler({ type: 'frame', frame, identity: { seq: 1, timestampUs: 100000, geometry: {
            sourceWidth: 2, sourceHeight: 2, rotation: 'none', cropX: 0, cropY: 0, cropWidth: 2, cropHeight: 2 } } });
        assert.equal(frame.closed, 1);
        assert.equal(replies.at(-1).type, 'error');
    }
    assert.equal(replies.at(-1).closedFrames, 2);
});
