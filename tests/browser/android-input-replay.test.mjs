import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs/promises';
import os from 'node:os';
import path from 'node:path';
import { parsePGM, validateRecording, summarizeResults, imuBatchRanges, main } from '../../scripts/dev/replay-android-input.mjs';

test('P5 parser preserves whitespace, hash and NUL valued first pixels', () => {
    const gray = Buffer.from([10, 13, 32, 35, 0, 255]);
    for (const header of ['P5\n3 2\n255\n', 'P5\r\n# retained comment\r\n3 2\r\n255\r\n']) {
        assert.deepEqual(parsePGM(Buffer.concat([Buffer.from(header), gray]), 3, 2), gray);
    }
});

test('P5 parser rejects incorrect dimensions, format, bit depth and payload length', () => {
    for (const header of ['P2\n3 2\n255\n', 'P5\n2 3\n255\n', 'P5\n3 2\n65535\n']) {
        assert.throws(() => parsePGM(Buffer.concat([Buffer.from(header), Buffer.alloc(6)]), 3, 2));
    }
    for (const count of [5, 7]) assert.throws(() => parsePGM(Buffer.concat([Buffer.from('P5\n3 2\n255\n'), Buffer.alloc(count)]), 3, 2), /payload/);
});

function recording() {
    return { schema: 'mobile-slam-android-input-v1', completed: true, recording: {
        width: 2, height: 2, config: { width: 2, height: 2 }, profile: 'mobile_default',
        frames: [1, 2].map((timestamp, index) => ({ index, timestamp, file: `${timestamp}.pgm`, sha256: 'a'.repeat(64) })),
        imu: [0.9, 1.1, 1.9, 2.1].map(timestamp => [timestamp, 0, 0, 9.81, 0, 0, 0]), counters: {},
    } };
}

test('Prefix keeps original double timestamps and requires IMU bracket', () => {
    const source = recording(), parsed = validateRecording(source, 1);
    assert.equal(parsed.frames.length, 1); assert.equal(source.recording.frames.length, 2);
    assert.equal(parsed.frames[0].timestamp, 1);
    source.recording.imu.shift(); assert.throws(() => validateRecording(source), /bracket/);
    const future = recording(); future.recording.imu.pop(); assert.throws(() => validateRecording(future), /bracket/);
    const endpoint = recording(); endpoint.recording.imu.at(-1)[0] = 2;
    assert.equal(validateRecording(endpoint).frames.length, 2, 'Exact terminal endpoint satisfies the Worker bracket contract');
});

test('Invalid, nonmonotonic and overflowing records fail before replay', () => {
    for (const change of [
        record => { record.recording.imu[1][0] = 0.9; },
        record => { record.recording.imu[1][3] = NaN; },
        record => { record.recording.frames[1].timestamp = 1; },
        record => { record.recording.frames[0].file = '../other.pgm'; },
        record => { record.recording.counters.imuOverflow = 1; },
        record => { record.completed = false; },
        record => { record.recording.profile = 'unknown'; },
    ]) { const source = recording(); change(source); assert.throws(() => validateRecording(source)); }
});

test('Streamed recordings must match every accepted index without missing saved images', () => {
    const source = recording();
    source.recording.counters.acceptedFrames = 2;
    source.recording.acceptedFrameMetadata = source.recording.frames.map(frame => ({ index: frame.index, timestamp: frame.timestamp, payloadStatus: 'saved' }));
    assert.equal(validateRecording(source, 1).frames.length, 1);
    source.recording.acceptedFrameMetadata[1].payloadStatus = 'queue_overflow';
    assert.throws(() => validateRecording(source, 1), /saved payloads/);
    const legacy = recording(); legacy.recording.frames[1].index = 2;
    assert.throws(() => validateRecording(legacy), /contiguous/);
});

test('Explicit long prefixes retain all prior frames within frame and raw-byte budgets', () => {
    const source = recording();
    source.recording.width = source.recording.config.width = 480;
    source.recording.height = source.recording.config.height = 640;
    source.recording.frames = Array.from({ length: 2000 }, (_, index) => ({
        index, timestamp: index + 1, file: `${index}.pgm`, sha256: 'a'.repeat(64),
    }));
    source.recording.imu = [0.9, 2001].map(timestamp => [timestamp, 0, 0, 9.81, 0, 0, 0]);
    const prefix = validateRecording(source, 1200).frames;
    assert.equal(prefix.length, 1200);
    assert.equal(prefix[0].index, 0);
    assert.equal(prefix.at(-1).index, 1199);
    assert.equal(validateRecording(source).frames.length, 300, 'Default remains the preserved prefix');
    assert.throws(() => validateRecording(source, 1800), /512 MiB budget/);
    assert.throws(() => validateRecording(source, 2001), /1\.\.2000/);
    source.recording.width = source.recording.config.width = 2;
    source.recording.height = source.recording.config.height = 2;
    assert.equal(validateRecording(source, 2000).frames.length, 2000);
});

test('Pose summaries exclude invalid poses and jumps across engine resets', () => {
    const pose = x => [1,0,0,x,0,1,0,0,0,0,1,0,0,0,0,1];
    const row = (x, epoch = 1) => ({ pose: pose(x), poseTimestamp: x, poseFresh: true, poseValid: true, engineEpoch: epoch, initialized: true, reason: 'tracking' });
    const rows = [row(0), row(3), { ...row(99), poseFresh: false }, row(100, 2)];
    const summary = summarizeResults(rows);
    assert.equal(summary.completedFrames, 4); assert.equal(summary.freshPoseFrames, 3);
    assert.equal(summary.freshPoseCoverage, 0.75); assert.equal(summary.maxPositionNormM, 100);
    assert.equal(summary.maxConsecutivePublishedTranslationM, 3); assert.equal(summary.observedEpochTransitions, 1);
    assert.equal(summarizeResults([]).maxPositionNormM, null);
});

test('Multiple images between IMU samples reuse the already submitted future bracket', () => {
    const imu = [0, 1, 2, 3].map(timestamp => [timestamp, 0, 0, 9.81, 0, 0, 0]);
    const frames = [0.1, 0.2, 0.9, 1, 1.1, 2.5].map(timestamp => ({ timestamp }));
    assert.deepEqual(imuBatchRanges(frames, imu), [
        { start: 0, count: 2 }, { start: 2, count: 0 }, { start: 2, count: 0 },
        { start: 2, count: 1 }, { start: 3, count: 0 }, { start: 3, count: 1 },
    ]);
});

test('Output inside a preserved input tree is rejected before creating directories', async () => {
    const temporary = await fs.mkdtemp(path.join(os.tmpdir(), 'android-replay-preserve-'));
    try {
        await fs.writeFile(path.join(temporary, 'record.json'), '{}');
        await fs.writeFile(path.join(temporary, 'loader.js'), '');
        await assert.rejects(main(['--record', path.join(temporary, 'record.json'), '--input-dir', temporary,
            '--loader', path.join(temporary, 'loader.js'), '--output', path.join(temporary, 'new-parent/output')]), /outside preserved/);
        assert.deepEqual((await fs.readdir(temporary)).sort(), ['loader.js', 'record.json']);
    } finally { await fs.rm(temporary, { recursive: true, force: true }); }
});
