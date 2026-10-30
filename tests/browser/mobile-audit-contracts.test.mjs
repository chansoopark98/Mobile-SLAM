import test from 'node:test';
import assert from 'node:assert/strict';
import { createRecorder, poseDelta } from '../../web/js/audit-mobile.js';

test('Bounded recorder stops accepting samples after stop and reports discarded rows', () => {
    let now = 1000;
    const recorder = createRecorder({ now: () => now, maxSamples: 2 });
    recorder.start({ device: 'mock' });
    recorder.push('imu', { timestamp: 1 });
    now += 10;
    recorder.push('imu', { timestamp: 2 });
    recorder.push('imu', { timestamp: 3 });
    recorder.stop('manual');
    recorder.push('imu', { timestamp: 4 });
    const report = recorder.export();
    assert.equal(report.samples.imu.length, 2);
    assert.equal(report.dropped.imu, 1);
    assert.equal(report.stopReason, 'manual');
});

test('Recording duration rejects trailing samples at its deadline', () => {
    let now = 0;
    const recorder = createRecorder({ now: () => now, maxDurationMs: 100 });
    recorder.start();
    now = 101;
    recorder.push('frames', { accepted: true });
    assert.equal(recorder.export().samples.frames?.length || 0, 0);
    assert.equal(recorder.export().stopReason, 'duration_limit');
});

test('Pose jump keeps translation metres and rotation degrees independently', () => {
    const a = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1];
    const b = [0, -1, 0, 3, 1, 0, 0, 4, 0, 0, 1, 0, 0, 0, 0, 1];
    const jump = poseDelta(a, b);
    assert.equal(jump.translationM, 5);
    assert.ok(Math.abs(jump.rotationDeg - 90) < 1e-10);
    assert.equal(poseDelta(a, [NaN, ...a.slice(1)]), null);
});

test('Recorder copies numeric samples before transferable buffers detach', () => {
    const recorder = createRecorder();
    recorder.start();
    const sample = { values: [1, 2, 3] };
    recorder.push('imu', sample);
    sample.values[0] = 99;
    assert.equal(recorder.export().samples.imu[0].values[0], 1);
});
