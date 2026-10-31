import test from 'node:test';
import assert from 'node:assert/strict';
import { fixture } from './frame-scheduler-fixture.mjs';

const waitReady = async (s) => {
  for (let i = 0; i < 100 && !s.scheduler().ready; i++) await new Promise((r) => setTimeout(r, 5));
  assert.equal(s.scheduler().ready, true, JSON.stringify(s.scheduler()));
};

test('D4 committed display survives a CPU command until a post-render fence signals', async () => {
  await fixture(true, async (s) => {
    const baseBytes = s.scheduler().memory.controlledBytes;
    s.stage(s.frame(0)); await waitReady(s); s.cpu(s.frame(1));
    s.stage(s.frame(1)); await waitReady(s);
    const answer = s.cpu(s.frame(2));
    assert.ok(answer?.realityTexture);
    const displayed = answer.realityTexture;

    // A command can arrive after CPU publication and before this frame's render.
    s.ctrl.recenter();
    assert.equal(s.deletedTextures.has(displayed), false);
    assert.ok(s.scheduler().memory.controlledBytes > baseBytes,
      'old display allocation remains in the shared 64 MiB ledger');

    s.drawFenceReady(false);
    s.stage(s.frame(2)); // post-render: insert the old pool's retirement fence
    assert.equal(s.deletedTextures.has(displayed), false);
    s.drawFenceReady(true);
    s.stage({ ...s.frame(3), repeatFrame: true });
    assert.equal(s.deletedTextures.has(displayed), true,
      'signaled post-render fence permits old GL allocation deletion');
    assert.ok(s.scheduler().memory.peakBytes <= 64 * 1024 * 1024);
  });
});

test('D4 old and new epochs share the four-snapshot limit while fences wait', async () => {
  await fixture(true, async (s) => {
    s.stage(s.frame(0)); await waitReady(s); s.cpu(s.frame(1));
    s.stage(s.frame(1)); await waitReady(s); assert.ok(s.cpu(s.frame(2))?.realityTexture);
    s.drawFenceReady(false); s.ctrl.recenter();

    // Let each new task read after its GPU callback; old render fences stay unsignaled.
    s.stage(s.frame(2)); await waitReady(s); s.cpu(s.frame(3));
    s.stage(s.frame(3)); await waitReady(s);
    assert.ok(s.cpu(s.frame(4))?.realityTexture);
    assert.equal(s.scheduler().snapshots.uniqueSlots, 4);
    s.ctrl.recenter();
    const before = s.scheduler().counters.droppedCapacity;
    s.stage(s.frame(4));
    assert.equal(s.scheduler().counters.droppedCapacity, before + 1);
    assert.equal(s.scheduler().snapshots.uniqueSlots, 4);
    assert.ok(s.scheduler().memory.controlledBytes <= 64 * 1024 * 1024);

    s.drawFenceReady(true);
    s.stage({ ...s.frame(5), repeatFrame: true });
    assert.equal(s.scheduler().snapshots.uniqueSlots, 0);
    s.stage(s.frame(5)); await waitReady(s);
    assert.equal(s.scheduler().counters.droppedCapacity, before + 1);
  });
});

test('D4 unsignaled retire fences trigger a barrier on the 32nd blocked GPU tick', async () => {
  await fixture(true, async (s) => {
    s.drawFenceReady(false);
    for (let epoch = 0; epoch < 2; epoch++) {
      s.stage(s.frame(2 * epoch)); await waitReady(s); s.cpu(s.frame(2 * epoch + 1));
      s.stage(s.frame(2 * epoch + 1)); await waitReady(s); s.cpu(s.frame(2 * epoch + 2));
      s.ctrl.recenter();
    }
    assert.equal(s.scheduler().snapshots.uniqueSlots, 4);
    assert.ok(s.scheduler().snapshots.pendingRetire > 0);
    for (let tick = 1; tick <= 31; tick++) {
      s.stage(s.frame(3 + tick));
      assert.equal(s.scheduler().counters.barrierResets, 2, `premature retire reset at tick ${tick}`);
    }
    s.stage(s.frame(35));
    assert.equal(s.scheduler().counters.watchdogResets, 1);
    assert.equal(s.scheduler().counters.barrierResets, 3);
    assert.ok(s.scheduler().failures.some((failure) => failure.where === 'scheduler-retire-watchdog'));
    assert.equal(s.scheduler().pending, false);
  });
});

for (const finishFails of [false, true]) test(`D4 stop releases logical ledger when draw finish ${finishFails ? 'fails' : 'succeeds'}`, async () => {
  await fixture(true, async (s) => {
    s.stage(s.frame(0)); await waitReady(s); s.cpu(s.frame(1));
    s.stage(s.frame(1)); await waitReady(s);
    const displayed = s.cpu(s.frame(2)).realityTexture;
    s.drawFenceReady(false);
    s.drawFinishFail(finishFails);
    s.ctrl.recenter();
    assert.ok(s.scheduler().memory.controlledBytes > 0);
    s.pm.onSessionDetach({});
    assert.equal(s.scheduler().memory.controlledBytes, 0);
    assert.equal(s.deletedTextures.has(displayed), true);
    assert.equal(s.scheduler().counters.errors >= 1, finishFails,
      'failed finish is recorded; successful finish subsumes an unsignaled fence');
  });
});

test('D4 orientation callback rebuild names the native predecessor it discards', async () => {
  await fixture(true, async (s) => {
    s.stage(s.frame(0)); await waitReady(s); s.cpu(s.frame(1));
    s.sensor(1020); s.setNow(1033);
    s.stage(s.frame(1)); await waitReady(s); assert.ok(s.cpu(s.frame(2))?.realityTexture);
    const oldEpoch = s.scheduler().epoch.sessionEpoch;
    s.pm.onDeviceOrientationChange({ videoWidth: 240, videoHeight: 320, orientation: 0,
      computeCtx: s.gl, framework: s.framework });
    const after = s.scheduler();
    assert.notEqual(after.epoch.sessionEpoch, oldEpoch);
    assert.equal(after.counters.lifecycleResets, 1);
    assert.equal(after.counters.discardedAdmissions, 1);
    assert.deepEqual(after.discards.map(({ reason, captureSeq, sourceVideoTimeS, acceptedMotionSeqs }) =>
      ({ reason, captureSeq, sourceVideoTimeS, acceptedMotionSeqs })),
    [{ reason: 'orientation', captureSeq: 1, sourceVideoTimeS: s.frame(1).videoTime,
      acceptedMotionSeqs: [1] }]);
    after.discards[0].acceptedMotionSeqs[0] = -1;
    assert.deepEqual(s.scheduler().discards[0].acceptedMotionSeqs, [1]);
  });
});

test('D4 accepted IMU before a failed native admission is discard-receipted once', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.setNow(1000);
    s.hooks.pushDeviceOrientation = () => { throw new Error('injected after accepted IMU'); };
    s.stage(s.frame(0));
    for (let i = 0; i < 100 && !s.scheduler().counters.errors; i++) {
      await new Promise((r) => setTimeout(r, 5));
    }
    const after = s.scheduler();
    assert.equal(after.counters.errors, 1);
    assert.equal(after.counters.preAdmissionDiscards, 1);
    assert.deepEqual(after.discards.map(({ kind, captureSeq, acceptedMotionSeqs }) =>
      ({ kind, captureSeq, acceptedMotionSeqs })),
    [{ kind: 'unknown-native-admission', captureSeq: 0, acceptedMotionSeqs: [1] }]);
    const delivery = s.snapshot().sensors;
    assert.equal(delivery.pushed, 1);
    assert.equal(delivery.dropped, 0);
    assert.equal(delivery.duplicateSettlements, 0);
    assert.equal(delivery.accountingError, 0);
  });
});

test('D4 accepted IMU discard receipt is bounded and reports overflow', async () => {
  await fixture(true, async (s) => {
    for (let i = 0; i < 130; i++) s.sensor(900 + i);
    s.setNow(1030);
    s.hooks.pushDeviceOrientation = () => { throw new Error('injected after accepted burst'); };
    s.stage(s.frame(0));
    for (let i = 0; i < 100 && !s.scheduler().counters.errors; i++) {
      await new Promise((r) => setTimeout(r, 5));
    }
    const after = s.scheduler();
    const discard = after.discards.find((e) => e.kind === 'unknown-native-admission');
    assert.equal(discard.acceptedMotionSeqs.length, 128);
    assert.deepEqual([discard.acceptedMotionSeqs[0], discard.acceptedMotionSeqs.at(-1)], [1, 128]);
    assert.equal(discard.acceptedMotionSeqOverflow, 2);
    assert.equal(after.counters.acceptedMotionSeqOverflow, 2);
    assert.equal(s.snapshot().sensors.pushed, 130);
    assert.equal(s.snapshot().sensors.accountingError, 0);
  });
});
