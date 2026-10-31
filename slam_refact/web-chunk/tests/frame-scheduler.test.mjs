import test from 'node:test';
import assert from 'node:assert/strict';
import { fixture } from './frame-scheduler-fixture.mjs';
import { OwnedDisplaySnapshots } from '../gr8-compute-pyramid.js';
const turn = () => new Promise((r) => setTimeout(r, 5));
const wait = async (s) => { for (let i = 0; i < 100 && !s.scheduler().ready && !s.scheduler().counters.errors; i++) await turn();
  assert.equal(s.scheduler().counters.errors, 0, JSON.stringify(s.scheduler())); assert.equal(s.scheduler().ready, true); };
const stage = async (s, k) => { s.sensor(990 + 33 * k); s.setNow(1000 + 33 * k); s.stage(s.frame(k)); await wait(s); };

test('D3 real task priming ACK allows next admission while queries stay guarded', async () => {
  await fixture(true, async (s) => {
    s.setNow(1000); s.stage(s.frame(0));
    assert.equal(s.poses.length, 0, 'GPU callback does not call native');
    assert.equal(s.cpu(s.frame(1)), null, 'no extra CPU work before task');
    await wait(s); assert.equal(s.scheduler().readyKind, 'nativePriming');
    assert.equal(s.cpu(s.frame(1)), null); assert.equal(s.scheduler().counters.primingAcked, 1);
    assert.equal(s.scheduler().pending, false); assert.deepEqual(s.ctrl.hitTest(0.5, 0.5), []);
    await stage(s, 1); const out = s.cpu(s.frame(2)); assert.ok(out?.realityTexture);
    assert.equal(s.latest().sourceFrameIndex, 0); assert.equal(s.latest().fresh, true);
    assert.equal(s.latest().newEngineResponse, true); assert.equal(s.latest().engineFresh, true);
    assert.equal(s.snapshot().sourceMetadata.entries, 2);
    assert.equal(s.scheduler().queryBusyReason, null);
  });
});

test('D1 busy frames leave new IMU queued; one packet keeps its original cutoff', async () => {
  await fixture(true, async (s) => {
    s.readyFence(false); s.sensor(990); s.setNow(1000); s.stage(s.frame(0));
    s.sensor(1005); s.stage(s.frame(1)); await turn(); await turn();
    assert.equal(s.reads(), 0); assert.equal(s.scheduler().pollsThisTick, 2);
    assert.equal(s.snapshot().sensors.pending.queued, 1); assert.equal(s.snapshot().sensors.pending.staged, 1);
    s.readyFence(true); s.stage({ ...s.frame(2), repeatFrame: true }); await wait(s);
    assert.equal(s.calls.length, 1); assert.equal(s.calls[0].timestampNanos, 990e6);
    s.cpu(s.frame(2)); await stage(s, 3); s.cpu(s.frame(4));
    assert.equal(s.snapshot().sensors.accountingError, 0); assert.equal(s.snapshot().sensors.duplicateSettlements, 0);
    assert.equal(s.calls.filter((x) => x.timestampNanos === 1005e6).length, 1);
    assert.equal(s.scheduler().counters.droppedBusy, 1);
  });
});

test('D4 first getter failure discards receipt, rebuilds once and never replays accepted IMU', async () => {
  await fixture(true, async (s) => {
    await stage(s, 0); s.cpu(s.frame(1));
    s.failWorldPoints(); s.sensor(1020); s.stage(s.frame(1)); await turn(); await turn();
    assert.equal(s.scheduler().counters.errors, 1); assert.equal(s.scheduler().counters.barrierResets, 1);
    assert.equal(s.scheduler().ready, false); assert.equal(s.cpu(s.frame(2)), null);
    const count = s.calls.filter((e) => e.timestampNanos === 1020e6).length;
    s.stage(s.frame(2)); await turn(); await turn(); s.cpu(s.frame(3));
    assert.equal(s.calls.filter((e) => e.timestampNanos === 1020e6).length, count);
    assert.equal(s.snapshot().sensors.accountingError, 0);
  });
});

test('D4 listener recenter invalidates old publication with no stale tail commit', async () => {
  await fixture(true, async (s) => {
    await stage(s, 0); s.cpu(s.frame(1)); await stage(s, 1);
    const epoch = s.scheduler().epoch.sessionEpoch;
    s.hooks.dispatch = (name) => { if (name === 'trackingstatus') { delete s.hooks.dispatch; s.ctrl.recenter(); } };
    assert.equal(s.cpu(s.frame(2)), null);
    assert.notEqual(s.scheduler().epoch.sessionEpoch, epoch);
    assert.equal(s.scheduler().counters.publications, 0); assert.equal(s.scheduler().counters.barrierResets, 1);
    assert.equal(s.scheduler().queryBusyReason, 'await-first-publication');
  });
});

test('D4 native callback query is blocked and mutator cannot delete an executing glue', async () => {
  await fixture(true, async (s) => {
    let queries = 0, deletes = 0;
    s.hooks.xrHitTest = () => queries++; s.hooks.delete = () => deletes++;
    s.hooks.pushDeviceOrientation = () => {
      assert.deepEqual(s.ctrl.hitTest(0.1, 0.1), []);
      assert.throws(() => s.ctrl.recenter(), /busy/); assert.equal(deletes, 0);
      delete s.hooks.pushDeviceOrientation;
    };
    s.stage(s.frame(0)); await turn(); await turn();
    assert.equal(queries, 0); assert.equal(s.scheduler().counters.barrierResets, 1);
    assert.equal(deletes, 1); assert.equal(s.scheduler().counters.errors, 1);
  });
});

test('D4 native-advanced hitTest miss is counted without querying unpublished native state', async () => {
  await fixture(true, async (s) => {
    let nativeQueries = 0, busyReason = null;
    s.hooks.xrHitTest = () => nativeQueries++;
    s.hooks.pushDeviceOrientation = () => {
      delete s.hooks.pushDeviceOrientation;
      busyReason = s.scheduler().queryBusyReason;
      assert.deepEqual(s.ctrl.hitTest(0.1, 0.2), []);
    };
    s.stage(s.frame(0)); await wait(s);
    assert.equal(busyReason, 'native-call');
    assert.equal(nativeQueries, 0);
    assert.equal(s.scheduler().counters.hitTests, 1);
    assert.equal(s.scheduler().counters.hitMisses, 1);
    assert.equal(s.scheduler().counters.errors, 0);
  });
});

test('D4 startup internal FIFO clear is followed by ACK and next processed source', async () => {
  await fixture(true, async (s) => {
    const kinds = [];
    for (let k = 0; k < 12; k++) { await stage(s, k); kinds.push(s.scheduler().readyKind); s.cpu(s.frame(k + 1)); }
    assert.equal(kinds[0], 'nativePriming'); assert.equal(kinds.filter((x) => x === 'nativePriming').length, 2);
    assert.equal(s.scheduler().counters.startupDiscardedAdmissions, 1);
    assert.equal(s.scheduler().counters.primingAcked, 2); assert.equal(s.scheduler().queryBusyReason, null);
    assert.equal(s.scheduler().counters.errors, 0);
  });
});

test('D4 queued old epoch task never advances rebuilt native session and noop configure does not rebuild', async () => {
  await fixture(true, async (s) => {
    s.stage(s.frame(0)); s.ctrl.configure({}); assert.equal(s.scheduler().counters.barrierResets, 0);
    s.ctrl.configure({ enableLighting: true }); const epoch = s.scheduler().epoch.sessionEpoch;
    await turn(); await turn(); assert.equal(s.poses.length, 0); assert.equal(s.scheduler().epoch.sessionEpoch, epoch);
    assert.equal(s.scheduler().pending, false); assert.equal(s.scheduler().counters.barrierResets, 1);
  });
});

for (const delay of [0, 20, 50, 100, 250]) test(`D2 ${delay}ms task/fence delay keeps reads nonblocking and cutoff fixed`, async () => {
  await fixture(true, async (s) => {
    s.readyFence(false); s.sensor(980); s.setNow(1000); s.stage(s.frame(0));
    await new Promise((r) => setTimeout(r, delay)); await turn();
    assert.equal(s.reads(), 0); assert.equal(s.scheduler().pollsThisTick, 2);
    s.sensor(1010); s.readyFence(true); s.stage({ ...s.frame(1), repeatFrame: true });
    await wait(s); assert.equal(s.calls.length, 1); assert.equal(s.calls[0].timestampNanos, 980e6);
    assert.equal(s.snapshot().sensors.pending.queued, 1); assert.equal(s.snapshot().sensors.accountingError, 0);
    s.cpu(s.frame(1));
  });
});

test('D4 read failure before native carries raw batch without dropped/duplicate settlement', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.failRead(); s.stage(s.frame(0)); await turn(); await turn();
    assert.equal(s.scheduler().counters.errors, 1); assert.equal(s.calls.length, 0);
    let d = s.snapshot().sensors;
    assert.equal(d.received, 1); assert.equal(d.dropped, 0); assert.equal(d.pending.queued, 1); assert.equal(d.duplicateSettlements, 0);
    s.stage(s.frame(1)); await turn(); await turn(); s.cpu(s.frame(2));
    d = s.snapshot().sensors; assert.equal(d.pushed, 1); assert.equal(d.dropped, 0); assert.equal(d.accountingError, 0); assert.equal(d.duplicateSettlements, 0);
  });
});

test('D4 stage failure carries batch and invalid command keeps prior JS mutation order', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.failStage(); s.stage(s.frame(0));
    assert.equal(s.snapshot().sensors.pending.queued, 1); assert.equal(s.snapshot().sensors.dropped, 0);
    const barriers = s.scheduler().counters.barrierResets;
    assert.throws(() => s.ctrl.configure({ scale: 'absolute' }), /Scale can only/);
    assert.equal(s.scheduler().counters.barrierResets, barriers, 'invalid no-mutation command does not reset');
    assert.throws(() => s.ctrl.configure({ enableLighting: true, scale: 'absolute' }), /Scale can only/);
    assert.equal(s.scheduler().counters.barrierResets, barriers, 'unreachable late lighting does not cause a reset');
    const configurations = []; s.hooks.configure = (args) => configurations.push({ ...args });
    assert.throws(() => s.ctrl.configure({ enableWorldPoints: true, scale: 'absolute' }), /Scale can only/);
    assert.equal(s.scheduler().counters.barrierResets, barriers + 2, 'changed reachable world-point prefix is preserved through error recovery');
    assert.equal(configurations.at(-1).enableWorldPoints, true);
    assert.equal(configurations.at(-1).enableLighting, false);
    assert.equal(globalThis.__slamRefactChunk.session.glueConfig.captureWidth, 240);
    s.stage(s.frame(1)); await turn(); await turn(); s.cpu(s.frame(2));
    assert.equal(s.calls.filter((e) => e.timestampNanos === 990e6).length, 1);
    assert.equal(s.snapshot().sensors.accountingError, 0);
  });
});

test('D4 context loss invalidates a busy task and releases all logical payloads', async () => {
  await fixture(true, async (s) => {
    s.readyFence(false); s.stage(s.frame(0)); await turn();
    s.loseContext(); assert.doesNotThrow(() => s.stage(s.frame(1))); await turn();
    assert.equal(s.poses.length, 0); assert.equal(s.scheduler().memory.controlledBytes, 0);
    assert.equal(s.scheduler().pending, false); assert.equal(s.scheduler().ready, false);
    assert.equal(s.scheduler().queryBusyReason, 'no-session'); assert.equal(s.cpu(s.frame(2)), null);
  });
});

test('D4 command-native callback mutator cannot delete the executing glue', async () => {
  await fixture(true, async (s) => {
    let count = 0, inside = false, deletedInside = 0;
    s.hooks.delete = () => { if (inside) deletedInside++; };
    s.hooks.configure = () => {
      if (++count === 2) { inside = true; try { s.ctrl.recenter(); } finally { inside = false; } }
    };
    assert.throws(() => s.ctrl.configure({ enableLighting: true }), /busy/);
    assert.equal(deletedInside, 0); assert.equal(s.scheduler().queryBusyReason, 'await-first-publication');
  });
});

for (const path of ['cpu', 'query']) test(`D4 draw-context-only loss invalidates committed ${path} before GPU tick`, async () => {
  await fixture(true, async (s) => {
    await stage(s, 0); s.cpu(s.frame(1)); await stage(s, 1); assert.ok(s.cpu(s.frame(2)));
    let queries = 0; s.hooks.xrHitTest = () => queries++; s.loseDraw();
    if (path === 'cpu') assert.equal(s.cpu(s.frame(3)), null);
    else assert.deepEqual(s.ctrl.hitTest(0.5, 0.5), []);
    assert.equal(queries, 0); assert.equal(s.scheduler().memory.controlledBytes, 0);
    assert.equal(s.scheduler().queryBusyReason, 'no-session');
  });
});

test('D4 throwing IMU push stays uncertain and never gets accepted again', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.sensor(995); s.failPush(); s.stage(s.frame(0)); await turn(); await turn();
    let d = s.snapshot().sensors; assert.equal(d.uncertainPushes, 1); assert.equal(d.dropped, 1); assert.equal(d.pushed, 0);
    s.stage(s.frame(1)); await turn(); await turn(); s.cpu(s.frame(2));
    d = s.snapshot().sensors; assert.equal(d.uncertainPushes, 1); assert.equal(d.pushed, 0);
    assert.equal(d.accountingError, 0); assert.equal(d.duplicateSettlements, 0);
  });
});

test('D4 dispatch failure is discarded once and never retried on a repeat callback', async () => {
  await fixture(true, async (s) => {
    await stage(s, 0); s.cpu(s.frame(1)); await stage(s, 1); s.failDispatch();
    assert.equal(s.cpu(s.frame(2)), null); const d = s.scheduler();
    assert.equal(d.counters.publications, 0); assert.equal(d.counters.barrierResets, 1);
    assert.equal(d.counters.partialDispatchCount, 1);
    assert.equal(s.cpu({ ...s.frame(3), repeatFrame: true }), null);
    assert.equal(s.scheduler().counters.errors, 1);
  });
});

for (const fault of ['unknown-source', 'invalid-admission', 'assembly']) test(`D4 ${fault} rejects receipt and recovers without stale publication`, async () => {
  await fixture(true, async (s) => {
    await stage(s, 0); s.cpu(s.frame(1));
    s.hooks.result = (name, result) => {
      if (name !== 'processFramePyramid') return result;
      if (fault === 'unknown-source') result.sourceFrameIndex = 123456;
      if (fault === 'invalid-admission') result.videoFrameIndex = NaN;
      if (fault === 'assembly') Object.defineProperty(result, 'position', { get() { throw new Error('injected receipt assembly'); } });
      delete s.hooks.result; return result;
    };
    s.stage(s.frame(1)); await turn(); await turn();
    assert.equal(s.scheduler().counters.errors, 1); assert.equal(s.scheduler().counters.barrierResets, 1);
    assert.equal(s.scheduler().ready, false); assert.equal(s.cpu(s.frame(2)), null);
    assert.equal(s.scheduler().counters.publications, 0);
  });
});

test('D0 OFF canvas callback arguments are observed once with no scheduler state', async () => {
  await fixture(false, async (s) => {
    let reads = 0;
    s.pm.onCanvasSizeChange({ get canvasWidth() { reads++; return 240 + reads; }, canvasHeight: 320 });
    assert.equal(reads, 1); assert.equal(s.scheduler(), undefined);
  });
});

for (const command of ['configure', 'projection']) {
  test(`D4 ${command} with liveStale coordinates barriers before touching the old native session`, async () => {
    await fixture(true, async (s) => {
      s.pm.onVideoSizeChange({ videoWidth: 240, videoHeight: 320, orientation: 0, computeCtx: s.gl });
      await stage(s, 0);
      assert.equal(s.scheduler().ready, true);
      const order = [];
      s.hooks.delete = () => order.push('delete');
      s.hooks.updateCameraProjectionMatrix = () => order.push('coordinate-update');
      if (command === 'configure') s.ctrl.configure({});
      else s.ctrl.updateCameraProjectionMatrix({ cam: { pixelRectWidth: 240, pixelRectHeight: 320 } });
      assert.equal(order[0], 'delete', 'old native state must be retired before coordinate resend');
      assert.equal(s.scheduler().counters.barrierResets, 1);
      assert.equal(s.scheduler().ready, false);
      assert.equal(s.cpu(s.frame(1)), null, 'old ready receipt cannot publish in the new epoch');
      assert.equal(s.errors.length, 0);
    });
  });
}

test('D4 liveStale invalid scale throws before native effects and keeps the ready receipt', async () => {
  await fixture(true, async (s) => {
    s.pm.onVideoSizeChange({ videoWidth: 240, videoHeight: 320, orientation: 0, computeCtx: s.gl });
    await stage(s, 0);
    assert.equal(s.scheduler().ready, true);
    const before = s.scheduler().counters.barrierResets, native = [];
    s.hooks.delete = () => native.push('delete');
    s.hooks.updateCameraProjectionMatrix = () => native.push('projection');
    s.hooks.configure = () => native.push('configure');
    assert.throws(() => s.ctrl.configure({ scale: 'absolute' }), /Scale can only/);
    assert.equal(s.scheduler().counters.barrierResets, before);
    assert.equal(s.scheduler().ready, true);
    assert.deepEqual(native, []);
  });
});

test('D4 native failure is reported before a barrier createSession failure and cannot escape the task', async () => {
  await fixture(true, async (s) => {
    s.failProcess(); s.failCreateGlue(); s.stage(s.frame(0));
    await turn(); await turn();
    assert.deepEqual(s.errors.map((e) => e.includes('injected before stage') ? 'original'
      : e.includes('injected createGlue failure') ? 'rebuild' : e), ['original', 'rebuild']);
    assert.ok(s.scheduler().counters.errors >= 1);
    assert.equal(s.scheduler().counters.barrierResets, 1);
    assert.equal(s.scheduler().epoch, null);
    assert.equal(s.scheduler().pending, false);
  });
});

test('D4 throwing errorCallback cannot skip the native failure barrier or escape onmessage', async () => {
  const uncaught = [];
  const monitor = (error) => uncaught.push(String(error));
  process.on('uncaughtExceptionMonitor', monitor);
  try {
    await fixture(true, async (s) => {
      s.errorCallbackThrows(true); s.failProcess(); s.sensor(990); s.stage(s.frame(0));
      await turn(); await turn();
      assert.match(s.errors[0], /injected before stage/);
      assert.ok(s.scheduler().failures.some((failure) => failure.where === 'scheduler-task'
        && /injected before stage/.test(failure.message)));
      assert.equal(s.scheduler().counters.barrierResets, 1);
      assert.equal(s.scheduler().pending, false);
      assert.equal(s.scheduler().nativeAdvanced, false);
      assert.equal(s.calls.filter((entry) => entry.timestampNanos === 990e6).length, 1);
      assert.equal(s.snapshot().sensors.accountingError, 0);
      assert.deepEqual(uncaught, []);
    });
  } finally { process.off('uncaughtExceptionMonitor', monitor); }
});

test('D4 preexisting draw GL error drops only one capture and carries its IMU', async () => {
  await fixture(true, async (s) => {
    s.sensor(990); s.drawError(0x0502); s.stage(s.frame(0));
    assert.equal(s.scheduler().counters.droppedGlError, 1);
    assert.equal(s.scheduler().counters.barrierResets, 0);
    assert.equal(s.scheduler().pending, false);
    assert.equal(s.snapshot().sensors.pending.queued, 1);
    assert.equal(s.snapshot().sensors.dropped, 0);
    s.stage(s.frame(1)); await wait(s); s.cpu(s.frame(2));
    assert.equal(s.calls.filter((e) => e.timestampNanos === 990e6).length, 1);
    assert.equal(s.scheduler().counters.barrierResets, 0);
    assert.equal(s.snapshot().sensors.accountingError, 0);
  });
});

test('D4 missing ON session returns LIMITED with the last pose and current camera texture', async () => {
  await fixture(true, async (s) => {
    await stage(s, 0); s.cpu(s.frame(1));
    await stage(s, 1); const previous = s.cpu(s.frame(2));
    assert.ok(previous?.realityTexture);
    s.pm.onDeviceOrientationChange({ videoWidth: 240, videoHeight: 320, orientation: 45, computeCtx: s.gl });
    const callback = s.frame(3), inactive = s.cpu(callback);
    assert.equal(s.scheduler().epoch, null);
    assert.equal(inactive.trackingStatus, 'LIMITED');
    assert.deepEqual(inactive.position, previous.position);
    assert.deepEqual(inactive.rotation, previous.rotation);
    assert.equal(inactive.realityTexture, callback.cameraTexture);
    assert.equal(s.latest().kind, 'inactive-held');
  });
});

test('D4 WebGL1 draw context reports ON fallback and keeps the legacy frame path usable', async () => {
  await fixture(true, async (s) => {
    s.pm.onSessionDetach({});
    const webgl1Draw = new Proxy(s.gl, { get: (target, name) => name === 'fenceSync' ? undefined : target[name] });
    assert.doesNotThrow(() => s.pm.onSessionAttach({ videoWidth: 240, videoHeight: 320,
      canvasWidth: 240, canvasHeight: 320, rotation: 0, computeCtx: s.gl,
      framework: s.framework, GLctx: webgl1Draw, sessionAttributes: { fillsCameraTexture: true } }));
    const state = s.scheduler();
    assert.equal(state.enabled, false);
    assert.equal(state.mode, 'legacy-fallback');
    assert.equal(state.fallbackReason, 'WebGL1 draw context has no WebGL2 owned-snapshot copy/fence support');
    assert.equal(state.queryBusyReason, null);
    assert.equal(s.errors.length, 0);
    assert.doesNotThrow(() => s.stage(s.frame(0)));
    assert.doesNotThrow(() => s.cpu(s.frame(1)));
  });
});

for (const stalled of ['unsignaled fence', 'missing read request']) {
  test(`D4 ${stalled} triggers one barrier on the 32nd following GPU tick`, async () => {
    await fixture(true, async (s) => {
      s.readyFence(false); s.sensor(990);
      if (stalled === 'missing read request') s.noReadRequest(true);
      s.stage(s.frame(0));
      await turn();
      assert.equal(s.reads(), 0); assert.equal(s.scheduler().pending, true);
      for (let tick = 1; tick <= 31; tick++) {
        s.stage({ ...s.frame(tick), repeatFrame: true });
        assert.equal(s.scheduler().counters.barrierResets, 0, `premature reset at following tick ${tick}`);
      }
      s.stage({ ...s.frame(32), repeatFrame: true });
      assert.equal(s.scheduler().counters.watchdogResets, 1);
      assert.equal(s.scheduler().counters.barrierResets, 1);
      assert.equal(s.scheduler().pending, false);
      assert.equal(s.calls.length, 0, 'stalled source never enters native');
      assert.equal(s.snapshot().sensors.pending.queued, 1);
      assert.equal(s.snapshot().sensors.accountingError, 0);
    });
  });
}

test('D4 no-op command native callback failure recovers once after native unwinds', async () => {
  await fixture(true, async (s) => {
    let deletesInside = 0, inside = false;
    s.hooks.delete = () => { if (inside) deletesInside++; };
    s.hooks.configure = () => { delete s.hooks.configure; inside = true;
      try { s.ctrl.recenter(); } finally { inside = false; } };
    assert.throws(() => s.ctrl.configure({}), /busy/);
    assert.equal(deletesInside, 0); assert.equal(s.scheduler().counters.barrierResets, 1);
    assert.equal(s.scheduler().queryBusyReason, 'await-first-publication');
  });
});

test('D1 fixed60Hz sensor /30Hz frame schedule delivers every unique event once', async () => {
  await fixture(true, async (s) => {
    const expected = [];
    for (let k = 0; k < 24; k++) {
      for (let j = 0; j < 2; j++) {
        const t = 1000 + (2 * k + j) * 1000 / 60; expected.push(1e3 * t * 1e3); s.sensor(t, 1000 / 60);
      }
      s.setNow(1000 + (2 * k + 2) * 1000 / 60); s.stage(s.frame(k)); await wait(s); s.cpu(s.frame(k + 1));
    }
    assert.deepEqual(s.calls.map((e) => e.timestampNanos), expected);
    assert.equal(new Set(s.calls.map((e) => e.timestampNanos)).size, 48);
    const d = s.snapshot().sensors;
    assert.equal(d.received, 48); assert.equal(d.pushed, 48); assert.equal(d.dropped, 0); assert.equal(d.uncertainPushes, 0);
    assert.deepEqual(d.pending, { queued: 0, staged: 0, inFlight: 0 });
    assert.equal(d.duplicateSettlements, 0); assert.equal(d.accountingError, 0);
    assert.equal(s.scheduler().counters.admitted, 24);
  });
});

test('D4 three loaded target replay with zero attach budget drains before native admission', async () => {
  const target = (name) => ({ type: 'PLANAR', name, imagePath: `targets/${name}`, metadata: '{}', userMetadataIsJson: true,
    properties: { left: 0, top: 0, width: 120, height: 160, isRotated: false, originalWidth: 120, originalHeight: 160 } });
  let seed = 123; const luminance = new Uint8Array(120 * 160);
  for (let i = 0; i < luminance.length; i++) { seed = (1664525 * seed + 1013904223) >>> 0; luminance[i] = seed >>> 24; }
  await fixture(true, async (s) => {
    const names = ['A', 'B', 'C']; s.ctrl.configure({ imageTargetData: names.map(target) });
    s.pm.onAppResourcesLoaded({ framework: s.framework }); await globalThis.__slamRefactChunk.imageTargets.whenIdle();
    assert.deepEqual(s.nativeNames().sort(), names);
    s.ctrl.recenter(); assert.equal(globalThis.__slamRefactChunk.imageTargets.replayPending(), 2);
    const startBarriers = s.scheduler().counters.barrierResets;
    s.sensor(990); s.stage(s.frame(0));
    assert.equal(globalThis.__slamRefactChunk.imageTargets.replayPending(), 1);
    assert.equal(s.scheduler().pending, false); assert.equal(s.snapshot().sensors.pending.queued, 1);
    s.stage(s.frame(1)); await wait(s); s.cpu(s.frame(2));
    assert.equal(globalThis.__slamRefactChunk.imageTargets.replayPending(), 0);
    assert.deepEqual(s.nativeNames().sort(), names); assert.equal(s.scheduler().counters.barrierResets, startBarriers);
    assert.equal(s.scheduler().counters.droppedTargetWarmup, 1); assert.equal(s.snapshot().sensors.accountingError, 0);
    assert.equal(s.calls.filter((e) => e.timestampNanos === 990e6).length, 1);
  }, undefined, { attachBudgetMs: 0, loadLuminance: async () => ({ width: 120, height: 160, luminance }) });
});

test('D4 snapshot capacity backpressure clears pending and carries each IMU once', async () => {
  await fixture(true, async (s) => {
    const capture = OwnedDisplaySnapshots.prototype.capture;
    try {
      OwnedDisplaySnapshots.prototype.capture = () => null;
      s.sensor(990); s.stage(s.frame(0));
    } finally { OwnedDisplaySnapshots.prototype.capture = capture; }
    assert.equal(s.scheduler().pending, false); assert.equal(s.scheduler().taskQueued, false);
    assert.equal(s.snapshot().sensors.pending.queued, 1); assert.equal(s.snapshot().sensors.dropped, 0);
    s.stage(s.frame(1)); await wait(s); s.cpu(s.frame(2));
    assert.equal(s.scheduler().counters.droppedCapacity, 1);
    assert.equal(s.calls.filter((e) => e.timestampNanos === 990e6).length, 1);
    assert.equal(s.snapshot().sensors.accountingError, 0); assert.equal(s.snapshot().sensors.duplicateSettlements, 0);
  });
});

test('D4 native query callback mutator is rejected and recovery waits for query unwind', async () => {
  await fixture(true, async (s) => {
    await stage(s, 0); s.cpu(s.frame(1)); await stage(s, 1); s.cpu(s.frame(2));
    let inside = false, deletesInside = 0;
    s.hooks.delete = () => { if (inside) deletesInside++; };
    s.hooks.xrHitTest = () => { inside = true; try { s.ctrl.recenter(); } finally { inside = false; } };
    assert.deepEqual(s.ctrl.hitTest(0.5, 0.5), []);
    assert.equal(deletesInside, 0); assert.equal(s.scheduler().counters.barrierResets, 1);
    assert.equal(s.scheduler().counters.errors, 1); assert.equal(s.scheduler().queryBusyReason, 'await-first-publication');
  });
});

test('D4 async target arrival after internal startup clear still requires a fresh-session barrier', async () => {
  let resolveImage;
  const image = new Promise((resolve) => { resolveImage = resolve; });
  const luminance = new Uint8Array(120 * 160); let seed = 13;
  for (let i = 0; i < luminance.length; i++) { seed = (1664525 * seed + 1013904223) >>> 0; luminance[i] = seed >>> 24; }
  await fixture(true, async (s) => {
    s.ctrl.configure({ imageTargetData: [{ type: 'PLANAR', name: 'late', imagePath: 'late.png', metadata: '{}',
      properties: { left: 0, top: 0, width: 120, height: 160, originalWidth: 120, originalHeight: 160, isRotated: false } }] });
    s.pm.onAppResourcesLoaded({ framework: s.framework });
    for (let k = 0; k < 6; k++) { await stage(s, k); s.cpu(s.frame(k + 1)); }
    assert.equal(s.scheduler().counters.startupDiscardedAdmissions, 1);
    assert.notEqual(s.scheduler().lastAdmission, null); const barriers = s.scheduler().counters.barrierResets;
    resolveImage({ width: 120, height: 160, luminance }); await globalThis.__slamRefactChunk.imageTargets.whenIdle();
    assert.equal(s.scheduler().counters.barrierResets, barriers + 1);
    assert.equal(s.scheduler().lastAdmission, null); assert.deepEqual(s.nativeNames(), ['late']);
  }, undefined, { attachBudgetMs: 0, loadLuminance: async () => image });
});

test('D4 retired epochs share the four-slot cap and resume after post-render fences signal', async () => {
  await fixture(true, async (s) => {
    s.drawFenceReady(false);
    for (let epoch = 0; epoch < 2; epoch++) {
      await stage(s, 2 * epoch); s.cpu(s.frame(2 * epoch + 1));
      await stage(s, 2 * epoch + 1); s.cpu(s.frame(2 * epoch + 2));
      s.ctrl.recenter();
    }
    let d = s.scheduler(); assert.equal(d.snapshots.uniqueSlots, 4); assert.equal(d.snapshots.retiredUniqueSlots, 4);
    assert.equal(d.snapshots.activeUniqueSlots, 0);
    s.sensor(1300); s.stage(s.frame(4));
    d = s.scheduler(); assert.equal(d.pending, false); assert.equal(d.taskQueued, false);
    assert.equal(d.snapshots.uniqueSlots, 4); assert.equal(d.snapshots.retiredPools, 2);
    assert.equal(s.snapshot().sensors.pending.queued, 1);
    s.drawFenceReady(true); s.stage(s.frame(5)); await wait(s); s.cpu(s.frame(6));
    d = s.scheduler(); assert.equal(d.snapshots.retiredPools, 0); assert.ok(d.snapshots.uniqueSlots <= 4);
    assert.equal(s.calls.filter((e) => e.timestampNanos === 1300e6).length, 1);
    assert.equal(s.snapshot().sensors.accountingError, 0); assert.ok(d.memory.peakBytes <= 64 * 1024 * 1024);
  });
});
