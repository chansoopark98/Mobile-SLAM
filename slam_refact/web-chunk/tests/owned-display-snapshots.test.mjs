import assert from 'node:assert/strict';
import test from 'node:test';
import * as pyramidModule from '../gr8-compute-pyramid.js';

// This models state/ownership contracts. Actual SDK/WebGL pixel evidence is a separate D5 gate.
function drawGl() {
  const names = ['NO_ERROR', 'TEXTURE0', 'TEXTURE_2D', 'ACTIVE_TEXTURE', 'TEXTURE_BINDING_2D',
    'READ_FRAMEBUFFER', 'DRAW_FRAMEBUFFER', 'READ_FRAMEBUFFER_BINDING', 'DRAW_FRAMEBUFFER_BINDING',
    'COLOR_ATTACHMENT0', 'FRAMEBUFFER_COMPLETE', 'RGBA8', 'TEXTURE_MIN_FILTER', 'TEXTURE_MAG_FILTER',
    'TEXTURE_WRAP_S', 'TEXTURE_WRAP_T', 'LINEAR', 'CLAMP_TO_EDGE', 'MAX_TEXTURE_SIZE',
    'SYNC_GPU_COMMANDS_COMPLETE', 'ALREADY_SIGNALED', 'CONDITION_SATISFIED', 'TIMEOUT_EXPIRED', 'WAIT_FAILED'];
  const gl = Object.fromEntries(names.map((n, i) => [n, i]));
  const s = { calls: [], textures: new Map(), attachments: new Map(), bindings: new Map(),
    active: gl.TEXTURE0 + 3, read: { name: 'hostile-read' }, draw: { name: 'hostile-draw' },
    // These are not touched by the adapter; they must remain unchanged even on failure.
    untouched: { sampler: { name: 'sampler' }, viewport: [9, 7, 53, 41], scissor: [2, 4, 11, 13],
      pack: { name: 'pack' }, unpack: { name: 'unpack' }, colorMask: [false, true, false, true],
      program: { name: 'program' }, vao: { name: 'vao' }, caps: [true, true, true] },
    error: 0, lost: false, wait: gl.ALREADY_SIGNALED, fail: null, serial: 0 };
  s.bindings.set(gl.TEXTURE0, { name: 'unit0' });
  s.bindings.set(s.active, { name: 'unit3' });
  const op = (name, fn) => (...args) => {
    s.calls.push([name, ...args]);
    if (s.fail === name) throw new Error(`injected ${name}`);
    return fn(...args);
  };
  Object.assign(gl, {
    isContextLost: op('isContextLost', () => s.lost),
    isTexture: op('isTexture', (t) => s.textures.has(t)),
    getError: op('getError', () => { const e = s.error; s.error = 0; return e; }),
    getParameter: op('getParameter', (p) => {
      if (p === gl.ACTIVE_TEXTURE) return s.active;
      if (p === gl.TEXTURE_BINDING_2D) return s.bindings.get(s.active) ?? null;
      if (p === gl.READ_FRAMEBUFFER_BINDING) return s.read;
      if (p === gl.DRAW_FRAMEBUFFER_BINDING) return s.draw;
      if (p === gl.MAX_TEXTURE_SIZE) return 4096;
      throw new Error(`unexpected parameter ${p}`);
    }),
    activeTexture: op('activeTexture', (v) => { s.active = v; }),
    bindTexture: op('bindTexture', (_t, v) => { s.bindings.set(s.active, v); }),
    createTexture: op('createTexture', () => { const t = { id: ++s.serial }; s.textures.set(t, null); return t; }),
    deleteTexture: op('deleteTexture', (t) => s.textures.delete(t)),
    texParameteri: op('texParameteri', () => {}),
    texStorage2D: op('texStorage2D', (_t, levels, format, w, h) => {
      assert.equal(levels, 1); assert.equal(format, gl.RGBA8);
      s.textures.set(s.bindings.get(s.active), { width: w, height: h, pixels: new Uint8Array(w * h * 4) });
    }),
    createFramebuffer: op('createFramebuffer', () => ({ fbo: ++s.serial })),
    deleteFramebuffer: op('deleteFramebuffer', (f) => s.attachments.delete(f)),
    bindFramebuffer: op('bindFramebuffer', (t, f) => {
      if (t === gl.READ_FRAMEBUFFER) s.read = f;
      else if (t === gl.DRAW_FRAMEBUFFER) s.draw = f;
      else throw new Error('adapter must not bind both framebuffers together');
    }),
    framebufferTexture2D: op('framebufferTexture2D', (t, _a, _tt, texture) => {
      assert.equal(t, gl.READ_FRAMEBUFFER); s.attachments.set(s.read, texture);
    }),
    checkFramebufferStatus: op('checkFramebufferStatus', () => gl.FRAMEBUFFER_COMPLETE),
    copyTexSubImage2D: op('copyTexSubImage2D', (_t, _level, x, y, sx, sy, w, h) => {
      assert.deepEqual([x, y, sx, sy], [0, 0, 0, 0]);
      const src = s.textures.get(s.attachments.get(s.read));
      const dst = s.textures.get(s.bindings.get(s.active));
      assert.deepEqual([w, h], [src.width, src.height]);
      assert.deepEqual([w, h], [dst.width, dst.height]);
      dst.pixels.set(src.pixels);
    }),
    fenceSync: op('fenceSync', () => ({ sync: ++s.serial })),
    clientWaitSync: op('clientWaitSync', (_sync, flags, timeout) => {
      assert.equal(flags, 0); assert.equal(timeout, 0); return s.wait;
    }),
    deleteSync: op('deleteSync', () => {}), flush: op('flush', () => {}),
  });
  return { gl, s, source(w = 3, h = 2, seed = 1) {
    const t = gl.createTexture();
    s.textures.set(t, { width: w, height: h, pixels: Uint8Array.from({ length: w * h * 4 }, (_, i) => (seed + i * 17) % 256) });
    return t;
  }, state() { return { active: s.active, read: s.read, draw: s.draw,
    bindings: [...s.bindings], untouched: structuredClone(s.untouched) }; } };
}

function setup({ cap = 64 * 1024 * 1024, maxSlots = 4 } = {}) {
  const f = drawGl(), tokens = new Set(); let live = 0, peak = 0;
  assert.equal(typeof pyramidModule.OwnedDisplaySnapshots, 'function');
  const pool = new pyramidModule.OwnedDisplaySnapshots(f.gl, { maxSlots,
    reserveBytes(kind, bytes) {
      if (live + bytes > cap) return null;
      const token = { kind, bytes }; tokens.add(token); live += bytes; peak = Math.max(peak, live); return token;
    },
    releaseBytes(token) { assert.ok(tokens.delete(token), 'budget token must settle once'); live -= token.bytes; },
  });
  return { ...f, pool, budget: () => ({ live, peak, tokens: tokens.size }) };
}
const epoch = (n = 1) => ({ sessionEpoch: 3, schedulerEpoch: n });
const capture = (f, seq = 1, e = epoch(), w = 3, h = 2) => f.pool.capture({
  sourceDrawTexture: f.source(w, h, seq), width: w, height: h,
  meta: { epoch: e, captureSeq: seq, rotation: 0 },
});

test('D5 copy preserves full RGBA pixels and hostile GL state without changing the SDK source', () => {
  const f = setup(), source = f.source(), before = f.state();
  const original = f.s.textures.get(source).pixels.slice();
  const key = f.pool.capture({ sourceDrawTexture: source, width: 3, height: 2,
    meta: { epoch: epoch(), captureSeq: 7, rotation: 270 } });
  assert.deepEqual(f.state(), before);
  const texture = f.pool.textureForPublication(key, epoch());
  assert.notEqual(texture, source);
  assert.deepEqual(f.s.textures.get(texture).pixels, original);
  f.s.textures.get(source).pixels.fill(255);
  assert.deepEqual(f.s.textures.get(texture).pixels, original, 'SDK reuse cannot change owned bytes');
  assert.ok(!f.s.calls.some(([n]) => ['finish', 'readPixels', 'getBufferSubData'].includes(n)));
  assert.equal(f.pool.stats().controlledBytes, 24); assert.equal(f.budget().live, 24);
  assert.equal(f.pool.stats().driverBytes, null);
  f.pool.dispose(); assert.equal(f.budget().live, 0);
});

test('D4 owner roles count one allocation and last release waits a post-render fence', () => {
  const f = setup(), key = capture(f);
  assert.equal(f.pool.retain(key, 'enginePrevious'), true);
  assert.equal(f.pool.retain(key, 'enginePrevious'), true);
  assert.equal(f.pool.retain(key, 'display'), true);
  assert.equal(f.pool.release(key, 'gpuPending'), true);
  assert.equal(f.pool.release(key, 'enginePrevious'), true);
  assert.equal(f.pool.stats().controlledBytes, 24);
  assert.ok(f.pool.textureForPublication(key, epoch()));
  assert.equal(f.pool.release(key, 'display'), true);
  assert.equal(f.pool.textureForPublication(key, epoch()), null);
  assert.equal(f.pool.stats().pendingRetire, 1);
  f.pool.pollRetired(); assert.equal(f.s.calls.filter(([n]) => n === 'clientWaitSync').length, 0);
  f.pool.fenceRetiredAfterRender();
  f.s.wait = f.gl.TIMEOUT_EXPIRED; f.pool.pollRetired();
  assert.equal(f.pool.stats().pendingRetire, 1);
  f.s.wait = f.gl.CONDITION_SATISFIED; f.pool.pollRetired();
  assert.equal(f.pool.stats().pendingRetire, 0);
  assert.equal(f.budget().live, 24, 'free allocated texture retains its budget reservation');
  const next = capture(f, 2);
  assert.equal(next.slot, key.slot); assert.equal(next.generation, key.generation + 1);
  assert.equal(f.pool.textureForPublication(key, epoch()), null);
  assert.equal(f.pool.release(key, 'gpuPending'), false, 'stale release cannot remove a newer owner');
  assert.ok(f.pool.textureForPublication(next, epoch()));
  f.pool.dispose();
});

test('D4 explicit retired role cannot bypass the post-render fence', () => {
  const f = setup({ maxSlots: 1 }), key = capture(f);
  f.pool.retain(key, 'retired'); f.pool.release(key, 'gpuPending');
  assert.equal(capture(f, 2), null);
  assert.equal(f.pool.retain(key, 'display'), false);
  f.pool.release(key, 'retired');
  assert.equal(capture(f, 3), null);
  f.pool.fenceRetiredAfterRender(); f.pool.pollRetired();
  assert.ok(capture(f, 4)); f.pool.dispose();
});

test('D4 epoch/capture identity rejects forged generations and invalidates only the requested epoch', () => {
  const f = setup(), a = capture(f, 1), b = capture(f, 2, epoch(2));
  assert.equal(f.pool.textureForPublication(a, epoch(2)), null);
  assert.equal(f.pool.textureForPublication({ ...a, captureSeq: 99 }, epoch()), null);
  f.pool.invalidateEpoch(epoch(), 'rebuild');
  assert.equal(f.pool.textureForPublication(a, epoch()), null);
  assert.ok(f.pool.textureForPublication(b, epoch(2)));
  f.pool.fenceRetiredAfterRender(); f.pool.pollRetired(); f.pool.dispose();
  assert.deepEqual(f.budget(), { live: 0, peak: 48, tokens: 0 });
});

test('D4 bounded pool refuses a fifth live allocation and a budget miss happens before GL allocation', () => {
  const f = setup(); for (let i = 0; i < 4; i++) assert.ok(capture(f, i));
  const allocations = f.s.calls.filter(([n]) => n === 'createTexture').length;
  // source() itself allocates the SDK texture; the adapter must allocate no owned texture on refusal.
  assert.equal(capture(f, 5), null);
  assert.equal(f.s.calls.filter(([n]) => n === 'createTexture').length, allocations + 1);
  assert.equal(f.pool.stats().uniqueSlots, 4); assert.equal(f.budget().live, 96);
  f.pool.dispose();
  const small = setup({ cap: 23 }), source = small.source();
  const before = small.s.calls.length;
  assert.equal(small.pool.capture({ sourceDrawTexture: source, width: 3, height: 2,
    meta: { epoch: epoch(), captureSeq: 1 } }), null);
  assert.ok(!small.s.calls.slice(before).some(([n]) => n === 'createTexture'));
  assert.equal(small.budget().live, 0); small.pool.dispose();
});

test('D4 safe geometry replacement returns old reservation and never rescales', () => {
  const f = setup({ maxSlots: 1 }), a = capture(f);
  f.pool.release(a, 'gpuPending'); f.pool.fenceRetiredAfterRender(); f.pool.pollRetired();
  const b = capture(f, 2, epoch(), 5, 7);
  const texture = f.pool.textureForPublication(b, epoch());
  assert.deepEqual([f.s.textures.get(texture).width, f.s.textures.get(texture).height], [5, 7]);
  assert.equal(f.budget().live, 140); assert.equal(f.pool.stats().controlledBytes, 140);
  f.pool.dispose();
});

for (const fail of ['createTexture', 'texStorage2D', 'createFramebuffer', 'checkFramebufferStatus', 'copyTexSubImage2D']) {
  test(`D4 ${fail} exception restores state and releases partially created reservation`, () => {
    const f = setup(), source = f.source(), before = f.state(); f.s.fail = fail;
    assert.throws(() => f.pool.capture({ sourceDrawTexture: source, width: 3, height: 2,
      meta: { epoch: epoch(), captureSeq: 1 } }), new RegExp(`injected ${fail}`));
    assert.deepEqual(f.state(), before); assert.equal(f.budget().live, 0);
    assert.equal(f.pool.stats().captures, 0); assert.equal(f.pool.stats().controlledBytes, 0);
    f.s.fail = null; assert.ok(capture(f, 2)); f.pool.dispose();
  });
}

test('D4 preexisting GL error is reported once and prevents allocation/copy', () => {
  const f = setup(), source = f.source(); f.s.error = 0x0502;
  const before = f.s.calls.length;
  assert.throws(() => f.pool.capture({ sourceDrawTexture: source, width: 3, height: 2,
    meta: { epoch: epoch(), captureSeq: 1 } }), /preexisting.*1282/);
  assert.equal(f.s.calls.slice(before).filter(([n]) => n === 'getError').length, 1);
  assert.ok(!f.s.calls.slice(before).some(([n]) => ['createTexture', 'copyTexSubImage2D'].includes(n)));
  assert.match(f.pool.stats().lastError, /preexisting/); assert.equal(f.budget().live, 0); f.pool.dispose();
});

for (const name of ['createTexture', 'createFramebuffer']) {
  test(`D4 ${name} null result restores state and releases its budget token`, () => {
    const f = setup(), source = f.source(), before = f.state(), original = f.gl[name];
    f.gl[name] = () => null;
    assert.throws(() => f.pool.capture({ sourceDrawTexture: source, width: 3, height: 2,
      meta: { epoch: epoch(), captureSeq: 1 } }), new RegExp(`${name} returned null`));
    assert.deepEqual(f.state(), before); assert.equal(f.budget().live, 0);
    f.gl[name] = original; f.pool.dispose();
  });
}

test('D4 incomplete source framebuffer and copy OOM cannot publish an allocation', () => {
  for (const failure of ['incomplete', 'oom']) {
    const f = setup(), source = f.source(), before = f.state();
    if (failure === 'incomplete') f.gl.checkFramebufferStatus = () => 0;
    else {
      const copy = f.gl.copyTexSubImage2D;
      f.gl.copyTexSubImage2D = (...args) => { copy(...args); f.s.error = 0x0505; };
    }
    assert.throws(() => f.pool.capture({ sourceDrawTexture: source, width: 3, height: 2,
      meta: { epoch: epoch(), captureSeq: 1 } }), failure === 'incomplete' ? /framebuffer incomplete/ : /copy GL error 1285/);
    assert.deepEqual(f.state(), before); assert.equal(f.budget().live, 0);
    assert.equal(f.pool.stats().uniqueSlots, 0); assert.equal(f.pool.stats().captures, 0); f.pool.dispose();
  }
});

test('D4 failed retirement fence and WAIT_FAILED never make the slot reusable', () => {
  const f = setup({ maxSlots: 1 }), key = capture(f), fence = f.gl.fenceSync;
  f.pool.release(key, 'gpuPending'); f.gl.fenceSync = () => null;
  assert.throws(() => f.pool.fenceRetiredAfterRender(), /fence allocation failed/);
  assert.equal(capture(f, 2), null);
  f.gl.fenceSync = fence; f.pool.fenceRetiredAfterRender();
  f.s.wait = f.gl.WAIT_FAILED; assert.throws(() => f.pool.pollRetired(), /fence wait failed/);
  assert.equal(capture(f, 3), null); assert.equal(f.pool.stats().pendingRetire, 1);
  f.pool.dispose(); assert.equal(f.budget().live, 0);
});

test('D4 invalid dimensions/metadata/source are rejected before GL allocation', () => {
  const f = setup(), source = f.source();
  for (const args of [
    { width: 0, height: 2 }, { width: 4097, height: 2 }, { width: 3.5, height: 2 },
    { width: 3, height: 2, sourceDrawTexture: {} },
    { width: 3, height: 2, meta: { epoch: epoch(), captureSeq: -1 } },
    { width: 3, height: 2, meta: { epoch: epoch(), captureSeq: 1, width: 4, height: 2 } },
  ]) assert.throws(() => f.pool.capture({ sourceDrawTexture: source, width: 3, height: 2,
    meta: { epoch: epoch(), captureSeq: 1 }, ...args }));
  assert.equal(f.budget().live, 0); f.pool.dispose();
});

test('D4 context loss disposes logical ownership without calling GL deletion on the lost context', () => {
  const f = setup(), key = capture(f); f.pool.release(key, 'gpuPending'); f.pool.fenceRetiredAfterRender();
  f.s.lost = true; const before = f.s.calls.length;
  assert.throws(() => capture(f, 2), /context lost/);
  f.pool.dispose();
  assert.ok(!f.s.calls.slice(before).some(([n]) => ['deleteTexture', 'deleteSync', 'deleteFramebuffer'].includes(n)));
  assert.equal(f.pool.stats().uniqueSlots, 0); assert.equal(f.pool.stats().controlledBytes, 0);
  assert.equal(f.budget().live, 0); f.pool.dispose();
});

test('D2 readiness only polls the existing fence with zero timeout and preserves default read state', () => {
  const f = drawGl(), calls = f.s.calls;
  assert.equal(typeof pyramidModule.Gr8ComputePyramid.prototype.isReadReady, 'function');
  const pyramid = { gl: f.gl, needsRead: true, sync: { id: 'pending' }, stats: {},
    _assertContext: pyramidModule.Gr8ComputePyramid.prototype._assertContext,
    isContextLost: () => f.s.lost };
  const poll = () => pyramidModule.Gr8ComputePyramid.prototype.isReadReady.call(pyramid);
  f.s.wait = f.gl.TIMEOUT_EXPIRED; assert.equal(poll(), false);
  f.s.wait = f.gl.ALREADY_SIGNALED; assert.equal(poll(), true);
  assert.deepEqual(pyramid.sync, { id: 'pending' }); assert.equal(pyramid.needsRead, true);
  assert.equal(calls.filter(([n]) => n === 'clientWaitSync').length, 2);
  assert.ok(!calls.some(([n]) => ['deleteSync', 'getBufferSubData', 'readPixels'].includes(n)));
  f.s.wait = f.gl.WAIT_FAILED; assert.throws(poll, /fence wait failed/);
  pyramid.needsRead = false; const before = calls.length; assert.equal(poll(), false); assert.equal(calls.length, before);
});

test('D2 ready on the second poll reads once without a third fence wait', () => {
  const f = drawGl(), { gl, s } = f, source = Uint8Array.from({ length: 16 }, (_, i) => i * 7);
  const callerPack = {}, ownPack = {}; let pack = callerPack, reads = 0;
  gl.PIXEL_PACK_BUFFER_BINDING = 1001; gl.PIXEL_PACK_BUFFER = 1002;
  const get = gl.getParameter;
  gl.getParameter = (p) => p === gl.PIXEL_PACK_BUFFER_BINDING ? pack : get(p);
  gl.bindBuffer = (target, buffer) => { assert.equal(target, gl.PIXEL_PACK_BUFFER); pack = buffer; };
  gl.getBufferSubData = (_target, _offset, view) => { assert.equal(pack, ownPack); reads++; view.set(source); };
  const pyramid = { gl, needsRead: true, sync: { id: 'first' }, stats: { signaledAtRead: 0, read: 0 },
    pixelBuffer: ownPack, out: new Uint8Array(16), size: 2, stageRows: 2, isContextLost: () => s.lost,
    _assertContext: pyramidModule.Gr8ComputePyramid.prototype._assertContext,
    _check: pyramidModule.Gr8ComputePyramid.prototype._check };
  const poll = () => pyramidModule.Gr8ComputePyramid.prototype.isReadReady.call(pyramid);
  const consume = () => pyramidModule.Gr8ComputePyramid.prototype.readReady.call(pyramid);
  assert.throws(consume, /readiness was not established/);
  s.wait = gl.TIMEOUT_EXPIRED; assert.equal(poll(), false);
  assert.throws(consume, /readiness was not established/);
  s.wait = gl.ALREADY_SIGNALED; assert.equal(poll(), true);
  assert.deepEqual(consume(), source);
  assert.equal(s.calls.filter(([n]) => n === 'clientWaitSync').length, 2);
  assert.equal(reads, 1); assert.equal(pack, callerPack);
  assert.equal(pyramid.stats.signaledAtRead, 1); assert.equal(pyramid.stats.read, 1);
  assert.equal(pyramid.needsRead, false); assert.equal(pyramid.sync, null); assert.equal(consume(), null);
  pyramid.needsRead = true; pyramid.sync = { id: 'second' };
  assert.throws(consume, /readiness was not established/, 'cached readiness cannot apply to a newer fence');
});

test('D4 B1 ready readback failure restores caller pack buffer and cannot be retried', () => {
  const f = drawGl(), { gl, s } = f;
  const callerPack = {}; let pack = callerPack;
  gl.PIXEL_PACK_BUFFER_BINDING = 1001; gl.PIXEL_PACK_BUFFER = 1002;
  const get = gl.getParameter;
  gl.getParameter = (p) => p === gl.PIXEL_PACK_BUFFER_BINDING ? pack : get(p);
  gl.bindBuffer = (_target, buffer) => { pack = buffer; };
  gl.getBufferSubData = () => { throw new Error('injected readReady copy failure'); };
  const sync = {};
  const pyramid = { gl, needsRead: true, sync, readySync: sync, stats: { signaledAtRead: 0, read: 0 },
    pixelBuffer: {}, out: new Uint8Array(16), size: 2, stageRows: 2, isContextLost: () => s.lost,
    _assertContext: pyramidModule.Gr8ComputePyramid.prototype._assertContext,
    _check: pyramidModule.Gr8ComputePyramid.prototype._check };
  const consume = () => pyramidModule.Gr8ComputePyramid.prototype.readReady.call(pyramid);
  assert.throws(consume, /injected readReady copy failure/);
  assert.equal(pack, callerPack); assert.equal(consume(), null); assert.equal(pyramid.stats.read, 0);
});
