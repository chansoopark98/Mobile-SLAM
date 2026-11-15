// node --test slam_refact/web-chunk/tests/message-listener.test.mjs
// FX-E3: the chunk's window `message` listener ignores non-object data (data: null used to throw a TypeError) and, only when
// SLAM_REFACT_CHUNK_OPTIONS.trustedMessageOrigins is set, drops messages from other origins and counts them. Unset = old behaviour.
// The one path from a message into the sensor queue is the window `devicemotion8w` event, so the tests observe that event.
import assert from 'node:assert/strict';
import test from 'node:test';
import { engineAvailable, loadRealModule, runChunk, syntheticAtlas } from './engine-worker-harness.mjs';

const skip = engineAvailable ? false : 'engine build not found (SLAM_REFACT_CHUNK_ENGINE_DIR)';
const W = 240, H = 320;
let Module = null;
const atlases = new Map();
test.before(async () => { if (!skip) Module = await loadRealModule(); });
const atlasFor = (k) => atlases.get(k) ?? (atlases.set(k, syntheticAtlas(Module, W, H, k).rgba), atlases.get(k));

const motion = { acceleration: { x: 0, y: 0, z: 0 }, accelerationIncludingGravity: { x: 0, y: 9.8, z: 0 }, rotationRate: { alpha: 0, beta: 0, gamma: 0 }, timeStamp: 1000, interval: 16 };

async function withChunk(extraOptions, body) {
  const seen = [];
  const r = await runChunk({ atlasFor, steps: 1, extraOptions, globals: (g) => {
    g.set('CustomEvent', class { constructor(type, init) { this.type = type; this.detail = init?.detail; } });
    g.set('dispatchEvent', (e) => g.dispatch(e));
    g.set('__post', (e) => g.dispatch({ type: 'message', ...e }));
  } });
  try {
    globalThis.addEventListener('devicemotion8w', () => seen.push('devicemotion8w'));
    await body({ post: globalThis.__post, seen, diag: r.diag });
  } finally { r.close(); }
}

test('E3-1: data null / string / number never throws, whatever the options', { skip }, async () => {
  for (const extra of [{}, { trustedMessageOrigins: ['https://a.example'] }]) {
    await withChunk(extra, async ({ post, seen }) => {
      for (const data of [null, undefined, 'text', 7, true]) assert.doesNotThrow(() => post({ data, origin: 'https://a.example' }));
      assert.deepEqual(seen, []);
    });
  }
});

test('E3-2: option unset -> a deviceMotion8w message from any origin is delivered, no new diagnostics key', { skip }, async () => {
  await withChunk({}, async ({ post, seen, diag }) => {
    post({ data: { deviceMotion8w: motion }, origin: 'https://evil.example' });
    post({ data: { deviceMotion8w: motion }, origin: 'https://a.example' });
    assert.equal(seen.length, 2);
    assert.equal('trustedMessageOrigins' in diag, false);
  });
});

test('E3-3: trustedMessageOrigins set -> a message from another origin is dropped and counted, a listed origin is delivered', { skip }, async () => {
  await withChunk({ trustedMessageOrigins: ['https://a.example'] }, async ({ post, seen, diag }) => {
    post({ data: { deviceMotion8w: motion }, origin: 'https://evil.example' });
    assert.deepEqual(seen, []);
    assert.equal(diag.trustedMessageOrigins.dropped, 1);
    post({ data: { deviceMotion8w: motion }, origin: 'https://a.example' });
    assert.equal(seen.length, 1);
    assert.equal(diag.trustedMessageOrigins.dropped, 1);
    assert.deepEqual(diag.trustedMessageOrigins.origins, ['https://a.example']);
  });
});

test('E3-4: a malformed trustedMessageOrigins is an [XR] error', { skip }, async () => {
  for (const bad of ['https://a.example', [1], {}]) {
    await assert.rejects(() => withChunk({ trustedMessageOrigins: bad }, async () => {}), /\[XR\].*trustedMessageOrigins/);
  }
});
