// samples/common/camera-resolution.js: only xr.js's first Android Chrome request (960x720 exact) is replaced; a refused size falls back to
// the SDK request unchanged; every other request and ?camera=960x720 pass through untouched.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const here = path.dirname(fileURLToPath(import.meta.url));
const source = fs.readFileSync(path.join(here, '../samples/common/camera-resolution.js'), 'utf8');
const SDK_FIRST = { width: { exact: 960 }, height: { exact: 720 }, facingMode: { exact: 'environment' } };

const load = (search = '', refuse = () => false) => {
  const calls = [];
  const stream = (c) => ({ getVideoTracks: () => [{ getSettings: () => ({ width: c.video.width.exact, height: c.video.height.exact, frameRate: 30 }) }] });
  const mediaDevices = {
    getUserMedia(c) {
      calls.push(JSON.parse(JSON.stringify(c)));
      if (refuse(c)) return Promise.reject(Object.assign(new Error('no'), { name: 'OverconstrainedError' }));
      return Promise.resolve(c.video && c.video.width ? stream(c) : { getVideoTracks: () => [] });
    },
  };
  const window = { location: { search } };
  vm.runInNewContext(source, { window, navigator: { mediaDevices }, URLSearchParams });
  return { window, gum: (c) => mediaDevices.getUserMedia(c), calls };
};

test('default: the SDK first request becomes 1920x1440 exact, other fields kept', async () => {
  const { window, gum, calls } = load();
  await gum({ video: SDK_FIRST });
  assert.deepEqual(calls, [{ video: { width: { exact: 1920 }, height: { exact: 1440 }, facingMode: { exact: 'environment' } } }]);
  assert.deepEqual({ ...window.XrCameraResolution.granted }, { width: 1920, height: 1440, frameRate: 30 });
  assert.equal(window.XrCameraResolution.fallbackReason, null);
});

test('?camera=WxH picks the size', async () => {
  const { gum, calls } = load('?camera=1440x1080');
  await gum({ video: SDK_FIRST });
  assert.deepEqual(calls[0].video.width, { exact: 1440 });
  assert.deepEqual(calls[0].video.height, { exact: 1080 });
});

test('a refused size falls back to the SDK request unchanged', async () => {
  const { window, gum, calls } = load('', (c) => c.video.width.exact === 1920);
  await gum({ video: SDK_FIRST });
  assert.equal(calls.length, 2);
  assert.deepEqual(calls[1], { video: SDK_FIRST });
  assert.deepEqual({ ...window.XrCameraResolution.granted }, { width: 960, height: 720, frameRate: 30 });
  assert.match(window.XrCameraResolution.fallbackReason, /1920x1440 refused \(OverconstrainedError\)/);
});

test('the SDK fallback refusing too rejects like the SDK request would (the cascade goes on in xr.js)', async () => {
  const { gum, calls } = load('', () => true);
  await assert.rejects(gum({ video: SDK_FIRST }), { name: 'OverconstrainedError' });
  assert.equal(calls.length, 2);
});

test('other requests pass through untouched', async () => {
  const { gum, calls } = load();
  const others = [{ video: true }, { video: { width: { min: 1920 }, height: { min: 1440 } } }, { video: { width: { exact: 1280 }, height: { exact: 720 } } }, { audio: true }];
  for (const c of others) await gum(c);
  assert.deepEqual(calls, others);
});

test('?camera=960x720 keeps the SDK request; an invalid value uses the default and says so', async () => {
  const off = load('?camera=960x720');
  await off.gum({ video: SDK_FIRST });
  assert.deepEqual(off.calls, [{ video: SDK_FIRST }]);
  const bad = load('?camera=big');
  await bad.gum({ video: SDK_FIRST });
  assert.deepEqual(bad.calls[0].video.width, { exact: 1920 });
  assert.equal(bad.window.XrCameraResolution.fallbackReason, 'ignored ?camera=big');
  const wide = load('?camera=1920x1080');  // 16:9 would change the sensor crop
  await wide.gum({ video: SDK_FIRST });
  assert.deepEqual(wide.calls[0].video.width, { exact: 1920 });
  assert.deepEqual(wide.calls[0].video.height, { exact: 1440 });
  assert.equal(wide.window.XrCameraResolution.fallbackReason, 'ignored ?camera=1920x1080');
});
