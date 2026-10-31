// samples/common/camera-resolution.js on iPhone: xr.js asks for {min 960x720}; the page tries 4:3 exact sizes first, then passes the request on.
import test from 'node:test';
import assert from 'node:assert/strict';
import fs from 'node:fs';
import path from 'node:path';
import vm from 'node:vm';
import { fileURLToPath } from 'node:url';

const source = fs.readFileSync(path.join(path.dirname(fileURLToPath(import.meta.url)), '../samples/common/camera-resolution.js'), 'utf8');
const IOS_UA = 'Mozilla/5.0 (iPhone; CPU iPhone OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) CriOS/126 Mobile/15E148 Safari/604.1';
const IOS_FIRST = { width: { min: 960 }, height: { min: 720 }, facingMode: ['environment'] };

const load = ({ search = '', ua = IOS_UA, refuse = () => false } = {}) => {
  const calls = [];
  const mediaDevices = {
    getUserMedia(c) {
      calls.push(JSON.parse(JSON.stringify(c)));
      if (refuse(c)) return Promise.reject(Object.assign(new Error('no'), { name: 'OverconstrainedError' }));
      const w = c.video.width.exact ?? 1280, h = c.video.height.exact ?? 720;
      return Promise.resolve({ getVideoTracks: () => [{ getSettings: () => ({ width: w, height: h, frameRate: 30 }) }] });
    },
  };
  const window = { location: { search } };
  vm.runInNewContext(source, { window, navigator: { mediaDevices, userAgent: ua, maxTouchPoints: 5 }, URLSearchParams });
  return { window, gum: (c) => mediaDevices.getUserMedia(c), calls };
};

test('iPhone: the first request is tried at 1920x1440 exact, other fields kept (facingMode survives)', async () => {
  const { window, gum, calls } = load();
  await gum({ video: IOS_FIRST });
  assert.deepEqual(calls, [{ video: { width: { exact: 1920 }, height: { exact: 1440 }, facingMode: ['environment'] } }]);
  assert.deepEqual({ ...window.XrCameraResolution.granted }, { width: 1920, height: 1440, frameRate: 30 });
  assert.equal(window.XrCameraResolution.fallbackReason, null);
});

test('iPhone: refused sizes go down the 4:3 ladder, then the SDK request is passed on unchanged', async () => {
  const { window, gum, calls } = load({ refuse: (c) => c.video.width.exact !== undefined });
  await gum({ video: IOS_FIRST });
  assert.deepEqual(calls.map((c) => c.video.width.exact ?? 'sdk'), [1920, 1440, 1280, 'sdk']);
  assert.deepEqual(calls[3], { video: IOS_FIRST });
  assert.match(window.XrCameraResolution.fallbackReason, /1920x1440 refused.*1440x1080 refused.*1280x960 refused.*SDK request used/);
  assert.ok(calls.slice(0, 3).every((c) => c.video.width.exact * 3 === c.video.height.exact * 4), 'every size asked is 4:3');
});

test('iPhone: a deviceId added by xr.js (iOS 16.4+ second open) is kept', async () => {
  const { gum, calls } = load();
  await gum({ video: { ...IOS_FIRST, deviceId: 'abc' } });
  assert.equal(calls[0].video.deviceId, 'abc');
});

test('iPhone: ?camera=1440x1080 asks that size only; ?camera=960x720 leaves the SDK request untouched', async () => {
  const a = load({ search: '?camera=1440x1080' });
  await a.gum({ video: IOS_FIRST });
  assert.deepEqual(a.calls.map((c) => c.video.width.exact), [1440]);
  const b = load({ search: '?camera=960x720' });
  await b.gum({ video: IOS_FIRST });
  assert.deepEqual(b.calls, [{ video: IOS_FIRST }]);
});

test('iPhone: other requests (the SDK cascade after the first) pass through untouched; Android UA is not affected', async () => {
  const { gum, calls } = load();
  const second = { video: { width: { min: 640 }, height: { min: 480 }, facingMode: ['environment'] } };
  await gum(second);
  assert.deepEqual(calls, [second]);
  const android = load({ ua: 'Mozilla/5.0 (Linux; Android 14; SM-F741N) AppleWebKit/537.36 Chrome/126 Mobile Safari/537.36' });
  await android.gum({ video: IOS_FIRST });
  assert.deepEqual(android.calls, [{ video: IOS_FIRST }]);
});
