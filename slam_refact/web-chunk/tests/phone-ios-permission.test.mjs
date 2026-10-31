// The Start tap on iOS: motion/orientation permission must be REQUESTED inside the tap (before the first await), and the engine script must
// not be loaded or the camera started before the answer. Pure logic with fakes; a real iPhone is still needed to see the prompt (UNVERIFIED here).
import test from 'node:test';
import assert from 'node:assert/strict';
import { runStartFlow, StartError } from '../samples/phone/start-flow.js';
import { detectPlatform, requestMotionPermissions, preflight, supportsWasmSimd } from '../samples/common/platform.js';

const IOS_CHROME = 'Mozilla/5.0 (iPhone; CPU iPhone OS 17_5 like Mac OS X) AppleWebKit/605.1.15 (KHTML, like Gecko) CriOS/126.0.6478.153 Mobile/15E148 Safari/604.1';
const ANDROID_CHROME = 'Mozilla/5.0 (Linux; Android 14; SM-F741N) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/126.0.0.0 Mobile Safari/537.36';
const WA_OK = { validate: () => true };

const makeEnv = ({ ua = IOS_CHROME, answer = 'granted', orientationAnswer = answer, ios = true, secure = true, wa = WA_OK, camera = true } = {}) => {
  const log = [];
  const perm = (name, a) => ({ requestPermission() { log.push(`${name}.requestPermission`); return a instanceof Error ? Promise.reject(a) : Promise.resolve(a); } });
  const win = { isSecureContext: secure };
  if (ios) { win.DeviceMotionEvent = perm('motion', answer); win.DeviceOrientationEvent = perm('orientation', orientationAnswer); }
  const nav = { userAgent: ua, maxTouchPoints: 5, mediaDevices: camera ? { getUserMedia() {} } : undefined };
  const steps = { onStep: (n) => log.push(`step:${n}`), loadXr8: async () => { log.push('loadXr8'); }, run: () => { log.push('run'); } };
  return { env: { win, nav, wa }, steps, log };
};

test('iOS: both permission requests are made synchronously in the tap, before anything is awaited', async () => {
  const { env, steps, log } = makeEnv();
  const pending = runStartFlow(env, steps);  // not awaited: only the synchronous part has run
  assert.deepEqual(log, ['motion.requestPermission', 'orientation.requestPermission', 'step:permission'], 'requested before the first microtask turn');
  await pending;
  assert.deepEqual(log, ['motion.requestPermission', 'orientation.requestPermission', 'step:permission', 'step:engine', 'loadXr8', 'step:camera', 'run']);
});

test('iOS: a denied answer stops the flow: no engine load, no camera', async () => {
  const { env, steps, log } = makeEnv({ answer: 'denied' });
  await assert.rejects(runStartFlow(env, steps), (e) => e instanceof StartError && e.info.code === 'motion-denied' && /모션 센서/.test(e.info.title));
  assert.ok(!log.includes('loadXr8') && !log.includes('run'));
});

test('iOS: requestPermission rejecting (not inside a gesture) is a motion-error, not a crash, and the engine is not loaded', async () => {
  const { env, steps, log } = makeEnv({ answer: Object.assign(new Error('not allowed'), { name: 'NotAllowedError' }) });
  await assert.rejects(runStartFlow(env, steps), (e) => e.info.code === 'motion-error');
  assert.ok(!log.includes('loadXr8'));
});

test('iOS: motion granted but orientation denied is also refused (the chunk listens to both)', async () => {
  const { env, steps } = makeEnv({ answer: 'granted', orientationAnswer: 'denied' });
  await assert.rejects(runStartFlow(env, steps), (e) => e.info.code === 'motion-denied');
});

test('Android Chrome: no permission function exists, none is called, the flow goes straight on', async () => {
  const { env, steps, log } = makeEnv({ ua: ANDROID_CHROME, ios: false });
  await runStartFlow(env, steps);
  assert.deepEqual(log, ['step:permission', 'step:engine', 'loadXr8', 'step:camera', 'run']);
});

test('preflight blocks before any permission request: insecure page, no camera API, no WASM SIMD', async () => {
  for (const [opts, code] of [[{ secure: false }, 'insecure'], [{ camera: false }, 'no-camera-api'], [{ wa: { validate: () => false } }, 'no-simd']]) {
    const { env, steps, log } = makeEnv(opts);
    await assert.rejects(runStartFlow(env, steps), (e) => e instanceof StartError && e.info.code === code, code);
    assert.deepEqual(log, [], `${code}: nothing was requested`);
  }
});

test('platform: iPhone Chrome is WebKit (iOS rules), iPadOS looks like a Mac with touch, Android is not iOS', () => {
  const p = detectPlatform({ userAgent: IOS_CHROME });
  assert.deepEqual([p.isIOS, p.browser, p.iosVersion], [true, 'chrome-ios', { major: 17, minor: 5 }]);
  assert.equal(detectPlatform({ userAgent: 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/605.1.15 Safari/605.1.15', maxTouchPoints: 5 }).isIOS, true);
  assert.equal(detectPlatform({ userAgent: 'Mozilla/5.0 (Macintosh; Intel Mac OS X 10_15_7) AppleWebKit/605.1.15 Safari/605.1.15', maxTouchPoints: 0 }).isIOS, false);
  assert.equal(detectPlatform({ userAgent: ANDROID_CHROME }).isIOS, false);
  assert.equal(detectPlatform({ userAgent: IOS_CHROME.replace('CriOS/126.0.6478.153 Mobile/15E148 Safari/604.1', 'Mobile/15E148 KAKAOTALK 10.4.0') }).inAppBrowser, true);
});

test('requestMotionPermissions: a synchronous throw becomes an error result; missing functions are unsupported', async () => {
  const r = await requestMotionPermissions({ DeviceMotionEvent: { requestPermission() { throw new Error('x'); } }, DeviceOrientationEvent: {} });
  assert.deepEqual([r.motion, r.orientation], ['error', 'unsupported']);
});

test('SIMD probe uses WebAssembly.validate and survives a missing or throwing WebAssembly', () => {
  assert.equal(supportsWasmSimd({ validate: () => true }), true);
  assert.equal(supportsWasmSimd({ validate: () => { throw new Error('x'); } }), false);
  assert.equal(supportsWasmSimd(null), false);
  assert.equal(preflight({ win: {}, nav: { mediaDevices: { getUserMedia() {} } }, wa: WA_OK }), null);
});
