// Platform facts the phone page needs before it asks for anything: which browser family this is (iOS = WebKit, also under Chrome/Firefox
// on iPhone), whether the motion-permission API exists, whether the secure-context / camera / WASM-SIMD requirements are met.
// Pure functions of the objects they are given (no globals at import time), so node tests can drive them with fakes.
//
// iOS facts this file relies on (WebKit, iOS 13+): DeviceMotionEvent.requestPermission() and DeviceOrientationEvent.requestPermission()
// exist and only resolve with a prompt when called from a user gesture (the Start tap); one prompt covers both; a denied answer is
// remembered until the page is reloaded. Android Chrome has neither function and delivers sensor events without a prompt.

const IN_APP = /KAKAOTALK|NAVER\(inapp|FBAN|FBAV|Instagram|Line\/|DaumApps|Snapchat|; wv\)/i;

export function detectPlatform(nav = {}) {
  const ua = String(nav.userAgent || '');
  // iPadOS 13+ reports a Mac user agent; touch points tell it apart from a Mac.
  const iPadAsMac = /Macintosh/.test(ua) && Number(nav.maxTouchPoints) > 1;
  const isIOS = /iPhone|iPad|iPod/.test(ua) || iPadAsMac;
  const isAndroid = /Android/.test(ua);
  const v = /OS (\d+)[_.](\d+)/.exec(ua);
  const iosVersion = isIOS && v ? { major: +v[1], minor: +v[2] } : null;
  return {
    isIOS, isAndroid,
    iosVersion,
    // Chrome on iPhone is "CriOS" but still WebKit: every iOS rule applies to it.
    browser: /CriOS/.test(ua) ? 'chrome-ios' : /FxiOS/.test(ua) ? 'firefox-ios' : isIOS ? 'safari-ios' : /Chrome\//.test(ua) ? 'chrome' : 'other',
    inAppBrowser: IN_APP.test(ua),
  };
}

// WebAssembly SIMD feature test: the smallest module with a v128 instruction (the wasm-feature-detect "simd" bytes). The engine is built with
// -msimd128, so a browser that cannot validate this module cannot load the engine (WebKit: iOS 16.4+).
const SIMD_PROBE = new Uint8Array([0, 97, 115, 109, 1, 0, 0, 0, 1, 5, 1, 96, 0, 1, 123, 3, 2, 1, 0, 10, 10, 1, 8, 0, 65, 0, 253, 15, 253, 98, 11]);
export function supportsWasmSimd(wa = typeof WebAssembly === 'undefined' ? null : WebAssembly) {
  try { return !!wa && typeof wa.validate === 'function' && wa.validate(SIMD_PROBE) === true; } catch (_) { return false; }
}

// Requirements checked before anything is asked. Returns null when everything needed exists, else { code, title, body } (Korean, shown on the start screen).
export function preflight({ win = {}, nav = {}, wa } = {}) {
  const platform = detectPlatform(nav);
  if (win.isSecureContext === false) {
    return { code: 'insecure', title: '보안 연결(HTTPS)이 필요해요', body: '카메라와 모션 센서는 https:// 주소에서만 쓸 수 있어요. 주소를 https로 열어 주세요.' };
  }
  if (!nav.mediaDevices || typeof nav.mediaDevices.getUserMedia !== 'function') {
    return { code: 'no-camera-api', title: '이 브라우저에서는 카메라를 쓸 수 없어요',
      body: platform.isIOS ? 'iPhone은 Safari 또는 Chrome에서 이 주소를 직접 열어 주세요. 앱 안의 미리보기 화면에서는 카메라가 막혀 있어요.' : 'Chrome 같은 최신 브라우저에서 이 주소를 직접 열어 주세요.' };
  }
  if (!supportsWasmSimd(wa)) {
    return { code: 'no-simd', title: '이 기기에서는 엔진을 실행할 수 없어요',
      body: platform.isIOS ? 'iOS 16.4 이상이 필요해요. 설정에서 iOS를 업데이트한 뒤 다시 열어 주세요.' : '브라우저를 최신 버전으로 업데이트한 뒤 다시 열어 주세요.' };
  }
  return null;
}

// Starts every motion-permission request that exists, synchronously (nothing is awaited before the calls), because iOS only accepts them
// inside the user gesture. Returns a promise of { motion, orientation }, each 'granted' | 'denied' | 'unsupported' | 'error'.
// Callers must call this in the click handler before their first await.
export function requestMotionPermissions(win = {}) {
  const start = (Ctor) => {
    if (!Ctor || typeof Ctor.requestPermission !== 'function') return Promise.resolve({ state: 'unsupported' });
    let p;
    try { p = Ctor.requestPermission(); } catch (e) { p = Promise.reject(e); }
    return Promise.resolve(p).then((state) => ({ state: state === 'granted' ? 'granted' : 'denied' }), (error) => ({ state: 'error', error }));
  };
  const motion = start(win.DeviceMotionEvent);
  const orientation = start(win.DeviceOrientationEvent);
  return Promise.all([motion, orientation]).then(([m, o]) => ({ motion: m.state, orientation: o.state, motionError: m.error || null, orientationError: o.error || null }));
}

// Why the permission step failed, in words a user can act on.
export function describeMotionFailure(result, platform) {
  const ios = platform && platform.isIOS;
  if (result.motion === 'denied' || result.orientation === 'denied') {
    return { code: 'motion-denied', title: '모션 센서를 허용해 주세요',
      body: ios ? '카메라가 움직임을 따라가려면 모션 센서가 필요해요. iPhone은 한 번 거절하면 이 페이지를 새로 열어야 다시 물어봐요.' : '움직임을 따라가려면 모션 센서가 필요해요. 주소창의 자물쇠 아이콘에서 모션 센서를 허용해 주세요.',
      action: 'reload' };
  }
  return { code: 'motion-error', title: '모션 센서를 켜지 못했어요',
    body: ios ? '시작 버튼을 직접 눌렀을 때만 권한을 물어볼 수 있어요. 페이지를 새로 연 뒤 시작 버튼을 눌러 주세요.' : '브라우저가 모션 센서 사용을 막았어요. 페이지를 새로 열고 다시 시도해 주세요.',
    action: 'reload' };
}

// Camera failure from xr.js (onException) or getUserMedia, sorted into the cases that have a different fix.
export function describeCameraFailure(error, platform) {
  const text = `${error && error.name || ''} ${error && error.message || error || ''}`;
  const ios = platform && platform.isIOS;
  if (/NotAllowed|Permission|denied|permission/.test(text)) {
    return { code: 'camera-denied', title: '카메라를 허용해 주세요',
      body: ios ? '주소창 왼쪽의 aA 또는 설정 아이콘에서 웹사이트 설정 > 카메라를 허용으로 바꾼 뒤 페이지를 새로 열어 주세요.'
        : '주소창 왼쪽의 자물쇠 아이콘 > 권한 > 카메라를 허용으로 바꾼 뒤 다시 시도해 주세요.', action: 'reload' };
  }
  if (/NotFound|DevicesNotFound/.test(text)) {
    return { code: 'camera-missing', title: '카메라를 찾지 못했어요', body: '후면 카메라가 있는 휴대폰에서 열어 주세요.', action: 'reload' };
  }
  if (/NotReadable|TrackStart|in use/i.test(text)) {
    return { code: 'camera-busy', title: '다른 앱이 카메라를 쓰고 있어요', body: '카메라를 쓰는 다른 앱이나 탭을 닫은 뒤 다시 시도해 주세요.', action: 'reload' };
  }
  return { code: 'camera-error', title: '카메라나 엔진을 시작하지 못했어요', body: '연결을 확인하고 다시 시도해 주세요. 계속되면 아래 자세히 보기의 내용을 알려 주세요.', action: 'reload' };
}
