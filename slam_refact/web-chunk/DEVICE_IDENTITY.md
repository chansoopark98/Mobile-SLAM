# Opt-in 기기 식별 (P1-I1)

- `XR8`가 chunk factory를 처음 만들기 **전에** 설정한다. 기본값은 OFF이며 기존 SDK의 `deviceEstimate()` 값을 그대로 쓴다.

```js
window.SLAM_REFACT_CHUNK_OPTIONS = {
  ...window.SLAM_REFACT_CHUNK_OPTIONS,
  deviceIdentity: {
    enabled: true,
    fallback: { manufacturer: 'Samsung', model: 'SM-F741N', profileId: 'optional-label' },
    timeoutMs: 1000,
  },
};
```

- 선택 순서: `navigator.userAgentData.getHighEntropyValues(['model'])`의 유효 model → 명시 `fallback` → 세션 생성 시 SDK `deviceEstimate()`.
- UA가 model만 제공하므로 선택된 UA model의 manufacturer는 빈 문자열이다. 제조사 추론은 추가하지 않는다.
- `fallback`은 선택사항. model은 비어 있지 않은 문자열, manufacturer/profileId는 선택 문자열이다. 문자열은 최대 128자이며 제어문자를 허용하지 않는다.
- timeout 기본 1,000ms, 허용 1–5,000ms. opt-in 초기화 예산이며 실시간 추적 성능 목표가 아니다. API 미지원·거부·빈 model·timeout이면 fallback을 사용한다.
- controller는 opt-in 조회가 끝난 뒤 반환한다. 늦은 UA 응답이나 이후 설정 객체 변경은 이미 선택된 identity와 resize/recenter 세션을 바꾸지 않는다. identity 변경은 새 factory/page lifecycle에서 한다. `XrController.configure()`의 runtime 옵션이 아니다.
- 기본 경로에는 UA 조회나 추가 비동기 대기가 없다. 새 runtime 파일/import/export를 추가하지 않는다.
- 읽기: `window.__slamRefactChunk.session.device`의 model/manufacturer/source, `session.glueConfig`의 실제 constructor 입력. `__slamRefactChunk.deviceIdentity`에는 조회 결과·timeout·선택된 fallback provenance를 기록한다.
- `profileId`는 식별 입력의 이름일 뿐 검증된 calibration이 아니다. K/distortion을 주입하거나 `accuracy`, BA, LC, metric 출력 옵션을 켜지 않는다. 현재 XR의 accuracy 설정 미전달은 그대로다.
- 구 WASM도 constructor model 입력을 받는다. capture K·compile-feature getter가 없으면 P0 readback은 계속 `null`/`unknown`이며 model로 실제 K나 컴파일 여부를 추론하지 않는다.
- fresh P0 WASM direct-binding 시험에서만 `accuracy.enabled=true`인 Flip6 model이 540 추정 focal을 선택하는지 확인한다. 이는 정확도 개선·실측 calibration·XR 배포 수락이 아니다.
- API 근거: [UA Client Hints getHighEntropyValues](https://wicg.github.io/ua-client-hints/#getHighEntropyValues) (2026-10-30 조회). UA가 model을 제공하지 않는 경우를 허용한다.
