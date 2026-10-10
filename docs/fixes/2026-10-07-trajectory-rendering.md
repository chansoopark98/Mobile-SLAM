# 모바일 궤적 캔버스 크기 수정

- 범위: `web/js/renderer.js`, `web/js/test-tumvi-app.js`, `web/js/app.js`, `web/index.html`; 추정 알고리즘·WASM·데이터셋·HTTPS 서비스 변경 제외
- 원인 후보: Three.js `setSize`의 inline CSS가 flex 높이 고정; `setSize` 이후 canvas 속성 직접 변경으로 DPR drawing buffer와 GL viewport 불일치
- 수정 순서: Chrome DPR1/3 회귀 재현 → CSS 크기 기준 renderer 초기화·`setSize(..., false)` → canvas 속성 중복 쓰기 제거 → 요소 resize 관측 → 같은 시나리오 재검증
- 검증: `node tests/browser/renderer-resize-browser.mjs --record-baseline`, `node tests/browser/renderer-resize-browser.mjs`, `bash scripts/dev/check.sh syntax`, `bash scripts/dev/check.sh browser-contracts`
- 수락 조건: 창 크기·상태문구·화면 방향 변경 후 CSS/논리 크기 일치, GL viewport/drawing buffer 일치, injected pose 선 픽셀 확인
- 한계: desktop Chrome mobile viewport와 합성 pose의 렌더링 검증; Galaxy Z Flip 6 센서·초기화·실제 궤적 정확도 미검증

## 실행 결과

- Chrome `144.0.7559.96` / Playwright `1.63.0`; 기존 `https://dev.serdic.com:7002` strict TLS 접속, SwiftShader WebGL
- 변경 전: 8개 시나리오 / 실패 조건 20건; [`before.json`](../../build/trajectory-rendering/before.json), `before-*.png`에 source SHA256·크기·GL viewport·픽셀 수 기록
- 변경 후: 14개 시나리오 / 실패 0건; [`after.json`](../../build/trajectory-rendering/after.json), `after-*.png`
- main: DPR1/3 × 최초 `412×915`, 높이 `412×800`, 상태문구 줄바꿈, 가로 `915×412`
- TUM factory: DPR1/3 × 최초 `1280×800`, resize `1000×650`, 상태문구 줄바꿈; dataset 재생 없이 합성 pose 주입
- 매 시나리오: 16개 합성 pose, trajectory 표시/숨김 간 녹색 픽셀 차이 >10, CSS/논리 크기·camera aspect·DPR buffer·GL viewport 일치
- 추가 검증: 크기 동일·0·NaN/Infinity 입력 시 canvas 재할당 0회, 기존 buffer/aspect 보존
- `bash scripts/dev/check.sh syntax`: 종료 0; JS syntax 검사, 별도 lint/typecheck 설정 없음
- `bash scripts/dev/check.sh browser-contracts`: Node 49 pass / fail·skip·TODO 0, Python exporter 3 pass
- `node --check tests/browser/renderer-resize-browser.mjs`, `git diff --check`: 종료 0
- 전후 screenshot 직접 확인; `.omx/state/trajectory-rendering/ralph-progress.json` 시각 판정 96 / pass

| DPR3 main 상태 | 변경 전 buffer / GL viewport | 변경 후 buffer / GL viewport |
|---|---|---|
| 높이 축소 | `412×507` / `824×1014` | `824×876` / `824×876` |
| 상태문구 줄바꿈 | `412×507` / `824×1014` | `824×842` / `824×842` |
| 가로 화면 | `915×507` / `1830×1014` | `1830×410` / `1830×410` |

- 단순화: renderer 초기화의 factory/constructor 중복 sizing 제거; app/TUM의 backing-buffer 수동 쓰기 제거
- 검증 한계: WebGPU 미실행; 실기기 `imu_missing_past`와 `initializing` 원인은 별도 IMU 검증 대상. 본 렌더링 통과로 실제 pose 생성·센서 clock·정확도 통과 주장 제외
