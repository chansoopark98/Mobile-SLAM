# 저특징 브라우저 구현 독립 코드 검토

- 날짜: 2026-10-14. 판정: **APPROVE — 검토한 브라우저 코드·계약 범위의 미해결 blocker 없음**.
- 기준 계획: `build/low-feature-20261014/plan-v3.md`, SHA256 `ce6e91442165d9ba5e93258958963a2b24dc262d177693bcc06418e687c46108`.
- 범위: `low-feature.mjs`, Worker/Wrapper, App/Renderer, Android recorder/snapshot, Android replay/synthetic harness의 제품9파일 및 관련 계약 시험. 다른 최종 reviewer 결과는 읽지 않음.
- 제품 파일은 읽기 전용 검토. 확인한 전송 예외 누락은 담당 구현자에게 전달하여 수정받았으며 reviewer는 이 문서와 `build/low-feature-20261014/reviews/browser-*` 증거만 작성. 기존 dirty·WASM·서비스 변경 없음.

## 발견 후 해소한 결함

- **[P2 / 해결] frame postMessage 실패 뒤 old IMU anchor 재사용 가능성.** 초기 [sendFrame catch](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/web/js/vio-wrapper.js:240)는 local result/map만 제거하고 predictor invalidation을 전달하지 않았음. core가 받지 못한 frame 후 다음 blank frame에서 이전 seed가 재사용될 수 있었고, 이는 frame/transport error 뒤 새 clean visual seed를 요구하는 v3 수명 계약과 불일치.
- 재현: 첫 prediction 수신→FakeWorker가 다음 `frame`의 `postMessage`에서 예외→다음 정상 blank reply. 새 회귀 [transport.test.mjs](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/tests/browser/transport.test.mjs:765)의 invalidation RPC assertion이 초기 실행에서 실패. `browser-contracts-initial.log`:204 tests,203 pass,1 fail.
- 담당자 수정: catch의 [244행](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/web/js/vio-wrapper.js:244)에 `_invalidateLowFeature()` 추가. Wrapper quarantine 및 Worker predictor-only invalidation을 같은 경로로 사용. 같은 회귀를 포함한 최종204/204 PASS. 초기 실패 증거 보존.

## 확인한 계약

| 경계 | 코드 확인·시험 근거 | 판정 |
| --- | --- | --- |
| 지연/취소 reply | requestId/clientEpoch/sequence 매칭 후 수락. locally rejected IMU는 fence sequence를 설정하고 이전 in-flight result를 차단. 더 늦은 clean visual만 quarantine 해제. shared `isCleanVisualContext`가 current timestamp·fresh/valid·homogeneous SO(3)까지 확인 | PASS |
| engine 미호출 실패 | Worker missing_bracket/input rejection에서 shadow만 무효화. missing bracket→complete IMU+blank에서 old seed 재개0, metric reset0의 명시 회귀 | PASS |
| malformed/quality | exact schema·finite/정상 SO(3)·epoch·input/engine timestamp·허용 Lost reason·빈 solverQualityReason 검증. warning/transport loss/unknown/부분 ABI/invalid metadata는 null | PASS |
| 구형 WASM | getter 없음 또는 getter만 있고 invalidate API 없음이면 prediction getter 미호출. 정상 metric 경로는 유지. floor getter 없으면 일반 map을 대체 입력으로 쓰지 않음 | PASS |
| plane 계보 | Worker 단일 tracker, 최대512점·64후보, 5Hz source-time fitting. Lost에서는 getter/fit 호출0, 기존 sourceTimestamp/sourceSequence 보존, ambiguous/nonfinite/error/epoch에서 clear | PASS |
| 표시 TTL | [App](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/web/js/app.js:1368)가 매 RAF source age100ms·prediction horizon0.5s·plane age1초를 독립 검사. 도착시각으로 수명 재시작0. reset/Worker error/visibility/blur는 추가 layer 제거 | PASS |
| metric 분리 | Wrapper의 pose/map fields는 기존 visual gate로만 갱신. prediction은 `_latestMapPoints`·`lastPublishedPose`·trajectory·follow pose에 사용하지 않음. harness의 기존 visual coverage 요약이 side metadata 유무에 불변 | PASS |
| renderer 좌표 | row-major pose를 `Matrix4.set`에 전달하고 prediction/plane을 worldRoot 자식으로만 배치. 실제 Renderer 메서드와 pinned Three math에서 `(1,2,3)→(1,3,−2)`, floorZ0→displayY0, 추가 회전0 확인 | PASS |
| recorder/harness | side payload를 bounded copy로 별도 저장, 기존 pose fields/scorer 입력은 불변. observed/retained/available/displayed 집계 분리; raw replay는 display count=null. `.mjs` fixture route와 기존 server allowlist 제공 | PASS |

## 실행 증거

- [receipt](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/low-feature-20261014/reviews/browser-review-receipt.json): 실제 Node `v22.21.1`, 제품9파일·시험8파일 SHA256, syntax9/9, 초기 실패와 수정 후 결과, Renderer 수치 probe 포함.
- 집중 회귀:7개 test 파일, **204 tests /204 PASS /0 FAIL /0 SKIP /0 TODO**. [최종 원문](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/low-feature-20261014/reviews/browser-contracts-final.log).
- [독립 Renderer probe](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/low-feature-20261014/reviews/browser-renderer-probe.mjs): 실제 `updateInertialPrediction`/`updateGroundPlane`/hide 메서드를 pinned Three0.170.0 math 객체로 실행. world pose·floor corners·observed/retained 색·opacity·hide 상태·metric `_trajCount=7` 및 follow pose object 불변 확인. [결과](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/low-feature-20261014/reviews/browser-renderer-probe.json).

```bash
node --test tests/browser/low-feature.test.mjs tests/browser/low-feature-harness.test.mjs \
  tests/browser/transport.test.mjs tests/browser/mobile-source-contracts.test.mjs \
  tests/browser/video-frame-contracts.test.mjs tests/browser/android-input-record.test.mjs \
  tests/browser/android-snapshot-record.test.mjs
node build/low-feature-20261014/reviews/browser-renderer-probe.mjs
```

## 검증 한계·유지보수 판단

- shared validator·Worker 단일 plane owner·predictor-only invalidation을 재사용하여 protocol/TTL 상태의 중복을 제한함. 신규 runtime dependency 없음. native 수학/gap 검증은 별도 core 검토 범위.
- 위204개는 contract/component 시험이며 새 WASM의 실제 영상 통합 GT 수락이 아님. Renderer probe도 WebGL pixel 판정이나 full App/WASM 실행을 대신하지 않음.
- 신규 WASM ABI·floor observed≥40/표시≥100·observed-only90% GT·blank prediction 수치·metric/window/prior 비간섭의 최종 수락은 상위 실행 증거로 별도 확인해야 함. 현재 실제 phone/field 정확도·semantic floor·고속 tracking 회복 PASS를 주장하지 않음.
- 검토 종료 시 재현된 미해결 결함 없음. evidence receipt 이후 위 제품 파일이 변경되면 해당 변경의 영향 범위는 다시 검토해야 함.
