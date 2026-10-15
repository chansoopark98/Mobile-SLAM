# U/R 구현 독립 검토 — intent axis

- 판정: **PASS — 승인된 U/R 의도 충족, blocking defect0**.
- 기준: `build/slam-revision-20261014/review-gate/`의 frozen 계획·3축 승인 receipt, 현재 개선 계획, tracked diff7파일.
- 검토: architecture/correctness 관점에서 승인 의도와 구현·회귀·raw 결과를 대조. `code-review` 체크리스트 적용; 재귀 위임·제품/계획 수정 없음.
- 범위: UI display freshness/error, mathematical raw replay policy. optimizer·geometry·실시간 센서·active WASM·배포 변경 검토가 아님.

## 7개 파일의 의도 일치

| 파일 | 확인한 변경 | 판정 |
| --- | --- | --- |
| `web/js/app.js` | 동일 result에서도 age/state 재평가; pose/map 반영은 identity별1회; inclusive500ms·error/stale transition·steady10Hz | PASS |
| `web/js/renderer.js` | 현재 frustum visibility만 변경하는 `hideCameraPose`; 기존 clear 유지 | PASS |
| `scripts/dev/synthetic-browser.mjs` | raw default future-bracket, 명시 captured-counts diagnostic, raw-only validation, policy provenance | PASS |
| `tests/browser/mobile-source-contracts.test.mjs` |500/501ms·null/error·history·정상reset·현재Limited/Lost/Unknown/epoch assertion | PASS |
| `tests/browser/video-frame-contracts.test.mjs` | null result에서도 수행되는 status 관측에 필요한 IMU diagnostics fixture 추가 | PASS |
| `tests/browser/synthetic-input-contracts.test.mjs` | 명시 과거정책·default·float endpoint·zero batch·malformed/CLI 거부·input 불변 | PASS |
| `scripts/dev/README.md` | 실제 IMU 값/순서와 공급 batch schedule 구분, 두 policy의 한계·재현 설명 | PASS |

## U 의도·경계

- `app.js:1335–1391`: 입력 result의 timestamp/status/poseFresh/quality/pose/map을 쓰지 않고 읽기만 함. displayFresh가 estimator result나 정확도 population을 변경하지 않음.
- `0 <= age <= 500`은 표시 허용,501ms부터 즉시 Stale/hide. 새로운 result 또는 표시상태 변경은 주기 throttle을 우회하며 같은 상태의 반복 text만100ms 간격으로 제한.
- 정상 pose/map/trajectory 적용은 `newResult` 안에 유지. 반복 status tick이 trajectory append를 늘리지 않음.
- 같은 epoch의 stale/new-old/null/current-worker-failure는 hide만 수행하여 historical trajectory/map 보존. 새 invalid producer state·명시 Lost/Unknown·invalid Limited 및 epoch 전환의 기존 clear 동작 유지.
- `renderer.js::hideCameraPose`는 frustum visibility만 false로 변경. buffer/draw range/과거 transform을 지우지 않음. 정상 새 result는 기존 `updateCameraPose`의 visible=true로 복귀.
- 현재 failure 조건은 `worker===null && configured===false && workerErrors>0`. 누적 error가 있어도 worker가 있는 정상 reset/configure 대기는 Waiting. 기존 Wrapper 자체 `_workerFailure`를 부른 회귀가 null invalidation과 reset clear까지 검사함.
- 기존 dedup/Lost/Limited 테스트는 now5000/pose1s의4초 stale fixture를 now1000으로 명시 정렬. 기존 의미 assertions를 삭제·스킵하지 않았으며 source age200ms·미래 IMU clock 검사는 유지.
- `ui/receipt.json`: red27건 중6실패 보존, target green78/78·skip0. 해당4파일의 현 SHA를 receipt와 직접 대조해 일치 확인.

## R 의도·동등성

- raw default는 captured counts 존재와 무관하게 기존 `imuBatchRanges`를 사용. input의 captured counts metadata를 삭제·수정하지 않음.
- explicit `captured-counts`는 검증된 counts가 있어야 실행. 잘못된 enum·live의 명시 policy는 input/output/browser 획득 전 거부. malformed count validation도 유지·보강됨.
- report에 requested/selected policy와 각 frame의 submittedIMUStart/Count 기록. 두 정책 모두 not-live-schedule 한계를 명시하여 수학적 입력 availability와 물리 arrival 재현을 혼동하지 않음.
- 신규 diff에는 IMU 값·timestamp·pixel·effective profile 변경 없음. 기존 rotationRescue profile 처리의 의미도 변경하지 않음.
- input 원본1201 IMU 중 default1199개·diagnostic1198개 prefix 사용은 각 frozen control과 동일. 뒤쪽2/3개 미사용 tail은 원본에 보존되어 있으며 sample 삭제가 아님.

### 독립 JSON 대조

- 현 implementation receipt7개 파일 SHA 모두 일치.
- `default-future-bracket/report.json` ↔ frozen `current-fast707-future-bracket/report.json`: **392행 ×19필드 exact 차이0**, effectiveProfile 동일.
- `explicit-captured-counts/report.json` ↔ frozen `current-fast707-matched-raw/report.json`: **392행 ×19필드 exact 차이0**, effectiveProfile 동일.
- 비교 필드: sourceIndex/sourceFrameIndex, input/physical/pose/timestamp, status/trackingState/reason, fresh/valid, pose16값, engine/client epoch, engine frame time, initialized, IMU endpoint, submitted batch start/count.
- default347fresh·6current-support failure·0missing; diagnostic299fresh·50missing 보존. fixed400 planned/392 accepted 분모와 warmup300 기준도 receipt에서 분리.
- `replay/implementation-receipt.json`: red21건 중4실패, green21/21·skip0. 독립 기대값 endpoint/count test와 실제 Worker replay는 서로 다른 증거로 사용.

## 최종 검증 경계

- 이번 reviewer 실행: diff7파일·frozen 계획·current source 확인, receipt SHA 대조, 실제 두 replay JSON의392행×19필드 재대조, `git diff --check` 통과. 동일 대형 replay·full suite를 중복 실행하지 않음.
- root 소유 full browser/E2E 최종 결과는 별도 integration receipt로 확인해야 함. 이 PASS는 계획 의도·현재 구현·검토한 증거의 일치에 대한 판정이며 전체 integration 완료 선언이 아님.
- 신규 phone collection·physical performance·GT 향상·자체 Lost 해결·상용 SDK 동급 주장 없음. 미래 endpoint 제공으로 제거한50건은 재생 도구의 인위적 실패이며 metric geometry6건은 그대로 남음.

## 검토 snapshot

- `build/slam-revision-20261014/review-gate/2026-10-14-slam-improvement-plan.md` SHA256 `8624759dd1d5c69b7f76c40ad6220f92c5192d15619ce65a9ef8532063af22b5`.
- `web/js/app.js` SHA256 `c8454d99aea7c75b6c6552919cdbcad5b6f14bf0d488520e7c7103fd73ccc481`.
- `web/js/renderer.js` SHA256 `ebbf1c75e2411170178502e07bd984965db807c9a8b8bb287862f6eb3c7e97e6`.
- `scripts/dev/synthetic-browser.mjs` SHA256 `a778285bf32482fe61a31a0e5607b33c0f9a1f8bc07d8b9b4d38662e2c9ad615`.
