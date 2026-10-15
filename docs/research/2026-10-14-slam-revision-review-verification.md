# U/R 구현 전 검증·회귀 리뷰 — 2026-10-14

- 역할: 작성자와 분리된 pre-implementation reviewer. 계획 수정·제품 구현 없음.
- 대상: `2026-10-14-slam-improvement-plan.md`의 `후속 실행 계약` U/R. 전체 SLAM recovery·GPU화·기본 estimator 정책 승격은 이번 승인 범위 밖.
- 판정: **APPROVE — U/R의 명시된 JS 표시·개발용 replay 범위**. 8th Wall/ARCore 심화 문서와 최종 RE→채택/보류 표를 읽었으며 blocking correction 없음. 제품 구현 완료·실기기 성능/정확도 통과 판정은 아님.
- 확인: 현재 App/Renderer, Wrapper/Worker timing 경계, synthetic raw batch 함수·392행 보존 증거, 기존 browser UI/synthetic fixtures.

## U의 검증 가능성

- 재현된 결함: `_consumeResult()`가 null과 동일 identity에서 즉시 return; 과거 Tracking·age·frustum이 갱신되지 않음. 결과 payload 수정 없이 표시 갱신과 pose 적용을 분리할 수 있음.
- 독립 red/green 기대값: 같은 result의 age100/500ms에서는 현재 frustum 표시,501ms/5초에서는 숨김. 경계 검사에는 fixture clock과 timestamp의 정확한 관계를 명시;500ms를1e-6 완화하지 않음.
- 10Hz는 동일 결과의 표시 text 갱신 제한. 새result·stale/error 상태 전이는 throttle를 우회해야 하므로501ms stale 테스트를 직전500ms UI write 다음1ms에 실행할 가치가 있음. 이는 계약에 이미 요구된 즉시 전이의 직접 검증.
- 반복 tick마다 Renderer.updateCameraPose/updateMapPoints를 호출하면 trajectory append·map upload가 증가. 동일 result의 pose/map 적용1회와 map/trajectory 보존을 별도 assertion으로 검사.
- 새sequence에 stale pose가 와도 표시하지 않기, 음수/NaN age가 Tracking으로 나오지 않기, null→Waiting과 현재 worker=null/terminated·configured=false·failure metric의 동시 조건→Worker error 구분.
- 과거 workerErrors/lastError는 현재 failure 판정의 충분조건이 아님. failure→새Worker 복구→configured=true fresh→같은 살아 있는Worker에서 정상reset/configure대기(configured=false) 순서를 검사; 마지막은 Worker error가 아니라 Waiting/reset. recovery의 새 fresh result는 pose/map을 정확히1회 적용해야 함. architecture review 지적이 수정된 U 계약에 반영됨을 확인.
- Explicit Lost/Unknown/invalid Limited·epoch/reset clear는 기존 의미 유지. stale/임시null에서 frustum hide와 full clear를 섞으면 과거 map/trajectory 보존 계약 위반.
- `tests/browser/mobile-source-contracts.test.mjs:194,210,223`는 poseTimestamp1s와 기본 now5s를 섞음. 기존 정상/Unknown/Lost/Limited assertion을 유지한 채 clock/pose/input timestamp를 정합시켜야 함. assertion 삭제·상태를전부stale로 바꿔 통과시키는 방식은 부적절.
- 기존 result-source timing 테스트의 pose4.8s/now5s→age200ms 의미는 유지; App.lastFrameTiming의 더 새로운 source에 age를 연결하지 않음.
- 500ms는 display 정책이며 estimator pose timestamp/status/fresh를 바꾸지 않음. UI stale count를 engine Lost 또는 GT accuracy 분모에서 빼지 않는 계약은 적절.

## R의 검증 가능성

- 원인 증거: captured-counts 직렬 replay의50개 missing_bracket은 pre-submit lastIMU<imageTimestamp50개와 정확히 일치. live는 해당50개 모두 frame 대기 중 추가 endpoint 수신.
- future-bracket 기본값은 기존 `imuBatchRanges()` 재사용으로 구현 가능. 입력 JSON mutation 없이 선택 정책과 batch를 report에 남겨야 함.
- 독립 fixture 기대값: frame0.1/0.2s와 IMU0/0.15/0.2/0.25s이면 future-batch `(start0,count2)`, `(start2,count2)`. 첫 frame의0.15s future sample을 다음 frame에 다시 보내지 않음. 중간0.12s frame을 넣으면 이미 보낸 future가 덮으므로 count0가 유효.
- image endpoint와 exact-equal IMU, 인접 binary64 timestamp, endpoint를 아직 덮는 future carry, 마지막 sample 소진·비단조 input의 기존 거부를 포함. timestamp round/epsilon 추가로 테스트 통과시키지 않음.
- captured-counts는 명시 diagnostic 선택만 허용하고 input counts 존재/정수/단조/범위 검사 유지. 잘못된 policy·counts없음·live에서 captured-counts 요청을 실행 전 거부.
- 과거 `Captured replay preserves submitted IMU membership...` 테스트를 삭제하지 않고 captured-counts 명시 분기의 회귀로 유지. 같은 fixture에 future-bracket 기본 분기 기대값을 추가하면 변경 의도가 독립적으로 검증됨.
- 실제392행 fixture: default future-bracket은 frozen control과 sourceIndex/input/pose timestamp/status/reason/fresh/valid/pose16/epoch exact 비교, fresh347/current failure6/missing0. captured diagnostic은 historical failure50개를 재현·보존.
- float exact 비교는 raw estimator 값에 적용. wall timings·browser jitter·새 report metadata까지 exact 동일을 요구하는 것은 의도와 다름. solver time-budget 때문에 결과가 바뀌면 조용히 tolerance/fixture 분모를 변경하지 않고 실패·환경 차이를 보고.
- 수학적 replay는 future sample을 선제 제공하는 availability control. live arrival schedule을 복원했다는 문구 금지. 현재 recorded input에 arrival metadata가 없다는 한계를 정확히 유지.

## 적정 검증 범위·성능 해석

- U/R의 실제 회귀를 red로 확인한 뒤 구현; 해당 tests→전체 browser contracts→syntax→실제392행 raw/captured 대조→실제 App smoke 순서가 적절.
- source/loader/effective-profile/pixel/IMU hash·400planned/392accepted 분모를 보존. main IMU 경로·Worker·C++·active WASM·failed control 수정은 U/R 범위 밖.
- JS UI/replay 변경에 full native/TUM/새WASM build를 무조건 요구하지 않음. 이미 확보한 frozen isolated loader를 사용해 실제 Worker output 보존을 확인하면 이번 변경 경계에 맞음.
- 새 fresh-source WASM synthetic App run은 연결 smoke/상태 회귀로 보고; 통과해도 phone timing·발열·render latency 개선 또는 물리 정확도 해결 주장은 불가.
- 10Hz 검증은 deterministic clock으로 반복 status-write 개수와 bypass 동작을 확인. 이 검사가 실제 UI CPU 시간을 줄였다는 성능 실험은 아님. renderer frustum hide가 기존 render loop·follow-camera smoothing·estimator pose를 변경하지 않는지 diff 확인.
- 예상 구현 산출물의 성공 기준은 stale/error 표시와 artificial missing-bracket 제거. 고회전 visual-support Lost·calibration·model recovery 미해결을 그대로 유지.

## 심화 RE 대조 및 pre-change baseline

- `2026-10-14-8thwall-deep-runtime.md` 전체 검토: 원본 code section을 보존한3 residual의79조건 수치 검증이며 full tracker/IMU fusion/recovery 성능 증거가 아님. Snavely의boolean1/nonfinite 경계, frame-step translation regularizer의dt 의존, scalar alignment와metric initialization의 구분을 확인.
- 이 결과에서 UI500ms·10Hz·수학적 raw future-bracket 정책을 vendor 상수로 유도할 수 없음. U/R은 자체 재현 결함과 명시된 표시/재생 계약으로 정당화되며, smoothness residual 추가·positive-depth/current-support gate 완화는 승인 범위 밖이라는 판단을 유지.
- 현재 변경 전 baseline 실행: `node --test tests/browser/mobile-source-contracts.test.mjs tests/browser/synthetic-input-contracts.test.mjs` **38/38 PASS**,fail0,skip0,TODO0. 현 suite가 stale5초/Worker null/error와 새 default replay 정책을 이미 검증한다는 뜻은 아님; 명시 red 추가가 필요.

- `2026-10-14-arcore-deep-runtime.md` 전체 검토: 설치 ARM64 service의46개 bounded 함수/200명령, cached timestamp/state와pose/intrinsics 별도 dispatch, sensor 설정mutex/condition wait, future/역순/초기시각 guard, atomic reference-count release를 확인. 이것은 정적 경계 분석이며 full estimator/runtime branch/latency 수식 복원이 아님.
- 특히 frame timestamp의 time base를 API가 확정하지 않는다는 한계와 source/getter/solver 구분을 유지. UI에서 과거 cached pose를 fresh로 취급하지 않는 U, recorded arrival와 미래endpoint 확보를 구분하는 R의 설계 방향과 모순 없음.
- ARCore의 일부 constant-value extrapolation diagnostic은 Mobile-SLAM의 bracket 검사를 없애거나 timestamp를 덮어쓰는 근거가 아님. 최종 계획이 future clock/monotonic 검사를 유지하고 extrapolation/geometry/smoothing 기본 채택을 보류한 판단은 적절.
- 최종 RE→채택/보류 표의 각 행은 성분 실행/정적 관측과 제품 적용을 분리함. 이번 승인은 U/R의 재현 결함 수정만 포함하며, vendor private threshold·500ms·10Hz를 동일시하지 않음.

## 최종 결정

- **APPROVE**. Blocking correction0. 앞의 독립 기대값은 이미 승인 대상 계약을 구체화한 구현 검증 지침이며 새로운 scope/추가 승인 gate가 아님.
- scope: App/Renderer의 display freshness·failure 표시, synthetic raw replay 기본 endpoint 공급·과거 count diagnostic 명시화, 관련 browser tests/runbook.
- 보존: estimator·IMU source ordering/units·WASM/default policy·frozen392/50failure artifacts·기존GT/phone 실패. binary 전체 알고리즘 도입·새 성능 목표 달성 주장은 범위 밖.
- 다음 단계: 해당 red→green 구현과 계획된 browser/syntax/실제 frozen replay 검증. 현재 reviewer의 baseline38PASS는 새 구현 수락 증거가 아님.

### 검토한 문서 fingerprint

- `docs/research/2026-10-14-slam-improvement-plan.md` SHA-256 `58c7c4036794a4d8a7572623486bb3bed841f684c3e6991cec20c59169fb6433`.
- `docs/research/2026-10-14-8thwall-deep-runtime.md` SHA-256 `9c37a0aaeebfc1281833b0951644b58825dd752fb14a8fd5c42284c5577dd558`.
- `docs/research/2026-10-14-arcore-deep-runtime.md` SHA-256 `e20d904c44e0649b675dfdeef414df757b6f1ad45e73b778b36a87481bb8bcc6`.

## Frozen gate ACK 및 최종 결과 표본 QA

- **ACK / APPROVE 유지**: `build/slam-revision-20261014/review-gate/2026-10-14-slam-improvement-plan.md`를 다시 읽고 실제 SHA-256 `8624759dd1d5c69b7f76c40ad6220f92c5192d15619ce65a9ef8532063af22b5`를 확인. `review-gate/approval.json`의 plan hash와 일치.
- 위 최초 검토 fingerprint 이후의 의미 정정은 ARCore 역순 buffer 종류를 전체IMU로 단정하지 않는 귀속 제한,10Hz가 동일result 주기text 갱신에만 적용된다는 명료화. U/R의500ms·즉시전이·dedup·future-bracket/default·captured diagnostic 동작과 승인 범위는 같음. 과거 approval/frozen receipt를 다시 쓰지 않고 이 후속 ACK만 추가.
- `browser-final.log` 원문 확인: browser297/297PASS,fail0,skip0,TODO0. 이어 exporter Python3개 모두OK. 이 확인은 root 실행 log 검토이며 reviewer가297개를 재실행했다는 주장이 아님.
- `replay/default-exact-comparison.json`:392행19필드 mismatch0,default347fresh·6current failure·0missing,loader/input/profile 일치. `captured-exact-comparison.json`:392행19필드 mismatch0,299fresh·50missing 보존. 이미 다른 컨텍스트 intent reviewer가 원시 JSON도 직접 대조한 receipt와 일치.
- `app-current-fast707/report.json`:completed=true,400planned/392accepted/352fresh,postwarmup297/300,pageErrors0. report SHA `94010567e5d2b3bc507ad6eaa3f7c9ab0d1c000e0ebddb4f21c03e3ad14b9467`가 evaluation의 inputs.results_sha256과 일치.
- 동일 evaluation: passed=true,failures0,SE3 scale1·352/352 associated,ATE RMSE0.0461956714m,rotation APE RMSE1.8536916061°,1sRPE0.0268944812m/0.8587523011°. **해당 synthetic rig/run GT gate 통과**이며 phone/field/full-cohort 수락이 아님.
- `app-active-clean120/report.json`:completed=true,120planned/108accepted/54fresh,pageErrors0,epoch transitions0. 이는 active loader의 실행·UI 연결 관측이며120-frame run의GT 통과 주장은 하지 않음.
- 새로운 App run의352fresh와 frozen replay의347fresh는 별개 실행·스케줄이다. 이 차이를 U의 표시 수정이 estimator 정확도/추적을 개선했다는 causal 증거로 표현하지 않기. U/R 수락은 각각 표시 계약과 frozen replay identity로 판단.
- **표본 QA 결과: material issue0**. 기존 physical GT·native geometry recovery·실기기 지연/열 미검증 한계 유지. 제품/계획/동결증거 수정0; 이 리뷰 문서에 ACK/QA만 추가.
