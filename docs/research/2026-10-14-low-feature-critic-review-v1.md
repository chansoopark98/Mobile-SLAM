# 저특징·바닥 후보 plan-v1 독립 Critic 리뷰

- 판정: **REQUEST CHANGES**. 선택안 B의 방향은 타당하나 아래 C1–C5를 계획에 고정한 후 구현 가능.
- 대상: `build/low-feature-20261014/plan-v1.md`, SHA256 `529b165709e1c70b9942e149516a5601f64da056ab150532151a9a0ecf26b7bc`.
- 독립성: Architect 및 다른 review 출력은 읽지 않음. 지정된 reference/core/browser/test-design 문서와 실제 소스만 대조.
- 변경 범위: 이 새 리뷰 문서만. 제품·계획·기존 dirty 파일·dataset·WASM·서비스 변경 없음. 구현/시험 실행 결과를 주장하지 않음.
- 실행 승인: 사용자의 분석→계획→구현 요청은 유지. 추가 사용자 승인을 요구하는 판정이 아니며 Planner가 계약을 보완할 사안.

## 유지할 원칙·대안

- B: 마지막 clean visual anchor의 독립 shadow state, 별도 prediction 출력, 현재 관측점의 기하 수평면 후보를 채택할 근거가 충분함.
- A: 실패 후 mutable backend P/R을 정상 pose로 내보내는 대안은 seed lineage와 기존 pose 의미를 파괴하므로 기각 유지.
- C: floor prior/height clamp/optimizer 결합은 이번 표시 기능에 불필요하며 관측성·상관관계 검증도 없으므로 보류 유지.
- IMU 위치·회전 전파 모두 새로운 visual observation이 아님. 0.5초·5m·100ms는 운영 상한이며 covariance·정확도 보증이 아님. 이번 범위에 covariance 구현을 강제할 필요는 없지만 `uncertaintyAvailable:false`에 해당하는 의미를 protocol과 표시에서 유지해야 함.
- geometry만으로 table과 raised floor를 구분할 수 없음. 현재의 `바닥 후보(수평면, 분류 미확인)`과 같은 제한 표시, 동일 입력/상이한 semantic oracle 시험을 유지.
- 현재 Lost를 그대로 보고하고 prediction coverage를 별도 집계하는 원칙, full-cohort GT 실패 보존, server 시작·중지 사용자 소유 원칙을 유지.

## C1 — 원본 IMU gap과 보간 substep을 구분해야 함

- 대상: plan-v1:30–32,63. **차단 이유:** `feedIMU`가 받는 dt를 0.1초와 비교하는 것만으로는 선언한 sample-gap 상한을 보장할 수 없음.
- 실제 코드: [processIMUData](../../src/vio_engine.cpp#L187)는 마지막 처리 endpoint와 pending right sample 사이를 보간하고, 그 endpoint를 다음 frame의 `current_time_`/`prev_acc_`로 보관함. [feedIMU](../../src/vio_engine.cpp#L175)는 원본 bracket의 시간/출처를 받지 않음.
- 반례: 마지막 실제 sample `t=0`, 다음 실제 sample `t=0.20`, frame `t=.05,.10,.15`. 보간 전파 dt는 매번 .05초라서 단순 feed dt gate는 통과하지만 실제 source gap은 .20초로 정책 위반. 아직 소비하지 않은 future sample이 있다는 사실도 gap을 해결하지 않음.
- 계획 수정:
  - 원본 real-sample left/right timestamp lineage를 보간 endpoint와 분리해서 유지.
  - prediction 구간이 소비/보간하는 원본 bracket의 gap을 검사. anchor가 bracket 내부에 있어도 원본 gap을 보존.
  - 새 frame·zero-dt·빈 IMU batch·carried future sample이 raw-gap 기준을 재시작하지 않음.
  - predictor만 fail-closed하고 기존 engine IMU 처리/reset 정책은 변경하지 않음. 새 clean visual seed의 자격도 해당 endpoint의 raw bracket 정책과 함께 명시.
- 필수 시험: `.1−ε/.1/.1+ε` raw gaps; `.20` bracket을 여러 `.05` frame으로 분할; zero-new-sample/carried future; 보간 anchor 후 첫 loss; invalidation 후 clean reanchor. 기존 engine 결과는 baseline과 같고 prediction만 거부되어야 함.

## C2 — clean seed의 실제 snapshot과 필요한 core interface가 빠짐

- 대상: plan-v1:27–30,34–35. **차단 이유:** 계획의 소유 파일만으로는 선언한 실제 estimator gravity에 접근할 수 없고, timestamp가 다른 state를 조합하지 않는 조건도 충분히 고정되지 않음.
- 실제 코드:
  - [Estimator::g_](../../include/backend/estimator.h#L86)는 private이며 public gravity getter가 없음. [setParameter](../../src/backend/estimator.cpp#L21)는 config gravity를 실제 추정 gravity로 복사하는 함수가 아님.
  - [getLatestVINSResult](../../src/backend/estimator.cpp#L503)는 post-slide `WINDOW_SIZE−1` state/timestamp를 반환. [publishPose](../../src/vio_engine.cpp#L241)는 별도 slot에서 전달된 body P/R을 camera pose로 공개.
  - [configure 실패](../../src/vio_engine.cpp#L67)는 기존 engine을 살려두고 pose만 invalid로 만들 수 있음. prediction clear를 성공 configure/reset에만 붙이면 부족함.
- 계획 수정:
  - `include/backend/estimator.h`의 읽기 전용 gravity copy getter 또는 동일시점 snapshot getter를 명시적으로 core 소유 범위에 추가. Estimator 수학/gate 변경 금지는 유지.
  - seed는 `snapshot.timestamp == frameTimestamp == IMU endpoint`, 동일 epoch, 해당 frame의 backend usable update, 성공한 clean visual publication을 모두 만족할 때만 수락.
  - P/R/V/Ba/Bg/g/extrinsic와 endpoint acc/gyro를 같은 수락 경계에서 복사. gravity 부호는 engine의 `a_world=R(f−Ba)−g_engine` convention을 사용하고 analytic fixture의 physical gravity와 명시적으로 변환.
  - PnP 설정 변경, 실패 configure, hard rejection은 즉시 clear; 다음 frame까지 오래된 getter 결과를 남기지 않음.
  - activation을 고정: 항상 활성인지, explicit enable인지, configure/reset에서 어떤 값인지 선언. enable API를 택하면 binding/readback/ownership까지 포함. 항상 활성로 결정할 수도 있으며 불필요한 toggle을 강제하지 않음.
  - `getInertialPrediction()`의 unavailable ABI를 정확히 고정: 예를 들어 `std::string`의 JSON literal `"null"`인지 Emscripten null 값인지 하나로 결정하고 native/Worker/old-WASM 시험을 일치시킴.
- 필수 시험: timestamp 불일치·wrong epoch·finite지만 invalid R·zero/invalid gravity·quality-warning·failed configure·PnP toggle·다른 engine instance. post-slide seed 값이 공개된 clean camera pose와 일치하고 이전 raw endpoint가 정확히 이어져야 함.

## C3 — 화면 deadline과 plane 상태·운반 계약의 단일 소유권 필요

- 대상: plan-v1:41–46,51–57,66,74. **차단 이유:** 병렬 구현 lane이 서로 다른 TTL/plane producer를 만들 수 있으며 동일 packet의 state/age 의미가 아직 하나로 정의되지 않음.
- prediction deadline:
  - `target−anchor≤.5`, `now−target≤.1`만 검사하면 anchor로부터 최대 .6초까지 표시 가능.
  - `now−anchor≤.5`까지 요구하면 전체 표시 수명은 .5초. plan-v1:54의 “anchor horizon0.5s를 매 RAF 재검사”는 두 해석을 구별하지 않음.
  - 둘 중 하나를 식으로 고정. 예: source horizon은 `.5`, display deadline은 `min(target+.1, anchor+.5)`로 선언하면 전체 수명도 .5초. 선택 자체보다 모든 계층/시험의 일치가 필요.
- plane 계약:
  - fitter/plane state의 인스턴스 소유자를 Worker·Wrapper·App 중 하나로 확정. pure module 공유는 상태 소유권을 대신하지 않음.
  - `groundSupport`와 최종 `groundPlane`의 schema, typed-array/count, source timestamp/sequence, engine/client epoch, normal/d/centroid/corners/support/RMS, fresh/retained 상태, validator 위치를 고정. arbitrary `...data` plane metadata도 prediction처럼 정규화·거부.
  - 최대5Hz 사이 clean tracking frame의 미실행, 실제 fit 실패, 새 support가0인 frame, Lost, hard error, worker timeout, epoch/reset, no-new-result를 구분한 전이표 작성.
  - retained TTL은 마지막 실제 support의 source timestamp 기준임을 명시. Lost 진입/수신/RAF 시각으로1초를 다시 시작하지 않음. clean tracking 중 fit이 계속 실패해도 무기한 fresh plane으로 남지 않게 정의.
  - [App::_consumeResult](../../web/js/app.js#L1383)의 invalid-result `renderer.clear()`는 매 Lost 결과에 실행됨. metric clear 후 별도 layer를 표시하는 순서와 hard reset의 전체 clear를 분리하여 retained plane 재활성화/소실을 방지.
- 필수 시험: target=.49초와 arrival=.57초처럼 source/display gate가 갈리는 경우, future source, real delayed receive, 같은 결과 반복 RAF, 새 결과 없이 expiry; 새 plane→fit skip→fit fail→Lost→reset. 모든 timestamp·epoch가 틀린 support/plane payload는 거부.

## C4 — 실제 floor 경로와 prediction 기능의 E2E 수락을 수치로 고정해야 함

- 대상: plan-v1:63–69. **차단 이유:** 현재 floor 수락은 unit geometry와 point-export 필터에 머물러 실제 camera input→신규 API→WASM→plane→표시 성공을 증명하지 못함. prediction E2E도 coverage/오차를 “기록”한다고만 되어 있어 빈약한 출력의 채택 기준이 불명확함.
- 기존 근거: [test-design](2026-10-14-low-feature-test-design.md#L100)의 floor-visible fixture는400 input/357 visual-fresh/280 legacy-map candidate/264 GT-floor gate 통과. 이는 **새 current-ID 교집합 API의 결과가 아님**. 세 blank arm의 기존 trajectory GT 실패도 유지해야 함.
- 계획 수정:
  - 실제 floor-visible fixture와 floor가 우세하지 않은 clean101 negative control을 신규 current-support getter와 actual Worker/WASM에 연결하는 필수 E2E gate 추가.
  - 처리 frame, clean eligible frame, point≥20 frame, fit accepted frame, GT normal/offset 통과 frame의 분모를 모두 유지. `candidate=0`인 구현은 floor 기능 완료로 판정하지 않음.
  - 독립 GT의 normal≤3°/offset≤.05m 등 이미 제안된 기준을 채택/수정하고, 최소 검출 frame 수·시간 범위 및 허용 실패율을 **결과를 보기 전에** 결정. legacy264를 신규 API의 기대값으로 그대로 복사하지 않음.
  - prediction short/long arm은 clean anchor 이후 whitelist+완전 coverage+한계 이내의 eligible frame별 expected availability와 실측 availability를 대조. 장기 blank의 source horizon 이후0을 전체 해당 frame에서 검증.
  - body↔camera 변환과 timestamp association을 포함한 prediction segment GT translation/rotation·preblank 고정 alignment·reacquire jump를 필수 출력으로 고정. 전체 경로를 다시 정렬해 구간 drift를 숨기지 않음.
  - 기본 활성화의 정확도 수락이 목적이면 허용 오차 기준을 사전 정의. 이번 범위를 unverified diagnostic display로 제한한다면 그 사실을 결과/화면/완료 문구에 유지하고 metric 추적 개선으로 보고하지 않음.
- 추가 음성 시험: 새 floor 없는 화면에 과거 map만 남음, table/raised-floor 동일 관측, 높은 motion+blank, 잘못된 seed bias/gravity/velocity의 오차 민감도. analytic integration PASS를 실제 seed 정확도로 바꾸지 않음.

## C5 — “backend 불변”의 회귀 증거를 공개 pose만으로 축소하지 않기

- 대상: plan-v1:18,22,67. **차단 이유:** 선택안의 핵심은 map/prior/optimizer와 비간섭인데 최종 회귀 기준은 metric pose/status/reason/epoch/source association만 나열함. 단기간 pose가 같아도 map admission/prior/window가 변경되면 선택 원칙을 어길 수 있음.
- 계획 수정:
  - fixed-work baseline 대비 pose/status/reason/epoch에 더해 solved map, feature admission, solver factor/iteration counts, window P/R/V/Ba/Bg 및 가능한 prior diagnostics를 비교하는 native integration 시험 포함.
  - failed optimizer/window-slide 및 empty-feature early return 양쪽에서 predictor의 snapshot이 이후 backend mutation과 독립임을 검증.
  - read-only export/fitting 호출 횟수를 늘려도 backend 결과가 같아야 함. 비교는 기존 고정 source/profile/seed/IMU packet/input 순서에서 수행.
  - exact 비교가 불가능한 항목은 기존 어떤 시험/수치 tolerance를 쓸지 명시. 신규 결과를 본 후 tolerance를 넓혀 통과시키지 않음.
  - 10초 fixed-work solver profile의 비간섭 증거와 실제 mobile .06초 profile의 실행·latency/drop 관측을 구분. 후자는 phone 정확도/성능 acceptance가 아님.
- 이 요구는 새 estimator architecture를 만드는 요구가 아님. 기존 test-access/diagnostics와 동일 입력 비교로 독립 consumer라는 주장에 필요한 최소 증거를 확보하는 것.

## 재검토 완료 조건

- C1 raw-bracket lineage·C2 seed/API·C3 deadline/FSM/ownership·C4 실제 E2E 수락·C5 비간섭 관측항목을 수정 계획에 반영.
- core/math, pure module, transport/harness, App/renderer/recorder의 파일 소유권과 경계 schema가 일치.
- 잘못된 현재 수락 주장은 없음: 기존 실패와 새 검증의 분리, semantic floor 미지원, 무기한 IMU navigation 미지원, 물리폰 GT 미검증을 유지.
- 이번 리뷰는 계획/소스 검토이며 신규 native/browser/WASM 시험은 실행하지 않음. 구현 단계의 기존 전체 회귀·새 sanitizer·실제 Worker 검증을 대체하지 않음.
