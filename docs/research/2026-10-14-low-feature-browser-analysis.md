# 저특징·빠른 이동: 브라우저 예측·바닥 후보 계약 분석

- 상태: **구현 전 제안**. 기존 uncommitted 제품/연구/배포 artifact 보존; 제품 변경·서버 시작·실기기 접근 없음.
- 범위: 현재 Worker→Wrapper→App→Renderer 경계, Limited 호환 코드, U의500ms display expiry, 별도 IMU prediction 및 floor-candidate 표시/평가 설계.
- 핵심: 무특징 구간의 IMU 전파는 **시각 추적 성공도, metric 위치 관측도 아니다**. 기존 `pose/poseFresh/poseValid/statusCode`와 별도 typed payload로 운반·표시·평가해야 함.
- 저텍스처 바닥은 영상만으로 plane offset을 항상 얻을 수 없음. gravity는 수평면 normal의 방향 제약을 주지만 높이/거리/scale을 관측하지 않음. 현재 grid를 검출된 floor라고 표시하지 않음.

## 현재 코드에서 확인한 경계

| 위치 | 현재 동작 | 설계 함의 |
| --- | --- | --- |
| `include/vio_engine.h:22–29`, `src/vio_engine.cpp:423–427` | 엔진 status0..4; initialized+valid이면 Tracking, 아니면 Lost | status5 UI 수용 코드가 존재해도 현 엔진이 Limited를 생성하는 것은 아님 |
| `src/vio_engine.cpp:487–514` | pipeline-v1 출력 | v2 quality/covariance가 현재 authoritative producer에서 생성된다고 가정 금지 |
| `wasm/vio_bindings.cpp:73–95` | pose/time/epoch/diagnostic getter; prediction state API 없음 | 현재 pose16만으로 velocity/bias/gravity/covariance를 정확하게 복원할 수 없음 |
| `vio-worker.js:329–340`, `vio-wrapper.js:27–38` | `projective_limited`는 current projective factor≥2·rotation rank3·covariance·2초/.25m/5° 등 엄격 수용 | 시각 support0의 IMU-only 상태와 다름; Limited 조건을 완화해 대체하지 않음 |
| `vio-worker.js:354–453` | IMU loss면 reset+empty, pose jump면 reset+empty; map은 valid visual result에서만 확보 | 예측 overlay가 이 reset/rejection을 우회하면 안 됨 |
| `vio-wrapper.js:332–376` | active request/client epoch/sequence/frame timestamp/engine epoch 검사; invalid pose/map null | 새로운 prediction도 동일 identity와 독립 validation 필요. `...data`에 실려왔다고 신뢰하지 않음 |
| `app.js:1335–1400` | 결과 dedup와 상태 갱신 분리; valid visual age0..500ms; stale는 frustum hide, Lost/reset/epoch는 clear | prediction용 age/visibility는 별도. visual age를 새 timestamp로 바꾸지 않음 |
| `renderer.js:62–69,196–277,296–316` | Z-up root→Three Y-up; raw frustum와 follow viewer; historical map/trajectory; hide와 clear 분리 | prediction geometry와 floor candidate를 기존 metric history에 append하지 않음 |

- 기존 strict guards는 삭제 대상이 아님: `processFrame`의 forward-only frame time, IMU bracket, epoch/reset, map validity, Wrapper stale reply guard, U freshness expiry를 유지.
- 현재 `invalidatePose`는 last pose를 invalid/zero로 초기화(`src/vio_engine.cpp:139–143`). Lost result의 빈 pose나 identity를 anchor로 사용하지 않음.
- `processIMUData`는 estimator에 실제 전파·endpoint interpolation을 수행(`:187–229`). 실패한 solver 뒤의 mutable sliding-window state를 “신뢰한 마지막 metric state”로 간주해서 읽어내는 우회는 금지.

## 제안 데이터 분리

```text
result
  identity: requestId / sequence / clientEpoch / engineEpoch / inputTimestamp
  기존 pose / poseFresh / poseValid / statusCode / pipeline / mapPoints  [의미 불변]
  prediction: null | inertial-prediction-v1                           [새 독립 결과]
  floorCandidate: null | floor-candidate-v1                          [관측 기반 후보]
```

- `prediction` 제안 필드:
  - `schema`, `available`, `reason`, `mode: orientation_only|inertial_6dof`.
  - `clientEpoch`, `engineEpoch`, `requestId`, `sequence`, `inputTimestamp`는 enclosing result와 exact 일치.
  - `anchorTimestamp`, `anchorSourceSequence`, `anchorSource: last_accepted_metric_visual`, `targetTimestamp`, `horizonS`.
  - `imuIntervalStart/End`, `imuLastTimestamp`, `imuSampleCount`, `imuContinuous`, `interpolatedEndpoint`.
  - `pose` 또는 명시 `rotation`+`translationAvailable`; row-major `T_W_C` 등 frame 계약. orientation-only에서 유지한 anchor 위치는 `positionHeld:true`이며 새 metric translation이 아님.
  - `rotationVisualSupported:false`, `metricTranslationSupported:false`, `feedsEstimator:false`는 고정 contract.
  - `uncertaintyAvailable`, covariance와 frame/order/model/source. 없으면 `null`/unavailable;0으로 대체하지 않음.
  - `producedAtMs`는 Worker의 `performance.timeOrigin+performance.now()` 계열 transport clock; source timestamp seconds와 혼용하지 않음.
- 별도의 `prediction.available=true`가 기존 `poseFresh=true`를 만들지 않음. 시각 Lost row는 Lost 그대로 남고 예측 coverage만 별도 증가.
- 내부 anchor는 마지막 **검증된 metric visual** state의 attitude/position/velocity/bias/gravity 및 유효 extrinsic을 producer가 확정할 때 복사. renderer pose 차분으로 velocity를 만들어 검증된 상태처럼 쓰지 않음.
- body 전파 뒤 camera pose로 바꾸는 곳은 한 군데: `p_WC=p_WB+R_WB·t_BC`, `R_WC=R_WB·R_BC`. 기존 `publishPose`가 같은 형태(`src/vio_engine.cpp:241–247`). body IMU와 camera lever arm·row-major를 명시.
- JS가 별도 gyro/acc를 다시 적분하는 중복 estimator보다 **authoritative C++ scratch propagation의 read-only export**를 우선 검토. 계산은 별도 state/covariance에서 수행하고 estimator window/prior/map에 write-back하지 않음. 이 API 설계는 알고리즘 lane에서 승인 필요.

## horizon·producer latency·identity 수락 제안

- 첫 버전은 accepted camera frame에 대응하는 결과로만 생성: `targetTimestamp == inputTimestamp == engineFrameTimestamp`. 그 구간의 실제 IMU 양쪽 endpoint가 있어야 함. **마지막 IMU 이후 extrapolation 금지**.
- 다음 수치는 구현 전 정할 **정책 제안값**이며 기존 성능 달성값이 아님: `H_source=0.25s`(마지막 metric anchor→target), `L_display=0.10s`(target source time→현재 display). 기존 visual500ms 정책과 별도.
- 두 gate를 동시에 만족: `0<=target−anchor<=H_source`, `0<=now−target<=L_display`. 따라서 anchor age는 최대350ms. producer→receive/display 지연도 기록하고, 부하로 이미 늦은 prediction은 도착해도 표시하지 않음. deadline은 payload arrival시각으로 새로 시작하지 않음.
- covariance/관측성 gate가 요구되는6DoF 모드에서 covariance source가 없으면 출력 unavailable. fixed horizon만으로 translation 정확도를 보증하지 않음. orientation-only fallback도 검증된 anchor·gyro bias·시간축·연속 samples 필요.
- 기존 Limited의2초/.25m/5° 임계값은 시각 회전 지지가 있는 contract이므로 새 inertial-only 허용값으로 자동 재사용하지 않음.
- main RAF는 prediction timestamp를 `now`로 재라벨하지 않음. 다음 IMU를 기다리는 동안 같은 prediction을 animate/lerp해서 새 추정처럼 만들지 않음.
- 현 C++ `engine.processFrame`은 Worker에서 동기 실행. 별도 payload를 추가해도 engine이 바쁜 동안 높은 주파수 producer가 생기지 않음. **60Hz IMU prediction/무지연 rendering은 이1차 frame-triggered 설계의 완료 주장이 될 수 없음.** 필요하면 후속으로 불변 anchor snapshot+mirrored IMU를 사용하는 독립 predictor를 설계하되 thread/ownership/CPU 비용을 다시 검토.

## fail-closed 조건과 기존 guard 접점

| 사건 | prediction 조치 | 기존 경로 |
| --- | --- | --- |
| 이전 metric anchor 없음·초기화 전 | unavailable, 합성 origin/velocity 생성 금지 | VIOStatus INITIALIZING 유지 |
| transport IMU overflow/stale/capacity loss | anchor/prediction cache invalidate; 해당 frame unavailable | `vio-worker.js:361–373,397–403` reset+loss reason |
| missing past/future bracket·timeout | unavailable; 마지막 IMU hold 외삽 금지 | `vio-worker.js:582–603`, `src/vio_engine.cpp:203–227` |
| engine/client epoch 변경·reset/configure/dispose | prediction/floor cache 모두 clear; old reply reject | Worker clearIMURing, Wrapper `_invalidate` |
| worker frame timeout/exception | unavailable 즉시 hide, error label 유지 | `_workerFailure`, Worker exception reset |
| candidate visual pose jump 또는 engine warning | **prediction으로 가리지 않고 reset 결과 유지**, anchor 무효화 | `vio-worker.js:414–428`의5m/relative threshold guard |
| prediction 자체의 nonfinite/rotation invalid/uncertainty 초과 | reject/hide + reason; estimate 원본 유지 | 별도 prediction validator 필요 |
| 새 metric tracking 재획득 | prediction hide, actual jump/offset을 기록; metric pose 직접 반영 | scene-jump smoothing으로 재연결 오차 숨기지 않음 |
| floor stale/epoch 불일치 | hide 또는 명시 historical 표시; correction 금지 | candidate TTL/epoch 별도 |

- 허용 loss reason을 좁혀 opt-in: 정상 source/IMU가 있고 **현재 visual support 부족**인 경우만 prediction 후보. `invalid_frame`, `frame_out_of_order`, `imu_transport_loss`, `frame_gap`, `invalid_pose`, `processing_exception`, `pose_translation_discontinuity`는 unavailable.
- `empty_features`의 첫 frame과 초기화 미완료를 featureless-after-tracking과 혼합하지 않음. 사용 가능한 anchor가 있어야 함.
- `lastPublishedPose`는 mobileMotionGuard의 비교 state. prediction을 이 변수에 저장하면 다음 실제 pose jump 검사 기준이 오염되므로 **공유 금지**.
- map은 visual accepted 결과에서만 갱신. prediction point/position을 solved feature, triangulation seed, local-map keyframe, PnP prior, optimizer correction 또는 ground constraint로 넣지 않음.

## floor candidate와 화면 world 축

- `floorCandidate`는 검증된 visual map snapshot에서 계산한 **기하 후보**. 임의 무텍스처 영상만으로 “바닥 감지 성공”을 반환하지 않음.
- 필요 provenance: `schema`, `clientEpoch/engineEpoch`, `mapSourceTimestamp`, `mapSourceSequence`, `normalW[3]`, `offsetD`, `centroidW[3]`, bounded extents/convex support, `supportCount`, fit residual, source map count/selection, `candidateAgeS`, confidence **정의**.
- 평면 식은 `n_W·p_W+d=0`, `||n_W||=1`, `n_W.z>=0`로 normal sign canonicalize. horizontal tolerance·support/extent/residual 기준은 알고리즘 lane에서 independent geometry fixture로 승인.
- gravity-aligned normal은 “수평면” 조건일 뿐 floor 의미를 완전히 결정하지 않음. tabletop·ceiling·multi-plane·sparse/outlier fixture가 필요. camera 아래라는 조건도 camera offset/scale/frame이 검증되어야 사용.
- 현재 `getMapPoints`는 최대2000개의 xyz이며 persistent feature IDs·per-point covariance·frame별 support age가 없음(`vio-worker.js:438–449`). 현 payload만으로 multi-frame 같은 point 중복지지나 point별 confidence를 정확히 셀 수 없으므로 aggregate fit 근거 수준을 명시.
- Renderer의 `worldRoot`는 Z-up을−π/2 around X로 한 번 변환. Three `PlaneGeometry` 기본 localXY(+Z normal)를 worldRoot 아래 두고 quaternion `+Z→n_W` 및 plane centroid를 적용하면 수평 normal을 Y-up으로 중복 회전할 필요가 없음. 기존 GridHelper는 원래XZ여서 별도+π/2 회전하는 경로와 구분.
- candidate mesh는 `floorGroup`의 amber translucent/wireframe, 기존 원점 grid와 다른 label 사용. frame timestamp/age/support를 UI에서 확인. 카메라에 맞춰 항상 plane을 이동시키거나 height를 smoothing해 붙어 보이게 만들지 않음.
- fresh candidate TTL의 초기 제안은 기존 visual display500ms; 그 이후는 hide가 기본. 필요시 별도 historical plane UI를 회색으로 표시하되 현재 floor recognition으로 집계하지 않음. epoch/reset에는 무조건 제거.
- predicted camera가 움직였다고 floor mesh를 재-fit/이동시키지 않음. floor height를 IMU 위치 correction에 쓰는 기능은 이번 **visual candidate 표시** 범위 밖.

## App/Renderer 표시와 기존 U 보존

- 기존 visual status는 `Lost`/`Tracking`/`Stale` 유지. 독립 보조 label 예: `IMU prediction · 120ms since visual anchor · position uncertain`; `Floor candidate · 34 support · age80ms`. `Tracking`으로 덮어쓰지 않음.
- 새로운 renderer 경로는 `showPrediction`/`hidePrediction`과 `updateFloorCandidate`/`clearFloorCandidate`처럼 역할을 명확히 분리. prediction frustum은 별도 색/객체, metric trajectory append0. `_latestPoseMatrix` 및 follow camera raw pose로 자동 대입하지 않음.
- 기존 `clear()`는 reset/epoch에서 전체 scene diagnostic를 지울 수 있게 확장; loss일 때 기존 visual map/trajectory clear 정책은 유지하되 **같은 frame identity를 검증한 prediction만 그 뒤 별도 layer로 표시**. 한 clear 호출이 invalid anchor를 재활성화하지 않도록 순서 테스트.
- expiry만 발생하면 해당 current prediction/floor mesh hide. historical metric map/trajectory는 U처럼 보존. engine Lost시의 기존 clear와 expiry hide를 혼합하지 않음.
- `App._consumeResult` 안에서 visual/newResult/quality와 prediction identity/age를 독립 계산. 새 prediction이 없으면 이전 것을 자동 연장하지 않음. UI text10Hz 제한과 즉시 error/stale 전이 계약 유지.
- visual reacquire 시 predicted→measured residual을 별도 diagnostic로 기록하되 선형 blend를 기본 도입하지 않음. blend가 필요하면 observation-preserving presentation으로 따로 승인·지연 평가.

## 수정 후보 파일·필수 회귀

| 경계 | 계획 승인 후 후보 파일 | 독립 기대값/회귀 |
| --- | --- | --- |
| authoritative anchor/prediction producer | `include/vio_engine.h`, `src/vio_engine.cpp`, `wasm/vio_bindings.cpp`; 알고리즘 lane과 소유권 합의 | known constant-velocity/constant-rotation/acceleration, body→camera lever arm, gravity 제거, epoch/reset/bias 상태, 원 estimator 결과·map·prior 불변 |
| Worker export/result | `web/js/vio-worker.js` | strict current frame identity, before/after epoch, loss/jump/exception/missing bracket에서 prediction null; prediction payload는 old pose output으로 우회하지 않음 |
| Wrapper ingress | `web/js/vio-wrapper.js`, `tests/browser/transport.test.mjs` | malformed size/NaN/non-orthogonal R/foreign epoch/future target/duplicate/reordered payload 거부; prediction으로 poseFresh/valid 변경0 |
| UI | `web/js/app.js`, `tests/browser/mobile-source-contracts.test.mjs` | visual Lost+prediction만 표시, ageH/H+ε·producer latencyL/L+ε, no-result/error/reset, map/trajectory count 불변, visual500ms 회귀 |
| Renderer | `web/js/renderer.js`, 기존 source-contract fixture | predicted frustum 별도 visible/color, metric append0, hide/clear, floor n=+Z/tilted world fixture, root 회전1회 |
| floor 후보 전달 | plane producer의 실제 위치에 따라 Worker/Wrapper; geometry lane 합의 | horizontal plane/table/ceiling/outlier/zero-support, map snapshot identity/age·epoch; prediction map 생성0 |
| recorder | `scripts/dev/android-input-record.mjs:76–86`, `web/js/audit-mobile.js:182–208` | source payload bytes/identity 불변, prediction/floor metadata 별도·bounded size, failure event 보존 |
| replay/평가 | `scripts/dev/synthetic-browser.mjs`, `tests/browser/synthetic-input-contracts.test.mjs`, 별도 evaluator | 기존 sourceCoverage/poseStateSummary/validateResultIdentity 의미 유지; prediction-only row metric fresh 증가0, source frame 분모 유지 |

- 현 Android input recorder는 result의 필드를 whitelist로 복사하고 audit page도 pose 결과를 명시 추출하므로, 새 payload가 자동 보존되지 않음. 두 recorder를 최소 확장하되 predicted pose를 기존 raw pose/jump 통계에 섞지 않음.
- harness는 현재 `poseFresh&&poseValid`를 metric/기존 Limited population으로 집계(`synthetic-browser.mjs:92–119`). prediction 값은 **별도 predictionCoverage/horizon/error** 통계로 기록하고 원 metric denominator에서 Lost row를 제거하지 않음.
- `validateResultIdentity`의 fresh pose timestamp=input timestamp 조건은 유지. prediction timestamp도 독립 exact identity validation; 서로 다른 시각의 원래 GT를 가져오거나 nearest-frame로 재라벨하지 않음.
- strict producer latency test는 실제 wall clock 및 source mapping을 사용. 가상 deterministic clock의 `performance.now` override로 late producer가 fresh처럼 보이지 않도록 source→produce→receive→display4시각을 보존. fake delay 후 deadline past payload는 반드시 hide.
- 실제 App/Worker/WASM 검증에서 featureless frame을 추가하되 IMU원본/epoch/source timestamps는 유지. highspeed-blur와 textureless를 분리하고 held-out 강한 motion·잘못된 bias·IMU gap에서 fail-closed를 확인.
- 이번 문서는 설계 분석만 완료. horizon/uncertainty/floor criteria·producer API가 승인되기 전 제품 구현/배포 없음. 예측 표시 성공이 featureless metric tracking 성공을 뜻하지 않음.
