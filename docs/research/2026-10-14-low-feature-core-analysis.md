# 저특징·빠른 움직임의 bounded IMU 출력 — core 설계 검토

- 날짜: 2026-10-14, Asia/Seoul. 요청: featureless wall/빠른 이동에서 IMU-only 동작과 floor recognition 추가 전 분석·계획.
- 모드: **read-only core 설계**. 이 문서만 추가; 제품 C++·Worker·Wrapper·active WASM·서비스 변경 없음. 현재 HEAD `5d48e4bd7466842dad98928fb1f12988488402df`; 기존 tracked/untracked 작업 모두 보존.
- 결론: 현재 엔진은 영상 실패 전에 IMU를 이미 적분하지만, 유효한 visual pose로 공개하지 않는다. 가장 작은 안전한 확장은 **마지막 정상 visual backend state에서 시작하는 별도 C++ 관성 예측 상태**를 짧게 전파해 새 `inertialPrediction` 채널로 공개하는 것이다. 기존 `poseValid/poseFresh`, Lost, map, prior를 성공으로 재분류하지 않는다.
- 이 제안은 frame timestamp까지 실제 센서로 전파하는 것. 미래 display time extrapolation이나 무기한 metric navigation이 아니다. floor recognition은 실제 관측한 평면 근거가 있어야 하며 IMU만으로 바닥 높이/위치를 알아내지는 못한다.

## 1. 현재 확인한 동작

| 위치 | 확인한 동작 | 설계 영향 |
|---|---|---|
| [VIOEngine::processFrame:268](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:268) | 매 호출 output16개 NaN, `invalidatePose`로 valid/fresh false부터 시작 | 새 관성 출력은 기존 pose buffer 재사용 금지 |
| [processIMUData:187](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:187) | timestamp 순서·finite·queue cap·gap 검사, image 전후 bracket, affine endpoint 보간 | 기존 interval 선택을 재사용하고 JS에서 별도 시간 정렬하지 않음 |
| [feedIMU:175](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:175) | Estimator/PnP 적분 후 `current_time_`, prev acc/gyro 갱신 | predictor는 같은 실제 sample/보간 endpoint를 받는 위치 |
| [Estimator::propagateIMUState:76](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:76) | R/P/V를 bias·gravity로 적분, nonfinite면 이전 R/P/V 복원 | finite 상태만 읽어도 해당 구간 적분 성공을 보장하지 않음; silent rollback 상태를 외부 성공으로 내보내면 안 됨 |
| [empty_features:347–358](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:347) | IMU·frontend 후 image_data.empty이면 backend processImage 이전 return | blank frame에도 IMU cursor/RPV는 이동; window timestamp/feature history는 image마다 진행하지 않을 수 있음 |
| [Estimator::processImage:275–298](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:275) | solve 실패여도 정상 finite면 window slide, feature lifecycle 진행 | 실패 후 current window를 그대로 읽는 구현은 seed·slot·prior 오염 경계가 불명확 |
| [Optimizer::optimize:143–216](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/optimizer.cpp:143) | current<2면 prior release, solver 실패면 parameter rollback/prior release | prediction용으로 optimizer 실패 정책을 변경할 필요 없음 |
| [publishPose:241](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:241) | body R/P를 extrinsic으로 camera pose 변환, visual pose timestamp·fresh 갱신 | prediction에는 같은 extrinsic 수식만 적용, 이 함수를 호출하지 않음 |
| [getMapPoints:411](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:411) | `has_valid_pose_`가 false면0 | Lost prediction에서 기존 map API가0인 계약 유지 |
| [resetState:145](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:145) | epoch 증가, Estimator/track/map/PnP/pending/endpoint reset | prediction seed·propagated state·published buffer도 함께 폐기 |

- 중요한 timestamp 차이: `getLatestVINSResult()`는 **solve 후 slide된 WINDOW_SIZE−1** slot의 timestamp/P/R/V/Ba/Bg를 돌려준다. blank early-return 직후의 현재 IMU endpoint를 읽는 API가 아니다. previsual prediction timestamp는 `current_time_==image_timestamp`로 인증해야 한다. [Estimator:503–517](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:503)
- `getFeaturePointCount()`는 track count>1만 센다. 앞선 분석에서 “이번 GFTT로 추가한1frame point까지 포함”이라고 일반화한 설명은 현재 getter에 맞지 않는다. 다만 count>1도 metric depth/current factor 인증은 아니다. [VIOEngine:407–409](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:407)

## 2. 기존 LIMITED schema를 사용하면 안 되는 이유

- C++ enum은0 NOT_CONFIGURED,1 INITIALIZING,2 TRACKING,3 LOST,4 COOLDOWN뿐. numeric5는 browser adapter가 향후 `tracking_limited`용으로 예약한 값이다. [header:22–28](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/vio_engine.h:22)
- Worker/Wrapper의5 수락 조건은 `projective_limited`, **visual rotation support=true**, projective factor≥2, visual rank3, joint covariance, uncertainty source,≤2초 및 sigma/innovation gate를 요구한다. pure IMU는 이를 충족하지 않는다. [Worker:304–340](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/web/js/vio-worker.js:304), [Wrapper:27](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/web/js/vio-wrapper.js:27)
- 과거 A3/H4는 이 graph/covariance 계약을 포함한 실험이며 채택되지 않았다. pure IMU 출력에 가짜 visual support/rank/covariance를 만들어5로 통과시키면 안 된다.
- 권고: 기존 `statusCode=LOST`, `pose=null`, `poseValid=false`, `poseFresh=false`, `mapPointCount=0` 유지. 별도 `inertialPrediction` 객체만 valid일 수 있다. UI는 이를 “IMU 추정”으로 표시하고 metric trajectory와 별도 집계한다.

## 3. 대안 비교

| 대안 | 크기/재사용 | 위험 | 판정 |
|---|---|---|---|
| 실패 후 `sliding_window_[10]`를 그냥 publish | 가장 적은 diff | solver/window/gauge mutation, seed 경계 불명확, silent propagation rollback, 기존 pose/map 성공 의미 파괴 | 기각 |
| IMU 직후·visual 전에 backend RPV snapshot을 떠서 별도 출력 | 현재 적분 재사용; 작은 diff | 이전 visual 실패/quality-warning 성공으로 변경된 backend가 다음 snapshot의 시작값; 긴 실패 동안 last-trusted lineage 보증이 어려움 | 진단 snapshot에는 적합, 단독 bounded predictor로는 부족 |
| **C++ 독립 shadow R/P/V + frozen bias/gravity/extrinsic** | endpoint 처리 재사용; fixed-size state와 작은 전파 함수 필요 | propagation 수식 중복·seed validation 필요. 테스트로 scope 제한 가능 | **권고** |
| Worker JS에서 pose/gyro/acc 재적분 | WASM API 수정 일부 회피 | body velocity/Ba/Bg/g가 현재 노출되지 않음; extrinsic/time축·bracket 중복, raw/정규화 sample 경계 위험 | 1차안 기각 |
| Worker에서 orientation만 회전, 위치 고정 | 적은 상태·translation drift 억제 | 사용자의 이동 추적 요구를 만족하지 못함; 위치 고정을 실제 stationary로 오인 가능 | 명시적 UI degraded mode 후보, IMU6DoF의 대체라고 보고하지 않음 |

- shadow state는 backend를 보완하는 display/diagnostic output이지 backend의 새로운 measurement가 아니다. 사용 중인 map point·PnP cache·feature depth·marginalization·initializer 어느 곳에도 predictor 값을 다시 넣지 않는다.
- predictor는 실제 sample마다 독립 propagation하고 image endpoint에서 **visual optimizer 호출 전** 불변 snapshot을 만든다. visual 성공이면 corrected state로 재anchor하고 해당 frame에는 visual pose만 사용한다. visual 실패면 snapshot을 선택적으로 공개한다.

## 4. 권고 state와 최소 interface

### 4.1 내부 state

```text
InertialPredictionState (VIOEngine instance 소유)
  enabled, seeded, available
  seedEpoch, seedTimestamp, endpointTimestamp
  body P, R, V
  frozen Ba, Bg, gravityWorld
  frozen R_B_C, t_B_C
  previous endpoint accelerometer, gyro
  durationSinceVisual, lastRejectReason
```

- `common::VINSResult`에 P/R/V/Ba/Bg/timestamp가 이미 있음. seed 복사에 재사용. gravity는 Estimator private `g_`이므로 **읽기 전용 copy getter** 한 개를 추가하는 방안이 가장 작다. global `g_config.estimator.g`를 정렬된 현재 gravity 대신 임의 사용하지 않는다. [common_types:20–28](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/common/common_types.h:20), [Estimator header:86](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/backend/estimator.h:86)
- seed extrinsic은 해당 accepted backend instance의 `r_ic_`, `t_ic_`. current calibrated body→camera 방향 보존.
- seed 수락: configured·NON_LINEAR·해당 image의 backend usable/current support 통과·qualityReason 없음·published visual pose 정상·timestamp==image endpoint·R valid/모든 vector finite. 단순 `isInitialized=true`만으로 seed 생성 금지.
- PnP-only frame의 `getLatestVINSResult()`는 오래된 backend snapshot일 수 있음. 1차 구현은 **PnP 활성 모드에서 fallback unavailable**로 명시하거나 backend-only seed timestamp를 그대로 유지해야 한다. PnP pose를 최신 visual seed라고 속여 backend V/Ba/Bg를 다른 시간에 붙이지 않는다. default PnP OFF 범위부터 검증 권고.

### 4.2 외부 interface 초안

```text
setInertialPredictionEnabled(bool)           // explicit mode, configure default false
getInertialPredictionAvailable() -> bool
getInertialPredictionPose(double out[16]) -> bool
getInertialPredictionTimestamp() -> double
getInertialPredictionSeedTimestamp() -> double
getInertialPredictionReason() -> string
```

- 구조화 getter1개와 기존 diagnostics 직렬화를 사용하는 등 최종 binding 수는 구현 시 줄일 수 있다. 필요한 의미는 아래와 같다.
- browser 결과 계약: `schema='mobile-slam-inertial-prediction-v1'`, `kind='imu_only'`, `pose`, `timestamp`, `seedTimestamp`, `durationS`, `horizonS`, `engineEpoch`, `clientEpoch/requestId` 연결, `visualSupport=false`, `uncertaintyAvailable=false`, `reason`.
- `timestamp`는 input frame timestamp와 같아야 하고 future output 금지. `durationS=timestamp-seedTimestamp`가 누적 horizon을 넘지 않아야 함. 매 prediction 성공마다 seed time을 새로 잡지 않는다.
- buffer는 visual pose와 별도. unavailable getter는 false 및 NaN/null 계약으로 stale 재사용을 차단. C++ engine epoch와 Worker client epoch 모두 맞지 않으면 browser가 제거한다.
- configure는 enabled=false·seed clear, reset은 mode 선택을 보존하더라도 seed/available은 clear. enabled 실제 변경 시 seed부터 clear; same-value 호출이 horizon을 재시작하면 안 됨.

## 5. 전파 수식과 실행 위치

- 기존 Estimator 수식과 같은 discrete 모델을 사용한다. `a₀=R(a_prev−Ba)−g`, `ω=(gyro_prev+gyro_now)/2−Bg`, `R₁=R·normalize(deltaQ(ωdt))`, `a₁=R₁(a_now−Ba)−g`.
- `P₁=P+Vdt+(a₀/3+a₁/6)dt²`, `V₁=V+(a₀+a₁)dt/2`. 고속 회전에서 deltaQ를 exact exponential로 바꾸는 별도 알고리즘 수정과 섞지 않는다. 기존 model parity와 독립 analytic 적분 오차를 둘 다 검사한다. [Estimator:86–101](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:86)
- `feedIMU`의 **기존 validated/interpolated interval**을 observer처럼 받는다. 이전 endpoint time/raw values와 새 endpoint를 먼저 복사한 뒤 shadow를 갱신; backend가 reset했으면 같은 frame prediction 폐기.
- 각 positive dt에 대해 shadow endpoint와 engine 이전 endpoint가 일치해야 한다. first sample dt0은 sample anchor만 갱신, 위치를 임의 전파하지 않는다. future sample은 기존 queue에 남긴 상태로 정확히 한 번만 사용.
- image endpoint 도달 시 shadow state를 snapshot한 뒤 LK/optimizer 진행. previsual snapshot을 local/copy로 고정하므로 실패한 solver candidate나 `slideWindow()`가 prediction을 바꿀 수 없다.
- camera 출력: `R_WC=R_WB·R_BC`, `p_WC=p_WB+R_WB·t_BC`. current `publishPose`와 같은 수식이나 visual getter flag는 건드리지 않음.

## 6. 초기 bounds·실패 정책

- 초기 개발 제안: **last clean visual seed부터≤0.5초**, 각 실제 IMU 간격≤0.05초. 이는 field accuracy 보증이 아니라 보수적인 운영 budget이며 plan review에서 고정할 값이다. 기존 engine gap0.5초를 IMU prediction의 acceptable sample gap으로 그대로 사용하지 않는다.
- horizon cap은 `seedTimestamp` 기준으로 absolute elapsed time을 계산. 중간 Lost, blank, valid prediction, display refresh는 cap을 연장하지 않음. cap 도달 후 새 clean visual seed 전에는 unavailable.
- finite만으로 orientation validity를 대체하지 않음: proper rotation, finite P/V/Ba/Bg/g, positive finite g norm, 이전/현재 timestamp, dt>0, camera extrinsic validity 확인. seed의 existing quality/bias warning이 있으면 seed 불가.
- 마지막 body velocity는 seed에 포함. 이를0으로 임의 설정하면 이동 중 featureless 진입 순간 정지로 왜곡된다. 반대로 velocity error가 있으면 위치 오차가 시간에 선형 누적됨을 숨기지 않는다.
- soft visual reason whitelist의 시작안: `empty_features`, `insufficient_current_features`. `insufficient_final_current_features`는 rollback 완결/quality 정상 증거와 추가 regression을 갖춘 뒤 별도 허용 검토. 임의 `return false` 전체를 fallback 성공으로 바꾸지 않음.
- hard invalidation: reset/configure/epoch mismatch, estimator generation 변경, timestamp 역행/중복, nonfinite sensor/calibration/state, IMU transport loss/overflow, frame gap, IMU gap, missing bracket, propagation endpoint 불일치, invalid solver state/처리 exception.
- missing_future에서 일부 실제 IMU까지만 적분된 상태를 image timestamp로 표시하면 안 됨. 1차안은 coverage 실패 시 seed를 폐기하고 다음 clean visual 수락까지 재개하지 않는 보수 정책 권고.
- 빠른 움직임의 한계: IMU-only는 visual observability를 복구하지 않고 기존 V/Ba/Bg/scale 오류를 유지한다. motion blur·센서 clipping·시간 오프셋·초기화 이전 무텍스처에서는 정확한 위치 추적을 약속할 수 없다. high angular step/acc magnitude는 기록하고, 실제 sensor range/수락 시험 없이 임의 “고속 지원” 표기 금지.
- 오차 감도 예: 정렬 오차θ의 중력 누출은 대략 `δp≈0.5gθT²`, accel-bias는 `0.5δaT²`, seed velocity error는 `δVT`. θ=1°·T=.5초면 약.0214m, δV=.2m/s면.10m. **이 예시는 측정 오차 상한이나 confidence가 아님**.

## 7. floor와 연결할 때의 core 경계

- 바닥은 마지막 visual-valid map의 **admitted·solved·nonpending** 점에서 얻은 plane이나 별도 검증된 관측이어야 한다. IMU gravity는 up 방향을 주지만 바닥 높이·거리·관측 여부를 주지 않는다. [map admission](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:410)
- floor detector는 engine epoch, visual observation timestamp, plane normal/distance, inlier count/spatial span, residual, source를 함께 반환하는 별도 기능으로 두는 것이 안전하다.
- IMU fallback 중 새 map point·floor support 생성0. 과거 인증 plane을 표시한다면 stale/age를 명시하고 epoch reset 시 제거. 예측 높이를 plane에 clamp하거나 plane을 backend prior/measurement로 넣는 것은 별도 알고리즘 변경이며 이번 최소 predictor에 포함하지 않는다.
- 한 장의 무텍스처 벽/바닥에서 plane을 새로 발견했다고 보고하지 않음. floor 표시가 안정해 보여도 metric translation 정확도·visual recovery 성공으로 승격하지 않는다.

## 8. 변경 파일 후보와 소유 경계

| 파일 | 최소 변경 후보 | 유지할 계약 |
|---|---|---|
| `include/vio_engine.h`, `src/vio_engine.cpp` | fixed-size predictor state, mode/getters, accepted seed/IMU observer/previsual snapshot/reset | processFrame return·visual pose flag/status·map·backend 정책 기존 의미 |
| `include/backend/estimator.h` | gravity copy getter 또는 VINSResult+gravity의 읽기 전용 snapshot getter | mutable state 노출·optimizer 변경 없음 |
| `wasm/vio_bindings.cpp` | 별도 prediction API binding | 기존 pose buffer/export 의미 보존 |
| `tests/test_engine_contracts.cpp` 또는 새 focused core test | analytic propagation/lifecycle/map separation/native parity | 기존 blank Lost/NaN/map0 assertion 삭제0 |
| Worker/Wrapper/UI | 별도 prediction object validation/display ownership | parent의 별도 lane; metric/limited schema와 혼합0 |

- mode를 켰을 때도 backend evolution이 OFF와 같아야 한다. 모든 visual pose·solver count·feature admission·prior·epoch가 같고 prediction output만 추가되는 것이 1차 수락 조건.
- floor 구현과 predictor는 서로 estimator data를 수정하지 않는 독립 consumer로 시작할 수 있어 병렬 검토 가능. 공유 API/epoch 계약은 leader가 통합한다.

## 9. 필수 tests와 수락

| 시험 | 독립 예상 / 반드시 확인할 것 |
|---|---|
| 초기화 전 blank+IMU | prediction 없음; NON_LINEAR flag만 강제로 세운 상태도 clean visual seed 없으면 없음 |
| seed→정지/gravity only | body P/V/R 일정, lever-arm camera 변환 정확; current timestamp의 별도 prediction만 생성 |
| seed→constant velocity | `P=P₀+V₀T`; V를0으로 강제하지 않음 |
| affine acceleration | analytic P/V 적분, irregular sample rate·phase·interpolated image endpoint·future carry0회중복 |
| constant angular velocity | 독립 rotation expected 및 기존 discrete integration tolerance; nonidentity extrinsic/lever arm 포함 |
| featureless early-return | 기존 `false`, Lost, poseNaN/false, map0 유지 + prediction은 whitelist 조건에서만 valid |
| optimizer current rejection | previsual snapshot이 solver/window 후 mutation과 달라지지 않음; OFF/ON backend state·prior·features 동일 |
| final invalid candidate/exception | whitelist 밖 prediction unavailable; 실패한 state를 seed로 쓰지 않음 |
| horizon boundary | .5초 직전/정확/직후; blank·Lost·반복 getter·mode same-value로 cap 재시작0 |
| gap/bracket/transport loss | .05 sample bound 양쪽, engine .5 gap, missing future/past, overflow/out-of-order/nonfinite에서 unavailable+seed policy 확인 |
| finite but invalid rotation / propagation overflow | old finite state를 fresh prediction으로 재사용0 |
| reset/configure/epoch/instance | 다른 Engine 영향0, reset 직후 없음, old getter/buffer/Worker message 재사용0 |
| visual recovery/reanchor | 정상 visual pose만 기존 채널에 전달, 새 seed가 실제 R/P/V/Ba/Bg/g/timestamp와 일치; 다음 loss horizon은 그 seed 기준 |
| PnP mode | unsupported policy 또는 backend-only timestamp 규칙 명시 검증; stale velocity/time stitching0 |
| full playback/GT | visual metric coverage는 기존 분모 유지; inertial segment 별도 수/시간/ATE/RPE·seed drift 기록. prediction을 제외해 전체 표시 trajectory 오류 숨기지 않음 |

- 재사용할 기존 regression: [blank removes tracks / no pose/map](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/tests/test_engine_contracts.cpp:637), [analytic IMU rate/phase/carry](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/tests/test_engine_contracts.cpp:672), [sensor pause extrapolation 금지](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/tests/test_engine_contracts.cpp:733), [bounded no-visual IMU history](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/tests/test_engine_contracts.cpp:800), [camera-body output transform](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/tests/test_engine_contracts.cpp:913).
- protocol tests는 prediction output을 기존 metric success로 세지 않도록 native/WASM/Worker/Wrapper/App까지 연결해야 한다. 실기기 GT가 없으면 “짧은 IMU 동작 실행 확인”까지만 주장.

## 10. 구현 전 결정할 항목

- 권고안: C++ bounded shadow prediction + 별도 output channel, configure default OFF, UI 명시적 mode, 초기≤.5초/≤.05초 sample-gap budget, clean backend seed, PnP unsupported부터 시작.
- 동작 자체에 새 사용자 permission은 필요하지 않으며 사용자가 구현까지 요청했다. 현재 단계의 제품 수정 보류는 leader가 요청한 **frozen plan/review gate** 때문이다.
- 이 문서는 설계·코드검토 결과. 신규 predictor 구현/실행/test 통과 또는 floor 인식 성공 근거는 아직 없다.
