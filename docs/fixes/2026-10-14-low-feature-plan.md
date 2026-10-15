# 바닥 후보·저특징 IMU 예측 개발 계획

- 요청: 고속 이동/무특징 벽에서 Lost가 발생하는 현재 경로를 reference와 비교하고, 바닥면 인식·IMU 사용을 추가.
- 상태: **v3 Architect·Critic APPROVE, 구현·검증 진행 중**. 승인 원본 `build/low-feature-20261014/plan-v3.md` SHA256 `ce6e91442165d9ba5e93258958963a2b24dc262d177693bcc06418e687c46108`; 리뷰 근거 `build/low-feature-20261014/review-gate/approval.json`. 사용자의 계획 후 구현 요청에 따라 `execute`로 전환.
- source baseline: `5d48e4b` + 직전 U/R·서버파일 승격 변경. 원본 전체 hash는 `build/low-feature-20261014/intake.json`. 이전 dirty·dataset·SDK·서비스 보존.
- Reference: [8th Wall/ARCore/ARKit 비교](../research/2026-10-14-low-feature-references.md), [core 경계](../research/2026-10-14-low-feature-core-analysis.md), [browser 경계](../research/2026-10-14-low-feature-browser-analysis.md), [시험 설계/실제 baseline](../research/2026-10-14-low-feature-test-design.md).

## 원칙·선택

- 관측된 metric pose와 무특징 구간의 IMU 예측은 별개. 예측은 `poseFresh`, 지도, optimizer/prior, GT visual coverage로 승격하지 않음.
- 바닥 높이는 gravity만으로 알 수 없음. 실제 현재 영상에서 지지되는 solved world points로 기하 수평면 후보만 생성. semantic floor/table 구분은 미지원으로 명시.
- 기존 geometry/rank/cheirality/reprojection·IMU clock/bracket·epoch/motion guard 유지. 실패한 H4/C3/D2/D3·floor-height prior·vendor smoothing 이식 금지.
- 한정된 시간의 시각화 연속성 확보와 실제 metric 추적 회복을 분리. 빠른 움직임의 blur/잘못된 correspondence를 예측으로 수리했다고 주장하지 않음.

| 대안 | 장점 | 제한/결정 |
| --- | --- | --- |
| A. 기존 mutable backend P/R을 Lost에서도 정상 pose로 출력 | 적은 변경 | 실패 solver/window 상태 혼입·false fresh·지도 오염. 기각 |
| B. 마지막 clean visual anchor의 별도 IMU shadow 전파 + 독립 평면 후보 | backend 불변, 검증 가능한 수식·수명·출력 경계 | 짧은 구간만 연결, 불확실한 translation. **선택** |
| C. floor prior/관성 예측을 optimizer에 직접 결합 | 장기 안정화 가능성 연구 | floorheight/관측성/정보상관을 입증하지 못함. 후속 연구로 보류 |

- 결정 요인: metric 상태 무오염, reset/시간 정합, 실제 browser에서 명확한 품질 표시.
- ADR: B를 구현하되 normal visual 상태/출력은 baseline과 동일하게 유지. 새로운 출력은 기존 metadata와 독립 검증. source/newWASM/active승격은 별도 증거로 기록.
- 위험 사전검토: (1) 과거/실패 state를 예측 anchor로 사용, (2) flat wall/table을 확정 floor로 오인, (3) TTL을 새 callback마다 연장. 대응은 아래 seed/domain/identity/expiry 회귀로 고정.

## P — C++ 한정 IMU 전파

- 소유 경계: 새 `include/utility/inertial_prediction.h`, `include/vio_engine.h`, `src/vio_engine.cpp`, `wasm/vio_bindings.cpp`, native 계약 테스트. `include/backend/estimator.h`에는 private `g_`를 값으로 반환하는 read-only `getGravity()`만 추가. Estimator/Optimizer/FeatureTracker 수학·gate는 변경하지 않음.
- shadow 상태: body P/R/V, frozen Ba/Bg/실제 estimator g, camera↔body extrinsic, raw previous acc/gyro, epoch, anchorTimestamp/currentTimestamp. 현재 backend에 write-back0.
- seed는 initialized + 실제 usable backend visual result + reason 정확히 `tracking` + finite/유효 SO(3) state에서만. `getLatestVINSResult()`의 P/R/V/Ba/Bg timestamp, `frame_timestamp_`, `current_time_`가 같은 imageTimestamp여야 하며 publish할 P/R과도 일치해야 함. getter로 읽은 실제 gravity를 복사하고 같은 engine/reset generation에서만 사용. PnP 활성은 초기 버전 미지원으로 명시하고 seed/prediction을 제거. quality warning이 있는 visual frame은 seed를 갱신하지 않고 기존 shadow도 제거.
- 전파는 `feedIMU`가 이미 보간한 동일 endpoint 구간을 전달. zero-dt는 새 측정 endpoint만 초기화하고 적분하지 않음. gyro bias 보정·SO(3) exponential, endpoint world acceleration의 affine moment로 P/V 갱신. 별도 gravity/bias 보정이나 정지검출 가정 없음.
- engineering 상한: source anchor→prediction **0.5s 이하**, 연속 **원본** sample gap **0.1s 이하**, anchor 대비 이동 **5m 이하**. 0.1s는 기존 phone 관측 max 약68ms를 포함하는 상한이며 정확도 보증이 아님. 모든 시간은 seconds. 경계의 부동소수 오차 처리는 명시된1e−9s 수준으로만 허용.
- raw gap 검사는 interpolated endpoint/current_time과 별도 raw-left timestamp로 수행. 원본sample→원본sample과 현재frame을감싸는 raw-left/right 구간을 확인. raw0.20s 구간이 카메라0.05s endpoint들로 나뉘어도 거부. 해당 frame의rawcoverage가부적합하면 재seed도금지; 정상raw구간+새cleanvisual때만재seed. 기존metric경로의gap0.5s 정책은변경하지않음.
- 이상 sample·missing bracket·out-of-order·overflow·gap·frame/config error·reset/configure·epoch변화·exception·expired horizon은 shadow/prediction을 무효화. 만료 뒤 다음 clean visual anchor 전까지 다시 시작하지 않음.
- 허용 visual failure reason은 `empty_features`, `insufficient_current_features`, `insufficient_final_current_features`만. 별도 `last_solver_quality_reason_`도 비어 있어야 함. `insufficient_final_current_features`+`relative_translation_threshold` 조합을차단. 이미 초기화됐더라도 invalid_pose/relative jump/IMU 오류는 예측으로 우회하지 않음.
- output getter `getInertialPrediction()`은 JSON string 또는 `null`. 값이 있으면 schema `mobile-slam-inertial-prediction-v1`, `mode:'inertial-only'`, `poseFrame:'camera'`, `engineEpoch`, `timestamp`, `anchorTimestamp`, `maxHorizonSeconds:0.5`, row-major `pose[16]`, `visualSupported:false`, `metricTranslationSupported:false`, `positionUncertain:true`. covariance/정량 confidence는 제공하지 않음.
- getter는 해당 frame의 완전한 IMU coverage와 정확히 같은 timestamp에서만 제공. 정상 visual pose가 있으면 prediction=null. 기존 `processFrame` 반환값·pose/status0..4·poseFresh/valid·map API는 그대로 유지.
- 이 API는 camera-frame 처리 시점에 생성. 별도60Hz/IMU event rate pose producer나 마지막 IMU 이후 외삽을 구현했다고 주장하지 않음.
- 활성화: 새core의독립side-channel은기본계산, 새Worker/App에서조건을만족하면기본표시. 기존metric경로기본값·status는변경0. 새API가없는구형WASM에서는unavailable이며임의JS추정기로대체하지않음. 정량covariance가없다는사실과positionUncertain=true를명시하며, 수치상valid는정확도certificate가아님.
- 새 public `invalidateInertialPrediction()`은 shadow만폐기하고backend/epoch를바꾸지않음. Worker의missing_bracket/실제frame입력거부처럼엔진호출이생략되는경로에서도호출. 이후completeIMU+blank가와도oldanchor에서재개0; 새cleanvisualseed만허용. getter가있어도invalidationAPI가없는부분호환binary는predictionunavailable.

## G — 현재 visual support의 바닥 후보

- core의 새 read-only `getGroundSupportPoints(output,maxCount)`를 추가. `has_valid_pose`, clean tracking, PnP OFF에서만 기존 `Estimator::getSolvedFeatures()`를 재사용하고 현재 tracker ID·track_cnt>1·finite 현재 normalized observation과 교집합. 이 경로의 기존 depth admission/solve_flag1/latest projection-outlier 배제를 유지. 원래 map 출력은 변경하지 않음.
- 중력 부호: 기존가속도식은`R*(acc-Ba)-g_`; 여기서`+normalize(g_)`가world up. `g2R`이이를+Z로맞춘현재계약을확인하고 groundexport는유한nonzero g 및+Z와1°이내정렬일때만허용. 임의g부호로ceiling/wall을floor로변환하지않음.
- 현재 feature ID 교집합이 아닌 과거 map 전체를 이번 frame의 새 지지로 계산하지 않음. API는 최대512 XYZ만 export; 상한/포인터/finite 검증과 실제 frame identity 유지.
- 새 pure JS module `web/js/low-feature.mjs`: prediction validator와 bounded horizontal-plane fitter/plane state를 공유. Worker·Wrapper의 중복 프로토콜 정의를 만들지 않음. VM/data-URL test bootstrap은 실제 module source를 주입/절대 URL로 연결해 테스트.
- floor fit: 유일 point만 사용, 최대512점·고정 seed/최대64후보 RANSAC. up-normal tilt≤15°, camera가 plane의 위쪽>0.1m, point-plane 오차≤0.03m, inlier≥20 및 전체의≥60%, 두 XY 방향 span≥0.3m·XY covariance 작은 eigenvalue≥0.0025m². inlier centered least-squares로 plane을 재추정하고 모든 gate 재검사.
- above-camera ceiling·vertical wall·line/cluster·sparse·nonfinite는 거부. 서로 다른 높이의 두 horizontal 후보가 비슷하게 지지되면 모호 상태로 거부(예: runner-up support가 best의80% 이상이고 높이 차>0.1m). table과floor가 동일 geometry이면 동일 geometric candidate일 수밖에 없으므로 semantic confidence를 생성하지 않음.
- 기본 plane 식 `n·X+d=0`, norm1/n.z>0, centroid·유한4corner extents·support·RMS·sourceTimestamp·engine/client epoch 제공. label은 **바닥 후보(수평면, 분류 미확인)**. 원점 grid는 바닥검출로 사용하지 않음.
- 최대5Hz로 clean current visual support에서만 재검출/갱신. Lost/IMU-only에서는 절대 새 fitting/point추가 없이 동일 epoch의 기존 후보만 `retained`로 최대1초 유지, sourceTimestamp를 갱신하지 않음. expiry·reset/epoch·quality warning·transport loss에는 제거.
- **plane state 소유자는Worker 한 곳**. 주기미도달·freshsupport부족·fit실패는유효기존후보를sourceTimestamp그대로retained로전달(TTL내); 명시모호multipleplane·nonfinite입력·quality/error/reset은즉시clear. Wrapper는매수신시검증, App은시각expiry만재검사하고별도fit/cache를만들지않음.
- `groundPlane` payload: schema `mobile-slam-ground-plane-v1`, `kind:'horizontal-surface-candidate'`, `semanticClassification:'unknown'`, `state:'observed'|'retained'`, engine/clientEpoch, sourceTimestamp/sourceSequence, normal[3], offset, centroid[3], corners[12], supportCount/sourcePointCount, rms, `maxAgeSeconds:1`. Worker는fit한frame만observed; 나머지는retained. counter/normal/corners/epoch/sourceTime·sourceSequence≤outersequence·TTL을검증. 미래/외부epoch/기형은null.
- plane은 renderer용 결과이며 P/V correction, cameraheight assumption, map/prior/PnP/optimizer factor로 사용하지 않음.

## B — Worker/Wrapper/화면 계약

- Worker는 새 getter가 없는 이전 WASM과도 호환: prediction/groundSupport=null. metric 처리·기존 jump/reset guard 이후에만 새 metadata를 붙임.
- prediction은 status3/Lost + initialized + 허용reason + finite homogeneous SO(3) pose + exact request/input/engine frame timestamp + epoch +0≤timestamp−anchor≤0.5에서만 수용. loss/motionDiscontinuity/error/unknown/old reply에는 강제로 null.
- Wrapper도 같은 validator로 검증하고 `...data`의 임의 payload를 그대로 통과시키지 않음. 예측으로 `_latestMapPoints`, `lastPublishedPose` 또는 metric fields를 갱신하지 않음.
- App은 visual pose를 우선. visual invalid이면 별도 `IMU prediction / position uncertain` 상태와 amber frustum을 표시. prediction display source age0..100ms, anchor horizon0.5s를 매 RAF 재검사. 도착시각으로 TTL을 다시 시작하지 않음.
- 시간식: `0≤target−anchor≤0.5s`와`0≤now−target≤0.1s`를동시에요구하므로최대anchor→display는0.6s. source horizon과wall deadline을더한0.6s를새source horizon으로쓰지않음. prediction,visual,plane은각자100/500/1000ms 표시한계를재검사; epoch/error에서는모든추가표시를먼저clear한후새결과를고려.
- renderer의 prediction 갱신은 raw trajectory에 append하지 않음. visual로 돌아오면 실제 pose를 그대로 적용하며 재연결 jump를 smoothing해 숨기지 않음. 기존 follow-view alpha와 visual500ms 정책은 유지.
- plane은 worldRoot 아래 별도 translucent mesh/wireframe. Z-up→Y-up 변환은 기존 worldRoot 한 번만 사용. retained는 색/상태로 구분, expired/epoch/reset은 제거. 바닥 후보의 support/나이 표시, IMU-only 새floor검출 주장0.
- recorder/raw reports에는 prediction과groundPlane metadata를 복사하되 원본pose·visualcoverage·ATE scorer에는 포함하지 않음. 통계는 `visualFresh`, `predictionAvailable`, `predictionDisplayed`, `planeCandidate`를 구분.
- 새 module은 synthetic/browser fixture·서버 allowlist의 기존 `.mjs` 경로로 제공. 기존 VM tests를 통과시키기 위해 product fallback을 만들지 않고 fixture loader를 실제 module import에 맞춤.

## 테스트·수락 조건

- baseline: 기존400frame3arm(control/0.25초blank/2초blank), 1200pixel hash·IMU/GT/profile 동일. 세 arm의 기존GT 실패를 보존. 근거 `build/low-feature-20261014/fixtures/`.
- positivefloor E2E: 고정`floor-visible-clean101`(camera40°down, 실제texturedfloor, 원래IMU/독립GT)을추가. 기존legacyexport기준357fresh/400·GT_PASS는feasibility이며신규getter검증대체아님. 새current-IDgetter→WASM→Workerfit→App/Renderer에서실제candidate>0을요구; 예상값은SE(3)scale1로engine/GTgauge를명시해평가. 실제observed fit≥40회 및observed+retained후보표시≥100frame을각각요구. GTpass율은observed fit만분모로삼아≥90%에서normal오차≤3°·offset오차≤0.05m를요구(반복retained표시를새관측으로세지않음),벽/ceiling/blank로새candidate생성0. 이수치는해당synthetic시험수락이며semantic/field정확도주장아님.
- positivefloor fixture에도short/longblank를주고floor.sourceTimestamp가갱신되지않는retained→1초만료,epochclear·재획득새관측을실제Worker에서검사. renderer축/색/expiry는독립knownplane이미지/수치로검사.
- native 독립 oracle: stationarygravity, constantvelocity, constantacceleration, known SO(3) rotation, extrinsic lever arm, dt partition·interpolated endpoint. gyro/bias/nonfinite/time/regression/domain tests 및 horizon±boundary, expired/unseeded/reseed/reset/failedconfigure/PnP unsupported.
- 추가red: raw0/.20s를frame.05/.10/.15s로분할해도prediction/seed0; Worker missing_bracket(엔진호출0)뒤completeIMU+blank는새anchor전까지prediction0. Lost의qualitywarning조합, 다른epoch/다른timestamp seed, actualg부호를검사.
- core integration: 실제 textured 초기화 후blank에서 prediction 존재, metric pose/status/map은 기존Lost/invalid/map0; covariance나confidence가 아닌unverified prediction으로 분리. 초기화전blank는없음. missingbracket·jump/reset은절대우회하지않음.
- floor unit: noisytiltedfloor/outliers PASS; verticalwall/ceiling/line/cluster/sparse/ambiguousmulti-plane FAIL; tablevsraisedfloor semanticambiguity표시; 중복points countinflate0; 현재ID/outlier/geometrypendingpoint제외.
- browser 계약: wrongidentity/future/expired/NaN/rotationinvalid/extraSDKmetadata reject; exactboundary100ms/0.5s; result 중복trajectory0; visualpriority·reset·predictionexpire·retainedfloorTTL·no-floor-fit-on-Lost; oldWASM gracefulunavailable.
- regression: 기존browser전체 및native전체, 새unit ASan/UBSan, freshWASM compile/APIbound/actualWorker 실행. 같은fixed-work 입력에서 metric pose/status/reason/epoch/source association baseline exact 또는 독립검증된기존numericaltolerance 내; 실패하면 기본제품승격금지.
- end-to-end: 짧은blank에 prediction coverage를 따로 세고 GT오차/anchorage 기록, 긴blank는0.5s 이후prediction0. textured복구 이후새cleananchor재시작. 기존GT_FAIL이prediction으로PASS로바뀌면평가기오염으로FAIL.
- prediction수치수락: 독립known-stateoracle는closedform오차≤1e−9. baseline부터GT_PASS인floor-visible-clean101의blank제어에서eligibleframe(직전cleanseed,rawcoverage,sourcecap)의producerprediction100%,0.5s초과0; anchor-relative GT displacement오차≤0.25m·relative rotation≤5°를전eligible구간에서요구. 기존fast707 GT_FAIL군은prediction오차도별도기록하되이결과를metricGT구제/물리정확도로승격하지않음. 실패하면기본표시승격보류·원시값보존.
- 비간섭범위: matchedfixed-work old/new native/WASM의visualpose/status/reason/epoch뿐아니라solveriterations/termination,currentfactor수,normalmap좌표/개수,windowP/R/V/Ba/Bg 및prior유무/차원/수치snapshot을비교. 새prediction·floorread-only호출전후도동일상태확인. 기존timer값만제외하고이름없는광범위tolerance로덮지않음. 실제runtimewallcap의timing-sensitive差이는별도실험으로격리.
- 속도: prediction·ground fit 비용을기록, source/drop/poseage 포함. 시간목표가아닌정책구현단계이며 phone속도향상주장0. native/WASM수락 후에도실제폰정확도는별도검증.

## 실행·소유권·완료

1. Planner가 이 고정안 작성 → Architect 검토 → 별도 Critic이 같은안 독립검토(Architect결과미전달). 둘의 결과를 Planner만합쳐수정, APPROVE 전 product edit0.
2. P/G core getter: C++/bindings/native tests 담당. pure module/floorfitter: `low-feature.mjs`+독립tests 담당. transport/harness: Worker/Wrapper+관련tests 담당. App/Renderer/recorder: 표시 담당. 공유파일 소유권 겹치면root통합.
3. `execute`에서 regression red→최소변경→green. C3/정책 기본값 변경0·새dependency0. 현재 서비스 시작/중지/재시작0.
4. 독립 correctness/evidence·standards 리뷰, 실제WASM/단계시험 후 문서갱신. 새WASM은별도build에서 검증하고 서버파일 승격은 검증통과범위만 별도 기록. 실제server실행은사용자직접.
- 사용가능 native agents: inherited-model analyzer/implementer/reviewer. 설치된 `executor` role의 gpt-5.4는계정에서미지원이므로 역할 override 없이 현재모델을상속. 하위위임0, 4개독립구현lane최대.
- 종료기준: 필요한변경/회귀/원시실패/신규ABI/provenance/리뷰확보, 미입증field범위명시. Lost해결을단순상태재분류로보고하지않음.
