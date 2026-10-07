# Engine lane 수정·회귀 계획

- 소유: `src/vio_engine.cpp`, `include/vio_engine.h`, `wasm/vio_bindings.cpp`, VIOSystem·MeasurementProcessor·tiny CLI·FeatureTracker 대응 source/header, `tests/test_engine_contracts.cpp`, native adapter 회귀
- 보존: 시작 전 dirty tracked/untracked 작업, 기존 build/artifact/dataset/service. CMake·Estimator·Optimizer·PnP 수학은 인접 lane 소유
- 냄새·결함: configure destination alias/무검증 calibration, frame raw pointer/count/크기 무검증, 미래 IMU 폐기·endpoint 값 혼동, reset tracker/cache 잔류, 빈 image/PnP 실패 stale pose, native 중복 IMU/feature/estimator orchestration
- 먼저 재현: invalid configure·SO(3)/model, alias noncommuting rotation, reset tracker feature 잔류. 기존 archive 기반 독립 fixture red 로그 보존
- 수정: 임시 소유 calibration 값 검증→적용; engine 한곳의 ordered bounded 미래 queue·dt0 seed·정확 endpoint 보간; 요청마다 pose fresh 무효화·frame/pose timestamp/reason·reset epoch; native raw image adapter가 같은 engine 호출
- 회귀 oracle: constant/ramp IMU60/100/200Hz × camera10/20/30/60Hz·phase, `sum_dt` 절대오차1e-9초, 중복/역순/미래0입력/gap/overflow; nonidentity extrinsic·lever arm·row-major, 빈 image·내부/외부 reset·불량 입력
- 검증: build owner `build/refactor-native` target 등록·2 jobs 빌드, EngineContracts/NativeAdapter/기존 회귀, sanitizer owner 통합 ASan/UBSan, matched fresh WASM/binding build. 원본 dataset 읽기 전용
- 삭제 gate: native viewer 소비는 engine camera pose/map adapter 유지; 기존 measurement feature API는 소비 조회·raw parity 후 제거. public `configure/processFrame`과 `T_W_C` row-major4×4 유지
- 한계: synthetic/native 계약은 실제 phone sensor·현장 GT·thermal/production 증거와 별도

## 재현·구현 상태

- 기존 archive 기반 configure/model·alias·tracker reset: 3/3 red; `build/refactor-evidence/engine-red/contracts.log`
- R0 원본 header + friend test seam / 기존 native archive 기반 independent ramp·future carry: 2/2 red; `build/refactor-evidence/engine-red/imu-oracle.log`. 원본에 friend 선언만 추가, runtime 동작 변경 없음
- 기존 ramp endpoint3.19≠3.0, delta_v2.164948728≠2.0(200Hz/60Hz); 미래 bracket만 전달 후 신규0 frame에서 integral 생성 없음
- 수정: copied calibration·KB0/PINHOLE2·finite/SO(3), 실제 dt0 seed·정확 endpoint/futurecarry, queue4096·visual 미갱신 IMUhistory4096 제한, current pose/time/map 무효화·epoch, native raw adapter 공통 engine·viewer compile guard·`--headless`, reused input image clone
- 회귀: engine15 cases 안의 constant/ramp·accel/gyro·rate/phase96 scenarios; native adapter6 cases. 새 getter/binding·native source6개 TU syntax pass; 실제 linked green은 통합 build 증거로 별도 기록
- shared API: `getEpoch` uint64_t(native)/number(WASM), `getPoseTimestamp`/`getFrameTimestamp` double(-1 invalid), `getPoseFresh`/`getPoseValid` bool, `getLastReason` string, `getLastSolverIterations` int, `getLastSolverTermination` named Ceres termination string

## 실제 검증

- 기존 실패6개: configure/model·alias·reset3, IMU ramp/futurecarry2, native ns1ULP1. `build/refactor-evidence/engine-red/` 로그 보존
- native engine12/12·raw/headless adapter5/5 이전 linked green; 이후 quality-warning·IMU pause·canonical ns fixture 추가 후 통합 native 재실행 예정
- ASan/UBSan: numerical18 +engine14 = **32/32 pass**; `build/refactor-numerical/sanitize-complete/combined-test.log`
- instrumentation: numerical20 +engine10 fresh repository translation units; Eigen/compile flags 유지, `-O1 -g1 -fsanitize=address,undefined -fno-omit-frame-pointer`; old repository `lib*.a` 제외·고정 third-party Ceres/OpenCV/gtest는 별도 미instrumented
- engine objects/commands/source hash: `build/refactor-numerical/sanitize-engine/compile-manifest.json`; canonical ns independent green: `build/refactor-evidence/engine-red/ns-oracle/green.log`
- IMU pause: camera1초 지속·신규IMU0, integration cursor 마지막 real0.1초 고정·freshfalse/LOST/imu_missing_future. 재개 original0.7초에서 gap0.6초→epoch reset/imu_gap
- 단순화: VIOSystem의 중복 estimator/IMU/feature math 삭제·camera/body viewer adapter 보존; repo caller0인 MeasurementMsg/ImageFeatureMsg/legacy extraction/FeatureTracker owner 제거. raw parser에만 집중
- 실제 phone·현장 GT·production 정확도·열성능은 이 lane 검증 범위 밖; fresh WASM build/served source hash·실 Worker replay는 leader/build/browser gate

## 최종 source freeze

- production/test source12개 fingerprint: `build/refactor-evidence/engine-green/source-frozen.json`; 이후 실제 통합 실패 수정 외 변경 없음
- 공통 execution API: `setExecutionParams(seed,cv_threads)` (seed≥0, threads1~4), `getExecutionSeed()` (configured·미설정-1), `getCVThreadCount()` (실제 OpenCV). Native/Worker에 explicit0/1 적용, reset에 동일 profile 유지
- engine15번째 회귀: 실제 RNG seed0 normalization(`0xffffffff`)·CVthreads1·reset/reject 확인. 이 추가 API 이후 마지막 native/WASM/sanitizer 재빌드로 최종 증거 갱신
- native adapter6번째 회귀: epoch1520216380280000000ns→1520216380.2800002sec exact equality; 이미지·IMU 모두 browser `Number(ns)*1e-9` 계약. 기존 `/1e9` 1ULP 차이 red→green
- packet golden oracle: `build/refactor-evidence/engine-green/browser-number-golden.txt` (Node 실제 Number 결과). 잘못된 `.01*i` 기대값을 canonical source bytes의 독립 값으로 교체
- decode 실패도 engine에 invalid frame으로 전달; 마지막 요청 timestamp·freshfalse·invalid_frame 확인. 실패 frame early-return의 이전 pose/status 노출 방지

## 최종 green·lifecycle

- 최종 native: EngineContracts **15/15**, NativeAdapter **6/6** exit0; `build/refactor-evidence/engine-green/{contracts-final,native-adapter-final}.log`
- 최종 fresh repository30 TU ASan/UBSan/LSan: **33/33, exit0**; `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1`, `UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1`; suppression·leak disable 없음
- 최종 sanitizer binary/command/object hashes: `build/refactor-numerical/sanitize-engine/{numerical-engine-raii-sanitized,raii-link-manifest.json,raii-test.log}`
- OpenCV4.5.4/TBB 별도 probe: explicit thread1 lifetime 미해제는792B/3alloc leak, 마지막 thread0 release는 exit0. `build/refactor-evidence/engine-green/tbb-profile/` original red/probe 로그 보존
- 실제 installed library에서 thread0의 reported count32 관측; active requested1은 reported1 유지. getter 숫자를 임의1로 치환하거나 sanitizer suppression으로 처리하지 않음
- ownership: explicit execution profile의 synchronous engine destructor에서 OpenCV scheduler0 release; unprofiled engine에 global 변경 없음. OpenCV RNG/thread control과 기존 config는 process-global, concurrent independently-profiled engine 지원 주장 없음
- final native/WASM artifact timestamp가 이번 source보다 최신 확인; source→artifact/served hash·HTTPS/browser replay 최종 판정은 leader 소유

## 최종 독립 ray 검토·필수 수정

- RANSAC의 기존 divide-before-validation 결함: zeroZ/negativeZ/NaN current·next ray6개 중 invalid pair1개가 살아남아25 vs golden24 관측
- 앞선 새 undistorted ray 필터의 historical vector 정리 누락도 재현: prev16/current8 잔류→다음 LK status8에서 heap-buffer-overflow. 이 회귀는 R0 전체 baseline이 아닌 최종 ray 수정 직전 source 기준
- red source/log: `build/refactor-evidence/engine-red/ray-validity/{feature_tracker.cpp,feature_tracker.h,red.log}`
- 수정: 양쪽 ray의 finite·positiveZ 검사 후 division, finite float-representable projection만 scoring, 동일 mask로 prev/cur/next point·normal·velocity·id·track 배열 정리. mask에 대응하지 않는 history tail 제거; clamp·camera model 변경 없음
- 기존 최소30개 RANSAC 조건은 invalid pair 제거 후 적용; valid geometry24개와 metadata identity golden 유지, 실제 다음 LK frame 실행·ASan 검증
- 최종 EngineContracts **17 cases** +NumericalContracts18 = **35/35 ASan/UBSan/LSan exit0**. `build/refactor-numerical/sanitize-engine/{rayguard-test.log,rayguard-link-manifest.json,numerical-engine-rayguard-sanitized}`
- Transport diagnostics: additive `getIMUEndpointTimestamp()`=실제 integration cursor, unknown/reset-1; frame input timestamp에서 endpoint를 추정하지 않기. 기존96 integral·pause·reset 회귀가 public getter 검증
- final native/WASM source/artifact 재빌드·pilot은 leader/build owner 후속; 이 수정 이후 frontend/code 추가 scope 없음

## 실제 parity 첫 divergence·total order

- 실제 같은5frame byte packet: frame0 substantive stage 동일, frame1 CLAHE/LK/status/mask_input/base_mask 동일; 최초 fork는 mask_sorted(native id97 vsWASM id70/count2). 이전 frame4 features148/147 차이의 선행 원인
- before evidence: `build/refactor-diagnostic/{native-before-total-order/results.json,wasm-before-total-order.json,frontend-before-comparison.json}` (5frame 미초기화는 expected bounded diagnostic, accuracy acceptance 제외)
- 기존 track_cnt-only comparator: libstdc++/libc++의 equal priority 순서 미지정. 수정은 track_cnt내림차순→id오름차순→finite pixel x/y 오름차순. 높은 track 우선·기존circlemask·min-dist·RANSAC 조건 유지
- 독립golden4nearbytracks: old{id9,id5}≠expected{id9,id2}; higherpriorityid9 유지·equalgroup lowerid2 선택. `build/refactor-evidence/engine-red/tie/red.log`
- defaultOFF opt-in `setDiagnosticCapture/getFeatureDiagnostics`: 각stage 최대1000point, CLAHE/imageFNV·GFTT/LK/status·Fvalid/inlier·mask input/sorted/output·final rays/velocity·effective config/Projection sqrtInfo. 캡처 중 성능 측정/주장 제외
- JSON fixture parser: `build/refactor-evidence/engine-green/{diagnostic-fixture.json,diagnostic-json-validation.log}`; valid JSON·actual maxrows16≤1000·OFF 시 enabledfalse 확인
- 최종 EngineContracts19 +NumericalContracts18 = **37/37 ASan/UBSan/LSan exit0**, suppression/leak disable 없음; `build/refactor-numerical/sanitize-engine/{totalorder-test.log,totalorder-link-manifest.json,numerical-engine-totalorder-sanitized}`
- actual after5/60 및 full room1/4 parity·GT·fresh deployment는 leader/build readout 후속; source fingerprint 이번 snapshot으로 갱신

- 최신 plain native source/binary 재검증: `build/refactor-native/test_engine_contracts` **19/19**, `test_native_adapter` **6/6**, exit0. 로그 `build/refactor-evidence/engine-green/{contracts-final,native-adapter-final}.log`; 파일/빌드 소유자와 분리된 최종 기능 회귀 확인

## Marginalization 진단 metadata 후속

- 수치 owner의 approved portability API 반영: `getFeatureDiagnostics().params.marginalization_assembly_threads`는 `backend::factor::kMarginalizationAssemblyThreads` 실제 constant1; CV/Ceres/BLAS thread 수와 구분
- 변경은 getter property 한 줄; Engine 기능·RNG·solver stopping·calibration/IMU math 추가 변경 없음. Read-only independent Marg review는 `final-numerical-peer-review.md`
- `source-frozen.json` 현재 Engine source hash 갱신; 새 MarginalizationInfo layout의 전체 transitive native/WASM/SAN/520은 build owner 후속. 이전37SAN을 새 layout 검증으로 보고하지 않기
