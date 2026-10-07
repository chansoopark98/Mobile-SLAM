# Build·Evaluation 실행 lane

- 소유: evaluator·ConfigManager·CMake/shared source·parity/config/evaluator/replay tests·build/staging/provenance/deploy helpers
- 기존 dirty 작업·동시 lane 보존; engine/numerics/browser/server 파일 수정 금지
- 적용 skill: repository `mobile-slam-validation`; 실행·정량·실기기 claim 분리

## Cleanup·회귀 사전 계획

1. 기존 evaluator의 scale·rotation·matched time·empty·unique association을 독립 analytic fixture로 red 재현
2. ConfigManager public API로 finite·extrinsic·값 범위 검증; literal comparison 테스트 대체
3. Native/WASM core source 목록 한 곳으로 이동, viewer optional·headless 실행·정상 transitive link·test 등록
4. Native replay의 Ninja command 추출 삭제, CMake target·실제 pose timestamp/epoch/reason/solver·단일 future delivery·manifest 재사용
5. NAS room4는 archive hash·safe member·counts 검증 후 local ignored staging, 원본 읽기 전용
6. Components-ready 이후 fresh native·sanitized·WASM build·flags/hash 검증; 최종 latency 실험은 다른 build와 분리
7. Paired artifact/config/source manifest·atomic deploy helper 준비; parent deploy flag 이후만 승격

- 완료 기준: unit red→green, 실제 integrated CTest·fresh WASM, headless native replay, 안전 staging·provenance·deploy gate 작동
- 미검증 한계: 물리 phone jitter·capture latency·thermal·full SLAM·상용 license 판정


## 구현·관찰된 회귀

- C++ evaluator: rigid SE(3) 기본·diagnostic Sim(3) 명시, full matched pose/time/quaternion·unique GT association, 실제 relative SE(3) translation/rotation RPE.
- 빈/무효 결과: NaN + valid=false·save 거부; scale fitting으로 metric 오차 제거 금지.
- ConfigManager: public API 회귀, finite/SO(3)/noise/range validation·유효하지 않은 load의 이전 config 보존·callback mutex 밖 호출.
- Shared core: native/WASM source 그룹 한 곳; native viewer OFF 기본·Pangolin 미요구·tiny headless 및 3개 새 CTest suite, CPU DENSE_SCHUR용 USE_CUDA OFF.
- Native parity: 중복 Estimator 경로 제거, direct/raw adapter 동일 VIOEngine·profile·future packet·nonempty initialized pose strict 비교.
- WASM: 실제 compile -msimd128/-fno-fast-math·strict undefined symbols·Embind epoch 지원; legacy generated artifacts 삭제 없음.
- Native replay: 정규 CMake target, 미래 bracket 한 번 전달·actual pose timestamp/epoch/reason/solver·정확한 counts/profile·옵션 원시 input oracle.
- NAS room4: local staging·archive SHA/CSV+YAML inventory hash·ordered counts·cam PNG512 검증; DSO convenience symlink2 추출 제외 기록.
- Helpers: source/flags/dependency/artifact provenance, independent-cap parity freeze, exact native gray/IEEE754 packet SHA, parent-authorized singlefile atomic deployment+backup.

| 회귀 | 실제 결과·근거 |
|---|---|
| Evaluator analytic red | 5/5 fail, scale·90°rotation·unmatched prefix·empty·unique GT; `build/refactor-evaluation/evaluator-red.log` |
| Config public API red | 4 pass/3 fail finite·extrinsic·noise; `config-red.log` |
| Isolated evaluator/config green | 12/12·7/7; subsequent fresh CTest에 포함 |
| 최신 native CPU-only CTest | 9/9 suites pass, `native-ready-ctest.log`; mandatory skip 없음 |
| Python independent scorer | 9/9, `python-evaluator-final.log`; 실제 SLAM accuracy와 분리 |
| Input SHA oracle helper | 2/2 exact bytes·truncation rejection; `input-oracle-final.log` |
| Fresh WASM | strict link 성공; `wasm-frozen-build.log`·`build/refactor-wasm/build-provenance.json` |
| Final full fresh ASan/UBSan/LSan | 9/9 suites, 74 GTests, 125.01s, strict detect_leaks/UB halt, suppression 없음; `sanitizer-teardown-ctest.log` |

- Actual OpenCV header: native4.5.4, WASM4.5.5. 차이를 report·manifest에 보존; parity tolerance 자동 확대 금지.
- Parity independent caps: translation p95≤0.001m/max≤0.01m, rotation p95≤0.01°/max≤0.1°, init/fresh coverage/state/reason/solver·relative epoch exact.
- Numeric cap은 pilot 전에 parent 확정; 3회 full room1 pilot·within-platform 반복이 모두 통과한 뒤 동결, room4 이후 완화 금지.
- Physical phone·field GT·capture-to-display·thermal·full SLAM·commercial license는 이번 native/virtual 검증만으로 판정 불가.

## 변경 파일

- `CMakeLists.txt`, `wasm/CMakeLists.txt`, `CMakePresets.json`, `cmake/SLAMCoreSources.cmake`
- `include/utility/trajectory_evaluator.h`, `src/utility/trajectory_evaluator.cpp`, `include/config/config_manager.h`, `src/config/config_manager.cpp`
- `tests/{test_trajectory_evaluator,test_config_validation,test_vio_engine_parity,audit_dataset_replay}.cpp`, `tests/test_input_oracle.py`
- `scripts/evaluation/compare_trajectories.py`, `scripts/dev/{native-replay,check}.sh`, `scripts/dev/{build-provenance,stage-tum-room4,deploy-candidate,compare-replay,hash-inputs}.py`, `scripts/dev/README.md` non-service section
- 이 문서·lane status/result, ignored local staging/build/evidence. 다른 lane 소유 제품 소스 변경 없음.


## 최종 gate·인계

- Native 최종: `build/refactor-evaluation/native-teardown-ctest.log`, 9/9 CTest·74 discovered GTests, mandatory skip0, Engine15/NativeAdapter6 포함.
- LSan 중간 실패: imported OpenCV4.5.4/TBB scheduler `new[]` 792 bytes/3 allocations. Minimal OpenCV-only 재현·active1/actual1 유지 + explicit-profile engine destructor의 scheduler 종료0로 실제 해제. Suppression/LSan 비활성 없음.
- Final SAN: `sanitizer-teardown-build.log`·`sanitizer-teardown-ctest.log`, own engine/test/adapter/replay object 강제 재컴파일 후 전체9 suite·74 cases 통과.
- Browser module/API: `wasm-candidate-smoke.json`, 실제 Chrome에서 final candidate import·mandatory bindings·configure·epoch/reset·seed0/actualCV1 통과. Tracking/accuracy gate와 분리.
- Final artifact: `build/refactor-wasm/vio_engine.js`, SHA-256 `b813c97cf636fad4eee68bd5e6240caae2b063b662544d294441f2800514e9fb`, 5,957,393 bytes, embedded WASM.
- Manifest: `build/refactor-wasm/build-provenance.json`, source manifest SHA-256 `05cae05d058c1abdceb0f651597a9a999cc92f132b4c5c14f8ada20e07e4990f`; native `build/refactor-native/native-replay-provenance.json`.
- Handoff: `build/refactor-orchestration/{evaluation-status,performance-ready}.json` performance_ready=true, 이 lane의 모든 build/test 종료.
- 다음: parent가 실제7002·3회 full room1 pilots·independent-cap freeze·room4·paced/mock/loss·최종 승격/served hash를 순차 검증. 이 lane은 아직 배포·dataset parity 합격을 주장하지 않음.


## R5 후속 원인·수정 검증

- Superseded full pilot: 입력 gray/IMU hash·init45·coverage는 일치, pose 약1m·solver state 차이. 독립 cap 완화 없이 진단 전환.
- Exact first5 trace: frame0 CLAHE/GFTT141/final packet identical; frame1 CLAHE/LK/mask_input/mask image identical → 최초 `mask_sorted` 차이(native first id97/WASM id70), greedy survivors·추가 GFTT 차이로 전파.
- 원인: equal-track-count-only comparator의 libstdc++/libc++ `std::sort` tie ordering. 독립24/150점 및 실제 setMask golden old-fail/new-pass 확보 후 owner의 ageDesc→idAsc→pixelX/Y total order 수정.
- Backend consistency: native Ceres2.2 SCHUR_SPECIALIZATIONS=ON, installed WASM 설정과 일치. Frontend fork의 원인으로 혼동 금지.
- After5: 모든 frontend stage exact equality. After60: frame0~59 모든 stage exact, init45·15fresh poses·state/reason/solver/epoch exact; translation p95 `4.749270733680702e-10 m`, max `4.807429385971528e-10 m`, rotation max `3.818191820832846e-6 degree`.
- Scope: 짧은 development 진단, full3 actual Worker pilots·room4·paced acceptance 대체 금지. 기존 independent caps 유지.
- Source capture: opt-in defaultOFF·1000point bounded. 최종 실제 timing은 diagnosticsOFF.
- Final native: 새19 Engine +18 Numeric +6 NativeAdapter 포함9 suites 통과. Full SAN은 SchurON generated variants 갱신 중; 완료 뒤 parent에 hashes/counts 전달.
- Pre-existing modified: CMakePresets/scripts-dev README/check/native-replay와 audit_dataset_replay는 이번 실제 구현 이전 audit 단계에 이미 존재; R0 baseline fingerprint 없는 파일은 pre-existing-unfingerprinted로 기록.
- Newly created actual-refactor helpers: shared cmake source, input-oracle test, build-provenance/stage-room4/deploy/compare-replay/hash-inputs/wasm-smoke/frontend-diagnostic/compare-frontend. `doctor.py`는 이 lane 미수정.


## 현재 최종 snapshot — 이전 artifact 인계 superseded

- Native 최종9/9·78 GTests(Engine19/Numeric18/NativeAdapter6), `native-final-parity-ctest.log`.
- Same-SCHUR-ON full fresh ASan/UBSan/LSan 최종9/9·78 GTests, 123.04s, skip0·suppression0, `sanitizer-final-parity-ctest.log`.
- 최신binding/default diagnosticOFF/invalid initial endpoint/seed0/actualCV1/epoch/reset Chrome smoke: `wasm-final-parity-smoke.json` pass.
- 현재 JS: SHA-256 `882791a5f876cfda36d413058ea83725e6f73663e974e641ae85da8ad4f75d12`, 5,980,340 bytes; earlier `b813…` artifact는 superseded.
- 현재 native/WASM 공통 source manifest: `5624dc8c9ff4fa6a02b4abf77a8bf38467aee7bf4a89f059f516ddf2dd92110c`, 현재 sources와 일치.
- Lane build/test 모두 exit0·live 작업 없음. `evaluation-status.json`: builds_ready=true; authoritative performance_ready=false는 parent의 exact candidate 승격·7002 재시작까지 유지.
- 추가 compiler/test 실행 없음; full3 actual Worker room1 pairs·room4·paced와 field/production 판정은 parent gate에 별도 기록.


## Run-3 실패·Marg portability 현재 snapshot

- Failed run-3 보존: 3쌍 결과 동일, solver iteration171/termination42 차이; within-platform translation0/state0 exact. Cross p95 0.408634m/max0.515326m, first solver479. Tolerance freeze 실패·caps 완화 없음.
- Existing100 diagnostic: 모든 frontend stage0~99 bit-exact이나 pose drift62/94 재현, 실제 backend portability 경계로 제한.
- Effective threading: Ceres Solver default1·Optimizer unset, cv1; native4는 별도 marginalization modulo4/reverse-join reduction. CV 실행 profile이 이를 제어하지 않는 source/profile 차이 확인.
- Owner fix: independent normal/Schur oracle old-fail2→green2, first-seen semantic block/drop-kept order + shared single ordered assembly. EPS/noise/manifold/caps 미변경; `kMarginalizationAssemblyThreads=1` truthful metadata.
- Fresh transitive native/WASM/fullSAN 갱신: 각각9 suites,80 GTests(Engine19/Numeric20/NativeAdapter6), full strictSAN123.58s pass·skip0/suppression0.
- After520: 모든 frontend stage0~519 exact, init45·475fresh·state/reason/solver/epoch exact. Translation p95 6.843388e-7m/max7.310283e-7m, rotation max1.550957e-5deg. Previous firstsolver479 mismatch 제거; `after-marginalization-portability/520-comparison.json`.
- 현재 JS SHA-256 `b75645443e52c8b51a0b78116ecad2f9cd02e4d0d94b7e64d6af2897f6008bff`, 5,981,589 bytes; source manifest `f0f96be4215384ae4504e682765df05608ee529889e972c80e54e848dec24be9`. Previous882 artifact는 superseded.
- Lane CPU jobs 종료; parent-controlled promotion/restart/full3/room4 gate는 별도. Short520은 latency/현장/production 합격 근거 제외.


## Selected backend trace helper mini plan — logging only

- Scope: `tests/audit_dataset_replay.cpp`, `scripts/dev/frontend-diagnostic.mjs`; product optimizer/EPS/noise/stopping logic unchanged by this lane.
- Optional capture range `[start,end)`: default full existing diagnostic range preserved; selected1080..1110 means1080~1109, covers1093 outgoing→1094 incoming and1105 solver fork.
- Toggle existing diagnostic flag before selected frame input; no algorithm-dependent filtering, replay all1110 exact input frames.
- Retain scalar pose/state/solver rows for all frames; include opt-in backend/feature diagnostic JSON only when enabled.
- Validate explicit range bounds; range metadata exported for reproducibility. No new dependencies.
- Wait Num+Engine frozen/API review before any build. Fresh native/WASM all ABI dependencies+real module smoke, selected1110 native→WASM sequential.
- Logging-only sources: previous fullSAN evidence is historical; parent deferred fresh fullSAN until actual behavior fix, no new-code sanitizer pass claim.

## Conditioning trace 결과·QR 수정 검증 준비

- Selected1110 `[1080,1110)` 실행: native/WASM 각30 frame capture, 기존 run-4와 플랫폼별 pose/state 차이0. 입력·frontend stage·factor membership·margin flag·semantic layout·gauge branch 동일.
- 실제 최초 branch 차이: frame1094 iteration2, native candidate cost=`DBL_MAX`·unsuccessful, WASM finite cost·successful. Frame1105 stopping 차이는 이후 결과.
- Pinned Ceres2.2 source: `trust_region_minimizer.cc:772` candidate Plus/Evaluate 실패→`DBL_MAX`, `:125` rejected iteration cost 기록. Invalid linear/model step의 `HandleInvalidStep:487`은 finite current cost·step norm0·relative decrease0. 정확한 실패 factor ID는 미확정.
- Trace-only artifact `edcb31c6c6ae21fe9140b4753282f5e46365800eae83832cc39cc058b9ab3239`; source manifest `bc316b8c20d959713ddd92262de08942f542edea08af70c2057127248da8e3d5`. 최신 logging-only 소스 fullSAN은 미검증. 실제7002의 이전 `b756…` artifact·PID1947692 보존.
- Parent 승인: Numerical owner의 독립 QR oracle/red→green·implicit QR prior·기존 iteration flag3개 추가. 이 lane의 제품 수치 로직 편집 없음; stopping/noise/GT/caps 고정.
- 빌드 진입: Numerical·Engine source freeze manifest 및 ABI/API peer review 완료 뒤 시작. 그전 CPU build/replay 없음.
- 검증 순서: aggregate≤4 compiler jobs → 모든 transitive native/WASM 재컴파일 → discovered native9 suite·신규 numeric case 포함 → strict fullASan/UBSan/LSan9 suite → source/dependency/artifact manifests·실제 module smoke → sequential selected1110 재현·1094 branch/pose/state caps → parent의3회 full room1·room4 gate.
- QR 실제 rows/cells·transient memory·latency는 실제 diagnostics/benchmark 기록 필요. 계산 완료·짧은 prefix만으로 full replay·phone·production 합격 판정 금지.

## QR 재개·helper 관측 계획

- 소유 파일: `scripts/dev/frontend-diagnostic.mjs`; 제품 수학·browser/Worker·server 경로 수정 제외.
- 기존 selected `[1080,1110)`·seed0/CV1/iteration10/time10 프로필 유지; 각 실제 `processFrame` 호출 wall ms와 호출 이후 실제 WASM linear-memory byte length만 추가.
- 출력에 candidate/raw-input SHA-256 기록; 이전 raw1110과 native 새 dump의 exact hash 확인 후 같은 bytes로 WASM 실행.
- Native peak RSS: `/usr/bin/time -v`의 process high-water 관측. WASM linear-memory high-water는 실제 capacity이며 browser total RSS·allocator live bytes와 구분.
- 검증: `node --check scripts/dev/frontend-diagnostic.mjs`, 소스 소유자 동결 이후 실제 candidate selected1110 실행·JSON finite/count/profile 확인. timing은 diagnostics ON 개발 관측이며 최종 diagnostics OFF 성능 gate와 분리.

## Stable QR 최신 실행 — 통합 회귀 통과·strict parity 미통과

- 최종 QR freeze: `square-root-prior/source-frozen.json` SHA-256 `74a931a0b1274152aacb7f87cc214aad273ab7eaf9763fdef64e6546e4331aaf`; 독립 source peer APPROVE.
- Fresh repository 객체: native49·active WASM29·sanitizer49 TU. Ceres/gtest immutable cache 보존, 실제 sanitizer compile flags·dependency commit/Schur ON/CUDA OFF 확인.
- Native 실제 discovered/executed **89/89 GTests·9/9 suites**; full ASan/UBSan/LSan **89/89·9/9·169.07초**, skip0·suppression0. `build/refactor-evaluation/square-root-prior/{native,sanitizer}-test-result.json`.
- 전체85개 core source manifest 세 빌드 동일: `e283cce61b7d063aabff336eb90eae210d2d74556c42175721c85c5ab5dadb63`. 별도 validation source manifest `8a3825a67daee3a77703a637c06b3630dbfe9e081106db07f088f3d49aa34ad2`·새 object hash/mtime 보존.
- JS 후보: `61d0e4b9ec232977c6badec90a7fde7be4835dee179795bac4b83afcf2b8d9fe`, 6,006,374 bytes embedded WASM. 실제 Chrome mandatory API·default diagnostics OFF·arm/warmup/reset JSON smoke 통과. 기존 served artifact 승격 없음.
- Selected1110 `[1080,1110)`: 원시 입력 SHA-256 `357788cb3281a8b57e0bf316aa1704a83984b9930469ee320e72f712b60f9cb7`·291,608,080 bytes exact. 양쪽1065 pose·init45, timestamp/state/reason/termination/relative epoch/coverage 동일.
- Numeric caps 통과: translation p95 `2.855957249e-6 m`/max `0.005340159972 m`, rotation p95 `1.045654895e-5°`/max `0.03055122723°`. **strict parity 실패**: solver iteration 차이253/254/658/1105, tolerance·GT·stopping/cap 변경 없음.
- 기존1094 iteration2 split은 양쪽 `validStep=true`·`DBL_MAX` rejection으로 닫힘. 새로운 최초 selected branch는1094 iteration10 Native `DBL_MAX` rejection 대 WASM finite36407.94 acceptance. Pose delta1093 `2.3413e-6 m`→1094 `0.000204337 m`; 장기 cliff closure 미통과.
- Selected30 frontend tracker·factor ID/depth-valid/residual membership·semantic layout·margin·gauge branch 동일. `inverseDepth/estimatedDepth` 수치는 discrete membership 판정에서 제외, raw trace 보존.
- QR 실제1093 `h=908,m=73,n=75,q=73,cells=492630`;1094 `h=1346,m=123,n=75,q=123,cells=887374`. 최대 workspace estimate887374 double cells; allocator 실제 live bytes·total RSS와 구분.
- 실제 peak 관측: Native process RSS92,316KiB, WASM linear-memory capacity268,435,456bytes. QR diagnostic ON assembly+QR p50/p95/max Native9.194/12.744/13.604ms, WASM7.435/10.8285/11.630ms; pre-evaluation·final spectrum/JSON serialization 제외, capture-on dropped Gram 포함. Diagnostics OFF 최종 latency·phone 성능과 구분.
- 근거: `build/refactor-diagnostic/square-root-prior/selected1110/{comparison,branch-membership-refinement,input-equality,commands}.json`. Gram eigenvalue만으로 새 QR prior의 gauge 정확도/결함 판정 제외.

## First253 조기 종료 read-only 재현

- 동일 후보·소스·프로필로300 prefix, capture `[245,265)`, Native→WASM 순차 실행. 기존 raw의 exact prefix SHA-256 `d9784890435bee900d9a5082c2f323eb834dea8663f19dbaf7155c502aceaf33`,78,812,816bytes.
- 양쪽255 pose, cross translation max `2.386234427e-6 m`; solver iterations253/254만 차이. Native 이전1110 prefix 재실행 translation max `1.743931232e-13 m`/scalar0, WASM pose bit-exact.
- 실제253 종료: Native8회·WASM7회, 양쪽 `CONVERGENCE`; 메시지 `Function tolerance reached` 및 f/g/p `1e-6/1e-10/1e-8`, max10/time10 유지. Gradient/parameter/time 종료로 판정 제외.
- Shared iteration0~6의 WASM−Native cost offset≈7.473775873, step·gradient 차이 작음. Native accepted trial7 Δcost0.002414793/current2412.14967=`1.00109584e-6`; 같은 Δcost/WASM current2419.62345=`9.98003639e-7`은 WASM message와 일치하는 **진단 비교**. 미기록 WASM converged candidate Δcost를 측정값으로 주장 제외.
- Cost offset의 생성 경로는 추가 수치 검토 필요. Workload profile proposal과 실제 적용 구분; 이 lane의 solver 설정/제품 source 변경 없음.
- 근거: `build/refactor-diagnostic/square-root-prior/selected300-245-265/first253-stopping-analysis.json`.
- 최종 상태: builds_ready=true, performance_ready=false; 모든 build/test/replay 종료. 새 후보 full3·room4·7002 승격·phone/production 판정 대기.

## 명시적 benchmark helper 변경 계획

- 승인 범위: `tests/audit_dataset_replay.cpp`, `scripts/dev/frontend-diagnostic.mjs`, `scripts/dev/wasm-smoke.mjs`; 제품 Optimizer/Estimator/Engine/bindings는 수치 담당 소유.
- Native opt-in: 환경변수 `MOBILE_SLAM_BENCHMARK_ZERO_TOLERANCES=0|1`, 미설정=false·기타 값은 실행 전 거부. 기존 positional CLI 유지.
- 짧은 WASM 진단 opt-in: 기존 positional 인자 뒤 `--benchmark-zero-tolerances 0|1`, 미설정=false·unknown/trailing/invalid flag 거부.
- 공통 API: `setBenchmarkSolverProfile(bool)` 호출·`getBenchmarkSolverProfile()` actual readback 확인. 명칭 `max10_zero_positive_tolerances`, max10·time10/seed0/CV1 유지; 실제 iteration count/termination/message 변경·합성 제외.
- Smoke: defaultfalse→enable→reset/reconfigure 유지→disable, actual API·fresh JSON 확인. Native 잘못된 환경값 rejection·actual selected capture의 tolerance0/0/0 확인.
- 소스 소유자 전체 동결 뒤 fresh all-ABI native/WASM/fullSAN·발견된 신규 case 수·해시 갱신, profiletrue short300/1110 순차 재실행. 기존 default61d0 실패 기록·raw/data·caps 보존.

## Opt-in benchmark 최신 인계

- 실제 Native **91/91 GTests·9/9 suites**, Numerical31 포함; 전체 strict ASan/UBSan/LSan **91/91·9/9·169.24초**, skip0·suppression0.
- 승인된 exact Ninja ABI closure: Native11·active WASM4·SAN11 객체 새 컴파일. 변경 없는38/25/38 객체의 source·실제 compile command·object SHA가 이전 full fresh QR 검증과 동일; Ceres immutable cache 보존·stale class-size 객체 혼용 없음.
- 공통85 core source manifest `11ca788ff68b39653984b2d384c672820fcf698cfc0a621d273a49311ae9075c`; 별도 web source `129d86aa51c4af840957b109bdb76231c9e4a7c7351399f5f38ef11f16f5a122`, validation source `188f494627a890fa1a670acea25628d3569d10f1c5efe316917897adf3fb6e61`.
- 현재 JS 후보 `fce25012d61a0d5965e4d5afadda25aecf96c219b30451e0394575c3caa10da1`,6,007,610bytes embedded. Native replay `81e249ed8d242a6dd40838a03a11efa1e035831f7e8984917180a96dae09ffb8`. 실제 module API defaultfalse→enable→reset/reconfigure 유지→disable/resetfalse, 진단 JSON·seed0/CV1 smoke 통과.
- Helper actual rejection: 잘못된 Native 환경값4개·WASM unknown/malformed flag2개를 output/config/browser 실행 전에 거부. 실제 default와 explicit request/readback 구분.
- Short300 `[245,265)`: raw prefix `d978489…` exact·양쪽255 pose·init45·모든 scalar/state/solver count/termination/epoch 동일. Translation p95 `2.71187159e-8m`/max `3.936335399e-8m`, rotation max `4.004553366e-6°`; strict gate 통과.
- Short1110 `[1080,1110)`: raw `357788…` exact·양쪽1065 pose·init45·모든 scalar/state/solver count/termination/epoch 동일. Translation p95 `9.864963448e-9m`/max `0.000332461561m`, rotation p95 `3.818191821e-6°`/max `0.002294986123°`; **기존 독립 numeric cap·strict gate 통과**, cap 변경 없음.
- 양쪽 실제 capture의 f/g/p `0/0/0`, maxIterations10/time10·API readback true 확인. 실제 Summary 분포 warmup45행0·나머지1065행11; iteration0 포함한 실제 결과이며 다른 입력의 exact10/11 보장으로 확대 제외. Independent stationary zero-gradient 회귀의 실제 조기 CONVERGENCE 유지.
- 1093→1094 translation delta `3.88296e-9→3.15243e-9m`; 양쪽1094 iteration2/10 `DBL_MAX` rejection 동일. 이후 internal trial branch6개 차이, 최초1099 iteration6; 최종 state/count/termination 동일·pose cap 내부. 장기 full3/room4 해결 판정은 parent gate.
- Selected30 tracker·discrete visual membership·semantic layout·margin·gauge branch 동일. Default61d0의4 solver-count 실패·cost-offset function-tolerance 원인 기록은 historical로 보존; 이번 controlled-work profile 통과로 default gate 소급 통과 처리 제외.
- QR 실제 최대 workspace estimate887374 double cells,1093 `(h,m,n,q)=(908,73,75,73)`,1094 `(1346,123,75,123)`; Native RSS peak91,840KiB·WASM linear-memory capacity peak268,435,456bytes. QR diagnostic ON assembly+QR p50/p95/max Native8.745/12.639/13.546ms·WASM7.448/10.504/11.590ms, pre-evaluation·final spectrum/JSON 제외/capture-on dropped Gram 포함. Allocator live bytes·browser total RSS·diagnostics OFF/phone 성능과 구분.
- 근거: `build/refactor-evaluation/benchmark-profile/{all-build-fingerprints,native-test-result,sanitizer-test-result,wasm-smoke}.json`, `build/refactor-diagnostic/benchmark-profile/selected{300,1110}/comparison.json`.
- 인계 상태: builds_ready=true·performance_ready=false·CPU job0. Parent의 paired 후보 승격·실제7002 served hash·full3 room1·room4·paced/loss/public mock gate 대기. 이 lane의 deploy/server/원본 dataset 변경 없음.
