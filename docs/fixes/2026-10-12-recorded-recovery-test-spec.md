# 기록 기반 회복 테스트 명세

- 날짜: 2026-10-12, Asia/Seoul
- 상태: Sol Ultra 작성, [개발 계획](2026-10-12-recorded-recovery-plan.md)의 Astra Ultra 사전 검토 대상
- 기준 A0: `build/research-20261012/baseline/manifest.json`의 미커밋 현재 source. 검증 결과·추가 held-out seed·실험 설정은 첫 후보 평가 전에 JSON으로 동결
- 결과 root: `build/recovery-20261012/`. 각 trial별 source patch/hash·빌드 manifest·exact command·exit status·raw rows·평가 JSON 보존
- tests 먼저: 기존 baseline 결과 확보 → 새로운 계약을 현재 code에서 red로 확인 → 제품 수정 → 같은 독립 기대값 green. 구현 내부 계산을 복사한 oracle 금지

## T0 구조·깊이·anchor 단위 검증

| ID | 독립 fixture·oracle | 통과 조건 |
|---|---|---|
| M1 | anchor0/7/8/9/10, 관측1/2/3, history-only/current, valid/unknown/failed, ray.z≠1, NaN/Inf 혼합 | 동일 구조적 feature ID 순서가 count/depthvector/setDepth/clearDepth/setup/final filter/OLD marginalization에서 유지; status 변경으로 reindex0 |
| M2 | cap999/1000/1001 및 invalid/deferred가 prefix 사이에 배치된 bank | 동일 1000 slot prefix·no OOB·나머지 feature depth 오염0 |
| M3 | 하나의 unknown(-1,0) + 현재 valid track≥2를 실제 optimize/apply; admitted track 최종 실패도 별도로 삽입 | untouched unknown 계속0/no map/no factor; 최종 실패 landmark는2. 한 실패가 다음 landmark ID/depth에 이동0 |
| M4 | 유한 양수depth·geometry_pending인 inactive track + admitted current≥2; solve중 final 실패 삽입 | 동일 slot table·고정 admission mask가 factor/final/current/apply/OLD prior에 적용, positive rho만으로 inactive 재진입0·map/PnP 승격0 |
| G1 | known point P_W=(0.4,-0.2,4), nonzero t_BC=(0.03,-0.02,0.04), 비항등 R_BC·각 body pose에서 직접 생성한 rays | anchor8/9에서 DLT depth·world error≤1e-9m; actual current residual ID·count 일치 |
| G2 | 동일 camera center의 순수 회전, 동일 rays, 거의 평행한 두 ray, zero/negative/nonfinite depth | unknown/deferred, current factors0/map0; init_depth 대입·rho0 metric publication0 |
| G3 | 두 ray 사이 각θ; `A`의 독립 고유값 `{2,1+cosθ,1-cosθ}` 사용 | κ=`2/(1-cosθ)` 경계 양쪽 수용/거부; 정확한 rank-deficient θ0 거부, NaN/Inf 거부 |
| G4 | fx≠fy·비정방 pixel·ray.z≠1; 고정 known candidate point의 projection에 1.4/1.6pixel residual, 새 view 하나 behind camera | 동결1.5pixel gate 양쪽·모든 view positive depth 적용. anchor age로 geometry 거부 우회0 |
| G5 | 순수회전/ill-conditioned deferred9→8→7을 실제 OLD로 이동, 이후 횡이동 생성; separate 기존solved mature track | deferred 표식·기하 거부 유지, age 우회0; 충분한 translation 때 true admission+usable+final 통과 후만 표식 해제; 기존 solved track 정책 불변 |
| G6 | 동일 rays에 양수 scale0.2/1/7을 곱해 공통 ingestion 후 전체 경로 실행; malformed도 별도 주입 | canonical z1·feature ID/slot·DLT optical depth·ProjectionFactor residual/Jacobian·final filter·map/PnP·OLD/NEW reanchor 모두 동일, 수치허용1e-9; pixel observation/normalized velocity 불변 |
| A1 | old slot9→new10, body translation+rotation+nonzero extrinsic. P_W 직접 재계산 | old/new pose 덮어쓰기 전 reanchor, exact rays world error≤1e-9m; opticalZ correct |
| A2 | unknown/negative/NaN/Inf olddepth, new camera behindpoint, 1관측/빈 track, OLD/NEW 혼합20회 | 잘못된 depth solved1 유지0; unknown은 다음 유효 baseline에서 재tri 가능; frame index/ray 연결/OOB 오류0 |
| P1 | OLD prior 생성→NEW→OLD, rejected update도 포함; feature ID 순서를 독립 추적 | NEW pose9 제거·10→9, inverse-depth 잔존0, stale pointer0; guard rejection 후 prior release 계약 유지 |
| C1 | history-only factor 충분, actual current0/1/2, 최종 negative/cheirality failure | current0/1은 기존 Lost 유지; current2라도 final filter가1이면 거부. finite state만으로 성공0 |
| I1 | initializer clearDepth→triangulate→scale s=2, optical point known; old/young 혼합 | 모든 selected depth 정확히2배, deferred no fake metric depth; map/PnP solved와 optimizer ID 일치 |

- 기존 tests 확장 우선: `tests/test_projection_trial_contracts.cpp`, `tests/test_numerical_contracts.cpp`; 기존 friend access 재사용. 필요한 최소 target만 build하고 native suite 전체에서 기존 assertion 약화·삭제0
- geometry 기대값은 직접 정의한 P_W·camera transform·투영식으로 계산. DLT/SVD 구현을 테스트 oracle로 다시 호출하지 않음
- 순수 회전의 known solved landmark 추적은 허용하되, unknown landmark의 metric depth 생성과 구분

## T1 관측 가능한 starvation 재현

- 직접 수학 fixture: 성숙 current ID1개 유지, young ID24개 재관측, fx=fy=focal_length400, z≈4m. **고정 slot8 camera x=0**, 다음 camera는 lateral x=0↔0.09m로20회 왕복; young 위치/FOV도 사전 고정. selector가 비교하는 slot8↔9 parallax는≤9pixel, 인접 young pair angle≈0.0225rad·κ≈7,900
- rays는 known P_W와 camera poses에서 생성; 매회 실제 `addFeatureAndCheckParallax`가 NEW를 반환하는지20/20 확인 후 slide. monotonic0.09m 누적 이동은 frozen slot8 대비 baseline이 커져 NEW 보장 못하므로 제외; 강제 margin 덮어쓰기0
- A0 증명: current factor1 유지, young bank 있지만 age로 제외. A2 증명: 최소2개 실제 현재 factor·기하 검증·정확한 anchor 변환, 다른 guard 변화0
- 부정 fixture: 동일24개·회전만·camera center 고정이면 young factor0 유지. 이미 solved 성숙1개만으로 Lost 해제 금지
- 이 fixture의 green만으로 회복 완료 판정 금지. 아래 rendered pixels/IMU/GT fixture에서도 효과가 재현되어야 채택

## T2 독립 image·IMU·withheld GT recovery fixture

- 기존 renderer/analytic IMU 생성기를 개발 fixture로 확장; estimator에 GT·feature ID/3D·정답 rotation 전달 금지. root product 수정과 별도 파일 소유
- 사전 frozen motion: 정상 초기화0–5초 → old scene/ID 교체와 빠른 복합 회전5–8초 → 제한된 translation+low parallax8–14초 → 정지14–20초. 현재 scene을 renderer가 독립 설정하고 image bytes·IMU·GT hash 보존
- 정확한 pose spline·texture seed·occlusion 시간·baseline·frame/IMU hz·noise는 A0 첫 실행 전에 spec JSON에 기입. A0 결과를 본 뒤 silent fixture retune 금지; 재설계 시 별도 실패 fixture로 보존하고 새 버전/검토
- 목적 구간의 A0 actual current factor≤1·새 track starvation을 먼저 확인. 원인 재현이 안 되면 `NOT_REPRODUCED`; 단위 fixture 성공을 영상 재현으로 대체 금지
- 400 sourceframe/20Hz·20초, analytic body IMU60Hz, warmup `[5,20)`300 sourceframe, body SE(3) scale1·timestamp1:1. engine warmup/초기화 실패를 분모에서 제거 금지
- 동결 acceptance: 기존 mobile GT cap ATE≤0.25m, rotation APE RMSE≤5°, warmup coverage≥80%; candidate reset0. 목표구간 actual current≤1 frame 및 Lost 구간 수가 A0 대비 감소하고 fresh coverage 최소+5percentage points
- baseline source·candidate source의 동일 raw input을 native/Worker로 처리. 새 fixture부터 paired diagnostics를 남기고 image→ID survival→triangulation reason→parameter slot→current factor lineage 연결
- seed: 설계101/707, 공개 검증2203/8809. replay lane의 churn12617은 계획 동결 전 생성된 **개발 fixture**이며 held-out 아님. 최종 신규 확인 seed는 root가 알고리즘·임계값·평가spec 동결 후 생성; 나쁜 seed 폐기 금지
- 기존 개발12617의 고정 원본359/400·warmup300/300 GT통과와 60% occlusion 파생324/400·warmup265/300·factor실패35(OLD)·Lost7구간·ATE49.835m/APE123.354° GT실패를 모두 보존. 파생48frame/[6,6.8),[10,10.8),[14,14.8)초·동일IMU/GT/caps 변경0; 후보는 파생 GT gate까지 통과해야 하며 coverage만 개선하면 실패
- 순수 회전·stationary·constant velocity degenerate fixture의 기존 expected-degenerate 계약도 유지; 안정적인 fake depth로 passing coverage 만들지 않음

## T3 기존 phone 기록

- 입력: `/home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-07-ZFlip6/`의 두 각300PGM accepted prefix. 정확한 record·input-dir·timestamp 범위는 연구 replay manifest에서 인계
- 이미지 P5 payload SHA·record SHA·contiguous accepted index·IMU past/future bracket·calibration/effective profile을 확인한 후 A0/A2 동일 입력 실행
- profile mobile0.06s/10iter/120feature, LK21/3/min30, PnP OFF/freq3, rotation rescue 선택을 명시·실제 C++ readback. 기존 generic offline benchmark profile과 구분
- baseline native와 baseline Worker를 먼저 각각 보존. old legacy recorder의 누락 parameter·pre-record estimator state·saved-prefix·실제 도착시각 미재현 한계를 report에 유지
- 지표: all frames/initialized/fresh·first pose·current factor0/1·Lost 연속 구간/최대 fresh gap·reset·bias warning·translation/rotation jump·최대norm. 각 reason/분모를 분리
- A0에서 재현된 visual-loss 구간이 있으면 candidate에서 최소1구간 fresh 회복과 current-factor 원인 개선, 다른 prefix의 fresh coverage 감소≤1pp·reset 증가0·max jump/bias warning 악화0
- 해당 prefix에 baseline failure가 없으면 개선 증거 없음으로 기록하고 무회귀만 확인. 최신30초 Lost33 또는 약41.12초 발산 재생 완료로 주장 금지
- 최신 numeric snapshot은 source 코드 lineage와 reason 요약에만 사용; gray를 추정하거나 synthetic image를 붙여 원본 replay로 만들지 않음
- frozen replay 진단은 OLD current-loss를 보였으므로 NEW unit starvation을 phone 원인으로 대체 금지. 기록의 OLD/current/final 부족 각각을 보존하고 A2 effect를 이유별 비교
- matched native LK20/.03은 browser와270/300 fresh 일치. frame82 OLD: detected120/eligible96인데 eligible 전부historical/current0; frame98 NEW: 초기 current67→final current 부족. 두 유형 모두 회귀 필수, LK30/.01 auxiliary261/300은 parity 근거 제외

## T4 TUM·기존 fast-motion GT

- 전체 TUM room1 2,821frame / room4 2,228frame; frozen 실제 dataset/config/profile·timestamps·calibration, OpenCV thread1/seed0. body GT·일대일 association·단일 SE(3) scale1, Sim(3)는 별도 scale 진단
- OFF control과 현 App ON을 각 matched A0에 비교; A0 ON의 기존 최적화 통과를 HEAD OFF와 혼동 금지
- pass 각 room/profile: translation ATE RMSE≤A0×1.05, rotation RMSE≤A0×1.05, fresh pose coverage≥A0−0.01. association0·rotation 누락·invalid metric는 fail
- baseline metric이0이면 해당 절대 oracle tolerance를 사전 기록하고 비율로 나누지 않음; 후보 결과를 보고 임의 tolerance 추가 금지
- fast101/707/2203/8809 기존 frozen inputs·각400frame, 기존 acceptance 그대로. A0/A2 전체·warmup coverage·ATE/rotationAPE/RPE·reset·quality warning 기록
- native fixed work10s/10iter와 live mobile0.06s를 별도 실행·평가. actual App DeviceMotion seed101·Generic 신규확인 seed에서 pixel/time/IMU provenance·live profile readback 확인
- 이전8809는 신규 held-out 아님. 신규 seed 결과가 실패하면 알고리즘 수정 후 또다른 신규 확인을 생성하고 실패 seed 보존

## T5 성능·자원·도구

- 사전 frozen native input: 기존 clean101 400frame와 fast8809 400frame. timed run은 고비용 numeric/membership JSON·hash I/O OFF; 동일 adapter stage getter만 timed scope 밖에서 수집. 별도 분석 run은 상세 진단 ON
- A0 원래 native는 process_ms만 존재. Sol performance lane가 `tests/audit_dataset_replay.cpp`의 동일 측정 adapter를 frozenA0 source 격리 build/A2 build에 적용; A0 core source/dependency SHA 일치·동일adapter SHA·flags 기록. `getPipelineDiagnostics()` getter는 process timed scope 밖에서 buffer하고 numeric diagnostics OFF
- frozenA0 binary와 instrumentedA0 fixed-work pilot의 exact input/status/fresh timestamp·기존 동결 pose tolerance parity 먼저 확인. mobile time-cap 실행은 실제 iteration·termination도 기록; adapter 자체 수치 영향이 확인되면 시간 비교 중단
- 대안은 양 WASM loader에 동일 actual getter harness. 경로/mode/serialization workload를 baseline/candidate 동일하게 고정; 미실행 stage null·sample count 유지. baseline에 없는 stage를 process_ms에서 분해하거나 다른 capture에서 빌려오지 않음
- 순차 ABBA 두쌍, 동일 host/build flags/thread·solver budget·입력, 실제 monotonic clock. build·다른 SLAM replay 동시 실행0
- 각 반복 init 전/후 tracking/triangulation/backend/Ceres/marginalization/engine p50/p95/p99, fresh coverage·reset·RSS. stage p95 합을 total p95로 계산 금지
- paired median engine/frontend/backend p95≤A0×1.05, p99≤A0×1.10. full TUM/heldout 기준을 느슨하게 하여 speed 얻는 후보 제외
- 실제 App 자연 rAF 3회는 scheduling/capture/queue/UI proxy 측정으로만 보고. virtual clock 비용·desktop sensor mock을 phone latency로 보고 금지
- baseline CTest 실제 discover14suite·168assertion 관측; 새 source 전 suite를 enumerate하고 각 outcome·assertion·skip/TODO·실패 log 기록
- native full regression, 영향 suite ASan/UBSan/LSan, syntax·browser contracts TODO/skip0, changed C++ clangd error0, fresh WASM binding·Worker 실행 필수. 미설치 ESLint/TypeScript/clang-tidy를 통과로 보고 금지

## T6 독립 capture Worker C1

- 실제 target Chrome에서 VideoFrame construct→transfer→OffscreenCanvas2D draw→getImageData 전체 실행; API 존재만으로 지원 판정 금지. 실패 경로의 기존 sync fallback까지 검증
- numbered/color/portrait·landscape/90·180·270도 rotation/noncenter crop fixture의 전체 grayscale byte가 기존 synchronous oracle와 동일; source callback timestamp·frameSeq·geometry 완전 일치
- Worker 지연·busy·overwrite·draw error·transfer 실패·stop/restart·reconfigure·hidden·구세대 응답 테스트: live VideoFrame≤2, 종료후0, close 누락0·구세대 제출0·같은 seq 중복 제출0
- 전송 후 worker error는 해당 frame drop으로 기록; 나중 영상으로 과거 source timestamp를 재사용하는 fallback0. 실패 다음 신규 frame부터 sync fallback 가능
- callback await 전에 frame constructor가 실행되는 실제 browser race fixture; color/frame number가 바뀐 다음 draw에서도 해당 source pixels 유지. metadata relabel을 실제 exposure 증명으로 사용 금지
- 기존 A2 같은 accepted-input captured replay로 native/Worker/GT 비교. live capture의 scheduled/accepted/coalesced/drop 분모도 별도 확인; 처리된 좋은 frame만 남겨 timing 계산 금지
- synchronous/capture Worker 같은 scene·host 직렬 ABBA 두쌍: main callback p95≥20% 감소, source→submit/engine/fresh-gap p95 악화≤5%, coverage 감소≤1pp·reset 증가0·GT gate 통과. Worker clock는 duration만 사용
- 한 gate라도 실패하면 sync 기본 유지. actual target probe가 불가하면 C1은 UNVERIFIED 실험으로 남기고 기본 승격하지 않음

## 재현 명령 계약

- 아래 경로 변수는 실행 전 존재·SHA 확인 후 `commands.json`에 **확장된 실제 값**으로 저장. 비어 있는 변수·기존 output 덮어쓰기 금지
- configure: `cmake -S . -B build/recovery-native-20261012 -G Ninja -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DMOBILE_SLAM_BUILD_VIEWER=OFF`
- build: `cmake --build build/recovery-native-20261012 --parallel 2`
- discover/test: `ctest --test-dir build/recovery-native-20261012 -N`; `ctest --test-dir build/recovery-native-20261012 --output-on-failure --parallel 2`
- native replay interface: `MOBILE_SLAM_NATIVE_BUILD=build/recovery-native-20261012 MOBILE_SLAM_SKIP_BUILD=1 bash scripts/dev/native-replay.sh DATASET CONFIG NEW_OUTPUT FRAME_COUNT bracket PNP CV_THREADS ITERATIONS SOLVER_TIME DUMP_INPUTS DIAGNOSTICS CAPTURE_START CAPTURE_END PROFILE ROTATION_RESCUE`. `tests/audit_dataset_replay.cpp` 현재 usage와 대조, benchmark는 기존 환경 선택·readback을 별도 기록
- preserved Worker: `node scripts/dev/replay-android-input.mjs --record "$recovery_record" --input-dir "$recovery_images" --loader "$recovery_loader" --output "$recovery_output" --frames 300`; 필요한 rotation profile explicit 지원·readback은 먼저 tool 검증
- 기존 fast fixture: `node scripts/dev/synthetic-browser.mjs --input "$recovery_fixture/input.json" --loader "$recovery_loader" --output "$recovery_output" --mode raw --frames 400 --rotation-rescue on`
- 실제 App: 같은 harness의 `--mode live-paced-native-raf --sensor devicemotion|generic --frames 400 --rotation-rescue on`. sensor별 새 output; captured-input raw 재생으로 같은 accepted pixels/IMU/config 비교
- GT: `python3 scripts/dev/evaluate-synthetic-vio.py --dataset "$recovery_fixture" --results "$recovery_output/report.json" --estimate-frame camera --output "$recovery_evaluation"`
- source/artifact: `python3 scripts/dev/build-provenance.py --build build/recovery-native-20261012 --artifact build/recovery-native-20261012/native_replay --output build/recovery-20261012/native-provenance.json`; WASM도 fresh build/artifact로 별도 생성
- quick: `bash scripts/dev/check.sh syntax`; `bash scripts/dev/check.sh browser-contracts`; `MOBILE_SLAM_COMPILE_DB=build/recovery-native-20261012 bash scripts/dev/check.sh clangd`
- isolated WASM: `node scripts/dev/wasm-smoke.mjs build/recovery-wasm-20261012/vio_engine.js`. sanitizer/WASM build flags·dependency instrument hash는 기존 검증 runbook에 따라 별도 exact command 기록

## 승격·완료 증거

- `verification.json`: source SHA·candidate matrix·test counts·failing preserved trials·각 gate 결과·review disposition·limits. 제품 코드 후속 변경이면 해당 영향 gate 재검증
- native/WASM core source SHA 동일, isolated loader mandatory bindings/config/reset/epoch·actual current diagnostics 실행 확인
- `/vio_engine.js` embedded WASM SHA = manifest artifact SHA = actual strict TLS GET SHA. App/Wrapper/Worker JS 해시도 manifest/검증 기록과 일치
- 기존 원자적 `deploy-candidate.py` 사용·이전loader/manifest backup·승격 승인 manifest는 이미 받은 task 권한 범위에서 root가 작성. 실제 제공 실패 시 보존 baseline 쌍으로 복원, 서버 무관 restart 금지
- 최종 문서: 변경 파일·중복 predicate 단순화·ablation/기각·원본 replay 지원 범위·독립GT 지표·성능·remaining risk. 물리 Flip6·열·실측 calibration·fieldGT 미검증 명시
