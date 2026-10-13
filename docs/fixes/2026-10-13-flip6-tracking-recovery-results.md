# Flip6 fresh 회귀 분리와 깊이 초기값 실험

- 시작: `d8090cc`, `codex/flip6-analysis-20261013`; [사전 계약](2026-10-13-flip6-tracking-recovery-plan.md).
- 결론: **239→53 원인 분리 완료. C3 추적 로직 구현·A/B 완료, 기본 채택 FAIL.** 보정 입력 개선은 확인했으나 원본 기록의 reset·불연속 비악화 기준을 통과하지 못함.
- 반영: C3는 `MOBILE_SLAM_EXPERIMENTAL_ANCHOR_DEPTH=ON`인 별도 native/WASM 빌드에만 포함. 기본 `OFF`, 기존 active loader·서비스 보존. 일반 빌드의 fresh 개선으로 보고하지 않음.
- 원본·실패 후보·중간 결과: `build/flip6-recovery-20261013/`. 데이터는 읽기 전용; 원본300frame과 gyro 보정300frame을 별도 집계.

## 239/300 → 53/300 원인

| 통제·관측 | 결과 |
|---|---|
| 원본 frame·calibration·설정·IMU batch·Worker·Wrapper·harness | 동일, loader만 변경 |
| A0→A1 구조·ray·NEW anchor 수정 | native 최초40frame 상태 동일; pose 수치 차이 최대 약 `8.2e-12` |
| A1→A2 persistent `geometry_pending`·기하 admission | 최초 초기화 frame28에서 current visual factor 감소 |
| 재빌드 A1 WASM | historical active **239/300**, 전체300행 pose·상태·reason 일치 |
| 재빌드 A2 WASM | current baseline **53/300**, 전체300행 pose·상태·reason 일치 |
| frame28 pre-solve / final current | A1 **74 / 68**, A2 **8 / 2** |
| 최초 fresh 실패 frame37 | tracker41 동일, current factor39→1 |

- frame28 eligible181개, 거부158개: rank/condition121, 재투영/cheirality36, invalid depth1. current tracked99개에서는 rank82·재투영9·admitted8.
- 초기화 실패 중 young anchor에서 생성된 pending 상태가 anchor 이동 이후에도 유지됨. A1의 age만으로 허용하던 depth가 A2에서는 실제 geometry 검사를 계속 받음.
- 최종8→2의 탈락6개는 모두 **재투영1.887–4.340px >1.5px**. rho2.626–3.194로 양수; `rho≥0` bound는 이 프레임의 직접 수리가 아님.
- 이후 reset과 tracker 차이는 후행 결과. IMU 전달·timing metadata 변경을 이 fresh 감소의 원인으로 판정하지 않음.
- historical239에는 acc-bias 경고128개·최대 공개 pose step3.786m 포함. age bypass 복원이나 fresh 재분류를 정확도 개선으로 채택하지 않음.
- 근거: [source·membership 분해](../../build/flip6-recovery-20261013/isolation-analysis/README.md), [WASM identity](../../build/flip6-recovery-20261013/browser-ablation-identity.json), [최종 cull](../../build/flip6-recovery-20261013/b1/frame28-classification.json).

## 상용·공개 엔진 분석의 적용

- [REA·WABT·readelf/objdump 실행](../research/2026-10-13-commercial-engine-reinspection.md), [ARKit·ARCore·8th Wall](../research/2026-10-13-commercial-tracking.md), [대체 엔진 조사](../research/2026-10-13-slam-engine-landscape.md)의 고정 원문·SHA를 사용. proprietary solver의 수식·threshold를 복원했다는 주장은 하지 않음.

| 조사에서 확인한 차이 | 실제 적용·검증 결과 |
|---|---|
| ARKit tracking quality/relocalization, ARCore frame·pose 계약 | fresh 개수와 실제 geometry 품질을 분리. Lost를 정상 pose로 재분류하거나 current support≥2를 완화하지 않음 |
| 8th Wall frame staging·sensor/clock 경계 | 이전 `d8090cc`의 request별 timing metadata 연결을 유지. 이번 인과 대조에서 해당 경계가 원인이 아님을 확인 |
| XRSLAM/RD-VIO의 회전·병진 관측가능성 분리 | rotation-compensated keyframe K1/K2를 실제 실행했으나 GT/phone 회귀로 폐기. pure rotation을 depth 정보로 세지 않음 |
| Basalt의 baseline·geometry 확인, 공개 VIO의 anchored landmark 표현 | 현재 DLT 3D 점과 소비자가 쓰는 anchor-ray×depth의 차이를 분석. 같은1.5px 제약을 만족하는 anchor depth 후보 C3를 독립 구현 |
| OpenVINS의 정보·선형화 일관성 | prior/IMU 보존 P1/P2를 Schur·gauge/lineage 회귀와 실제 궤적으로 검증; 일부 합성 개선과 phone 악화가 공존하여 폐기 |
| AlvaAR/ORB-SLAM3의 map projection·재대응 | 저장 prefix의 실제 visibility를 분석. 새 correspondence 인증 없이 factor 수를 늘리는 변경은 적용하지 않음 |

- C3는 위 조사에서 얻은 geometry·상태 수명 구분을 현재 실패 지점에 적용한 **자체 알고리즘 실험**. ARKit/ARCore/8th Wall이 같은 interval solver를 쓴다는 근거는 없음.
- sqrtVINS·OpenVINS·Basalt·ORB-SLAM3·Spectacular AI 등은 비교 후보. 동일 Flip6 입력·calibration·GT로 더 우수함을 실측한 엔진은 아직 없음. 이번에 dependency·상용 SDK를 추가하지 않음.

## C3: 소비자가 사용하는 anchor depth에서 직접 후보 생성

- 대상: pending, 기존 full condition≤10000, finite positive DLT depth, 기존 all-view reprojection 실패일 때만 fallback.
- 실제 모델: `X_j(d)=a_j*d+b_j`, `d>1e-10`, 각 view의 `Z_j>1e-10`.
- pixel 오차 제약: `||s_j*d+o_j||² − 1.5²*(a_j.z*d+b_j.z)² ≤ 0`. 각 quadratic/linear 허용 구간과 positive-Z halfspace의 교집합 계산.
- 유한·bounded 구간의 strict midpoint만 제안. **기존 `hasConsistentReprojection`으로 다시 검사**하므로 root 계산은 승인 권한을 갖지 않음. tolerance·condition·current support·최종 optimizer cull 변경0.
- 기존 DLT PASS·solved legacy depth는 그대로. fallback 성공도 pending=true/solve_flag0 유지; solver 최종 검증 전 성숙 landmark로 승격하지 않음.
- 독립 fixture: true depth4m, centers0/.4/.8m, pixel noise−1.4/+1. DLT4.04273377m의 최대오차1.88624px는 실패. 허용구간 `[368/92.5,184/45.9]`의 중점3.99354649m는1.32566px로 통과.
- frame28 오프라인 full-condition PASS·positive-DLT·재투영 FAIL36/current9 중 추가가능2/current1. 고정 pose 후보 가능성을 Ceres 최종 성공과 동일시하지 않음.
- 수학/구현 독립 리뷰: [C3 review](../../build/flip6-recovery-20261013/review/c3-implementation-review.md). singleton·unbounded·극단수치는 보수적으로 거부; minimax 최적화·depth uncertainty 추정·완전한 feasibility solver가 아님.
- 단순화: 새 runtime API·UI·dependency 없이 기존 triangulation 분기에 한정. prior/keyframe/gyro patch 후보는 최종 diff에서 제거하고 실패 patch를 보존.

## 동일 입력의 추적 안정성

| 실제 Worker/WASM, 각300frame | historical active | current baseline | C3 ON |
|---|---:|---:|---:|
| 원본 gyro / rotation rescue OFF: fresh |239|53|93|
| 원본: reset |1|2|3|
| 원본: 최대 공개 pose step(m) |3.786|0.546416|3.622329|
| 보정 gyro / rotation rescue ON: fresh |별도 과거270|179|250|
| 보정: reset |—|0|0|
| 보정: 최대 공개 pose step(m) |—|0.825812|0.332142|

- **원본 raw 안정성 gate FAIL.** 보정 입력은 sensitivity control이며 원본 실패를 대체하지 못함. 공개 pose step은 GT 오차도 속도도 아니며, Lost 간격·guard로 검열된 출력의 영향을 받음.
- C3 raw guard가 막은 candidate jump21.7077/7.5192/56.7182m. baseline에도344.8435/52.3744m candidate가 있음. 숨긴 큰 step 감소만으로 reset3회·공개 불연속 증가를 통과 처리하지 않음.
- raw Lost87→47, 최장 연속61→28frame; 보정 Lost93→22, 최장73→13frame. 공개 acc-bias 경고는 raw0→28, 보정0→2로 늘어남. Lost 감소만으로 안정성 전체 통과를 주장하지 않음.
- 전체 engine 처리시간 p50/p95/p99(ms): raw `6.805/35.720/44.900→8.465/40.070/46.915`, 보정 `8.015/22.065/26.020→9.740/24.110/29.720`. 단일 desktop run·부하/thermal/초기화 집단 미통제; 속도 개선 근거 없음. 초기화 전/후·guard 계산 분포는 비교 JSON에 보존.
- 정확한 분모·연속 Lost·경고·epoch·guard 필드: [phone comparison](../../build/flip6-recovery-20261013/final-evidence/phone-comparison.json).
- 기록 경로: `/home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-07-ZFlip6/records/controlled-minute-1/record.json`; 보정본 `build/research-20261012/replay-diagnosis/input-records/minute1-current-on.json`. 원본 PGM은 동일.
- minute2 저장prefix300frame은 초기화 전18.7335초. zero pose는 UNKNOWN; 정확도 통과로 집계하지 않음. 10/12 numeric 기록만으로 새 image/IMU replay를 만들지 않음.

## GT 정확도와 coverage

- body-frame 변환·일대일 timestamp association·SE(3) scale1. synthetic400frame, warmup 분모300; camera GT를 estimator에 제공하지 않음.
- 기존 기준: translation ATE≤0.25m, rotation APE≤5°, 1s RPE≤0.10m/3°, 기존 coverage 기준. strict recovery warmup280/300과 evaluator의 기본 GT_PASS는 다른 gate.
- 아래는 native current baseline→C3 ON; 전체 상세·RPE·입력 SHA는 [GT comparison](../../build/flip6-recovery-20261013/final-evidence/gt-comparison.json).

| Synthetic | ATE RMSE(m) | rotation RMSE(°) | C3 warmup fresh/300 | GT 판정 baseline→C3 |
|---|---:|---:|---:|---|
| fast101 |.043353→.037020|1.547→1.095|292|PASS→PASS|
| fast707 |.206458→.195108|7.055→6.485|293|FAIL→FAIL|
| fast2203 |.432814→.144810|11.369→4.354|287|FAIL→PASS|
| fast8809 |.116635→.095375|1.630→3.646|283|PASS→PASS|
| fast12617 |.095282→.087669|2.413→1.657|283|PASS→PASS|
| churn12617 |.317694→.168428|13.265→6.411|256|FAIL→FAIL|
| 새 confirmation17131 |.118188→.032250|4.136→1.224|285|PASS→PASS|

- 새 확인 seed17131은 C3 구현 전 고정: spec SHA `940211e2cdf4f4642fff94c2e84dd7c241e4243b74c16e5f8fccf7694e2f2cc6`. G1은 같은 seed에서 실패; 실패를 삭제하지 않음.
- 새 categorical GT 실패0은 모든 수치 비악화가 아님. fast8809 rotation1.630→3.646°, fast12617 전체 coverage87.25→85.5% 악화. fast707/churn rotation5° gate 실패 지속; churn256/300은 strict280에도 미달.
- TUM room1 full OFF/ON: ATE `.0500596→.0492401m`, rotation `1.230998→1.231742°`; room4 full OFF/ON: `.0469034→.0465855m`, `1.098636→1.040842°`. current baseline 대비≤1.05·coverage 손실≤1%p gate는 통과. 별도 historical A0 기준 rotation gate 실패는 계속 보존.
- 실제 WASM/Worker clean101:339fresh/400, warmup300/300, ATE.014161m·rotation.457878°·RPE.011314m/.245642°, PASS. confirmation17131:342fresh/400, ATE.032430m·rotation1.230730°·RPE.027304m/1.369982°, PASS.
- native fixed-work와 browser replay는 각각의 effective profile·artifact SHA를 사용. 동일 source라는 이유로 native 수치를 browser·실기기 수치로 대체하지 않음.

## 폐기한 후보

| 후보 | 폐기 근거 |
|---|---|
| P1 prior+outgoing IMU 보존 | churn 개선에도 corrected fresh179→77/reset1, 숨긴33.69m candidate |
| P2 prior-only, P1K1/P2K1 | 일부 개선과 phone/GT 회귀 혼재, raw 회복 실패 |
| K1 회전보상 keyframe, K2 conservative min | GT·phone 안정성 회귀; K2 corrected116/reset1 |
| G1 full condition OR best-bearing-pair | 독립 기하·unit은 통과하나 corrected17/reset1, confirmation17131 ATE.453m/rotation8.335° 실패 |
| C2 anchored scalar algebraic fit | frame28 재투영 탈락36/current9에서 추가허용0 |

- C1 point-BA warm start, projection-guided mature rescue는 설계/관측만; 구현 완료로 계산하지 않음.
- 후보별 patch·source·실행 receipt·실패 evaluator는 local evidence에 유지. 제품 변경은 C3와 opt-in fence에 한정.

## 재현·검증 범위

- [최종 자동 대조 receipt](../../build/flip6-recovery-20261013/final-builds/verification.json): OFF/ON 실제 compile definition·source86개·artifact·회귀 log·baseline 동일성 검증. 구버전 C3 무조건활성 snapshot과 최종 opt-in build를 구분.
- 최종 native: ON217/217, OFF210/210, 각각16suite. 실제 WASM public API smoke ON/OFF 통과. Node285/285(TODO·skip0), Python63/63(내부 exporter3개 포함), JS/Python syntax 통과. ESLint/TypeScript 검사로 보고하지 않음.
- ASan/UBSan/LSan: ON165/165, OFF158/158, 각각8suite, skip·suppress0. 새 project/test object와 SHA 검증한 instrumented Ceres157members 사용; system OpenCV·YAML 전체 계측을 주장하지 않음. gate 추가 전 중단한 sanitizer build의 exit2 로그도 보존하며 위 최종 두 실행과 구분.
- clangd: 최종 ON/OFF compile definition으로 `feature_manager.cpp` 검사, 각각 error0. 새 runtime API·typecheck 체계 추가0.
- source86개 공통 manifest SHA: `a2fa4dd95030c70fcc221534bb2b77aa8db585f1793ab9a294c41392885c9e70`. 동일 source라도 실제 feature_manager compile definition0/1과 산출물은 구분.

| 불변 산출물 | SHA256 | 비교 |
|---|---|---|
| `final-builds/ON/native_replay` | `5cf9100d67c61bf639f0944ffdec1fed3e4a9b815c053b78d99092ad80ddf2d5` | GT에 사용한 C3와 byte 동일 |
| `final-builds/ON/vio_engine.js` | `9f5f737607a14f6aeae6daba9c2be8933a17e6853b98da684ba0834ee36b6ca2` | phone·합성 Worker에 사용한 C3와 byte 동일 |
| `final-builds/OFF/native_replay` | `809a7ca310707877791e9252ac198c441ad8437e9bfad5596e3827c44a27a405` | 시작 d8090cc baseline과 byte 동일 |
| `final-builds/OFF/vio_engine.js` | `38837d4b79abb088610484f4e2c016aac4336fe9761dcfc56874d48e64785589` | 시작 d8090cc baseline과 byte 동일 |

- 재생 당시 `c3/wasm/`은 이후 ON/OFF 빌드에 재사용한 cache. C3 ON의 불변 loader는 `final-builds/ON/vio_engine.js`; 기존 보고서의 당시 loader SHA와 같은 bytes를 보존. 현재 cache 경로의 파일을 과거 C3 결과로 사용하지 않음.
- 기본 빌드: `cmake -S . -B <fresh-native>`; opt-in: `-DMOBILE_SLAM_EXPERIMENTAL_ANCHOR_DEPTH=ON`. WASM도 `emcmake cmake -S wasm -B <fresh-wasm>`에 같은 옵션 사용.
- 원본재생: `node scripts/dev/replay-android-input.mjs --record <record.json> --input-dir <PGM-dir> --loader <isolated/vio_engine.js> --frames 300 --output <new-output>`.
- 고정GT: `OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 python3 build/flip6-recovery-20261013/evaluation/run-case.py --case <case> --binary <pinned-native_replay> --output <new-output-under-evaluation>`.
- WASM GT: `node scripts/dev/synthetic-browser.mjs --input <input.json> --loader <isolated-loader> --output <new-output> --mode raw --frames 400 --rotation-rescue on`; 별도 `evaluate-synthetic-vio.py --dataset <dataset> --results <report.json> --estimate-frame camera --output <new-evaluation.json>`.
- 신규 physical Flip6, independent field GT, thermal/battery, 실제 exposure/calibration, ARKit/ARCore/8th Wall 동일 입력 실행은 미검증. desktop throughput·synthetic GT를 해당 성능으로 확대하지 않음.
- 전달 상태: 실험 branch에 원인·후보·실패를 기록. 기본 제품·active WASM·main 승격 및 remote push는 수행하지 않음.
- 보존: 시작 tracked1302파일 중 소유한6개 외 변경0·누락0, 새 문서2개. active loader·manifest byte 동일, HTTPS7002 stopped, ADB 연결0. `git fetch origin` 성공, 원격 main의 미반영 commit0.
- 변경 파일: `src/frontend/feature_manager.cpp`·`tests/test_projection_trial_contracts.cpp`·`cmake/SLAMCoreSources.cmake`·`CMakeLists.txt`, 이 결과/계획 문서, 이전 종합 분석의 후속 링크, `scripts/dev/README.md`의 빌드 옵션 설명.
