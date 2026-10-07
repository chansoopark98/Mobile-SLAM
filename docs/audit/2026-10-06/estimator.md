# 추정기·센서·정확도 평가 진단

- 대상: 현재 dirty working tree의 `VIOEngine`, `Estimator`, `Optimizer`, 초기 정렬, feature manager, PnP, factor, C++/Python trajectory evaluator
- 기존 알고리즘/제품 코드 변경 없음; 진단 fixture와 기록만 추가
- 증거 수준: 소형 입력으로 현재 소스 재현 / 호출 경로로 확인 / 실제 장치·전체 dataset 검증 필요 항목 구분
- 결론: 현재 `TRACKING`, finite pose, native evaluator의 0 rotation RPE를 production 정확도 근거로 사용 불가
- 전체 native 빌드·dataset·browser 실행 결과는 다른 진단 lane의 기록 참조; 이 문서는 독립 수치 계약 진단

## 확인된 결함

| ID | 우선순위 | 증거 수준 | 위치 | 입력·결과·영향 |
|---|---|---|---|---|
| E01 | P1 | 실제 소스 재현 | `src/utility/trajectory_evaluator.cpp:315–325` | GT identity / 추정 자세 0°, 90°, 180° → 1초 rotation RPE 기대 90°; C++ 반환 0 rad. 계산 대신 `rot_errors.push_back(0.0)` 실행. 자세 정확도 평가 자체 누락 |
| E02 | P1 | 실제 소스 재현 | `src/utility/trajectory_evaluator.cpp:137–139,251–256,299–302` | association은 위치만 저장; RPE는 원본 VIO 배열 timestamp 재사용. 앞의 unmatched 3행을 추가하면 올바른 3 matches / 2 RPE pairs가 0 RPE pairs로 변경 |
| E03 | P1 | 동일 Eigen 대입 재현 | `tests/test_vio_engine_parity.cpp:165`, `src/vio_engine.cpp:45–49` | `cfg.camera.r_ic.data()`는 column-major이며 destination과 alias. row-major 순차 대입으로 원래 SO(3)를 훼손; `||RᵀR−I||` 1.99e−15 → 0.04349. 현재 parity fixture의 calibration 입력 무효 |
| E04 | P1 | 호출 경로 확인 | `src/backend/estimator.cpp:36–63,229–230,247–248`, `src/backend/optimizer.cpp:79–85,399–402` | 내부 divergence/NaN reset에서 sliding window만 초기화; optimizer의 이전 marginalization prior와 parameter block 목록 유지. 재초기화 후 이전 trajectory prior를 새 상태에 적용. 외부 `VIOEngine::reset()`의 estimator 재생성은 별도 경로 |
| E05 | P1 | 호출 경로 확인 | `src/vio_engine.cpp:279–302,341–343,360–387`, `src/backend/estimator.cpp:15,234–251` | feature가 0이면 backend image update 생략 후 기존 NON_LINEAR 상태로 valid pose 반환; PnP matched 0이면 image 처리 생략 후 과거 hasPose/position 재사용. `FailureDetector::detectFailure` 호출 0건; bias·feature·relative jump 감지가 tracking lifecycle과 연결되지 않음 |
| E06 | P1 | 현재 preintegration 소스 재현 | `include/backend/factor/imu_factor.h:37–40`, `include/backend/factor/imu_factor_pnp.h:50–55`, `src/backend/optimizer.cpp:88–101` | 0 sample covariance rank 0 / 60Hz 1 sample rank 12 → covariance inverse 및 sqrt_info NaN; LLT `info()`는 Success도 반환. factor 추가는 null/10초 초과만 검사. 유효 integration coverage·finite whitening 확인 필요 |
| E07 | P1 | 호출 경로 확인 | `src/vio_engine.cpp:120–181`, `src/utility/measurement_processor.cpp:272–289`, `tests/test_vio_engine_parity.cpp:99–113` | native 및 parity packet은 camera 시간 이하 IMU만 포함 → interpolation 분기 미실행, preintegration 끝과 image 시간 불일치. VIOEngine의 future batch는 첫 future로 camera 경계 처리 후 나머지 모두 dt=0 continue; future sample carry 없음. camera 시간 이후 IMU가 실제 browser batch에 포함되는 빈도는 browser lane에서 별도 검증 |
| E08 | P2 | 실제 helper 재현 + 호출 확인 | `include/utility/utility.h:25–35`, `src/backend/estimator.cpp:81`, `src/frontend/pnp_frontend.cpp:97` | normalize 없는 `deltaQ`를 `toRotationMatrix()`에 전달. `ω·dt=1 rad` → det 1.25 / `||RᵀR−I||` 0.35355; finite guard는 통과. engine은 dt ≤0.5초 허용. 일반 60Hz 작은 회전의 누적 영향은 장치 replay 검증 필요 |
| E09 | P2 | 소유권 경로 확인 | `include/frontend/pnp_frontend.h:24–26,73–74`, `src/frontend/pnp_frontend.cpp:16–26,277–281`, `src/vio_engine.cpp:254,351,375,535,562` | raw `IntegrationBase*`를 소유하며 destructor 없음. PnP object reset/destruction 시 최종 window allocation과 IMU buffer 해제 누락. leak 양·반복 reset 메모리 영향은 sanitizer/RSS 측정 필요 |
| E10 | P2 | 호출 경로 확인 | `src/backend/optimizer.cpp:145–172`, `src/frontend/pnp_frontend.cpp:249–252,135–139` | Ceres summary의 termination·solution usability·cost·residual count 미사용; PnP는 feature 1개라도 있으면 solve 후 무조건 hasPose=true. 시간 제한·solver failure·퇴화 기하가 품질 상태로 노출되지 않음 |

- P1: 리팩토링 및 성능 비교 전에 수정·회귀 기준 확보 필요
- P2: 재현 가능한 수치/소유권/품질 계약 문제; 실기기 발생 빈도와 영향 크기는 추가 측정
- E03의 범위: parity test에서 확인된 alias 호출; 별도 row-major buffer를 전달하는 browser 호출의 calibration까지 이 결과로 단정 금지
- parity의 `model_type=1`은 enum상 MEI지만 현재 `setIntrinsicParameter()`의 else에서 equidistant 생성; 모델을 pinhole로 잘못 전달했다는 해석은 부정확. 다만 `configure()`의 fisheye mask는 0일 때만 활성화(`src/vio_engine.cpp:70`) → native YAML의 `fisheye=1`과 불일치

## 평가 계약의 한계

- C++ ATE: `Eigen::umeyama(..., true)`만 제공(`src/utility/trajectory_evaluator.cpp:164–177`); alignment scale 미보고
- Python ATE: Sim(3)만 제공(`scripts/evaluation/compare_trajectories.py:329–348`); scale은 보고
- doubled metric trajectory 재현:
  - rigid SE(3) alignment oracle ATE: 0.666666666667 m
  - C++ Sim(3) ATE: 1.76e−16 m
  - Python Sim(3) ATE: 3.55e−16 m; 보정 scale 0.5
  - Sim(3)는 shape 진단으로 유효; metric VIO의 scale 정확도를 별도로 보고해야 비교 가능
- C++ translation RPE: aligned world displacement 차이만 계산(`src/utility/trajectory_evaluator.cpp:272–276`); 회전이 있는 SE(3) relative pose error와 다른 지표
- Python RPE: matched index를 사용한 `T_gt_rel⁻¹ · T_est_rel` 및 실제 rotation angle 계산(`scripts/evaluation/compare_trajectories.py:354–435`); synthetic 90°/association fixtures의 독립 oracle와 일치
- C++/Python association: nearest GT 재사용 허용, timestamp gap·전체 eligible frame 대비 coverage·reset epoch 미보고; Python tolerance 50ms / C++ default 10ms
- no-pair 기본 C++ 결과는 0.0(`include/utility/trajectory_evaluator.h:10–23`); 미측정 상태와 0 error 구분 필요
- 기존 native evaluator tests의 RPE는 position 중심; quaternion test는 loading size만 검사. `RPEComputationStraightLine`은 `rmse_trans >= 0`만 요구(`tests/test_trajectory_evaluator.cpp:204–227`)
- parity comparison:
  - 한 pipeline 초기화 실패 시 skip(`tests/test_vio_engine_parity.cpp:341–343`)
  - matched 0이면 comparison assertion 없음(`tests/test_vio_engine_parity.cpp:372–384`)
  - sanity test는 empty trajectory에서도 통과(`tests/test_vio_engine_parity.cpp:388–396`)
  - 두 pipeline가 GT 없이 서로 일치해도 공통 오차·metric scale·production 성능 보장 불가

## 수학·calibration 리팩토링 주의점

- 좌표 계약은 현재 일관된 형태:
  - `R_wc = R_wi R_ic`
  - `P_wc = P_wi + R_wi t_ic`
  - evaluator의 camera→body inverse는 위 식의 역변환
  - feature는 `liftProjective` 후 `(x/z,y/z,1)`; inverse depth는 optical Z 기준. Euclidean range로 무심코 변경 금지
- fisheye ray를 z로 나눌 때 z≈0/negative/finite 검사 없음(`src/frontend/feature_tracker.cpp:343–352`); projection factors도 camera Z와 inverse depth division guard 없음(`src/backend/factor/projection_factor.cpp:22–32`, `src/backend/factor/perspective_factor.cpp:28–31`)
- feature depth 0 또는 NaN은 `depth < 0` 검사로 실패 분류되지 않음(`src/frontend/feature_manager.cpp:95–109`); map visualization의 finite filtering만으로 estimator 입력 보호 불가
- `PoseLocalParameterization::PlusJacobian`의 quaternion 부분은 실제 derivative 대신 identity를 반환(`src/backend/factor/pose_local_parameterization.cpp:23–28`)
  - factor Jacobian도 ambient quaternion derivative 대신 tangent derivative를 7열에 배치한 VINS legacy 관례(`src/backend/factor/projection_factor.cpp:45–53`)
  - `MinusJacobian` rotation 2I와 Plus identity의 곱은 I가 아님(`src/backend/factor/pose_local_parameterization.cpp:50–59`)
  - 정상 Ceres manifold로의 변환 시 모든 analytic factor와 marginalization Jacobian을 함께 검증; manifold만 교체하면 기존 local convention 훼손 가능
- initialization motion excitation·gyro condition number·gravity magnitude/scale sign guard 존재; observability·metric scale accuracy의 충분조건은 아님
- `RefineGravity`는 네 차례 iteration에서 A/b를 재설정하지 않고 이전 normal equation에 더한 후 매번 ×1000(`src/frontend/initialization/initial_alignment.cpp:92–99,134–145`); 첫 linearization 잔존 영향은 synthetic known-gravity/scale 및 upstream differential 검증 필요
- 초기 gyro bias clamp ±0.35 rad/s, scale/initial depth/noise/solver time 주석은 경험값; device별 독립 calibration·noise identification·ablation 없이 보편 성능 근거로 사용 불가
- `pos.norm() > 100m` reset(`src/backend/estimator.cpp:244–249`)은 정상 장거리 이동도 거부; 거리 제한과 divergence 판단을 제품 사용 범위에 맞게 분리 필요

## 재현 명령·결과

- 환경: `g++ (Ubuntu 11.4.0-1ubuntu1~22.04.3) 11.4.0`, system Eigen `/usr/include/eigen3`(native audit CMake의 `/usr/share/eigen3/cmake`와 동일 설치), Python scientific imports 사용
- 입력: 3개의 synthetic pose / unmatched prefix / 현재 TUM config의 SO(3) 행렬; 실제 SLAM trajectory accuracy benchmark와 구분
- 새 파일:
  - `tests/audit_estimator_contracts.cpp`
  - `tests/audit_python_evaluation.py`
  - `tests/audit_imu_covariance.cpp`
  - `docs/audit/2026-10-06/estimator-fixtures/{gt.csv,rotation90.txt,unmatched_prefix.txt,double_scale.txt}`
- 출력:
  - `estimator-contracts.log`: alias·rotation RPE·association RPE·SO(3) 계약 실패 4건, Sim(3) 한계 1건
  - `estimator-python-contracts.log`: rotation 90° / matched RPE 2 pairs oracle 일치, Sim(3) scale 한계 확인
  - `estimator-imu-covariance.log`: 0/1 sample whitening invalid; 2/6 sample finite
  - `estimator-source-fingerprint.log`: 실제 재현 대상·fixture SHA-256
- 진단 executable exit 0은 기록 생성 성공; `CONTRACT_FAIL`, `WHITENING_INVALID`는 결함 재현. green regression / 품질 gate 통과로 해석 금지

```bash
mkdir -p /tmp/mobile-slam-estimator-audit
g++ -std=c++17 -O0 -Iinclude -I/usr/include/eigen3 tests/audit_estimator_contracts.cpp src/utility/trajectory_evaluator.cpp -o /tmp/mobile-slam-estimator-audit/audit_estimator_contracts
/tmp/mobile-slam-estimator-audit/audit_estimator_contracts docs/audit/2026-10-06/estimator-fixtures
PYTHONDONTWRITEBYTECODE=1 python3 tests/audit_python_evaluation.py docs/audit/2026-10-06/estimator-fixtures
g++ -std=c++17 -O0 -Iinclude -I/usr/include/eigen3 -I/usr/include/opencv4 tests/audit_imu_covariance.cpp -o /tmp/mobile-slam-estimator-audit/audit_imu_covariance
/tmp/mobile-slam-estimator-audit/audit_imu_covariance
```

## 다음 회귀·평가 gate

1. 수치 계약: nonidentity/noncommuting extrinsic의 row-major round trip·source alias 금지/보호; SO(3) finite/orthogonality; inverse depth·camera Z 유효성; covariance factorization·analytic local Jacobian finite difference
2. 시간 계약: 60/100/200Hz IMU × 10/20/30/60Hz camera, asynchronous phase offset, duplicate/out-of-order, no-data interval, 여러 future sample, gap/pause/resume. integration sum_dt가 camera 간격과 일치하고 sample ownership 유일성 확인
3. lifecycle 계약: textured→blank→textured, blur/occlusion, solver unusable, corrupted bias, 반복 reset. stale pose의 source timestamp·age·prediction/measurement 구분; 모든 estimator/PnP/prior state epoch 일치
4. metric 계약: 실제 SE(3) ATE + raw/relative scale error, 실제 SE(3) translation/rotation RPE, match tolerance·coverage·GT segment·init time·reset/recovery 횟수; metric 없는 구간은 NA/invalid 상태
5. runtime gate: native / headless WASM / desktop browser / 실제 mobile 같은 recording replay 및 calibration; PnP off/on, source features·solver time ablation; p50/p95/p99 end-to-end pose age와 processing stage latency
6. 독립 정확도 gate: held-out motion/lighting/device와 외부 GT; dataset/dev tuning과 test 분리. 성공 trajectory만 추리는 selection·reset 후 align 재적합·Sim(3) 보정으로 실패를 숨기지 않도록 전체 eligible coverage 병기

- 아직 미검증: 전체 dataset GT 정확도, 실제 mobile 센서·rolling shutter/time offset·calibration, device 온도/지속 실행 latency, reset prior 영향 크기, PnP leak 실제 RSS, production 목표 수치
