# VINS-Mono 직접 실행과 현재 VIOEngine 비교

- 계획 날짜: 2026-10-13. 실행 완료: **2026-10-14 00시대, Asia/Seoul**.
- 판정: **오픈소스 VINS-Mono와 현재 VIOEngine의 실제 영상·IMU 추적 및 독립 GT 비교 완료. 상용 엔진 내부 복원·실기기 정확도 수락과 별도.**
- 변경: `scripts/dev/references/vins-mono/{replay.py,ros_replay.py,verify_inputs.py,evaluate.py,README.md}`, 이 문서. 제품 코드·원본 참조 소스·공유 dataset·기존 서비스 변경 없음.
- 증거 root: `build/commercial-slam-20261013/open-reference/`. 기존 evidence 덮어쓰기 없음. 처음 `vins-room1-600/`은 leading IMU3개가 다른 pilot로 보존; 최종 비교는 `vins-room1-600-matched/`와 `current-room1-600-oracle/`만 사용.

## 환경·실제 입력

- VINS-Mono source: `assets/references/VINS-Mono`, clean commit `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d`, GPLv3. 별도 Docker snapshot은 기존 `build/reference-engines/vins-mono/source-manifest.json`.
- 동결 image: `sha256:0b13b81efd04612178cc6a5bfb5e6d85e49c7e1d7b17fd6571ad81bc672f5ba9`. ROS Kinetic·OpenCV3.3.1-dev·Ceres1.12.0. 직접 실행한 binary: `feature_tracker/feature_tracker`, `vins_estimator/vins_estimator`. pose graph·loop closure 제외.
- 현재 소스 HEAD: `2789f64534f5f8a5d6b4d65fc616a7bdeeb7613f`. 제품 object는 새 `current-native/`에 Release C++17·jobs2로 빌드. anchor-depth compile option OFF·rotation rescue OFF·experimental observation policy0·PnP OFF·yaml profile.
- 현재 dependency: OpenCV4.5.4·Ceres2.2.0. 이미 검증된 Ceres archive를 read-only import; Ceres 자체 fresh build는 아님. archive SHA256 `348d012e74faf65d2370616f9b49016540cc89e267528cb5f785dc5a5e5b3418`. `current-native-provenance.json`의 `nativeCeresArchives`와 실제 Ninja link line 확인. imported target이라 해당 manifest의 자동 Ceres version field는 null; 실제 source `include/ceres/version.h`에서2.2.0 확인.
- 현재 executable SHA256: `809a7ca310707877791e9252ac198c441ad8437e9bfad5596e3827c44a27a405`. source·compile flag·dependency·artifact hash: `current-native-provenance.json`.
- 입력: 이미 공식 복원된 `assets/datasets/tum/dataset-room1_512_16`, camera0 첫600frame·512×512 gray·약20Hz, **29.951416053s**. IMU5976개·약200Hz·timestamp초·acceleration m/s²·gyro rad/s. canonical `/mnt/backup/SLAM`에는 쓰지 않음.
- K·distortion·`T_I_C`·noise·max_cnt150·min_dist20·F threshold1·10iterations/0.1s는 현재 room1 YAML 사용. 실제 `dso/camchain.yaml`의 `T_C_I` inverse와 일치하는 calibrated extrinsic 고정. upstream 기본 TUM extrinsic을 임의로 대신 사용하지 않음.
- ROS container: `--network none --cpus 2 --user 현재UID:GID`; dataset·wrapper mount `:ro`, output만 쓰기. 별도 localhost roscore; 기존 ROS·Docker 서비스 건드리지 않음. replay0.5배속은 input 안정성을 위한 조건이며 real-time 성능 결과가 아님.
- **실제 입력 byte 동등성 PASS**: ROS bag의 decoded pixels와 현재 C++가 처리한 `input-oracle.bin` 비교.600 images SHA256 `4c7a9190f38dcd97c03fd44ee02152bf30fd8c15131a512bd32412533cd9bd4c`, ordered unique5976IMU의 timestamp/acc/gyro 7-double SHA256 `0910d2c8f5e4d1187e1fffb90361e6908826594378945cb2840917b767bc95d2` 양쪽 동일. `input-equivalence.json`에 count·hash 보존. packet 묶음·ROS callback timing·내부 처리 순서까지 동일하다는 의미는 아님.

## 관측 결과

| 항목 | VINS-Mono | 현재 VIOEngine native |
|---|---:|---:|
| 원본 camera/고유IMU 입력 | 600/5976 | 600/5976 |
| feature message | 598 | 직접 engine 호출600 |
| 유효 pose/고정600분모 | 570/600, **95.0%** | 555/600, **92.5%** |
| 첫 pose의 dataset 경과 | **1.500152264s** | **2.250179052s**, frame45 |
| GT 일대일 association | 570/570 | 555/555 |
| 전체 SE3 scale1 translation ATE RMSE | **0.014971m** | **0.014515m** |
| 전체 rotation APE RMSE | **0.417153°** | **0.652218°** |
| 공통555timestamp ATE RMSE | **0.015078m** | **0.014515m** |
| 공통555timestamp rotation APE RMSE | **0.415282°** | **0.652218°** |
| 공통555timestamp 1s RPE translation RMSE | **0.007593m** | **0.008776m** |
| 공통555timestamp 1s RPE rotation RMSE | **0.473935°** | **0.321725°** |
| 전체 Sim3 진단 fitted scale | 1.003514 | 0.993926 |

- 평가: VINS ROS odometry는 `T_W_I`; 현재 `T_W_C`는 fixed `T_I_C`로 body 변환. GT는 `dso/gt_imu.csv`; 일대일 nearest timestamp≤0.01s, translation alignment SE3 scale=1, rotation 포함. Sim3는 진단만 보고. 공통 timestamp 매칭 허용≤1µs. 각 metric의 association·rank·RPE pair 수는 `metrics-room1-600.json`.
- VINS 입력600image가 관측되었으나 feature598은 upstream의 first image return 및 first feature publish 생략 계약. pose570개 존재를 frame600전체 정확도 통과로 바꾸지 않음.
- 현재 initializing45·tracking555·Lost0·invalid pose0·initialization loss transition0. VINS는 실행 중 roscore/tracker/estimator 모두 alive였고570pose를 출력. 내부 reset/feature loss 계측을 따로 추가하지 않았으므로 해당 상태0을 주장하지 않음.
- 현재 두 번째 native 실행: wall17.887624s, engine p50/p95/p99 **26.102488/43.367334/58.676903ms**. 첫 실행은 wall20.987169s·p95 61.729005ms. 호스트 다른 작업 영향이 있어 둘 다 보존. VINS playback wall67.494754s는0.5배속·5s drain을 포함하므로 engine latency와 비교 금지.
- 종료 한계: bounded playback 종료 이후 SIGINT 요청에서 roscore0·tracker0·**vins_estimator−6**·player0. 로그의 `terminate called without an active exception`; upstream `estimator_node.cpp::main()`은 `std::thread measurement_process{process}` 생성 뒤 `ros::spin()`에서 돌아와 join/detach 없이 종료. 이 source와 일치하는 종료 abort로 해석하며 tracking 중 충돌로 분류하지 않음. `shutdown-summary.json`과 원본 log 보존. 정상 shutdown 통과는 아님.

## 소스에서 확인한 차이

| 계약 | 고정 VINS-Mono | 현재 소스·이번 실행 영향 |
|---|---|---|
| focal로 정규화하는 threshold/weight | [parameters.h](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/parameters.h)의 `FOCAL_LENGTH460`; [parameters.cpp](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/parameters.cpp)는 keyframe threshold `10/460`; [estimator.cpp](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/estimator.cpp)의 projection sqrt info `460/1.5` | current readback focal190.975892·threshold0.05236263·sqrt info127.317261. YAML 숫자가 같아도 internal geometry threshold·vision/IMU weight가 같지 않음. |
| 불충분한 depth | [feature_manager.cpp](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/feature_manager.cpp)는 DLT depth<0.1이면 `INIT_DEPTH5m` 대체 | `src/frontend/feature_manager.cpp`는 nonfinite·비양수 depth를 invalid 또는 geometry pending으로 분리. 특정 회전 구간 복구 차이의 후보이며 이번 짧은 room1 결과만으로 원인 확정 불가. |
| fisheye mask | 고정 `config/fisheye_mask.jpg` 읽기 | current `src/frontend/feature_tracker.cpp`는 mask path 없을 때 image 중앙·radius0.95배 원형 생성. 실제 image 입력은 같아도 검출 영역이 다름. |
| 처리 경로 | ROS 이미지 callback→feature topic→IMU/feature queue→estimator thread | current `tests/audit_dataset_replay.cpp`→`VIOEngine::processFrame` 직접 호출. `VIOSystem` PC entry·browser Worker·물리 phone 시험이 아님. |
| 빌드·solver 환경 | ROS Kinetic/OpenCV3.3.1/Ceres1.12, upstream ROS runtime | C++17/OpenCV4.5.4/Ceres2.2, 이번 별도 native object. 두 버전의 backend 비용·부동소수 동작·thread scheduling 차이 존재. |

- 결과 해석: VINS가 이 구간에서0.75s 먼저 pose를 출력하고95% coverage를 보였으며, 공통 timestamp에서는 translation ATE가 현재 쪽에서 약0.56mm 낮고 rotation APE는 VINS 쪽이 낮음. 한 데이터 구간·1회 최종 실행으로 엔진 전체 우열·빠른 회전 복구·장시간 안정성을 확정하지 않음.
- 현재 Flip6의 약42.5Hz DeviceMotion·미측정 camera calibration/exposure timing과 이번200Hz IMU·calibrated global-shutter rig는 다른 조건. 이 결과를 실제 phone Lost 해결·상용 SDK 동급으로 확대하지 않음.

## 재현·검증

```bash
# source object 새 빌드; Ceres archive는 기존 검증본 read-only reuse
cmake -S . -B build/commercial-slam-20261013/open-reference/current-native -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
  -DMOBILE_SLAM_BUILD_VIEWER=OFF -DMOBILE_SLAM_EXPERIMENTAL_ANCHOR_DEPTH=OFF \
  -DFETCHCONTENT_SOURCE_DIR_CERES-SOLVER="$PWD/build/tracking-performance-20261013/ceres-verified" \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$PWD/build/_deps/googletest-src"
flock build/reference-env-build.lock cmake --build \
  build/commercial-slam-20261013/open-reference/current-native --target native_replay --parallel 2

python3 scripts/dev/references/vins-mono/replay.py \
  --dataset assets/datasets/tum/dataset-room1_512_16 \
  --output build/commercial-slam-20261013/open-reference/vins-room1-600-matched \
  --frames 600 --rate 0.5
build/commercial-slam-20261013/open-reference/current-native/native_replay \
  assets/datasets/tum/dataset-room1_512_16 config/tum_vi_room1.yaml \
  build/commercial-slam-20261013/open-reference/current-room1-600-oracle \
  600 bracket 0 1 10 0.1 1 0 0 600 yaml 0 0
python3 scripts/dev/references/vins-mono/verify_inputs.py \
  --vins build/commercial-slam-20261013/open-reference/vins-room1-600-matched \
  --native build/commercial-slam-20261013/open-reference/current-room1-600-oracle \
  --output build/commercial-slam-20261013/open-reference/input-equivalence.json
python3 scripts/dev/references/vins-mono/evaluate.py \
  --vins build/commercial-slam-20261013/open-reference/vins-room1-600-matched \
  --native build/commercial-slam-20261013/open-reference/current-room1-600-oracle \
  --ground-truth assets/datasets/tum/dataset-room1_512_16/dso/gt_imu.csv \
  --config config/tum_vi_room1.yaml \
  --output build/commercial-slam-20261013/open-reference/metrics-room1-600.json
```

- 위 output은 이미 존재하므로 재시험 시 **새 output 이름** 사용. wrapper는 기존 replay/metrics 덮어쓰기 거부.
- 실행 증거: `current-{configure,build,replay-oracle}.log`, `vins-room1-600-matched/{manifest,input-summary,runtime-summary,shutdown-summary}.json`, `input-equivalence.json`, `metrics-room1-600.json`.
- 검증: 새 wrapper4개 Python syntax, `git diff --check`, 기존 strict metric regression **9/9**, image+IMU 실제 byte parity, 두 엔진 실제 trajectory·GT association 완료. 별도 full native suite·WASM/browser·휴대전화·full room1/room4 benchmark는 이번 lane에서 실행하지 않음.
