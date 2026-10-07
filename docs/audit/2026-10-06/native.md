# Native 빌드·테스트 진단

- 기준: 2026-10-06, Git HEAD `b2826c037a50036d286329b1a56daa1a75ba57a3` + 시작 시 존재한 tracked/untracked 구현 변경
- 보존: 기존 `build/`, `wasm/`, 제품 소스 변경 유지; 새 산출물 `build/audit-native/`에 한정
- 소스 85개 SHA-256 manifest: `d4eaa2b25142c68f051360dc0374f92a6fd6af4eef3bf12c82dc9202128511a5`; 개별 hash [native-source-fingerprint.log](native-source-fingerprint.log)
- 기존 바이너리: `build/test_vio_engine_parity` 2026-03-16 16:04:15 +0900; 이전 산출물을 이번 검증 근거로 사용하지 않음
- 새 빌드: Release `-O2 -DNDEBUG`, C++17, Eigen vectorization 비활성화; 새 컴파일 194단계 성공
- 실제 속도/정확도: 아래 dataset 재현 범위와 분리; 빌드 및 단위 테스트 통과만으로 production 성능 판단 불가

## 환경

| 항목 | 확인 값 |
|---|---|
| CMake | 3.22.1 |
| C++ compiler | GNU 11.4.0 |
| Ninja | 1.10.1 |
| OpenCV | 시스템 4.5.4 |
| Eigen | 시스템 3.4.0 |
| yaml-cpp | 시스템 0.7.0 |
| Pangolin | `/usr/local/lib/cmake/Pangolin`, 0.9.4 |
| Ceres | 로컬 FetchContent source tag `2.2.0` |
| GoogleTest | 로컬 FetchContent source tag `v1.14.0` |
| CUDA | configure 자동 감지 12.8.93; Ceres CUDA object 컴파일 포함 |

- 환경·dirty snapshot: [native-environment.log](native-environment.log)
- Ceres CUDA 컴파일 성공 ≠ SLAM GPU 가속 확인; 실제 추정기 device 실행 미검증
- source 다운로드 없이 기존 `build/_deps/*-src`만 재사용; 기존 object/library 미재사용
- clangd에 사용할 새 `build/audit-native/compile_commands.json` 생성

## 재현 명령

- 실행 위치: `/home/park-ubuntu/park/SOLUTION/Mobile-SLAM`

```bash
cmake -S . -B build/audit-native -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DFETCHCONTENT_SOURCE_DIR_CERES-SOLVER=/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/_deps/ceres-solver-src \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/_deps/googletest-src \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
/usr/bin/time -v cmake --build build/audit-native --parallel 4
/usr/bin/time -v ctest --test-dir build/audit-native --output-on-failure --timeout 180 -j 1
```

| 단계 | 결과 | 관측 |
|---|---|---|
| clean configure | exit 0 | 새로운 Ninja build tree |
| native 전체 build | exit 0 | 194단계, wall 81.27초, peak compiler RSS 1,906,544 KiB |
| CTest 기본 cwd | exit 8 | 5/6 실행 파일 통과; parity fixture 실패 |
| repo cwd 직접 GTest | 24 통과 / 3 실패 | parity fixture 데이터 미존재; dataset 복원 전 결과 |

- wall/RSS: 이번 desktop build 관측값; 모바일 frame latency 수치 아님
- 로그: [native-configure.log](native-configure.log), [native-build.log](native-build.log), [native-ctest.log](native-ctest.log), [native-gtest.log](native-gtest.log)

## 단위 테스트 실행 범위

| 실행 파일 | GTest 개수 | dataset 복원 전 직접 실행 |
|---|---:|---|
| test_integration_base | 3 | 3 통과 |
| test_sliding_window | 5 | 5 통과 |
| test_trajectory_evaluator | 7 | 7 통과 |
| test_config_validation | 4 | 4 통과 |
| test_measurement_robustness | 5 | 5 통과 |
| test_vio_engine_parity | 3 | fixture 3 실패 |
| 합계 | 27 | 24 통과 / 3 실패 |

- 최초 fixture 실패: `./assets/datasets/tum/dataset-room1_512_16/mav0/cam0/data.csv` 미존재
- CTest 실행 cwd: `build/audit-native`; parity `DATASET_PATH`, `CONFIG_PATH`는 repo 상대 경로; 데이터 복원 후에도 기본 CTest는 cwd 문제로 실패 가능
- parity native 경로: YAML `show_track: 1` → `cv::imshow`; headless 재현 시 Qt offscreen 또는 GUI 비활성화 설정 필요
- 일반 CLI `tiny_vins_mono`: `VIOSystem::processSequence()`의 Pangolin viewer 실행 필수; 현재 bounded headless CLI 옵션 없음

## ASan·UBSan

- 별도 ignored harness: `build/audit-native/sanitizer/CMakeLists.txt`; 영속 복제 [native-sanitizer-harness.log](native-sanitizer-harness.log)
- compiler flags: `-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer`; native Eigen 정의 동일
- instrumented product 범위: `IntegrationBase` header, `src/backend/sliding_window.cpp`, `src/common/frame.cpp`, `src/utility/trajectory_evaluator.cpp`, `src/utility/config.cpp`, `src/utility/utility.cpp`
- GoogleTest: 새 Release build의 archive 재사용; GoogleTest/시스템 Eigen·yaml-cpp 전체 instrumentation 없음
- 결과: 3 실행 파일 / 15 cases 통과, sanitizer 진단 없음, wall 0.06초
- 범위 제외: 전체 estimator/optimizer/marginalization/PnP/feature tracker/WASM/live input

```bash
cmake -S build/audit-native/sanitizer -B build/audit-native/sanitizer/build -G Ninja
/usr/bin/time -v cmake --build build/audit-native/sanitizer/build --parallel 2
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
ctest --test-dir build/audit-native/sanitizer/build --output-on-failure --timeout 60 -j 1
```

- 로그: [native-sanitizer-configure.log](native-sanitizer-configure.log), [native-sanitizer-build.log](native-sanitizer-build.log), [native-sanitizer-ctest.log](native-sanitizer-ctest.log)

## 검증 계약 결함

- `tests/test_config_validation.cpp:31`: `ConfigManager::validateConfiguration()` 호출 없음; config literal의 `> 0` 조건만 검증; validation 회귀 보호 없음
- `tests/test_vio_engine_parity.cpp:372`: `matched > 0`일 때만 parity threshold 적용; non-empty trajectory끼리 timestamp 공통점 0개면 통과 가능
- `tests/test_vio_engine_parity.cpp:388`: sanity loop 이전 최소 pose 개수 assert 없음; trajectory 비어 있으면 통과 가능
- `tests/test_vio_engine_parity.cpp:341`: trajectory 미생성 시 `GTEST_SKIP`; 테스트 실행 완료와 추적 초기화 성공의 구분 필요
- `tests/test_measurement_robustness.cpp:53`: empty IMU field 처리 기대값 `>= 1`; malformed row 수용 여부 엄밀히 검증하지 못함
- trajectory evaluator 테스트: synthetic position alignment 중심; independent GT에 대한 실제 VIO translation/rotation/scale/coverage 성능 증거 없음
- 고정 `/tmp/trajectory_eval_test`, `/tmp/measurement_robust_test` 생성·`rm -rf` 정리; 동시 동종 테스트 충돌 가능

## 다음 회귀 기준

- parity 입력 경로를 절대 경로/명시적 test fixture로 전달; CTest 실행 위치에 독립적인 데이터 계약
- 초기화 성공, 최소 pose 수, timestamp 매칭 수/전체 coverage, NaN/rotation checks 모두 필수
- configuration public API invalid input 회귀; literal assertion 교체
- sanitizer 전체 backend/실제 sequence로 확대; invalid timestamp/IMU gap/재초기화/reset API 회귀
- GT groundtruth와 independent pose/rotation/drift 평가; desktop throughput와 실제 모바일 p50/p95/p99 frame/pose latency 분리

## 교정된 headless replay harness

- 추가 파일: `tests/audit_dataset_replay.cpp`, `scripts/dev/native-replay.sh`; 제품 CMake/추정기 변경 없음
- 컴파일: 새 native CDB의 compiler flags/link library 재사용; 엔진 prerequisite 새로 확인 후 harness 직접 컴파일
- 입력: dataset/config/output 절대 경로 변환, monotonically increasing timestamps, CSV filename, image 크기/내용, IMU camera span 확인
- 교정: YAML config의 독립 복사에서 row-major extrinsics 생성; 실제 `Camera::KANNALA_BRANDT=0` 사용; `configure()` 이후 덮어쓴 defaults를 YAML 복사로 복원
- 실행: GUI 미사용, OpenCV threads 명시, RNG seed 0; backend `processFrame()`과 image decode 소요 분리
- IMU 정책: `bracket` = image까지의 샘플 + 첫 future sample, future sample을 다음 image interval에서 재사용; `until-image` = 해당 image timestamp 이하만 전달
- 기록: `frames.csv` per-frame process/decode/status/features/IMU 수, `trajectory-camera.txt` camera-to-world TUM pose, `summary.json` 초기화/coverage/status/timing
- quantiles: 정렬된 값의 `(n-1)*q` 선형 보간; p50/p95/p99/max; 전체 frame과 tracking status frame 별도
- exit: `0` 최소 1 valid pose, `2` 입력/IO 오류, `3` invalid pose, `4` pose 미생성; exit 0만으로 정확도 보증 불가
- 검증: compile 성공, usage/invalid option/missing dataset 모두 exit 2; synthetic static 20 frames 실행 완료, 기대한 no-pose exit 4, JSON/CSV shape 통과; 실제 추적 성능 근거에서 제외; [native-replay-preflight.log](native-replay-preflight.log)
- source/binary provenance: `build/audit-baseline/native-replay/build-provenance.json`

```bash
scripts/dev/native-replay.sh
build/audit-baseline/native-replay/native-replay \
  assets/datasets/tum/dataset-room1_512_16 config/tum_vi_room1.yaml \
  build/audit-baseline/native-replay/room1-bracket-600 600 bracket 0 1
```

- 기존 parity와 구분: parity 자체의 calibration 입력/GUI/cwd 결함을 보존한 재현과 교정된 입력으로 실행한 엔진 성능 기록 모두 필요
- 정확도 평가: camera-to-world trajectory를 YAML camera→IMU extrinsics로 body pose에 변환 후 official mocap GT와 timestamp association; independent SE(3) 평가 및 coverage 필수

## 기존 parity calibration 런타임 probe

- dataset 이전에 `configure(...g_config.camera.r_ic.data(), g_config.camera.t_ic.data()...)` 동일 입력 패턴 재현
- 관측: expected 대비 R Frobenius 차이 `0.03075313400415542`; `RᵀR-I` norm `0.04348667173503357`; determinant `0.9999375946865844`
- 원인: column-major Eigen `.data()`를 row-major 입력으로 해석 + 자기 alias matrix에 comma initialization 순차 쓰기; 단순 transpose보다 큰 계약 결함
- enum `1` 입력의 `fisheye=0`; YAML `fisheye: 1`과 불일치
- 실행 로그: [native-config-probe.log](native-config-probe.log); 제품 소스 수정 없음

## Official room1 교정 입력 native 600-frame baseline

- 데이터 복원 후 fresh harness 실행; 2,821 available frames / 28,122 IMU에서 첫 600 frames, dataset time span 29.951416초
- 결과: exit 0, valid poses 555/600 (92.5%), 첫 pose frame 45 / 2.250179초, invalid pose 0, initialization loss 0
- 파라미터: Kannala-Brandt enum 0, 실제 YAML extrinsics/IMU/solver/tracking, PnP off, bracket IMU, OpenCV 1 thread, RNG seed 0
- `processFrame()` p50 20.078 ms / p95 27.136 ms / p99 33.038 ms / max 78.376 ms
- image decode mean 2.522 ms 별도; replay wall 13.498초
- 제한: concurrent browser benchmark 진행 환경; desktop native offline throughput 관측; 실제 모바일 camera→display latency/GPU 실행 증거 없음
- 산출물: `build/audit-baseline/native-replay/room1-bracket-600/{summary.json,frames.csv,trajectory-camera.txt}`
- 로그: [native-replay-room1-bracket-600.log](native-replay-room1-bracket-600.log)
- root independent GT scorer 결과와 결합 전까지 translation/rotation 정확도 판단 보류

## Official room1 전체 sequence native baseline

- 실행: fresh calibrated harness, 전 frame 2821 / IMU 28122, dataset span 141.004464초
- 결과: exit 0, valid poses 2776 (98.4048%), first frame 45 / 2.250179초, invalid 0, initialization loss 0
- `processFrame()` p50 19.126 ms / p95 25.452 ms / p99 29.764 ms / max 77.686 ms
- image decode mean 2.560 ms, wall 62.151초; 파라미터·측정 제한은 600-frame run과 동일
- 산출물: `build/audit-baseline/native-replay/room1-bracket-full/{summary.json,frames.csv,trajectory-camera.txt}`
- 로그: [native-replay-room1-bracket-full.log](native-replay-room1-bracket-full.log)
- 독립 GT 평가: root scorer 담당; 전체 sequence도 finite/orthogonal pose 생성 성공이 정확도 통과의 대체 근거는 아님

## 데이터 복원 후 기존 parity fixture 재현

- 원본 repo cwd + `QT_QPA_PLATFORM=offscreen OPENCV_FOR_THREADS_NUM=1` 실행: 3 cases 중 1 통과 / 2 실패
- 실패 원인: 시스템 OpenCV GTK HighGUI `Can't initialize GTK backend`, `GTK backend is not available`; Qt offscreen 미적용
- 실행된 engine 구간: 300 frames→172 poses (두 test), 200 frames→72 poses; dataset 미존재 실패와 구분
- 로그: [native-parity-restored-dataset.log](native-parity-restored-dataset.log)
- 기본 CTest parity 복원 후 재실행: build cwd에서 dataset 상대 경로 오해석; fixture dataset missing 실패 유지; [native-ctest-restored-dataset.log](native-ctest-restored-dataset.log)
- 대안 fixture: `build/audit-native/parity-headless-fixture` cwd, YAML 복사에서 `show_track: 1`만 `0` 변경, 실제 dataset symlink; 원본 YAML/제품 소스 보존

## IMU feed policy ablation, native 600 frames

| 정책 | pose count | init frame | process p95 ms | process p99 ms | invalid/loss |
|---|---:|---:|---:|---:|---|
| bracket | 555/600 | 45 | 27.136 | 33.038 | 0/0 |
| until-image | 555/600 | 45 | 26.071 | 28.234 | 0/0 |

- 동일 calibrated input / YAML / PnP off / 1 thread; 궤적은 서로 다름; accuracy 우열은 root GT scorer 평가 필요
- 동시에 browser benchmark 진행; 위 시간 차이를 입력 정책만의 latency 개선으로 귀속하지 않음
- 산출물: `build/audit-baseline/native-replay/room1-until-image-600/{summary.json,frames.csv,trajectory-camera.txt}`
- 로그: [native-replay-room1-until-image-600.log](native-replay-room1-until-image-600.log)

## GUI를 끈 복제 fixture의 기존 parity 결과

- `build/audit-native/parity-headless-fixture`에서 원본 바이너리 실행; 복제 YAML `show_track: 0`만 적용
- 결과: 3/3 cases 통과, exit 0, wall 26.18초
- VIOEngine: 300→172 poses, 200→72 poses; native pipeline: 300→255 poses
- timestamp matched 172; position difference 평균 0.0958 m / max 0.1357 m; rotation 평균 3.1233° / max 7.5735°
- 차이 원인 분리 미완료: fixture calibration alias/model/mask 및 IMU policy/초기화 구간이 서로 다름
- 결론: broad threshold의 parity 통과는 동일 calibration/timestamp 계약, 초기화 coverage, independent GT accuracy 보장 불가
- 최종 existing suite 상태: synthetic unit 24/24 통과; 원본 root cwd parity 1/3 통과 + 2 GTK 실패; GUI 비활성 복제 parity 3/3 통과; 기본 CTest parity는 cwd 실패
- 로그: [native-parity-headless-fixture.log](native-parity-headless-fixture.log)

## PnP dual-rate ablation, native 600 frames

- bracket/교정 입력 동일, PnP enabled, FREQ 3; 기본 source 설정 PnP off는 유지
- 결과: 551/600 poses (91.8333%), first frame 45, invalid 0, initialization loss 0
- status TRACKING 555 frames 중 4 frames pose 미반환; status와 fresh pose 구분 필요
- process mean 10.719 ms / p50 7.949 ms / p95 21.258 ms / p99 29.360 ms / max 76.943 ms; wall 7.929초
- 산출물: `build/audit-baseline/native-replay/room1-pnp-bracket-600/{summary.json,frames.csv,trajectory-camera.txt}`
- 로그: [native-replay-room1-pnp-bracket-600.log](native-replay-room1-pnp-bracket-600.log)
- 속도 단독 통과 판단 제외; PnP trajectory의 independent GT accuracy/coverage와 결합 필요

## 최종 보존·검증

- 기존 tracked/untracked source 85개 시작 snapshot 대비 hash 동일; [native-source-stability.log](native-source-stability.log)
- 제품 CMake/estimator/frontend/browser 파일 수정 없음
- 새 코드: `tests/audit_dataset_replay.cpp`, `scripts/dev/native-replay.sh`; 문서/증거 `docs/audit/2026-10-06/native*`; build 출력 ignored tree 한정
- 새 harness compiler/link 성공, shell syntax 통과, owned-file trailing whitespace 없음
- 기존 generated `wasm/libs/ceres_build/*/progress.make` trailing whitespace 관측; 이전 변경 보존; 이번 새 코드 문제와 구분

- 추가 strict compile check: `-fsyntax-only -Wall -Wextra -Wpedantic`, exit 0; 새 harness 진단 없음
- 기존 header warning: Ceres miniglog unused parameter, `include/utility/utility.h:140` extra semicolon; 변경 보존
- 로그: [native-replay-static-check.log](native-replay-static-check.log)
