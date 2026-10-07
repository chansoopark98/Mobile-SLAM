# 구조 진단 — 2026-10-06

- 범위: 현재 working tree 읽기 전용 조사; `CLAUDE.md`, `.omx/plans/audit-setup-scope.md` 준수
- 보존: 조사 시작 시 C++/WASM/browser 기존 수정·untracked PnP 파일 존재; 제품 소스 변경 없음
- 근거 구분: **확인** = 코드/빌드 설정/파일 비교, **추론** = 영향 또는 개선 후보, **미검증** = 실행·GT·실기기 측정 필요
- 이 문서의 구조 확인만으로 정확도, frame rate, production 적합성 증명 불가; 실행 결과는 병렬 테스트 담당 보고서 참조

## 우선 작업

1. **P0 / 공통 입력 계약·회귀 확보**: camera model enum, extrinsic 저장 순서, IMU 경계 샘플, reset 상태, 출력 좌표·timestamp를 native/WASM 동일 fixture로 검증
2. **P0 / 빌드 증거 확보**: 실제 compile/link flags, dependencies 버전·source fingerprint, 배포 JS hash를 결과에 기록; native C++ 성공과 배포 WASM 성공 구분
3. **P0 / 차용 코드 provenance 정리**: MIT 표기와 GPLv3 reference 차용 관계 확인; 배포 정책 결정 전 코드별 origin·license inventory 작성
4. **P1 / orchestration 통합**: dataset 실행과 browser 실행이 같은 `VIOEngine` 계약을 사용하도록 단계적으로 통합; GUI·파일 I/O는 adapter로 유지
5. **P1 / 비용·정확도 동시 비교**: PnP off/on, backend decimation, capture 해상도를 동일 데이터로 비교; 처리시간과 pose age/coverage/GT error 함께 측정
6. **P2 / SLAM 범위 결정**: 검증된 VIO 이후 loop closure, relocalization, map persistence 필요성·수락 기준 확정

## 현재 실행 경로

```text
Native dataset
tiny_vins_mono → ConfigManager → VIOSystem
  └ MeasurementProcessor (YAML/image/IMU/FeatureTracker)
    → Estimator → Initializer / FeatureManager / Optimizer / SlidingWindow
  └ Pangolin / IMUGraphVisualizer / TestResultLogger / TrajectoryEvaluator

Live browser + TUM-VI replay browser
app.js 또는 test-tumvi-app.js → VIOWrapper → module Worker(vio-worker.js)
  └ WASM heap + Embind → VIOEngine
    ├ FeatureTracker → Estimator → 공통 backend
    └ PnPFrontend (opt-in, backend 사이 frame)
  → 최신 pose/map points → Three.js renderer
```

| 경계 | 확인 | 근거 |
|---|---|---|
| Native 진입 | `VIOSystem` 실행; `vio_engine` library를 실행체에 링크하지 않음 | `src/tiny_vins_mono.cpp:33`, `CMakeLists.txt:186`, `CMakeLists.txt:194` |
| Native 데이터 경로 | `MeasurementProcessor`가 YAML/feature tracker/data 로딩; VIOSystem이 직접 IMU·image 전달 | `src/utility/measurement_processor.cpp:19`, `src/vio_system.cpp:97` |
| Browser 진입 | live/replay 모두 wrapper 사용; wrapper가 module worker 생성 | `web/js/app.js:11`, `web/js/test-tumvi-app.js:10`, `web/js/vio-wrapper.js:38` |
| WASM 연결 | `/vio_engine.js`가 기본 module; worker에서 `new wasm.VIOEngine()` | `web/js/vio-wrapper.js:38`, `web/js/vio-worker.js:375`, `web/js/vio-worker.js:400` |
| 공통 backend | `Estimator`가 Optimizer, Initializer, FeatureManager, SlidingWindow 소유 | `include/backend/estimator.h:33`, `include/backend/estimator.h:80` |
| PnP 경로 | VIOEngine에서 초기화 후 every FREQ backend / 나머지 PnP; native VIOSystem에는 대응 호출 없음 | `src/vio_engine.cpp:264`, `src/vio_engine.cpp:389`, `CMakeLists.txt:194` |
| PnP 기본 상태 | config false, live `?pnp=1` opt-in | `include/utility/config.h:85`, `web/js/app.js:737` |

## 분리·중복·비활성 후보

- **확인 / IMU·image orchestration 중복**: `VIOSystem::processIMUData/processImageData`와 `VIOEngine::processIMUData/processFrame` 별도 구현 — `src/vio_system.cpp:178`, `src/vio_system.cpp:226`, `src/vio_engine.cpp:120`, `src/vio_engine.cpp:217`
- **확인 / 입력 처리 차이**: VIOSystem의 이전 IMU 값은 호출마다 local zero 초기화; VIOEngine은 member 유지 + invalid dt filter — `src/vio_system.cpp:179`, `src/vio_engine.cpp:128`, `src/vio_engine.cpp:179`
- **확인 / 설정 소유 중복**: ConfigManager의 shared Config, MeasurementProcessor의 global `g_config` 재로딩, VIOEngine의 global overwrite 공존 — `src/config/config_manager.cpp:14`, `src/utility/measurement_processor.cpp:24`, `src/vio_engine.cpp:35`
- **추론 / 다중 engine 제한**: `g_config`와 ProjectionFactor static 정보 공유로 동일 process의 독립 engine/동시 calibration 안전성 부족 — `include/utility/config.h:115`, `src/backend/estimator.cpp:29`; 동시 실행 테스트 미수행
- **확인 / duplicate build 목록**: native 여러 library target, WASM 별도 `VIO_SOURCES` 수동 목록 — `CMakeLists.txt:86`, `wasm/CMakeLists.txt:46`; 동일 core를 별도 정의하는 구조
- **확인 / legacy helper 후보**: `web/js/shared-memory.js`는 class 제공하지만 현재 wrapper/worker import 없음; worker 내부 별도 `WorkerSharedMemory` 사용 — `web/js/shared-memory.js:5`, `web/js/vio-worker.js:171`; `rg -n 'shared-memory|SharedMemory' web` 확인 범위
- **확인 / legacy bundle 후보**: `web/js/vio_engine_worker.js` 존재, 현재 wrapper 기본 경로는 `/vio_engine.js`; CMake에는 별도 worker bundle target도 존재 — `web/js/vio-wrapper.js:38`, `wasm/CMakeLists.txt:141`. 외부 consumer 존재 여부 미검증; 삭제 보류
- **확인 / adaptive solver 미연결**: `solveCeresProblem(problem)`는 default `pending_frames=0`; queue 기반 reduction 분기는 현재 호출에서 활성화되지 않음 — `src/backend/optimizer.cpp:44`, `include/backend/optimizer.h:47`, `src/backend/optimizer.cpp:151`
- **확인 / window 크기 계약 분산**: runtime `estimator.window_size`, compile constant `WINDOW_SIZE=10`, engine 출력의 constant index 공존 — `include/utility/config.h:11`, `include/utility/config.h:60`, `src/vio_system.cpp:255`, `src/vio_engine.cpp:362`; runtime 가변 크기의 동작 보장 미검증

## native ↔ WASM parity의 현재 한계

| 항목 | 확인된 차이/누락 | 근거 |
|---|---|---|
| core compile | native `-O2`, `EIGEN_DONT_VECTORIZE`; WASM Release `-O3`, vectorize 금지 없음 | `CMakeLists.txt:21`, `CMakeLists.txt:30`, `wasm/CMakeLists.txt:89` |
| SIMD | WASM core `-msimd128`는 linker flags에만 지정; 현재 generated core compile flags에 없음 | `wasm/CMakeLists.txt:129`, `wasm/CMakeLists.txt:137`, `wasm/build/CMakeFiles/vio_engine.dir/flags.make:9` |
| dependencies SIMD | OpenCV/Ceres build script는 compile flags에 `-msimd128` 지정 | `wasm/build_opencv.sh:35`, `wasm/build_ceres.sh:43` |
| headless native 빌드 | native 구성 시 Pangolin REQUIRED; headless core/test도 구성 의존성 부담 | `CMakeLists.txt:37`, `CMakeLists.txt:163` |
| unresolved symbols | WASM link `ERROR_ON_UNDEFINED_SYMBOLS=0`, runtime assertions off | `wasm/CMakeLists.txt:124`, `wasm/CMakeLists.txt:125` |
| parity 테스트의 실행 범위 | 실제 WASM/Worker가 아닌 native C++ engine와 native processor 비교 | `tests/test_vio_engine_parity.cpp:126`, `tests/test_vio_engine_parity.cpp:207`, `CMakeLists.txt:259` |
| parity camera enum | test `1 // KANNALA_BRANDT`; 실제 enum 0. tracker는 non-pinhole을 equidistant로 fallback하지만 auto fisheye mask 조건 미충족 | `tests/test_vio_engine_parity.cpp:163`, `include/common/camera_models/Camera.h:15`, `src/frontend/feature_tracker.cpp:330`, `src/vio_engine.cpp:70` |
| parity extrinsic 순서 | test `r_ic.data()` 전달; engine은 명시 row-major 해석. Eigen 기본 Matrix3d column-major와 순서 불일치 | `tests/test_vio_engine_parity.cpp:165`, `src/vio_engine.cpp:45` |
| parity 성공 기준 | comparison 초기화 실패 skip, matched=0이면 error assertion 미실행; sanity는 empty trajectory도 loop 미실행 | `tests/test_vio_engine_parity.cpp:341`, `tests/test_vio_engine_parity.cpp:372`, `tests/test_vio_engine_parity.cpp:389` |

- **추론**: native 경로 통과로 실제 browser 정확도/속도 또는 SIMD 사용을 주장하기 어려움
- **필요 증거**: 동일 calibrated sequence를 native VIOEngine과 실제 배포 WASM에서 실행; initialized frame, pose coverage, per-frame body/camera SE(3) 오차, 전체 ATE/RPE·rotation error, reset 수, 처리시간 p50/p95/p99, pose age 기록
- **빌드 관찰 한계**: `flags.make`는 조사 시 현존 generated 설정; binary의 instruction 검사·fresh rebuild 성능 측정 결과와 구분

## 실시간 추정·SLAM 범위

- **확인 / 현재 제품 기능은 VIO**: README 첫 설명 VIO, binding은 frame processing·local map extraction·reset 제공 — `README.md:3`, `wasm/vio_bindings.cpp:52`
- **확인 / pose update는 frame 호출 결과**: IMU는 worker ring에 저장 후 frame에 drain; 개별 IMU message는 출력 pose 생성 없음 — `web/js/vio-worker.js:105`, `web/js/vio-worker.js:299`, `web/js/vio-worker.js:491`
- **추론 / render pose age 발생**: rendering은 최신 완료된 frame pose 재사용; backend 지연·frame drop 시 현재시각 pose와 차이 가능 — `web/js/app.js:1154`; 실제 device motion latency 미측정
- **확인 / main thread capture 비용 유지**: grayscale capture, CPU canvas readback/conversion, optional downsample 후 worker 송신 — `web/js/app.js:1074`, `web/js/camera.js:532`, `web/js/camera.js:556`, `web/js/app.js:1118`
- **확인 / copy 경계 유지**: wrapper image buffer slice → transfer → worker WASM heap write, 결과 buffer slice — `web/js/vio-wrapper.js:178`, `web/js/vio-worker.js:193`, `web/js/vio-worker.js:202`; wrapper 설명의 zero-copy는 transfer 경계만 해당
- **확인 / 처리량 지표의 한계**: `sendFrame()`은 busy 시 false; app은 반환값 확인 없이 frame counter 증가 — `web/js/vio-wrapper.js:170`, `web/js/app.js:1136`; UI frame count를 처리 완료 frame 수로 사용 불가
- **확인 / WebGPU 범위**: WebGPU는 Three.js rendering backend; tracking/optimization compute shader 호출은 `src/include/web`의 검색 범위에서 없음 — `web/js/renderer.js:134`, `web/js/vio-worker.js:299`
- **확인 / full SLAM 기능 미발견**: 제품 `src/include` 및 WASM API/build 목록에 loop detector, pose graph, global relocalization, map save/load 없음; `getMapPoints()`는 sliding window 점 추출 — `src/vio_engine.cpp:464`, `include/backend/estimator.h:38`, `wasm/CMakeLists.txt:46`, `wasm/vio_bindings.cpp:52`
- **확인 / calibration 확장 미발견**: 제품 source/config 검색에서 temporal-offset state, rolling-shutter factor 없음; reference VINS-Mono에는 관련 설정 안내 존재 — `assets/references/VINS-Mono/README.md:130`; browser timestamp·rolling shutter 적합성은 별도 실험 대상
- **추론 / 단계적 목표**: 입력/시간/좌표 회귀 → 동일 engine 실행·실측 profiler → low-latency prediction/PnP ablation → full SLAM 기능 순서; 이 조사만으로 특정 알고리즘 교체 결정 불가

## 차용 코드·license 근거

| 자료 | 확인된 표기/사용 | 근거 |
|---|---|---|
| root | MIT | `LICENSE:1`, `README.md:272` |
| VINS-Mono | GPLv3 | `assets/references/VINS-Mono/README.md:156`, `assets/references/VINS-Mono/LICENCE:1` |
| tiny_vins_mono | README GPLv3; 제품과 동일 byte 파일 확인 | `assets/references/tiny_vins_mono/README.md:275` |
| VINS-Mobile | GPLv3, 제품 PnP 설명에서 해당 pattern 차용 명시 | `assets/references/VINS-Mobile/LICENSE:1`, `src/vio_engine.cpp:405`, `include/utility/config.h:79` |
| AlvaAR | GPLv3 표기; WASM OpenCV build source는 AlvaAR의 vendored OpenCV | `assets/references/AlvaAR/README.md:157`, `wasm/build_opencv.sh:7` |
| DPVO | MIT; root build target에서는 직접 링크/compile 미발견 | `assets/references/DPVO/LICENSE:1`, `CMakeLists.txt:86` |

- **확인 / 파일 동일성**: Python `Path.read_bytes()`로 제품 `src/include`와 같은 상대경로 `assets/references/tiny_vins_mono` 비교, 17개 byte-identical 파일
- 실제 compile되는 동일 파일 예: `src/backend/factor/projection_factor.cpp`, `src/backend/sliding_window.cpp`, `src/common/gpl/gpl.cc`, `src/common/camera_models/Camera.cc` — `CMakeLists.txt:96`, `CMakeLists.txt:119`, `CMakeLists.txt:125`, `CMakeLists.txt:145`
- SHA-256 예: `src/backend/factor/projection_factor.cpp` = `e40f2d10cec431a474a1b6a8897fa3847ce0f42d04104d06059665964616c45f`
- SHA-256 예: `src/common/gpl/gpl.cc` = `da0a326c040abeb6372b5916b874be807a4344afa15ac5b5cb31b92e8c382e0e`
- **추론 / provenance 불일치 조사 필요**: GPLv3 reference와 동일/수정된 코드가 제품 build에 포함되므로 root MIT 한 줄을 전체 배포 허용 근거로 사용 불가; 파일별 원저작자·허용조건·변경 내역 조사 필요
- **미검증**: 권리자 별도 허가, 모든 차용 파일의 provenance, 상용 배포 조건, vendored dependency notice 완전성. 이 문서는 법적 허용 여부 판정 제외

## 조사 검증

- 코드 조회: `nl -ba`, `rg -n`, CMake native/WASM source 및 link target 추적
- 부재 확인 범위: 제품 `src`, `include`, `config`, `web`, `wasm/vio_bindings.cpp`; reference 전체를 제품 기능으로 포함하지 않음
- legacy 확인 범위: first-party browser 파일의 import/reference 검색; 외부 consumer·runtime bundle hash 확인은 다른 진단 결과 필요
- 파일 비교: 제품/reference 같은 상대경로의 raw bytes 동일성; 동일 파일 17개. 단순 명칭·폴더명만으로 license 판단하지 않음
- 변경 파일: 본 문서만 작성; cleanup 후보를 실제 삭제/통합하지 않음
