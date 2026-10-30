# slam_refact

별도 C++17 SLAM 엔진과 WebAssembly·XR 연결 코드입니다. 공개 소스 재사용과 reference binary 관찰 기반 구현을 포함하며 출처·개별 라이선스는 [루트 README](../README.md#external-code)와 [제3자 고지](engine/third_party/THIRD_PARTY_NOTICES.md)를 확인합니다.

## Structure and API

| 경로 | 용도 |
|---|---|
| `engine/include/slam_refact/` | 공개 C++ API |
| `engine/src/` | frontend, map, tracking, bundle adjustment, metric scale, image targets, optional ML 모듈 |
| `engine/wasm/` | `SlamRefactGlue` 바인딩 |
| `engine/tests/` | unit·evidence·WASM 회귀 |
| `engine/third_party/` | vendored source·shim·호환 library 및 출처 manifest |
| `tools/harness/` | native replay와 궤적 평가 도구 |
| `web-chunk/` | engine loader·GPU pyramid·센서·XR 연결·샘플 |
| `web-smoke/` | desktop browser 검증 |

- 코어: `Engine::Track(FrameInput)` — 지연 없는 단일 추적 step.
- 앱 연결: `GlueRunner` — `PushDeviceMotion`, `PushDeviceOrientation`, `ProcessFrame`, `Configure`, `Recenter`.
- C++ frame timestamp는 int64 nanoseconds입니다. 영상·센서의 시각과 순서를 호출자가 맞춥니다.
- `PublishedFrame`은 반복 여부·원본 frame index·tracking status/reason·pose·world points를 함께 제공합니다.
- 코어 내부 좌표는 y-up·left-handed·camera +Z, published pose는 y-up·right-handed·GL camera −Z입니다. quaternion은 wxyz입니다.
- `Engine`과 `GlueRunner`는 thread-safe하지 않습니다. 별도 세션에는 별도 instance를 사용합니다.

## Native build

- 필요: CMake 3.21+, Ninja, C++17 compiler, Eigen 3.4, Ceres 2.2.0.
- GTest 1.14.0은 시험 전용, zlib은 replay 전용입니다. Python·NumPy·Pillow는 일부 fixture/평가 도구에 필요합니다.
- 엔진 CMake는 기본 offline 모드입니다. 처음에는 source cache를 준비하거나 CMake 경로를 지정합니다.

```bash
mkdir -p build/_deps
git clone --depth 1 --branch 2.2.0 https://github.com/ceres-solver/ceres-solver.git build/_deps/ceres-solver-src
git clone --depth 1 --branch v1.14.0 https://github.com/google/googletest.git build/_deps/googletest-src
cmake -S slam_refact/engine -B build/slam-refact-native -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build/slam-refact-native --parallel 2
ctest --test-dir build/slam-refact-native --no-tests=error -L '^unit$' --parallel 2 --output-on-failure
```

이미 source cache가 있다면 clone을 반복하지 않습니다. 다른 경로는 `SLAM_REFACT_CERES_SOURCE_DIR`, `SLAM_REFACT_GTEST_SOURCE_DIR`로 지정합니다.

- 현재 알려진 unit 실패: `FrameFrameTrackerVfft.AcceptedMotionAccuracy` 1개. rotation 오차 `0.10087055545943382 > 0.05`로 기준을 충족하지 않습니다.
- optional 입력 관련 skip: `OrientGateReport.ExploreDirectory`, `GlueImageTargetGeometry.DemoCylinderFromFiles`.
- `evidence`, `mlmodel` 시험은 별도 원본 입력·모델이 필요합니다. 입력이 없다는 사실을 정확도 통과로 취급하지 않습니다.
- `held` 계열은 고정 입력의 실행 무결성을 보호하는 별도 시험입니다. 일반 unit 명령에 섞어 통과 수로 집계하지 않습니다.

## WASM build

Ceres의 WASM 설치를 먼저 준비합니다. OpenCV는 이 엔진에 필요하지 않습니다.

```bash
source "$EMSDK/emsdk_env.sh"
bash wasm/build_ceres.sh
emcmake cmake -S slam_refact/engine -B build/slam-refact-wasm -G Ninja \
  -DCMAKE_BUILD_TYPE=Release -DSLAM_REFACT_BUILD_TESTS=OFF -DSLAM_REFACT_BUILD_HARNESS=OFF
cmake --build build/slam-refact-wasm --parallel 2
SLAM_REFACT_WASM_DIR="$PWD/build/slam-refact-wasm/wasm" \
  node --test slam_refact/engine/tests/wasm/smoke.test.mjs
node slam_refact/web-chunk/tools/make-manifest.mjs --engine-dir build/slam-refact-wasm/wasm
```

출력: `build/slam-refact-wasm/wasm/slam_refact_engine.{js,wasm}`와 `manifest.json`.

- 전체 WASM 회귀에는 replay bundle·native trace와 개발용 image-target fixture가 추가로 필요합니다.
- curved image-target 회귀의 개발 fixture 경로는 `SLAM_REFACT_IT_CYL_DIR`, `SLAM_REFACT_IT_CONE_DIR`로 지정합니다.
- 실제 모델·실기기·현장 GT 없이 모델 정확도나 모바일 성능을 확정하지 않습니다.

## Replay and browser

- `build/slam-refact-native/harness/slam_refact_replay`는 engine config·IMU·frame 목록으로 구성된 replay bundle을 읽습니다.
- bundle 준비·평가는 `tools/harness/`, 독립 평가 기준은 [개발 도구 안내](../scripts/dev/README.md)를 참고합니다.
- 측정 페이지: `/slam-refact.html`. XR 샘플: `/slam-refact-xr.html`.
- 엔진 loader는 JS·WASM의 SHA와 manifest를 검증합니다. 같은 디렉터리에서 온 manifest는 무결성 대조이며 배포자 인증을 대신하지 않습니다.
- 새 artifact는 먼저 isolated staging에서 검사합니다. `tools/deploy_xr.py --label <unique-label> --engine-dir <build-dir>`의 기본 동작은 dry-run입니다.
- 실시간 추적·map point 표시·metric scale·surface/hit-test 결과는 각각 검증해야 합니다. desktop smoke는 실제 전화의 camera/IMU·GPU·발열·장시간 정확도를 보증하지 않습니다.
