# Mobile-SLAM

VINS-Mono 기반 Visual-Inertial Odometry(VIO) 엔진.
Native C++ 검증 후 WASM 브라우저 이식을 통한 모바일 웹 Visual SLAM.

## Architecture

```
src/
  tiny_vins_mono.cpp          # Native entry point
  vio_system.cpp              # VIO pipeline orchestration
  vio_engine.cpp              # Headless VIO engine (WASM/programmatic API)
  frontend/
    feature_tracker.cpp        # KLT optical flow feature tracking
    feature_manager.cpp        # Feature lifecycle management
    failure_detector.cpp       # VIO failure detection
    initialization/
      initializer.cpp          # VIO initialization coordinator
      initial_sfm.cpp          # Structure from Motion
      initial_alignment.cpp    # Visual-inertial alignment
      solve_5pts.cpp           # 5-point essential matrix
  backend/
    estimator.cpp              # Sliding window VIO estimator
    sliding_window.cpp         # Window state management
    optimizer.cpp              # Ceres-based nonlinear optimization
    factor/
      integration_base.h       # IMU preintegration
      imu_factor.h             # IMU residual
      projection_factor.cpp    # Visual reprojection residual
      marginalization_factor.cpp # Schur complement marginalization
      pose_local_parameterization.cpp
  common/
    camera_models/             # Multi-model camera (Pinhole, Equidistant, Cata, Scaramuzza)
    gpl/                       # Geometry utility (quaternion parameterization)
    frame.cpp                  # Frame data structure
  config/
    config_manager.cpp         # Singleton config with validation
  utility/
    config.cpp                 # YAML config loading
    measurement_processor.cpp  # IMU/Image data loading, feature extraction
    trajectory_evaluator.cpp   # ATE/RPE trajectory evaluation
    test_result_logger.cpp     # Trajectory file output
    visualizer.cpp             # Pangolin 3D visualization
    imu_graph_visualizer.cpp   # Real-time IMU graph
    logging.h                  # Header-only macro logging

include/                       # Header files (mirrors src/ structure)
tests/                         # Google Test suites (5 suites, 23 tests)
config/                        # YAML configuration files

wasm/
  CMakeLists.txt               # WASM build configuration (Emscripten)
  vio_bindings.cpp             # embind C++ <-> JavaScript bindings
  build_opencv.sh              # OpenCV WASM static library build script
  build_ceres.sh               # Ceres Solver WASM static library build script
  test_wasm_module.mjs         # WASM module load test
  test_wasm_integration.mjs    # Multi-frame integration test
  dist/                        # Built WASM module output (vio_engine.js)

web/
  index.html                   # Mobile VIO web application
  server.js                    # HTTPS dev server (real cert + self-signed fallback)
  vio_engine.js                # WASM module (copied from wasm/dist/)
  js/
    app.js                     # Main application (Camera+IMU+VIO+Renderer)
    vio-wrapper.js             # High-level JS API wrapping WASM VIOEngine
    shared-memory.js           # WASM heap memory management (malloc/free)
    camera.js                  # getUserMedia camera capture + grayscale
    imu.js                     # DeviceMotion IMU capture (iOS 13+ permission)
    renderer.js                # Three.js trajectory + map point renderer

assets/
  datasets/                    # Test datasets (TUM-VI)
  references/                  # Reference implementations (VINS-Mono, DPVO, AlvaAR)
  keys/                        # SSL certificates for HTTPS
```

## Dependencies

### Native Build

| Library | Version | Purpose |
|---------|---------|---------|
| Eigen3 | 3.3+ | Linear algebra, SE(3) alignment |
| OpenCV | 4.x | Feature tracking, image I/O |
| Ceres Solver | 2.2.0 | Nonlinear optimization (FetchContent) |
| Pangolin | - | 3D visualization |
| yaml-cpp | - | Configuration parsing |
| Google Test | 1.14.0 | Unit testing (FetchContent) |

### WASM Build

| Library | Version | Purpose |
|---------|---------|---------|
| Emscripten | 5.0.0 | C++ to WASM compiler toolchain |
| OpenCV (WASM) | 4.5.4 | Feature tracking (static, no GUI/I/O) |
| Ceres Solver (WASM) | 2.2.0 | Optimization (NO_THREADS, miniglog, EIGENSPARSE) |
| Eigen3 | 3.3+ | Header-only, shared with native |

## Native Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## Native Run

```bash
# TUM-VI room1 dataset
./build/tiny_vins_mono config/tum_vi_room1.yaml
```

실행 완료 후 `logs/<timestamp>/` 디렉토리에 결과가 저장됩니다:
- `trajectory_pose.txt` - VIO 궤적 (TUM format: timestamp tx ty tz qx qy qz qw)
- `evaluation.txt` - ATE/RPE 정량 평가 결과 (Ground Truth 존재 시 자동 생성)

### Benchmark (TUM-VI room1)

| Metric | Value |
|--------|-------|
| Poses | 2787 / 2821 |
| ATE RMSE | 0.8951 m |
| ATE Median | 0.7876 m |
| RPE Trans RMSE | 0.7346 m |

## Native Test

```bash
cd build && ctest --output-on-failure
```

| Test Suite | Tests | Description |
|-----------|-------|-------------|
| IntegrationBaseTest | 3 | IMU preintegration 정확성 |
| SlidingWindowTest | 4 | Sliding window 상태 관리 |
| TrajectoryEvaluatorTest | 7 | ATE/RPE 계산, SE(3) alignment, frame 변환 |
| ConfigValidationTest | 4 | Config 파라미터 유효성 검증 |
| MeasurementRobustnessTest | 5 | Malformed 입력 처리, path traversal 방어 |

## WASM Build

### 1. Prerequisites

```bash
# Emscripten SDK
cd ~/emsdk && ./emsdk activate latest
source ~/emsdk/emsdk_env.sh
```

### 2. Build WASM Dependencies

```bash
# OpenCV (core, imgproc, calib3d, features2d, video, flann)
cd wasm && bash build_opencv.sh

# Ceres Solver (NO_THREADS, miniglog, EIGENSPARSE)
bash build_ceres.sh
```

### 3. Build VIO WASM Module

```bash
cd wasm
mkdir -p build && cd build
emcmake cmake ..
emmake make -j$(nproc)
```

Output: `wasm/build/vio_engine.js` (3.4MB, SINGLE_FILE with embedded WASM)

### 4. WASM Module Test

```bash
cd wasm
node test_wasm_module.mjs        # Module load + API test
node test_wasm_integration.mjs   # Multi-frame processing test
```

## Web Client

### Start Dev Server

```bash
cd web
node server.js [port]
```

- 기본 포트: 8444
- `assets/keys/`에 실제 SSL 인증서가 있으면 자동 사용, 없으면 자체 서명 인증서 생성
- 모바일 브라우저에서 접속: `https://<server-ip>:8444`

### VIOEngine JavaScript API

```javascript
import VIOWasm from './vio_engine.js';

// 1. Initialize
const wasm = await VIOWasm();
const engine = new wasm.VIOEngine();

// 2. Configure
engine.configure(width, height, fx, fy, cx, cy,
                 modelType, k2, k3, k4, k5,
                 r_ic_ptr, t_ic_ptr,
                 acc_n, acc_w, gyr_n, gyr_w, g_norm);

// 3. Process frames
const hasPose = engine.processFrame(
    imagePtr, width, height,
    imuPtr, imuCount,
    poseOutputPtr);

// 4. Query state
engine.isInitialized();
engine.getFeaturePointCount();
engine.getMapPoints(outputPtr, maxCount);
engine.reset();
```

메모리 관리는 `SharedMemory` 클래스를 통해 WASM heap에서 `_malloc`/`_free`로 수행합니다.

## Configuration

`config/tum_vi_room1.yaml` 예시:

```yaml
dataset_path: ./assets/datasets/tum/dataset-room1_512_16

frame_skip: 0
start_frame: 0
end_frame: -1

model_type: KANNALA_BRANDT
image_width: 512
image_height: 512

# IMU noise parameters
acc_n: 0.0028
gyr_n: 0.00016
acc_w: 0.00086
gyr_w: 0.000022
g_norm: 9.81007
```

지원 카메라 모델: `KANNALA_BRANDT` (equidistant), `PINHOLE`, `MEI` (catadioptric), `SCARAMUZZA`

## License

MIT License. See [LICENSE](LICENSE).
