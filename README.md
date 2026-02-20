# Mobile-SLAM

VINS-Mono 기반 Visual-Inertial Odometry(VIO) 엔진.
Native C++ 검증 후 WASM+WebGPU 브라우저 이식을 목표로 합니다.

## Architecture

```
src/
  tiny_vins_mono.cpp          # Entry point
  vio_system.cpp              # VIO pipeline orchestration
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
    camera_models/             # Multi-model camera support (Pinhole, Equidistant, Cata, Scaramuzza)
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
tests/                         # Google Test suites
config/                        # YAML configuration files
assets/datasets/               # Test datasets (TUM-VI, EuRoC)
assets/references/             # Reference implementations (VINS-Mono, DPVO, AlvaAR)
```

## Dependencies

| Library | Version | Purpose |
|---------|---------|---------|
| Eigen3 | 3.3+ | Linear algebra, SE(3) alignment |
| OpenCV | 4.x | Feature tracking, image I/O |
| Ceres Solver | 2.2.0 | Nonlinear optimization (FetchContent) |
| Pangolin | - | 3D visualization |
| yaml-cpp | - | Configuration parsing |
| Google Test | 1.14.0 | Unit testing (FetchContent) |

## Build

```bash
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

## Run

```bash
# TUM-VI room1 dataset
./build/tiny_vins_mono config/tum_vi_room1.yaml
```

실행 완료 후 `logs/<timestamp>/` 디렉토리에 결과가 저장됩니다:
- `trajectory_pose.txt` - VIO 궤적 (TUM format: timestamp tx ty tz qx qy qz qw)
- `evaluation.txt` - ATE/RPE 정량 평가 결과 (Ground Truth 존재 시 자동 생성)

## Test

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
