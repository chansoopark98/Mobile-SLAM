# Mobile-SLAM
Mobile Visual-SLAM Project

Tightly-coupled Visual-Inertial Odometry engine based on [tiny_vins_mono](assets/references/tiny_vins_mono/) (VINS-Mono ROS-free C++17).

**Phase A**: Native C++ VO/VIO — 데이터셋/웹캠 실시간 검증
**Phase B**: WASM + WebGPU — 브라우저 이식 (30fps+ mobile)

## Tech Stack

| Layer | Native (Phase A) | Web (Phase B) |
|-------|------------------|---------------|
| SLAM Core | C++17 | C++17 → WASM (Emscripten) |
| Solver | Ceres 2.2 (DENSE_SCHUR) | Ceres → WASM |
| Linear Algebra | Eigen 3.4 | Eigen (WASM) |
| Feature Tracking | OpenCV 4.x (LK flow) | OpenCV core → WASM |
| GPU | N/A | WebGPU Compute Shaders |
| 3D Viewer | Pangolin | Three.js |
| Test | Google Test | GTest (C++) + Vitest (TS) |

## Build & Run

### Prerequisites

- GCC/Clang (C++17), CMake 3.16+, OpenCV 4.x (`apt install libopencv-dev`)
- Ceres 2.2 + Eigen 3.4 + GTest: FetchContent 자동 다운로드

### Build

```bash
# CMake 설정 (최초 1회 — Ceres/Eigen/GTest FetchContent 다운로드)
cmake -B build-native -DCMAKE_BUILD_TYPE=Release

# 전체 빌드
cmake --build build-native -j$(nproc)
```

### Test

```bash
# 전체 테스트 (135 tests, 20 suites)
cmake --build build-native --target slam_tests -j$(nproc)
./build-native/tests/cpp/slam_tests

# 특정 테스트만
./build-native/tests/cpp/slam_tests --gtest_filter="Estimator.*"
./build-native/tests/cpp/slam_tests --gtest_filter="SlidingWindow.*"
```

### Webcam Demo

```bash
cmake --build build-native --target slam_webcam -j$(nproc)
./build-native/src/core/slam_webcam
# SSH: DISPLAY=:1 ./build-native/src/core/slam_webcam
# Custom intrinsics: --fx 500 --fy 500 --cx 320 --cy 240
```

GUI: Camera 영상 + tracked features, Pangolin 3D Viewer (point cloud, trajectory, frustum)
Keys: `q`=종료, `r`=리셋, `p`=일시정지

### TUM VI Dataset Evaluation

```bash
cmake --build build-native --target slam_dataset_tumvi -j$(nproc)

# VO mode (visual only)
./build-native/src/core/slam_dataset_tumvi

# VIO mode (visual-inertial)
./build-native/src/core/slam_dataset_tumvi --vio

# With 3D visualization
./build-native/src/core/slam_dataset_tumvi --vio --visualize
```

### Adding New Files

1. `src/core/CMakeLists.txt`에 소스 추가
2. `tests/cpp/CMakeLists.txt`에 테스트 추가
3. `cmake -B build-native` (재설정)
4. `cmake --build build-native -j$(nproc)`

### WASM Build (Phase B)

```bash
source ~/emsdk/emsdk_env.sh
emcmake cmake -B build-wasm -DCMAKE_BUILD_TYPE=Release
emmake make -C build-wasm -j$(nproc)
```