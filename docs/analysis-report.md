# Mobile-SLAM 종합 분석 리포트

> **작성일**: 2026-03-10
> **범위**: VINS-Mono Paper, VINS-Mono 원본 코드, tiny_vins_mono 참조 코드, Mobile-SLAM 현재 코드
> **목적**: 차이점, 버그, 미구현 항목, 잘못된 구현 식별 및 개발 플랜 수립

---

## 목차

1. [코드베이스 관계도](#1-코드베이스-관계도)
2. [VINS-Mono Paper 핵심 요약](#2-vins-mono-paper-핵심-요약)
3. [원본 VINS-Mono 코드 분석](#3-원본-vins-mono-코드-분석)
4. [tiny_vins_mono 분석](#4-tiny_vins_mono-분석)
5. [현재 Mobile-SLAM 분석](#5-현재-mobile-slam-분석)
6. [차이점 분석](#6-차이점-분석)
7. [버그 및 이슈](#7-버그-및-이슈)
8. [미구현 항목](#8-미구현-항목)
9. [잘못 구현된 부분](#9-잘못-구현된-부분)
10. [개발 플랜](#10-개발-플랜)

---

## 1. 코드베이스 관계도

```
VINS-Mono (원본, ROS 기반, 200Hz IMU, 멀티스레드)
    │
    │ refactoring: ROS 제거, namespace 정리, ConfigManager 추가
    ▼
tiny_vins_mono (ROS-free, 동일 알고리즘, Pangolin 시각화)
    │
    │ WASM porting + mobile adaptation + Web JS pipeline
    ▼
Mobile-SLAM (현재 프로젝트, WASM + WebWorker + 모바일 카메라/IMU)
```

**핵심 결론**: 코어 VIO 알고리즘(IMU pre-integration, sliding window optimization, marginalization)은 원본 VINS-Mono와 거의 동일하게 보존됨. 문제는 **모바일 웹 환경 적응** 과정에서 발생하는 데이터 품질/타이밍 이슈.

---

## 2. VINS-Mono Paper 핵심 요약

### 2.1 시스템 파이프라인

| Stage | 모듈 | 설명 |
|-------|------|------|
| 1 | Measurement Preprocessing | KLT optical flow tracking + IMU pre-integration |
| 2 | Initialization | Vision-only SfM → Visual-Inertial alignment |
| 3 | Sliding Window Optimization | IMU + Visual + Marginalization prior 최적화 |
| 4 | Marginalization | Schur complement로 오래된 상태 제거 (정보 보존) |
| 5 | Loop Closure | DBoW2 장소 인식 + 4-DOF pose graph 최적화 |

### 2.2 핵심 수학 공식

**IMU Pre-integration** (Eq. 5-7):
- 위치/속도/회전을 local frame에서 사전 적분
- Mid-point 적분 (구현에서 사용, 논문에서는 Euler로 간략화)
- Bias 변화 시 first-order correction으로 재사용 (repropagate 불필요)

**Visual Residual** (Eq. 25):
- Unit sphere 위의 reprojection error (카메라 모델 독립적)
- Tangent plane에 투영하여 2D residual 생성
- `sqrt_info = focal_length / 1.5` — focal length가 visual weight를 결정

**Sliding Window Cost Function** (Eq. 22):
```
min { ||prior||² + Σ||IMU_residual||²_Mahalanobis + Σ ρ(||visual_residual||²) }
```
- Prior: marginalization에서 생성된 선형화 prior
- IMU: covariance의 역행렬(information matrix)로 가중
- Visual: Huber/Cauchy robust kernel 적용

**Initialization** (Sect. V):
1. Vision-only SfM (five-point + PnP + global BA)
2. Gyroscope bias calibration (rotation matching)
3. Linear alignment (velocity + gravity + scale)
4. Gravity refinement (tangent-plane 2D parameterization, 4회 반복)

### 2.3 설계 결정사항

| 결정 | 이유 |
|------|------|
| Inverse depth parameterization | 원거리 feature에 수치적으로 유리 |
| 초기화 시 accelerometer bias 미추정 | Gravity와 결합되어 짧은 구간에서 관측 불가 |
| Marginalization → linearization lock-in | 계산량 제한을 위한 트레이드오프 (약간의 정확도 손실 허용) |
| Unit sphere residual | 임의의 카메라 모델 지원 |
| Keyframe parallax > threshold | Spatial separation 유지, 정보량 확보 |

### 2.4 성능 기준 (Paper Table I)

| 모듈 | 시간 | 주파수 |
|------|------|--------|
| Feature detection + KLT | 15ms + 5ms | 25Hz |
| Sliding window optimization | 50ms | 10Hz |
| Loop detection | 100ms | async |
| Pose graph optimization | 130ms | async |
| IMU | - | 200Hz |

---

## 3. 원본 VINS-Mono 코드 분석

### 3.1 Feature Tracker (`feature_tracker/src/`)

**파이프라인** (`feature_tracker.cpp:81-167`):
1. CLAHE 히스토그램 균등화 (`clipLimit=3.0, tileGridSize=8x8`)
2. LK optical flow (`cv::Size(21,21)`, 3 pyramids — **하드코딩**)
3. F-matrix RANSAC 아웃라이어 제거 (virtual pinhole FOCAL_LENGTH=460으로 재투영)
4. `setMask()`: track_cnt 내림차순 정렬 → MIN_DIST 원형 마스크
5. `goodFeaturesToTrack`: MAX_CNT까지 새 feature 검출
6. `undistortedPoints()`: normalized coordinates + feature velocity 계산

**출력 형식**: 7D 벡터 `[normalized_x, normalized_y, 1.0, pixel_u, pixel_v, velocity_x, velocity_y]`

**핵심 파라미터**:

| Parameter | 값 | 위치 |
|-----------|---|------|
| `FOCAL_LENGTH` | 460.0 (하드코딩) | `parameters.h:11` |
| `WINDOW_SIZE` | 10 | `parameters.h:12` |
| LK window | 21x21 (하드코딩) | `feature_tracker.cpp:113` |
| LK pyramids | 3 (하드코딩) | `feature_tracker.cpp:113` |

### 3.2 Estimator (`vins_estimator/src/estimator.cpp`)

**State 변수** (WINDOW_SIZE+1 = 11 frames):
```
Ps[11], Rs[11], Vs[11], Bas[11], Bgs[11], pre_integrations[11]
```

**Ceres Parameter Layout**:
```
para_Pose[i][7]      = [tx, ty, tz, qx, qy, qz, qw]
para_SpeedBias[i][9] = [vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
para_Feature[j][1]   = [inverse_depth]
para_Ex_Pose[0][7]   = [tic_x, tic_y, tic_z, qic_x, qic_y, qic_z, qic_w]
para_Td[0][1]        = [td]
```

**핵심 라인**:
- `estimator.cpp:17`: `ProjectionFactor::sqrt_info = FOCAL_LENGTH / 1.5 * Matrix2d::Identity()`
- `estimator.cpp:462`: `average_parallax * 460 > 30` — 초기화 parallax gate (**460 하드코딩**)
- `estimator.cpp:675`: `CauchyLoss(1.0)` — robust kernel
- `estimator.cpp:807`: `DOGLEG` trust region strategy

**Optimization 후 처리** (`double2vector`, line 530-619):
- Yaw-only rotation correction으로 gauge freedom 유지
- Gimbal lock 처리 (pitch ≈ 90°)

**Failure Detection** (`estimator.cpp:621-667`):

| Check | Threshold | Action |
|-------|-----------|--------|
| Ba norm | > 2.5 m/s² | Reset |
| Bg norm | > 1.0 rad/s | Reset |
| Position jump | > 5.0 m | Reset |
| Z-translation | > 1.0 m | Reset |
| Rotation jump | > 50° | Warn only |

### 3.3 IMU Factor & Pre-integration

**`integration_base.h`**:
- Midpoint integration scheme
- 15x15 Jacobian + 15x15 covariance propagation
- `delta_q.normalize()` only in `propagate()`, NOT in `midPointIntegration()`
- Noise matrix: 18x18 (양쪽 endpoint 각각 acc/gyr noise + random walk)

**`imu_factor.h`**:
- `SizedCostFunction<15, 7, 9, 7, 9>`
- sqrt_info: `LLT<>(covariance.inverse()).matrixL().transpose()` — **Cholesky decomposition**
- First-order bias correction 적용

**`projection_factor.cpp`**:
- `SizedCostFunction<2, 7, 7, 7, 1>`
- `pts_camera_i = pts_i / inv_dep_i` — **zero guard 없음** (원본 의도)
- `sqrt_info * residual` 로 가중

**`marginalization_factor.cpp`**:
- Schur complement, `eps=1e-8`
- `NUM_THREADS=4` (pthread)
- SelfAdjointEigenSolver로 sqrt decomposition

### 3.4 Initialization

**`initialStructure()`**:
1. IMU 관측 가능성 체크 (gravity variance > 0.25, 경고만)
2. Global SFM: relativePose → construct → PnP
3. Visual-Inertial Alignment: gyro bias → linear alignment → gravity refinement

**Linear Alignment** (`initial_alignment.cpp:125-197`):
- `A * 1000.0` scaling for numerical conditioning
- Gravity norm validation: `|g.norm() - G.norm()| < 1.0`
- Scale validation: `s > 0`
- Gravity refinement: 4회 tangent-plane 반복

### 3.5 Feature Manager

**Keyframe Decision** (`addFeatureCheckParallax`):
1. `frame_count < 2` → always keyframe
2. `last_track_num < 20` → always keyframe
3. `compensatedParallax2` average ≥ `MIN_PARALLAX` → keyframe

**`compensatedParallax2`**: `p_i_comp = p_i` (compensation is no-op, 원본도 동일)

**`MIN_PARALLAX`**: `yaml_value / FOCAL_LENGTH` (normalized coordinates)

---

## 4. tiny_vins_mono 분석

### 4.1 원본 VINS-Mono와의 관계

tiny_vins_mono는 VINS-Mono의 **구조적 리팩토링**:
- ROS 의존성 완전 제거 (msg, topic, parameter server → 직접 구현)
- Namespace/클래스 재조직: `Estimator`가 `Optimizer`, `FeatureManager`, `Initializer` 소유
- `ConfigManager` 싱글톤: YAML 로딩, 타입 안전 get/set, 변경 콜백
- `SlidingWindow` 전용 클래스 분리
- 동기식 파이프라인 (파일 읽기 → tracking → estimation → visualization)

### 4.2 알고리즘 차이점: 없음

코어 알고리즘은 원본과 **동일**:
- 동일한 IMU factor, projection factor, marginalization
- 동일한 initialization pipeline (SfM + alignment)
- 동일한 feature management (parallax check, depth management)
- 동일한 LLT sqrt_info, midpoint integration, Cauchy loss

### 4.3 주요 구현 디테일

| 항목 | tiny_vins_mono | 비고 |
|------|---------------|------|
| LK params | `cv::Size(21,21)`, 3 pyramids (하드코딩) | 원본과 동일 |
| Extrinsic | `SetParameterBlockConstant` | 최적화 안 함 |
| Solver | DOGLEG + DENSE_SCHUR | 원본과 동일 (원본도 DOGLEG) |
| Camera enum | `KANNALA_BRANDT=0, MEI=1, PINHOLE=2, SCARAMUZZA=3` | 원본과 동일 |
| Default focal | 460.0 (EuRoC) | Config에서 변경 가능 |
| Marginalization thread | `NUM_THREADS=4` | `__EMSCRIPTEN__` 가드 없음 |

---

## 5. 현재 Mobile-SLAM 분석

### 5.1 C++ Engine 구조

**`VIOEngine`** (`src/vio_engine.cpp`): WASM용 headless wrapper
- `configure()`: 파라미터 설정 + Estimator/FeatureTracker 생성
- `processFrame()`: IMU 처리 → feature tracking → estimation → pose 출력
- Cooldown mechanism (30 frames), Init timeout (15s), NaN/divergence detection

**Backend** (원본과 동일한 구조):
- `Estimator`: processIMU/processImage, sliding window 관리
- `Optimizer`: Ceres DOGLEG + DENSE_SCHUR, CauchyLoss(1.0), NaN rollback
- `Marginalization`: Schur complement, WASM에서 single-thread

**Frontend**:
- `FeatureTracker`: 설정 가능한 LK params, edge recovery 추가
- `FeatureManager`: depth clamping [0.1, 200.0] 추가
- `Initializer`: WASM `-O3` Eigen auto 버그 수정, gyro bias conditioning 추가

### 5.2 Web JS Pipeline

```
Camera (camera.js)          IMU (imu.js)
    │                           │
    │ grayscale frame           │ 60Hz accel+gyro
    │ (portrait, downscaled)    │ (bias calibrated)
    ▼                           ▼
VIOWrapper (vio-wrapper.js) ─── async API
    │
    │ Transferable ArrayBuffer
    ▼
VIO Worker (vio-worker.js)
    │
    │ Ring buffer drain + WASM heap write
    ▼
WASM VIO Engine (vio_engine.js)
    │
    │ 4x4 pose matrix
    ▼
Three.js Renderer (renderer.js)
```

**주요 설정 프로파일** (`app.js` `VIO_CONFIGS`):

| 파라미터 | mobile_default | euroc | 비고 |
|----------|---------------|-------|------|
| acc_n | 1.0 | 0.08 | 12.5x 차이 |
| acc_w | 0.001 | 0.00004 | 25x 차이 |
| gyr_n | 0.05 | 0.004 | 12.5x 차이 |
| gyr_w | 0.0005 | 2e-6 | 250x 차이 |
| focal_length | ~232 | ~460 | 2x 차이 |
| processScale | 0.5 | 1.0 | 해상도 절반 |
| frame rate | 30fps (target) | 20fps | 카메라 30fps, VIO 30fps target |
| IMU rate | 60Hz | 200Hz | 3.3x 차이 |
| solver_time | 0.04s | 0.04s | 30fps에 맞춰 조정 |
| num_iterations | 8 | 8 | 30fps에 맞춰 조정 |

### 5.3 WASM Build

- Emscripten + SIMD (`-msimd128`)
- `INITIAL_MEMORY=256MB`, `MAX=512MB`, `STACK=8MB`
- OpenCV SIMD 활성화 (`intrin_wasm.hpp` 패치)
- Eigen SIMD 활성화 (`EIGEN_DONT_VECTORIZE` 제거)
- Ceres: NO_THREADS, MINIGLOG

---

## 6. 차이점 분석

### 6.1 의도적 변경 (정상, 유지)

| # | 영역 | 원본 VINS-Mono | Mobile-SLAM | 이유 |
|---|------|---------------|-------------|------|
| D1 | sqrt_info | `460/1.5=307` (하드코딩) | `focal/1.5` (동적, mobile≈155) | 다양한 카메라 지원 |
| D2 | Solver | DOGLEG (원본도 동일) | DOGLEG | WASM 7.3% 빠름 벤치마크 완료 |
| D3 | F-matrix threshold | 1.0 | 5.0 + edge recovery | 모바일 barrel distortion 대응 |
| D4 | LK params | 21x21, 3 pyramids (하드코딩) | 설정 가능 (`setTrackingParams`) | 모바일 해상도 최적화 |
| D5 | Marginalization threads | 4 (pthread) | 1 (`__EMSCRIPTEN__`) | WASM 단일 스레드 |
| D6 | Camera loading | YAML 파일 | `setIntrinsicParameter()` | WASM sandbox |
| D7 | Depth clamping | 없음 (0.1 floor만) | [0.1, 200.0] range guard | 극단적 depth 방지 |
| D8 | Divergence detection | Ba>2.5, pos>5 | vel>10, pos>100 + auto reset | 무인 운영 대응 |
| D9 | Eigen auto 수정 | `auto` 사용 | `Matrix3d` 명시 | WASM `-O3` dangling ref 방지 |
| D10 | Init alignment guards | 없음 | NaN/Inf check, gravity norm 검증 | WASM 수치 안정성 |

### 6.2 잠재적 문제가 있는 변경

| # | 영역 | 현재 상태 | 문제점 | 심각도 |
|---|------|----------|--------|--------|
| **P1** | IMU noise params | acc_n=1.0, gyr_n=0.05 | IMU 가중치 극도로 낮음 → visual-only에 가까움 | **HIGH** |
| **P2** | Frame interval | ~~80ms (12.5fps)~~ → 33ms (30fps) **수정됨** | 기존 80ms로 인한 displacement 문제 해결 | ~~MED~~ **RESOLVED** |
| **P3** | Init bias clamp | Ba≤2.0, Bg≤0.1 (post-init) | Optimizer 일관성 훼손 가능 (CLAUDE.md death spiral 경고) | **MED** |
| **P4** | solver_time | 0.06s, 10 iterations | WASM에서 각 iteration 느림 → 실시간성 트레이드오프 | **LOW** |

### 6.3 환경 차이 (알고리즘 변경 불가, 적응 필요)

| 요소 | 원본 VINS-Mono | Mobile-SLAM | 영향 |
|------|---------------|-------------|------|
| IMU 주파수 | 200Hz | 60Hz | 프레임당 IMU 3-5개 (최소 수준) |
| IMU 품질 | 연구용 (BMI160 등) | 모바일 MEMS | 노이즈 5-10x 높음 |
| 카메라 해상도 | 752x480 | 240x320 (downscale 후) | Feature 품질 저하 |
| 카메라-IMU 동기 | 하드웨어 동기 | 소프트웨어 (performance.now) | 30-100ms 오차 |
| 렌즈 왜곡 | 보정됨 (YAML) | 미보정 (ISP 하드웨어) | Edge 영역 residual 증가 |
| 프레임 레이트 | 20fps (안정) | 30fps target (33ms interval) | getUserMedia에 30fps 명시 요청 |

---

## 7. 버그 및 이슈

### 7.1 확인된 버그

#### ~~BUG-1: Frame timestamp가 실제 카메라 캡쳐 시점이 아님~~ **수정됨**
- **파일**: `web/js/app.js` (`processLoop`)
- **문제**: `performance.now()`는 프레임 처리 루프 시작 시간 사용. 실제 카메라 캡쳐 시점과 30-100ms 차이.
- **수정**: `requestVideoFrameCallback` API로 `metadata.mediaTime` 하드웨어 타임스탬프 사용. 미지원 브라우저는 `performance.now()` fallback.
- **상태**: **RESOLVED**

#### BUG-2: `drainIMUToWasm` 과다 소비 (수정 진행 중)
- **파일**: `web/js/vio-worker.js`
- **문제**: 원래 코드는 모든 IMU를 drain하여 다음 프레임용 데이터 손실. 현재 diff에서 수정 시도 중 — frameTs 이후 1개만 포함하고 나머지 보존.
- **상태**: 수정 로직은 올바르나, 아직 커밋되지 않은 상태
- **심각도**: **MED** (수정 진행 중)

#### BUG-3: Feature velocity dt=0 방어 없음
- **파일**: `src/frontend/feature_tracker.cpp` (`undistortedPoints`)
- **상황**: 프레임 간 dt가 매우 작을 때 (중복 프레임 등) velocity가 무한대 가능
- **영향**: Feature manager의 parallax/tracking 판단 오류
- **심각도**: **MED**

#### BUG-4: Static `diag_counter` across resets
- **파일**: `src/vio_engine.cpp:283`
- **문제**: VIO reset 시에도 counter가 초기화되지 않음
- **영향**: 진단 로깅 주기만 영향 (기능적 영향 없음)
- **심각도**: **LOW**

### 7.2 구조적 이슈

#### ISSUE-1: Visual weight 과소 + IMU 과소 = 양쪽 모두 약함
- **근본 원인**: Mobile `focal_length≈232` → `sqrt_info = 232/1.5 = 155` (EuRoC의 절반). 동시에 `acc_n=1.0`으로 IMU도 극도로 약화.
- **효과**: 양쪽 모두 약한 constraint → optimizer가 안정적인 해를 찾지 못함
- **증상**: Feature 부족 시 즉시 발산, 정상 tracking 시에도 drift 큼

#### ISSUE-2: 초기화 성공률 저조
- **원인 체인**:
  1. 240x320 해상도 → 특징점 품질 낮음
  2. `relativePose()` threshold: `parallax * focal > 30` → focal=232에서 약 7.4° 필요
  3. 60Hz IMU with variable timing → gyro bias calibration 정밀도 저하
  4. `solveGyroscopeBias()` condition number check → bias=0 반환 가능

#### ISSUE-3: Feature center clustering
- **원인**: PINHOLE model에 k1=0, k2=0. 실제 barrel distortion 존재 → F-matrix RANSAC가 edge feature reject → 중앙 집중
- **현재 대응**: `f_threshold_edge_factor=2.0`
- **한계**: threshold 높이면 outlier도 복원

---

## 8. 미구현 항목

### 8.1 원본 VINS-Mono 대비 미구현

| # | 항목 | 원본 구현 | Mobile-SLAM | 영향도 | 구현 난이도 |
|---|------|----------|-------------|--------|------------|
| **M1** | Loop Closure | DBoW2 + 4-DOF pose graph | ❌ 미구현 | 장기 drift 축적 | VERY HIGH |
| **M2** | Relocalization | Feature DB + PnP | ❌ 미구현 | 트래킹 실패 후 복구 불가 | HIGH |
| **M3** | Online Extrinsic Calibration | `InitialEXRotation` + optimize | ❌ 고정 (`SetParameterBlockConstant`) | R_ic 오차 영구적 | MED |
| **M4** | Time Offset (td) Estimation | `ESTIMATE_TD=1`, `ProjectionTdFactor` | ❌ 미구현 | 카메라-IMU 시간차 미보정 | HIGH |
| **M5** | Motion-only BA | Camera-rate fast optimization (~5ms) | ❌ 미구현 | IMU 전파만으로 중간 포즈 | HIGH |
| **M6** | Rolling Shutter Compensation | `TR` parameter | ❌ 미구현 | 모바일 rolling shutter 미보정 | MED |
| **M7** | Feature Velocity td Correction | `td * velocity` 보정 | ❌ 미구현 | 시간 오프셋 반영 안됨 | LOW |
| **M8** | Parallax Rotation Compensation | `compensatedParallax2` 내 IMU rotation | ❌ no-op (`p_i_comp = p_i`) | Rotation-only 시 불필요한 keyframe | LOW |

### 8.2 구현 완료 항목

| 항목 | 상태 | 비고 |
|------|------|------|
| Failure Detection & Recovery | ✅ 구현 | vel/pos threshold + auto reset |
| WASM Single-thread | ✅ 구현 | `#ifdef __EMSCRIPTEN__` NUM_THREADS=1 |
| Edge Feature Recovery | ✅ 구현 | Distance-aware adaptive threshold |
| Mobile Camera Model | ✅ 구현 | `setIntrinsicParameter()` |
| Depth Range Guard | ✅ 구현 | [0.1, 200.0] clamping |
| Init NaN/Inf Guard | ✅ 구현 | All sliding window states 검증 |
| Configurable LK Params | ✅ 구현 | `setTrackingParams()` API |
| WASM SIMD | ✅ 구현 | Eigen + OpenCV + Ceres SIMD 활성화 |

---

## 9. 잘못 구현된 부분

### WRONG-1: IMU noise 파라미터 과도한 약화

**현재**: `acc_n=1.0` (실제 모바일 IMU 노이즈의 5-10배 이상)

**문제**:
- IMU factor의 information matrix가 극도로 작아짐
- 사실상 visual-only SLAM으로 동작
- Feature가 일시적으로 부족해지면 즉시 발산 (IMU가 버텨주지 못함)
- VINS-Mono의 핵심 장점인 tightly-coupled fusion이 무력화됨

**올바른 접근**:
- IMU noise는 실측 기반으로 설정 (acc_n=0.1~0.3, gyr_n=0.01~0.03)
- Visual weight 조절이 필요하면 `sqrt_info` 스케일링 또는 robust kernel 조정
- 또는 `FOCAL_LENGTH / 1.5` 대신 별도의 visual weight factor 도입

### WRONG-2: 초기화 후 Bias Clamping

**현재**: `estimator.cpp` — initialization 직후 모든 window frame의 Ba≤2.0, Bg≤0.1 clamp

**문제**:
- Optimizer가 이 clamped state를 본 적이 없음
- 첫 optimization에서 clamped 값과 실제 IMU 데이터 간 불일치 발생
- CLAUDE.md에 "Ba/Velocity Clamping Causes Death Spiral" 명시적 경고 있음

**올바른 접근**:
- `solveGyroscopeBias()`와 `LinearAlignment()` **내부**에서 제약 적용
- 또는 initialization 결과가 비정상이면 **재시도** (clamp보다 안전)

### WRONG-3: `compensatedParallax2` No-op

**현재**: `feature_manager.cpp:291` — `p_i_comp = p_i` (원본 VINS-Mono도 동일)

**영향**:
- 순수 rotation 움직임에서 parallax가 실제보다 크게 측정
- 불필요한 keyframe 생성 → sliding window에 informative하지 않은 frame 포함
- 모바일에서 손떨림(주로 rotation)이 많아 영향 더 큼

**참고**: 원본도 동일하므로 "버그"보다는 "개선 가능 항목"에 해당

---

## 10. 개발 플랜

### Phase 1: 데이터 품질 개선 (최우선 — 가장 큰 효과)

> **근거**: VIO는 시간 동기화가 핵심. BUG-1(timestamp 오차)과 P1(IMU 파라미터)이 발산의 주요 원인.

| Task | 설명 | 파일 | 난이도 | 효과 |
|------|------|------|--------|------|
| **T1.1** | ~~Frame timestamp 교정~~ | `web/js/app.js` | ~~MED~~ | ✅ **DONE** — `requestVideoFrameCallback` 적용 |
| **T1.2** | IMU noise 파라미터 재조정 (acc_n=0.2~0.5, gyr_n=0.02) | `web/js/app.js` | LOW | **HIGH** — 미수정 |
| **T1.3** | ~~`requestVideoFrameCallback` API~~ | `web/js/app.js` | ~~MED~~ | ✅ **DONE** — T1.1에 통합 |
| **T1.4** | ~~drainIMUToWasm 수정~~ | `web/js/vio-worker.js` | ~~LOW~~ | ✅ **DONE** — frameTs 기반 drain |

**예상 효과**: T1.1, T1.3, T1.4 완료. T1.2(IMU noise 재조정)는 실측 기반 튜닝 필요.

### Phase 2: Tracking 안정성 개선

> **근거**: Frame interval과 feature 품질이 tracking 지속성을 결정.

| Task | 설명 | 파일 | 난이도 | 효과 |
|------|------|------|--------|------|
| **T2.1** | ~~Frame interval 최적화~~ | `web/js/app.js` | ~~MED~~ | ✅ **DONE** — 30fps tracking / 20fps init |
| **T2.2** | Feature velocity 무한대 방어 (dt < threshold → skip) | `feature_tracker.cpp` | LOW | **MED** |
| **T2.3** | 초기화 parallax threshold 동적 조정 (`focal`에 비례) | `initializer.cpp` | LOW | **MED** |
| **T2.4** | processScale 0.67 옵션 추가 (320x427) | `web/js/app.js` | LOW | **MED** |
| **T2.5** | Degenerate baseline 감지 (너무 짧은 frame interval skip) | `web/js/app.js` | LOW | **LOW** |

### Phase 3: 초기화 성공률 개선

> **근거**: 초기화 실패 → 반복 reset → 사용자 경험 저하.

| Task | 설명 | 파일 | 난이도 | 효과 |
|------|------|------|--------|------|
| **T3.1** | Gyro bias calibration 개선 (정지 감지 자동화) | `web/js/imu.js` | MED | **MED** |
| **T3.2** | Initial bias clamping 제거 → alignment 내부 제약으로 대체 | `estimator.cpp`, `initial_alignment.cpp` | MED | **MED** |
| **T3.3** | IMU excitation check 완화 (모바일 환경 적응) | `initializer.cpp` | LOW | **LOW** |
| **T3.4** | Init frame 축적 전략 개선 (parallax 부족 시 대기) | `estimator.cpp` | MED | **MED** |

### Phase 4: 고급 기능 구현 (장기)

> **근거**: 단기 tracking 안정화 후 장기 정확도와 사용성 향상.

| Task | 설명 | 파일 | 난이도 | 효과 |
|------|------|------|--------|------|
| **T4.1** | td (time offset) estimation 구현 | `estimator.cpp`, `projection_factor` | HIGH | **HIGH** |
| **T4.2** | Motion-only BA (camera-rate pose) | `estimator.cpp` | HIGH | **MED** |
| **T4.3** | Parallax rotation compensation 구현 | `feature_manager.cpp` | MED | **LOW** |
| **T4.4** | Online extrinsic calibration | `estimator.cpp` | MED | **MED** |
| **T4.5** | Loop closure (DBoW2 WASM port) | 새 모듈 | VERY HIGH | **HIGH** |

### 우선순위 로드맵

```
[즉시] Phase 1 ── T1.1 + T1.2 ────────────────── 발산 근본 원인 해결
         │
[1주]  Phase 2 ── T2.1~T2.5 ────────────────── tracking loss 감소
         │
[2주]  Phase 3 ── T3.1~T3.4 ────────────────── 초기화 성공률 향상
         │
[장기] Phase 4 ── T4.1 (td) → T4.2 (fast BA) ── 정밀도 향상
                   T4.4 (extrinsic) → T4.5 (loop closure)
```

---

## 부록 A: FOCAL_LENGTH 사용처 (3곳)

| # | 위치 | 용도 | 현재 Mobile-SLAM |
|---|------|------|-----------------|
| 1 | `estimator.cpp` `setParameter()` | `ProjectionFactor::sqrt_info = focal / 1.5` | ✅ 동적 focal 사용 |
| 2 | `feature_manager.cpp` `addFeatureCheckParallax()` | `min_parallax / focal` (normalized threshold) | ✅ 동적 focal 사용 |
| 3 | `initializer.cpp` `relativePose()` | `parallax * focal > 30` (init gate) | ✅ 동적 focal 사용 |

## 부록 B: 절대 적용 금지 패치 목록

CLAUDE.md "NOT Applied" 섹션 참조. 아래 패치들은 WASM NaN 방지용으로 설계되었으나 **Native C++ 빌드에서 즉시 발산 유발**:

| 패치 | 파일 | 위험 |
|------|------|------|
| SelfAdjointEigenSolver | `imu_factor.h` | Covariance 분해 결과 변질 |
| inv_dep_i clamp (1e-5) | `projection_factor.cpp` | Ceres optimizer 수렴 방해 |
| Marginalization eps 1e-4 | `marginalization_factor.h` | 정보 손실 → 발산 |
| Quaternion normalize after raw params | `imu_factor.h`, `projection_factor.cpp` | Ceres parameterization 충돌 |
| Jacobian/Covariance clamp | `integration_base.h` | Pre-integration 정확도 저하 |
| NaN guard → zero residual | `integration_base.h`, `marginalization_factor.cpp` | 에러 은폐 → 발산 가속 |

## 부록 C: 핵심 인사이트

> **모바일 웹 VIO의 근본적 제약**: 60Hz IMU + 30fps camera target (하드웨어 타임스탬프 활용)은 원본 VINS-Mono(200Hz IMU + 20fps camera + 하드웨어 동기화)에 근접하지만, IMU 밀도(2-3 readings/frame vs 10)가 여전히 제한적. 알고리즘 자체보다 **데이터 품질**(IMU 밀도, 해상도, noise 수준)이 성능의 병목.

> **IMU-Visual Balance가 핵심**: 현재 설정은 IMU를 극도로 약화(acc_n=1.0)하여 visual-only에 가까움. 이는 feature tracking이 안정적일 때만 작동하며, 모바일의 움직임/조명 변화에 취약. IMU noise를 적절히 설정(0.2~0.5)하면 feature 부족 시에도 IMU가 pose를 유지하는 VINS-Mono 본래의 강점을 활용 가능.
