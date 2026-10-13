# 모바일 VIO 엔진 조사와 적용 후보

- 조회일: **2026-10-13, Asia/Seoul**. 질문: Android Chrome·monocular camera·browser IMU·WASM 조건에서 현재 추적 소실을 줄일 설계와 비교 엔진은 무엇인가.
- 결론: **현재 입력으로 Mobile-SLAM보다 우수하다고 검증된 대체 엔진은 없음.** 공개 구현의 관측가능성·keyframe·map 재사용 설계는 참고 가능. 저전력 CPU 후보 `sqrtVINS`, 2026년 신규 후보 `LEVIO`·`VkVIO`도 확인.
- 범위: 공식 저장소·논문·SDK 문서 조사, 공개 소스 정적 대조, 독립 기하 계산. 제품 코드·의존성 추가·엔진 빌드·신규 Flip6 실행 없음.
- 선행 근거: [공개 VIO](2026-10-12-open-vio.md), [웹 엔진](2026-10-12-web-engines.md), [ARKit·ARCore·8th Wall](2026-10-13-commercial-tracking.md), [실패 후보·현재 수락 상태](../fixes/2026-10-13-tracking-performance-results.md).
- 원문·HEAD/release·SHA256: [sources-manifest.json](../../build/flip6-analysis-20261013/online-research/sources-manifest.json). 조회한 원문 79개; 공식 API·텍스트만 저장, 모델·SDK binary·패키지 설치 없음.
- `HEAD date`는 commit committer 시각. release와 구분. GitHub release 배열이 비어 있어도 프로젝트 전체 배포 부재를 뜻하지 않음.

## 목적에 맞는 6개 비교군

| 비교군 | 확인한 입력·추정·복구 | 실행 조건·브라우저 이식 판단 | 이번 프로젝트에 유용한 차이 |
|---|---|---|---|
| [AlvaAR](https://github.com/alanross/AlvaAR) | monocular visual SLAM, WASM. README에서 IMU fusion은 미완료 항목. `findCameraPoseWithIMU`는 gyro/acc를 estimator에 전달하지 않는 현 소스 확인 | 실제 WASM 경로 있음. C++·OpenCV·Eigen·Ceres 계열 CPU 경로. GPU feature tracking도 미완료 항목. metric VIO 비교 엔진으로 분류하지 않음 | map point 투영→저층 LK→실패 subset 재시도. 현재 rotation rescue와 중복되는 부분보다 성숙 landmark의 투영 대응 유지가 핵심. [system.cpp](https://github.com/alanross/AlvaAR/blob/7796af500ee92001ac2a9888363ff64d7a3bee75/src/slam/src/system.cpp), [visual_frontend.cpp](https://github.com/alanross/AlvaAR/blob/7796af500ee92001ac2a9888363ff64d7a3bee75/src/slam/src/visual_frontend.cpp) |
| [OpenVINS](https://github.com/rpng/open_vins) / [sqrtVINS](https://github.com/rpng/sqrtVINS) | monocular/stereo+IMU, metric VIO. OpenVINS의 MSCKF/SLAM feature·FEJ·online calibration. sqrtVINS는 LLT square-root covariance filter와 dynamic initialization | CPU·Eigen·OpenCV, 공식 실행 예제는 ROS. sqrtVINS는 single-thread float32 embedded 평가. 공식 browser/WASM 배포는 이번 조회에서 확인 못함 | Ceres sliding-window 최적화와 다른 비용 구조. 장기 비교 엔진 후보. 단순히 double을 float로 바꾸는 변경과 다름. estimator 내 SLAM landmark를 global loop closure와 동일시하지 않음 |
| [Basalt](https://gitlab.com/VladyslavUsenko/basalt) | calibrated camera+IMU VIO/VO·mapping, square-root marginalization. 현재 frame과 연결되는 landmark 비율·keyframe 간격·baseline을 사용 | CPU Eigen/TBB/OpenCV, vcpkg의 Pangolin·ROS bag·RealSense는 도구/실행 구성까지 포함. native Linux/macOS 배포, 공식 WASM 확인 못함 | 양방향 patch 검증, 충분한 baseline의 관측쌍 선택, factor 구성과 marginalization의 수치 안정성. [구현](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/src/vi_estimator/sqrt_keypoint_vio.cpp), [의존성](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/vcpkg.json) |
| [XRSLAM / RD-VIO](https://github.com/openxrlab/xrslam) | monocular+IMU, rotation-dominant 구간의 R/N subframe·지연 triangulation·rotation constraint. 별도 사전 지도 localization 모듈 | C++17·Eigen/OpenCV/Ceres 계열, Linux/macOS/iOS 실행 문서. iPhone 앱 존재가 Android Chrome 지원 근거는 아님 | 순수회전에서 translation/depth가 관측되지 않는다는 사실을 estimator 구조에 반영. 회전 factor는 metric translation factor 수로 세지 않음. [논문](https://arxiv.org/html/2310.15072v3), [frame 판정](https://github.com/openxrlab/xrslam/blob/4beb1a942f33da9afbfae2d70e2c641cfc2bb675/xrslam/src/xrslam/map/frame.cpp) |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | monocular/stereo/RGB-D, visual/visual-inertial, multi-map Atlas·place recognition·relocalization·map merge. monocular visual-only는 metric scale 관측과 구분 | CPU C++11, OpenCV/Eigen, DBoW2/g2o, Pangolin viewer; ROS 예제는 선택. 공식 browser/WASM 배포 확인 못함 | 현재 bounded window의 tracking recovery와 달리 과거 지도 검색·기하 검증·map lifecycle 제공. vocabulary·비동기 mapping/loop closure·memory budget을 추가해야 하므로 좁은 수정 아님 |
| [Spectacular AI](https://spectacularai.github.io/docs/sdk/core.html) | camera+IMU의 metric VISLAM, Tracking/Mapping/Recording/Replay API. 공개 출력은 tracking status·입력 sensor clock의 pose time·velocity 포함 | native SDK·device wrapper. Python 1.53.2 확인. ARM/custom Input API는 별도 제공 조건, 공개 browser/WASM 배포 확인 못함 | calibrated native 입력의 비교 기준 후보. SDK 내부 solver를 알아냈다는 근거 없음. examples의 Apache2와 proprietary SDK를 구분. [출력](https://spectacularai.github.io/docs/sdk/tracking.html), [wrappers](https://spectacularai.github.io/docs/sdk/wrappers.html), [라이선스 구분](https://github.com/SpectacularAI/sdk-examples) |

- 위 표의 `확인 못함`은 조회 범위의 한계. 구현 불가능·기능 부재의 증명이 아님.
- 현재 프로젝트의 browser IMU sampling·camera calibration·exposure 시각·rolling-shutter readout을 native SDK 입력 조건과 같다고 가정하지 않음.
- 상용 플랫폼의 현재 확인 내용은 [ARKit·ARCore·8th Wall 별도 조사](2026-10-13-commercial-tracking.md)에 유지. 이 문서는 비공개 binary 내부를 추정하여 빈칸을 채우지 않음.

## 버전과 라이선스 고정

| 구현 | 이번 조회 HEAD · committer 날짜 | release 확인 | 최상위 라이선스 |
|---|---|---|---|
| AlvaAR | `7796af500ee92001ac2a9888363ff64d7a3bee75` · 2023-07-03 | GitHub release 없음 | GPLv3 |
| OpenVINS | `69488123ed9362dd44b6f28e7f4680abbff1442b` · 2025-11-30 | v2.7 · 2023-06-20 | GPLv3 |
| sqrtVINS | `30fafc89d89cfb896d06fdaa68fbda8545e02e9f` · 2025-10-30 | GitHub release 없음 | LGPLv3 |
| Basalt | `0f3b2b52c807f70ff4e2973ce253c73329eea7bc` · 2026-03-22 | **GitLab 0.1.7 · 2026-03-21**; GitHub는 mirror | BSD3 |
| XRSLAM | `4beb1a942f33da9afbfae2d70e2c641cfc2bb675` · 2026-02-10 | v0.6.0 · 2024-01-15 | Apache2; [추가 method/dependency](https://github.com/openxrlab/xrslam/blob/4beb1a942f33da9afbfae2d70e2c641cfc2bb675/docs/en/additional_licenses.md) 별도 |
| ORB-SLAM3 | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` · 2022-02-10 | v1.0-release · 2021-12-22 | GPLv3 |
| Spectacular SDK examples | `df74f5ea91a85b7c622b9c40b30214c8185817a2` · 2026-08-06 | [PyPI SDK 1.53.2](https://pypi.org/project/spectacularai/1.53.2/) · 2026-08-06 | examples Apache2, SDK proprietary/non-commercial 배포 조건 |
| VINS-Mono | `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d` · 2024-05-23 | GitHub release 없음 | GPLv3 |
| LEVIO | `00d925f166bff859496fbda85049b2ce68bf7aa1` · 2026-03-26 | CITATION.cff v1.0.0 · 2026-02-01 | MIT |

- release·라이선스 원문은 manifest의 공식 API/고정 commit URL로 추적. 저장소 날짜·별 수·README의 최고 성능 문구를 품질 순위로 사용하지 않음.
- 실제 코드 도입 시 해당 파일 header·하위 dependency·model 권리가 추가로 적용됨. 이번 작업은 제품 소스 복사·링크 없음.

## 최근 후보의 실질적인 의미

| 후보 | 공식 원문에서 확인한 근거 | 이번 채택 판단 |
|---|---|---|
| [sqrtVINS, TRO 2025](https://arxiv.org/html/2510.10346v1) | LLT 기반 SRF, 0.1s window dynamic initialization 실험. Table VII/Fig.12의 runtime은 **feature tracking 제외 estimator 시간**. initialization 비교는 i7-11800H·Jetson Nano single thread | 저비용 backend 연구의 우선 비교 후보. frontend 포함 Flip6 end-to-end 속도나 모든 trajectory 정확도가 2배 우수하다는 뜻 아님. 현재 Ceres 파라미터만 바꿔 재현 불가 |
| [LEVIO, Sensors Journal 2026](https://arxiv.org/html/2602.03294v1), [공식 구현](https://github.com/ETH-PBL/levio) | Python reference와 GAP9 multi-core RISC-V용 C 구현. EuRoC MH01–MH05 monocular+IMU 평가, 저자 보고 20FPS·100mW 미만. README의 working memory 110KB 미만은 해당 설계/하드웨어 범위 | 작은 fixed budget과 keyframe 수명 설계 참고. GAP9 kernel·parallel runtime·입력 해상도 조건을 Chrome/WASM 성능으로 전용하지 않음. 제품 전체 교체보다 자원 상한 검증 후보. 유지보수 README는 적극적 feature 개발 계획 없음 명시 |
| [VkVIO, 2026-09-24](https://arxiv.org/html/2609.30459v1) | Basalt의 CPU backend를 두고 pyramid/detection/tracking을 Vulkan GPU로 이동. §IV-C i5-13600+RTX3070 EMH02 frontend 평균0.63ms, Basalt4.83ms; 둘의 backend 약3.1ms. online causal 출력으로 평가 | frontend/backend 경계와 tail latency 실험 설계 참고. native Vulkan은 browser WebGPU와 다른 API·shader·동기화 조건. 이 조회에서는 공식 code/release/license 링크를 확인 못하여 구현 채택 보류 |
| [ASPIRE-VINS, RA-L 2026](https://arxiv.org/html/2608.12840v1) | adaptive knot·multi-resolution continuous-time spline·3D bearing residual. 9개 VIO sequence 평균 ATE0.212m, 논문 비교군 OKVIS-CT0.224m. controlled-backend ablation은 동일 track으로 따로 평가 | timestamp별 residual과 motion에 따른 시간 해상도 조절의 장기 참고. 최신 논문 수치를 동일 Flip6 입력 성능으로 사용하지 않음. 공개 code/commit/license 확인 못하여 재현 가능 후보 순위에 포함하지 않음 |

- 신규 논문 성능은 저자의 해당 입력·하드웨어·평가 방식 결과. 이 세션의 재실행·에너지 측정·GT 검증이 아님.
- sqrtVINS·LEVIO는 실제 공개 구현 존재, VkVIO·ASPIRE-VINS는 이번 조회에서 공식 논문까지 확인. 최신성보다 재현 가능한 소스·입력 계약을 우선.

## timing·rolling shutter·metric scale

- [VINS-Mono 공식 요구](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/README.md): image>20Hz, IMU>100Hz, 정확한 timestamp·가속도 gravity 포함. `estimate_td`와 `rolling_shutter_tr`는 별도 설정. readout은 exposure time과 다름. 현재 Flip6 DeviceMotion 기록의 약42.5Hz를 동일 전제로 취급하지 않음.
- [VINS-Mono ProjectionTdFactor](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/factor/projection_td_factor.cpp): feature velocity·time offset·row/readout 보정 경로 존재. 미측정 readout을 임의 상수로 넣는 것은 개선 근거가 아님.
- [OpenVINS calibration](https://docs.openvins.com/gs-calibration.html): calibration·camera/IMU time offset의 오차는 동적 궤적에서 성능을 빠르게 악화. metric scale은 IMU 입력과 운동의 관측가능성을 전제. VIO라는 이름만으로 browser callback clock이 물리 exposure clock과 일치하지 않음.
- AlvaAR·Basalt·XRSLAM·ORB-SLAM3·sqrtVINS·LEVIO의 이번 확인 범위에서 rolling-shutter 보정 지원을 확정하지 않음. calibration spline 존재와 online rolling-shutter residual을 구분.
- [Spectacular 출력 계약](https://spectacularai.github.io/docs/sdk/tracking.html)은 입력 sensor clock의 timestamp·metric position·status를 제공. 출력/입력 시각의 명시적 계약을 참고할 수 있으나 SDK 내부 보정 방식은 미확인.

## 현재 목적에서 제외하거나 뒤로 미룬 엔진

| 구현·확인 revision | 제한·제외 사유 |
|---|---|
| [DROID-SLAM](https://github.com/princeton-vl/DROID-SLAM/tree/2dfd39f0dcad44012ca7bbb8aa70b55edbfa9c99), 2025-05-05, BSD3 | 공식 README inference GPU memory 최소11GB·CUDA/PyTorch 요구. monocular/stereo/RGB-D visual 경로, 이 조회에서 IMU fusion 확인 못함. desktop offline 대응 분석은 가능하나 mobile WASM 대체 후보에서 제외 |
| [DPVO/DPV-SLAM](https://github.com/princeton-vl/DPVO/tree/0ac95b656d1fda91c271d2a106460d19ad966fc7), 2026-09-20, MIT | CUDA11/12·PyTorch·learned patch tracker, 선택적 큰 loop용 DBoW2/retrieval. iPhone 영상 demo는 PC에서 영상 처리. Chrome phone 실시간 동작·metric IMU 입력 근거로 사용하지 않음 |
| [MASt3R-SLAM](https://github.com/rmurai0610/MASt3R-SLAM/tree/e6f4e3d474fad0e11f561482012be864ba8c3f17), 2025-11-09 | CUDA/PyTorch·MASt3R 모델, dense reconstruction prior. code CC BY-NC-SA4.0, checkpoint 별도 notice. learned metric prior는 calibrated IMU scale 검증과 다름. 현재 CPU/WASM·제품 도입 후보에서 제외 |
| MindAR·image-target AR | 알려진 target의 pose를 추정하는 입력 조건. arbitrary scene의 metric visual-inertial SLAM과 다른 문제. 기존 [웹 조사](2026-10-12-web-engines.md)에 유지 |

## 이번 세션에 적용 가능한 작은 후보

### 1. 회전 보상 keyframe 지표를 먼저 측정

- 확인한 현재 코드: [addFeatureAndCheckParallax](../../src/frontend/feature_manager.cpp), `compensatedParallax2()`는 `p_i_comp=p_i`; raw/compensated 항이 같다. 이름과 달리 회전 제거가 없음.
- 독립 계산: optical center 고정·camera Y축15° 회전·fx460에서 raw displacement **123.2566px**, 회전 보상 bearing 각 **0rad**, translation baseline0m. [analytic probe](../../build/flip6-analysis-20261013/online-research/pure-rotation-keyframe-probe.json). 실제 엔진 실행·Flip6 인과 증명 아님.
- RD-VIO의 회전/translation 구분을 참고하여 `rawParallax`, `rotationCompensatedAngle`, `matureCurrentSupport`, `keyframeDecisionReason`을 함께 기록하는 작은 변경 권고. `R_WC_i b_i`와 `R_WC_j b_j`의 각을 사용하며 camera/body/extrinsic 방향 검증.
- rotation 보상만으로 keyframe을 억제하지 않음. FOV 변화·새 texture·성숙 track 소실 때문에 순수회전에서도 새 keyframe이 필요할 수 있음. keyframe 필요성과 depth 활성화 가능성을 분리.
- 검증: analytic pure rotation/translation/mixed motion·wrong gyro/extrinsic control, OLD/NEW slide 후 관측 연결·prior slot, fixed input baseline decision parity. 우선 진단만 추가하면 기존 동작 보존 가능; 정책 변경은 별도 opt-in AB·GT gate 필요.

### 2. global 복구 전에 local correspondence의 신뢰도와 가림을 구분

- AlvaAR의 map projection과 ORB-SLAM3의 지도 후보→기하 검증은 유용하지만 현재 회복 시도와 중복 구현하지 않음.
- 기존 [gyro patch warp](../fixes/2026-10-13-frontend-recovery.md)는 실제 가려진 ID2774를 reverse/F 통과 후 오채택. forward/backward 오차와 F inlier만으로 동일 landmark라고 인증할 수 없다는 반례 보존.
- 작은 후속 후보: mature map point의 투영 불확실성·영상 경계·candidate ambiguity를 기록하고, 재매칭 track을 검증 전 일반 metric factor로 집계하지 않는 계약. occlusion·반복 texture·wrong prior에서 false recovery0을 먼저 확인.
- 이 단계 통과 전 descriptor bank·global vocabulary·loop closure를 추가하면 잘못된 ID 연결을 더 오래 유지할 수 있음. 현재 bank 지원 부족을 global SLAM 추가만으로 해결한다고 주장하지 않음.

### 3. low-cost backend는 독립 evaluator에서 비교

- OpenVINS [FEJ 유도](https://docs.openvins.com/fej.html): global translation3DoF+yaw1DoF의 비관측 방향에 허위 정보가 축적되지 않도록 선형화 계약 필요. sqrtVINS의 square-root 구조는 단순 precision 교체와 다름.
- 이번에 가능한 작은 검증: 현재 prior·marginalization의 gauge/nullspace·rank·condition을 독립 analytic fixture로 검사하고, depth가 관측되지 않는 구간의 factor 수와 구분하여 보고.
- 큰 후속 작업: 동일 recorded images/IMU에 OpenVINS/sqrtVINS를 native comparator로 연결. estimator runtime뿐 아니라 frontend·전체 처리·fresh coverage·reset·SE(3) scale1 ATE/RPE·rotation·association을 같은 분모로 평가.
- `sqrtVINS`·LEVIO·VkVIO의 hardware/architecture 교체는 이번 작은 수정 범위를 넘음. 이미 실패한 [D0/D2/D3·H4·marginalization 실험](../fixes/2026-10-13-tracking-performance-results.md)을 새 이름으로 재채택하지 않음.

## 검증과 미확인 사항

- 원문 SHA·문서 상대 경로·독립 기하 계산 확인. 결과: [verification.json](../../build/flip6-analysis-20261013/online-research/verification.json).
- 제품 코드 변경0, 새 dependency0, upstream 빌드0, 전화 측정0. 성능·정확도 표는 공식 원문 또는 명시된 기존 local 결과만 사용.
- 미확인: 현 Flip6 입력에서 각 대체 엔진의 실행 성공·SE(3) 정확도·thermal·battery·rolling shutter 보정, global relocalization, browser SIMD/pthread/GPU 성능.
- 가장 작은 다음 실행: 회전 보상 keyframe 진단과 기존 기하 회귀 확인 → 실제 소실 구간의 ID/age/current factor와 대조 → 단일 정책 후보만 opt-in AB. 실패 threshold·fixed denominator·independent GT를 유지.
