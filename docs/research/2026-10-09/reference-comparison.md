# 모바일 웹 VIO·SLAM 레퍼런스 비교

- 조회일: **2026-10-09**, Asia/Seoul
- 범위: 공개 소스·논문·공식 API 문서, 현재 작업트리의 native/browser 입력·추정 경로
- 판정: **공통 C++ 엔진만으로 PC와 모바일 입력 동등성 확보 불가**. 촬영 시각, 센서 시간축, calibration, 픽셀 변환, 실행 예산의 별도 검증 필요
- 현재 제품 경로: sliding-window VIO. loop closure·장기 지도 재사용·relocalization을 포함한 전체 SLAM 기능과 구분
- 근거 보관: [`build/mobile-web-20261009/research/`](../../../build/mobile-web-20261009/research/), 외부 서비스로 프로젝트 코드·영상·센서 기록 전송 없음
- 이 문서의 합성 실험표: 검증 설계·권고의 예시. 실제 seed·rate·수락 조건은 [확정 PRD](../../../.omx/plans/prd-mobile-web-system-20261009.md)·[확정 test-spec](../../../.omx/plans/test-spec-mobile-web-system-20261009.md)의 `Architect revision1` 우선. 실행 완료·실기기 정확도 주장 제외; 최종 채택·실행 결과는 통합 검증 보고서 기준

## 1. PC와 모바일의 실제 차이

| 항목 | PC 데이터셋 경로 | 모바일 웹 경로 | 판단 |
|---|---|---|---|
| 추정 엔진 | `VIOSystem::onFrameProcessed()` → `VIOEngine::processFrame()` | JS → Worker → WASM `VIOEngine::processFrame()` | 현 소스의 엔진 공유 확인. 과거 문서의 독립 알고리즘 설명을 현재 사실로 재사용 금지 |
| calibration | `configureFromConfig()`의 YAML·camera factory | FOV/focal 추정, orientation 표, 기본 2cm lever arm, 왜곡 0 가정; 작업트리에 명시 profile 경로 존재 | 추정값·제공 profile·독립 검증 완료를 구분 |
| 실행 설정 | `configureFromConfig()`에서 YAML 복원 | `configure()` 기본값 후 `setMobileParams()`·`setTrackingParams()` 적용 | min-distance·solver budget·PnP·noise가 같아야 수치 비교 가능 |
| 영상 시각 | 파일의 원래 image timestamp | video callback의 `captureTime`/`presentationTime` 등 실제 제공 필드 | presentation 시각과 광학 exposure 시각 동일성 보장 없음 |
| IMU | 개별 센서 timestamp와 metric CSV | Generic Sensor 또는 DeviceMotion, 비동기 callback·양자화·제한된 실제 전달률 | 요청 Hz, 실제 Hz, 센서 Hz 구분 |
| 영상 변환 | dataset 모델·해상도 | browser orientation·명시 회전·crop·resize·grayscale | K·왜곡·camera/body 외부정합을 최종 픽셀 영역으로 전달 |
| IMU frame 경계 | 파일에서 미래 bracket 확보 가능 | 실시간 도착 대기·queue·drop 정책 필요 | 같은 입력 파일 재생 성공으로 실제 live scheduling 검증 대체 불가 |
| 결과 표시 | 저장 궤적 및 evaluator | Worker 응답·렌더링·guard·reset·세션 세대 | pose timestamp·pose age·reset epoch를 별도 비교 |

- 소스 근거: [`src/vio_system.cpp`](../../../src/vio_system.cpp), [`src/vio_engine.cpp`](../../../src/vio_engine.cpp), [`web/js/app.js`](../../../web/js/app.js), [`web/js/camera.js`](../../../web/js/camera.js), [`web/js/imu.js`](../../../web/js/imu.js), [`web/js/vio-worker.js`](../../../web/js/vio-worker.js), [`web/js/test-tumvi-app.js`](../../../web/js/test-tumvi-app.js)
- 조사 시점 commit: `f2105709e4e9a4d8006f5a78e1f1fe9e989a6072`; 미커밋 변경 포함. [조사 시점 파일 SHA-256](../../../build/mobile-web-20261009/research/local-inspection-snapshot.json), 병행 수정 후 최종 빌드 fingerprint와 구분
- 보존 Z Flip 6 기록: [2026-10-07 보고서](../../fixes/2026-10-07-zflip6-followup.md)의 native/Worker 설정·결과 차이 및 후반 영상 누락. 해당 역사적 수치를 신규 소스 결과로 재명명 금지

## 2. 공개 알고리즘 비교

| 레퍼런스 | 공개 구조·입력 가정 | 프로젝트 차이·채택 판단 |
|---|---|---|
| [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/tree/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d), [논문 v1](https://arxiv.org/abs/1708.03852v1) | tightly coupled sliding-window 최적화, IMU preintegration/bias, 초기화, failure recovery, loop detection·4-DoF pose graph. 보정된 카메라·정확한 시각의 IMU 전제 | 현재 엔진의 가장 가까운 비교 대상. 현 optimizer의 camera/body extrinsic은 constant; loop graph·relocalization 경로 미확인 |
| [VINS-Mono 시간 보정](https://arxiv.org/abs/1808.00692v1), [공개 projection factor](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/factor/projection_td_factor.cpp) | time offset과 상태·landmark 공동 추정, feature velocity를 이용한 관측 시각 보정. 소스에 row/readout 보정 경로 | 현 `ProjectionFactor`에는 time-offset/readout 상태 부재. clock 정규화만으로 물리적 camera/IMU offset 해결 주장 불가 |
| [VINS-Mobile](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile/tree/ab1c4ffb8d3ea1fcc01c4f5c651abc0c390807f9) | iOS native 예제, motion-only frontend로 pose/render 30Hz 공개. README의 테스트 환경은 iPhone 7 Plus·iOS 10.2.1 | 과거 iPhone 설정·solver budget을 Z Flip 6/browser의 보정·성능 근거로 사용 불가. 고속 출력과 backend 최적화 주기 분리 구조는 참고 가능 |
| [VINS-Fusion](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/tree/be55a937a57436548ddfb1bd324bc1e9a9e828e0) | mono+IMU, stereo+IMU, stereo; online spatial/temporal calibration, loop closure, 별도 GPS fusion 예제 | browser의 단안 입력을 stereo 시스템 수치와 직접 비교 불가. 현재 고정 extrinsic·시각 모델 보완에 참고 |
| [OpenVINS](https://github.com/rpng/open_vins/tree/69488123ed9362dd44b6f28e7f4680abbff1442b), [ICRA 2020 저자 PDF](https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf) | sliding-window MSCKF 계열, covariance·FEJ·다양한 calibration 상태·시뮬레이션 지원 | Ceres optimizer를 단순 치환하는 대상 아님. 관측가능성·consistency·독립 시뮬레이터 검증 방법 참고 |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3/tree/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4), [논문 v2](https://arxiv.org/abs/2007.11898v2) | visual/visual-inertial·multi-map, monocular/stereo/RGB-D, place recognition·map merge | 장기 지도·재방문 보정 목표의 비교 기준. 브라우저 실시간 예산·메모리·카메라 보정이 해결된 것으로 간주 불가 |

- VINS-Mono README의 이미지 `>20Hz`, IMU `>100Hz`는 upstream 권장 입력 조건. 모바일 웹의 60Hz 전달률을 자동 합격시키거나 절대 불가능으로 단정하는 임계값으로 전용 금지. [원문](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/README.md#5-run-with-your-device)
- OpenVINS FEJ: global translation 3축·yaw 1축의 비관측 방향에 허위 정보 축적 방지. 현 최적화계에 그대로 붙이기보다 gauge·marginalization prior consistency 검증부터 적용. [공식 유도](https://docs.openvins.com/fej.html)
- 충분한 움직임 없는 online calibration은 해결책이 아님. OpenVINS의 calibration 가이드는 translation과 복수 회전축의 excitation 필요성을 명시. [공식 가이드](https://docs.openvins.com/gs-calibration.html)

### 소스 버전·라이선스

| 저장소 | 조사 commit | 확인 파일·표기 |
|---|---|---|
| VINS-Mono | `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d` | [LICENCE](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/LICENCE), GPL v3 |
| VINS-Mobile | `ab1c4ffb8d3ea1fcc01c4f5c651abc0c390807f9` | [LICENSE](https://github.com/HKUST-Aerial-Robotics/VINS-Mobile/blob/ab1c4ffb8d3ea1fcc01c4f5c651abc0c390807f9/LICENSE), GPL v3 |
| VINS-Fusion | `be55a937a57436548ddfb1bd324bc1e9a9e828e0` | [LICENCE](https://github.com/HKUST-Aerial-Robotics/VINS-Fusion/blob/be55a937a57436548ddfb1bd324bc1e9a9e828e0/LICENCE), GPL v3 |
| OpenVINS | `69488123ed9362dd44b6f28e7f4680abbff1442b` | [LICENSE](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/LICENSE), GPL v3 |
| ORB-SLAM3 | `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4` | [LICENSE](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/LICENSE), GPL v3 |

- 위 표: 각 저장소 최상위 코드 라이선스. 제3자 library·vocabulary·가중치·dataset 전체 사용권으로 확대 금지
- 이번 조사에서 제품 runtime 의존성 추가 없음. 원문·LICENSE·SHA-256: [GitHub manifest](../../../build/mobile-web-20261009/research/github-sources.json)
- 최초 잘못된 대소문자 `OpenVINS/README.md`, `readme.md` 및 `VINS-Mobile/LICENCE` 요청의 HTTP 404 보존. Git tree 확인 후 `ReadMe.md`, `LICENSE`로 재조회 성공

## 3. ARKit·ARCore·상용 모바일/웹 SDK

| 대상 | 공식적으로 확인 가능한 기능 | 확인 불가·현재 프로젝트 적용 판단 |
|---|---|---|
| [ARKit world tracking](https://developer.apple.com/documentation/arkit/understanding-world-tracking), [tracking lifecycle](https://developer.apple.com/documentation/arkit/managing-session-life-cycle-and-tracking-quality) | camera+motion VIO, tracking state/reason, plane/raycast, 세션·world-map 복원 절차. [ARCamera.intrinsics](https://developer.apple.com/documentation/arkit/arcamera/intrinsics) 제공 | 비공개 optimizer·sensor fusion·calibration 알고리즘 추정 금지. API 기능과 독립 위치 정확도 별도 |
| [ARCore fundamentals](https://developers.google.com/ar/develop/fundamentals), [CameraIntrinsics](https://developers.google.com/ar/reference/java/com/google/ar/core/CameraIntrinsics) | visual feature+IMU pose 추정, plane·depth 등 기능; SDK camera calibration API | Z Flip 6에서 실제 호출·정확도 검증 없음. 내장 보정 절차·잔차·비선형 solver의 상세 공개 여부와 구분 |
| [8th Wall 배포 구조](https://8thwall.org/docs/open-source) | 2026 공식 문서: SLAM 포함 Distributed Engine Binary, 별도 binary license. MIT framework는 Face/Image/Sky 등이며 SLAM 제외 | 오픈소스 framework 분석으로 SLAM 내부를 확인했다고 주장 불가. 공개 [world tracking 제약](https://8thwall.org/docs/troubleshooting/world-tracking-issues)은 grayscale feature와 단일 ground-plane 기반 설명 |
| [Zappar Instant World Tracking](https://docs.zap.works/universal-ar/a-frame/tracking/instant-world-tracking/), [WebAR 설명](https://docs.zap.works/studio/getting-started/instant-world-tracking/) | 사용자가 선택한 주변 지점에 콘텐츠 고정; WebAR 경로는 ARKit/ARCore에 의존하지 않는다고 명시 | 웹 추적 가능성의 사례. 공개 기능만으로 metric scale·거리 오차·loop closure 품질을 입증하지 못함 |
| [Spectacular AI SDK](https://spectacularai.github.io/docs/sdk/), [calibration](https://spectacularai.github.io/docs/sdk/tools/calibration.html) | camera+IMU 실시간 pose/velocity; VISLAM의 loop closure로 VIO drift 보정. custom 장치의 intrinsic·stereo·camera/IMU calibration 도구 | 지원 장치 wrapper와 임의 모바일 browser 입력 구분. 공개 문서가 보장하지 않는 내부 구현·Z Flip 성능 단정 금지 |
| [WebXR Device API](https://immersive-web.github.io/webxr/) | `XRFrame`/`XRPose`/reference space의 추적 결과 접근; `emulatedPosition` 상태 | WebXR 자체가 JS로 구현한 VIO 엔진은 아님. `isSessionSupported('immersive-ar')` 런타임 확인 필요. engine 전환 시 pose source·좌표·reset·GT 평가를 별도 표시 |

- 제품 판단: **정확한 AR pose가 우선이고 해당 브라우저가 지원하면 native-backed WebXR 경로가 비교 후보**. 사용자 요구인 자체 웹 VIO 검증은 별도로 유지
- native SDK 사용도 정확도 자동 보증 아님. 실제 hardware·browser 지원, 자산 고정·장기 drift·재지역화 이후 불연속, 독립 GT 확인 필요
- 상용 숫자의 프로젝트 수락 임계값 전용·공개 demo 영상의 정량 정확도 근거 사용 제외

## 4. 웹 입력에서 숨길 수 없는 제약

| 계약 | 공식 사실 | 구현·검증 요구 |
|---|---|---|
| DeviceMotion 의미 | raw sensor API가 아닌 high-level 정보. device frame은 보통 표준 portrait 기준이며 screen 회전으로 변경되지 않음 | screen 회전과 IMU frame 회전을 중복 적용 금지. x/y/z 단축 회전 및 화면 방향별 가상 검증. [W3C §1–3](https://www.w3.org/TR/2025/CRD-orientation-event-20250212/) |
| 단위·정밀도 | acceleration m/s², rotation deg/s. CRD 2025-02-12 §6.3에서 acceleration 0.1m/s², rotation 0.1deg/s보다 세밀하지 않도록 제한 | deg→rad 정확히 1회. 0.1deg/s ≈0.001745329rad/s 양자화 주입. 동일 숫자 반복만으로 센서 정지 판정 금지. [W3C §6.3](https://www.w3.org/TR/2025/CRD-orientation-event-20250212/#devicemotion) |
| 이벤트 시각 | `Event.timeStamp`는 이벤트 생성 시점의 문서 time origin 상대 시각 | IMU hardware acquisition 시각으로 자동 승격 금지. 수집 API·source/arrival 시각·offset 불확실성 기록. [DOM Event](https://dom.spec.whatwg.org/#dom-event-timestamp) |
| Generic Sensor Hz | sampling/reporting frequency 구분; 실제 hardware rate는 opaque, 요청 frequency와 실제 delivery가 다를 수 있음 | 실제 interarrival·sample spacing·drop·accel/gyro skew 측정. `new Gyroscope({frequency:200})`만으로 200Hz 주장 금지. [공식 §5.4](https://www.w3.org/TR/generic-sensor/#sampling-frequency-and-reporting-frequency) |
| 영상 시각 | `presentationTime`은 compositor 제출 시각, `mediaTime`은 미디어 timeline. `captureTime`은 optional | callback 도착 시각·presentation·capture를 별도 보존. optional 값 부재 시 제한 표시; 물리 offset 검증 필요. [rVFC specification](https://wicg.github.io/video-rvfc/#dom-videoframecallbackmetadata) |
| K·왜곡·extrinsic | 기본 Media Capture 표준의 video 설정만으로 camera/IMU calibration 전체 미제공 | camera label/FOV/해상도를 calibration 대체값으로 사용 금지. profile의 실제 장치·렌즈·orientation·pixelFrame 일치 검사. [Media Capture](https://www.w3.org/TR/mediacapture-streams/#mediatracksettings) |
| native metadata | Camera2는 exposure 시작 `SENSOR_TIMESTAMP`, rolling-shutter skew 등 제공; timestamp source `UNKNOWN`이면 타 subsystem과 비교 불보장 | native 경로도 지원 metadata·clock 확인 필요. 웹 API에 동일 값이 있다고 가정 금지. [Camera2 CaptureResult](https://developer.android.com/reference/android/hardware/camera2/CaptureResult#SENSOR_TIMESTAMP), [timestamp source](https://developer.android.com/reference/android/hardware/camera2/CameraCharacteristics#SENSOR_INFO_TIMESTAMP_SOURCE) |

- 고정 숫자 시퀀스의 가능한 원인: 실제 정지·양자화·OS 필터·이벤트 재사용·센서 정지. timestamp와 별도 시각 관측 없이 단일 원인 단정 불가
- OIS/EIS·초점·자동 lens 전환·ISP crop 가능성: 실제 metadata 또는 실측 calibration 변화로 확인할 항목. 현재 Z Flip 6 원인으로 확정한 사실 아님

## 5. 수학 계약과 독립 검증

- 표기: `R_WB`는 body→world, `R_BC`는 camera→body, `p_BC`는 body 원점에서 camera 원점까지 body 표현, 물리 중력 `g_W=(0,0,-9.81)`
- 소스의 양의 `g=(0,0,+9.81)`와 물리 중력 `g_W`의 부호 구분. IMU specific force를 world로 회전한 뒤 소스의 `g` 차감

| 항목 | 수식·불변량 | 독립 검증 방법 |
|---|---|---|
| IMU 생성 | `f_B=R_WBᵀ(a_W−g_W)+b_a+n_a`, `ω_B=vee(R_WBᵀ dR_WB/dt)+b_g+n_g` | analytic trajectory로 image·IMU·GT 각각 생성. estimator propagation을 GT 생성기로 재사용 금지 |
| 단위·부호 | `ω_rad=ω_deg π/180`; 정지 시 `‖f_B−b_a‖≈g` | 각 축 ±90° 회전, 자유낙하/정지/일정가속도 사례. norm만 맞고 축이 틀린 경우 포함 |
| camera/body | `R_WC=R_WB R_BC`, `p_WC=p_WB+R_WB p_BC` | nonzero lever arm·순수회전에서 camera translation과 body translation 구분 |
| 영상 회전 | `C'=Q C` → `R_BC'=R_BC Qᵀ`; `p_BC` 불변 | none/cw/ccw/180°의 독립 landmark projection. 회전 후 tangential distortion 방향 포함 |
| crop/resize | `f'_x=s_x f_x`, `c'_x=s_x(c_x−crop_x+0.5)−0.5`, y도 동일; 90°에서 축 교환 | 최종 정수 해상도의 실제 `s_x,s_y` 사용. 중심이 아닌 principal point·비등방 resize 포함 |
| frame 경계 | `t_left≤t_frame≤t_right`, `ΣΔt=t_frame(k)−t_frame(k−1)` | 원본 right bracket 1회 보존, endpoint interpolation, duplicate/out-of-order·큰 gap·late packet 테스트 |
| bias·preintegration | 초기화/재적분 bias 기준 동일성, 실패 후보의 상태 불변 | analytic 정지·bias·rotation 입력; 실패 전후 state/preintegration snapshot 비교. [Forster v3](https://arxiv.org/abs/1512.02363v3) |
| projection | 최종 metric landmark는 `ρ>0`·anchor/target cheirality 필요. optimizer의 signed/zero `ρ` trial은 별도 수학적 정의역 | 독립 projection·중심 차분 Jacobian·`ρ=0` projective limit 검증. 유한한 signed trial의 연속 평가와 최종 per-feature cull 구분; 단일 negative trial을 전체 solve 거부 조건으로 일반화 금지 |
| 관측가능성 | global translation/yaw gauge; metric scale의 운동 조건 의존 | 정지·순수회전·직선 일정속도·충분한 translation/rotation 분리. 퇴화 입력에서 강제 초기화 금지 |
| marginalization | linearization point·parameter-block identity·prior PSD·gauge 일관성 | 동일 작은 문제의 batch optimum과 window 결과 비교, prior 재정렬·reset·실패 rollback 검사 |
| time offset | `t_imu=t_cam+Δt`; `Δu≈f ω Δt`는 작은 회전의 1차 근사 | `Δt` 부호를 고정하고 ±5/15/30ms sweep. `Δt=0` control과 보정 ablation 비교 |
| rolling shutter | 예시 convention: `t_row=t_first+(row/H)τ_readout+τ_exposure/2` | timestamp가 첫 row exposure 시작인 경우만 해당. sensor row→회전/crop 후 row 매핑, 0/10/20ms 가상 readout 비교 |

- 시간 오차 민감도 예시: `f=300px`, `ω=1rad/s`, `Δt=30ms` → 약 `9px`; 실제 Z Flip 6 추정치 아닌 1차 해석 예시
- 가속도 bias 예시: visual correction 없는 단순 적분에서 `|δp|≈0.5|b_a|t²`; `b_a=0.1m/s²`, `t=10s` → `5m`. filter·VIO 전체 오차의 확정 예측 아님
- 관측성 유도·증거: [OpenVINS FEJ](https://docs.openvins.com/fej.html), [calibration motion](https://docs.openvins.com/gs-calibration.html). true projection singularity·비유한 값 검사와 최종 물리 유효성 검사를 구분; 어느 쪽도 정확도의 충분조건이 아님
- 현 소스 확인: [`Optimizer::setupOptimizationProblem()`](../../../src/backend/optimizer.cpp)의 extrinsic constant, [`ProjectionFactor::Evaluate()`](../../../src/backend/factor/projection_factor.cpp)의 단일 시각 관측. online `Δt`, row-time, loop-graph 상태 부재
- trial/final 분리의 upstream 근거: [VINS-Mono projection factor](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/factor/projection_factor.cpp)의 signed inverse-depth 부호 거부 부재, [FeatureManager::setDepth/removeFailures](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/feature_manager.cpp)의 최종 negative-depth 개별 제거
- 현재 구조 차이와 재현 근거: [별도 수학 감사](../../../build/mobile-web-20261009/math-audit/README.md)의 negative-trial 거부·단일 최종 invalid depth에 의한 전체 결과 거부. 이전 기록의 반복 trial 실패와 연결되지만 Z Flip 6 발산의 단일 물리 원인 확정은 아님
- 검토 후보: homogeneous projection으로 `ρ=0`까지 유한하게 연장 후 최종 유효 landmark만 유지. upstream의 직접 `1/ρ` 식은 `ρ=0` 특이점 보유; 원문 단순 복사와 연속 연장 구분. 독립 Jacobian·현재 frame support·prior ID/address·map publication·calibrated 회귀 검증 후 채택 판단

## 6. 수정 우선순위·기각 기준

| 순서 | 수정/검증 대상 | 판단·수락 조건 |
|---|---|---|
| P0 | source·artifact·runtime configuration fingerprint, 동일 입력 native/WASM 비교 | 동일 gray bytes·IMU·timestamp·K·extrinsic·noise·tracker·solver profile·seed 고정. 실행파일 동일성 없는 성능/정확도 비교 기각 |
| P0 | 픽셀·K·extrinsic·왜곡 동시 변환 | 변환 전후 동일 3D landmark ray와 projection 일치. 단순 화면 orientation 표만으로 calibrated 판정 기각 |
| P0 | 실제 timestamp·IMU bracket·queue·세션 상태 | source timestamp와 arrival 분리, boundary 두 번 적분/누락 방지, stale result 거부. worker 복사·queue·backend 시간 별도 계측 |
| P0 | initialization·bias·solver/prior 실패의 원자성 | 실패 후보의 상태 오염 없음, 정상 baseline pose coverage·오차 악화 없음. solver warning만으로 무조건 prior 폐기 기각 |
| P0 | optimizer trial domain·최종 landmark 유효성 분리 | signed/zero trial의 정의된 projective residual·Jacobian 검증, 최종 invalid landmark 개별 제거. 모든 현재 관측이 제거된 pose를 visual correction 성공으로 수락 금지 |
| P0 | mobile calibration provenance 및 상태 표시 | heuristic·제공 profile·독립 검증을 구분. invalid/uncalibrated 상태를 metric 검증 완료로 표시하지 않음 |
| P1 | camera/IMU time offset | 먼저 offline known-offset 합성 검증. 충분한 excitation·조건수·불확실성·bounds·동일입력 ablation 후 online 추정 검토 |
| P1 | rolling shutter·noise·quantization 모델 | 실측 또는 명시 가상 readout/noise provenance. 무근거 readout 상수·정밀도 복원·bias 강제 0 설정 기각 |
| P1 | 실시간 frontend/backend 예산 | pose freshness와 coverage를 함께 보존. 처리 프레임을 버려 latency만 낮춘 결과·재사용 pose FPS를 새 pose 처리량으로 보고하는 방식 기각 |
| P2 | relocalization·loop closure·map lifecycle | local VIO가 검증된 후 place recognition→geometric verification→pose graph→epoch/relocalization 상태 추가 검토. loop 오검출 음성 사례 필수 |
| 선택 | WebXR/native-backed 추적 비교 | capability 확인·pose source 명시·공통 body frame GT 비교. 자체 WASM 정확도 검증 결과와 혼합 금지 |

- 우선 채택: 확인된 입력 변환·상태 관리 결함의 좁은 수정, 독립 회귀, 재현 가능한 profile 계약
- 즉시 대규모 추가 비채택: 준비되지 않은 online calibration·rolling-shutter 보정·loop closure 동시 도입. 현재 실패와 신규 구현 오류의 원인 분리·baseline 유지 필요
- 분포 외 값 숨기기 비채택: pose clamp·시각 smoothing·과도한 reset만으로 실측 정확도 개선 주장 금지

## 7. 실기기 없이 가능한 가상 검증 설계

- 아래 표의 30Hz·60/100/200Hz·offset/readout 숫자는 문헌 조사용 예시이며, 실행한 수락 실험 목록 아님
- 실제 실행 기준: [확정 test-spec](../../../.omx/plans/test-spec-mobile-web-system-20261009.md), clean `20Hz image/200Hz IMU`, mobile-like `20Hz image/40·60Hz IMU`, `20s`, development seed `101/202`, confirmation seed `707`. 해당 spec의 GT·noise·coverage 분모·오차 임계값·회귀 조건 우선
- GT: estimator와 독립인 analytic `SE(3)` 궤적·3D landmark/texture·camera model, 고정 seed
- 출력: image·IMU·camera/body GT·수집 metadata·생성 설정·SHA-256; 결과는 local `build/`에 저장
- 정상 세트와 교란 세트 분리; 노이즈 없는 정상 세트만으로 mobile web 현실성 주장 금지

| 세트 | 가상 조건 | 확인 대상 |
|---|---|---|
| A 정상 | 충분한 6-DoF motion, 알려진 K·distortion·nonzero lever arm, 30Hz image·200Hz IMU | native/WASM 공통 엔진 정확도, 초기화·coverage |
| B 저주파·양자화 | 60/100/200Hz IMU, 0.1m/s²·0.1deg/s 양자화 | 실제 browser 전달 조건에 대한 민감도. sample rate·quantization을 각각 변경 |
| C callback 지연 | sensor timestamp 불변, arrival 지연·순서 교란·CPU stall | 도착 시각이 물리 궤적에 섞이지 않는지, queue/drop 계약 |
| D 센서 시각 오차 | camera/IMU ±5/15/30ms, 일정 offset 및 시간 가변 jitter 분리 | 정규화와 물리 동기 보정의 구분 |
| E 영상 처리 | 4방향·crop·resize·noncentral K·pinhole tangential/KB distortion | 변환된 calibration의 ray/projection 일관성 |
| F 모델 불일치 | fx/fy·principal point·extrinsic 각도/translation 교란 | pose가 finite여도 metric 정확도가 나쁠 수 있는 음성 사례 |
| G 관측 퇴화 | 정지, 순수회전, 작은 baseline, 일정속도 | 초기화 보류·명확한 failure reason, 거짓 scale 확정 방지 |
| H 품질 저하 | blur, feature 부족, 반복 무늬, 움직이는 object | frontend outlier·coverage·recovery, 성공 구간만 남기는 평가 방지 |
| I rolling shutter | global shutter control, 10/20ms readout, exposure 변화 | 현 global-time 모델의 한계 정량화 |
| J lifecycle | hidden→visible, orientation 변경, stop→start, in-flight 응답 | prior·IMU history·camera profile·session epoch 무결성 |
| K 장기 loop | 원점 복귀·재방문·가림 후 재등장 | 현재 VIO drift 보고, 미구현 loop closure를 통과 조건으로 오인 방지 |
| L 관측자 부하 | recording 켬/끔, grayscale 저장·snapshot | recorder·test harness가 실시간 결과에 미치는 영향 |

- 필수 결과: 전체 frame 수, 실제 image update 수, initialized/pose coverage, reset 횟수, pose age, capture/copy/queue/frontend/backend/render p50/p95/p99, 메모리 peak
- 정확도: one-to-one timestamp association, camera→body 변환, `SE(3)` scale=1 위치/회전 ATE·RPE, association coverage·누락 구간
- `Sim(3)`는 scale 진단으로만 별도 보고; 0 pose·0 association·회전 평가 누락은 정확도 통과 제외
- 합성 acceptance는 수치 계약·회귀 검증용. 업무상 거리/각도 허용 오차와 실제 장치 GT 없는 상태에서 production 임계값 임의 선언 금지

## 8. 조사 증거와 남은 한계

- 외부 조사 수단: 웹 검색→공식 원문 확인, GitHub public API로 commit 확정→동일 commit raw source/README/LICENSE 보존
- 원문 목록·URL·조회일·SHA-256: [web-sources.json](../../../build/mobile-web-20261009/research/web-sources.json), [github-sources.json](../../../build/mobile-web-20261009/research/github-sources.json)
- 논문 버전: VINS-Mono `1708.03852v1`, temporal calibration `1808.00692v1`, ORB-SLAM3 `2007.11898v2`, preintegration `1512.02363v3`, OpenVINS ICRA 2020 저자 PDF
- 참조 구현 실행·동일 데이터셋 benchmark는 이번 문헌 조사에서 미수행. paper 결과를 프로젝트 재현 결과로 사용하지 않음
- 상용 SDK 내부 solver·보정·지도 구현은 공개 API 기능 이상의 확인 불가
- 가상 데이터가 검증할 수 없는 항목: Z Flip 6 실측 K/extrinsic·실제 exposure/IMU delay·OIS/EIS·발열·sensor fusion/quantization 구현·실측 이동 GT
- 최종 판단: 입력/수학/시스템 계약의 검증 가능한 수정과 회귀 검증을 먼저 완료. 실제 모바일 정확도는 그 뒤의 독립 현장 검증 항목으로 유지
