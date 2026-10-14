# 엔진 소스·SDK 계약 차이와 재현 경계

- 조사: **2026-10-13 시작, 2026-10-14 종료, Asia/Seoul**. 공개 소스·공식 SDK 계약 비교 lane. 제품 알고리즘 변경·상용 실행·동일 입력 benchmark는 다른 실행 lane의 증거로 판정.
- 답변: **상용 엔진의 제한된 binary 구조 검사는 이미 있었으나, 상용 SLAM 수치 엔진 전체를 복원·실행 검증했다고 답할 근거는 없음.** 공개 framework, client SDK, 실제 service solver, 자체 fixture를 구분해야 함.
- 현 소스 기준: `2789f64534f5f8a5d6b4d65fc616a7bdeeb7613f`. 비교 대상 14파일의 SHA256·수집 시점 dirty 상태: [local-source-manifest.json](../../build/commercial-slam-20261013/research/local-source-manifest.json). served WASM이 이 HEAD 빌드라는 뜻은 아님.
- 이번 직접 수행: Context7 ARCore·ARKit·8th Wall resolve/query → 공식 웹 원문 확인 → pinned 공개 소스·문서 **28파일, 1,901,344bytes** 새 획득. [primary-manifest.json](../../build/commercial-slam-20261013/research/primary-manifest.json).
- 최초 `2109.02182v2` GET404는 [첫 실패 기록](../../build/commercial-slam-20261013/research/fetch-summary-first-attempt.json)에 보존. 실제 submission은 v1만 존재; v1로 수정한 최종 획득 실패0. 문헌의 존재·조회 성공을 코드 실행 성공으로 세지 않음.

## 기존 수행과 누락

| 대상 | 이미 보존된 직접 증거 | 그 증거로 완료할 수 없는 주장 |
|---|---|---|
| 8th Wall 배포 binary | [10월13일 재검사](2026-10-13-commercial-engine-reinspection.md): REA6.4.0·WABT1.0.42·JS AST, embedded WASM exports/imports·SIMD 정적 inventory | private SLAM solver 수식·threshold 완전 복원, source 재빌드, 현 Flip6 영상에서 tracking/ATE 우위 |
| ARCore SDK | 같은 문서의 ELF/readelf/objdump: x86_64 `ArSession_update`13byte forwarding jump; SDK client68,072B와 service ELF47,269,520B 분리 | client disassembly를 VIO estimator 역공학으로 간주; service 전체 의미 복구·runtime tracking 완료 |
| ARKit | 공개 VIO·timestamp·intrinsics·tracking 상태 계약 조사 | Apple runtime/SDK binary 실행·역공학, simulator가 물리 AR tracking 지원한다는 주장 |
| Vuforia | host-contract fixture와 환경 manifest 보존 | fixture CTest를 `libVuforia.so` 호출·Vuforia engine 추적으로 간주 |
| VINS-Mono/OpenVINS/Basalt | pinned source 정적 비교와 기존 lifecycle microprobe | reference 전체 engine 실행·GT benchmark·브라우저 이식 완료. 이번 별도 실행 lane의 결과와 합쳐서 판정 필요 |

- 위 기존 수행은 보존된 명령·manifest를 현재 읽어 확인한 범위. 이 소스 비교 lane에서 해당 binary 명령을 재실행한 것은 아님.
- 설치 목록·source checkout·SDK API 조회·process exit0·tracking pose 발행·독립 GT 정확도는 각각 별도 검증 단계.

## 공개 framework와 비공개 world SLAM

| 공개 근거 | 관측한 계약 | 현재 프로젝트와의 차이·해석 한계 |
|---|---|---|
| [8th Wall 공식 배포 구분](https://8thwall.org/docs/open-source) | MIT framework는 architecture/Face/Image/Sky 포함, world SLAM은 binary에만 포함 | MIT repository에 sensor/GL/pipeline 코드가 있다고 world estimator까지 공개됐다고 해석하지 않음. restored pseudocode를 proprietary 원본 소스로 표시하지 않음 |
| [tracking-controller.ts:439–475](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L439) | repeatFrame이면 stage 생략; videoTime·frameTime·performance.now 기반 시간을 별도 전달 | 자체 rVFC dedup·입력 timing/request association은 이미 존재. `web/js/app.js:1090–1149`, `web/js/vio-wrapper.js:203–226,329–369`. 다른 wrapper 시간값 존재만으로 물리 exposure 동기 방식 복원 불가 |
| [sensors.ts:49–63](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/sensors.ts#L49) | DeviceMotion arrival/event timestamp/interval 별도 저장, 지연된 event queue 정리 | 자체 `web/js/imu.js:45–50,155–167,341–343`은 clock normalization·future rejection·gyro/acc bracket pairing. clock normalization은 camera/IMU 물리 동기 보정과 다름 |
| [ARCore Frame](https://developers.google.com/ar/reference/java/com/google/ar/core/Frame), [Camera](https://developers.google.com/ar/reference/java/com/google/ar/core/Camera) | capture timestamp(ns), Android camera timestamp 별도; frame time base는 명시되지 않음. camera pose는 중앙 행 노출 중앙 시점, TRACKING일 때만 사용 | 자체 C++ `src/vio_engine.cpp:234–246`은 영상 timestamp에 camera pose를 발행. `web/js/app.js:1109–1122`는 capture/presentation/arrival fallback과 `physicalExposureVerified:false`를 기록. native SDK capture contract와 browser proxy를 동일 품질로 비교하면 안 됨 |
| [ARCore UpdateMode](https://developers.google.com/ar/reference/java/com/google/ar/core/Config.UpdateMode) | blocking timeout 또는 최신 frame 모드에서 같은 camera image가 다시 반환될 수 있음 | comparator는 callback 수 대신 unique image timestamp·fresh pose coverage 집계. reference 높은 callback FPS를 자체 처리량보다 우수하다고 바로 판단하지 않음 |
| [ARKit Lifecycle](https://developer.apple.com/documentation/arkit/managing-session-life-cycle-and-tracking-quality), [timestamp](https://developer.apple.com/documentation/arkit/arframe/timestamp), [intrinsics](https://developer.apple.com/documentation/arkit/arcamera/intrinsics) | VIO pose, normal/limited/notAvailable 및 relocalization, captured frame 시각·pixel K 제공 | 자체 JS consumer는 limited/v2 품질을 검사하지만 C++ producer는 `pipeline-v1` 발행(`src/vio_engine.cpp:490`); limited inertial bridge가 제품에 구현·수락되었다고 주장 불가. 기존 map relocalization과 새 epoch reset도 구분 |

- 위 표는 공개 API·MIT boundary에서 확인한 사실. commercial 내부가 특정 Ceres/MSCKF/QR/threshold를 사용한다는 추론 근거는 없음.
- camera K·body extrinsic·crop/resize/rotation의 자체 변환 계약은 이미 `web/js/camera.js:11–28,38–76`에 존재. native SDK K를 다른 browser video stream에 재사용하는 계획은 채택하지 않음.

## 고정 공개 구현과 현재 코드의 실질 차이

| 구현 | 고정 commit·논문 | source에서 확인한 설계 | 현재 코드 대조 |
|---|---|---|---|
| VINS-Mono | `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d`, [TRO 논문](https://arxiv.org/abs/1708.03852v1), GPLv3 | camera+IMU sliding-window nonlinear estimator; [ProjectionTdFactor:34–64](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/vins_estimator/src/factor/projection_td_factor.cpp#L34)의 time offset·feature velocity·row/readout 항 | 자체 `src/backend/optimizer.cpp:233–246,348–353,395–408`: pose/speed/bias/inverse depth Ceres DENSE_SCHUR/DOGLEG, extrinsic constant. 현재 `ProjectionFactor` 호출에는 td/row parameter가 없음. 영상-IMU clock 정규화를 online temporal calibration으로 간주하지 않음 |
| OpenVINS | `69488123ed9362dd44b6f28e7f4680abbff1442b`, [ICRA2020 저자 논문](https://pgeneva.com/downloads/papers/Geneva2020ICRA.pdf), GPLv3 | [UpdaterMSCKF:168–284](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/update/UpdaterMSCKF.cpp#L168): feature nullspace projection·chi-square rejection·measurement compression·EKF update. [StateOptions:38–68](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/StateOptions.h#L38): FEJ true, camera pose/K/timeoffset calibration defaults false, max clones11 | 자체는 nonlinear window 재최적화. OpenVINS의 calibration 지원을 모든 profile에서 항상 켜진 것으로 설명하지 않음. 자체 prior의 frozen linearization은 전체 EKF FEJ consistency 구현과 동일하지 않음 |
| OpenVINS geometry | 같은 commit, [공식 feature 유도](https://docs.openvins.com/update-featinit.html) | [FeatureInitializer:30–108,341–368](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_core/src/feat/FeatureInitializer.cpp#L30): bearing matrix·condition·depth·refinement 이후 baseline ratio 검사 | **자체 young admission은 이미 존재**. `include/frontend/feature_manager.h:74–86`, `src/frontend/feature_manager.cpp:284–329,349–357`: structural slot과 depth admission 분리, world-bearing condition≤10000, all-view positive depth·pixel reprojection≤1.5. 이전 anchor<8 설명만으로 현재 결함을 진단하면 틀림. 같은 condition threshold라도 전체 geometry/estimator 동등성은 아님 |
| Basalt keyframe/track | `0f3b2b52c807f70ff4e2973ce253c73329eea7bc`, BSD3 | [sqrt_keypoint_vio.cpp:337–449](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/src/vi_estimator/sqrt_keypoint_vio.cpp#L337): current landmark 연결률·keyframe 간격, 이전 관측에서 충분한 baseline 쌍 선택. [flow:214–226](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/include/basalt/optical_flow/frame_to_frame_optical_flow.h#L214): 전후 patch tracking 검증 | 자체 `src/frontend/feature_tracker.cpp:245–342`는 ordinary LK/F majority 뒤 회전 hint subset·reverse level0·cycle≤0.25px² 검증. Basalt default recovered distance²0.09·baseline0.05m은 해당 구현값이며 전화용 수락값 아님. 자체 `compensatedParallax2:556–584`는 p_i_comp=p_i로 실제 회전 보상 없음 |
| Basalt marginalization | 같은 commit, [Square Root Marginalization, ICCV2021 v1](https://arxiv.org/abs/2109.02182v1) | [sqrt_keypoint_vio.cpp:702–760](https://github.com/VladyslavUsenko/basalt/blob/0f3b2b52c807f70ff4e2973ce253c73329eea7bc/src/vi_estimator/sqrt_keypoint_vio.cpp#L702): configured square-root linearization/QR/prior 경로 | **자체 marginalization도 이미 square-root QR**. `src/backend/factor/marginalization_factor.cpp:375–418`: dropped CPQR→nullspace rows→kept QR. normal matrices는 진단 경로. 'Basalt QR 도입'을 신규 개선으로 제안할 근거 없음. Ceres solve의 DENSE_SCHUR와 marginalization QR은 서로 다른 단계 |

- VINS README의 image>20Hz·IMU>100Hz·정확한 시각 요구는 [고정 README](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d/README.md)에 명시. browser DeviceMotion 약42.5Hz 기록과 동일 입력 전제로 성능 순위를 만들 수 없음.
- OpenVINS [FEJ 유도](https://docs.openvins.com/fej.html)는 global translation3+yaw1의 비관측 방향을 다룸. 자체 QR 존재만으로 gauge/nullspace·uncertainty 일관성 수락까지 완료되지 않음.
- 현재 native `VIOSystem`과 browser `VIOEngine`은 별도 입력 경로. browser의 IMU 경계는 `src/vio_engine.cpp:187–229`에 strict monotonic·past/future bracket·affine endpoint, factor interval 허용오차는 `src/backend/optimizer.cpp:268–276`의1e-6s. native reference 실행이 이 경계를 검증하지 않음.

## 실행 결과를 비교하기 위한 우선순위

| 순위 | 구체적 비교·다음 gate | 실패/미수락 조건 |
|---|---|---|
| P0 | 같은 image bytes·IMU·timestamps·K/distortion/extrinsic·seed·threads·source/artifact fingerprint를 reference별 보존 | ARCore emulator native camera/sensor 입력을 browser 입력과 같은 것으로 표기; 다른 frontends의 총 runtime을 backend 차이로 단정 |
| P0 | 입력 timestamp/출력 timestamp·unique frame·tracking state·reset epoch·Lost/current residual membership를 같은 분모로 집계 | 반복 pose·처리 생략을 fresh FPS로 집계; limited/inertial pose를 metric visual correction으로 승격 |
| P0 | baseline 대비 independent GT, one-to-one association, body frame, SE(3) scale1 translation/rotation ATE·RPE·initialized/pose coverage·Lost/reset/max gap | zero pose/association, 회전 누락, 성공 구간만 남기기, Sim(3) scale 보정만으로 정확도 통과 |
| P1 | raw parallax·회전 보상 world-bearing angle·mature current support·keyframe decision reason부터 진단 비교 | 순수회전 pixel displacement를 translation baseline으로 판정. 진단 성공만으로 keyframe 정책 변경 수락 |
| P1 | timeoffset·jitter·rolling shutter를 독립 analytic GT fixture에서 단일 변수 sweep 후 profile별 민감도 비교 | 추측한 readout/offset 주입, 카메라 timestamp 출처만 바꾸고 physical synchronization 해결 주장 |
| P2 | map relocalization·false recovery·좌표 epoch continuity를 local VIO와 분리 평가 | ARKit 상태명 도입·reset 횟수 숨기기·출력 smoothing을 지도 복구 구현으로 계산 |

- 기존 회귀 기준을 유지: 동일 입력 baseline 대비 SE(3) translation ATE·rotation RMSE 각각≤5% 증가, pose coverage 감소≤1percentage point. 추가 복구는 fixed denominator의 Lost 감소·false-ID 음성 사례도 통과해야 함. [기존 gate](2026-10-12-open-vio.md).
- 속도: capture/copy/queue/frontend/backend/render와 초기화 전후 p50/p95/p99 분리, 직렬 ABBA 측정. 설치·compile·짧은 smoke와 end-to-end tracking 성능 수락을 구분.
- 현재 source에 이미 있는 admission·QR·timing identity를 재도입하지 않음. 이번 연구 lane의 제품 변경0·신규 runtime dependency0·실기기 GT0. 실제 상용/core 실행 결과는 별도 lane 문서로 연결해야 함.

## 재현·검증

```bash
python3 build/commercial-slam-20261013/research/fetch_primary.py
git diff --check -- docs/research/2026-10-13-engine-gap-analysis.md
git diff --no-index --check -- /dev/null docs/research/2026-10-13-engine-gap-analysis.md
```

- `fetch_primary.py`: primary text만 파일당2MiB, 동시4개로 획득; local source14개 fingerprint. dataset·TLS·서비스·제품 source 변경 없음.
- fetched source는 표의 SHA 고정; live API 문서는 재조회 시 바뀔 수 있으므로 manifest의 획득시각·resolved URL·SHA로 이번 원문 식별.
- [verification.json](../../build/commercial-slam-20261013/research/verification.json): primary28개·local14개의 hash mismatch0, 문서 local link 누락0, source 핵심 계약7개 확인. untracked 신규 문서는 `--no-index --check`로 공백 검사.
- 미확인: 상용 solver 전체 복원·binary 원본 source rebuild, 동일 Flip6 입력의 상용/공개 엔진 정확도, 물리 exposure/IMU 동기, phone thermal/battery, 독립 현장 GT. 임의로 완료 처리하지 않음.
