# Vuforia 공개 API·개발 환경·동일 입력 비교

- 조회일: **2026-10-12**, Asia/Seoul; 담당: GPT-6.1 Sol Ultra.
- 고정 기준: 공식 **Vuforia Engine11.4.4 / Driver API7**. 공개 문서·배포 경계·독립 계약 검사 완료; **Vuforia proprietary runtime·tracking·ATE·latency 미실행/미측정**.
- 개발 환경: [재현 명령](../../scripts/dev/references/vuforia/README.md), [실행 manifest](../../build/research-20261012/vuforia/environment/self-test.json), [자산 manifest](../../build/research-20261012/vuforia/reference-environment-summary.json).
- 제품 estimator·tracker·capture 설정 변경 없음. 기존 tracked/untracked 변경·dataset·TLS·서비스 보존.

## 엔진 범위·플랫폼

| 항목 | 공식 근거·이번 판정 |
|---|---|
| Device Pose | 환경 영상·센서로 device6DoF 추적. Fusion이 ARKit/ARCore 또는 Vuforia 기술 선택; **Vuforia version만으로 실제 VIO backend 고정 불가**. [Device Tracking](https://developer.vuforia.com/library/vuforia-engine/environments/device-tracking/device-tracking/) |
| Fusion | 플랫폼 provider 우선, Vuforia VISLAM/vision SLAM fallback 설명. 현재 문서에 **VISLAM deprecated, 향후 제거** 경고와 이전 fallback 예시 공존; actual provider API 기록 필요. VISLAM을 최신 모든 기기의 독립 Vuforia solver로 단정 금지. [Fusion](https://developer.vuforia.com/library/vuforia-engine/environments/device-tracking/vuforia-fusion/vuforia-fusion/) |
| Image/Model Targets | [Image Targets](https://developer.vuforia.com/library/vuforia-engine/images-and-objects/image-targets/image-targets/)는 참조 영상, [Model Targets](https://developer.vuforia.com/library/vuforia-engine/images-and-objects/model-targets/model-targets/)는 객체 geometry 기반 인식·추적. Device Pose의 환경 VIO 정확도와 다른 출력·수락 기준 |
| Ground Plane/Area Targets | Device Pose 의존. Ground Plane은 수평면·anchor 배치, Area Target은 사전 scan 공간과의 localization. target 성공을 raw VIO trajectory 정확도로 전환 금지. [Ground Plane](https://developer.vuforia.com/library/vuforia-engine/environments/ground-plane/ground-plane/), [Device Tracking](https://developer.vuforia.com/library/vuforia-engine/environments/device-tracking/device-tracking/) |
| Native | Android10+/ARM64, NDKr26b+; iOS16+/ARKit; UWP Windows10/11. Android platform Fusion 최소 ARCore1.45. Linux Engine native target 없음. **Linux Ubuntu20.04+는 Model Target CLI authoring 도구 지원**. [Supported Versions](https://developer.vuforia.com/library/vuforia-engine/platform-support/supported-versions/) |
| Browser | 공개 배포/지원표에 WebAssembly/WebGL tracking SDK 없음. [VWS](https://developer.vuforia.com/library/vuforia-engine/web-api/vuforia-web-services-api/)는 target/cloud 서비스의 REST API. browser VIO runtime으로 분류할 근거 없음 |
| Unity Play Mode | webcam/simulator/session recording 경로. webcam에는 IMU가 없으며 Ground Plane은 Image Target으로 **emulation**. 이 경로를 phone VIO나 실제 바닥 추적 실험으로 보고 금지. [Play Mode](https://developer.vuforia.com/library/vuforia-engine/unity-extension/vuforia-play-mode-unity/) |

## 배포·라이선스·실제 구축 상태

- 공식 [C API](https://developer.vuforia.com/references/native/index.html)와 [umbrella header reference](https://developer.vuforia.com/references/native/VuforiaEngine_8h.html)의11.4.4 확인; ABI release 간 호환 보장 없음. SDK 교체만으로 기존 wrapper ABI 유지 가정 금지.
- 공식 [`PTCInc/vuforia-engine`](https://github.com/PTCInc/vuforia-engine/tree/1d42568da43bf850270fe9e17fbb511e349c95b7) checkout: `assets/references/Vuforia/upstream-issue-tracker`; commit `1d42568da43bf850270fe9e17fbb511e349c95b7`.
- repository는 **issue-only**, pinned tree에 SDK/sample source·LICENSE 없음. [공식 release](https://github.com/PTCInc/vuforia-engine/releases/tag/11.3.4)는 assets0개, SDK/sample/tool은 Developer Portal에서만 배포한다고 명시. GitHub latest11.3.4를 실제 현재 SDK11.4.4와 혼동 금지.
- 조회한 portal은 `vuforia-sdk-android-11-4-4.zip`, iOS/UWP/Unity 파일명을 표시. SDK ZIP/공식 checksum/개발 key는 이번 환경에 없음. 인증 우회·타인 mirror·계정 생성·유료 약정·binary 추출 없음.
- [Developer Agreement](https://developer.vuforia.com/legal/vuforia-developer-agreement), effective2022-12-31: Software §4(b)의 proprietary reverse engineering 및 developer tool 사용 제한 확인. Sample Code도 §3(d)의 조건부 사용권이며 MIT가 아님. **일반 end-user reference 앱의 compile/link와 proprietary solver 추출은 별개 작업**.
- 실제 환경 구축: `scripts/dev/references/vuforia/`의 CMake/C++17 host executable·Python/Node fixture·SDK provision 검사·Android version viewer source. 자산·빌드는 gitignored `assets/references/Vuforia/`; 전역 설치·서비스 시작 없음.
- `provision`: 공식 SDK ZIP+operator-supplied 공식 SHA-256 요구, checksum·path traversal/symlink·크기 상한 검사 후 격리 추출;11.4.4 header/ARM64 `libVuforia.so`·hash 확인. 사용자 제공 hash를 upstream 인증으로 확대하지 않음.
- `build-native`: 공급 SDK+NDK로 public headers와 `vuEngineGetLibraryVersionInfo()`를 실제 link하는 Android ARM64 `.so` 경로. `build-app`: Java17/Gradle8.7/AGP8.6.1/SDK35/NDK26.1.10909125/CMake3.22.1 기반 version viewer APK 경로. **현재 둘 다 SDK 부재로 BLOCKED; compile/link/APK 성공으로 주장하지 않음**.
- Android11.4.4는 ARM64만 지원. 별도 ARCore lane의 x86_64 emulator로 Vuforia runtime을 실행했다고 보고 불가. version viewer 자체도 camera·IMU·engine activation 없는 앱; 향후 version API 성공은 tracking 검증과 구별.
- 문서 획득 한계: urllib의 portal HTTP200은 client-rendered shell. 공개 legal/download DOM을 Playwright로 읽은 **뒤** portal UI scripting 제한 조항 확인; 후속 portal 자동화 중단·브라우저 종료. 로그인·버튼 클릭·SDK 다운로드·계정/약정 수락 없음. [획득 순서](../../build/research-20261012/vuforia/public-browser-metadata.json)에 실패한 저장 시도와 범위 기록.
- Context7: Vuforia resolve 결과 없음; 무관한 Engine library를 근거로 사용하지 않고 공식 HTTP/API 문서로 전환. [원문·해시](../../build/research-20261012/vuforia/primary-documents.json) 보존.

## 입력·좌표·품질 계약

| 경계 | 확인된 계약·실험에 미치는 영향 |
|---|---|
| CameraFrame | timestamp는 **노출 종료 시각/ns**, exposure duration도ns; Android time base는 `CLOCK_MONOTONIC`, 플랫폼마다 다름. browser rVFC/currentTime·수신 시각을 그대로 같은 물리 시각으로 해석 불가. [CameraFrame](https://developer.vuforia.com/references/native/structVuforiaDriver_1_1CameraFrame.html) |
| Driver pose | **world→camera, RH CV(x-right/y-down/z-away)**. IMU/body pose를 공급하면 camera extrinsic 적용 필요. 같은 timestamp의 **pose를 먼저, frame을 바로 뒤에** 전달. pose capability 선언 후 pose 미공급 시 frame retention/memory 증가 경고. [Advanced Driver](https://developer.vuforia.com/library/vuforia-engine/platform-support/camera/driver-framework/vuforia-support-custom-devices-using-vuforia-driver/) |
| Device Pose output | **camera→world, RH GL(Y-up)**; render viewMatrix는 반대 방향. target pose는 target→world. 4×4 matrix는 column-major, translation은 indices12/13/14. [Spatial Reference](https://developer.vuforia.com/library/vuforia-engine/environments/device-tracking/spatial-frame-reference-native/) |
| Calibration | frame별 fx/fy/cx/cy·distortion 제공. OIS/AF로 camera mapping 변화 시 intrinsics 보정 요구. distortion 순서 `[r0,r1,t0,t1,r2,r3,r4,r5]`. [CameraIntrinsics](https://developer.vuforia.com/references/native/structVuforiaDriver_1_1CameraIntrinsics.html), [Advanced Driver](https://developer.vuforia.com/library/vuforia-engine/platform-support/camera/driver-framework/vuforia-support-custom-devices-using-vuforia-driver/) |
| Raw IMU | 검사한 [Driver API7](https://developer.vuforia.com/references/native/Driver_8h.html) 표면은 camera·6DoF pose·anchor. **gyro/accelerometer packet submission endpoint 확인 안 됨**. SDK wrapper의 pose 공급을 proprietary VIO raw IMU 재생으로 명명 불가 |
| 품질·recovery | `NO_POSE/LIMITED/TRACKED`, initializing/relocalizing 등의 info 분리. excessive motion/insufficient features는 문서상 ARKit, insufficient light는 ARCore 전용. reason이 provider마다 다르므로 단일 threshold 의미로 비교 불가. [Pose Status](https://developer.vuforia.com/library/vuforia-engine/getting-started/vuforia-engine-api/observer-and-observations/pose-status-and-status-info-native/) |
| Reset | `vuEngineResetWorldTracking()`은 device tracking 재초기화·pose observation reset·extended tracking 상실. reset 후 좌표 epoch와 Lost/recovery를 별도 기록. [Device Tracking API](https://developer.vuforia.com/library/vuforia-engine/environments/device-tracking/device-tracking-api-overview/) |

## 실제 독립 계약 실험

- 명령: `python3 scripts/dev/references/vuforia/reference.py self-test`.
- 실제 순서: CMake configure → `cmake --build ... --parallel 1` → CTest → native executable → Python → Node → 공급 거절 fixture. **7개 명령 모두 exit0**; [전체 argv/log](../../build/research-20261012/vuforia/environment/self-test.json).
- CTest **1/1**, C++ executable **8 checks**, Python **12/12**, Node **6 assertions**, 공급 거절 **3/3**.
- Vuforia SDK 함수 호출 **0**. 이 결과는 독립 산술·입력 ledger·공급 절차 검사; SDK ABI·proprietary solver·실기기 정확도·속도 검증과 구분.

| 실행 fixture | 실제 확인 |
|---|---|
| 알려진Rz90°·translation(1,2,3) | `T_Ccv_W = Rx(180°) × inverse(T_W_Cgl)`의 알려진3D point 결과 일치; 잘못된 곱셈 방향은 불일치로 검출 |
| camera/body lever arm0.1m | 올바른 extrinsic의 analytical point 일치; body pose를 camera pose로 사용하면0.1m 오차 검출. 실제 센서 오차 측정값 아님 |
| pose/frame ns pair | 동일ns·pose-first 수락,1ns mismatch·frame-first 거절. Vuforia callback 자체를 실행한 결과는 아님 |
| 노출20ms·회전180°/s | start→end3.6°, midpoint→end1.8°의 민감도 산술 확인. 실제 Flip6 exposure·offset 값 아님 |
| 큰ns 정수 | `9007199254740992/9007199254740993`를 JS Number로 바꾸면 차이0, BigInt 차이1ns·차감 후 rebase 차이1ns. 실제 phone uptime/SDK 결함으로 해석 금지 |
| NV21 stride | width4/height2/stride6의18B와 tightly-packed12B 구분; 입력 format/stride의 독립 계산 |
| 공급 실패 | wrong checksum·`../escape.txt`·SDK 아닌 ZIP 모두 거절. [입력 ZIP·hash·결과](../../build/research-20261012/vuforia/environment/provision-failures.json) |

- fixture SHA256: `b312fd3de7e75bf2333b5be5e86af5d6f75520d2c0310f85f5160f9ad7a8796e`.
- 실제 host executable SHA256: `9244d7ae089001e81fc89a30b58fa5a5253a768e3b5c7f9588d66ee979eef558`; [source·artifact manifest](../../build/research-20261012/vuforia/reference-environment-summary.json).
- 실제 SDK presence compile: `c++ -std=c++17 -fsyntax-only sdk_presence_probe.cpp` → **exit1**, `VuforiaEngine/VuforiaEngine.h: No such file or directory`. [stderr](../../build/research-20261012/vuforia/sdk-presence.stderr.log). SDK 미설치 근거이며 Engine 결함 근거 아님.
- 실제 `status`, `provision`, `build-native`, `build-app` → 각각 **exit2/BLOCKED**, SDK 공급 요구. [status](../../build/research-20261012/vuforia/environment/status.json), [provision](../../build/research-20261012/vuforia/environment/provision.json), [native](../../build/research-20261012/vuforia/environment/build-native.json), [APK](../../build/research-20261012/vuforia/environment/build-app.json).

## FileDriver와 우리 A0/A1/A2의 비교 가능성

- 공식 [FileDriver](https://developer.vuforia.com/library/vuforia-engine/platform-support/camera/driver-framework/building-and-using-file-driver-sample-native/)는 SessionRecorder MP4에서 영상과 **기록된 device tracking pose**를 가져와 함께 공급. recorded pose가 VIO 입력으로 들어가는 실험은 영상+IMU로 pose를 독립 재추정하는 실험과 다름.
- 공식 [Recording and Playback](https://developer.vuforia.com/library/vuforia-engine/platform-support/recording-and-playback/recording-and-playback/)은 **Ground Plane recording/playback 미지원**, orientation 변경·video 재압축의 metadata 소실 제한 명시. FileDriver의 WAIT(all frames)와 RESPECT(real-time/drop 가능)도 구분 필요.
- 이번 환경에서 같은 원시 image+IMU bytes를 A0/A1/A2와 Vuforia 내부 VIO에 주입하는 공개·실행 가능 경로 확보 실패. Vuforia ATE·fresh coverage·latency는 **null** 유지.
- 우리 고정 출력은 다시 실행하지 않고 원본 results SHA·evaluation 입력 hash를 확인, A0/A1의400행 numeric11필드·pose exact parity 재검사. [명령](../../build/research-20261012/vuforia/compare_frozen_ours.py), [결과](../../build/research-20261012/vuforia/same-input-comparison.json).

| 대상 | 입력·관측 수준 | fresh / 400 | warmup fresh / 300 | translation ATE RMSE / rotation APE RMSE | 판정 |
|---|---|---:|---:|---|---|
| Frozen A0/A1 | 동일 churn12617 synthetic 영상·IMU;400행 numeric parity 차이0 | 324 | 265 / 88.333% | 49.835073m / 123.353636° | 기존 GT 실패 보존; 성능 비교 아님 |
| Rejected A2 | 동일 synthetic fixture·GT, 원본 hash 확인 | 274 | 215 / 71.667% | 0.317694m / 13.265090° | coverage·translation·rotation·RPE gate 실패; 미채택 |
| Vuforia11.4.4 | 독립 API 계약 fixture만 실행; SDK/target/key 미확보 | null | null | null | proprietary runtime·동일 입력 정확도·속도 미검증 |

- 위 own-engine metric: single global SE(3), scale1, unique timestamp association, source camera-frame 평가. physical phone/field accuracy 근거 아님. frozen 성능 계측·A2 ABBA 보류 상태는 [기존 보고서](../fixes/2026-10-12-recovery-performance.md) 유지.
- 공통 입력 spec SHA `3e1170150ba46befaca20c1da92121ac175feabba9474efe5178b82523f51427`, GT `209cf2dd937b06e8553d2d1351ca9f44bc09e7e17fd90e7350551f68f83e5af3`, manifest `d94c22148d55834b7618da2338f0ea0a99dcf2fe64353c62723eb1f2fe2ee744`.
- A1 binary 기록 SHA `ede92663d8cf22018ca85a405d155ea4718e103eaf3c8ce63203a45e2f4ef6aa`, A2 `ed3a9c7a7b2033df3dbce54dbe478614fa1339358b81755c9401d5f0740fd5a4`; executable 새 실행·성능 재측정 아님.

## 적용 가능한 후속 작업

1. 공식 SDK11.4.4 ZIP·공식 checksum 공급 후 `reference.py provision/build-native/build-app` 수행. ARM64 target에서 version API 호출 성공을 tracking·VIO 성공과 별도 기록.
2. 유효 license와 지원되는 기기 확보 후 actual Fusion provider/version, frame timestamp/exposure/K·distortion, pose frame/epoch, quality reason, queue/drop을 함께 저장. provider 비교와 Vuforia target tracking 비교를 각각 설계.
3. pose/frame pairing·camera/body transform·노출 시각 계약 검사를 우리 recorder/evaluator 설계 검토에 사용. 현재 phone의 calibration/EIS/OIS·물리 동기는 미확인 유지.
4. near-zero Lost·정확도 개선 근거는 우리 frozen GT/coverage gate와 독립 phone 시험에서 확보. Vuforia의 API 상태명·광고·독립 산술 PASS로 미수락 A2를 승인하거나 비공개 solver를 복구했다고 주장 금지.
