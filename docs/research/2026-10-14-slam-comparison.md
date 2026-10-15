# Mobile-SLAM·8th Wall·ARCore 종합 비교와 문제 진단

- 기준: 2026-10-14, checkout `5d48e4bd7466842dad98928fb1f12988488402df`.
- 목적: 데이터 로드·센서·영상 처리·SLAM·시각화 전체를 비교하고, IMU 지연/긴 SLAM 처리/추적 소실의 관계를 검증.
- **주 비교 대상은 8th Wall 모바일 웹**. ARCore native는 입력·보정·API·기기 통합 설계의 참고이며 웹 엔진과 처리시간을 직접 순위화하지 않음.
- 수행: 현재 소스 격리 WASM 빌드·provenance 검증, 원본 실측 재계산, 실제 App/Worker 및 8th Wall browser 실행, 저장 영상 replay, UI failure 재현, 공식 문서·실제 source 비교.
- 제품 변경·서비스 배포·신규 실기기 녹화 없음. 기존 8th Wall 분석 파일18개와 원본 기록 보존. 개선 구현 순서는 [실행 계획](2026-10-14-slam-improvement-plan.md)에 정의.

## 결론

1. **IMU 직접 전파 계산이 주병목이라는 근거는 없음.** Oct12 phone 원본의 측정 시간창607행에서 IMU 계산 p95 **0.235ms**, bracket 대기 p95 **12.685ms**. frontend **41.566ms**, backend **42.466ms**와 비교해야 함. 단, `imuMs`는 Ceres의 IMU residual·backend 재적분 전체를 포함하지 않음.
2. **추적 소실의 직접 gate는 현재 영상 제약 부족.** 같은 기록의 Lost33개는 current visual factor0/1, Ceres 미실행. 최신 source의 저장 phone prefix replay도300개 중53fresh·current support 부족86개·guard reset2회. 속도만 줄여서는 해결된다고 볼 수 없음.
3. **지연은 별도 문제로 존재.** capture/readback, frontend, solve/QR, IMU bracket, result→화면을 각각 줄여야 함. `max_solver_time=.06`은 Ceres 구간에만 적용되므로 전체60ms deadline이 아님.
4. **화면도 느리거나 잘못 보일 수 있음.** 새 결과가 없거나 Worker가 실패해도 이전 Tracking/age/frustum이 남는 결함을 재현. follow-view alpha=.08은20Hz 가정에서 목표 위치95% 추종에1.8초가 필요하나 raw SLAM pose 자체의 지연은 아님.
5. **비교 전 provenance·replay 경계부터 고정해야 함.** 활성 loader와 현재 source15파일 차이 확인. 또 live의 제출 전 IMU 묶음만 순차 replay하면 미래 bracket이 도착하지 않아 인위적 실패50개가 생김.

- 수치 근거: [입력·시간 재계산](2026-10-14-slam-comparison-input-timing.md), [알고리즘](2026-10-14-slam-comparison-algorithms.md), [웹/UI 재현](2026-10-14-slam-comparison-web.md), [새 replay 결과](../../build/slam-comparison-20261014/replay-summary.json).
- 이 결론은 과거 phone 기록과 신규 desktop 제어 실험을 구분한 진단. 최신 코드의 실제 phone 정확도·지속 성능을 입증한 것은 아님.

## 상세 문서

| 문서 | 범위 |
| --- | --- |
| [입력·IMU·시간](2026-10-14-slam-comparison-input-timing.md) | sensor source/arrival, batch·queue·bracket, propagation/solver 비용, 원본 통계, replay 결함 |
| [SLAM 알고리즘](2026-10-14-slam-comparison-algorithms.md) | 특징·KLT/GORB·geometry·초기화·window·Ceres·QR·recovery·기각된 후보 |
| [웹·시각화](2026-10-14-slam-comparison-web.md) | load/capture/copy/Worker/render, texture-pose 연결, stale UI, follow view |
| [ARCore 계약 비교](2026-10-14-slam-comparison-arcore.md) | exposure·CPU/GPU K·sensor pose·tracking·map/depth/anchor·실측 API |
| [개선 순서·수락 조건](2026-10-14-slam-improvement-plan.md) | baseline→표시/관측→입력→성능→기하/재연결, 실패 시 중단·복귀 조건 |
| [8th Wall 상세 연구](2026-10-14-8thwall-study.md) | 52개 공식 repo, 공개 source, 실제 binary·내부 residual·실험 근거 |

## 전체 기술 비교

- 아래 8th Wall의 공개 source와 배포 binary는 서로 다른 revision. 공개 image-target 구현을 private world solver 전체로 확대하지 않음.
- 각 행의 source/file/line 및 공식 URL은 위 상세 문서에 연결. “미확인”은 해당 기술이 없다는 뜻이 아님.

| 영역 | Mobile-SLAM 현재 | 8th Wall 모바일 웹 | ARCore native |
| --- | --- | --- | --- |
| 실행·빌드 | C++17/OpenCV/Ceres→single-file WASM, 별도 Worker | JS framework+분리 WASM chunks; 공개 build C++20/Bazel/Emscripten | 앱 SDK+기기 설치 service, 내부 solver 비공개 |
| 엔진 로드 | 약6MB JS에 WASM 내장, initialize 완료 후 Start | core 준비 후 slam/face 등 선택 chunk preload | Session 생성·설정·resume |
| 리소스·모델 | camera profile/config와 renderer CDN | 상대 ResourceUrls, 모델/worker fetch, ECS asset cache | camera config·기기 지원·별도 depth/feature 설정 |
| 영상 획득 | rVFC에서 owned gray snapshot | HTML video→draw/compute texture, currentTime/repeat 검사 | Session frame의 GPU camera texture; CPU image 별도 |
| calibration | profile 또는 heuristic K, crop/rotation 변환 구현; 실제 폰 K 미측정 | device heuristics/내부 calibration, public projection; exact K 미확인 | frame별 image/texture intrinsics 공개, 둘이 다를 수 있음 |
| camera/body extrinsic | 설정값·좌표 변환 계약; 실제 rig 검증 부족 | stage orientation·private 처리 경계 | camera pose와 Android sensor pose 구분 |
| 노출/시간 | capture/presentation/callback fallback, 물리 exposure 미확인 | video/frame/stage clock 분리; source orientation timestamp TODO | physical camera center-row exposure pose 계약 |
| 센서 입력 | Generic Sensor 또는 DeviceMotion; 미래 시각 거부·동기 pairing | DeviceOrientation quaternion+DeviceMotion queue | native service 내부 입력, 앱 SensorManager observer와 동일성 미확인 |
| 실제 센서 관측 | phone DeviceMotion 약42.5Hz | 본 desktop fixture60Hz 주입; 실제 phone rate 미측정 | 외부 SensorManager 약461Hz 실측; 내부 사용 rate 단정 불가 |
| IMU 전송 | 독립10ms flush+frame 직전, Worker busy에도 전송 | stage 직전 event queue 직렬화 | SDK 내부 수집/보정 공개 범위 밖 |
| 영상/IMU 결합 | frame 앞/뒤 bracket, interpolation, 손실 검사 | stage frame과 event queue 연결; exact private fusion 미확인 | 하드웨어/기기 통합; 실제 내부 buffering 미확인 |
| backpressure | in-flight1+최신 pending1, 오래된 영상 교체 | repeat/staged texture pipeline, 보조 ML worker busy skip | BLOCKING/LATEST_CAMERA_IMAGE, timestamp 중복 확인 필요 |
| capture 비용 | main drawImage/getImageData→RGBA→gray→owned copy | GPU resize/pyramid/score, downstream packed readback | native texture 경로, CPU image acquire 비용 별도 |
| 병렬화 | SLAM Worker, capture-worker opt-in 기본OFF | GPU/CPU staging; tracking callback과 semantics/media worker 구분 | native SDK/service/GPU, 내부 thread schedule 미확인 |
| 특징 검출 | Shi–Tomasi/goodFeaturesToTrack, mask/min-distance | FAST계열 GPU score/pyramid + CPU ORB/GORB descriptor | detector/descriptor 종류 미확인 |
| 프레임 tracking | KLT pyramid, F-RANSAC, gyro-seeded rescue·reverse 검증 존재 | descriptor/gravity·ROI/image-target matching 공개; private world detail 별도 | motion tracking 결과·실패 원인 API, 내부 matcher 미확인 |
| feature identity | 연속 KLT ID 중심; 소실 ID의 범용 descriptor 재연결 없음 | image-target local/global descriptor 경로 확인 | 내부 feature ID/map 연계 미공개 |
| 기하 승인 | DLT/rank/condition/cheirality/reprojection; young geometry 허용 | 공개 image-target homography/PnP/P2P; world map gate 미확인 | 내부 depth/landmark gate 미확인 |
| 초기화·scale | metric VIO 초기화/중력·bias·depth 조건 | responsive/absolute 별도; finite output/NORMAL≠metric 수락 | TRACKING/PAUSED, metric API; 동일 입력 정확도 별도 |
| backend | 11-state window·10 IMU factor·visual·prior | binary Ceres/BA 경로 및 residual 일부 수치 확인 | solver 종류·수식·window 크기 미확인 |
| solver/marginalization | DENSE_SCHUR/DOGLEG, square-root QR; 기존 재사용 최적화 존재 | Ceres 연결 확인, effective 옵션/전체 상태 미확인 | 비공개 |
| 처리 budget | Ceres cap; 그 밖 삼각측량/assembly/QR은 별도 비용 | 단계 분산 및 반복 출력; total deadline 미확인 | Session.update 호출 시간은 전체 solver 시간 아님 |
| current support 실패 | 현재 factor<2에서 Lost·prior/window 처리 | 공개 status와 render output 계속될 수 있음 | PAUSED와 failureReason 별도; NONE도 valid tracking 보장 안 함 |
| 복구 | 일부 회전 rescue, guard reset; 범용 지도 재지역화/loop closure 미검증 | stage/recenter·image-target 재검출; private world recovery 일부 미확인 | pause/추적 재개·anchor API, 내부 재지역화 수식 미확인 |
| 지도 수명 | local VIO window·시각화 point cloud, 영속 SLAM map 없음 | worldPoints와 ECS scene 분리, private map 미확인 | point cloud·anchors/planes/depth; 일반 map dump와 다름 |
| 렌더링 | live video+독립 trajectory viewer, 고정60°FOV | 대응 realityTexture+pose+projection으로 AR scene | 동일 Session frame의 background+view/projection |
| 후처리 | frustum raw pose, 관찰 camera follow lerp | adapter의 camera pose 직접 적용; face adaptive filter·sky blur 별개 | sample은 tracking pose/view 직접 사용 |
| failure 화면 | 명시 Lost clear; null/정지 result 만료 누락 재현 | pipeline/status별 처리; 앱 정책에 좌우 | sample PAUSED에서 가상 물체 draw 중단 |
| 기록·재생 | 이미지/IMU/metadata 별도; partial prefix·도착 일정 손실 주의 | 기존 synthetic adapter; 완전한 phone sensor replay 미확보 | MP4 record/playback; CPU decoded bytes와 frame timestamp 별도 |
| 정확도 근거 | 일부 synthetic/TUM GT, 실제 phone GT 없음 | 이번 source-shared fixture GT/metric association 미검증 | 실측 tracking 지속은 확인, 독립 GT 없음 |

## 8th Wall에서 참고할 설계와 그대로 가져오면 안 되는 것

- **참고1: GPU frontend.** 영상 전체 RGBA를 main에서 읽은 뒤 gray로 바꾸는 현재 경로와 다르게, resize/pyramid/score를 GPU에 두고 필요한 결과를 CPU로 옮김. WebGL/PBO fence·iOS fallback·queue age까지 계측해야 하며, 단순 shader 도입이 속도 개선 증거는 아님.
- **참고2: 프레임을 가로지르는 stage와 texture identity.** CPU가 이전 staged frame을 처리하는 동안 현재 texture를 준비. pose와 대응 texture를 함께 render할 수 있음. 기존 우리의 최신 pending/epoch 구조는 이미 존재하므로 새 queue를 추가하기보다 image/pose/render identity를 유지하는 확장이 필요.
- **참고3: descriptor+gravity 정보.** KLT 단절 후 재연결을 위한 연구 방향. 공개 ORB/GORB/image-target 알고리즘의 존재는 참고 가능하나 closed world solver의 모든 branch가 동일하다고 단정하지 않음.
- **참고4: 표시 목적과 metric 상태 분리.** responsive rendering의 계속되는 transform을 우리 metric `fresh`로 옮기지 않음. prediction/3DoF/limited 표시가 필요하면 별도 출력으로 제공하고 metric 정답처럼 평가하지 않음.
- **그대로 채택하지 않을 것:** vendor threshold, arbitrary scale, status 문자열, 미검증 pose smoothing, 관측 불가능한 depth의 임의 승격. 이전 D0/D2/D3/H4/K1/K2/P1/P2/C3 실패를 숨기며 같은 방향을 다시 켜지 않음.

## 새로 실행한 비교·대조

### A. 현재 source와 실제 web artifact 분리

- 새 빌드: [manifest](../../build/slam-comparison-20261014/build/manifest.json), [11/11 검증](../../build/slam-comparison-20261014/build/verification.json). Emscripten5.0.0, Ceres2.2.0, OpenCV4.5.5, Eigen3.4.0, jobs2.
- 현재 source86파일 유지, C3 OFF 실제 compile 확인. 새 loader SHA `38837d4b79abb088610484f4e2c016aac4336fe9761dcfc56874d48e64785589`; 기존 불변 OFF artifact와 byte 동일.
- web loader SHA `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`; source15파일 차이. 파일 수정시각만으로 최신 빌드 여부를 판단할 수 없음. 원본 web 파일을 교체하지 않음.

### B. 같은 fixture를 두 모바일 웹 경계에 공급

- 입력: fast-rotation seed707, portrait480×640, image20Hz/IMU60Hz, 400image·19.95초. `input.json` SHA **717fc9c312a5dede6b316c0f13d2e72b8c9805b30cfe3c74308a266a24e2772a** 두 runner 일치.
- 새 browser에서 순차 실행. Mobile-SLAM은 실제 App/Camera/IMU/Worker/Renderer + native rAF + DeviceMotion, 8th Wall은 원본 SDK + mobile emulation + generated video/sensor events.
- **동등성 범위는 원본 fixture**. 서로 다른 adapter, viewport/방향, K/extrinsic, 내부 scale/fusion, frame 소비·sensor dispatch 때문에 최종 estimator 입력까지 동일하지 않음. 특히 8th Wall videoTime과 pose의 일대일 association 미입증.

| 관측 | Mobile-SLAM 새 source | 8th Wall binary responsive |
| --- | ---: | ---: |
| 계획/영상 제출 | 400 / 392 | 400 / 400 |
| 결과 callback | 392 | 960 |
| fresh valid metric pose / 비반복 callback | fresh347 | nonrepeat392; 같은 의미 아님 |
| warmup 이후 | 294/300 fresh | NORMAL/metric warmup 동등 정의 없음 |
| 추적 상태 | initializing39, Tracking347, current-support Lost6 | 비반복 NORMAL223 / LIMITED169 |
| 원점/scale | configured metric rig | origin `(0,2,0)`, responsive scale |
| GT | SE(3) scale1, ATE0.13054m, rotation2.79257°, 1s RPE0.08529m/1.02696°; 단일 합성 gate PASS | 미검증; 수치 trajectory 크기를 m 단위 오차로 해석하지 않음 |

- 8th Wall도 이 빠른 회전/가상 센서 조건에서 LIMITED가 나옴. 이 결과는 실제 phone 열등/우수 판정이 아니라 동일 source fixture로도 초기화·상태·입력 계약을 확인해야 한다는 근거.
- 자체 새 run stage 선형보간 p95: IMU **0.08ms**, frontend **20.2065ms**, backend **22.7925ms**, total **33.5083ms**. desktop 수치이며 phone 성능과 분리. stage percentile 합산 금지.
- 기존 runner의 `engineProcessingMs` 통계는 다른 분위수 구현과 경계로 p95 **33.615ms**. 양자를 같은 정의의 숫자로 혼합하지 않음.

### C. 시간이 충분하면 Lost가 사라지는가

- live가 실제 제출한392영상·IMU·effective profile을 저장하여 순차 raw replay. 처음에는 제출 전 IMU count만 보내고 결과를 기다려 **missing_bracket50개**, fresh299 발생.
- 50개는 모두 해당 frame 제출 시 미래 endpoint가 없던 행과 일치. live에서는 대기 중 후속IMU가 들어왔지만 순차 runner는 다음 frame 전에만 전송하므로 원래 schedule을 재현하지 못함. **엔진 회귀로 집계하지 않음**.
- 별도 future-bracket control은 동일392영상과 IMU에서 각 frame의 뒤쪽 sample을 먼저 제공. fresh347·Lost6·missing0; live와 **392행의 sourceIndex/input·pose timestamp/status/reason/fresh/valid/pose16개/epoch exact 동일**.
- 따라서 이 고정 입력·profile의6개 Lost는 main capture/render 압박이나 미래 IMU 대기가 없어도 재현. 모든 phone 손실에 대한 보편적 인과 증명은 아니며, 현재 visual-support 실패를 별도로 개선해야 할 근거.
- 제어 입력·비교 상세: [replay-summary.json](../../build/slam-comparison-20261014/replay-summary.json), [schedule 결함](../../build/slam-comparison-20261014/input-timing/matched-replay-schedule-failure.json). future-bracket 제공은 live 도착 일정 재현과 다름.

### D. 저장된 실제 phone 이미지의 최신 source 재시험

- Oct7 minute1 원본300PGM+원본gyro+기록profile, 현재 Wrapper/Worker와 새 WASM으로 재실행. [report](../../build/slam-comparison-20261014/replay/current-phone-prefix/report.json).
- 완료300, initialized140, fresh53, current-support 부족86, final-support 부족1, guard reset2. engine p95 **33.965ms**인 desktop 순차 실행에서도 추적 문제가 남음.
- 기존 current baseline53/300을 재현. 초기화/reset/빠른실패 집단이 섞여 있으므로 이 낮은 처리시간을 추적 효율 개선으로 보고하지 않음.
- 원본은 과거 gyro 매핑·미측정 calibration·녹화 중간 상태를 포함. 보정입력/rotation-rescue ON의 sensitivity control과 구분하며, 최신 phone 입력의 정확도로 확대하지 않음.

## 진단 우선순위

| 우선순위 | 문제 | 증거 수준 | 다음 조치 |
| --- | --- | --- | --- |
| P0 | source/artifact/실측 버전 혼합 가능 | source15파일 차이·fresh build 검증 | 비교 기준 pin, 배포는 별도 gate |
| P0 | UI stale 상태가 정상 추적처럼 남음 | 실제 method+wrapper failure 격리 재현 | freshness/error 상태 표시 분리 및 회귀 |
| P0 | camera/IMU calibration·물리 노출 정합 미확인 | 현재 profile/기록 한계 | camera/K/crop/축/offset 실측 및 기록 |
| P1 | 현재 metric visual-support 고갈 | phone33Lost, 새 synthetic6Lost exact control, prefix86support fail | ID→geometry→최종 factor 탈락 원인 추적, 대응·기하 후보 분리 |
| P1 | capture/frontend/backend tail | 실측 stage 및 새 desktop 계측 | 비용 세분화 후 한 단계씩 A/B; total pose age 수락 |
| P1 | offline replay의 IMU 도착 일정 손실 |50행 exact 분리·future endpoint control | recorded-arrival scheduler와 수학적 replay 용도 분리 |
| P2 | follow view/독립 video가 만드는 체감 차이 | source 수식·렌더 구조 | raw/follow/AR overlay 의미 명확화, matched texture 출력 실험 |
| P2 | 범용 descriptor 재연결·map/relocalization 부재 | source scope 및 과거 false-ID 실패 | 별도 bounded prototype, false acceptance gate |

- 코드·실험으로 확인한 결함과 가설을 분리했으며, 공개되지 않은 ARCore/8th Wall 내부를 추측해 우리 구현의 결함으로 단정하지 않음.
- 다음 구현의 수락 조건과 기존 실패 회귀는 [개선 계획](2026-10-14-slam-improvement-plan.md) 참조.

## 재현·검증·변경 파일

```bash
# 출력은 반드시 새 이름 사용. source/dependency build 명령은 build/README.md 참조.
python3 scripts/dev/analyze-slam-bottlenecks.py \
  --output build/slam-comparison-20261014/input-timing/new-recomputed.json
node scripts/dev/synthetic-browser.mjs \
  --input build/motion-20261010/fast-rotation-707/input.json \
  --loader build/slam-comparison-20261014/build/wasm/vio_engine.js \
  --output build/slam-comparison-20261014/replay/new-fast707-live \
  --mode live-paced-native-raf --sensor device-motion --frames 400 \
  --f-threshold default --rotation-rescue default --timeout-ms 180000
node scripts/dev/references/eighthwall/experiment-matrix.mjs \
  --label new-comparison-fast707-responsive-400 --arm moving --mode mobile \
  --scale responsive --frames 400 --dataset build/motion-20261010/fast-rotation-707
python3 scripts/dev/evaluate-synthetic-vio.py \
  --dataset build/motion-20261010/fast-rotation-707 \
  --results build/slam-comparison-20261014/replay/new-fast707-live/report.json \
  --estimate-frame camera \
  --output build/slam-comparison-20261014/replay/new-fast707-live/evaluation.json
```

- [build/README.md](../../build/slam-comparison-20261014/build/README.md), [새 실행 결과](../../build/slam-comparison-20261014/replay-summary.json), [최종 검증](../../build/slam-comparison-20261014/verification.json).
- 분석기 독립검사5개, UI source-method probe, freshWASM build/smoke·provenance11개,392행 exact control을 각각 구분. 발견한 UI/replay 문제와 과거GT 실패는 제품 통과로 집계하지 않음.
- 신규 versioned 후보: 비교 문서5개와 개선 계획1개, `scripts/dev/analyze-slam-bottlenecks.py`. 기존 분석/record/replay/browser/evaluator 도구 재사용; dependency 추가 없음.
- 소스 구조 비교와 계측·재현이 이번 완료 범위. 제품 수정, native 전체회귀, 실기기 신규촬영, 배포·push·merge는 수행하지 않음.
