# ARCore 관점의 3종 SLAM 비교 — 2026-10-14

- 대상: ARCore Android native SDK·공식 sample·실기기 기록, 8th Wall standalone browser engine, 현재 Mobile-SLAM browser `VIOEngine` 경로.
- 결론: ARCore에서 우선 참고할 것은 **카메라/센서/표시 좌표의 분리, 유효 출력 상태, 자원 수명, 측정 가능한 입력 계약**. 공개되지 않은 estimator를 ORB·EKF·특정 sliding window라고 가정하여 복제할 근거는 없음.
- 기존 자료 재검토와 공식 문서 조회만 수행. 신규 기기 기록·설치·제품 코드 변경 없음.
- 이번에 확인한 파일·소스 SHA: [source-provenance.json](../../build/slam-comparison-20261014/arcore/source-provenance.json). 공식 문서 12개 HTTP200, 선택 실측 수치 원본 JSON 대조 통과.
- 현재 확보한 세 실험은 입력·동작·시간 측정 경계가 다르므로 **속도·정확도 순위 없음**. 461Hz 대 42.5Hz, 3.5ms 대 33.5ms를 같은 분모로 비교하지 않음.

## 근거와 확정 범위

| 계층 | 실제 근거 | 확정 가능한 범위 |
| --- | --- | --- |
| ARCore 공개 source | `assets/references/ARCore/source@3abfeb18669c2cbb2d07057f135d117ee9d91826`, `libraries/include/arcore_c_api.h`, HelloAR Java/C sample | 호출 순서·ABI·공개 입력/출력·resource 계약. sample은 private SLAM source가 아님 |
| 실제 배포 service | Z Flip6의 ARCore 1.56.262080393, ARM64 `libarcore_c.so` SHA `7c32d45c2ef1ebaf92deae86a7ba31b0dd00d39bde09ab0890535ff396d62050` | 4개 공개 함수의 명령/분기 inventory. private dispatch·전체 solver 복원 아님 |
| 실기기 실행 | `build/physical-arcore-20261014/analysis/all-three-verified.json`, 원본 capture 3개 | 지정 30초 실측·같은 MP4 재생 2회, 영상/IMU/pose 상태·hash 검증 |
| 현재 공식 문서 | [primary-manifest.json](../../build/slam-comparison-20261014/arcore/primary-manifest.json) | URL·retrieval UTC·HTTP status·원문 bytes/SHA. local pinned header와 현재 문서를 구분 |
| 8th Wall | [종합 연구](2026-10-14-8thwall-study.md), [binary](2026-10-14-8thwall-binary.md), [실험](2026-10-14-8thwall-experiments.md) | 공개 frontend/framework, private world binary의 일부 Ceres/residual, synthetic browser 실험 |
| Mobile-SLAM | `web/js/app.js`, `vio-wrapper.js`, `vio-worker.js`, `src/vio_engine.cpp`; [Flip6 재분석](2026-10-13-flip6-analysis-and-application.md) | 현재 코드의 구현·과거 fixed recording 관측. native `VIOSystem` 검증과 별도 |

- [ARCore fundamentals](https://developers.google.com/ar/develop/fundamentals)는 영상 feature와 IMU를 결합하여 카메라 pose를 추정한다고 설명. feature 검출기/descriptor, IMU preintegration 방식, 최적화 상태·window·marginalization·robust loss·loop closure 구현은 이 공개 설명으로 확정할 수 없음.
- ARM64 분석의 `ArSession_update` 254명령·18blocks·34callsites는 공개 entrypoint 분석이다. 이를 ARCore 전체 solver instruction 수나 실행 시간으로 사용하지 않음. [기존 실행 증거](2026-10-14-physical-arcore.md).

## 획득·처리·출력 아키텍처

| 경계 | ARCore | 8th Wall | Mobile-SLAM 현재 경로 / 적용 판단 |
| --- | --- | --- | --- |
| 입력 소유 | Session이 camera/IMU 취득·추적 상태 관리; CPU image는 acquire/release, GPU는 external texture | browser getUserMedia·sensor events→camera texture→공개 stage/process 경계 | browser camera→gray owned snapshot→Worker IMU bracket→C++ `VIOEngine`; 입력 관측과 solver 계약을 직접 검사 가능 |
| 프레임 요청 | `Session.update`가 camera·anchor·plane 상태를 갱신; BLOCKING도 timeout 시 이전 frame 반환 가능 | callback 반복 가능; `repeatFrame` 및 staged previous-frame CPU 결과 구분 필요 | rVFC `presentedFrames` dedup·최신 pending snapshot·requestId/clientEpoch 이미 존재; 새 queue 추가보다 현재 계약 검증 우선 |
| 처리 배치 | update는 GPU 작업도 발생시킬 수 있는 SDK 경계; 내부 thread/solver timing은 비공개 | 공개 source: 이전 staging CPU→scene update→camera/render→현재 GPU 준비 | Worker wall·frontend·backend·render 관측 경계를 별도 기록; 서로 다른 구현의 stage 이름만 맞춰 kernel 시간으로 비교 금지 |
| 구성 | camera config·display geometry·depth/lighting/image/instant placement를 공개 API로 선택 | XrController/FaceController/LayersController 구성·chunk 분리 | resolution/calibration/rotation/feature/solver 설정과 epoch 전환 기록; 변경 후 기존 frame/pose를 이어 쓰지 않는 정책 검증 |
| 출력 | TRACKING 여부와 failure reason, physical/display/sensor pose, pointcloud·trackables·anchors | pose/intrinsics/worldPoints/status 및 image/face/sky feature 출력 | `poseValid`·`poseFresh`·poseTimestamp·engine epoch·quality/current visual support를 명시; finite matrix만으로 성공 처리하지 않음 |
| 재생 | SDK recording MP4의 영상·AR sensor data를 session playback으로 사용 | 이번 연구는 grayscale generator·synthetic sensor event 입력 | raw image/config/IMU input replay 가능. 동일 engine-start state·drop schedule·exposure까지 자동 동등하지 않음 |

- ARCore sample 순서: [hello_ar_application.cc](https://github.com/google-ar/arcore-android-sdk/blob/3abfeb18669c2cbb2d07057f135d117ee9d91826/samples/hello_ar_c/app/src/main/cpp/hello_ar_application.cc#L142)에서 camera texture 지정→`ArSession_update`→camera 획득→display geometry UV 변환→view/projection→camera 배경→tracking 검사→depth/lighting/anchor 렌더. sample render loop를 private solver thread 구조로 오인하지 않음.
- [ArSession 계약](https://developers.google.com/ar/reference/c/group/ar-session): blocking update는 새 영상 대기 또는 내장 timeout 후 반환, 시작 시 timestamp0 가능. local header 3366–3406행은 timeout을 당시 66ms로 명시. 따라서 `update` 횟수와 실제 새 영상 수를 timestamp로 구분해야 함. texture의 pause/destroy 이후 수명도 유지 보장되지 않음.
- 현재 코드 대응: `web/js/app.js:1087` rVFC·geometry/generation 검사·`gray.slice()` 소유권; `web/js/vio-wrapper.js:330` response identity·timestamp·epoch 검사. 이들은 이미 구현되어 있으므로 ARCore 참고라는 이유로 중복 계층을 만들 필요 없음.

## 카메라·시각·보정의 구체적 차이

| 항목 | ARCore 계약 / 실측 | 현재 프로젝트에서 남은 조건 |
| --- | --- | --- |
| 물리 pose 정의 | camera `getPose`: 영상 중앙 행의 exposure 중심 시각, OpenGL camera +X right/+Y up/−Z forward | body pose와 camera pose 방향·`T_B_C`/`T_C_B`, central exposure 시각을 실제 보정으로 연결해야 함 |
| 표시 pose | display-oriented pose는 화면 회전에 따른 Z축 90° 배수 변환; sensor-frame pose는 방향과 위치도 다름 | CSS/display 회전·gray pixel 회전·intrinsic 회전·IMU body 회전을 같은 변환으로 취급하지 않기 |
| intrinsics | CPU image와 GPU texture 각각 unrotated/uncropped K 제공; frame마다 변할 수 있음 | `app.js:200` FOV/config/URL 추정 경로와 `:447` `heuristic_unverified` 존재. profile 지원이 실제 phone의 K·extrinsic·rolling shutter·OIS/EIS 측정 완료를 뜻하지 않음 |
| viewport | setDisplayGeometry가 UV·projection·hit-test 좌표에 영향 | render viewport 크기와 처리 gray 크기를 분리하고 crop/resize 이후 K를 추적 |
| frame time | `ArFrame_getTimestamp` ns, timebase는 명시적으로 미정의; local header4463행 | ns를 s로 단위 변환하는 것과 IMU/performance clock 공통 원점 증명은 별개 |
| CPU image time | 실측 image−AR frame p50 +1.994077ms, 범위 −2.943422…+2.799020ms | CPU 영상과 공개 pose time을 임의로 같게 덮어쓰지 않기. image/source/published/pose time을 별도 보존 |
| camera resource | acquireCameraImage에 NOT_YET_AVAILABLE/DEADLINE_EXCEEDED/RESOURCE_EXHAUSTED가 구별됨 | 읽기 실패·너무 오래된 프레임·수명 초과·Worker busy를 서로 다른 원인으로 기록 |

- [Camera C API](https://developers.google.com/ar/reference/c/group/ar-camera), pinned header 4238–4394행에서 physical/display/sensor 좌표와 CPU/GPU K를 교차 확인. 공개 계약은 확인했지만 이번 실측으로 ARCore 내부 camera–IMU calibration 자체를 복원한 것은 아님.
- `web/js/app.js:1111`은 captureTime→presentationTime→callback arrival 순서로 선택하고, `physicalExposureVerified:false`를 기록. timestamp 출처가 있으므로 실제 exposure와 같다고 승격시키지 않는 현재 정책을 유지.
- [camera config](https://developers.google.com/ar/develop/java/camera-configs)는 지원 configuration을 조회하고 camera FPS/depth 사용/GPU texture 크기를 선택하도록 제공. 낮은 texture 해상도가 메모리 대역폭·GPU 부하를 줄일 수 있지만 성능 개선 보장은 아님. browser `getUserMedia` constraint와 ARCore camera config는 동등한 calibration 보장 API가 아님.

## tracking 실패·지도·후처리

- [TrackingFailureReason](https://developers.google.com/ar/reference/java/com/google/ar/core/TrackingFailureReason): BAD_STATE, INSUFFICIENT_LIGHT, EXCESSIVE_MOTION, INSUFFICIENT_FEATURES, CAMERA_UNAVAILABLE 등 복구 안내용 원인. NONE은 초기 PAUSED에도 나올 수 있어 성공 flag가 아님. 여러 원인이 있으면 사용자가 대응하기 좋은 하나를 고르며 내부 잔차/condition number 전체를 뜻하지 않음.
- 실제 live 기록의 failure reason은 **607개 모두 NONE**, 그중 첫6개는 PAUSED. 따라서 실패 원인 NONE·finite pose·TRACKING·독립 GT 정확도는 네 개의 별도 판정.
- 8th Wall 역시 blank/disabled-world에서 finite transform을 유지했고 absolute arm에서는 INITIALIZING인데 pose가 존재. Mobile-SLAM의 `statusCode`, `poseFresh`, `currentVisualFactors`를 ARCore/8th Wall의 단일 enum으로 덮어쓰지 않음.
- 공개 ARCore pointcloud는 visualization/debug 목적으로 제공되고 형식·특성이 변경될 수 있음(pinned header4725행). point count85/203을 내부 map landmark 총수·BA residual 수·전체 지도로 해석하지 않음.
- [Anchor 계약](https://developers.google.com/ar/develop/anchors): world 추정이 변하면 anchor pose도 갱신될 수 있어, 물체를 world 고정 수치에 묶기보다 anchor와의 관계로 유지. 가까운 물체의 anchor 재사용·미사용 anchor 해제가 resource 관리 항목. anchor는 trajectory의 외부 GT나 drift 제거 보증이 아님.
- [Cloud Anchors](https://developers.google.com/ar/develop/cloud-anchors)는 host/resolve와 visual feature map 대응을 제공하는 별도 cloud 기능. 세션 내 Lost→복구, local map 유지, cloud anchor resolve, geospatial localization을 하나의 재지역화 알고리즘으로 묶지 않음. 이번 실기기에서는 가림 후 복구·map persistence·cloud host/resolve 미실험.
- Mobile-SLAM의 `feature_manager.cpp`의 anchor는 inverse-depth 기준 관측, PnP의 backend anchor는 pose 기준이다. 이 이름만으로 ARCore의 persistent scene `ArAnchor` API가 있다고 주장할 수 없음. 이번 현재 `src/include/web/js` 범위 검색에서 그와 동등한 scene-anchor/loop-closure/relocalization 서비스 경로는 확인하지 못함.
- [Depth](https://developers.google.com/ar/develop/depth)는 motion 기반 depth, 선택적 ML, 지원 장치의 ToF 융합과 occlusion/hit-test 활용을 설명. 제공 capability와 이번 기록에서 실제 depth 사용·깊이 정확도 측정은 다름. ARCore 추적 성공의 원인을 ToF·dense depth로 단정하지 않음. 8th Wall worldPoints·Mobile-SLAM sparse mapPoints도 dense depth와 같은 출력이 아님.

## 성능: 공정하게 비교할 수 없는 숫자

| 기록 | 실제 관측 | 동일 지표로 간주하면 안 되는 이유 |
| --- | --- | --- |
| ARCore physical live | 607 frame/601 TRACKING, 첫 TRACKING +0.248706s, 이후 loss0; update p50/p95/p99 3.519739/4.480568/4.952851ms | sample의 blocking API 구간. 전체 VIO 계산·capture-to-pose 시간 아님; startup 기준도 첫 frame 시각 |
| ARCore SensorManager observer | accel461.340852Hz·gyro461.269894Hz; source interval p50 2.166458ms | 외부 observer 측정. 내부 ARCore가 같은 packet·rate·filter를 소비하는지 미확인 |
| Mobile-SLAM 과거 Flip6 browser | 약42.5Hz DeviceMotion; 10/12 시간창607결과/574fresh, Lost33 | 다른 날짜·동작·보정·OS/browser 경로. source/arrival·sampling/filter 특성이 다름; native 대비 약11배 정보/정확도 차이라는 결론 불가 |
| Mobile-SLAM 과거 engine | 10/12 engine p50/p95/p99 33.525/69.767/94.278ms | 자체 Worker/C++ 계측. ARCore update와 범위가 다르고 당시 source bundle·동작도 다름 |
| 8th Wall responsive synthetic | 240 source, 238fresh; declared-video→observer p50 41.90ms | desktop SwiftShader·합성 camera/IMU·다른 pipeline. 물리 exposure latency도 아니고 ARCore/native 실측 속도와 비교 불가 |

- 원본 재확인: `all-three-verified.json`의 live `sessionUpdateBlockingApiMs`, `externalImu`, `camera`, `imageTimestampMinusFrameNs`, `stateEpisodes`; capture/manifest 검증 결과 PASS.
- `HelloArActivity.java:493–495` snapshot은 `session.update()` 호출만 `elapsedRealtimeNanos`로 감싼 뒤 observer를 호출. 영상 파일 I/O·후속 렌더는 그 3.5ms 밖에 있지만 관측 frame rate에는 영향을 줌. 전체 source span28.227617초에서 관측21.468338Hz는 engine 최대 처리량 아님.
- `ReferenceAudit.java:156–207` snapshot은 physical camera pose·CPU intrinsics·pointcloud time·CPU image time을 별도 기록. session output과 외부 IMU의 internal parity 주장은 없음.
- [공식 성능 안내](https://developers.google.com/ar/develop/performance): CPU 경쟁/thermal 저하와 `VIO frequency low` 로그를 확인하고, 불필요한 Instant Placement/Augmented Images를 끄며 기기별 자원 차이를 측정. SDK 기능 ON/OFF·thermal·recording 부하가 다른 결과를 solver 우열로 해석하지 않음. 이번 기록에는 장시간 thermal/battery 검증 없음.

## 기록·재생으로 확인한 비동등성

- [Recording/Playback](https://developers.google.com/ar/develop/java/recording-and-playback/developer-guide)는 영상·AR sensor data MP4 재생을 제공하나 추적 결과는 반복 실행마다 달라질 수 있음. 저장한 pose를 그대로 반환하는 deterministic GT player라는 계약이 아님.
- 실제 live↔playback01: CPU image timestamp 공통453개, PGM hash 동일 **0/453**. MP4 압축/변환 영향 후보가 있으나 원인 분리 완료 아님.
- playback01↔02: CPU image timestamp 공통452개, PGM hash **452/452** 동일; 양쪽 TRACKING443개에서 정렬 없는 translation/rotation 차 중앙 **0.009095m/0.104142°**.
- 위 차이는 world gauge·image/frame time차·관측 schedule을 포함. ARCore 오차가9mm라는 정확도 수치로 전용 금지. 같은 MP4를 읽었다는 사실만으로 내부 IMU·solver state·실행 순서 동일성을 입증하지 않음.
- ARCore SDK의 공개 recording 경로는 보유한 임의 PGM+CSV를 단순 함수 인자로 넣는 Mobile-SLAM replay API와 다름. 세 엔진 비교를 위해 SDK-native MP4와 외부 raw capture의 byte/time/calibration 대응표부터 만들어야 함.

## 현재 프로젝트에 적용할 우선순위

| 순서 | 참고할 ARCore 설계 | 현재 확인된 기반·빈틈 | 후속 수락 증거 |
| --- | --- | --- | --- |
| P0 | physical camera / display / sensor pose 분리 | current frame/epoch/sourceTiming 경계는 있음; 실제 K/extrinsic/rolling shutter/exposure 미측정 | 고정 기기·lens·해상도·회전의 K와 `T_B_C`, 각 timestamp 의미·offset, crop 후 K 검증; pose/body frame 명시 |
| P0 | stale frame와 invalid tracking의 명시 계약 | dedup·poseFresh·poseValid 구현됨; 과거 current visual support 소실·jump 기록 남음 | blank/정지/순수회전/병진/가림/복구의 fresh coverage·visual factor·jump/GT 오류를 동시 기록. 상태를 정상으로 완화하여 통과시키지 않음 |
| P1 | CPU image·texture·session 수명 명시 | owned snapshot·epoch fence 있음; resize/pause/reset 조합의 실제 phone 검증 필요 | stale pixel/pose0, release/close 확인, source gap·pose age p95/p99와 메모리 동시 측정 |
| P1 | update API와 내부/렌더 작업 분리 | 자체 stage 계측 있음; 서로 다른 API 시간을 같은 표로 비교할 위험 | source→capture/copy→queue→frontend/backend→published→render 정의 고정, raw trace 제공; stage percentile 합산 금지 |
| P1 | feature 비용·thermal 별도 관리 | feature/solver configuration 가능; bounded desktop 실험이 대부분 | fixed input·기기·전원·thermal 기준, 렌더/recording 부하 별도, 기능별 ON/OFF 실제 invocation 증거 |
| P2 | anchor를 통한 scene 관계 유지 | estimator inverse-depth anchor는 scene-anchor 제품 API와 다름 | underlying world 갱신/재초기화 시 scene continuity 시험; 원시 pose/표시 pose를 함께 저장하여 오차 은폐 방지 |
| P2 | SDK-native replay와 raw observation 연결 | PGM/IMU/manifest 보존·ARCore MP4 반복 증거 있음 | one-to-one timestamp association·동일 gray preprocessing·IMU/calibration parity가 증명되는 subset, independent GT + SE(3) scale1 ATE/RPE/coverage |

- P0/P1 항목은 비교에서 도출한 후속 실험·설계 후보이며 이번에 구현하거나 통과시킨 항목이 아님.
- 비공개 알고리즘의 추정 복사, CPU/GPU/WASM/native라는 실행 형식만으로 성능 순위, 다른 동작의 성공률 비교는 제외.
- 기록 없는 내용은 미확인: ARCore 특징점 알고리즘/상태 차원/IMU noise·bias model/BA window/loop threshold, 자체 엔진 대비 동일 입력 정확도, 세 엔진 실기기 장시간·열·배터리·가림 후 재지역화.
