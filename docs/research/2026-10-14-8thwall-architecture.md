# 8th Wall 소스 아키텍처 상세 분석 — 2026-10-14

- 분석 대상: 공식 `8thwall/8thwall` commit `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd`.
- 로컬 원본: [공식 core clone](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/assets/references/8thwall/official/8thwall).
- 방법: 실제 TypeScript → Emscripten C export → C++ executor → frontend/추적 → Cap’n Proto → renderer 호출 경로 추적. 각 근거 링크는 위 commit과 줄 범위에 고정.
- 증거 구분: 이 문서는 **소스에서 확인한 동작**과 **이식 실험 가설**을 기록. 공개 바이너리 실행·디스어셈블·성능 측정 결과는 별도 실험 보고서에서 확인.
- 핵심 경계: 공식 README는 framework/core MIT, world SLAM 별도 binary-only 배포를 명시. 공개 `Tracker::track()`는 특징 추출·이미지 타깃 탐색 뒤 `tamExtrinsic_`를 반환하고, 그 멤버 초기값은 identity. 공개 트리만으로 상용 world SLAM의 VIO/BA/맵 유지/재위치추정 구현 전체를 확보했다고 볼 수 없음. [S01] [S13] [S14]
- 버전 경계: 공개 core 기본 `loadChunk('slam')`은 `xr-tracking.js`를 가져옴. 기존 독립 배포물의 `xr-slam.js`와 파일명·구현이 같지 않음. 공개 wrapper가 바이너리 내부 전체와 일치한다는 전제 금지. [S03]

## 1. 계층 및 기술 범위

| 계층 | 확인한 실제 역할 | 관련 기술 / 경계 |
|---|---|---|
| 배포 / 시작 | script URL 기준 리소스 경로, XRCC 준비, 선택 chunk preload, `window.XR8`, `xrloaded` | dynamic import, Promise; 최초 이벤트 전에 요청 preload 완료 [S02] [S03] [S04] |
| 플랫폼 / 세션 | WebGL context, 권한, 모듈 제공 session manager → getUserMedia → desktop3d 후보 선택 | 카메라 실패 시 대체 세션 존재; 카메라가 안 보여도 6DoF 성공으로 오인 금지 [S05] |
| 입력 / 프레임 | HTML video currentTime 기반 새 프레임 판정, compute/draw texture 업로드 | WebGL, texture 재사용, repeatFrame [S06] |
| 센서 | deviceorientation quaternion, devicemotion queue, 세 가지 시간축 | Cap’n Proto; 센서 수집과 실제 VIO 융합은 별개 [S07] [S08] [S09] |
| 프레임워크 파이프라인 | module별 GPU/CPU/update/render callback와 lifecycle | 단계별 결과를 module name으로 저장; 같은 프레임 단순 직렬 루프 아님 [S10] [S11] [S12] |
| 특징 frontend | GPU pyramid/filter/score + CPU ORB/GORB descriptor와 ROI | C++, GLSL, PBO 또는 glReadPixels [S15] [S16] [S17] [S18] |
| 공개 tracking | 특징 추출, 이미지 타깃 탐색, pose executor 전달 | 공개 world SLAM solver 없음 [S13] [S14] |
| 이미지 타깃 | planar / curvy 모델, local/global matching, homography/PnP/P2P | 실제 구현 공개; 그 알고리즘을 world SLAM backend로 일반화 금지 [S19] |
| 얼굴 | 전역 검출 + 국소 mesh ROI 추적 + 귀, 이벤트 | TFLite, XNNPACK, 468 mesh points; 별도 WASM [S20] [S21] [S22] [S23] [S46] |
| 하늘 / 레이어 | 영상 축소 → semantic worker → 회전 연계 mask → 합성 | transferable buffer, busy 시 새 추론 생략; 별도 경로 [S24] [S25] [S26] |
| 렌더 / 후처리 | projection/pose 적용, camera feed, layer mask 합성, 좌표계/recenter, hit-test | Three.js 등의 adapter; SLAM 오차 보정과 화면 합성 구분 [S27] [S28] [S29] |
| ECS / Studio | entity/component state, scene, asset loading, physics, XR adapter | FLECS + Three.js + Jolt; SLAM solver가 아님 [S30] [S31] [S32] |
| 녹화 / 미디어 | 브라우저 MP4 → 브라우저 녹화 후 transcode → WASM fallback | media worker는 SLAM worker가 아님 [S33] [S34] [S35] |
| 빌드 | Bazel/Bzlmod, Emscripten, Cap’n Proto, SIMD release, 별도 C++/JS bundle | root npm install만으로 전체 엔진 빌드하는 구조 아님 [S36] [S37] [S38] |

## 2. 시작·데이터 로드·캐시

- `index.ts`: `document.currentScript.src`의 디렉터리를 `ResourceUrls` base로 설정 → `loadJsxr()` → `data-preload-chunks` 파싱 → `Promise.all(loadChunk)` → `window.XR8` → `XRloaded`/`xrloaded` 이벤트. 따라서 JS 다운로드 종료, core WASM 초기화, 선택 module/resource 준비를 서로 구분해야 함. [S02]
- `loadJsxr()`에서 WASM compile/instantiate와 UA client hints, GPU renderer 정보 수집을 병렬 시작. 이후 device info를 Cap’n Proto buffer로 malloc/write/init/free. run loop는 이 Promise를 공유. [S04]
- `chunk-loader.ts`: FaceController가 이미 있으면 face import 생략, XrController가 이미 있으면 tracking import 생략. 문자열 또는 `{name,url}`을 허용하며 override URL 기준으로 해당 chunk 리소스 base도 바꿈. **저장소 전체의 영속 캐시가 아니라 런타임 module 존재 검사**. [S03]
- face 모델은 두 fetch를 먼저 시작한 뒤 WASM 준비를 기다려 bytes를 각 WASM heap에 복사. `_c8EmAsm_loadFaceDetectModel` / `_c8EmAsm_loadFaceMeshModel` 호출 후 임시 malloc 영역 free. ear 모델은 별도 load. [S20]
- semantics 모델도 fetch → arrayBuffer → WASM malloc/write → load 모델 → free. worker가 생성한 heap과 main-thread core heap은 별도. [S24]
- `ResourceUrls`는 얼굴·귀·semantics·media·tablet 파일을 로컬 base 상대경로로 해결. Draco 리소스 resolver는 현재 open source에서 `unsupported()`를 호출하여 throw; 과거 VPS/mesh 다운로드 기능이 현재 배포에서 그대로 사용 가능하다고 볼 수 없음. [S39]
- ECS asset cache는 실제 구현이 다름: manifest path→배포 URL → fetch/blob → object URL; 완료 `assets` Map, 진행 중 `pendingLoads` Map로 중복 load 합침. `clear()`는 object URL revoke와 asset entry 삭제. 관찰 범위에는 persistent IndexedDB/service-worker cache 구현 없음. **XR chunk cache와 ECS asset cache를 혼동하지 않음**. [S32]
- cold/warm 실험은 HTTP cache, ES module cache, Emscripten 초기화, 모델 fetch/compile, ECS Map을 각각 통제해야 함. 같은 page에서 `loadChunk` 재호출만으로 cold-load 속도를 측정할 수 없음.

## 3. 세션·권한·카메라 획득

- `runInternal`: context 생성 → session 후보 생성 → `onBeforeRun` → `onRunConfigure` → 유효 세션 초기화 → WASM/context 대기 → 첫 frame 처리. 각 후보는 `onBeforeSessionInitialize`, initialize, permission, start를 거침. `sessionInitBehavior='fallback'` 여부에 따라 오류/취소 다음 후보 진행을 결정. [S05]
- 세션 후보 순서: module에서 제공한 session manager들 → `SessionManagerGetUserMedia` → `SessionManagerDesktop3D`. 외부 run loop일 때 자체 loop를 제공하는 세션은 skip. 실제 device session 종류 기록이 필요. [S05]
- 카메라 해상도는 무조건 가장 높은 값이 아님. Android 기본 선호 `960×720`, 일부 crop 문제가 있는 Pixel 계열 `1280×720` 등을 먼저 시도, iOS `min 960×720` 이후 `640×480`, SamsungBrowser 별도 후보. 이는 source의 device heuristic이며 현재 모든 기기에 유효한 표준값이 아님. [S40]
- 여러 rear camera의 잘못된 FOV 선택을 피하기 위해 `enumerateDevices`, label, 특정 제조사/deviceId 규칙 사용. resolution 선택과 calibration 일관성을 같이 다루는 구조. 현 Mobile-SLAM에 기기 목록을 그대로 복사할 근거는 없음. [S40]
- `hasNewFrame()`은 video pause/0 크기 또는 `video.currentTime === videoTime_`이면 repeat. `requestVideoFrameCallback`의 capture metadata를 사용하는 경로로 해석하면 안 됨. [S06]
- 새 프레임만 `texImage2D(video)` 호출. compute texture와 draw texture에 각각 업로드하고 draw GL texture binding / UNPACK_FLIP_Y 상태를 보존. 즉 관찰 경로에서 **영상 전체의 JS CPU RGBA 복사는 피하지만 두 GL context 업로드가 존재**. “완전 zero-copy”는 입증되지 않음. [S06]
- `CameraLoop.drawTexForComputeTex()`는 compute texture name에 대응하는 draw texture를 캐시하며, frame release를 repeat 여부와 결합. 카메라 이미지만 최신 프레임으로 무조건 교체하면 pose와 texture 대응이 깨질 수 있음. [S10]

## 4. 실제 frame 시간 순서와 callback 계약

```text
RAF n
  pre-render:
    video/canvas 크기 변경 이벤트
    onProcessCpu(previous staged input)
    onUpdate(previous output + associated texture)
    readCameraToTexture(current video frame)
  render:
    onRender()
  post-render:
    onProcessGpu(current frame; stage C++ + draw pyramid)
    lastFrameTime 갱신

Tracking GPU stage:
  repeat 검사 → sensors.setEventQueue → stageFrame(texture, dimensions,
  orientation, Euler, quaternion, videoTime, frameTime, timeNanos, GPS)
  → GlRealityFrame.draw(DEFER_READ)
  → 이전 GPU buffer read → 이번 GPU pyramid draw

Tracking CPU stage:
  processStagedFrame → processPipeline → XREngine.execute
  → RealityResponse buffer + 대응 texture 노출
  → JS Cap’n Proto parse → tracking/image events + pose/intrinsics return
```

- 위 순서는 함수 본문 확인 결과. “onProcessGpu→onProcessCpu→onUpdate→onRender가 같은 RAF에서 같은 입력을 처리”한다는 설명은 부정확. 실제 source comment도 GPU d0; 다음 RAF r0,d1; CPU 0의 지연 구조를 명시. 다중 단계 때문에 최종 frame age는 실행 계측으로 확인해야 함. [S11] [S15]
- `runCallbacks`는 등록 callback를 reduce 순서로 호출하고 반환값을 module name key로 저장. 결과는 `processGpuResult`, `processCpuResult`, `updateResults`로 전달되며 단계 뒤 framework events dispatch. Promise를 await하는 frame callback 경로가 아니므로 긴 main-thread callback은 나머지 모듈과 렌더를 막음. [S10]
- 모듈 lifetime과 session lifetime이 분리됨. attach/pause/resume/remove 상태에 따라 새 module의 시작 callback를 늦추거나 즉시 호출하며 remove 시 이미 attached된 module만 detach. [S12]
- repeat 프레임에는 새 tracking stage/process를 하지 않지만 기존 response를 반환할 수 있음. renderer FPS, video FPS, unique tracking output FPS를 따로 세어야 함. [S06] [S16]
- core default `ownRunLoop=true`, `webgl2=true`; ECS mobile path는 자체 loop를 위해 `ownRunLoop=isHeadset`를 전달. 설정만으로 모든 실행을 같은 RAF 구조라고 단정하지 않고 session/host loop를 확인. [S04] [S31]

## 5. IMU·시간·좌표 계약

| 값 | JS 수집 | C++ 소비 / 주의 |
|---|---|---|
| orientation | alpha=z, beta=x, gamma=y degrees를 quaternion 변환 | stage 당시 마지막 device orientation 사용; source TODO는 orientation event timestamp 전달 누락을 인정 [S08] [S16] |
| motion wall clock | `performance.now()*1e6` | `getTimestampNanos()`를 `eventTimeNanos`로 사용 [S07] [S09] |
| motion event clock | `event.timeStamp*1e6`도 별도 직렬화 | 이 `prepareTrackingSensorFrame` 경로가 해당 event timestamp를 선택하는 것은 아님 [S07] [S09] |
| interval | JS interval × 1e6 | source는 iOS ×1000 보정, Android 16,000,000→16,666,667 ns 보정 [S09] |
| video clock | video.currentTime seconds | ns 변환 후 보관하나 frame clock과 별도 시간축이라고 source에서 명시 [S09] [S15] |
| frame clock | upload/read 시 `performance.now()/1000` | 우선 `frameTimestampNanos`, 0이면 stage timestamp 사용 [S09] |
| stage clock | GPU callback에서 performance.now ns | frame 읽기 이후 호출 지연 포함 [S16] |

- motion queue는 linear acceleration, acceleration including gravity, rotationRate를 종류별 보관. pause/uninitialized 시 무시, queue oldest가 현재보다 5초 이상 뒤처지면 clear, stage 직전 일괄 직렬화 후 queue clear. sample interpolation/경계 분할이 JS에 구현되어 있다는 증거는 없음. [S07]
- C++는 portrait sensor 회전, iOS/Android 가속도 부호, gyro 부호를 적용하고 event를 sort. **legacy heuristic이 현 기기 측정값과 같다는 보장은 없음**. 특히 Mobile-SLAM의 rad/s 계약에 raw alpha/beta/gamma를 그대로 이식 금지; source의 해당 구간에 단위 변환이 없고 world VIO 내부는 비공개. [S09]
- quaternion 변환·센서 큐의 존재만으로 tightly coupled VIO, preintegration, EKF 또는 sliding-window optimizer 사용을 증명하지 못함.

## 6. GPU/CPU 특징 추출과 world SLAM 경계

- `stageFrame`은 frame별 texture ID, capture 크기/회전, pose, 3개 timestamp, event queue, GPS를 `PipelineData`에 저장하고 `GlRealityFrame::draw(DEFER_READ)` 호출. CPU 소비는 queue earliest를 active로 이동 후 response bytes/texture를 함께 노출. [S15]
- GPU 영상 처리: 필요하면 입력 prefilter → 초기 pyramid draw → 3회 blur/decimate → 선택적 alternate gradient/moment score → stage2/stage3 shader. GL state save/restore 경로 존재. [S17]
- WebGL2 pixel buffer 사용 시 `readPixels(..., offset=0)`으로 PIXEL_PACK_BUFFER에 요청, fence 기록; 다음 read에서 `getBufferSubData`로 WASM heap view에 읽음. fallback은 `glReadPixels` 직접 CPU. fence를 긴 busy wait하는 코드는 주석 처리됨; 무조건 비동기·무차단이라고 할 수 없음. [S17]
- `FeatureDetector::detectFeatures`는 grayscale CPU fallback `Gr8Cpu.detectAndCompute`, GPU pyramid가 있으면 `Gr8Gl.detectAndCompute` 및 ROI variant 사용. **Gr8Gl이라는 이름만으로 descriptor 전체가 GPU라고 분류하면 틀림**. shader 전처리 결과에 대해 C++가 keypoint/angle/descriptor를 구성. [S18]
- ORB angle은 영상 intensity center-of-mass; GORB angle은 device pose로 ray를 world에 올린 뒤 up 방향 epsilon을 투영해 계산. gravity region별 ORB/GORB 선택 가능, image target이 있으면 양쪽 descriptor 활성화. 공개 tracker default는 GORB, 낮은 두 gravity portion에는 ORB 추가. [S13] [S41] [S48]
- `retainBest`는 response 정렬과 spatial bin collision count를 이용해 균일 분포 선택. 텍스처가 강한 한 영역에만 feature가 몰리는 것을 제한하려는 구현으로 해석 가능하나 Mobile-SLAM 정확도 향상은 별도 실험 필요. [S41]
- 공개 실행 chain: `processPipelineData → XREngine::execute → XRRequestExecutor::execute → PoseRequestExecutor → updateCameraPoseImu → Tracker::track`. 이어 lighting/camera/feature-set/image-detection executor를 mask에 따라 수행. [S15] [S42] [S43]
- `Tracker::track`의 공개 본문에는 world map triangulation/BA/loop closure update가 없음. image target locate 뒤 frame count 증가, `tamExtrinsic_` 반환. `tamExtrinsic_`는 header에서 identity로 초기화. 이 tree의 함수명·주석·deprecated schema만 보고 closed world solver를 재구성하면 안 됨. [S13] [S14]
- ARKit/ARCore/Tango request 처리 분기는 실제 존재. 그러나 browser getUserMedia 경로가 그 센서 request를 공급한다는 증거와는 다름. ARCore lost 상태에서는 기존 위치와 IMU 3DoF 회전 fallback이 있지만 이를 8th Wall 웹 SLAM 자체의 recovery라고 부를 수 없음. [S43]
- 공개 tree에는 `bundle-residual.h`의 CameraMovement, RotationTarget, ZeroTarget, reprojection 등의 **최적화 구성 수식**이 존재. 이는 world tracker 전체 구현 공개와 다른 범위. 별도 [바이너리 분석](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/docs/research/2026-10-14-8thwall-binary.md)은 frame export에서 Ceres 관련 routine·BA 설정 getter의 정적 직접 도달과 RTTI/vtable 대응을 확인하고, 원본 Code section이 같은 분석 사본에서 잔차/Jacobian 3종×2조건 **6/6** 수치 검사를 수행. 실제 세션의 branch 선택·유효 weight·iteration·최적화 수렴은 미측정. [S58]
- 공개 소스만으로 완전히 알 수 없는 world solver 항목: initialization observability/scale, bias 추정, IMU preintegration, rolling shutter, robust cost, window/keyframe 조건, marginalization, bundle adjustment, point culling, relocalization, loop closure, map persistence, pose prediction. BA/Ceres와 일부 residual은 위 별도 분석에서 부분 입증되었으나, 전체 orchestration과 나머지 항목은 추가 binary/입력-출력 실험 필요.

## 7. 이미지 타깃 — 실제 공개 알고리즘

- target loading은 metadata/이미지 fetch → texture upload → C++ detection image loader의 직렬 처리 → geometry/feature 준비 → engine 등록. 새 batch ID는 진행 중 이전 batch를 취소하며, 기존 target과 새 target의 diff를 유지. target 바이트 로드와 실제 인식 성공을 분리해야 함. [S44] [S55]
- 추적 입력은 global pyramid features와 예측 target ROI features. local mode는 이전 target pose에서 image ray를 target 좌표계로 투영 → scale/descriptor/radius filter local matching → pose solve. [S19]
- planar target: `solveImageTargetHomography`; curvy target: target geometry의 3D point + camera rays로 `solvePnP`. 회전 변화가 충분하면 작은 검색 반경으로 다시 매칭하여 pose refinement. [S19]
- global recovery/search: globalMatcher → ORB target이면 `robustPnP`; GORB target이면 gravity constraint를 이용하는 `robustP2P`. 이때 source의 RANSAC 10–200 iteration, initial inlier 0.3 등의 값은 **image-tracking 해당 분기**의 설정일 뿐 world SLAM 값이 아님. [S19]
- 출력은 target pose/scale/ROI와 name별 map. 새 name `imagefound`, 지속 `imageupdated`, 현재 frame에서 사라진 name `imagelost`. 이미지 타깃 검출 실패와 world camera tracking 실패는 별도 event/status로 다루어야 함. [S16]

## 8. 얼굴·귀 ML 파이프라인

- 별도 `xr-face.js` chunk와 WASM, face detector/mesh/ear 모델. stage hook은 texture ID를 C++로 전달, CPU hook은 staged face frame을 처리한 뒤 `FaceResponse`를 읽고 found/updated/lost 및 expression 관련 event를 전달. [S03] [S20] [S21]
- `FaceTracker::track`: 이전 ROI가 있으면 local mesh detector로 분석 → 결과에서 다음 face 영역 유지 → local face 수가 max 미만이면 global detector 사용 → bounding-box overlap/거리로 global/local/lost ID 연계 → 3D face geometry로 변환. source에는 병행 pipeline 사이 global detection flicker 주의사항도 남아 있음. [S22]
- mesh 모델 출력 설명은 **468 xyz points**. 기존 로컬 archive README의 “28 랜드마크”를 full face mesh 개수로 채택하지 않음. 귀 모델·귀 결과는 face ID 연계의 추가 경로. [S22] [S23]
- TFLite wrapper는 FlatBuffer model bytes를 자체 복사, custom transform ops 등록, XNNPACK delegate, weight cache, `AllocateTensors` 사용. XNNPACK thread count 적용은 native 또는 Emscripten pthread build 조건부. “TensorFlow.js GPU 모델 실행”이라는 요약은 실제 이 경로와 다름. [S46]
- 실제 얼굴 후처리 chain: `TrackedFaceState::locateFace → detectionToRaySpace → FacePointsFilter::apply → point별 RayPointFilter3 → computeAnchorTransform → worldPoints → poseEstimationFull3d(subset) → normals/attachment/expression`. ray point filter는 XY 오차에서 EMA velocity를 계산하고 `a=1/(1+1/(minCutoff+vScale*(abs(vx)+abs(vy))))`, `filtered=a*raw+(1-a)*previous`를 XYZ에 적용. 얼굴 landmark 적응형 저역통과이며 world camera pose filter로 입증된 것이 아님. [S49] [S50]
- face renderer ROI/mesh 안정화와 추정 head pose는 world SLAM map/metric trajectory 성능으로 평가하지 않음. face 모델 hash/shape 검증 및 visual tracking 실험은 별도 증거가 필요.

## 9. Sky Effects·semantics·화면 후처리

- main thread는 영상 texture를 작은 inference 입력으로 만들고, worker는 TFLite semantic model을 수행. worker ready flag가 false이면 새 inference를 보내지 않음. 무한 frame queue 대신 처리 가능한 시점의 새 입력 선택. [S24] [S25]
- worker invoke는 256×144 인자를 C++에 넘기며 main thread는 orientation에 따라 rotate/letterbox하여 고정 portrait 입력을 만듦. source 상단의 옛 144×192 주석보다 실제 constants/호출을 우선해야 함. [S24] [S25]
- transfer list를 통해 입력 RGBA ArrayBuffer와 출력 Float32Array buffer 소유권 이동. worker 결과는 해당 frame rotation과 함께 반환되어 current pose에 맞춘 mask/cubemap rendering에 사용. 오래된 추론 결과를 현재 frame에 단순 덮어씌우는 구조가 아님. [S24] [S25] [S26]
- model threshold 0.4는 worker에서 valid pixel percentage를 계산하는 값. 이것을 전체 segmentation 모델 정확도나 최종 compositing alpha의 유일 임계값으로 해석하지 않음. [S24]
- layer camera rendering은 결과 pose/projection에 맞춘 별도 Three.js layer scene을 texture로 렌더하고 mask와 함께 GlTextureRenderer 합성에 제공. GL state restore로 애플리케이션 scene state 간섭을 줄임. [S25] [S27]
- sky mask 실제 GPU 후처리: `drawOutputSemantics(cameraExtrinsics,texId,...) → direct current mask 또는 cached cubemap render → 작은 FBO downscale → 세로 Gaussian → 가로 Gaussian → renderSkyTexture`. edgeSmoothness를 `1-(1-edgeSmoothness)^2`로 변환해 raw mask와 blurred edge를 혼합. 이는 공간 mask smoothing이며 SLAM pose smoothing이 아님. [S51]
- “후처리” 구분: (1) scene coordinate/recenter, (2) target pose refinement, (3) face/semantic geometry와 mask, (4) visual compositing, (5) 녹화 frame 합성. 이 모든 것을 SLAM pose smoothing이라고 부를 수 없음.

## 10. 결과·렌더·질의

- JS tracking 결과: `rotation`, `position`, `intrinsics`, 대응 `realityTexture`, tracking status/reason, optional worldPoints/lighting, detectedImages. worldPoints는 INITIALIZING에서 비워지는 경로, surfaces 반환은 현재 source에서 주석 처리. [S16]
- tracking status event는 status 또는 reason이 바뀔 때만 발생. frame별 status를 수집하려면 callback 반환값도 기록해야 함. event count를 frame coverage로 쓰면 안 됨. [S16]
- Three.js adapter는 projection 16개 요소 복사, inverse 갱신, quaternion/position 직접 적용. 이 구간에는 범용 temporal pose smoothing이 없음. scene 안정성이 이 renderer의 보간 효과인지 바이너리 pose의 안정성인지 별도 확인 필요. [S27]
- 엔진 출력 순서는 내부 request portrait preprocess → executor → **raw response를 내부 state에 저장** → display orientation intrinsics/extrinsics postprocess → texture rotation → axes/mirror/recenter/scale 변환. 따라서 public application pose와 다음 프레임 내부 상태 pose의 좌표계는 구분해서 비교해야 함. [S52] [S53]
- pose prediction/temporal smoothing: 조사한 공개 world pose executor·tracking JS·Three.js update chain에서 display time까지 extrapolate하거나 world pose를 Kalman/One-Euro/slerp로 필터하는 활성 호출은 확인하지 못함. delta/velocity 계산 자체는 prediction 적용 증거가 아님. 바이너리 내부 여부는 미확인.
- 초기 좌표와 pose를 ground-plane/yaw 및 initial transform 기준으로 recenter하고 delta/velocity를 별도 계산. 실제 C++ stage 첫 프레임들에서 origin 고정/recenter 단계도 있음. “첫 pose가 0이다”만으로 stationary tracking 정확도를 주장하지 않음. [S15] [S28]
- hitTest는 Cap’n Proto query → 이전 engine request/response snapshot → mirror/orientation 보정 → feature set 보완 → feature/estimated/detected surface test → coordinate postprocess. raycasting 구현과 실제 map/surface를 생산하는 solver는 별개. [S29] [S47]
- 공개 tracking controller의 `setNewMeshesFromJs`는 빈 본문. resource Draco resolver 또한 unsupported. schema/API 이름에 mesh/VPS가 남아 있어도 live feature 가용성은 별도 검증 필요. [S15] [S39]

## 11. ECS·asset·media 계층

- ECS는 FLECS state, Three.js rendering, Jolt physics 위의 Studio game engine. XR pipeline은 그 scene/camera를 연동하는 adapter. [S30]
- world-camera pipeline 구성은 XR8 lazy load → slam chunk → camera-feed renderer 및 CloudStudioThreejs module → XrController → configure → run. ECS quaternion을 XR8 좌표로 변환할 때 Y 180° flip 적용. camera entity/scene과 tracking coordinate frame을 혼동하면 결과가 맞아 보여도 부호가 틀릴 수 있음. [S31]
- asset manager는 fetch/object URL/cache 기능, effects manager는 fog와 sky object를 갱신하고 XR 시 sky를 숨기는 역할. “모든 효과가 SLAM postprocess”가 아님. [S32] [S45]
- recorder 선택 우선순위는 direct native MP4 → native 녹화 후 transcode → WASM recorder. direct 지원 판단은 iOS와 video/mp4·audio/mp4 capability를 함께 봄. media worker는 lazy singleton이며 blob bootstrap에서 `importScripts`로 worker module 로드. [S33] [S34] [S35]
- app viewport/overlay/camera와 audio mixing, recording은 tracking CPU/GPU budget을 추가 소비할 수 있으므로 성능 비교 실험에서 recorder 활성 여부를 고정해야 함.
- webXR hand/controller rendering source가 존재하더라도 모바일 RGB 기반 hand-tracking 모델과 같은 기능이라는 증거는 아님. 이번 source 분석은 WebXR controller 계층의 존재 확인까지이며 hand model 전 경로 분석·실행은 미완료.

## 12. dependency·빌드 구조

- root `package.json`은 ESLint editor 연결용; 의존성은 Bazel npm package rule과 각 package에 분산. engine prepack은 version 파일 작성 → `bazel build //reality/app/xr/js:bundle --config=wasmreleasesimd` → bundle unzip. 이 명령을 여기서 실행하거나 build 성공으로 보고하지 않았음. [S36] [S54]
- 공식 WASM toolchain은 `-std=c++20`, `-DJAVASCRIPT`, `-fno-exceptions`를 지정. Mobile-SLAM C++17 계약과 다른 부분이므로 소스 직접 이식 시 language feature 검토 필요. WORKSPACE는 Emscripten 3.1.24 및 SDK/cache archive hash를 고정. 이것은 clone build recipe의 버전이며 다운로드한 SLAM 바이너리 compiler 버전으로 자동 승격하지 않음. [S56] [S57]
- tracking WASM target: MODULARIZE + EXPORT_ES6, WebGL2 backwards emulation, malloc/free와 GL/writeArray runtime export, memory growth helper, 초기 16 MB. default single-file=1, pthread branch single-file=0. **소스의 build 옵션과 배포된 바이너리 flags는 별도 증거**. [S37]
- MODULE.bazel에 TensorFlow 2.11.0 override와 patched archive/hash, Abseil, zlib, protobuf 등 pinned dependency 선언. Cap’n Proto는 JS/C++ 경계 schema와 코드 생성의 핵심; `.capnp` 정의를 실제 source와 비교할 수 있어 과거 minified wrapper 추정 schema보다 강한 근거. [S38]
- source의 pthread 분기, ThreadPool 클래스, worker bootstrap이 있다는 사실만으로 현재 공개 engine이 SharedArrayBuffer/pthread로 SLAM을 worker에서 돌린다고 단정하지 않음. 실제 downloaded artifact의 import/memory/build fingerprint와 runtime thread 검증이 필요.
- SIMD 빌드 구성, XNNPACK dependency, WebGL2 API 존재도 해당 실험에서 SIMD/kernel/GPU device가 실제 활성화되었다는 증거와 구분.

## 13. Mobile-SLAM에 적용할 실험 가설

| 우선순위 | source에서 얻은 가설 | 필요한 대조 실험 / 수락 관찰 | 아직 주장하지 않는 것 |
|---|---|---|---|
| P0 | capture/queue/stage/process/render timestamp와 texture ID를 유지 | 동일 입력으로 각 pose가 어떤 frame/IMU interval에 대응하는지 one-to-one 검사; p50/p95/p99 age와 fresh coverage | 고 FPS = 낮은 pose age |
| P0 | duplicate video frame과 renderer frame 분리 | 입력 frame count / unique processed / output coverage / display count 분리, duplicate injection 대조 | repeated pose도 새 visual update |
| P0 | portrait/crop/FOV/calibration 동시 관리 | 실제 폰 rear camera별 K, 해상도/crop/회전 변환, IMU 축 단위 측정 | 8th Wall device table 복사만으로 calibration 완료 |
| P1 | GPU pyramid와 readback를 프레임 사이에 분산 | sync readback vs PBO/deferred A/B; 실제 GPU, copy/queue/frontend 비용, accuracy/age 동시 평가 | deferred가 항상 빠름·정확함 |
| P1 | GORB gravity orientation 및 공간 feature 분포 | 고정 frame/GT/동일 keypoint budget에서 ORB vs gravity-oriented descriptor, 회전/중력 오차 sweep | world VIO algorithm 동일 |
| P1 | 비싼 보조 모델의 최신 입력 우선·busy frame skip | worker busy 시간, drop/age/renderer stall 측정; 기존 pose/IMU loss 없이 queue bound | SLAM 상태 추정에도 무조건 frame drop 허용 |
| P2 | output texture와 pose가 같은 pipeline frame에 결합 | visible camera/pose skew 측정, 외부 loop와 recorder on/off 대조 | screenshot 안정성 = metric accuracy |
| P2 | recovery 평가를 초기화/정상/재획득별 분리 | low texture/occlusion/rotation 후 coverage, jump, SE(3) scale=1 ATE/RPE 및 reacquire time | 공개 binary가 어떤 solver를 쓰는지 확정 |

- 위 항목은 구현 변경 승인·성능 수락이 아니라 실험 후보. 이번 소스 분석은 Mobile-SLAM runtime 코드를 수정하지 않았음.
- 독립 GT, 같은 입력, 센서 시간/좌표 정합이 없으면 두 엔진 출력 trajectory 차이는 상대 관찰일 뿐 정확도 우열 근거가 아님.

## 14. 기존 분석물에서 승격하면 안 되는 주장

| 기존 문구 유형 | 이번 판정 |
|---|---|
| “SLAM 완전 복원”, “ORB→PnP→VIO 전체 확인” | open frontend/image-target 증거와 binary world solver 증거를 분리. 전체 복원 아님 |
| “EKF / IMU preintegration / BA / BoW loop closure 전체 복원” | 공개 source만으로는 부족. 별도 binary 보고서에서 Ceres/BA code 도달·일부 잔차 수치 실행은 부분 입증; EKF/IMU/loop closure 및 전체 solver는 여전히 미복원 |
| “face mesh 28 landmarks” | 실제 공개 local detector 출력 설명 468 xyz. 28이 어느 subset인지 별도 입증 필요 |
| “ARKit/LiDAR가 iOS Safari 기본 처리” | request schema/branch 존재만 확인; 실제 웹 입력/네이티브 세션 공급 미입증 |
| “Draco / VPS / mesh API가 존재하므로 지원” | unsupported resolver와 빈 stub 확인. 실행 가용성 따로 검증 |
| “worker 기반 전체 SLAM” | semantics/media explicit worker와 tracking main-thread hooks 구분 |
| “매 frame GPU→CPU→render 순서” | 프레임을 가로지르는 staging/processing/render 순서를 기록해야 함 |

- 분석 한계: 전체 repo의 모든 helper·editor UI·network endpoint·binary 함수에 대한 전수 검토는 아님. 이 문서는 웹 runtime의 주요 종단 호출 경로와 공개 알고리즘·부수 계층을 상세 매핑한 것. 실행 검증/바이너리 함수 복원/현장 KPI는 별도 상태로 유지.

## 근거 링크

- 아래 모든 core source 링크는 `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd` 고정. S 번호는 각 행의 source path와 줄 범위에 대응.

[S01]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/README.md#L1-L31
- **S01** — [README.md:1–31](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/README.md#L1-L31)

[S02]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/index.ts#L12-L33
- **S02** — [reality/app/xr/js/index.ts:12–33](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/index.ts#L12-L33)

[S03]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/chunk-loader.ts#L14-L59
- **S03** — [reality/app/xr/js/src/chunk-loader.ts:14–59](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/chunk-loader.ts#L14-L59)

[S04]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/jsxr.ts#L42-L135
- **S04** — [reality/app/xr/js/src/jsxr.ts:42–135](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/jsxr.ts#L42-L135)

[S05]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/camera-loop.ts#L827-L948
- **S05** — [reality/app/xr/js/src/camera-loop.ts:827–948](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/camera-loop.ts#L827-L948)

[S06]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/session-manager-getusermedia.ts#L482-L556
- **S06** — [reality/app/xr/js/src/session-manager-getusermedia.ts:482–556](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/session-manager-getusermedia.ts#L482-L556)

[S07]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/sensors.ts#L16-L85
- **S07** — [reality/app/xr/js/src/sensors.ts:16–85](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/sensors.ts#L16-L85)

[S08]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/quaternion.ts#L8-L39
- **S08** — [reality/app/xr/js/src/quaternion.ts:8–39](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/quaternion.ts#L8-L39)

[S09]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/tracking-sensor-event.cc#L86-L173
- **S09** — [reality/engine/tracking/tracking-sensor-event.cc:86–173](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/tracking-sensor-event.cc#L86-L173)

[S10]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/camera-loop.ts#L296-L465
- **S10** — [reality/app/xr/js/src/camera-loop.ts:296–465](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/camera-loop.ts#L296-L465)

[S11]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/camera-loop.ts#L479-L565
- **S11** — [reality/app/xr/js/src/camera-loop.ts:479–565](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/camera-loop.ts#L479-L565)

[S12]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/lifecycle.ts#L3-L65
- **S12** — [reality/app/xr/js/src/lifecycle.ts:3–65](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/lifecycle.ts#L3-L65)

[S13]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/tracker.cc#L55-L135
- **S13** — [reality/engine/tracking/tracker.cc:55–135](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/tracker.cc#L55-L135)

[S14]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/tracker.h#L34-L89
- **S14** — [reality/engine/tracking/tracker.h:34–89](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/tracker.h#L34-L89)

[S15]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/tracking-controller.cc#L288-L742
- **S15** — [reality/app/xr/js/tracking-controller.cc:288–742](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/tracking-controller.cc#L288-L742)

[S16]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L439-L793
- **S16** — [reality/app/xr/js/src/tracking-controller.ts:439–793](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L439-L793)

[S17]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/gl-reality-frame.cc#L484-L670
- **S17** — [reality/engine/features/gl-reality-frame.cc:484–670](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/gl-reality-frame.cc#L484-L670)

[S18]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/feature-detector.cc#L37-L108
- **S18** — [reality/engine/features/feature-detector.cc:37–108](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/feature-detector.cc#L37-L108)

[S19]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/imagedetection/image-tracking-state.cc#L61-L253
- **S19** — [reality/engine/imagedetection/image-tracking-state.cc:61–253](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/imagedetection/image-tracking-state.cc#L61-L253)

[S20]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/faces/load-models.ts#L6-L46
- **S20** — [reality/app/xr/js/src/faces/load-models.ts:6–46](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/faces/load-models.ts#L6-L46)

[S21]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/face-controller.ts#L923-L1068
- **S21** — [reality/app/xr/js/src/face-controller.ts:923–1068](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/face-controller.ts#L923-L1068)

[S22]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/faces/face-tracker.cc#L86-L375
- **S22** — [reality/engine/faces/face-tracker.cc:86–375](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/faces/face-tracker.cc#L86-L375)

[S23]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/faces/face-detector-local.cc#L35-L99
- **S23** — [reality/engine/faces/face-detector-local.cc:35–99](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/faces/face-detector-local.cc#L35-L99)

[S24]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/semantics/semantics-worker.js#L17-L155
- **S24** — [reality/app/xr/js/src/semantics/semantics-worker.js:17–155](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/semantics/semantics-worker.js#L17-L155)

[S25]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/layers-controller.ts#L989-L1101
- **S25** — [reality/app/xr/js/src/layers-controller.ts:989–1101](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/layers-controller.ts#L989-L1101)

[S26]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/layers-controller.ts#L633-L731
- **S26** — [reality/app/xr/js/src/layers-controller.ts:633–731](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/layers-controller.ts#L633-L731)

[S27]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/threejs-renderer.ts#L163-L237
- **S27** — [reality/app/xr/js/src/threejs-renderer.ts:163–237](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/threejs-renderer.ts#L163-L237)

[S28]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/pose/pose-request-executor.cc#L279-L356
- **S28** — [reality/engine/pose/pose-request-executor.cc:279–356](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/pose/pose-request-executor.cc#L279-L356)

[S29]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/executor/xr-engine.cc#L88-L160
- **S29** — [reality/engine/executor/xr-engine.cc:88–160](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/executor/xr-engine.cc#L88-L160)

[S30]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/packages/ecs/README.md#L1-L9
- **S30** — [packages/ecs/README.md:1–9](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/packages/ecs/README.md#L1-L9)

[S31]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/c8/ecs/src/runtime/xr/xr-camera-pipeline.ts#L66-L189
- **S31** — [c8/ecs/src/runtime/xr/xr-camera-pipeline.ts:66–189](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/c8/ecs/src/runtime/xr/xr-camera-pipeline.ts#L66-L189)

[S32]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/c8/ecs/src/runtime/asset-manager.ts#L42-L118
- **S32** — [c8/ecs/src/runtime/asset-manager.ts:42–118](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/c8/ecs/src/runtime/asset-manager.ts#L42-L118)

[S33]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/mp4-recorder.js#L83-L101
- **S33** — [reality/app/xr/js/src/mp4-recorder.js:83–101](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/mp4-recorder.js#L83-L101)

[S34]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/media/direct-recorder.js#L4-L24
- **S34** — [reality/app/xr/js/src/media/direct-recorder.js:4–24](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/media/direct-recorder.js#L4-L24)

[S35]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/media/worker.ts#L3-L32
- **S35** — [reality/app/xr/js/src/media/worker.ts:3–32](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/media/worker.ts#L3-L32)

[S36]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/packages/engine/tools/prepack.sh#L1-L14
- **S36** — [packages/engine/tools/prepack.sh:1–14](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/packages/engine/tools/prepack.sh#L1-L14)

[S37]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/BUILD#L308-L371
- **S37** — [reality/app/xr/js/BUILD:308–371](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/BUILD#L308-L371)

[S38]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/MODULE.bazel#L24-L87
- **S38** — [MODULE.bazel:24–87](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/MODULE.bazel#L24-L87)

[S39]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/resources.ts#L17-L37
- **S39** — [reality/app/xr/js/src/resources.ts:17–37](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/resources.ts#L17-L37)

[S40]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/constraints-helper.ts#L19-L201
- **S40** — [reality/app/xr/js/src/constraints-helper.ts:19–201](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/constraints-helper.ts#L19-L201)

[S41]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/gr8gl.cc#L135-L328
- **S41** — [reality/engine/features/gr8gl.cc:135–328](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/gr8gl.cc#L135-L328)

[S42]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/executor/xr-request-executor.cc#L44-L147
- **S42** — [reality/engine/executor/xr-request-executor.cc:44–147](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/executor/xr-request-executor.cc#L44-L147)

[S43]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/pose/pose-request-executor.cc#L111-L275
- **S43** — [reality/engine/pose/pose-request-executor.cc:111–275](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/pose/pose-request-executor.cc#L111-L275)

[S44]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L800-L930
- **S44** — [reality/app/xr/js/src/tracking-controller.ts:800–930](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L800-L930)

[S45]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/c8/ecs/src/runtime/effects-manager.ts#L14-L114
- **S45** — [c8/ecs/src/runtime/effects-manager.ts:14–114](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/c8/ecs/src/runtime/effects-manager.ts#L14-L114)

[S46]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/deepnets/tflite-interpreter.cc#L34-L78
- **S46** — [reality/engine/deepnets/tflite-interpreter.cc:34–78](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/deepnets/tflite-interpreter.cc#L34-L78)

[S47]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/hittest/hit-test-performer.cc#L156-L286
- **S47** — [reality/engine/hittest/hit-test-performer.cc:156–286](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/hittest/hit-test-performer.cc#L156-L286)

[S48]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/gr8gl.cc#L1074-L1108
- **S48** — [reality/engine/features/gr8gl.cc:1074–1108](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/gr8gl.cc#L1074-L1108)

[S49]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/faces/tracked-face-state.cc#L52-L153
- **S49** — [reality/engine/faces/tracked-face-state.cc:52–153](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/faces/tracked-face-state.cc#L52-L153)

[S50]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/ray-point-filter.cc#L39-L82
- **S50** — [reality/engine/tracking/ray-point-filter.cc:39–82](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/tracking/ray-point-filter.cc#L39-L82)

[S51]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/semantics/semantics-cubemap-renderer.cc#L811-L917
- **S51** — [reality/engine/semantics/semantics-cubemap-renderer.cc:811–917](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/semantics/semantics-cubemap-renderer.cc#L811-L917)

[S52]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/executor/xr-engine.cc#L178-L234
- **S52** — [reality/engine/executor/xr-engine.cc:178–234](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/executor/xr-engine.cc#L178-L234)

[S53]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/orientation.cc#L456-L482
- **S53** — [reality/engine/geometry/orientation.cc:456–482](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/orientation.cc#L456-L482)

[S54]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/package.json#L1-L3
- **S54** — [package.json:1–3](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/package.json#L1-L3)

[S55]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L1111-L1180
- **S55** — [reality/app/xr/js/src/tracking-controller.ts:1111–1180](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L1111-L1180)

[S56]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/bzl/crosstool/wasm-cc-toolchain.bzl#L291-L310
- **S56** — [bzl/crosstool/wasm-cc-toolchain.bzl:291–310](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/bzl/crosstool/wasm-cc-toolchain.bzl#L291-L310)

[S57]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/WORKSPACE#L368-L394
- **S57** — [WORKSPACE:368–394](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/WORKSPACE#L368-L394)

[S58]: https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle-residual.h#L253-L354
- **S58** — [reality/engine/geometry/bundle-residual.h:253–354](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle-residual.h#L253-L354)
