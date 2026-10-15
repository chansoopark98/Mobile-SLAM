# Mobile-SLAM / 8th Wall / ARCore 웹·화면 경로 비교 — 2026-10-14

- 조사 목적: 느린 반응·추적 불안정의 원인을 **입력/엔진/전달/화면**으로 분리. 화면 smoothing이나 FPS 숫자를 SLAM 정확도 증거로 사용하지 않음.
- Mobile-SLAM HEAD: `5d48e4bd7466842dad98928fb1f12988488402df`; 조사 대상 tracked 제품 파일 수정 없음. 별도 lane의 작업 파일 보존.
- 8th Wall: `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd`; ARCore 공식 SDK clone: `3abfeb18669c2cbb2d07057f135d117ee9d91826`.
- 검증: 현재 `_consumeResult` method와 실제 `VIOWrapper`를 실행하는 격리 JS probe; renderer/IMU/clock은 mock. 실제 camera/browser/WASM·physical sensor 성능 실험이 아님.
- 증거: `build/slam-comparison-20261014/web/probe.{mjs,json}`, `probe.stdout.json`. 8개 web 파일 SHA256, Node v22.21.1, source HEAD 포함.
- 검증 규칙: [mobile-slam-validation](../../.agents/skills/mobile-slam-validation/SKILL.md). 숫자 정적 설정과 실제 처리율, 코드 경로 존재와 측정 병목을 구분.

## 우선 발견

| 우선순위 | 발견 | 분류·증거 | 사용자 영향 |
| --- | --- | --- | --- |
| P1 | 새 result 중단 또는 Worker 실패 시 기존 `Tracking`/`pose age`/frustum이 만료되지 않음 | 격리 실행 재현; `app.js:1333–1337`, `vio-wrapper.js:296–302` | 화면이 멈췄는데 정상 추적처럼 남을 수 있음 |
| P1 | wall-clock age가 큰 새 result도 `poseFresh`가 true이면 표시 | 격리 실행에서 age=5000ms 출력 허용; `app.js:1346–1372` | engine-relative freshness와 현재 화면 freshness 혼동 |
| P2 | follow view에 fresh result당 `lerp(...,0.08)` 적용 | 현재 source 및 수식; `renderer.js:244` | 20Hz에서 step 95% 도달 1.8초, constant-velocity 등가 lag 0.575초; raw pose가 빨라도 관찰 시점이 늦게 따라감 |
| P2 | processing admission 전에 매 video callback에서 full capture/readback/gray/slice 수행 | `app.js:1098–1138` → `camera.js:576–619` | VIO busy·rate-limit로 버릴 영상도 main thread 비용과 allocation 발생; 크기는 확인, 실제 시간은 미측정 |
| P2 | 화면 video와 3D viewer가 timestamp·projection으로 연결되지 않음 | `index.html:58–85`, `app.js:630–635`, `renderer.js:40,236–246` | ARCore/8th Wall AR overlay와 체감 비교가 다른 제품 동작을 비교하는 셈 |
| P2 | UI FPS는 제출 성공 수, gray preview는 제출 후보 5개당 한 번 | `app.js:1213–1235,1271–1277,1310–1322` | FPS가 tracking output/render FPS로 오해될 수 있고 gray preview 자체가 느림 |
| P3 | result당 전체 map 복사·structured clone·전체 GPU buffer update 후보 | `vio-worker.js:193–202,438–454`, `renderer.js:208,268` | 2,000 point 상한으로 bounded; 프로파일 전 주병목으로 단정 불가 |

- 위 P1/P2는 **출력 presentation과 admission 동작**의 확인 결과. 엔진 Lost/기하 오차의 직접 원인으로 증명된 것은 아님.
- 기존 구현을 없는 것으로 보고하지 않음: frame dedup, 최신 pending 1개, callback-owned pixels, epoch/sequence check, Worker frame timeout, IMU 독립 전달, Lost clear, rotation/crop 후 calibration 변환, buffer 재사용 모두 현재 존재.

## 같은 화면을 비교하고 있는가

| 항목 | Mobile-SLAM 현재 App | 8th Wall 공개 adapter | ARCore HelloAR |
| --- | --- | --- | --- |
| 제품 화면 | 위쪽 live HTML video, 아래쪽 독립 3D trajectory viewer | camera texture + engine pose/intrinsics로 가상 scene 합성 | session frame의 camera background + camera view/projection으로 가상 scene |
| 카메라 view | 고정 60° PerspectiveCamera; pose 위치 뒤 1.5m/위 .5m follow | intrinsics 16개 element 적용, quaternion/position 직접 set | `camera.getProjectionMatrix/getViewMatrix` |
| pose view 보간 | 위치에 alpha=.08; view lookAt은 현재 camera pose | 조사한 `updateCamera`에 temporal lerp 없음 | 조사한 frame rendering에 별도 follow-view lerp 없음 |
| 표시 영상 시각 | HTML video가 독립 재생; VIO 결과는 이전 snapshot | staged CPU/GPU 처리 및 texture 관리; 무지연이라고 단정 불가 | `session.update`의 frame + display geometry 기반 |
| 추적 중단 | 명시 Lost 결과는 renderer.clear; 무응답/null은 만료 누락 | tracking state와 framework pipeline별 정책 | PAUSED이면 background 이후 3D draw return |
| 맵 표시 | trajectory 10,000점, map 2,000점 제한 | 출력 worldPoints와 게임 scene 별도 | pointCloud timestamp 증가 때 vertex buffer 갱신 |

- Mobile-SLAM의 `worldRoot.rotation.x=-π/2`는 engine Z-up→Three Y-up 시각 변환(`renderer.js:59–61`); row-major pose를 `Matrix4.set`에 전달하는 변환(`:213–224`) 존재. 이것을 누락된 좌표계 보정으로 지적하면 안 됨.
- viewer frustum은 고정 비율 wireframe, 실제 K 기반 frustum이 아님(`renderer.js:159–183`). 이것만으로 estimator K 오류를 판정할 수 없음.
- HTML video CSS `object-fit:cover`는 화면 crop이며 engine input crop과 별개. 현재 두 panel이 분리되어 있으므로 직접적인 AR registration defect는 아니지만, 같은 pixels의 overlay 정합 검증은 불가능.
- 8th Wall source: [threejs-renderer.ts:163–188](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/threejs-renderer.ts#L163-L188).
- ARCore source: [HelloArRenderer.kt:254–365](https://github.com/google-ar/arcore-android-sdk/blob/3abfeb18669c2cbb2d07057f135d117ee9d91826/samples/hello_ar_kotlin/app/src/main/java/com/google/ar/core/examples/kotlin/helloar/HelloArRenderer.kt#L254-L365). `session.update` BLOCKING 기본 구성은 camera frame rate에 rendering을 맞춘다는 샘플 주석; 내부 estimator 전체 동작·센서 rate는 이 코드로 역산 불가.

## 시작·로딩·카메라 확보

- 현재 App은 page init에서 Worker 생성 → `/vio_engine.js` dynamic import → ES module factory → `new wasm.VIOEngine()` 완료 후 Start 허용. `app.js:552–568`, `vio-wrapper.js:122–143`, `vio-worker.js:500–519`.
- 현재 loader 파일: **6,025,183 bytes**, SHA256 `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`; manifest는 `single_file_embedded_wasm=true`. 따라서 별도 외부 wasm 하나의 네트워크 load 시간만으로 startup을 평가할 수 없음.
- manifest build head는 `98b396c29ce9f6994b738190094e6397a2c6ace8`이고 dirty/source manifest hash도 보유. 현재 checkout HEAD와 다르다는 사실만으로 source/artifact mismatch 확정 불가; build source fingerprint parity 확인 필요. 이 lane은 JS artifact byte inventory만 수집.
- main module의 Three 0.170.0은 CDN importmap(`index.html:282–286`). offline/CDN 장애와 engine compile 지연은 별도. 현재 App은 async WebGPU factory를 호출하지 않고 `new Renderer(canvas)` 사용(`app.js:575`); renderer에 WebGPU 코드가 있다고 현재 GPU backend를 WebGPU라고 보고하지 않음.
- Start 순서: motion permission 요청을 user gesture 안에서 먼저 시작 → optional same-origin camera profile fetch → portrait lock 시도 → camera initialize(640,480) → VIO configure/runtime parameters → IMU 60Hz request 및 1.5초 still 후보 수집 → capture/flush 시작 → 200ms 대기 → main loop. `app.js:597–613,734–745,782–789`.
- 1.5초 수집은 검증된 센서 bias calibration 완료가 아님. `imu.js:237–300`은 candidate만 저장하고 `verified:false,applied:false`; engine이 bias estimation 담당.
- camera request는 30fps ideal/20fps min(`camera.js:139–143`), 실제 cadence는 browser/기기 결과. warm-up 5회(`app.js:143,1243–1248`)와 init 50ms/추적 33ms gate(`:157,180,1164`)는 startup/throughput에 각각 별도 반영.
- 8th Wall은 loader/chunk·capability 기반 경로 및 module pipeline을 사용하며 관련 source는 [기존 구조 보고서](2026-10-14-8thwall-architecture.md). 코드 구조만으로 startup이 더 빠르다고 주장하지 않음.

## frame·copy·queue 경로

```text
Mobile-SLAM default
video callback → drawImage → getImageData RGBA → JS gray → gray.slice owned snapshot
  → latest pending slot (replacement/coalescing)
  → pacing + worker admission → optional downsample
  → transfer ArrayBuffer → WASM HEAP copy → IMU bracket wait / engine work
  → pose/map HEAP slice → structured clone reply
  → RAF consume latest distinct result → trajectory/map + smoothed follow view → render
```

- `requestVideoFrameCallback`의 `presentedFrames` 단조 증가 확인. metadata `captureTime → presentationTime → callback arrival` 순으로 시각 선택, `physicalExposureVerified:false` 명시. frame packet에 source/process geometry, clientEpoch, configuration 객체, capture generation 묶음. `app.js:1051–1066,1098–1138`.
- callback 안에서 즉시 readback 및 owned copy. 나중에 RAF에서 video를 재촬영하지 않도록 하여 timestamp/image association을 보존. 최신 pending 1개 replacement는 `:1078–1083`; Worker 완료 callback에서 즉시 제출 재시도는 `:1091`.
- VIO busy에도 매 callback capture가 실행되는 것은 사실. 이때 computation backlog는 쌓이지 않지만 full readback과 gray conversion의 main thread 비용은 이미 소비. 단순히 캡처를 뒤로 옮기면 pixels/timestamp 대응이 깨지므로 최적화 시 contract 보존 필요.
- 640×480 예시의 payload: RGBA 1,228,800 B/capture, gray 307,200 B, owned snapshot 추가 307,200 B. 30 camera callbacks/s의 RGBA readback 논리량 36,864,000 B/s와 snapshot copy 9,216,000 B/s. 실제 bus traffic/GC time이나 특정 phone 측정값이 아님.
- VIO wrapper는 owned whole ArrayBuffer면 transfer, otherwise slice; transferred frame가 모든 단계 zero-copy라는 뜻은 아님(`vio-wrapper.js:203–229`). Worker는 HEAPU8.set copy(`vio-worker.js:184–190`).
- Worker pose/map 반환은 WASM heap `.slice()` copy 후 `self.postMessage` transfer list 없이 structured clone(`vio-worker.js:193–202,451–454`). map 최대 2,000점일 때 Float64 payload 48,000 B; 숫자 크기만으로 주요 병목으로 단정하지 않음.
- 8th Wall은 repeat frame 판정 후 video→compute texture와 draw texture에 각각 `texImage2D` 업로드. JS full RGBA/gray conversion 대신 GPU path 사용; 두 upload 및 CPU readback downstream이 있어 완전 zero-copy 아님. [session-manager-getusermedia.ts:482–556](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/session-manager-getusermedia.ts#L482-L556).
- 8th Wall 프레임 순서는 `CPU(previous stage)→update→read current texture→render→GPU stage(current)`. 같은 RAF의 모든 단계가 같은 input을 처리한다고 설명하면 안 됨. [camera-loop.ts:479–549](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/camera-loop.ts#L479-L549).
- Mobile `?grayscale=webgl`은 `video→2D canvas→GL texture→RGBA readPixels→JS R extract/flip`; GPU 이름만으로 CPU 경로보다 빠르지 않음. 매 호출 vertex Float32Array/bufferData, 첫 5회 CPU 대조 capture도 수행(`camera.js:400–492,532–548`). 기본 경로가 아님.
- `?capture=worker`는 명시 opt-in, `grayscale=webgl`과 함께이면 비활성(`app.js:428–430`). 별도 capture worker는 VideoFrame in-flight 1개+pending 1개, exact identity/geometry, finally frame.close, timeout/termination 관리(`capture-worker.js:39–124,192–239`). 현 소스의 기본 동작은 sync 유지; worker 지원/호출 성공과 최종 latency 개선은 별개.

## 시각·IMU·성능 계측 의미

| 필드/화면 | 실제 의미 | 포함되지 않는 것 |
| --- | --- | --- |
| 화면 FPS | 최근 1초 sendFrame accepted 수 | camera callback 수, completed/fresh pose 수, draw FPS |
| `featureCount` | engine 보고 tracker 수 | 현재 metric visual factor 수/기하 지지 품질 |
| `engineProcessingMs` | `engine.processFrame` 호출 구간 | HEAP write, bracket wait, reply clone, UI/render |
| `workerQueueMs` | timeOrigin+now 기반 post→receive 차 | camera readback, pending slot time |
| `imuWaitMs` | frame 기준 미래 IMU bracket 기다린 시간 | engine process 시간 |
| `roundtripMs` | main post→reply 처리 | source→post latency, 결과→화면 시각 |
| `frameCopyMs` | wrapper copy만; transfer면 0 | camera getImageData/gray.slice, WASM copy |
| `poseAgeSeconds` | inputTimestamp−poseTimestamp | wall-clock presentation age |
| status `pose age` | 새 result를 소비하는 순간 now−poseTimestamp | 이후 같은 result를 재표시하는 시간; 현재 만료 안 됨 |

- source timestamp는 performance clock 상대 seconds. DeviceMotion epoch timestamps는 `timeOrigin`을 빼며 미래/잘못된 시각 reject(`imu.js:45–50,553–561`). 화면 숫자 시계가 맞는 것과 physical exposure/IMU hardware sync를 검증한 것은 다름.
- IMU flush는 10ms 독립 timer(`app.js:136,939–945`)와 frame 직전; workerBusy로 IMU 전송 자체를 막지 않음. 단, browser main thread가 capture/DOM에 묶이면 timer와 sensor callbacks의 실제 dispatch가 늦어질 수 있음.
- Worker bracket budget은 40–80ms adaptive bound, 최근 source cadence p95×2 이용(`vio-worker.js:14–16,55–63`). 이것은 항상 40ms를 추가한다는 뜻이 아님; 이미 미래 sample이 있으면 즉시 진행. root replay에서 `imuWaitMs` 실제 분포와 함께 평가해야 함.
- 확인된 future/invalid/stale epoch guard를 유지. 빠르게 보이도록 입력 clock을 임의로 당기거나 ingress bias를 적용하는 제안은 현재 증거에서 정당화되지 않음.

## 격리 실행으로 확인한 표시 오류와 smoothing

```bash
node build/slam-comparison-20261014/web/probe.mjs
```

| probe | 실제 관찰 | 판정 |
| --- | --- | --- |
| fresh result, now=1000ms, pose=1s | Tracking, age 0ms, visible=true | 정상 기준 |
| 동일 result를 now=6000ms에 재소비 | Tracking, age **0ms 그대로**, visible=true | 만료·age 갱신 누락 재현 |
| 실제 wrapper `_workerFailure` 후 null 소비 | wrapper configured=false/error 기록; 화면 그대로 | 오류 전파 경계 누락 재현 |
| 새 sequence의 pose=1s를 now=6000ms에 소비 | age 5000ms 표시, visible=true | wall age 상한 없음 재현 |
| 명시 Lost result | visible=false, clear 1회 | 기존 Lost 보호는 정상 |

- 첫 harness 실행은 mock에 constructor의 `_renderEpoch=null`을 누락해 초기 clear 횟수 assertion 실패. 이를 실제 constructor 상태와 맞춰 수정 후 위 결과 통과. 실패 원인 기록은 `probe-initial-harness-failure.txt`에 보존; 제품 결함으로 집계하지 않음.
- 실제 Worker failure의 callback은 `_latestResult=null`로 초기화하고 terminate하지만 App은 `null`에서 즉시 return. 정상 result key의 dedup도 state/age 갱신 전에 return. 따라서 새 valid output 없이 무한히 동일 pose를 그릴 수 있음. 새로운 Lost 결과가 도착하면 clear하므로 모든 Lost가 오래 표시된다고 일반화하지 않음.
- `fresh` 조건은 engine fresh/status/finite pose를 검증하나 `now−poseTimestamp` 제한 없음. frame timeout 기본 30초는 Worker queue/call hang bound이며 화면 freshness SLA가 아님(`vio-wrapper.js:46,61–62,220–225`).
- follow 수식: `x[n]=0.92*x[n−1]+0.08*target[n]`; fixed target 오차 `0.92^n`, 95% 수렴은 `ceil(log(.05)/log(.92))=36` fresh outputs. 10/20/30Hz에서 3.6/1.8/1.2초.
- 일정 속도 목표에 대한 정착 lag: `(1−α)/(α*f)=11.5/f`초. 20Hz=.575초. 이는 source alpha로 계산한 관찰 카메라 모델이며 실제 엔진 source-to-pose latency 측정이 아님.
- frustum pose matrix 자체는 현재 raw pose를 바로 적용. 지연되는 것은 외부 관찰 카메라 위치; SLAM trajectory를 smoothing했다고 표현하지 않음. 카메라 lookAt은 현재 pose를 보므로 turning/translation 동안 framing이 뒤따르는 체감 가능.

## trajectory·map·DOM·lifecycle

- trajectory 10,000점과 map 2,000점 preallocation, scratch Matrix/Vector 재사용, fixed boundingSphere, drawRange 이미 구현(`renderer.js:14–15,74–104`). 무제한 JS array growth 또는 매 frame geometry 재생성이라는 지적은 틀림.
- trajectory는 ring이 아니므로 최대치 이후 append 중단; 20 fresh updates/s이면 500초에 해당. frustum은 계속 이동. 성능 문제와 장시간 관측 기록 누락을 구분.
- `needsUpdate`만 지정하고 updateRanges를 지정하지 않음(`renderer.js:208,268`): Three 일반 buffer upload 경로에서는 전체 attribute 갱신 후보. 크기는 trajectory120,000B+map24,000B로 bounded; 실제 driver upload capture 없이 비용 확정 불가.
- `Array.from(pose)` validation이 App 및 Renderer 양쪽에 존재(`app.js:1352`, `renderer.js:196`); “zero per-frame allocation” 주석은 문자 그대로 정확하지 않음. 16 elements라 다른 복사보다 작은 후보.
- status text 및 console 출력은 새 result마다(`app.js:1366,1513–1517`), IMU DOM은 10Hz, FPS DOM은 1Hz. layout 변화가 ResizeObserver를 부르면 resize/render가 추가될 수 있음. 크기 변화 guard는 renderer에 존재. 측정 전 DOM을 주병목으로 지목하지 않음.
- WASM log는 console+sendBeacon `/log`로 전달(`app.js:420–426`); Worker는 solver 일부 noise 정규식 필터(`vio-worker.js:503–506`). remaining log 빈도·network pressure 계측 필요.
- blur/visibility: snapshot 취소, IMU flush 중단, resume에서 stale IMU discard+reset; orientation은 serialized reconfigure와 snapshot generation guard. 이들 lifecycle 처리 자체의 부재를 가정하지 않음(`app.js:1394–1443`).
- `stop()`은 capture/IMU/engine Worker 정리·observer disconnect·renderer.clear. Renderer에는 GPU resource dispose가 없음. 현재 페이지가 renderer를 한 번 만드는 사용에서는 즉시 반복 누수로 확정할 수 없지만 mount/unmount 재사용에는 별도 dispose 경로 필요. startup catch도 init 후 일부 lifecycle listener 정리를 stop 전체와 동일하게 수행하지는 않음(`app.js:791–802,1520–1544`).

## 8th Wall 모바일 웹을 우선 기준으로 볼 때

- 두 제품 모두 브라우저 입력과 JS/WASM 경계를 갖는다. **Mobile-SLAM에도 engine Worker가 이미 있으므로 “8th Wall은 Worker라 빠르고 우리는 main-thread engine이라 느리다”는 비교는 성립하지 않음.** 차이는 어떤 작업을 어느 thread/GPU stage에서, 어떤 input/output identity로 실행하는가에 있음.
- 8th Wall은 처리한 `active` pipeline의 `realityTextureId`를 response와 함께 JS로 노출(`tracking-controller.cc:710–741`); JS는 rotation/position/intrinsics와 `realityTexture`를 같은 result로 반환(`tracking-controller.ts:776–786`). `GlTextureRenderer.setTextureProvider`가 `processCpuResult.reality.realityTexture`를 사용(`:1019–1021`). 즉 표시 배경 선택과 처리 결과에 명시적인 연결이 있음. 이것은 배경/pose association 설계 근거이지 physical exposure synchronization 또는 motion-to-photon 0ms 보증이 아님.
- Mobile-SLAM은 live video independent playback + 과거 fresh pose + 별도 follow viewer. 따라서 8th Wall처럼 잘 붙어 보이는 화면을 목표로 한다면 **더 강한 smoothing보다 먼저 결과 pose/projection과 대응 camera texture의 계약**을 정해야 함. 현재 raw frustum matrix는 직접 적용되지만 observer camera alpha=.08이 별도 체감 지연을 더함.
- 8th Wall Three adapter의 direct pose set과 Mobile-SLAM observer lerp는 명확히 다름. 그러나 adapter에 lerp가 없다는 사실만으로 비공개 binary 내부 smoothing/prediction이 없다고 주장하지 않음.
- 8th Wall GPU frontend는 filtered input → pyramid blur/decimate → CPU에 필요한 packed 결과 readback. `GlRealityFrame::draw(DEFER_READ)`는 이전 readback을 먼저 소비하고 현재 texture를 draw(`gl-reality-frame.cc:653–660`). 이를 Mobile-SLAM의 full RGBA→JS gray→CPU/WASM frontend와 비교해야 함. [GPU/readback source](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/features/gl-reality-frame.cc#L484-L567).
- WebGL2 PBO 경로는 `PIXEL_PACK_BUFFER + getBufferSubData(...HEAPU8.subarray...)`; fallback은 `glReadPixels` 직접. fence busy wait가 lag를 만들었다는 source 주석과 비활성화된 loop가 존재(`:494–515`). CPU readback을 없앤 설계가 아니라 **GPU 작업과 다음 frame CPU 소비를 분리해 readback 위치·양·stall을 관리하는 설계**로 해석.
- iOS는 해당 pinned source에서 `setDisablePixelBuffer(os==='iOS')`로 pixel-buffer 경로 비활성화(`tracking-controller.ts:1009–1011`). 그러므로 “모든 8th Wall mobile browser에서 PBO라 빠르다”는 주장은 틀림. [runtime selection](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts#L1009-L1021).
- GPU pyramid 이식은 preprocessing/descriptor/ROI 및 CPU frontend 계약 변경을 동반한다. 단순 `grayscale=webgl` 활성화나 capture Worker 추가만으로 동등한 파이프라인이 되지 않으며, semantic parity·source-to-pose p95·coverage·Lost를 함께 비교해야 함.
- 비교 우선 측정값: (1) source pixels→owned snapshot의 main time, (2) coalesced frame에 소비한 capture 비용, (3) readback stall/GPU stage time, (4) source→fresh result, (5) result→display, (6) 화면 background/pose 시각차. 같은 recorded input·같은 pacing/browser에서 비교한 후 실제 camera/IMU 기기 검증으로 확장.

## 다음 확인 순서와 수락 근거

1. **화면 상태 contract**: 새 output 중단/Worker error 시 freshness 만료·오류 표시; explicit Lost/epoch clear/dedup을 보존. 독립 clock 테스트와 실제 browser timeout에서 재확인.
2. **체감 지연 분리**: viewer follow on/off를 동일 recorded output에 적용; raw pose 변화시각·viewer camera 변화시각·video시각 각각 기록. 위치 smoothing 제거만으로 tracking 성공률 향상 주장 금지.
3. **capture 비용**: camera callback/readback/gray/copy/pending residence p50/p95/p99 및 실제 main-thread long task를 수집. owned snapshot association과 bounded queue를 유지한 최적화만 비교.
4. **엔진 시간**: workerQueue/imuWait/engine/response/display를 합산하고 frame IDs와 연결. source callback 수, accepted/completed/fresh/Lost/reset 각각 보고.
5. **같은 제품 기준**: AR overlay 비교라면 intrinsic projection·camera frame/pose 대응을 구현한 별도 fixture가 필요; 현재 trajectory viewer를 HelloAR overlay와 그대로 비교하지 않음.
6. **추적 품질**: measured K/distortion/R_ic/t_ic 및 hardware timestamp sync 검증, 동일 motion/scene/GT/coverage에서 비교. 현 heuristic calibration을 ARCore 기기 calibration과 동등하다고 가정하지 않음.

- 이번 범위에서 제품 수정·새 dependency 설치·브라우저/기기 독점 사용·dataset 변경 없음.
- 완료: source 경로 추적, 고정 공식 sample 비교, deterministic UI fault probe, smoothing 수식, artifact/source fingerprints.
- 미검증: 실제 render FPS/GC/GPU 업로드 시간, physical exposure sync, phone end-to-end lag, SLAM trajectory 정확도, 원인별 기여율. 상위 replay/실기 lane의 별도 근거와 합쳐 판단.
