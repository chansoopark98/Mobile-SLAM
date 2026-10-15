# 8th Wall 공식 binary의 제어 실험 — 2026-10-14

- 범위: 공식 `xr.js` + `xr-slam.js` 원본 bytes를 Chromium에서 실행; public API/camera pipeline callback 관측.
- 판정: 실행·입력 교란 반응 검증. SLAM 정확도·실제 휴대폰·metric scale 수락 판정 없음.
- 연구 harness: `scripts/dev/references/eighthwall/experiment-matrix.mjs`, `experiment-runtime.js`, `summarize-experiments.py`.
- 보존: 기존 `runtime-probe.mjs`, `runtime.js`, `sensor_adapter.mjs`, 공식 엔진 bytes, 기존 증거 수정 없음.
- 증거: `build/8thwall-deep-20261014/runtime/<label>/result.json`, `source/`, `screenshot.png`; JSON에 명령·SDK/input/source hash·전체 callback·sensor dispatch·network·cleanup 기록.

## 입력·격리·엔진 경계

- 공식 출처: [8thwall/engine](https://github.com/8thwall/engine), 고정 commit `e1421ae742a9c5d654871367c64c1de3ad044caf`의 `xr-standalone.zip`.
- 획득 이력: `scripts/dev/references/eighthwall/bootstrap.py`, `build/research-20261012/8thwall-binary/acquisition.json`.
- archive: 13,998,848 bytes, SHA-256 `a2fc73ba450c1a96ee2a20db442f7fb328a3c8b76565e7e9d15b68cd75c86a93`.
- engine: `assets/references/8thwall/analysis-runtime/standalone/`; SDK source 수정·WASM export wrapping 없음.
- 입력: `build/mobile-web-20261009/synthetic/clean-seed101-v2/input.json` 및 hash 검증한 320×240 gray PGM 240장, 20Hz, source 시간 0–11.95초.
- 영상: gray→RGB canvas→`VideoFrame`→`MediaStreamTrackGenerator`→가상 `getUserMedia`; bootstrap에서 첫 frame 반복 후 `onStart` 기준 본 입력 publication 시작.
- 센서: 원시 IMU→body/device 축 변환→gyro 적분 quaternion→W3C orientation. 첫 가속도 gravity 정렬·임의 yaw, bias/OS fusion 미복원; `isTrusted=false`.
- GT trajectory/pose/orientation은 읽거나 엔진에 제공하지 않음. fixture의 카메라 K·extrinsic·노출과 SDK의 device calibration 일치도 미검증.
- context: Pixel 7 Android13 UA, mobile/touch emulation, headless desktop Chrome, SwiftShader 허용. 사용자 브라우저·물리 카메라·현재 서비스 미사용.
- 각 arm은 별도 browser/context/ephemeral localhost server. 외부 HTTP 차단; 내부 SDK chunk 요청·Resource Timing 수집.
- `initialize()`·`loadChunk('slam')`→`configure()`→`updateCameraProjectionMatrix()`→renderer/controller/observer modules→`run()`→`stop()` 순서.
- render camera origin `(0,2,0)` 설정. 따라서 `position!=0` 자체는 이동의 증거가 아님.

## 실험 구성

- moving responsive/absolute: 동일 gray·raw IMU, scale만 변경.
- disabled-world: `disableWorldTracking=true`, 나머지 responsive moving 동일.
- blank: 모든 gray pixel=0; moving 센서는 유지.
- static: 모든 영상은 첫 gray frame; moving 센서는 유지. 정지 센서/정지 카메라의 일치된 static rig 실험이 아니라 시각·관성 불일치 교란.
- no-sensors: harness의 motion/orientation dispatch 모두 제거.
- no-motion / no-orientation: 각 public event 종류만 제거; 나머지 종류 유지.
- freeze-recover: frame index 80–159는 frame80 pixels 반복; 전·후는 원래 영상, 센서는 계속 진행.
- recenter: frame120 publication 전 public `XrController.recenter()` 호출.
- pause-resume: frame120 publication 전 `XR8.pause()`, 1초 후 `XR8.resume()`; publication schedule·sensor clock은 계속 진행하되 pause 중 writer 제출 생략. SDK가 기존 track을 stop하므로 resume의 `getUserMedia`에서 새 stream 생성.

## 시간·개수 해석

- `sourceFrames`: harness publication 시도 수; pause 중 실제 writer 제출을 건너뛴 경우 포함. writer accepted 수·source pixel 종류 수와 별도.
- `callbacks`: public `onUpdate` 결과 수; `repeatFrame=false` 및 unique `videoTime` 별도. callback 수를 SLAM solve 수로 환산하지 않음.
- `videoTimeMatchedPublications`: frameStart `videoTime`이 generated `VideoFrame.timestamp`와 2µs 이내인 source publication 수. pose와 그 영상이 동일하다는 증명 아님.
- 실제 SDK pipeline은 CPU(previous staging)→update→camera upload→render→GPU(current frame). callback의 최신 publication·frameStart metadata와 pose가 동일 영상이라는 가정 금지.
- `pipelineFrameStartToObserverMs = callback performance.now() - 1000×frameTime`: public frame 시작부터 observer까지, scheduling/앞선 stages 포함; 순수 SLAM kernel 시간 아님.
- `declaredVideoTimestampToObserverMs = callback performance.now() - 1000×videoTime`, fresh callback 한정. 합성 timestamp 기준; 물리 노출→pose latency 아님.
- `latestPublicationToObserverMs`: callback 당시 가장 최근 source publication과 시간 차이. 영상-포즈 정확한 association 아님; 마지막 frame 뒤 500ms tail 포함.
- `writerAcceptanceMs`: publish→writer accepted; `sensorDispatchLagMs`: 실제 dispatch wall time−합성 event timestamp.
- 각 통계: n/min/p50/p95/p99/max. 단일 bounded run이며 concurrent 연구 작업·software GPU 영향을 통제한 성능 벤치마크 아님.
- 보존된 최초 `moving-responsive-240/result.json`의 `pipelineFrameStartToObserverMs`는 frameTime 초↔ms 혼용 오류. 이 지표 및 해당 run의 비교 표 사용 금지. `*-v2`부터 단위 수정; 최초 실패 증거 삭제·덮어쓰기 없음.

## 재현

```bash
node --check scripts/dev/references/eighthwall/experiment-matrix.mjs
node --check scripts/dev/references/eighthwall/experiment-runtime.js
node --test scripts/dev/references/eighthwall/sensor_adapter.test.mjs
python3 -m py_compile scripts/dev/references/eighthwall/summarize-experiments.py

node scripts/dev/references/eighthwall/experiment-matrix.mjs \
  --label new-moving-responsive-240-v2 --arm moving --scale responsive --frames 240
# --arm: moving|disabled-world|blank|static|no-sensors|no-motion|no-orientation|freeze-recover|recenter|pause-resume|face|sky
# --scale absolute로 scale 비교. label 존재 시 실패하여 기존 evidence 보존.
python3 scripts/dev/references/eighthwall/summarize-experiments.py \
  --output build/8thwall-deep-20261014/runtime/new-matrix-summary.json
```

- syntax 검증은 lint/typecheck로 부르지 않음. 개발 dependency 추가 없음.
- sensor adapter: 기존 113 수학 검사 PASS. 실제 browser sensor fusion 정확도 검증 아님.
- production C++·WASM 구현 수정 없음; 이 연구에 native regression/build/실기기 테스트는 실행하지 않음.

## 실행 결과

- 집계: `build/8thwall-deep-20261014/runtime/matrix-summary-final.json`; 13개 구성, 정상 관측 14회(수정 후 moving 120-frame 회귀 포함), harness 실패 2회 보존. 별도 최초 단위오류 run 1회 보존.
- 환경: Chrome `144.0.7559.96`, ANGLE Vulkan `SwiftShader Device (Subzero)`, WebGL2·OffscreenCanvas 사용 로그 확인. `XR8.version()`은 `0.0.0.0`을 반환하므로 버전 provenance는 commit/asset hash로 고정.
- xr.js: 1,036,695 bytes, SHA-256 `b3973936862f4cb77ae32d24f253d8a0e8b1de4fa2a0cf688833742889ee829d`.
- xr-slam.js: 5,538,007 bytes, SHA-256 `398f675c920892afd4521d5196df2b6b8a1b8a90b6a5c749cf5ad1a6e0a85270`.
- 아래 상태·points는 fresh callback 기준. `N=NORMAL/UNSPECIFIED`, `L=LIMITED/UNSPECIFIED`, `I=LIMITED/INITIALIZING`.

| 240-frame arm | writer 제출 | callbacks | fresh / unique videoTime | 최대 points | fresh 상태 |
|---|---:|---:|---:|---:|---|
| moving-responsive | 240 | 693 | 238 / 238 | 399 | N 238 |
| moving-absolute | 240 | 670 | 236 / 236 | 0 | I 236 |
| disabled-world-responsive | 240 | 700 | 239 / 239 | 0 | N 47, L 192 |
| blank-responsive | 240 | 703 | 239 / 239 | 0 | N 47, L 192 |
| static-responsive | 240 | 659 | 239 / 239 | 399 | N 239 |
| no-sensors-responsive | 240 | 606 | 231 / 231 | 300 | N 231 |
| no-motion-responsive | 240 | 705 | 238 / 238 | 400 | N 238 |
| no-orientation-responsive | 240 | 706 | 238 / 238 | 398 | N 238 |
| freeze-recover-responsive | 240 | 680 | 239 / 239 | 396 | N 239 |
| recenter-responsive | 240 | 631 | 234 / 234 | 400 | N 234 |
| pause-resume-responsive (fixed) | 219 | 652 | 218 / 218 | 399 | N 218 |

- responsive: mapping/world points와 finite transform 발행 확인. 동일 pose의 반복 callback이 많으므로 693 callbacks를 693 처리 frame으로 해석하면 안 됨.
- absolute: 11.95초 source 구간 끝까지 전 fresh callback `LIMITED/INITIALIZING`, points=0. finite/moving transform은 metric initialization 성공 증거가 아님. 더 긴 입력·실기기 absolute 수락 미검증.
- blank·disabled-world: 초기 47 fresh는 NORMAL이지만 points=0·position range `(0,0,0)`, 이후 192 fresh LIMITED. NORMAL만으로 시각 추적 성공 판정 불가.
- static + moving sensors: 영상 hash 1종인데 NORMAL·points399, position range `(0.8421,0.9119,1.3671)`. 입력 불일치에 대한 출력이며 실제 정지 drift/측정 단위 검증 아님.
- no-sensors: harness IMU dispatch 0, NORMAL·points300; visual 경로 실행 가능 관측. 엔진 내부 IMU 미사용 전체 증명이나 센서 필요성 부정 아님.
- no-motion/no-orientation: 한 event 종류를 제거해도 bounded responsive callback 발행 지속. 공개 API 출력만으로 내부 fusion weight·bias 모델 식별 불가.
- freeze-recover: 전/동결/복구 fresh `79/80/80`, 각각 최대 points `358/396/367`, 모두 NORMAL. 시각 동결 중에도 mapping 출력 유지; fresh timestamp와 새로운 image content는 다른 개념.
- recenter: 호출 후 첫 fresh(+36.3ms)는 이전 pose·points204, 다음 fresh(+88.3ms)는 origin `(0,2,0)`·points0. 비동기 재설정 관측; 첫 callback이 즉시 재설정된다는 계약으로 사용 금지.
- pause-resume fixed: 1.001초 pause, 240 publication 시도 중 219 writer 제출·21 skip, 두 번째 `getUserMedia`에서 이전 track=`ended`, 새 track 획득. callbacks 재개·218 fresh NORMAL·owned track2 종료 확인.

## 공개 callback 시간 통계

- 아래는 합성 `videoTime`→fresh observer ms의 p50/p95/p99; kernel latency·실기기 capture latency·pose와 정확히 대응된 영상 latency 아님.

| arm | n | p50 ms | p95 ms | p99 ms |
|---|---:|---:|---:|---:|
| moving-responsive-240-v2 | 238 | 41.90 | 51.20 | 63.70 |
| moving-absolute-240-v2 | 236 | 41.60 | 76.10 | 136.30 |
| disabled-world-responsive-240-v2 | 239 | 42.00 | 51.90 | 72.30 |
| blank-responsive-240-v2 | 239 | 35.00 | 50.90 | 101.60 |
| static-responsive-240-v2 | 239 | 45.40 | 58.60 | 109.90 |
| no-sensors-responsive-240-v2 | 231 | 43.90 | 94.30 | 116.30 |
| no-motion-responsive-240-v2 | 238 | 44.60 | 48.90 | 59.10 |
| no-orientation-responsive-240-v2 | 238 | 43.40 | 51.60 | 72.50 |
| freeze-recover-responsive-240-v2 | 239 | 44.80 | 70.80 | 99.90 |
| recenter-responsive-240-v2 | 234 | 43.20 | 119.40 | 167.20 |
| pause-resume-responsive-240-fixed-v2 | 218 | 42.30 | 51.00 | 74.10 |

- 각 result JSON에 frame-start→observer, latest-publication→observer, writer acceptance, IMU dispatch lag의 전체 percentile 보존.
- pause에서 frame-start→observer max1083.1ms: pipeline pause 포함. 이를 SLAM kernel 1초 소요로 보고하면 안 됨.
- 본 실험은 반복성·CPU affinity·열·독점 GPU 통제 없음. arm 간 작은 ms 차이의 우열 판정 금지.

## Face·Sky 실제 binary 실행

| 구성 | 실제 로드 | 관측 | 판정 한계 |
|---|---|---|---|
| Face, 120 frames | `xr.js`, `xr-face.js`, `face-model.tflite`, `face-mesh-model.tflite` HTTP200 | `faceloading` topology metadata, face/ear ROI renderer init, 383 feature callbacks / 119 fresh | facefound/update/lost 0회. 실제 얼굴 없는 합성 영상; 얼굴 검출·landmark 정확도 미검증 |
| Sky, 120 frames | `xr.js`, `xr-slam.js`, `semantics-worker.js`, `semantics-model.tflite` HTTP200 | `layerscontroller.layers.sky`, cameraFeedTexture, 382 feature callbacks / 119 fresh; `Created TensorFlow Lite XNNPACK delegate for CPU` 로그 | sky mask pixel 검증·분할 GT·IoU 없음. layer 반환과 모델 실행 준비는 분할 정확도 수락이 아님 |

- Face는 front camera config·`MeshGeometry.FACE`, mirroredDisplay/right-handed 설정. xr-face만 preload/loadChunk하면 XrController가 없을 수 있음: feature의 독립 chunk 경계를 실제 관측.
- FaceController camera pose는 origin0, Sky는 origin `(0,2,0)`에 고정. trackingStatus/trackingReason은 제공되지 않음; JSON의 undefined/None는 SLAM 상태가 아님.
- face ear model은 FACE mesh 옵션에서 요청되지 않음. ear landmark·hand/VPS/image target 알고리즘 실행 검증은 이 실험 범위 밖.
- feature 추가 후 moving120 회귀: 120 writer 제출, 362 callbacks/118 fresh, NORMAL118, 최대points390, SDK/page/harness error0.

## 실패 보존·resource cleanup

- `pause-resume-responsive-240-v2`: 초기 harness가 SDK pause로 종료된 stream을 재사용; `Stream closed` 및 `Cannot close a ERRORED writable stream`, timeout. cleanup 전에 관측 serialize하지 않아 해당 실패 run의 partial rows는 보존되지 못함. 이 결손을 성공·0출력으로 해석 금지; raw error/console/source는 보존.
- 수정: cleanup 전 snapshot, writer close 오류와 관계없이 `finally`에서 전체 소유 track 종료, pause 중 writer 제출 생략, 새 getUserMedia에 stream 재생성. `pause-resume-responsive-240-fixed-v2`로 별도 성공 증거 확보.
- `face-responsive-120-feature-v2`: face chunk만 로드했는데 harness가 `Object.keys(XR8.XrController)`를 호출해 TypeError. SDK의 face 실패 아님. null-safe API inspection 후 `face-responsive-120-fixed-v2` 별도 성공.
- 정상 run14회: owned stream 종료=true, browserClosed=true, serverClosed=true; pageErrors/harness errors/requestFailures/blockedExternal=0. 최초 실패2회도 browser/server 종료 확인; 개별 track 종료는 해당 result에서 미검증.
- 로컬 SDK assets만 요청; SLAM은 xr.js+xr-slam.js, Sky는 semantics worker/model, Face는 별도 face chunk/model을 실제 요청. local `Cache-Control:no-store`; release CDN cache·압축·remote latency 실험 아님.
- Chrome console GPU `ReadPixels` stall 경고는 보존. software WebGL을 물리 GPU 가속 성공으로 확대하지 않음.
- result/source snapshots가 각 실행 당시 파일을 고정. lifecycle/feature 수정 이전·이후 harness source hash가 다르며, final helper만 보고 과거 실행 소스를 추정하지 않음.
