# 8th Wall 원본 SDK 실제 실행과 가상 입력 관측

- 작업 시작 **2026-10-13**, 새 실행 **2026-10-14 00:01–00:04 KST**. 문서 파일명은 조사 시작일 유지.
- 결과: 공식 배포 `xr.js`·`xr-slam.js`를 원본 그대로 실제 Chrome에서 초기화하고 camera pipeline 호출. mobile emulation에서 공개 API의 finite pose·intrinsics·world points를 새로 관측.
- 수락 범위: **실제 SDK 초기화·가상 입력·공개 결과 경계 실행 확인**. commercial solver 완전 복원·상용/자체 동일 입력 정확도 비교·실기기 성능 수락은 미완료.
- 새 코드: [runtime-probe.mjs](../../scripts/dev/references/eighthwall/runtime-probe.mjs), [README-runtime.md](../../scripts/dev/references/eighthwall/README-runtime.md). 기존 `runtime.js`·`sensor_adapter.mjs` 재사용, 제품 source·SDK·dataset 변경0.
- 정적 분석과 별도 증거: [상용 자산 재검사](2026-10-13-commercial-engine-reinspection.md), [공개 source 차이](2026-10-13-engine-gap-analysis.md).

## 실행 환경·자산 identity

- Node **v22.21.1**, Playwright **1.63.0**, Google Chrome **144.0.7559.96**, Linux headless, WebGL2.
- 실제 GL backend: `ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device (Subzero) (0x0000C0DE)), SwiftShader driver)`.
- wrapper·입력·SDK SHA: [manifest.json](../../build/commercial-slam-20261013/8thwall-runtime/manifest.json), 각 run `result.json`·`source/`. source revision `2789f64534f5f8a5d6b4d65fc616a7bdeeb7613f`; wrapper는 새 untracked 파일. revision을 해당 SDK의 upstream source 빌드로 해석하지 않음.
- SDK acquisition: 기존 `bootstrap.py` 고정 upstream `8thwall/engine@e1421ae742a9c5d654871367c64c1de3ad044caf`의 standalone 배포. 이번 실행 시 파일을 다시 hash했으며 신규 SDK 다운로드·token 발급·login 없음.

| 실행 자산 | bytes | SHA256 |
|---|---:|---|
| `xr.js` | 1,036,695 | `b3973936862f4cb77ae32d24f253d8a0e8b1de4fa2a0cf688833742889ee829d` |
| `xr-slam.js` | 5,538,007 | `398f675c920892afd4521d5196df2b6b8a1b8a90b6a5c749cf5ad1a6e0a85270` |
| `LICENSE` | 15,994 | `ed166ff1e5002cd7557f2c595a2398b092e76481070e6bc3b253e51517f3ae61` |

- `XR8.version()` 실제 반환값 **`0.0.0.0`**. release marketing version을 추측하지 않고 배포 byte hash로 식별.
- [공식 배포 구분](https://8thwall.org/docs/open-source): MIT framework와 별도 world-SLAM binary를 구분. engine binary·라이선스 보존; 이번 probe는 공개 API 실행이며 수치 구현·SDK 수정본을 제품에 편입하지 않음.

## 직접 실행한 경로

1. 전용 `127.0.0.1:<ephemeral>`에서 원본 SDK·입력 fixture만 serve; SDK page의 외부 HTTP 요청은 차단.
2. `data-preload-chunks="slam"`, 실제 `await XR8.initialize()`, `await XR8.loadChunk('slam')`, `XR8.isInitialized() === true`를 확인. [initialize 계약](https://8thwall.org/docs/api/engine/xr8/initialize).
3. `XR8.XrController.configure({disableWorldTracking:false,enableWorldPoints:true,scale})`, 공개 `updateCameraProjectionMatrix()` 호출.
4. `GlTextureRenderer.pipelineModule()`·`XrController.pipelineModule()`·공개 observer module 설치, button click에서 `XR8.run()` 호출.
5. 가상 gray PGM→RGBA canvas→`VideoFrame`→`MediaStreamTrackGenerator`를 `getUserMedia` 입력으로 제공. SDK는 실제 HTMLVideo·GL·WASM 경로 실행. raw IMU→가상 `devicemotion`/`deviceorientation` event 제공.
6. 공개 `onUpdate({frameStartResult,processCpuResult})`의 결과 저장; `XR8.stop()`·track 종료·browser/server 종료. 모든 run `ownedResourcesClosed:true`.

- **SDK mock0·SDK byte patch0·WASM export wrapper0**. 입력 browser API의 가상화와 SDK 실행을 구분.
- 기존 `run_browser.mjs`의 `observer.js`는 instantiate/export 관측 wrapper를 사용. 이번 새 실행은 이를 제외; 내부 export 호출 수를 새로 관측했다고 주장하지 않음.
- input GT 파일은 미사용. adapter는 첫 accelerometer의 gravity 방향과 raw gyro 적분을 사용; yaw gauge 임의·bias correction 없음·OS fused orientation 미재현. event `isTrusted:false` 기록.
- 400frame 원본 구간: **0–19.95s**, 320×240 gray, source20Hz·IMU200Hz. callback cadence와 입력 publication cadence는 별도.

## 새 관측 결과

| run | source frame / IMU event | 공개 callback / nonrepeat callback | finite pose·intrinsics | tracking status/reason | world point min–max |
|---|---:|---:|---:|---|---:|
| [mobile responsive400](../../build/commercial-slam-20261013/8thwall-runtime/public-mobile-responsive-400/result.json) | 400 / 3,994 | 599 / 296 | 599 / 599 | `NORMAL/UNSPECIFIED`:599 | 0–397 |
| [mobile absolute400](../../build/commercial-slam-20261013/8thwall-runtime/public-mobile-absolute-400/result.json) | 400 / 3,994 | 610 / 309 | 610 / 610 | `LIMITED/INITIALIZING`:610 | 0–0 |
| [desktop responsive120](../../build/commercial-slam-20261013/8thwall-runtime/public-desktop-responsive-120/result.json) | 120 / 1,194 | 387 / 387 | 0 / 0 | status/reason 미제공 | 미제공 |
| [mobile responsive120](../../build/commercial-slam-20261013/8thwall-runtime/public-mobile-responsive-120/result.json) | 120 / 1,194 | 203 / 94 | 203 / 203 | `NORMAL/UNSPECIFIED`:203 | 0–344 |
| [mobile absolute120](../../build/commercial-slam-20261013/8thwall-runtime/public-mobile-absolute-120/result.json) | 120 / 1,194 | 209 / 95 | 209 / 209 | `LIMITED/INITIALIZING`:209 | 0–0 |

- 모든5건: SDK `initialize`·`loadChunk` resolved, process exit0, page/runtime error0, blocked external0, failed request0. stderr0byte.
- 보존한 warning: software GL의 `GPU stall due to ReadPixels`; desktop1회, 각 mobile4회. warning 존재를 무오류 성능 수락으로 바꾸지 않음.
- 400frame 두 run은 독립 browser에서 병렬 실행. source/callback 수는 경계 관측값이며 직렬 ABBA 속도 비교·phone FPS·latency 결과 아님.
- 400frame responsive position 좌표별 범위 `[0.461152,2.394047,0.700660]`, absolute `[0.197269,1.155530,0.346715]`. 출력 변화 존재만 확인; responsive unit를 m로 표기하지 않음.
- absolute는 pose가 변해도 **전체 callback `LIMITED/INITIALIZING`·world point0**. metric 초기화 통과·정확도 수락 근거 없음.
- desktop은 camera callback387회여도 finite pose0. [공식 device 계약](https://8thwall.org/docs/api/engine/xrconfig/device)의 `ANY` camera 허용과 mobile world-tracking 범위 차이를 실제 경계에서 확인. Android UA emulation을 물리 Android 실행으로 표기하지 않음.
- nonrepeat callback과 unique `videoTime`은 mobile에서 같았으나, source publication400보다 적음. 내부 processed frame 수·정확한 SDK drop count는 이 공개 observer만으로 확정하지 않음. desktop의 `nonrepeat=true`는 world tracking 처리 증거가 아님.

## 현재 프로젝트와의 차이

| 계약/관측 | 8th Wall 실제 관측·공식 문서 | Mobile-SLAM에서 비교할 항목 |
|---|---|---|
| scale·초기화 | responsive400 `NORMAL`, absolute400 `INITIALIZING`. [scale 계약](https://8thwall.org/docs/api/engine/xrcontroller/configure)의 responsive/absolute 별도 | 자체 initialized·metric translation support·fresh/current visual support를 같은 성공 flag로 합치지 않음. `NORMAL`만으로 자체 metric KPI와 직접 비교 불가 |
| 반복 결과 | 599 callback 대 nonrepeat296; [공개 module 결과](https://8thwall.org/docs/api/engine/xrcontroller/pipelinemodule)는 position/rotation/4×4 projection/status/reason 제공 | 자체 `web/js/app.js:1336–1356`은 epoch/sequence dedup·pose freshness·finite matrix gate를 수행. callback FPS와 자체 completed unique frame 처리량 구분 필요 |
| frame·pose identity | 이번 observer의 sourcePublication은 callback 당시 최신 입력 index. callback pose와 source input의 one-to-one 연결을 검증한 값 아님 | 자체 `web/js/vio-wrapper.js:329–371` requestId/inputTimestamp/engineEpoch·future pose 검사·sourceTiming association과 계약 수준이 다름. 동일 clock·timestamp를 자동 가정하지 않음 |
| calibration·sensor | 실제 SDK에 render projection 정보 제공, SDK 내부 K·pixel 처리와 rig의 exact equality 미검증. 가상 gyro orientation은 OS fusion 아님 | 자체 K/distortion/body extrinsic·physical exposure provenance와 matching gate가 먼저 필요. 같은 gray fixture라는 이유만으로 같은 estimator 입력이라고 판정 불가 |

- 이미 자체 코드에 존재하는 dedup·timing identity를 신규 기능처럼 재도입하지 않음. 이번 결과는 입력/출력 계약 차이 확인이며 제품 estimator 개선이나 Flip6 Lost 해결 증거가 아님.

## 이전 evidence와 재현

- 기존 Oct12 기록: mobile responsive400에서 `processStagedFrame`398회, finite transform1104 callback; absolute400에서384회·`LIMITED/INITIALIZING`820 callback; desktop80에서0회. 각 새 run `priorEvidence`에 path·SHA·summary 보존.
- 이 숫자는 **이전 export observer 실행**이며 이번 새 공개 API callback 숫자와 합산하지 않음. 기존 정적 재검사 문서의 '신규 runtime 미검증' 한계는 그 문서의 실행 범위; 이 새 보고서에서 별도 runtime 증거를 추가함.
- 공식 문서 새 GET6건·HTTP200: [primary/manifest.json](../../build/commercial-slam-20261013/8thwall-runtime/primary/manifest.json). URL·획득시각·HTML bytes·SHA 저장. Context7 resolve/query 후 공식 원문 확인.

```bash
node --check scripts/dev/references/eighthwall/runtime-probe.mjs
node scripts/dev/references/eighthwall/runtime-probe.mjs \
  --label rerun-mobile-responsive-400 --mode mobile --scale responsive --frames 400
node scripts/dev/references/eighthwall/runtime-probe.mjs \
  --label rerun-mobile-absolute-400 --mode mobile --scale absolute --frames 400
node scripts/dev/references/eighthwall/runtime-probe.mjs \
  --label rerun-desktop-responsive-120 --mode desktop --scale responsive --frames 120
```

- label 기존 경로가 있으면 실패시켜 덮어쓰기 방지. 원본 dataset·SDK·기존 service·개인 browser profile 보존.
- 검사: 새 MJS parse·sensor adapter regression·신규 파일 whitespace·문서 local link·report/source hash 검사. [verification.json](../../build/commercial-slam-20261013/8thwall-runtime/verification.json).
- 미검증: 내부 수치 solver 전체 복원·원본 source 재빌드, 물리 phone/OS fused sensor, physical calibration·camera/IMU synchronization, SDK 내부 processed frame 정확한 분모, independent GT/SE(3) ATE·RPE·coverage, 동일 입력/동일 하드웨어 accuracy·latency 순위.
