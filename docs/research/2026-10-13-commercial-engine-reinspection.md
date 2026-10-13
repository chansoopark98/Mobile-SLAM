# 상용 엔진 재검사와 입력·출력 계약 적용

- 확인일: **2026-10-13**, Asia/Seoul. 기존 상용 자산에 REA·WABT·GNU ELF 도구를 **새로 실행**; 이전 출력 재인용만으로 완료 판정하지 않음.
- 결과: 8th Wall의 JS→WASM 입력 경계, ARCore SDK forwarding stub과 service binary 분리 확인. **ARKit/Vuforia 바이너리 부재, proprietary solver 수식·실기기 성능·정확도 미복구/미검증**.
- 적용 판단: 입력 시각의 출처를 요청과 함께 보존해 **완료 pose와 동일 프레임의 timing metadata**를 표시. 상용 엔진 내부 알고리즘을 복사하는 변경 아님.
- 범위: 이 문서와 `build/flip6-analysis-20261013/commercial/`; 제품 구현은 별도 integration lane. 기존 dirty 작업·dataset·서비스·레퍼런스 자산 보존.

## 실행 근거

- 도구: REA npm **6.4.0**, WABT **1.0.42**, GNU readelf/objdump **2.38**, Node **v22.21.1**, Python **3.13.5**. 기존 설치 사용; 신규 설치·등록·로그인 없음.
- 실제 명령 **20개, exit0**: [commands.json](../../build/flip6-analysis-20261013/commercial/commands.json) 12개와 [extra-commands.json](../../build/flip6-analysis-20261013/commercial/extra-commands.json) 8개. argv·duration·stdout/stderr hash 보존.
- [manifest.json](../../build/flip6-analysis-20261013/commercial/manifest.json): 입력/도구 SHA256·모듈 exports·memory; [glue-boundaries.json](../../build/flip6-analysis-20261013/commercial/glue-boundaries.json): 배포 JS AST **119,112 nodes**, 선택한 경계 호출8개.
- REA `inspect-artifact` 성공은 byte identity·format inventory 범위. 결과의 `complete-within-substeps`는 전체 semantic graph·수치 알고리즘 복원 완료가 아님. 이전 heap exhaustion 분석을 새 성공으로 승격하지 않음.
- Context7 ARCore·ARKit·8th Wall 조회 후 공식 웹/고정 MIT source 재조회. [primary-manifest.json](../../build/flip6-analysis-20261013/commercial/primary-manifest.json) **9파일/실패0**, URL·획득시각·SHA256 보존. Apple web Markdown 렌더 실패는 직접 공식 Markdown GET으로 보완.

| 자산 | 이번 직접 관측 | 한계 |
|---|---|---|
| 8th Wall `xr-slam.js` | JS5,538,007B, embedded WASM3,826,095B, export59, nonshared memory 초기16MiB/최대2GiB | 선언 maximum은 실제 peak memory가 아님. 이름만으로 solver·threshold 추정 금지 |
| 8th Wall `xr.js` | embedded WASM415,711B, export50, nonshared memory 초기96MiB/최대2GiB | world-SLAM module과 별도 runtime/semantics 경계. 크기 합만으로 runtime working set 비교 불가 |
| ARCore `libarcore_sdk_c.so`, x86_64 | 68,072B. `ArSession_update`13byte의 간접 jump를 objdump로 확인; `dlsym` import·Android/log/dl 의존성 | SDK client를 VIO core로 오인 금지. [stub 출력](../../build/flip6-analysis-20261013/commercial/arcore-client-update-stub.stdout) |
| ARCore1.56.0 emulator APK | ZIP97,705,272B; native entry4개. 실제 `lib/x86_64/libarcore_c.so`47,269,520B | 이번 APK 검사는 ZIP metadata만. service solver의 실행·disassembly 없음. [container](../../build/flip6-analysis-20261013/commercial/arcore-service-container.json) |
| ARKit | Linux checkout에 Apple SDK binary·Apple runtime 없음 | 공식 API·문서 비교만. ARKit 실행·역공학 완료로 보고하지 않음 |
| Vuforia | `assets/references/Vuforia/`는 독립 host-contract fixture·issue tracker·manifest만; SDK ZIP/`libVuforia.so` 없음 | fixture CTest 성공은 SDK 호출 성공이 아님. 추가 계정·download·key 취득 없음 |
| Mobile-SLAM served 후보 | 현 `web/vio_engine.js` 한 번의 byte read 후 별도 snapshot; embedded WASM5,347,031B, import61/export23, nonshared256/512MiB | 아래 SHA는 Oct12 snapshot과 동일. 현재 dirty source 재빌드·HTTP served identity 증거가 아님 |

- world WASM SHA256: `780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422`.
- ARCore client SHA256: `8c9c0af1f5225a31c0baf21c8785d362fef64ad133b645ddc5d2cedb68d71b4b`.
- 자체 JS SHA256: `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`; WASM: `86206cfbd5edc6566ec1075794345c03f17018c3d3a0b339144fc49ae4c29f11`.
- [opcode-summary.json](../../build/flip6-analysis-20261013/commercial/opcode-summary.json): xr SIMD1,433, xr-slam34,052, 자체65,790; atomic은 모두0. **정적 opcode 존재만 확인**. SIMD 숫자로 성능·hot kernel 비율을 순위화하거나 atomic0으로 전체 browser worker 부재를 주장하지 않음.

## 상용 경계와 현재 구현의 차이

| 비교 | 직접 관측/공식 계약 | 현 프로젝트 판단 |
|---|---|---|
| 8th Wall frame staging | `stageFrame` 호출에 `videoTime`, `frameTime`, 별도 `performance.now()` 기반 값이 각각 전달됨. `repeatFrame`이면 stage/process 생략. 별도 `setEventQueue`, `prepareYuvBuffer`, `processStagedFrame` export 존재 | 이미 최신 pending slot·rVFC dedup·capture Worker가 있음. 같은 비동기 구조를 재구현할 이유 없음. **입력 timing과 결과의 identity 연결**을 검토 |
| 8th Wall sensor queue | DeviceMotion arrival time, event timestamp, interval을 별도 필드로 직렬화. 오래된 queue를 비우는 wrapper 경계 확인 | 현재 IMU drop·clock 진단 유지. 이 값들의 존재는 실제 exposure 동기·bias 보정 정확도 증거가 아님. wrapper의5초 값을 자체 propagation budget으로 복사하지 않음 |
| ARCore frame/camera | [Frame](https://developers.google.com/ar/reference/java/com/google/ar/core/Frame)은 capture timestamp, 별도 Android camera timestamp·metadata correlation을 제공. time base는 명시적으로 정의하지 않음. [Camera](https://developers.google.com/ar/reference/java/com/google/ar/core/Camera)는 frame에 대응하는 physical camera pose와 tracking state 제공 | SDK별 clock을 자동 동일시하지 않음. 자체 `timestampS/source/timeOriginMs/association/physicalExposureVerified`를 **해당 request와 함께** 보존해야 오인 방지 |
| ARKit 품질·회복 | [Lifecycle](https://developer.apple.com/documentation/arkit/managing-session-life-cycle-and-tracking-quality)은 normal/limited/notAvailable과 reason·relocalization을 구분. [timestamp](https://developer.apple.com/documentation/arkit/arframe/timestamp)는 frame capture 시각 | JS consumer에 `tracking_limited`·quality 검증 경로가 있으나 현 C++ producer의 limited recovery는 미구현·미수락. 상용 상태명을 근거로 current-factor gate를 완화하거나 inertial output을 metric으로 승격하지 않음 |
| Vuforia backend | [Fusion](https://developer.vuforia.com/library/vuforia-engine/environments/device-tracking/vuforia-fusion/vuforia-fusion/)은 ARKit/ARCore 등 실제 provider 선택. 현재 문서의 VISLAM deprecation은 ARKit/ARCore 미지원 기기 범위에 적용 | Vuforia version만으로 독립 VIO algorithm을 고정할 수 없음. future comparison에는 provider·frame/pose timestamp·좌표 epoch를 함께 기록 |

- 8th Wall 공개 MIT source의 [tracking-controller.ts](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/tracking-controller.ts)와 [sensors.ts](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/src/sensors.ts)를 고정 SHA에서 새로 획득하여 위 입력 경계를 교차 확인.
- [Open Source](https://8thwall.org/docs/open-source): MIT framework와 별도 SLAM binary를 구분. 기존 `engine/restored/pseudocode/`·복원 header를 proprietary 원본 소스처럼 사용하지 않음.
- 배포 binary의 제한적 license와 framework MIT는 별개. 이번 raw binary 결과는 구조 관측의 근거이며 제품 채택 근거는 공식 API와 공개 MIT boundary에 한정; proprietary 수치 구현·threshold·데이터를 제품으로 이식하지 않음.

## 적용 후보와 결정

1. **채택 대상으로 전달: 요청별 timing metadata 보존**
   - 관측: [app.js](../../web/js/app.js)의 `_consumeResult()`가 완료 result의 clock 출처 대신 현재 `this.lastFrameTiming.source`를 사용.
   - `_handleWorkerMessage()`가 완료 직후 `onFrameComplete`를 호출하고 다음 pending frame을 제출할 수 있어, 화면이 이전 완료 pose를 소비할 때 `lastFrameTiming`은 다음 요청의 값일 수 있음.
   - 수정: `sendFrame()`에서 입력 timing metadata를 snapshot하고 request identity에 연결; 완료 result에 같은 snapshot 전달; UI는 그 metadata의 source를 사용. reply input/epoch 불일치 시 metadata 제거, 미제공 시 camera time proxy 표시. metadata 내부의 임의 timestamp 값을 별도로 검증한다는 뜻은 아님.
   - 검증: A 완료 후 B 즉시 제출·A/B clock source 다름·caller metadata 사후변경·stale epoch·timestamp mismatch·metadata 없는 호출. A pose에 B clock label이 붙지 않아야 함.
   - 기대 효과는 기록/표시의 정확성. estimator latency·Flip6 ATE 개선으로 확대하지 않음. 구현·테스트 결과는 integration 보고서에 기록.
2. **보류: metric pose와 input timestamp equality 강화**
   - 최초 검토에서는 status5에만 strict equality가 있고 status2는 fresh/valid flag에 의존하는 차이를 발견.
   - 추가 확인: [transport.test.mjs](../../tests/browser/transport.test.mjs)에 **poseTimestamp=inputTimestamp−0.01s**를 의도적으로 보존·보고하는 기존 계약 존재. 무조건 equality로 바꾸면 이를 깨므로 이번 적용 제외.
   - 후속 질문: corrected pose의 시각과 처리 frame 시각의 허용 관계·fresh의 정의를 명시적으로 합의할 필요. 현재 엔진이 잘못된 stale pose를 실제 발행했다는 증거는 없음.
3. **후순위: limited reason 표시**
   - `_consumeResult()`는 limited 상태에서 result.reason을 숨김. 제한 원인의 사용자 표시·기록 대응 검토 가능.
   - metric/limited 구분은 JS consumer의 검증 계약. 현 C++ producer는 pipeline-v1이며 limited recovery 구현·수락 근거가 없다. 이번 핵심 변경에 추가하지 않으며 상용 state명 도입을 새 알고리즘 개선으로 계산하지 않음.

## 재현과 수락 범위

```bash
python3 build/flip6-analysis-20261013/commercial/reinspect.py
python3 build/flip6-analysis-20261013/commercial/inspect-extra.py
python3 build/flip6-analysis-20261013/commercial/fetch-primary.py
git diff --check -- docs/research/2026-10-13-commercial-engine-reinspection.md
```

- 실행 대상은 기존 로컬 파일; application·WASM target 실행 없음. `fetch-primary.py`는 공개 text9개만 최대2MiB/파일로 저장; binary download 없음.
- source snapshot·tools SHA·실패 stderr를 보존. 재실행하면 현재 byte 및 live 문서가 바뀔 수 있으므로 이전 manifest와 비교하고 동일 snapshot으로 오인하지 않음.
- 완료 범위: 상용 자산 정적 재검사·공식 문서 재조회·현재 코드와 비교·작은 적용 후보 전달.
- 미검증: ARKit/Vuforia runtime, 신규 ARCore/8th Wall runtime, 동일 image/IMU 상용 A/B, phone thermal/latency, physical exposure/calibration, independent GT/SE(3) ATE·RPE. 신규 phone capture 없음.
