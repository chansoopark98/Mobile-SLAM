# Mobile-SLAM 입력·시간·Lost 재계산 — 2026-10-14

- 결론: 보존 Flip6 기록에서 남은 Lost33개를 IMU 적분 계산량·IMU bracket 대기 증가로 설명하는 근거는 없음. 현재 frame에 연결된 성숙 visual factor 부족과 회전 강도의 동반 관측이 강함. 이것은 전체 IMU 영향 부정이나 정확한 인과관계 확정이 아님.
- 비용: 정상 tracking의 backend/Ceres, camera pixel read/main dispatch, 회전 때 frontend tail이 우선 계측 대상. IMU 입력 도착·전달 지연과 IMU 수치 적분 계산량을 분리해야 함.
- 새 산출물: `scripts/dev/analyze-slam-bottlenecks.py`, `build/slam-comparison-20261014/input-timing/recomputed-v3.json`, `reference-timings.json`, `analytic-checks.json`.
- 작업 범위: 원본 numeric JSON/JSONL 직접 재계산 + 현재 source 읽기. 제품 변경·빌드·배포·새 실기기 capture·독립 GT 검증 없음.

## 원본·분모·provenance

- 이전: `build/dev-android/snapshot-2026-10-12T05-40-50-044Z.json`, SHA-256 `78419959777e38dfdd2e5264a182e112dcff41c022c5e94c205b5395f80d9e34`.
- 최적화 후: `build/dev-android/snapshot-2026-10-12T11-59-15-089Z.json`, SHA-256 `11f0bba39fe08871a3bd9c26ab3c72df406f9c330e7a9e3e5fcfa91904ade07d`.
- `startedAtMs <= inputTimestamp×1000 < stoppedAtMs`; 이전479/fresh446/Lost33, 이후607/fresh574/Lost33. camera 전체 source frame coverage가 아니라 수신 완료 결과의 분모.
- 기존 전체609행 stage 통계와 이번607행 통계는 첫2개 pre-window frame 제외 때문에 약간 다름. Ceres/marginalization은 미실행 null을0으로 대체하지 않으며 n574로 집계.
- 분위수: 정렬된 n개 값의 `(n−1)×q` 선형 보간. 단계별 p95 합산 없음. 동일 행에서 가능한 시간차만 계산.
- 현재 HEAD `5d48e4bd7466842dad98928fb1f12988488402df`; 현 source9개 SHA와 bytes를 JSON에 저장.
- active `web/vio_engine.js`: 6,025,183 bytes, SHA `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`; Oct12 phone 기록 당시 binary와 일치.
- `build/research-20261012/baseline/manifest.json` 동결본과 현 파일 비교: `web/js/imu.js`·active loader 일치; App/wrapper/worker·VIOEngine/Estimator/Optimizer/IntegrationBase는 불일치. **현재 C++ source가 현재 served WASM에 빌드되었다는 증거가 아님.**
- 동일 loader hash만으로 최신 JS transport·프로파일까지 Oct12와 동일하다고 주장하지 않음. 이번 raw timing은 역사적 Oct12 실행이며 최신 전체 코드의 phone 재시험이 아님.

## 현재 입력 경로와 clock

1. `web/js/imu.js:40` — DeviceMotion epoch-ms이면 `timeOrigin` 차감, 상대ms면 그대로 사용, 마지막에 `/1000`; 내부 timestamp초. Generic Sensor timestamp도ms→초. 미래1ms 초과·비단조·비유한 입력 거부.
2. `web/js/imu.js:514` — Generic gyro timestamp에 accel 좌우 sample을 보간, 요청60Hz 기준 허용 pair skew25ms. accel/gyro callback 동시 도착을 가정하지 않음. source clock 실패 시 DeviceMotion fallback.
3. `web/js/imu.js:546` — `accelerationIncludingGravity` m/s², rotationRate deg/s→rad/s; alpha=x,beta=y,gamma=z. 짧은 gyro 평균 bias는 metadata로 보존하고 강제 차감하지 않음.
4. `web/js/imu.js:669` — main ring512 sample을 새 Float64Array에 복사. `[timestamp,ax,ay,az,gx,gy,gz]` 7×Float64=56bytes/sample.
5. `web/js/app.js:858` — device→body `(x,−z,y)` 변환 후 `sendIMU`; 10ms interval timer와 frame 직전/Worker busy에서도 flush. `setInterval(10)`은 실제 callback 간격10ms 보장이 아님; main pixel read/DOM 등과 같은 thread.
6. `web/js/vio-wrapper.js:192` — 선언된 IMU view를 다시 `.slice()` 복사한 buffer를 transferable로 Worker 전송. 주석의 zero-copy는 end-to-end 무복사를 뜻하지 않음. frame은 소유 full-view이면 transfer, 아니면 복사; 한 frame만 in-flight.
7. `web/js/vio-worker.js:100` — ring1024, 최대 전달512, timestamp order 검증, 0.5초 이상 오래된 sample/overflow를 명시 기록. frame에 첫 future bracket 한 개까지만 전송; 원본 future sample 재전송 금지, Engine이 carry 소유.
8. `web/js/vio-worker.js:54` — bracket가 없으면40–80ms 비동기 대기. 최근16개 source 간격의 p95×2를40–80ms로 제한. await 동안 IMU message 수신 가능; timeout은 missing_bracket.
9. `web/js/vio-worker.js:375` — grayscale/IMU를 WASM heap에 복사한 뒤 동기 `engine.processFrame()` 호출. 동기 실행 중에는 같은 Worker의 다음 IMU message handler가 실행되지 않음. main 전송의 독립성은 Worker 즉시 소비와 다름.
10. `src/vio_engine.cpp:187` — IMU sample 순서·범위 검증, image timestamp까지 적분, 마지막 accel/gyro를 보간하여 정확한 영상 경계 endpoint 구성. 남은 오른쪽 sample carry 유지.

- camera: requestVideoFrameCallback snapshot과 sourceTiming을 묶고 busy일 때 최신 owned snapshot을 보관/coalesce. 기본 capture는 synchronous; `?capture=worker`는 opt-in (`web/js/app.js:27,428`). 물리 exposure와 metadata 시각 일치 미검증.
- 영상/IMU: document-relative초. transport queue: `performance.timeOrigin+performance.now()` absolute-ms로 main/Worker를 비교 (`vio-wrapper.js:75`, `vio-worker.js:471`). recorder는 sentAtMs에서 timeOrigin을 빼 같은 상대ms로 보존 (`android-snapshot.mjs:125`).
- `workerQueueMs`는 **frame message** enqueue→handler 시작. 모든 IMU message의 queue wait·flush residence time을 측정한 값이 아님.
- `workerProcessingMs`는 bracket wait 이후부터 결과 구성까지; `engineProcessingMs`는 WASM processFrame 동기 호출만. worker−engine 차이에는 heap copy/IMU drain/진단 getter/pose 검사 등이 포함.

## IMU 비용의 세 범위

- **도착/정렬 대기**: event callback·main flush·Worker queue·future bracket 확보. `imuWaitMs`는 이 중 Worker가 frame 처리 직전에 기다린 구간만 측정.
- **새 sample 적분**: `pipeline.imuMs`는 VIOEngine의 `processIMUData()` scope (`src/vio_engine.cpp:304`). Estimator의 sliding-window·temporary preintegration 두 경로 및 상태 전파 포함 (`src/backend/estimator.cpp:117–160`).
- **backend의 IMU 관련 연산**: Ceres residual/Jacobian 반복·bias correction, NEW-frame slide에서 최신 frame dt buffer를 이전 keyframe preintegration에 붙이는 작업 (`estimator.cpp:340–358`), initialization bias 변경 뒤 `repropagate()` 등. 이것은 앞의 `imuMs`에 전부 포함되지 않음.
- 현재 `repropagate()` 호출점: initializer/initial_alignment. 일반 optimizer의 매 iteration마다 전체 raw IMU를 무조건 재적분하는 구조라고 설명하면 틀림. bias Jacobian 보정은 residual 평가에서 사용.
- backend dt merge/repropagate/IMU residual별 계측은 raw에 없음. `backendMs−ceresMs−marginalizationMs`도 일반 bookkeeping/feature 처리 포함이므로 IMU reintegration 시간으로 이름 붙이면 안 됨.

## 원본 재계산: 최적화 후 전체607행

| 지표 | n | p50 ms | p95 ms | p99 ms |
|---|---:|---:|---:|---:|
| `engineProcessingMs` | 607 | 33.525 | 69.767 | 94.278 |
| `imuWaitMs` | 607 | 0.000 | 12.685 | 30.125 |
| `workerQueueMs` | 607 | 0.145 | 0.864 | 1.213 |
| `imuMs` | 607 | 0.105 | 0.235 | 0.329 |
| `frontendMs` | 607 | 10.855 | 41.566 | 63.400 |
| `backendMs` | 607 | 21.605 | 42.466 | 67.886 |
| `ceresMs` | 574 | 20.245 | 29.965 | 34.226 |
| `marginalizationMs` | 574 | 0.005 | 16.054 | 36.042 |
| `backendOtherMs` | 574 | 1.340 | 1.733 | 2.428 |
| `sourceToSubmitMs` | 607 | 31.975 | 44.477 | 50.147 |
| `pixelReadMs` | 607 | 16.935 | 23.502 | 26.728 |
| `snapshotToSubmitMs` | 607 | 3.460 | 17.738 | 23.681 |
| `sourceToCameraCallbackMs` | 607 | 7.475 | 17.350 | 21.012 |
| `roundtripResidualMs` | 607 | 2.505 | 19.291 | 25.288 |
| `poseAgeAtReceptionMs` | 574 | 74.972 | 120.534 | 137.812 |
| `receptionToUiMs` | 607 | 16.275 | 32.812 | 39.076 |

- `roundtripResidualMs = roundtrip−frameQueue−bracketWait−workerProcessing`: reply/serialization/main handler 지연 등을 포함한 나머지. 독립 render GPU 계측 아님.
- `sourceToSubmitMs`: capture metadata→frame 전송; `pixelReadMs`: main snapshot pixel-read scope; `snapshotToSubmitMs`: pixel read 종료→전송. source clock/실제 exposure는 검증되지 않은 proxy.
- pipeline.imuMs 합75.455ms / engineProcessingMs 합23,365.860ms = **0.323%**. 새 sample 적분 scope의 비중이며 Ceres 내부 IMU residual을 포함한 총 관성 연산 비중이 아님.
- frameCopyMs: 이번607행 모두0. transfer가 main pixelRead16.935ms를 없애는 것은 아님.
- native D1 ABBA/microbench는 다른 desktop/fixed-work 실험: `docs/fixes/2026-10-13-tracking-performance-results.md`. IMU microbench 개선2.62%·전체 engine p95 목표실패를 위 phone latency 개선으로 바꾸어 해석하지 않음.

## Lost와 fresh를 분리한 관측

| 지표 | fresh574 p50 / p95 | Lost33 p50 / p95 |
|---|---:|---:|
| `frontendMs` | 10.565 / 29.536 | 26.935 / 66.225 |
| `backendMs` | 21.780 / 42.792 | 0.645 / 1.380 |
| `imuMs` | 0.105 / 0.235 | 0.120 / 0.227 |
| `imuWaitMs` | 0.000 / 12.282 | 0.000 / 11.307 |
| `workerQueueMs` | 0.145 / 0.865 | 0.245 / 0.735 |
| `featureCount` | 120.000 / 120.000 | 53.000 / 82.400 |
| `currentVisualFactors` | 90.000 / 110.000 | 0.000 / 1.000 |
| `eligibleFeatures` | 135.000 / 146.000 | 80.000 / 143.200 |
| `immatureFeatures` | 72.000 / 134.000 | 146.000 / 180.400 |
| `gyroIntervalMaxDegS` | 3.829 / 91.818 | 95.749 / 151.265 |

- 시간 단위ms, feature/factor 단위개, gyro 단위deg/s. gyro는 직전 관측 image timestamp부터 현재 timestamp까지 기록된 IMU의 최대 norm;4개 fresh interval은 해당 범위 sample 없음.
- Lost33개 모두 `insufficient_current_features`; currentVisualFactors0=29개/1=4개. Ceres·marginalization은 null(n0), reject IMU factor0. Lost를 optimizer 계산 과부하 때문에 solve가 길어진 결과라고 단정할 수 없음.
- no-wait477개 중 Lost28(5.87%), positive-wait130개 중 Lost5(3.85%). 이 관측에서 bracket wait와 Lost의 양의 관계가 확인되지 않음.
- 탐색용 gyro30deg/s 구분: 미만442개/Lost0, 이상161개/Lost33(20.50%); sample 없는4개는 제외. 가장 낮은 Lost interval 최대 gyro도68.586deg/s. 30deg/s는 안전/수락 threshold가 아니라 기술적 집계 기준.
- Lost indicator와 단순 Pearson: gyro0.521(n603), frontend0.430, current factors−0.550, imuMs0.033, imuWait−0.010, frameQueue−0.008. 시계열 자기상관·동작·조도·온도·입력 품질 미통제; 유의성 검정/인과 추론/일반화 없음.
- current factor 수는 tracking guard와 직접 연결되므로 그 상관은 독립 원인 증거가 아님. 높은 회전과 장기 track 부족이 왜 생겼는지는 pixels/track lineage 없이는 LK blur·K/extrinsic·rolling shutter·sync·bias로 분리 불가.
- 낮은 backend 시간과 Lost의 음의 관계는 solve를 생략한 결과일 수 있음. 이를 backend를 빠르게 하면 Lost가 늘어난다는 뜻으로 해석하지 않음.
- 이전479행에는 stage 진단 없음(n0). 이전 IMU wait p95/p99=25.802/34.301ms, 이후12.685/30.125ms; 기존 interval/bracket Lost6→이후0. 서로 다른 실제 동작이라 코드의 단독 효과 증명은 아님.

## 센서 도착률과 실제 전달 지연

- 이후 기록: IMU1277개, recorder30.031325초 기준42.522Hz; first/last source span 기준 `(1277−1)/span=42.608Hz`. 분모 차이를 숨기지 않음.
- IMU source 간격 p50/p95/p99/max:22.9825/47.20375/56.34/68.285ms. 요청60Hz와 실측 emission 빈도는 다름.
- raw IMU rows는 wrapper.sendIMU 호출 때 복사한 source timestamp+수치뿐. **1277개 각각의 original callback arrival/flush time/Worker append/Engine consume timestamp는 없음.** 기록 timestamp 간격을 callback arrival 간격으로 부르면 안 됨.
- snapshot5개 lastSample에서 event timestamp→handler arrival 차이 p50≈0.010ms/p95≈0.019ms. 선택된5개 event metadata 관측일 뿐 전체 분포·hardware IMU→browser latency가 아님.
- Generic Sensor timestamp가 document clock보다 약1.1e9ms 미래로 보인 초기 source clock 실패 후 DeviceMotion fallback 이력 존재. recording 중 lost33의 발생 원인으로 초기 fallback을 단정하지 않음; recording 전체는 DeviceMotion.
- main pixel read가 약17ms이고 source emission 약23ms이면 callback·flush jitter를 악화시킬 가능성이 있음. 현재 기록은 per-event arrival/flush 계측이 없어 이 경로의 실제 기여도를 분리하지 못함.

## 8th Wall·ARCore와 같은 의미로 비교할 수 있는 범위

- 참조 원본도 직접 재계산: `reference-timings.json`에 ARCore poses/IMU JSONL·8th Wall result SHA 기록. 다른 장면·날짜·해상도·실행환경이므로 속도/정확도 순위 없음.

| 관측 경계 | p50 / p95 / p99 ms | 의미 |
|---|---|---|
| Mobile-SLAM engineProcessing |33.525 /69.767 /94.278|Flip6 실제 WASM 동기 호출; engine 전체, browser capture 제외|
| 8th Wall frameStart→observer |14.400 /36.500 /42.724|desktop SwiftShader public pipeline scheduling 포함, 내부 solver 시간 아님|
| 8th Wall declared videoTime→fresh observer |41.900 /51.770 /69.244|합성240-frame 입력238 fresh; CPU previous staging 때문에 pose-input 일대일 지연 아님|
| ARCore Session.update |3.519739 /4.480568 /4.952851|동일 Flip6의 다른30초 native run, BLOCKING API 호출/IPC 경계; background solver 전체 아님|

- ARCore native SensorManager 외부 observer source≈461Hz(accel461.341/gyro461.270), p50 interval2.166458ms. Web DeviceMotion42.5Hz와 전달 경로·rate·timestamp 의미가 다름; ARCore 내부 소비 stream과 동일하다는 증거 없음.
- ARCore sensor source→observer callback p50/p95/p99: accel3.473/5.761/18.888ms, gyro3.344/4.733/18.366ms; max≈127ms. 높은 hardware source rate와 callback batching/지연은 동시에 가능.
- ARCore `arrivalElapsedNs−frame.timestampNs` raw 차 p50/p95/p99=198.761/228.707/243.179ms. arrival은 현재 PGM 저장 전 (`ReferenceAudit.java:162`)이나 frame timestamp와 clock/exposure alignment의 독립 인증 없음. 이 차이를 end-to-end latency로 확정하거나 Web pose age와 순위 비교 금지.
- 핵심 비교: Mobile은 frontend+backend를 한 Worker의 동기 processFrame에서 수행; 8th Wall 공개 파이프라인은 CPU/GPU stage를 엇갈리게 사용하고 worker/model 경로를 분리; ARCore 공개 update는 비공개 서비스/카메라 추정 결과를 수신하는 경계. API 한 번의 소요 시간을 같은 solver 계산량으로 취급하면 구조적 오판.

## 필요한 다음 계측

- 모든 IMU에 source/event-arrival/flush-send/Worker-append/Engine-consume 시각·sample sequence·generation을 붙여 queue residence와 실제 source cadence를 분리.
- source image pixels와 physical/metadata timestamp·IMU 원본을 같은 run에서 보존. track ID/age·LK/F inlier·triangulation rejection·current factor 연결 lineage를 기록하여 회전Lost 원인 분해.
- Ceres IMU/visual residual 평가시간·회수, slide dt merge 수/시간, initialization repropagate 수/시간을 독립 계측. `imuMs` 하나로 총 IMU 계산량 주장 금지.
- 동일 장면/움직임·config·WASM/source fingerprint의 controlled repeat 후 capture/readback, frontend, backend 최적화를 각각 비교. 현재 numeric 원본만으로 GT 정확도·센서 지연 causal A/B를 수행할 수 없음.
- UI 갱신은 평균13.741ms 추가, p95 32.812ms. display 예측/보간을 추가하더라도 fresh estimator pose·stale render pose를 분리해야 하며 tracking 회복으로 보고하지 않음.

## 재현·검증

```bash
python3 -m py_compile scripts/dev/analyze-slam-bottlenecks.py
python3 scripts/dev/analyze-slam-bottlenecks.py \
  --output build/slam-comparison-20261014/input-timing/new-recomputed.json
```

- default 두 원본 재계산; `--recording` 반복으로 지정 가능. 기존 출력은 덮어쓰기 거부.
- 독립 analytic fixture5검사 PASS: group4/3/1, 기대 분위수25/38.5/39.7, 누락 IMU stage는null,10Hz source cadence, 출력 overwrite 거부. 근거 `analytic-checks.json`·fixture/result 보존.
- 원본 파일·제품 코드 수정 없음. 분석기 Python compile 성공을 제품 regression/native/WASM/phone 수락으로 부르지 않음.

## 추가 실험: captured raw replay의 시간표 불일치 확인

- root의 새 실제 browser run 원본을 독립 분석: `build/slam-comparison-20261014/replay/current-fast707-live/{report.json,captured-input.json}`, `current-fast707-matched-raw/report.json`.
- live: planned400/accepted392/fresh347, insufficient_current6. 같은 accepted pixels/profile/IMU 누적수를 사용한 raw:392/fresh299, missing_bracket50, current2/final-current2.
- 새 근거: `build/slam-comparison-20261014/input-timing/matched-replay-schedule-failure.json`; source/input/report SHA와392개 frame 대응을 보존.
- `synthetic-browser.mjs:71`은 capture의 **frame 제출 직전 누적 IMU 수**로 batch를 나눔. `:223`에서 그 batch만 보내고 `:230`의 `await vio.waitForFree()`가 끝나야 다음 batch 전송. 실제 live의 frame 대기 중 도착하는 IMU 시간표는 복원하지 않음.
- frame 제출 시점 lastIMU<imageTimestamp인 경우 정확히50개. raw missing_bracket50개와 **50/50 일치**.
- 같은50개 live 결과에는 대기 중 추가IMU가1개 이상 도착, lastIMU≥imageTimestamp 확보 **50/50**; live47개 TRACKING. raw는40.09–40.695ms 대기 뒤 timeout, 그다음에야 다음 batch 전달.
- 예: source frame53, image5.245520s; 제출 전 lastIMU5.228853s. live는1개 추가 후5.245520s endpoint 확보·4.98ms 대기·tracking. raw는같은 추가 sample을 result 이후까지 차단하여40.21ms 뒤 missing_bracket.
- 판정: **실패한 matched-schedule 비교**. 픽셀·총 sample·profile 일치가 arrival schedule 일치를 보장하지 않음. raw의47개 tracking 손실을 동일 입력에서 엔진이 악화한 증거로 사용 금지.
- 수정하지 않은 상태를 증거로 보존. 재생기 개선 시 frame 전후 모든 IMU post 시각을 따로 기록·스케줄하거나, 계산-only control임을 명시하고 각 image의 실제 future bracket를 먼저 전달해야 함. 후자는 원래 live 시간표 재현이 아니며 별도 실험.

- 후속 availability control: `replay/current-fast707-future-bracket/report.json`. 파생 입력에서 captured pre-submit counts를 제거하고 기존 `imuBatchRanges`가 각 frame의 future bracket를 선제 전달. 392개 accepted pixels/source timestamps/profile을 유지하고 **fresh347·current failure6·missing_bracket0** 확인; live의 fresh 개수와 같음.
- 확인 근거: `input-timing/future-bracket-confirmation.json`에 report SHA·summary·`imuPolicy=one_future_bracket`. 이는 bracket 공급 차이가 raw missing_bracket50을 만든 경로를 지지하지만 원래 live event arrival 시간표 복원·pose exact parity·실기기 인과효과를 의미하지 않음. 엔진 코드 수정 없음.
