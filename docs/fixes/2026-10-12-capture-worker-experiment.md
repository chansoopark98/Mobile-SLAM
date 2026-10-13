# VideoFrame 수집 Worker C1 실험

- 날짜: 2026-10-12; [승인 계획](2026-10-12-recorded-recovery-plan.md)의 C1, [T6 명세](2026-10-12-recorded-recovery-test-spec.md#t6-독립-capture-worker-c1) 적용
- 판정: **desktop 계약·동결 A0 합성 GT 통과, desktop 직렬 지연 gate 실패. 동기 수집 기본값 유지**
- 연결: [웹 엔진·API 조사](../research/2026-10-12-web-engines.md), [실기기 baseline](../recordings/2026-10-12-zflip6-after-optimization.md)
- 전화 gate: root의 ADB device/mdns 부재·connect timeout 기록에 따라 `PHYSICAL_GATE_UNAVAILABLE`. 이번 lane의 Chrome144 실행을 Flip6 Chrome154 지원·지연 개선으로 확대하지 않음

## 변경·선택

| 파일 | 변경 |
|---|---|
| `web/js/capture-worker.js` | optional bounded client와 실제 module Worker. VideoFrame 동기 확보→transfer→OffscreenCanvas2D draw/read→기존 integer gray |
| `web/js/app.js` | `?capture=worker` 명시 선택, 기존 pending-gray/전송/IMU 제출 경로 재사용, 공통 snapshot 수락·세대/설정/기하 fence |
| `tests/browser/capture-worker.test.mjs` | capture/수명주기/오류/재진입 계약12개 |
| `scripts/dev/capture-worker-browser.mjs` | 실제 video·VideoFrame·Worker·OffscreenCanvas 픽셀 oracle·fault·App 복구 probe |
| `scripts/dev/synthetic-browser.mjs` | 새 module serve, `--capture sync\|worker` 선택·readback·worker 선택 때만 유한 drain. 기본 `sync`, raw의 worker 선택 거부 |

- URL 기본값: 동기 rVFC grayscale snapshot. 실험: 기존 URL에 `capture=worker` 추가; `grayscale=webgl`과 함께 선택하면 기존 WebGL 경로 사용
- 실험 상태: `window.__mobileSLAM.diagnostics().cameraCapture.workerExperiment`; `physicalTargetVerified=false` 유지
- 첫 actual VideoFrame→transfer→draw/read 성공 후 `active`; 준비/생성 실패는 전송 전 동기 경로 선택 가능
- 전송 시도 뒤 실패: 해당 callback 요청 폐기, 다음 신규 rVFC부터 동기 fallback. 과거 시각으로 나중 video 재촬영0
- 단순화: 완성된 gray는 기존 최신 slot·소유 전송·event 제출 재사용. 새 queue framework·제품 의존성 추가 없음

## 소유권·시각 계약

- 동기 rVFC 구간에서 `new VideoFrame(video,{timestamp:round(existingProxyS×1e6)})`; constructor 전에 await0
- VideoFrame timestamp는 정수 µs metadata. 실제 VIO timestamp는 기존 proxy double 초 그대로 보존, `physicalExposureVerified=false`
- 최대 처리1+최신 대기1. **기존 pending을 close한 뒤** 새 frame 생성, 대체 중 순간3개 보유 방지
- frameSeq·capture generation·configuration ID·client epoch·geometry·presentedFrames·timestamp를 Worker 응답에서 확인; App는 원래 configuration 객체·현재 video/geometry·단조 frame ID도 재확인
- stop/reset/reconfigure는 pending을 즉시 close·전송된 요청 취소 표식. 전송된 객체의 Worker `finally close()` 응답을 받은 뒤 종료·restart 가능
- 기본2초 deadline: 준비·frame 처리 hang과 stop drain 유한. crash/hang 종료 요청은 `releasedByTermination`에 기록하며 명시 `close()` ACK로 계산하지 않음
- Worker는 기존 회전/crop 후 `(77R+150G+29B)>>8`; Y-plane·GPU gray·resize 알고리즘 변경 없음. 처리 해상도 downsample은 기존 App 경로 적용
- Worker `workerDrawReadMs/workerGrayMs`는 duration만 전달. main `mainSnapshotStartMs/EndMs`, `captureWorkerReceiveMs` 별도; Worker timeOrigin과 main 시각을 혼합하지 않음
- Worker 경로의 기존 `pixelReadStartMs/EndMs`는 null. constructor 비용을 실제 pixel read 비용으로 보고하지 않음

## 검증 근거

| 항목 | 결과·원본 |
|---|---|
| 수명주기 unit | 12/12, fail/skip/TODO0; [최종 로그](../../build/recovery-20261012/capture-worker-contracts-final.log) |
| 전체 browser 계약 | Node262/262, fail/skip/TODO0; exporter 독립 해석3/3; [로그](../../build/recovery-20261012/capture-browser-contracts-final.log) |
| syntax | `bash scripts/dev/check.sh syntax` exit0; ESLint/TypeScript 검증으로 명명하지 않음 |
| 기존 동기 실제 browser | Chrome144.0.7559.96, A/tA 확보 뒤 video B 진행·RAF 재읽기0 통과; [원본](../../build/recovery-20261012/capture-sync-default-browser/report.json) |
| C1 실제 browser | Chrome144.0.7559.96, secure localhost, 실제 VideoFrame/transfer/OffscreenCanvas2D/readback; [최종 원본](../../build/recovery-20261012/capture-worker-1791810816204/report.json) |
| 픽셀 oracle | portrait/landscape×none/cw/ccw/half×full/noncenter crop16개. **모든 gray byte**가 기존 Camera CPU와 독립 색상/회전/crop 기대값 모두 일치 |
| frame identity race | 실제 video B가 표시된 후 지연 Worker가 보존 A를 읽어도 A pixels/time 유지. API 존재 검사·fake gray 전송으로 대체하지 않음 |
| 실제 fault | draw error의 remote close ACK1, crash/hang의 ACK0·termination 요청1 구분. 실제 invalid transfer-list·constructor·Offscreen 초기화 실패도 실행 |
| App fallback | draw/transfer로 소비된 실패 callback의 동기 read0·출력0; 다음 신규 callback에서 새 pixels/time으로 동기 수집. constructor/init 전송 전 실패는 해당 신규 callback 동기 수집 |
| 실제 stop/drain/restart | 처리1+대기1에서 stop→pending close1→old remote close1→새 Worker ready→새 세대만 출력. scheduled3/posted2/completed2/accepted1/drop2, 최종 local close1/remote close2/termination0·대기0 |
| App fence | busy 최신 coalescing, configuration 교체·generation/epoch 변경 응답 제출0, 종료 후 inFlight/pending0. Node에서는 늦은 구 Worker ready/error와 완료 callback 재진입도 검사 |

- `scheduled`: VideoFrame 생성 성공 요청, `posted`: 실제 전송 성공, `completed`: 현재 요청 terminal 응답, `accepted`: App snapshot fence 수락; VIO accepted·fresh와 별도 분모
- `coalesced/dropped`, `closedLocal/closedRemote/releasedByTermination`, `inFlight/pending` 함께 보존. 생성·전송 전 실패와 이후 실패를 구분
- `close()`는 해당 참조 해제 증거. crash의 실제 native memory 회수 시각·phone RSS/thermal은 이번 probe에서 측정하지 않음

## 독립 리뷰에서 수정한 결함

- 최초 후보: pending 교체 constructor 내부에서 순간 live3; 완료 callback 재진입 시 posted seq `[1,3,2]`, remote2 발생
- [독립 수정 전 probe](../../build/recovery-20261012/capture-review/edge-probes-before.json), [선작성 red 회귀](../../build/recovery-20261012/capture-review-regression-red.log) 보존
- 수정: pending close를 constructor 앞으로 이동, 완료 처리 뒤 `!inFlight`일 때만 pending dispatch. allocation 순간 peak와 reentrant callback을 회귀 테스트에 포함
- 최초16개 desktop 통과본은 수정 전 SHA의 근거. 최종 판정은 위 최신 probe·manifest의 수정 후 SHA로 구분

## 버전·재현

- 계약 검증 단계 [source manifest](../../build/recovery-20261012/capture-worker-final-manifest.json); 후속 통합·측정의 [최종 원문·source manifest](../../build/recovery-20261012/capture-integration/final-manifest.json)
- `capture-worker.js` SHA256: `6447aff5c06b4f189d1e4dc2d057fab617e1b2ee18278e11f8a048519cddf5ad`
- `app.js` SHA256: `173d987f4d8537a30c2adeb2950b49a673de8cd10fb84da82e966c2a6f70f595`
- 독립 픽셀 기준 Camera SHA256: `fa31559030d598bd0fb14d152d8f9a9b5c4a383096bdc7741f955d12530b01cd`

```bash
node --test tests/browser/capture-worker.test.mjs
bash scripts/dev/check.sh syntax
bash scripts/dev/check.sh browser-contracts
node scripts/dev/capture-worker-browser.mjs
```

- browser probe는 매 실행 새 `build/recovery-20261012/capture-worker-TIMESTAMP/`에 결과 보존. `CAPTURE_WORKER_OUTPUT=NEW_DIRECTORY`, `BROWSER_EXECUTABLE`로 명시 변경 가능
- root의 실제 App/Camera/IMU/WASM 합성 통합: 기존 명령에 `--capture worker` 추가. 동기 control은 `--capture sync` 또는 옵션 생략. source bytes·GT·estimator profile·association guard 변경 없음
- 합성 adapter의 source 캡처 검증·SHA observer 부하는 그대로 존재; worker 선택의 capture timing을 phone 또는 무계측 성능으로 사용 금지

## 실제 App·동결 A0 합성 GT 통합

- 원본: 이전 DeviceMotion GT proof의 `build/motion-20261010/fast-rotation-101/input.json`; SHA256 `13068480d25b9b0b231b8e1a69b2847345cb7c11709be47ba14983fcd29c8b7b` 재확인
- 엔진: 격리 보존 `build/research-20261012/baseline/vio_engine.js`, SHA256 `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`. 기각된 A2 엔진 사용0·엔진 수정0·native compile0
- 경로: 실제 HTMLVideo/Camera/DeviceMotion/App/Worker/WASM/renderer; generated image20Hz·IMU60Hz·400frame, independent analytic body GT, camera pose→body 변환·단일 SE(3) scale1·timestamp1:1
- 가상 clock 직렬 worker→sync: 두 실행 모두 VIO accepted/completed395, 초기 camera warmup5개 제외; fresh359/400(89.75%), `[5,20)`300/300, reset0
- **395개의 source ID·timestamp·gray SHA·프레임 전 IMU 누적 수·pose matrix exact 동일**, 제출 IMU1199행도 exact 동일. 실제 readback profile은 solver0.06s/10iterations/features120, LK21/3·min_dist30, rotationRescue1·PnP0
- C1 actual readback: `state=active`, scheduled/posted/completed/snapshot accepted400·remote close ACK400, drop/error/termination0, 최종 inFlight/pending0. Worker 선택의 동기 `captureGrayscale` 호출0
- 두 실행 GT: ATE0.059866m/rotation APE1.882184°, 1초 RPE0.028699m/0.499132°, invalid/unassociated0·동결 GT gate 통과
- source pixel readback400·ledger400 보존. Worker IMU loss counters0·inverse transform 일치; browser startup `nonfinite=1`은 별도 보존하며 모든 sensor counter0이라고 보고하지 않음
- 근거: [명령·초기 fingerprint](../../build/recovery-20261012/capture-integration/manifest.json), [worker 원본](../../build/recovery-20261012/capture-integration/fast101-worker/report.json), [worker GT](../../build/recovery-20261012/capture-integration/fast101-worker-evaluation.json), [sync 원본](../../build/recovery-20261012/capture-integration/fast101-sync/report.json), [sync GT](../../build/recovery-20261012/capture-integration/fast101-sync-evaluation.json)
- 가상 clock 결과의 queue/capture/pose-age 값을 실제 latency로 사용하지 않음. 기존 공개 seed101만 실행; 추가 held-out/native captured-input 비교·실기기 정확도는 별도
- 최초 deterministic 원본의 `disposition`은 async 완료 전 snapshot 상태를 읽어 일부 `accepted=true` 행에도 `not_captured_rate_or_lifecycle` 표기. 판정은 accepted/completed/fresh bool·timestamp·보존 pixel/IMU로 수행; 이후 adapter는 Worker drain 뒤 상태를 기록. 원본은 수정하지 않음

## 실제 desktop rAF 직렬 ABBA: 지연 gate 실패

- 사전 동결 [측정 계약](../../build/recovery-20261012/capture-integration/performance-protocol.json): `A sync→B worker→B worker→A sync`, 같은 source/A0 artifact/profile, `live-paced-native-raf`, DeviceMotion, 각400frame. 총4회·재시험0
- root가 다른 native build/replay를 중단한 독립 window; [실제 명령·시각·exit code](../../build/recovery-20261012/capture-integration/performance-execution.json) 보존
- 모든 실행 실제 rAF·video readback400·source ledger400. C1 각2회 actual Worker active·scheduled/posted/completed400·close ACK400, 오류/취소/termination0
- 측정 adapter SHA256 `b59515b15a7c29d9086eb49926aa85c9dceb7f268f1485cca492883b37c04cdf`: App callback 호출 전후 wall-clock duration, 별도 비동기 수집 결과 task, 실제 result receivedWallMs. source readback/hash observer는 callback 계측 밖이며 전체 source→submit proxy에는 포함
- 사전 primary 기준: 두 A/두 B의 **all population pooled nearest-rank p95**, warmup 별도. source→submit/engine은 실제 accepted/completed784행씩; 누락은400×2 source 분모에서 유지. run 사이 fresh-gap 생성0

| all population, ms | sync p50 / p95 / p99 | C1 p50 / p95 / p99 | p95 변화·판정 |
|---|---:|---:|---|
| main App rVFC callback, 각800개 | 2.115 / 4.035 / 5.750 | 0.075 / 0.130 / 0.965 | −96.78%, 감소20% 조건 통과 |
| 비동기 capture 결과의 main task, C1 800개 | 해당 없음 | 0.610 / 1.730 / 2.890 | 분리 계측; callback p95와 합산 금지 |
| source→submit proxy, 각784개 | 16.135 / 37.170 / 51.735 | 18.745 / 40.750 / 50.060 | **+9.63%, 악화5% 상한 실패** |
| engineProcessing, 각784개 | 21.460 / 38.240 / 44.555 | 21.430 / 39.590 / 48.920 | +3.53%, 상한 통과 |
| fresh result 수신 간격, 711/710쌍 | 49.995 / 65.890 / 75.240 | 49.685 / 73.570 / 84.225 | **+11.66%, 악화5% 상한 실패** |
| fresh pose age proxy, 713/712개 | 42.440 / 60.165 / 73.705 | 45.075 / 68.925 / 80.920 | +14.56%, 별도 진단 |

- warmup300×2의 p95: callback3.975→0.110ms, source→submit20.310→30.440ms, engine37.545→40.085ms, fresh wall gap62.930→73.640ms, pose age59.285→68.235ms. primary와 별도로 보존; 유리한 시간창으로 선정 기준 변경0
- main callback 점유 감소와 최종 지연 개선은 별도 결과. 추가 비동기 단계·task 비용의 단일 원인 귀속은 이 기록만으로 확정하지 않음

| 실행 | accepted / completed | fresh /400 | warmup /300 | ATE m / APE ° | RPE m / ° | GT/reset |
|---|---:|---:|---:|---:|---:|---|
| A sync1 | 392 /392 | 356 | 300 | 0.057162 /1.774202 | 0.028012 /0.512703 | PASS/0 |
| B C1 1 | 391 /391 | 355 | 300 | 0.048471 /1.502385 | 0.024759 /0.496740 | PASS/0 |
| B C1 2 | 393 /393 | 357 | 300 | 0.052464 /1.894299 | 0.032332 /0.506112 | PASS/0 |
| A sync2 | 392 /392 | 357 | 300 | 0.067065 /2.087144 | 0.032525 /0.517516 | PASS/0 |

- pooled fresh: sync713/800(89.125%), C1712/800(89.000%), −0.125pp; warmup 양쪽600/600. coverage−1pp·reset 증가0·모든4회 GT 조건 통과
- live accepted source pattern은 서로 다름. 각 실행의 frozen GT 판정과 deterministic exact-input/pose proof를 각각 보존; live ATE 차이를 수집 알고리즘의 정확도 개선으로 해석하지 않음
- 최종 **desktop 상대 지연 gate FAIL**, 실험 코드/실패 원본 보존·동기 기본값 유지. 이 desktop 결과로 실제 Flip6의 개선 또는 악화를 주장하지 않음
- 전체 per-run·all/warmup p50/p95/p99·입력 계약·fingerprint: [comparison.json](../../build/recovery-20261012/capture-integration/comparison.json), [독립 재계산 스크립트](../../build/recovery-20261012/capture-integration/compare.py)

## 남은 수락 조건

- 실제 Flip6 Chrome154 construct/transfer/draw/read probe 및 원본 image/IMU 기록
- 같은 accepted-input의 native/실제 Worker/WASM·held-out SE(3) ATE/RPE/rotation/coverage 무회귀
- 실제 target 직렬 비교: main callback p95≥20% 감소, source→submit/engine/fresh-gap p95 악화≤5%, coverage 감소≤1pp·reset 증가0. 위 desktop 실패를 상쇄하거나 target 통과로 명명하지 않음
- CPU/GPU 경쟁, 지속 실행 메모리·온도·센서 arrival/drop, C1이 실제 main starvation을 줄이는지 확인
- 어느 gate라도 실패·불가하면 동기 기본값 유지. C1 계약 통과를 A2 visual recovery·실기기 Lost 해소로 보고하지 않음
