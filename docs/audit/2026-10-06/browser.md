# 브라우저·WASM 기준선 — 2026-10-06

- 범위: 현 브라우저 앱 / Web Worker / 기존 생성 WASM / 현재 C++ 소스의 isolated fresh WASM
- 보존: 기존 dirty 제품 소스, `web/vio_engine.js`, `wasm/build`, `wasm/dist`, dataset·인증서·기존 로그
- 테스트 서버: audit script 내부 localhost 임시 HTTP 서버; COOP `same-origin`, COEP `credentialless`, MIME·no-store 적용
- `web/server.js` 미실행: 시작 시 `logs/test-tumvi.log` truncate 경로 회피
- 환경: Node `22.21.1`, Playwright `1.63.0`, headless Google Chrome `144.0.7559.96`, Emscripten `5.0.0`
- workstation headless 결과; 실기기 camera/IMU·Android·Safari·thermal·장시간 drift 범위 제외

## 실행 결과

| 검사 | 결과 | 해석 |
|---|---|---|
| 기존 `wasm/test_wasm_module.mjs` | exit 0 | 단일 gray frame·API 초기 상태 smoke |
| 기존 `wasm/test_wasm_integration.mjs` | 15 pass / 0 fail, synthetic 20 frame 평균 3.2ms | map 0, 초기화·pose 출력 필수 assertion 부재 |
| 현재 소스 fresh build | `build/audit-wasm/vio_engine.js` 생성, `-j2`, exit 0 | web/dist 배포 아티팩트 교체 없음 |
| fresh 기존 WASM smoke/integration | module exit 0, integration 15 pass / 0 fail, 평균 1.6ms | 기존 테스트 코드의 import 대상만 isolated module로 치환 |
| JS syntax | `web/js/*.js`, `wasm/test_wasm*.mjs`, 신규 audit script 통과 | 정적 parsing 범위 |
| 브라우저 계약 | 정상 계약 4 pass, 결함 계약 TODO 5 재현 | TODO를 acceptance pass로 집계 금지 |
| trajectory exporter | 3 analytic pass | 90도 회전·translation, invalid rotation·duplicate time 거부 |
| 앱 페이지 | `/`, `/test-tumvi.html` WASM load 성공 | 실제 sensor start·실시간 추정 검증과 구분 |
| classic worker 생성 파일 | `VIOWasmFactory is not a function` | `/js/vio_engine_worker.js`를 현재 module Worker에서 load 시 실패 |

## Synthetic frame 실측

- 입력: 320×240 checkerboard/gradient 60 frame, stationary synthetic IMU 10개/frame
- 처리: Worker API로 순차 제출; `setMobileParams`, `setFThreshold`, `setTrackingParams`, `setPnPParams` 모두 success
- busy gate: 중복 frame 거절, 독립 IMU 전달 유지
- malformed image: status 3; 3초 frame gap: status 1·reset
- pose 0 / initialized 0 / map 0: optimization·tracking 속도나 VIO 정확도 근거 제외

| module | load ms | roundtrip p50 ms | p95 ms | p99/max ms | initialized / pose |
|---|---:|---:|---:|---:|---:|
| active web artifact | 54.31 | 2.585 | 4.320 | 13.245 | 0 / 0 |
| fresh current source | 71.23 | 2.530 | 3.415 | 10.130 | 0 / 0 |

- roundtrip: 제출 직전 → Worker result 수신; 생성 image·camera capture·decode·render·live sensor latency 제외
- 이 두 수치만으로 fresh 개선, 모바일 20/30Hz 추적 달성, production 적합 주장 불가
- 직접 evidence: `browser-synthetic-active.json`, `browser-synthetic-fresh.json`

## 생성 아티팩트 계보

| 파일 | SHA-256 |
|---|---|
| `web/vio_engine.js`, `wasm/build/vio_engine.js` | `f683f7d6341105c28cab4e5724f5250a5b9356000a9017c138150c4e6979afce` |
| `web/js/vio_engine_worker.js` | `fa5f5ec9835b884a5dd1b6069976a09fc4045339778daeaa30346c06f5240796` |
| `wasm/build/vio_engine_worker.js` | `5f2a6119754973a4fd6c0d4dd775d19fb2ea93570de55685fb06bd3b6c9dfab1` |
| `wasm/dist/vio_engine.js` | `d89643d02056a5fb193286e995a0fc88f79a614312a28f88a6b6a9e75e7ecda0` |
| `build/audit-wasm/vio_engine.js` | `a50189b8beb375bb2fc9c23cec57903c83dd389288c89512ad30654a8fad14e1` |

- active JS 약 3.9MiB, fresh 약 5.5MiB; 버전·소스·링크 설정 manifest 부재
- SHA 차이만으로 특정 알고리즘 drift 판단 불가; 동일 입력을 active/fresh에 재생하여 행태 비교 필요

## 결함·후속 우선순위

| 우선순위 | 위치 | 재현·영향 | 수정 완료 조건 |
|---|---|---|---|
| P1 | `web/js/vio-worker.js:32`, `web/js/vio-wrapper.js:332` | result에 frame sequence/timestamp 없음; stale pose age·sensor-to-pose latency 계산 불가 | 제출·결과·렌더 시각과 동일 frame ID 전달; result age p50/p95/p99 기록 |
| P1 | `web/js/app.js:1136` | `sendFrame=false` 거절에도 frame/FPS counter 증가 | captured/sent/dropped/completed/pose FPS 개별 집계, rejected frame은 processed 집계 제외 |
| P1 | `web/js/imu.js:370`, `:389`, `:445`; `web/js/app.js:873` | IMU callback 수신 `performance.now`, video `presentationTime`; sample/capture 시각 계약 부재 | sensor timestamp·camera capture timestamp와 arrival time 분리, 실제 동기화 offset/jitter 측정 |
| P1 | `web/js/vio-worker.js:84` | 1100 IMU append로 capacity1024 wrap 후 delayed frame5.8s drain이 1개 반환; 예상 최근0.5s 약100개 | overflow read index clipping, ordered retained samples·drop counter 검증 |
| P1 | `web/js/vio-wrapper.js:151` | IMU subarray 전달 시 backing buffer 전체 전송; byteOffset 누락으로 다른 reading 처리 | offset/length 적용 또는 full contiguous input 강제 계약 |
| P2 | `web/js/vio-wrapper.js:251` | concurrent wait 2개 중 첫 waiter 영구 pending; 자체 timeout도 closure guard로 무효 | 모든 waiter settle 또는 명시적으로 concurrent consumer 거부 |
| P2 | `web/js/vio-wrapper.js:339` | mapPointCount0 result가 기존 map cache 제거 불가 | tracking loss/empty map에서 cache·render state 일치 |
| P2 | `web/js/test-tumvi-app.js:772` | 처리 완료 후50ms를 추가 대기; 1× playback interval = processing+50ms | dataset 절대 시각 기반 deadline, 실제 wall/dataset 속도·drop policy 측정 |
| P2 | `web/js/vio-worker.js:382` | module Worker에서 `importScripts` 불가; classic 생성 파일은 ES6 default 부재 | 단일 ES6 artifact 경로로 통합 또는 Worker 타입에 맞는 loader |
| P2 | `web/js/app.js:1198`, `web/js/renderer.js:199` | rAF마다 같은 latest pose를 trajectory에 다시 append;10000개 버퍼 중복·장시간 뒤 포화 | 새 frame sequence에서만 trajectory append, 렌더는 현재 pose 사용 |
| P2 | `wasm/test_wasm_module.mjs:58`, `wasm/test_wasm_integration.mjs:57` | `0 // PINHOLE` 입력 실제 enum은 KANNALA_BRANDT0, PINHOLE2 | 정확한 camera enum + calibrated initialization/finite pose/GT coverage 필수 assertion |
| P2 | `wasm/test_wasm_module.mjs:65`, `wasm/test_wasm_integration.mjs:120` | module test `FAIL` 문자열만 출력 가능·integration map0도 pass | smoke, tracking, accuracy gate 분리·exit와 assertion 일치 |

- IMU overflow/subarray/concurrent waiter/map cache/result timestamp: `tests/browser/contracts.test.mjs` TODO 계약으로 재현
- clock 오류 크기·실제 device pose age: 본 desktop replay에서 미측정
- 단일 Worker에서 main thread 비차단 ≠ pose latency 보장; capture/readback CPU, 큐, estimator 시간, 렌더 별도 profiling 필요

## 실제 TUM replay 및 독립 평가

- 초기 상태: 예상 `assets/datasets/tum/dataset-room1_512_16/mav0` 미존재; 3 CSV 요청 404, UI `No IMU data found in imu0/data.csv`
- 초기 parser 한계: CSV fetch HTTP 상태 검사 없음; cam404 문자열도 image entry로 parsing 가능
- 공식 dataset 복구·checksum evidence: `dataset-provenance.json`; camera2821 /IMU28122 /GT16541
- active → fresh 순차 실행, 각601 frame /30.001023초 input, max-speed UI replay
- 초기화: 두 module 모두 frame45 /약2.25초 input; pose556 /601 =92.5125%, 초기화 후 LOST0 /non-finite0
- calib: 512×512, KANNALA_BRANDT0, `config/tum_vi_room1.yaml` camera0 calibration/extrinsic, `g_norm=9.81007`
- solver `0.1s /10iter /150feature`; tracker `LK21 /pyramid3 /min_dist20 /edge0`
- noise: `acc_n=0.04`, `acc_w=0.0004`, `gyr_n=0.004`, `gyr_w=0.00002`
- fresh `replaySettings`: configure/mobile/tracking args와 success 캡처; TUM UI의 F-threshold/PnP API 호출 없음, native 비교 시 내부 default 별도 검증 필요

| 실제 replay 지표 | active | fresh current source |
|---|---:|---:|
| 전체 Worker roundtrip p50 ms | 22.675 | 21.790 |
| p95 /p99 ms | 30.735 /40.100 | 28.750 /40.110 |
| max ms | 122.825 | 96.605 |
| tracking만 p95 /p99 ms | 30.834 /40.332 | 28.834 /42.369 |
| SE(3) ATE RMSE m | 0.282962126 | 0.282962036 |
| aligned APE rotation RMSE deg | 6.531665 | 6.531668 |
| 1s RPE translation RMSE m | 0.078069707 | 0.078069690 |
| 1s RPE rotation RMSE deg | 0.881804 | 0.881803 |
| GT associated pose | 556/556 | 556/556 |
| association max delta ms | 3.500 | 3.500 |
| diagnostic Sim(3) fitted scale | 0.892801733 | 0.892801838 |

- active/fresh 556개4×4 matrix 최대 절대 차이 `3.1258840604841964e-7`; 이 bounded input에서 사실상 동일 행태
- SHA 불일치를 근거로 무조건 stale algorithm 탓으로 결론 불가; build provenance 문제와 runtime 결과 분리
- fresh wall19.642235초 /input30.001023초: serial max-speed 처리; 실제20Hz paced sensor·pose age 검증 제외
- desktop native 평가가 일부 시간 동시 실행: latency 수치는 제한된 기준선, 엄밀한 단일 작업 성능 비교 제외
- 초기301 frame prefix의 ATE RMSE0.040810841m 대비601 frame0.282962126m; 별도 alignment window 결과로 장기 drift rate 추정 금지
- 이미지 decode3표본(frame0/45/600): 원본uint16, Chrome canvas R-channel uint8와 Python OpenCV4.12.0 `IMREAD_GRAYSCALE`의262144 pixel 모두 일치 /delta0
- 이미지 parity는3표본 범위; 전체 dataset·native C++ decoder의 모든 버전·실제 camera photometric calibration 보증 제외
- 실제 evidence: `browser-tum-active.json`, `browser-tum-fresh.json`, `browser-tum-summary.json`, `browser-tum-active-metrics.json`, `browser-tum-fresh-metrics.json`, `browser-image-decode.json`
- camera output → body GT 변환: `src/vio_engine.cpp:384`; `config/tum_vi_room1.yaml` extrinsic 사용
- exporter: 제출한 dataset frame timestamp를 association proxy로 사용; engine 자체 pose timestamp 부재 명시
- accuracy scorer: `scripts/evaluation/audit_metrics.py`, SE(3) primary / Sim(3) scale diagnostic /1s true relative RPE /unique timestamp association
- 전체 trajectory·held-out sequence·실기기 production acceptance는 bounded replay 결과와 분리

## 재현 명령

```bash
node wasm/test_wasm_module.mjs
node wasm/test_wasm_integration.mjs
node --test tests/browser/contracts.test.mjs
python3 -m unittest discover -s tests/browser -p 'test_*.py' -v
npm --prefix scripts/dev run browser:baseline

source /home/park-ubuntu/emsdk/emsdk_env.sh
emcmake cmake -S wasm -B build/audit-wasm -DCMAKE_BUILD_TYPE=Release
cmake --build build/audit-wasm --target vio_engine -j2

BROWSER_AUDIT_REPLAY_FRAMES=600 BROWSER_AUDIT_REPLAY_TIMEOUT_MS=180000 BROWSER_AUDIT_OUTPUT=build/audit-browser/tum-active-600 npm --prefix scripts/dev run browser:baseline
BROWSER_AUDIT_WASM=fresh BROWSER_AUDIT_REPLAY_FRAMES=600 BROWSER_AUDIT_REPLAY_TIMEOUT_MS=180000 BROWSER_AUDIT_OUTPUT=build/audit-browser/tum-fresh-600 npm --prefix scripts/dev run browser:baseline
python3 tests/browser/export-trajectory.py build/audit-browser/tum-active-600/browser-baseline.json build/audit-browser/tum-active-600/camera.tum
python3 scripts/evaluation/audit_metrics.py build/audit-browser/tum-active-600/camera.tum assets/datasets/tum/dataset-room1_512_16/mav0/mocap0/data.csv --estimate-frame camera --config config/tum_vi_room1.yaml --expected-frames 601 --output build/audit-browser/tum-active-600/metrics.json
node tests/browser/image-decode.mjs
```

- frame 제한600에서 pause 중 완료1개가 추가로 수신된 이번 실측601; scorer `--expected-frames`는 실제 JSON count 사용
- 로그: `browser-wasm-module.log`, `browser-wasm-integration.log`, `browser-wasm-fresh-tests.log`, `browser-wasm-fresh-configure.log`, `browser-wasm-fresh-build.log`, `browser-contracts.log`, `browser-export-tests.log`, `browser-tum-active.log`, `browser-tum-fresh.log`, `browser-image-decode.log`, `browser-source-fingerprints.log`
- 변경 파일: 신규 audit harness, 브라우저 계약/trajectory export tests, 본 문서·실행 로그
- 제품 단순화 적용 없음; defect 재현·계보·분리된 검증 기준 확보 후 refactor 대상 제시
