# 모바일 회전·웹 입력 처리 수정 결과

- 날짜: 2026-10-10, Asia/Seoul. 기준 revision: `3177951eed7dd89243426c0ba7f733bfeb6b0802`, 시작 working tree clean
- 요청: Z Flip 6 책상 테스트에서 근거리·원거리 모두 빠른 회전 시 Lost/발산, 2m 이상 장면의 특징점 감소; 처리 속도·웹 데이터·IMU 결함 수정
- 판정: **기본 OFF의 TUM-VI 회귀 통과**, **실험 ON의 빠른 합성 회전 개선**, **ON의 TUM-VI 정확도 유지 gate 실패**, **실제 휴대폰 해결 미확인**
- 적용 판단: 기존 정확도 유지 조건에 따라 회전 복구 기본 OFF. `?rotation-rescue=1`에서만 실험 ON
- 보존: 원본 NAS `/mnt/backup/SLAM`, 전화 기록, dataset, TLS, 회귀용 핵심 baseline binary·실패 후보. 기존 빌드 디렉터리는 유지하고 재빌드 캐시는 갱신. 검증 당시 dependency 추가·commit·push 없음, `main` working tree 수정 상태였음. Git 반영 범위는 아래 기록
- 계획: [수정 개발 플랜](2026-10-10-rotation-plan.md). baseline source/binary: `build/motion-20261010/baseline/`

## 기본 적용한 결함 수정

- `web/js/app.js`: Worker busy이면 최신 소유 pixel snapshot 보존·병합, frame 소비와 pacing 시각 갱신 보류. IMU flush는 계속 수행
- busy 종료 후 첫 ready rAF에서 보존 packet 제출; 같은 영상 재읽기·불필요한 다음 frame 대기 제거. API 미지원 경로도 busy 중 영상 읽기 보류
- `web/js/imu.js`: Generic Sensor의 callback 순서에 따른 오래된 가속도 hold 제거. gyro source timestamp의 exact acceleration 또는 양쪽 bracket 선형 보간으로 한 번씩 출력
- accel/대기 gyro history 각각 512개 제한, 양쪽 시차 각각 `1.5/requestedHz` 이내. 불가능한 bracket·capacity·lifecycle drop과 원시 bracket timestamp 기록; 하드웨어 동기화 증명은 아님
- `src/backend/estimator.cpp`: 정상 `solve_flag=1`, 양수 depth 점을 `init_depth ±0.01m`라는 이유로 map export에서 버리던 필터 삭제. 기본 2m 부근의 얇은 구간 결함이며, 모든 `>2m` 점을 삭제하던 cutoff는 없었음
- 정상 2m/5m/10m export와 실패 depth 제외 회귀 확보. 화면의 원거리 특징점 감소 전체 원인이나 실측 깊이 정확도를 해결했다고 판정하지 않음

## 실험 ON 경로

- 초기화 이후 tracker 이전 시각과 IMU endpoint가 일치하고 현재 endpoint까지 coverage가 있을 때만 사용. reset·invalid SO(3)·coverage 누락 시 ordinary LK 유지
- camera 회전: `R_Cnew_Cold = R_BCᵀ R_afterᵀ R_before R_BC`; 기존 camera lift/project로 왜곡·extrinsic 반영
- ordinary LK/F consensus가 원본 track 수의 과반이면 기존 결과 유지. 약한 consensus에서만 큰 pixel 변위의 회전 hint로 재시도
- pyramid 재사용. 기존 성공점을 교체할 때 patch L1 오차 절반 미만, 모든 채택 후보에 local reverse 오차 `≤0.5px` 요구. 실제 채택 시에만 mask 병합·F 재추정·최종 pruning
- C++ `rotation_rescue_enabled=0|1`, Worker `rotationRescueEnabled` 실제 readback 사용. ON 요청은 setter/getter 둘 다 필수; configure는 OFF 초기화, reset은 설정 유지, 처리 중 값 변경은 상태 reset
- 첫 전체 회전 후보·radius-only·photometric/reverse 후보 및 full TUM gate 실패 결과 보존. gate cap·noise·guard·smoothing으로 결과를 통과시키지 않음
- 추가 LK·patch/reverse 비용 존재. ON의 실제 휴대폰 처리 속도·열 안정성 미검증

## TUM-VI 전체 구간

- body frame 변환 후 일대일 timestamp association, 단일 SE(3), scale=1. 위치 ATE·자세 APE는 RMSE; Sim(3)는 진단용
- 기준: baseline 대비 위치/회전 RMSE 5% 초과 악화 또는 coverage 1 percentage point 초과 감소 시 실패

| 구간 | baseline ATE m / APE ° | 기본 OFF ATE m / APE ° | OFF pose/전체 frame | ON ATE m / APE ° | 판정 |
|---|---:|---:|---:|---:|---|
| room1 | 0.05174410 / 1.16332285 | 0.05174545 / 1.16332455 | 2776/2821 (98.40%) | 0.04338553 / 1.35785528 | OFF 통과, ON 자세 회귀 |
| room4 | 0.04692054 / 0.91832147 | 0.04692050 / 0.91831158 | 2148/2228 (96.41%) | 0.05407656 / 1.19163638 | OFF 통과, ON 위치·자세 회귀 |

- OFF association: room1 2719/2776, room4 2108/2148; ON pose: room1 2776, room4 2152
- 근거: `build/motion-20261010/opt-in/default-control-gate.json`, `default-room{1,4}-evaluation.json`, `enabled-room{1,4}-evaluation.json`
- native `VIOEngine` replay 결과; PC `VIOSystem`·실제 휴대폰·browser 속도 검증으로 확대하지 않음

## 빠른 회전 합성 입력

- 20초·400 frame, 영상 20Hz·portrait 480×640, IMU 60Hz. 독립 analytic body GT/IMU·ray-cast 영상; GT는 estimator에 전달하지 않음
- 첫 5초 warm-up 이후 복합축 빠른 회전. 확인 seed2203의 analytic 최대 각속도 `119.45°/s`; 실제 전화 허용 속도 근거는 아님
- seed101/707 입력 고정. seed2203은 알고리즘 동결 후 새 생성. body frame·일대일 source association `≤0.0005s`·SE(3) scale=1·회전/RPE 포함

| 입력·경로 | ATE m | APE ° | warm-up 이후 pose/300 | 전체 pose/400 | 합성 gate |
|---|---:|---:|---:|---:|---|
| seed101 기존 baseline native | 7.081512 | 85.286813 | 182/300 (60.67%) | 241/400 | 실패 |
| seed101 native ON | 0.048075 | 1.251541 | 300/300 (100%) | 359/400 | 통과 |
| seed707 기존 baseline native | 43.896390 | 93.175794 | 180/300 (60.00%) | 239/400 | 실패 |
| seed707 native ON | 0.040171 | 1.116521 | 300/300 (100%) | 359/400 | 통과 |
| 확인 seed2203 native OFF | 1.241776 | 23.920349 | 223/300 (74.33%) | 282/400 | 실패 |
| 확인 seed2203 native ON | 0.021716 | 1.295713 | 300/300 (100%) | 359/400 | 통과 |
| 확인 seed2203 Generic Sensor App ON | 0.019045 | 0.769660 | 300/300 (100%) | 359/400 | 통과 |

- ON 100%는 warm-up 이후 고정 300 frame 분모; 전체 coverage는 359/400=89.75%. App은 395 input rows 처리; 누락 frame도 고정 분모에 포함
- 확인 seed2203 1초 RPE: native ON `0.018703m / 0.559452°`, Generic App ON `0.017105m / 0.417641°`, 각각 339쌍
- DeviceMotion App seed101: OFF `0.364574m / 12.535321°`, ON `0.068781m / 2.215236°`; warm-up 이후 coverage `85.33%→100%`
- 근거: `build/motion-20261010/{baseline,opt-in}/*evaluation.json`, `confirmation-fast2203/{spec,manifest}.json`; App은 `live-deterministic` desktop mock 실행
- 중첩 부하의 자연 rAF seed707: warm-up 이후 `233/300=77.67%`, coverage gate 실패 보존 (`accepted/live-fast707-raf-r1-evaluation.json`). 최종 순차 독립 3회는 아래 결과 참조

- 최종 자연 desktop rAF·Generic Sensor·실험 ON 순차 3회: 모든 source pixel/timestamp 계약·합성 GT gate 통과, reset 0. 실제 휴대폰·열·20Hz 외 처리율 증명 아님

| 반복 | 처리/입력 | 고정 warm-up 이후 coverage | ATE m / APE ° | 초기화 이후 engine p50/p95/p99 ms |
|---|---:|---:|---:|---:|
| r2 | 385/400 | 99.67% | 0.056638 / 1.884278 | 23.49/43.85/56.61 |
| r3 | 383/400 | 99.00% | 0.044616 / 1.746192 | 25.74/46.57/53.63 |
| r4 | 382/400 | 98.67% | 0.050176 / 1.749969 | 27.03/49.20/53.88 |

- 근거: `opt-in/raf-fast707-on-r{2,3,4}/{report.json}`, 각 `.evaluation.json`; 관측·source 검증 비용을 포함한 desktop wall clock

## 처리·회귀 검증

- busy/pacing 제어 fixture: busy `5000ms`→ready `5016ms` 즉시 제출; 기존 다음 pacing `5067ms`까지의 51ms 대기 제거. 실제 engine/전화 latency 향상 증명은 아님
- 실제 desktop HTMLVideo/canvas snapshot 소유 확인: `opt-in/video-snapshot/report.json`; `physicalExposureVerified=false`, engine 정확도 검증과 별개
- native: GoogleTest 160개, CTest 13/13 통과 (`opt-in/native-test-counts.json`, `native-tests.log`)
- browser: Node 221/221, TODO/skip 0; exporter analytic 3/3 (`opt-in/browser-final.log`)
- quick: syntax 및 Python 9+19=28 통과 (`opt-in/quick-final.log`). plain JS이므로 ESLint/TypeScript typecheck 통과로 보고하지 않음
- ASan/UBSan: engine 33+projection 11=44 통과. genuine clangd 23.1.0: engine/tracker/estimator 3 source, error_count=0 (`opt-in/clangd-*.log`, `clangd-estimator.log`)
- 활성 파일의 실제 Worker: 기본 raw 60 frame에서 C++ OFF=0 readback 확인. 실험 ON 400 frame GT 통과, 같은 입력·설정의 native와 timestamp/pose-valid 동일·행렬 원소 최대차 `1.52e-11` (`opt-in/active-native-worker-comparison.json`)
- WASM smoke: 실제 binding·기본 OFF·enable/reset/configure lifecycle 통과 (`opt-in/wasm-smoke.json`); loader/public API 검증이며 tracking/정확도 증명과 별개

## 변경 파일

- 입력·배선: `web/js/{app,imu,vio-worker}.js`, `wasm/vio_bindings.cpp`
- engine·추적·map: `include/vio_engine.h`, `include/frontend/feature_tracker.h`, `src/vio_engine.cpp`, `src/frontend/feature_tracker.cpp`, `src/backend/estimator.cpp`
- 검증 도구: `scripts/dev/{generate-synthetic-vio.py,synthetic-browser.mjs,wasm-smoke.mjs,README.md}`, `tests/audit_dataset_replay.cpp`
- 회귀: `tests/{test_engine_contracts.cpp,test_projection_trial_contracts.cpp,test_synthetic_vio.py}`, `tests/browser/{video-frame,synthetic-input,imu-pairing,rotation-mode}-contracts.test.mjs`
- 문서: `docs/fixes/2026-10-10-{rotation-plan,mobile-rotation}.md`

## 재현 명령

- repo root에서 실행; 출력 폴더가 이미 있으면 새 이름 사용. 원본 dataset은 읽기 전용; parallel jobs=2. room4도 동일한 rig의 `config/tum_vi_room1.yaml` 사용

```bash
MOBILE_SLAM_JOBS=2 bash scripts/dev/check.sh native
bash scripts/dev/check.sh quick
bash scripts/dev/check.sh browser-contracts
# 기본 OFF full TUM; benchmark solver profile 기본 OFF, seed=0, cvThreads=1
build/dev-native/native_replay assets/datasets/tum/dataset-room1_512_16 config/tum_vi_room1.yaml build/rotation-recheck-room1 2821 bracket 0 1 10 10 0 0 0 2821 legacy 0
build/dev-native/native_replay build/refactor-data/tum/dataset-room4_512_16 config/tum_vi_room1.yaml build/rotation-recheck-room4 2228 bracket 0 1 10 10 0 0 0 2228 legacy 0
python3 scripts/evaluation/audit_metrics.py build/rotation-recheck-room1/trajectory-camera.txt assets/datasets/tum/dataset-room1_512_16/mav0/mocap0/data.csv --gt-format euroc --estimate-frame camera --config config/tum_vi_room1.yaml --expected-frames 2821 --max-dt .01 --output build/rotation-recheck-room1-evaluation.json
# 선택 실험 ON: native 마지막 PROFILE 뒤 1; 0 또는 생략이면 OFF
python3 scripts/dev/generate-synthetic-vio.py --output build/rotation-recheck2203 --seed 2203 --scenario fast-rotation --portrait
build/dev-native/native_replay build/rotation-recheck2203 build/rotation-recheck2203/config.yaml build/rotation-recheck2203-native-on 400 bracket 0 1 10 10 0 0 0 400 yaml 1
node scripts/dev/synthetic-browser.mjs --input build/rotation-recheck2203/input.json --loader build/motion-wasm-20261010/vio_engine.js --output build/rotation-recheck2203-browser-on --mode live-deterministic --sensor generic --frames 400 --rotation-rescue on
python3 scripts/dev/evaluate-synthetic-vio.py --dataset build/rotation-recheck2203 --results build/rotation-recheck2203-browser-on/report.json --estimate-frame camera --output build/rotation-recheck2203-browser-evaluation.json
```

## 로컬 산출물·배포·남은 한계

- 검증 candidate loader: `build/motion-wasm-20261010/vio_engine.js` (embedded WASM). 로컬 활성 `web/vio_engine.js` 승격·실제 API smoke 통과. SHA256 `1066a22d647fd4d170eb3dba5bd66ba0b49463143a6dc5ee8cccbad07b68e0fc`
- native/WASM source manifest SHA256 `c47bf0a43a14f06a19fd311527464f31359b8f6612be9865ef8b7eeeca14490b` 일치·현재 소스 전체 hash 확인. manifest: `opt-in/{native,wasm}-manifest.json`, 종합 검증 `opt-in/verification.json`
- 교체 전 JS/manifest 백업: `/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/build/refactor-deploy/1791630846740468081`. 활성 manifest `web/vio_engine.manifest.json`, 로컬 승격 기록 `opt-in/deployment.json`
- HTTPS7002는 정지 상태 유지. 로컬 artifact promotion은 서버 시작·HTTPS 실제 served hash·전화 실행 증명이 아님
- 실제 Z Flip 6의 새 image/IMU 기록, 실측 calibration, exposure/sensor timing, independent GT, 지속 처리/열 시험 없음. 실제 Lost/발산·원거리 특징점·phone 속도 해결 판정 보류

## Git 반영 범위 (2026-10-11)

- 요청: 이번 작업 커밋 → 로컬 `main`에 `--ff-only` 병합 → `main` 체크아웃
- 작업 브랜치: `codex/mobile-rotation-input-20261011`; 소스·회귀·문서와 WASM·provenance를 분리 커밋
- 컴파일된 source hash·artifact byte가 동일한 상태에서 manifest의 source commit을 갱신. 커밋 때문에 재빌드했다고 주장하지 않음
- 원격 push·서버 시작은 반영 범위에서 제외. 실험 ON의 TUM 회귀·실기기 미검증 판정 유지
- 소스·회귀 커밋: `009006db49bdf627f2c40c1c795343de3693cdc5`. 활성 WASM manifest의 source commit도 동일하며, 검증 source hash와 loader byte는 변경 없음
