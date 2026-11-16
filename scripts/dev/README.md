# Development and validation tools

개발 의존성과 실행 결과를 제품 코드·입력 데이터에서 분리합니다. 명령은 저장소 루트에서 실행합니다.

## Environment

- Native: C++17 compiler, CMake 3.21+, Ninja, Eigen, OpenCV, yaml-cpp, pkg-config. Pangolin은 viewer를 켤 때만 필요합니다.
- WASM: Emscripten 5.0.0, 로컬에서 빌드한 OpenCV 4.5.5·Ceres 2.2.0. 공식 source cache는 ignored `build/_deps/`에 두며 `OPENCV_SOURCE_DIR`·`CERES_SOURCE_DIR`로 기존 소스를 지정할 수 있습니다.
- 검증: Node 22+, Python 3, NumPy·SciPy·Pillow. 일부 영상 도구에는 Python OpenCV가 필요합니다.
- browser harness는 `scripts/dev/package-lock.json`의 고정 의존성을 사용합니다.

```bash
npm ci --prefix scripts/dev
```

빌드·캐시·로그는 ignored `build/` 등에 저장합니다. 입력 데이터는 읽기 전용으로 취급하고 실행마다 새 output 경로를 사용합니다.

## Routine checks

| 명령 | 확인 범위 |
|---|---|
| `bash scripts/dev/check.sh syntax` | web JS·개발 script 구문 |
| `bash scripts/dev/check.sh browser-contracts` | browser·Worker·sensor·transport·exporter 계약 |
| `bash scripts/dev/check.sh eval` | 궤적 평가·합성 입력·recording 도구 |
| `bash scripts/dev/check.sh native` | 기존 VIO headless build와 native CTest |
| `bash scripts/dev/check.sh refact-native` | slam_refact 엔진 구성·빌드·`ctest -L '^unit$'`(출력 `build/dev-refact-native`), 알려진 실패 지문 대조 후 원장 기록 |
| `bash scripts/dev/check.sh refact-web` | `slam_refact/web-chunk/tests`·`web-smoke` node 시험, 통과·skip 수를 원장에 기록 |
| `bash scripts/dev/check.sh refact-tools` | `check_vendor.py --data`, `check_std_algos.py` |
| `bash scripts/dev/check.sh wasm-build` | 별도 VIO WASM build; `web/`는 수정하지 않음 |
| `node --test tests/server/server.test.cjs` | read-only server·TLS·경로 허용 목록 |
| `python3 -m unittest discover -s tests -p 'test_*.py'` | top-level Python 회귀 |

- 기본 compile jobs는 2, `MOBILE_SLAM_JOBS=1..4`로 변경할 수 있습니다.
- `refact-native`는 의존성·증거 경로를 환경변수 `SLAM_REFACT_CERES_SOURCE_DIR`, `SLAM_REFACT_GTEST_SOURCE_DIR`, `SLAM_REFACT_EVIDENCE_ROOT`에서만 받습니다(없으면 엔진 CMake 기본값). 앞의 둘을 지정하면 `FETCHCONTENT_FULLY_DISCONNECTED=ON`이 켜집니다. 빌드 디렉터리는 `SLAM_REFACT_NATIVE_BUILD_DIR`로 바꿀 수 있습니다.
- `scripts/dev/ci-ledger.py`는 JUnit을 `slam_refact/engine/tests/known-failures.json`(이름과 메시지 지문)과 대조해 실패 집합이 같을 때만 0으로 끝나며, 알려진 실패가 통과로 바뀐 경우도 불일치로 봅니다. 결과 한 줄을 append-only `build/ci-ledger/ledger.jsonl`에 남깁니다.
- `check.sh all`은 doctor, quick, native, browser-contracts, refact-tools, refact-web, refact-native를 순서대로 실행합니다.
- `MOBILE_SLAM_TIMEOUT`은 build/test subprocess timeout(초), `MOBILE_SLAM_PYTHON`은 Python 실행 파일입니다.
- 원본 데이터·추가 모델이 필요한 시험과 self-contained unit을 구분합니다. skip·missing input·실패를 PASS에 포함하지 않습니다.
- JavaScript syntax 검사는 ESLint나 TypeScript typecheck가 아닙니다.
- 별도 SLAM 엔진의 build·unit·WASM 명령은 [slam_refact 안내](../../slam_refact/README.md)를 확인합니다.

- `refact-native`의 ctest 동시 실행 수는 `MOBILE_SLAM_TEST_JOBS`(기본 16, 1..32)입니다. compile jobs와 별개입니다. 같은 g2 build의 unit 877건에서 `-j2` 369 s → `-j16` 116 s, 실패·skip 집합 동일(알려진 실패 1건, skip 2건). 하한은 가장 긴 test(약 94 s)입니다.

## slam_refact RS-0 parity gate (fast / full)

```bash
SLAM_REFACT_RS0_NEW_NATIVE=<build>/harness/slam_refact_replay bash scripts/dev/check.sh rs0-fast
SLAM_REFACT_RS0_NEW_NATIVE=<build>/harness/slam_refact_replay SLAM_REFACT_RS0_NEW_WASM=<wasm build>/wasm \
  bash scripts/dev/check.sh rs0-full
python3 scripts/dev/rs0-gate.py --tier fast --new-native <bin> --out build/<lane>/rs0-fast --plan-only
```

| tier | 용도 | 내용 |
|---|---|---|
| `rs0-full` | merge gate | RS-0a exact-input 24건, RS-0b native parity 9건(frozen or1pair vs new), RS-0c WASM parity 5건, RS-0d OFF 결정성 6건 × 2회. 항목·비교 field·제외 항목(trace `srStageMs`, meta `wallSeconds`·`contractVersion`, WASM `wallMs`·`pose.stageMs`)은 `build/audit-run1-20261115/rs0/scripts/run-rs0.sh`와 같음 |
| `rs0-fast` | 일상 반복 | 같은 항목을 native bundle 앞 300 input frame, WASM 120 frame으로 자름(frozen과 new가 같은 잘린 bundle 재생). merge gate가 아님. WASM은 `SLAM_REFACT_RS0_NEW_WASM`이 있을 때만 |

- 항목 목록은 `scripts/dev/rs0-gate.json`, 결과는 `build/rs0-gate/<mode>-<time>/SUMMARY.json`(구간별 verdict, 단계별 시간, 실행·재사용 replay 수, 평균 사용 core 수)입니다. 기록 한 줄이 `build/rs0-cache/ledger.jsonl`에 추가됩니다.
- 모든 replay(native·WASM)는 독립 process로 한 pool에서 긴 것부터 실행합니다. 기본 `--jobs`는 `min(24, nproc×3/4)`(i9-13900K: 8 P-core + 16 E-core에서 16 → 24로 cold full 11m19s → 9m53s), `SLAM_REFACT_RS0_JOBS`로 바꿉니다. 이것은 replay 실행 수이며 build jobs(1–4) 규칙과 별개입니다. 엔진 solver는 thread 1개이고 `iterative_bundle_adjustment.cc`의 `max_solver_time_in_seconds=0.2`(1점 문제)가 있으므로 core 수보다 많은 jobs는 쓰지 않습니다.
- replay 결과는 `build/rs0-cache/runs/<key>`에 저장합니다. key는 binary sha256(+ `ldd` shared library sha256), `--bundle` 인자 문자열(meta.json에 기록됨), bundle 파일과 `frames.tsv`가 가리키는 image·pyramid 파일의 내용 hash, frame 상한, 반복 tag(RS-0d a/b), 환경 scrub 규칙입니다. 같은 key의 성공한 run이 있으면 재실행하지 않습니다. frozen 기준 결과는 한 번만 계산되고, 바뀌지 않은 new binary의 parity·exact·WASM replay도 다시 돌지 않습니다. 실패한 run은 cache하지 않습니다(`build/rs0-cache/failed/`).
- RS-0d(OFF 결정성)의 rep a·b는 cache와 무관하게 gate를 돌릴 때마다 두 번 새로 실행합니다(`run-rs0.sh`와 같음). `--fresh`는 new binary·WASM replay도 모두 다시 실행합니다. 다시 실행한 run이 cache 항목을 대체하고, 이전 항목은 `build/rs0-cache/superseded/`로 옮깁니다. gate 출력 디렉터리에는 run 파일의 hardlink 복사본이 들어가므로 cache가 바뀌어도 이전 증거는 그대로입니다.
- frozen 기준은 `defaults.frozen_pins`(native·WASM js·wasm sha256)로 고정합니다. 다르면 실행을 거부하고, `--allow-unpinned-frozen`이면 실행하되 `merge_gate: false`입니다. new가 frozen과 같은 sha256이면 거부합니다. `merge_gate: true`는 full tier, 네 구간 모두, pin 일치일 때만입니다.
- 엔진에는 wall-clock 상한이 있는 solve가 있습니다(원본 그대로인 vendored `bundle.cc` pose estimation 1.0 s, mapTracker 단계; `iterative_bundle_adjustment.cc` 0.2 s, mapBuilder 단계). process가 그보다 오래 멈추면(메모리 부족·swap, 과부하) 결과가 바뀔 수 있습니다. 2026-11-16 14:10 전역 OOM(다른 작업의 compile 중, swap 0) 동안 실행한 full run은 mapTracker 최대 8.1 s 정지와 함께 FAIL했고, 부하 없는 run은 모든 단계 0.45 s 미만이었습니다. 그래서 단계 시간이 그 상한에 닿은 frame이 있는 replay는 `stalled_replays`로 보고하고 cache하지 않습니다(`build/rs0-cache/unstable/`). verdict는 바꾸지 않습니다. 그런 FAIL은 부하 없는 상태에서 다시 실행한 뒤 판단합니다. replay는 `MemAvailable`이 `--min-free-gb`(기본 4) 이상일 때만 시작합니다.
- 병렬 실행이 결과를 바꾸지 않음: 같은 binary로 16 jobs·24 jobs cold full 두 번의 trace·worldpoints·exact·WASM 출력이 순차 실행 증거(`build/audit-run1-20261115/rs0`)와 `wallSeconds` 외 동일했습니다. accuracy OFF에서 시간 상한이 있는 solver는 `iterative_bundle_adjustment.cc`(1점 문제, 0.2 s)뿐이고 `local_bundle_adjustment`는 accuracy mode에서만 실행됩니다.
- replay 환경에서 `SLAM_REFACT_*`, `SR_*`, `TRKPROBE_*` 변수를 모두 제거합니다(accuracy mode OFF).
- 끝에 quarantine symlink(`defaults.quarantine_links`)에서 시작 marker보다 새 파일을 찾고, 하나라도 있으면 FAIL입니다.
- 이 checkout에 `build/`가 없으면(worktree) `SLAM_REFACT_RS0_ROOT`로 data root를 지정합니다.

## Reproducible input and replay

```bash
python3 scripts/dev/generate-synthetic-vio.py --output build/synthetic-example --seed 101 --scenario clean
python3 scripts/dev/pc-entry-check.py --dataset build/synthetic-example \
  --pc build/dev-native/tiny_vins_mono --native build/dev-native/native_replay --output build/synthetic-example-pc
node scripts/dev/synthetic-browser.mjs --input build/synthetic-example/input.json \
  --loader build/dev-wasm/vio_engine.js --output build/synthetic-example-browser \
  --mode live-paced-native-raf --sensor generic --frames 400
python3 scripts/dev/evaluate-synthetic-vio.py --dataset build/synthetic-example \
  --results build/synthetic-example-browser/report.json --estimate-frame camera \
  --output build/synthetic-example-evaluation.json
```

- `raw`: 기록된 입력을 직접 Worker에 재생. `live-paced-native-raf`: 실제 desktop rAF와 mock camera·sensor 사용.
- 영상·IMU·calibration·frame association·실행 설정과 artifact hash를 함께 기록합니다.
- ATE/RPE는 body frame, 일대일 timestamp association, SE(3) scale=1을 기본으로 사용합니다. Sim(3)는 scale 진단으로 별도 표시합니다.
- 처리 frame 수, initialized/pose coverage, reset, stale pose, capture-to-pose latency를 분리합니다. 누락 frame을 평가 분모에서 숨기지 않습니다.

## Artifacts and browser checks

```bash
python3 scripts/dev/build-provenance.py --build build/dev-wasm \
  --artifact build/dev-wasm/vio_engine.js --output build/dev-wasm/vio_engine.manifest.json
python3 slam_refact/engine/tools/check_vendor.py --data
python3 slam_refact/engine/tools/check_std_algos.py
node --test slam_refact/web-smoke/*.test.mjs
```

- provenance는 실제 source·compile flags·dependency·artifact를 함께 확인합니다. 링크 성공만으로 브라우저가 그 build를 사용했다고 판단하지 않습니다.
- 공개 source와 포함된 실행 bundle은 구분합니다. 공개 artifact metadata는 실제 hash·크기를 기록하며, 소스 재빌드나 새 배포의 증거를 대신하지 않습니다.
- XR tree 무결성 정보는 `config/runtime-integrity/xr-tree.json`을 사용합니다.
- UI·측정 smoke는 준비된 replay fixture를 필요로 합니다. HTTPS·camera/IMU 허용 조건과 실행한 artifact hash를 결과에 남깁니다.
- local self-signed certificate·desktop emulation·합성 센서 결과를 실제 모바일 추적·기기 GPU·현장 정확도 수락으로 확대하지 않습니다.

- 서버의 trusted TLS 통합 시험은 로컬 인증서 fixture가 있을 때만 실행합니다. `MOBILE_SLAM_TEST_TLS_DIR`로 fixture 디렉터리를 지정할 수 있으며, 미제공 시 해당 시험은 SKIP입니다. 개인 key/cert는 커밋하지 않습니다.
