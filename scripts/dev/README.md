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
| `bash scripts/dev/check.sh wasm-build` | 별도 VIO WASM build; `web/`는 수정하지 않음 |
| `node --test tests/server/server.test.cjs` | read-only server·TLS·경로 허용 목록 |
| `python3 -m unittest discover -s tests -p 'test_*.py'` | top-level Python 회귀 |

- 기본 compile jobs는 2, `MOBILE_SLAM_JOBS=1..4`로 변경할 수 있습니다.
- `MOBILE_SLAM_TIMEOUT`은 build/test subprocess timeout(초), `MOBILE_SLAM_PYTHON`은 Python 실행 파일입니다.
- 원본 데이터·추가 모델이 필요한 시험과 self-contained unit을 구분합니다. skip·missing input·실패를 PASS에 포함하지 않습니다.
- JavaScript syntax 검사는 ESLint나 TypeScript typecheck가 아닙니다.
- 별도 SLAM 엔진의 build·unit·WASM 명령은 [slam_refact 안내](../../slam_refact/README.md)를 확인합니다.

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
