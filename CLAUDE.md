# CLAUDE.md — Mobile-SLAM (VINS-Mono 기반 Mobile VIO)

VINS-Mono를 기반으로 한 Visual-Inertial Odometry 엔진입니다. Native C++에서 검증한 뒤 WASM으로 빌드해 모바일 웹(Chrome)에서 실행합니다.
- 원칙: 원본 VINS-Mono 코드를 최대한 그대로 활용합니다.
- 실기기 기준: Galaxy Z Flip6 (SM-F741N, Chrome).

## 세션 시작 시 먼저 읽을 것

1. **`docs/STATUS.md`** — 현재 상태, 미커밋 WIP, 미해결 과제, 다음 단계. 작업 전 반드시 확인합니다.
2. `docs/README.md` — 문서 색인. 기준 문서와 대체된(stale) 문서를 구분합니다.
3. `scripts/dev/README.md` — 개발 도구와 runbook의 전체 명령.
4. 장기 목표와 gate 정의: `docs/refactoring-plan.md` (P0–P6, G0–G6).

작업 전에 `git status --short --branch`로 dirty tree를 확인하고, 내가 만들지 않은 변경은 보존합니다.

## 아키텍처 요약

- **Native**: `tiny_vins_mono` → `VIOSystem` (`src/vio_system.cpp`) → `MeasurementProcessor` → `VIOEngine`
- **Browser**: `web/js/app.js` → Camera/IMU → `vio-wrapper.js` → `vio-worker.js` → WASM `VIOEngine` (`wasm/vio_bindings.cpp`, embind, `SINGLE_FILE` embedded)
- 두 경로는 `VIOEngine` core를 공유하지만 입력, 스케줄링, 설정이 다릅니다. **한쪽에서 통과한 테스트를 다른 쪽 검증 결과로 보고하지 않습니다.**
- 주요 위치:
  - frontend: `src/frontend/` (KLT tracker, feature manager, initialization)
  - backend: `src/backend/` (Ceres sliding window, factors, marginalization)
  - 테스트: `tests/` (C++ GoogleTest, Python unittest), `tests/browser/*.test.mjs` (node --test), `tests/server/`
- 참조 소스 (gitignored, 읽기 전용):
  - `assets/references/VINS-Mono`
  - `assets/references/DPVO` (DPViewer)
  - `assets/references/AlvaAR` (ORB-SLAM WASM)

## 자주 쓰는 명령

전체 목록은 `scripts/dev/README.md`에 있습니다. 빌드 jobs는 **기본 2, 최대 4**입니다. `check.sh`와 `native-replay.sh`는 1–4 범위 밖의 값을 거부합니다.

```bash
bash scripts/dev/check.sh syntax             # node --check / bash -n / py_compile (lint·typecheck 아님)
bash scripts/dev/check.sh browser-contracts  # node --test tests/browser/*.test.mjs
bash scripts/dev/check.sh eval               # Python 평가 단위 테스트
bash scripts/dev/check.sh native             # preset native-dev 구성 + 빌드 + ctest (build/dev-native)
bash scripts/dev/check.sh wasm-build         # emsdk 환경 자동 source, build/dev-wasm (web/ 에는 쓰지 않음)
MOBILE_SLAM_COMPILE_DB=build/dev-native bash scripts/dev/check.sh clangd
python3 -m unittest discover -s tests -p 'test_*.py' -v
node --test tests/server/server.test.cjs
```

**Native 직접 빌드**: Ceres와 GTest는 FetchContent로 받으며, 캐시는 `build/_deps`에 있습니다.

```bash
cmake --preset native-dev \
  -DFETCHCONTENT_SOURCE_DIR_CERES-SOLVER="$PWD/build/_deps/ceres-solver-src" \
  -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$PWD/build/_deps/googletest-src"
cmake --build --preset native-tests --parallel 2
ctest --preset native-tests --parallel 2
```

**환경 주의**
- `emcc`는 PATH에 없습니다. 필요하면 `source ~/emsdk/emsdk_env.sh`를 실행하거나 `check.sh wasm-build`를 사용합니다.
- clang은 설치되어 있지 않습니다. clangd는 `scripts/dev/tools/clangd_23.1.0/`에 있습니다.
- `build/`는 80GB가 넘습니다. 날짜별 증거 디렉터리가 많으므로 `build/` 전체를 grep하거나 glob하지 않습니다.
- dataset 원본은 `/mnt/backup/SLAM`(NFS, **읽기 전용**)이며, `MOBILE_SLAM_DATA_ROOT`로 변경할 수 있습니다. 공식 fixture는 `assets/datasets/tum/dataset-room1_512_16`입니다. 실행 결과는 `build/` 아래에 저장합니다.

## 절대 규칙

### 1. Artifact와 배포

- `web/vio_engine.js`와 `web/vio_engine.manifest.json`을 직접 수정하지 않습니다.
  - 승격은 `scripts/dev/deploy-candidate.py`로만 합니다.
  - 이 스크립트는 사용자의 명시적 요청과, 새 lane 경로에 둔 authorization JSON(`--authorization`으로 지정)이 있어야 실행됩니다.
  - 기본 경로 `build/refactor-orchestration/deploy-authorized.json`은 과거 승인 기록이므로 덮어쓰지 않습니다.
  - 절차는 `.claude/skills/wasm-release/` skill을 따릅니다.
- 기존 WASM을 새 소스의 빌드라고 주장하지 않습니다. source/artifact hash(`build-provenance.py`)로 확인합니다.
- HTTPS 서버(`scripts/ops/serve.sh`, `web/server.js`)의 시작과 중지는 사용자가 합니다. 내가 시작하지 않은 `node server.js` 프로세스는 종료하지 않습니다.
- 실기기 측정은 readiness 점검을 마친 뒤 사용자가 "측정 시작"이라고 한 다음에만 시작합니다. 영상, IMU, MP4 저장은 각각 따로 허락받습니다.

### 2. 알고리즘 — 결함을 숨기는 수정 금지

- 임계값 완화, clamp, smoothing, pose 숨김, stale pose를 성공으로 집계, `INIT_DEPTH` 대입, 가짜 IMU sample, epsilon covariance, guard 우회, solver cap 축소, F gate 완화를 하지 않습니다.
- 결과를 본 뒤 tolerance를 바꾸지 않습니다.
- 다음 후보는 이미 기각되었으므로 재시도하지 않습니다. 근거는 `docs/STATUS.md` §5에 있습니다.
  - inverse-depth bound
  - log-depth manifold
  - unusable→prior 폐기
  - A2 geometry admission
  - H4 projective
  - observation policy D0/D2/D3 (`setExperimentalObservationPolicy(1|2)`)
  - gyro patch warp
  - capture Worker의 기본값 채택
- 다음 실험 옵션은 기본값으로 승격하지 않습니다. gate를 통과하고 사용자가 승인해야만 승격합니다.
  - `MOBILE_SLAM_EXPERIMENTAL_ANCHOR_DEPTH`
  - C++ 기본 `rotation_rescue`
  - `setExperimentalObservationPolicy(1|2)`
  - `?capture=worker`
- Backend 수치 패치 금지 (2026-03 교훈):
  - LLT를 SelfAdjointEigenSolver로 교체
  - inverse depth clamp
  - marginalization eps 변경
  - quaternion 강제 normalize
  - NaN을 zero residual로 처리
  - 이런 패치는 native 발산을 일으킵니다.
- Solver 하한: `solver_time` ≥ 0.04s, `num_iterations` ≥ 8입니다. 이보다 줄이면 bias가 수렴하지 않아 발산합니다.
- 최적화 이후 Ba/velocity를 clamp하지 않습니다. 최적화기 상태가 서로 맞지 않게 되어 발산을 가속합니다.
- IMU 단위: timestamp는 초, 가속도는 m/s², 각속도는 rad/s입니다. 영상과 같은 시간축을 쓰는지, frame 경계 샘플이 맞는지 확인합니다.
- 좌표계: camera, body, world를 구분합니다. extrinsic 방향, row/column-major, crop/회전 이후의 K를 확인합니다.

### 3. 증거와 보고

- 정확도:
  - 독립 GT를 쓰고, timestamp를 1:1로 매칭하고, body frame에서 SE(3) scale=1로 평가합니다.
  - translation과 rotation 각각의 ATE·RPE, coverage를 보고합니다.
  - Sim(3)는 scale 진단 용도로만 따로 보고합니다.
  - zero pose나 zero association은 통과 근거로 쓰지 않습니다.
- 성능:
  - 초기화 전과 후를 나눕니다.
  - capture, copy, queue, frontend, backend, render 단계를 나눕니다.
  - p50/p95/p99, 처리 frame 수와 drop frame 수, reset 수, pose age를 보고합니다.
- dataset/desktop replay나 합성 데이터 결과를 실기기 실시간 성능이나 production 정확도로 확대 해석하지 않습니다.
- 결과에는 `VERIFIED`(실행 관측), `CONFIGURED`(설정만 존재), `BLOCKED`/`UNVERIFIED`를 구분해 표시합니다. 실행한 명령, 증거 경로, 한계를 함께 적습니다.
- 유지할 gate:
  - TUM: baseline 대비 위치와 회전 각각 ≤1.05배, coverage 손실 ≤1%p.
  - churn gate와 기하 gate: `docs/STATUS.md` §4 참조.

### 4. Git

- 커밋 메시지는 intent를 먼저 쓰고, 필요하면 Lore trailer(`Tested:`, `Not-tested:`, `Constraint:`, `Directive:`)를 붙입니다. 마지막에 Co-Authored-By 줄을 넣습니다.
- 커밋과 push는 사용자가 요청할 때만 합니다. 로컬 `main`은 `origin/main`보다 앞서 있으며 push되지 않았습니다.

## Coding Conventions

C++17, Google Style:
- 변수/함수: `snake_case`
- 클래스: `PascalCase`
- 상수: `kCamelCase`
- 멤버: `trailing_`

기존 타입, utility, 패턴을 먼저 재사용합니다. 제품 runtime 의존성을 추가하기 전에는 사용자에게 확인합니다. 개발 전용 의존성은 `scripts/dev/`에 둡니다.

## Agent Roles

| Role | 담당 | 금지 |
|------|------|------|
| PM | 작업 관리, 할당, Phase Gate 판단 | 문서 업데이트 |
| Research | `assets/references/` 분석, 파라미터 제안, 온라인 서치 | 코드 수정 (읽기 전용) |
| Lead Developer | SLAM Engine, System, 알고리즘 구현·개발 총괄 | |
| SLAM Frontend Developer | Frontend 구현, 빌드 검증 | 테스트 |
| SLAM Backend Developer | Backend 구현, 빌드 검증 | 테스트 |
| Tester | QA 결과를 Developer agent에 피드백 | |

- 통신 형식: 완료는 "Task #N 완료. [결과]", 실패는 "Task #N 실패. [에러]. [파일:라인]"
- 병렬 작업 시 파일 소유권을 명시합니다. 하위 에이전트는 다른 작업의 변경을 되돌리지 않습니다.

## Permissions

- 자동 허용: 프로젝트 내 파일 편집, 빌드/테스트 실행, Git 로컬 작업, 온라인 서치
- 확인 필요: 파일 삭제, 네트워크 요청, push, WASM 승격, 서버 시작·중지, 실기기 측정

## WASM 구성 참고

| Component | Config |
|-----------|--------|
| Eigen | SIMD 활성화 (`EIGEN_DONT_VECTORIZE` 제거). 경로 `/usr/include/eigen3` 하드코딩 |
| OpenCV | `wasm/libs/opencv` (`build_opencv.sh`), `CV_ENABLE_INTRINSICS=ON`, `-msimd128`, `intrin_wasm.hpp` 패치 |
| Ceres | `wasm/libs/ceres` (`build_ceres.sh`), `-O3 -msimd128`, NO_THREADS, MINIGLOG |
| VIO WASM | `-O3 -msimd128`, `SINGLE_FILE=1`, 출력은 `build/dev-wasm` 또는 `build/<lane>-wasm` |
| Memory | INITIAL=256MB, MAX=512MB, STACK=8MB |

`wasm/build/`와 `build_wasm/`은 과거 빌드 디렉터리입니다. 예전 `cp wasm/build/vio_engine.js web/` 절차는 폐기되었습니다.

## 다른 에이전트 설정과의 관계

- `AGENTS.md`, `.codex/`, `.agents/skills/`, `.omx/`은 Codex가 사용합니다.
- Claude Code가 사용하는 위치:
  - 이 파일
  - `.claude/settings.json`
  - `.claude/skills/`
  - `.mcp.json` (Playwright MCP)
- 규칙을 바꾸면 이 파일과 `AGENTS.md`를 함께 갱신합니다.
