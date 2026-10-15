---
name: mobile-slam-validation
description: Mobile-SLAM의 native C++, WASM, 브라우저 실행과 개발 도구를 재현 가능한 증거로 검증하는 절차. 회귀 확인, profiling, 정확도 실험, "테스트 통과/정확도 개선" 같은 주장을 하기 전에 사용한다. smoke 실행과 실기기 tracking·궤적 정확도를 구분한다.
---

# Mobile-SLAM 검증

Codex용 원본은 `.agents/skills/mobile-slam-validation/SKILL.md`에 있다. 이 문서는 Claude Code용 버전이며, 원본에서 오래된 경로와 개수를 바로잡았다.

## 0. 시작 전 기록

- 저장소 루트에서 시작한다.
- 다음을 기록한다:
  - `git rev-parse HEAD`
  - `git status --short`
  - 컴파일러와 빌드 플래그
  - 서빙되는 JS/WASM의 hash: `sha256sum web/vio_engine.js`와 `web/vio_engine.manifest.json`의 artifact 값
- 도구 버전과 hot-file fingerprint는 `python3 scripts/dev/doctor.py --output build/dev-tools/doctor.json`으로 기록한다.
  - 이 스크립트는 build provenance를 증명하지 않는다.
- 빌드 출력은 격리된 디렉터리를 쓴다: `build/dev-native`, `build/dev-wasm`, 또는 `build/<lane>-YYYYMMDD/`.
- 기존 binary, 증거 디렉터리, dataset은 보존한다.
- jobs는 2로 한다. `MOBILE_SLAM_JOBS`는 1–4 범위에서만 쓴다.
- dataset:
  - `/mnt/backup/SLAM`은 읽기 전용이다. `MOBILE_SLAM_DATA_ROOT`로 바꿀 수 있다.
  - 공유 dataset을 기본으로 scan, hash, download하지 않는다.
  - 공식 fixture는 `assets/datasets/tum/dataset-room1_512_16`이다.

## 1. 주장별로 필요한 증거

| 주장 | 명령 | 주의 |
|---|---|---|
| Native 회귀 | `bash scripts/dev/check.sh native` | preset `native-tests`이며 test target은 16개다. CTest가 발견한 suite의 결과, GoogleTest 개수, 실패한 assertion을 모두 기록한다. binary 누락, 빈 suite, TODO/skip은 실패로 본다. 원래부터 실패하던 테스트는 baseline 증거일 뿐 통과 gate가 아니다 |
| JS 문법 | `bash scripts/dev/check.sh syntax` | ESLint나 TypeScript typecheck가 아니다. 그렇게 보고하지 않는다 |
| Browser 계약 | `bash scripts/dev/check.sh browser-contracts` | 일반 통과, TODO 수, exporter 통과 수를 따로 적는다. 현재 기준 349 pass (`docs/fixes/2026-10-14-low-feature-results.md`) |
| Python 평가 | `bash scripts/dev/check.sh eval`, `python3 -m unittest discover -s tests -p 'test_*.py'` | |
| Server | `node --test tests/server/server.test.cjs` | |
| WASM 빌드 | `bash scripts/dev/check.sh wasm-build` → `node scripts/dev/wasm-smoke.mjs build/dev-wasm/vio_engine.js build/dev-wasm/wasm-smoke.json` | link 성공이 브라우저가 그 빌드를 서빙했다는 증거는 아니다. 배포는 `wasm-release` skill을 따른다 |
| 브라우저 실행 | `npm --prefix scripts/dev run browser:baseline` | 데이터셋과 frame 제한, capability, network 오류, source hash를 확인한다. Chromium emulation은 실기기 camera/IMU 정확도, WebGPU, 열 성능을 증명하지 않는다 |
| C++ 언어 도구 | `MOBILE_SLAM_COMPILE_DB=build/dev-native bash scripts/dev/check.sh clangd` | 실제 clangd(`scripts/dev/tools/clangd_23.1.0`)로 확인한다 |
| MCP | `bash scripts/dev/check.sh mcp` | initialize, tools/list, 실제 tool 결과까지 확인한다. 설정이 존재하는 것만으로는 부족하다 |
| Native replay | `bash scripts/dev/native-replay.sh ...` (인자는 `scripts/dev/README.md` 참조) | 기본 빌드 디렉터리는 `build/refactor-native`(오래됨)이다. `MOBILE_SLAM_NATIVE_BUILD=build/dev-native`를 권장한다. provenance JSON을 함께 보관한다 |

## 2. SLAM 성능과 정확도

- **시간 정보**
  - capture/IMU timestamp, 순서, clock domain 가정을 적는다.
  - 처리 frame 수와 drop frame 수, queue 대기, capture-to-pose 지연은 engine 처리 시간과 분리해 보고한다.
- **분포와 메타데이터**
  - p50/p95/p99를 보고한다.
  - 초기화 결과, reset·Lost 횟수, 메모리, 기기/브라우저/빌드 메타데이터를 적는다.
  - dataset 길이와 seed를 명시한다.
- **궤적 정확도**
  - 다음을 모두 만족해야 정확도로 인정한다:
    - GT timestamp와 동기화
    - SE(3), scale=1, body frame
    - warm-up 구간과 Lost 구간의 coverage 명시
  - 보고 항목: ATE, RPE, 회전 오차.
  - Sim(3)는 scale 진단 용도로 따로 보고한다.
  - 다음은 정확도 증거가 아니다: 그려진 경로, 보기에 그럴듯함, 프로세스 exit 0, zero pose, zero association.
- **Gate**
  - `docs/STATUS.md` §4의 gate를 적용한다. TUM은 위치와 회전 각각 baseline 대비 ≤1.05배다.
  - 결과를 본 뒤 tolerance를 바꾸지 않는다.
- **해석 범위**
  - desktop replay나 합성 데이터 결과를 production 수준이라고 부르지 않는다.
- **실기기 pose jitter**
  - smoothing을 제안하기 전에 raw IMU, capture, result 시각과 translation/rotation jump를 기록한다.
  - 정지 상태와 통제된 동작 상태는 따로 검증한다.
- **실기기 관측 도구**
  - `web/audit-mobile.html`과 `scripts/dev/serve-mobile-audit.py`를 쓴다.
  - 결과 JSON(`mobile-slam-observation-v1`)은 범위가 제한된 로컬 다운로드다. 완전한 sensor replay도, 독립 GT도 아니다.
  - 측정은 사용자가 "측정 시작"이라고 한 다음에만 시작한다.
- **외부 도구**
  - 낯선 라이브러리 API는 Context7 MCP로 찾는다. 찾은 내용은 실제 사용 버전의 소스와 대조한다.
  - 브라우저 진단에는 project Playwright MCP(`.mcp.json`, 격리 profile)를 쓴다.

## 3. 보고 형식

- 각 항목을 `VERIFIED`(실행 관측), `CONFIGURED`(설정만 존재), `BLOCKED`/`UNVERIFIED`(실패 또는 증거 없음)로 구분한다.
- 실행한 명령, 증거 경로, 변경한 파일, 알려진 한계를 적는다.
- 실패하던 baseline assertion은 범위가 정해진 수정이 독립적인 기대값을 가질 때까지 그대로 보존한다.
- 결과를 문서로 남겼다면 `docs/STATUS.md`를 갱신한다.
