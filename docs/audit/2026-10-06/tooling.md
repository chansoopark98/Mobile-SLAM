# 개발·테스트 환경 진단 — 2026-10-06

- 실행 범위: 프로젝트 개발 도구/검증 스크립트/MCP/skill 신규 설정
- 보존: 기존 C++/web/WASM 구현, 기존 local changes, dataset, 전역 Codex 설정/credentials
- 설정 위치: `.codex/config.toml`, `.agents/skills/mobile-slam-validation/`, `scripts/dev/`, `CMakePresets.json`
- 재실행: [개발 도구 runbook](../../../scripts/dev/README.md)

## 상태 및 근거

| 항목 | 상태 | 실제 확인 | 제한 |
|---|---|---|---|
| 개발 전용 npm package | VERIFIED | 고정 4개 패키지 + lock, `npm install --ignore-scripts`, `npm ls` | 제품 의존성과 분리 |
| Playwright MCP | VERIFIED | initialize → 38 tools/list → localhost fixture navigate/snapshot/close | 모바일 SLAM/실기기 검증과 구분 |
| Context7 MCP | VERIFIED | initialize → 2 tools/list → Emscripten resolve-library-id/query-docs | 이번 실행 keyless 성공; 향후 rate limit 가능 |
| Codex project MCP 설정 | VERIFIED | `codex mcp get`가 두 stdio 서버/cwd/timeout 인식 | 현재 desktop 세션 tool catalog reload는 미확인 |
| genuine clangd 23.1.0 | VERIFIED | 공식 바이너리 SHA256 일치, LSP initialize + `vio_engine.cpp` symbol 16개 + compiler error 0 | 단일 native compilation DB 대상; WASM cross analysis 미확인 |
| 프로젝트 validation skill | VERIFIED | repository `.agents/skills` 작성, bundled `quick_validate.py` 성공, 현재 Codex available skills 목록에서 발견 | 실제 validation 실행 범위는 각 test 결과 참조 |
| 환경 doctor | VERIFIED | 실제 실행 파일/package/Python module 버전, hot-file fingerprint, strict exit 0 | 전체 build provenance/SLAM 품질 증거 아님 |
| quick gate | VERIFIED | web/개발 JS syntax + Python compile + shell syntax + 독립 평가 초기8/8, rank 회귀 추가 후9/9 | ESLint/TypeScript typecheck 아님 |
| browser contracts/exporter | VERIFIED | 정상4개 + 재현된 TODO5개, 독립 exporter analytic3개 통과 | Node exit 0은 TODO5개 acceptance 통과 아님 |
| canonical dataset inventory | VERIFIED | `/mnt/backup/SLAM`: 최대2깊이, root16/child8 presence 확인; `MOBILE_SLAM_DATA_ROOT` override | 내용 읽기/hash/recursive scan/download/write 없음 |
| dev-only 모바일 관측 recorder | CONFIGURED / contracts VERIFIED | trusted phone TLS/localhost mock 명령 연결, bounded/stop/copy/pose-delta 계약4개 통과 | 실제 phone/독립 GT/완전 sensor replay 미확인; mock+download는 browser lane 소유 |
| CMake configure/build/test preset | VERIFIED | wrapper configure → test targets fresh build 183단계 성공 → CTest 실행 | regression gate exit 8: 5/6 suites 통과, parity 3개 fixture/cwd 실패 보존 |
| WASM 별도 빌드 helper | CONFIGURED | `EMSDK` subprocess 환경 활성화 + `build/dev-wasm` 출력 경로 설정 | 이 helper의 완전한 빌드 실행은 미확인; fresh WASM 검증은 browser lane 참조 |
| clang-tidy/clang-format/TypeScript/ast-grep | BLOCKED | 실제 실행 파일 미설치 | 통과 주장 없음; plain JS에 불필요한 TypeScript 의존성 추가 없음 |
| marketplace plugin 추가 설치 | 대체 완료 | 공식 local stdio MCP + repository skill 사용 | 기존 plugin 환경 유지 |

## 확인된 버전

| 구성 | 버전/경로 |
|---|---|
| GCC C++ | 11.4.0 (`/usr/bin/g++`) |
| CMake / CTest | 3.22.1 |
| Ninja | 1.10.1 |
| Node.js / npm | 22.21.1 / 11.10.1 |
| Python | 3.13.5 (`/home/park-ubuntu/miniconda3/bin/python3`) |
| Chrome | 144.0.7559.96 (`/usr/bin/google-chrome`) |
| Emscripten | 5.0.0 (`/home/park-ubuntu/emsdk/upstream/emscripten/emcc`); 기본 PATH 미활성 |
| clangd | 23.1.0, official LLVM commit `ea7d852a70e8bdfaf601d6626a760f9771b2c4b4` |
| native pkg-config | Eigen 3.4.0 / OpenCV 4.5.4 / yaml-cpp 0.7.0 |
| evaluator | NumPy 2.2.6 / SciPy 1.16.3 / PyYAML 6.0.3 |
| browser harness | Playwright 1.63.0 stable |
| MCP | `@playwright/mcp` 0.0.83 / `@upstash/context7-mcp` 4.1.1 / SDK 1.32.1 |

- Playwright MCP 내부 serverInfo: upstream 고정 `1.64.0-alpha-1790635538000`; 독립 browser harness의 stable Playwright 버전과 별도
- clangd archive SHA256: `e53b1a96196095faedb7642cf64964f7fb9ad4a0c1f00dd2c172a3d9dcbafdfd`
- C++ include 탐색 보정: `--query-driver=/usr/bin/c++,/usr/bin/g++`; 기본 autodetect는 설치된 GCC 11 대신 불완전한 GCC 12 header 경로 선택
- `clangd --check`는 compiler parse 후 `ExtractFunction` tweak self-check 4개를 오류로 집계; LSP documentSymbol + publishDiagnostics에서 compiler error 0으로 분리 검증
- `/usr/bin/sg`: Unix group 실행 명령; ast-grep 아님
- OMX `omx_code_intel`의 TypeScript availability/fallback: genuine C++ LSP로 해석 금지

## 명령 및 산출물

```bash
bash scripts/dev/check.sh doctor
bash scripts/dev/check.sh quick
bash scripts/dev/check.sh mcp
MOBILE_SLAM_COMPILE_DB=build/audit-native bash scripts/dev/check.sh clangd
python3 /home/park-ubuntu/.codex/skills/.system/skill-creator/scripts/quick_validate.py .agents/skills/mobile-slam-validation
cmake --list-presets
ctest --list-presets
```

- [MCP initialize/list/tool results](tooling-mcp-probe.json)
- [도구/의존성/fingerprint snapshot](tooling-doctor.json)
- [clangd LSP 실제 결과](tooling-clangd-probe.json)
- [estimator.cpp LSP 추가 결과](tooling-clangd-estimator.json): compiler error 0
- [quick regression 출력](tooling-quick.log)
- [추가 rank 회귀 후 quick9 결과](tooling-quick-followup.log)
- [canonical data root/별도 repo fixture presence](tooling-data-root.json)
- [browser contracts4/TODO5 + exporter3](tooling-browser-contracts.log)
- [mobile observation recorder 계약4](tooling-mobile-contracts.log)
- [native preset wrapper 전체 로그](tooling-native-preset.log): build 성공, CTest exit 8; 기존 fixture/cwd 실패 보존
- fresh native regression/sanitizer 결과: [native audit](native.md)
- web/WASM baseline: [browser audit](browser.md); 독립 fresh WASM build 및 synthetic Chrome replay 결과

## 환경 경계 및 후속 확인

- [공식 Codex 설정](https://developers.openai.com/codex/config-basic) 기준 repository `.codex/config.toml` 사용; 신뢰된 프로젝트에서 로드, global config 변경 없음
- [공식 MCP 설정](https://developers.openai.com/codex/mcp) 기준 stdio launcher/cwd/startup/tool timeout 구성; 현재 checkout 절대 경로 사용, 이동 시 경로 수정
- [공식 repository skills](https://developers.openai.com/codex/skills) 기준 `.agents/skills`에 검증 계약 저장
- [공식 Playwright MCP](https://github.com/microsoft/playwright-mcp) headless/isolated profile + 설치된 Chrome 사용; 개인 browsing profile 공유 없음
- [공식 Context7](https://github.com/upstash/context7) keyless 요청 직접 확인; 선택적 `CONTEXT7_API_KEY` 이름만 config에 전달, 비밀값 기록 없음
- [공식 clangd release](https://github.com/clangd/clangd/releases/tag/23.1.0) 바이너리/공식 asset digest 사용; 프로젝트 로컬 경로, 전역 설치 없음
- [CMake 3.22 presets](https://cmake.org/cmake/help/v3.22/manual/cmake-presets.7.html)과 호환되는 version 3 / bounded jobs=2 / CTest empty-suite 오류
- 다음 단계: fresh build와 실제 served JS/WASM 대응 확인, timestamp/coordinate/scale 계약 검증, 독립 GT ATE/RPE/rotation/coverage 측정, 실제 휴대전화 장시간 실행
- production-level 정확도/지속 FPS/열 성능: 이번 개발 도구 설정으로 입증 불가

## 추가 데이터·모바일 steering

- 사용자 제공 canonical read-only root: `/mnt/backup/SLAM`
- shallow presence: `TUM VI`(room1/room2/room3/room4/magistrale1/outdoors1/slides1 tar), `EuROC/machine_hall.zip`, `redwood`, `TartanAir`; 후자 두 디렉터리는 child8개에서 중단/truncated 표시
- repo 공식 복원 room1 fixture와 canonical 공유 원본 구분; doctor는 데이터 archive 내용·무결성·센서 품질 미평가
- 실제 모바일 position/orientation jitter: dev recorder에서 capture/IMU/result timestamp, pose translation/rotation jump, frame drop/lost/reset 수집 후 stationary/controlled-motion을 나눠 검증
- 별도 실행: `python3 scripts/dev/serve-mobile-audit.py --host 0.0.0.0 --port 8766 --cert /absolute/trusted-cert.pem --key /absolute/key.pem`; phone에서 `https://<phone-access-host>:8766/audit-mobile.html`
- JSON `mobile-slam-observation-v1`: browser local download, 기본60초/최대120초, channel10k/전체40k; 관측/limit/drop 메타데이터와 완전 replay/GT 구분
- smoothing 전 원시 추정기 오류와 pose age 확인; display smoothing만으로 정확도 개선 주장 금지

## Native subagent 모델 호환 관측

- 본 세션의 일부 role preset은 `gpt-5.4`를 선택하여 ChatGPT 계정에서 unsupported model 오류 반환
- 복구: native subagent의 default 역할·현재 모델 상속 + architect/reviewer/tester 등 명시 작업 지침으로 독립 병렬 실행 성공
- 프로젝트 AGENTS: 과거 모델명을 고정하지 않고 현재 설정 상속; 전역 모델/계정 설정 변경 없음
- 이 항목은 role preset 연결의 제약이며 SLAM algorithm/compiler 오류와 구분
