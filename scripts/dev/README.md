# Mobile-SLAM 개발·검증 도구

- 범위: 개발 전용 npm 패키지, 로컬 MCP, native C++ LSP, 재현 가능한 검증 명령
- 제품 의존성: 기존 `web/package.json` 유지
- 산출물: `build/dev-tools/`, `build/dev-native/`, `build/dev-wasm/`; WASM 빌드 후 `web/` 자동 덮어쓰기 없음
- 기준: [2026-10-06 도구 진단](../../docs/audit/2026-10-06/tooling.md), [전체 진단](../../docs/audit/2026-10-06/)

## 데이터 경로

- canonical 데이터: `/mnt/backup/SLAM`; 읽기 전용 사용, 외부 원본 변경/삭제/재다운로드 없음
- 다른 경로: `MOBILE_SLAM_DATA_ROOT=/path/to/datasets`; doctor가 해당 경로의 root 최대16개 + 각 디렉터리 child 최대8개 목록만 확인
- doctor 데이터 검사: 최대2깊이, 데이터 내용 읽기/hash/recursive scan/download 없음; tool/source fingerprint와 구분
- 데이터 경로 접근 실패는 JSON `BLOCKED` 기록; doctor `--strict`는 필수 개발 도구/package gate이며 dataset 품질·가용성 gate와 별도
- repo 공식 복원 fixture: `assets/datasets/tum/dataset-room1_512_16`; canonical 공유 원본과 별도 provenance
- 기존 parity test/browser TUM 페이지의 repo 경로 고정은 현재 유지; 공유 데이터를 자동 symlink/copy하지 않음
- 공식 room1 복구 fallback: `python3 scripts/dev/fetch-tum-room1.py --evidence build/dev-tools/dataset-provenance.json`; 기존 target/staging 보존, 공식 archive MD5 확인 + SHA256 기록 + 안전한 staging extraction. canonical root 확인 후 해당 fixture가 필요한 경우에만 실행

## 설치

```bash
npm ci --prefix scripts/dev --ignore-scripts --no-fund
python3 scripts/dev/install-clangd.py
python3 scripts/dev/doctor.py --strict --output build/dev-tools/doctor.json
```

- 평가용 Python 환경 복제: `python3 -m venv scripts/dev/.venv`, `scripts/dev/.venv/bin/python -m pip install -r scripts/dev/requirements-evaluation.txt`; 검사 시 `MOBILE_SLAM_PYTHON=scripts/dev/.venv/bin/python`
- Node.js 22 이상, Python 3.11 이상, CMake 3.21 이상, Ninja, GCC C++17
- native 라이브러리: Eigen3, OpenCV, yaml-cpp, Pangolin; CMake가 고정된 Ceres 2.2.0 / GoogleTest 1.14.0 확보
- Emscripten: 기존 SDK 사용; 기본 `$HOME/emsdk`, 다른 경로는 `EMSDK=/path/to/emsdk`
- Chrome: 기존 시스템 Chrome 사용; 다른 실행 파일은 `MOBILE_SLAM_BROWSER=/path/to/chrome`
- clangd: 공식 Linux x86_64 23.1.0 ZIP + SHA256 확인, `scripts/dev/tools/clangd_23.1.0/` 격리 설치; 다른 OS는 공식 대응 바이너리 준비 후 `MOBILE_SLAM_CLANGD` 지정
- 고정 npm: Playwright 1.63.0, Playwright MCP 0.0.83, Context7 MCP 4.1.1, MCP SDK 1.32.1
- Playwright MCP 0.0.83의 내부 Playwright: upstream 고정 `1.64.0-alpha-1790635538000`; 브라우저 baseline harness는 별도 stable 1.63.0 사용

## 검증 명령

| 명령 | 검증 범위 | 한계 |
|---|---|---|
| `bash scripts/dev/check.sh doctor` | 실제 실행 파일 버전, package pin, hot-file SHA256 | 빌드 신선도/정확도 보증 없음 |
| `bash scripts/dev/check.sh syntax` | 기존 web JS + 개발 MJS parse, Python compile | ESLint/TypeScript typecheck 아님 |
| `bash scripts/dev/check.sh eval` | 독립 analytic fixture로 ATE/RPE/frame/scale/rank 평가 9개 회귀 | 실데이터 정확도와 구분 |
| `bash scripts/dev/check.sh quick` | syntax + evaluator regression | native/WASM/browser는 별도 |
| `bash scripts/dev/check.sh browser-contracts` | strict browser contracts + transport + exporter analytic | mandatory TODO/skip는 gate 실패; 실기기와 구분 |
| `bash scripts/dev/check.sh mobile-contracts` | dev mobile recorder bounded/copy/pose-delta 계약 | 실제 전화 camera/IMU/추정 정확도와 구분 |
| `bash scripts/dev/check.sh native` | headless configure → 현재9개 suite target build → discovered CTest | empty/missing suite·mandatory skip·실패는 gate 실패 |
| `bash scripts/dev/check.sh wasm-build` | Emsdk subprocess 활성화 → 별도 WASM configure/build | `web/`로 미배포, 실기기 성능 미확인 |
| `bash scripts/dev/check.sh clangd` | C++ LSP initialize/symbol/diagnostics | 컴파일 DB 필수, 단일 source 검사 |
| `bash scripts/dev/check.sh mcp` | MCP initialize/tools/list + 브라우저 fixture + 문서 조회 | Codex 현재 세션 도구 reload와 구분 |
| `npm --prefix scripts/dev run browser:baseline` | browser 소유 harness의 실행/산출물 | 실제 휴대전화 camera/IMU와 구분 |
| `bash scripts/dev/check.sh all` | doctor + quick + native | WASM/MCP/browser는 별도 실행 |

```bash
# CMake presets 직접 사용
cmake --preset native-dev
cmake --build --preset native-tests
ctest --preset native-tests

# 기존 fresh baseline DB로 genuine C++ LSP 확인
MOBILE_SLAM_COMPILE_DB=build/audit-native bash scripts/dev/check.sh clangd

# 검사 파일 선택
python3 scripts/dev/clangd-probe.py --compile-db build/audit-native --file src/backend/estimator.cpp
```

- 기본 병렬 jobs=2; `MOBILE_SLAM_JOBS=1..8`
- configure/build/test subprocess 기본 timeout=900초; `MOBILE_SLAM_TIMEOUT`으로 조정
- 각 CTest 기본 timeout=180초; 모든 실패 결과 확인, empty CTest는 오류
- dependency source cache 재사용 예시: `cmake --preset native-dev -DFETCHCONTENT_SOURCE_DIR_CERES-SOLVER="$PWD/build/_deps/ceres-solver-src" -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST="$PWD/build/_deps/googletest-src"`
- 최초 설정과 기존 test 실패 원인은 날짜별 audit 문서/로그 확인

## 실기기 pose 흔들림 기록

- 장면 구분: 정지 구간, 제어된 천천한 이동/회전, 시작 위치 복귀; 장면/장치/browser/카메라 해상도/calibration/화면 방향과 기록시간 함께 보존
- 관측 필드: IMU event/sensor/callback 시각과 clock origin, video presentation/callback/capture-call 메타데이터, frame accept/drop, submitted-result roundtrip, pose 수신시각, translation/rotation jump, initialization/lost/reset, 오류
- timing provenance: 물리 exposure/capture 시각·독립 engine pose 시각·queue duration은 없거나 proxy일 수 있음. 누락/출처를 JSON에 명시; Worker stage instrumentation 없이 roundtrip을 queue/processing으로 분리 금지
- 기본 portrait480×640에서 landscape crop 없음; crop-K 위험은 crop 활성화/다른 profile 사용 시 조건부 확인
- smoothing 적용 전 기록 우선; 표시 jitter 감소와 실제 estimator 정확도/pose-age 개선 구분
- 정지 구간 jitter는 self-consistency 근거; 외부 GT 없으면 ATE/RPE 정확도 판단 제외

```bash
# 실기기에서 접근 가능한 HTTPS host + 해당 host의 trusted TLS cert 사용
python3 scripts/dev/serve-mobile-audit.py --host 0.0.0.0 --port 8766 \
  --cert /absolute/trusted-cert.pem --key /absolute/key.pem

# localhost mock 검증용 임시 인증서; 실제 phone trusted TLS와 별도
python3 scripts/dev/serve-mobile-audit.py --generate-dev-cert --host 127.0.0.1 --port 8766
```

- 전화 페이지: `https://<phone-access-host>:8766/audit-mobile.html`; 실제 카메라/IMU 권한 허용 후 recorder 사용
- localhost mock: `https://127.0.0.1:8766/audit-mobile.html`; browser mock 결과는 실제 sensor/phone 성능과 구분
- 관측 시간: 기본60초/최대120초; channel 최대10,000 samples, 전체40,000 samples. 제한 초과·stop 이후 drop 수를 export에서 확인
- Stop: sensor 중지, worker dispose, embedded app unload; JSON은 기기 browser 다운로드, schema `mobile-slam-observation-v1`
- raw image/device ID/track label 제외; 완전한 sensor replay 데이터와 구분
- 기존 `index.html?v=11` 관측 prototype 사용; 기본 제품 페이지/SLAM 추정 알고리즘 변경 없이 별도 audit 페이지 사용
- 기존 `sendBeacon` 차단 및 server POST405; 서버로 관측 로그 자동 업로드 없음
- 임시 인증서: `build/dev-mobile-audit/certs`, 7일/localhost SAN; phone 신뢰 인증서로 해석 금지

## 실제 HTTPS 7002 수동 운영

```bash
# 검증된 paired candidate 승격/부모 start flag 이후
bash scripts/ops/serve.sh start
bash scripts/ops/serve.sh status

# 소유 PID/명령/start ticks 검증 후 종료; 타 서비스에는 신호 없음
bash scripts/ops/serve.sh stop

# strict hostname/SNI + 기본 시스템 trust, localhost TCP 확인
curl -q --noproxy '*' --cacert /etc/ssl/certs/ca-certificates.crt \
  --resolve dev.serdic.com:7002:127.0.0.1 --fail --head https://dev.serdic.com:7002/
```

- 접속: `https://dev.serdic.com:7002/`, `/test-tumvi.html?dataset=room1|room4`, `/audit-mobile.html`; 실제 listener `0.0.0.0:7002`
- 기존 leaf + ordered intermediate2 + private key 사용; root 전송/자동 self-signed fallback 없음. SAN/keymatch/validity/system trust startup 검증
- TLS 파일 내용·ownership 보존, key600/cert644/keys directory750. 인증서 교체 후 보호 mode 확인·수동 restart
- PID record: `build/server/service.json`; exact repo/uid/exe/cwd/argv/start ticks 확인. 충돌/foreign/stale 기록은 자동 종료·덮어쓰기 없이 실패
- stale/dead 소유 record는 `stop`으로 archive 후 `start`; 포트/host 변경도 명시 `stop` 후 실행
- 로그: `logs/https/<UTC>-7002-<instance>.log`, 세션별 owner-only 새 파일, 1MiB 뒤 discard/drain. 기존 로그 truncate 없음; remote `/log`405
- health: `/__health__`의 active loader startup hash·`wasmMode`·server/source manifest hash·TLS metadata·PID 확인; 전체 NAS·private path/credentials 비공개
- room1 local fixture, room4 local stage만 공개. 선택 `MOBILE_SLAM_ROOM4_ROOT=/abs/checkout/local-room4/mav0`; symlink로 checkout 밖을 가리키면403
- 현재 candidate는 JS에 WASM을 embed한 SINGLE_FILE; flag `single_file_embedded_wasm:true`, `wasm_sha256:null`, 검증된 `js_sha256` 사용. 외부 `.wasm` 불필요, 기존 파일은 보존하되 direct/alias403·health active:false
- 향후 external paired mode는 `single_file_embedded_wasm:false` + 실제 JS/WASM 각각 hash 확인
- 시작 flag: `build/refactor-orchestration/server-start-authorized.json`의 authorized·active artifact hash·mode 필요. 실제 public7002는 fresh candidate 준비 후 실행
- 검증: `node --test tests/server/server.test.cjs`, `python3 -m unittest discover -s tests/server -p 'test_*.py' -v`
- localhost/LAN-host strict TLS 성공과 외부 internet/실제 phone 접근·센서 품질을 구분
- 현재 실행 기록(2026-10-07 01:48:26 KST): PID `3002812`, instance `c98f10df76144a3c80311b09d1e0d5cf`, deployment `refactor-final-20261007`, embedded loader `6,009,632 bytes` / SHA256 `9973d58eca6f95fcf5fcaf2d553d02bc2215f419e4ba1fc1cc94ae417eb3014d`; source manifest `b936982e5fea0c8ad0fb77e3e8b311770998763ff77ef2ef4bb082945866674b`
- 세션 로그: `logs/https/20261006T164826765565Z-7002-c98f10df76144a3c80311b09d1e0d5cf.log`; exact-owned PID2449945 종료 → 기존 helper atomic backup/promotion → 새 후보 시작1회. 이전 후보·session 로그·TLS 보존, 현재 상태는 `status`로 재확인. 상세 근거: [HTTPS/Ops 결과](../../docs/refactor/2026-10-06/server-lane.md)
- 같은 host의 기본 hostname은 기존 `/etc/hosts` development override로 `127.0.0.1` 연결. public 경로 근거는 `dig @1.1.1.1`의 A `183.98.179.103` 조회 후 해당 주소로 `--resolve` strict SNI 연결; 둘을 구분, ops에서 DNS/hosts 변경 없음

## MCP / skills / plugin

- `.codex/config.toml`: `mobile_slam_playwright`, `mobile_slam_context7`; 이 checkout의 절대 경로 사용, 이동 시 `cwd`/launcher 경로 변경
- [공식 Codex 설정](https://developers.openai.com/codex/config-basic): 신뢰된 프로젝트에서 project config 로드; 사용자 global config/credentials 수정 없음
- [공식 Codex MCP](https://developers.openai.com/codex/mcp): stdio launcher + startup/tool timeout 설정
- [공식 Playwright MCP](https://github.com/microsoft/playwright-mcp): headless + isolated profile, 시스템 Chrome, devtools 기능
- [공식 Context7](https://github.com/upstash/context7): keyless 실제 조회 확인; API key 선택 사항, 이미 보유한 키는 `CONTEXT7_API_KEY` 환경변수로만 전달
- `codex mcp get mobile_slam_playwright`, `codex mcp get mobile_slam_context7`: 설정 확인; 실제 서버 연결은 probe로 별도 확인
- 추가된 서버가 현재 세션 목록에 없으면 새 Codex 세션에서 project config 로드 확인
- `.agents/skills/mobile-slam-validation/SKILL.md`: [공식 repository skill 경로](https://developers.openai.com/codex/skills); 회귀/성능/정확도 증거 계약
- 외부 marketplace plugin 설치 대신 공식 stdio MCP + 프로젝트 스킬 직접 구성; 같은 기능의 중복 plugin 없음
- 사용 가능한 기존 Codex/OMX 도구는 유지. `omx_code_intel`의 TypeScript fallback 또는 `/usr/bin/sg`를 genuine C++ LSP/static analysis로 해석 금지
- `clang-tidy`, `clang-format`, `tsc`, `ast-grep`는 현재 미설치; 해당 lint/typecheck 통과 주장 금지


## Refactor build·replay·staging

- Shared source 목록: `cmake/SLAMCoreSources.cmake`; native viewer 기본 OFF, Pangolin 없는 headless `tiny_vins_mono --headless`/`native_replay`.
- Native: `cmake --preset native-dev && cmake --build --preset native-tests && ctest --preset native-tests`.
- 이번 candidate: `build/refactor-native`, `build/refactor-wasm`; 새 artifact 생성만으로 배포 완료 처리 금지.
- Native replay 정규 CMake target: `bash scripts/dev/native-replay.sh DATASET CONFIG OUTPUT MAX_FRAMES bracket PNP CV_THREADS ITERATIONS SOLVER_TIME DUMP_INPUTS`.
- Browser profile parity: tracking 21/3/min20/edge0 → LK criteria20/0.03, F threshold1, PnP 기본 OFF/freq3; summary에 실제 적용값 기록.
- Fixed pilot: `ITERATIONS=10 SOLVER_TIME=10`, OpenCV threads1/seed0; replay timing은 capture latency와 구분.
- Profiling 재빌드 생략: 검증된 build만 `MOBILE_SLAM_SKIP_BUILD=1`; 실행 중 source/build 변경 없이 순차 측정.
- NAS room4 staging: `python3 scripts/dev/stage-tum-room4.py`; `/mnt/backup/SLAM` read-only, `build/refactor-data/tum`만 local 공개 가능. Convenience DSO image symlinks는 추출 제외·manifest 기록.
- Native/Worker parity: `scripts/dev/compare-replay.py --freeze-pilot NATIVE_JSON WORKER_JSON`을 3쌍 + `--output CONTRACT`로 사전 independent caps 내부 tolerance 동결; room4 이후 재동결 금지. 비교는 `--native --browser --expected-frames --contract --output`.
- Source/flags/dependency/artifact manifest: `python3 scripts/dev/build-provenance.py --build BUILD --artifact ARTIFACT --output MANIFEST`.
- 승격: `python3 scripts/dev/deploy-candidate.py --candidate JS --manifest MANIFEST`; parent의 `build/refactor-orchestration/deploy-authorized.json`에서 `authorized: true`, `candidate_sha256` 일치 필수. Single-file embedded WASM만 원자적 교체, 이전 JS/manifest는 `build/refactor-deploy/` 보존.
- ASan/UBSan은 별도 fresh build 전체 core/dependency instrumentation; 오래된 unsanitized core objects를 link하여 통과 판정 금지.
- 단위 테스트·calibrated dataset·HTTPS browser 검증으로 physical phone stability/full SLAM/production claim 금지.

- Candidate isolated browser API smoke: `node scripts/dev/wasm-smoke.mjs build/refactor-wasm/vio_engine.js`; loader/mandatory bindings/configure/epoch/reset/seed/thread만 검증, tracking·accuracy와 분리.
- Native raw input oracle: replay 마지막 `DUMP_INPUTS=1` → `input-oracle.bin`; `python3 scripts/dev/hash-inputs.py --raw FILE --dataset DATASET --config CONFIG --output MANIFEST`로 native OpenCV decoded gray와 exact Float64 LE IMU packets SHA 기록. Hash I/O는 pilot 검증용, paced latency 실행에서는 비활성.
