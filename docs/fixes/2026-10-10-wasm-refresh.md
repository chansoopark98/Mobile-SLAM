# WASM 최신화·서버 종료·로컬 main 통합

- 날짜: 2026-10-10, Asia/Seoul
- 요청: 기존 서버 종료, 최신 소스 WASM 빌드, 기존 변경 커밋과 로컬 `main` 병합·체크아웃
- 실행 증거: `build/refresh-20261010/`
- 소스 커밋: `9624998fc8472267dcbcab7bed39d2e93bcd8e83`; 기존 소스·테스트·문서 84개 보존
- WASM 출력: `build/refresh-wasm-20261010/vio_engine.js`; 기존 빌드·원본 dataset·전화 기록·TLS 키 유지

## 서버·활성 파일

- `bash scripts/ops/serve.sh stop`: HTTPS7002 PID `1636898` 정상 종료
- `bash scripts/ops/serve.sh status`: `stopped`; 7002 listener 없음. 검증용 임시 localhost 서버도 각 harness 종료 시 닫힘
- 별도 빌드 → candidate browser smoke → provenance → `deploy-candidate.py` 원자적 승격 → 활성 파일 browser smoke 순서로 확인
- 활성 파일: `web/vio_engine.js`, `web/vio_engine.manifest.json`
- 형식: ES module loader에 WASM 내장(`SINGLE_FILE=1`); 실제 Worker도 `/vio_engine.js` import
- 최신 loader SHA256: `7b37399960ff4397344d27a0e0ebf2f4f47acb51448b4c9b49d586be3ae5b0d8`
- 재빌드 결과는 작업 시작 시 loader와 byte 동일. 기존 manifest는 `f480f9f9ada9c108b293ae973f4b66e6468085db0461b101072dbc1c33ad1cba`를 가리켜 불일치했으며 이번에 최신 소스·의존성·컴파일 명령·artifact hash로 갱신
- `source_files` 전체와 실제 소스 SHA256 일치, loader·artifact·deployment SHA256 일치 확인
- 교체 전 파일 백업: `build/refactor-deploy/1791624186504806242/`
- 승격 권한 기록: `build/refresh-20261010/deploy-authorized.json`; 이번 사용자 요청과 candidate hash 기록
- 서버 종료 상태이므로 새 HTTPS 응답·실기기 접속 검증은 수행하지 않음

## 검증

- `bash scripts/dev/check.sh syntax`: JS·Python·shell syntax 통과. 프로젝트에 별도 TypeScript typecheck·ESLint 설정 없음
- `bash scripts/dev/check.sh eval`: Python 27개 통과
- `bash scripts/dev/check.sh browser-contracts`: Node 196개·Python exporter 3개 통과, skip/TODO 0
- native: 최신 `build/mobile-web-native`의 `cmake --build ... --parallel 2` 결과 `no work to do`; CTest 13/13·GoogleTest 146개 통과
- 오래된 `build/dev-native`의 중복 전체 rebuild는 중단하고 최신 native build를 재확인해 사용; 중단 로그 `native.log`, 실제 통과 로그 `native-current-build.log`·`native-current-tests.log` 구분
- `clangd-probe.py --compile-db build/dev-native`: `src/vio_engine.cpp` error_count=0
- WASM: Emscripten 별도 configure 및 `vio_engine --parallel 2` 30단계 빌드 통과. GCC include-directory 선택에 관한 clang 비차단 경고 1건 기록
- `wasm-smoke.mjs`: candidate와 활성 loader 모두 load/configure/API/reset/diagnostic 계약 통과
- `browser-baseline.mjs`: candidate 실제 Worker 및 `/`, `/test-tumvi.html` 로딩 성공
- TUM-VI bounded replay: 요청150·실제151 frame, 초기화45 frame·유한 pose106개·nonfinite0; 종료 조건과 in-flight로 1 frame 초과하는 기존 harness 동작 유지
- 별도 60-frame 무특징 synthetic smoke는 전 frame 처리만 확인; 초기화·pose0이며 정확도 합격 근거 제외
- 시작 시 변경86개와 비교: 이번 갱신 대상 manifest 외 기존 내용 hash 유지
- `git diff --check` 통과; 원본 dataset·영상·전화 기록 바이너리·TLS 키는 커밋 대상 제외

## 재현 명령

```bash
source /home/park-ubuntu/emsdk/emsdk_env.sh
emcmake cmake -S wasm -B build/refresh-wasm-20261010 -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
cmake --build build/refresh-wasm-20261010 --target vio_engine --parallel 2
node scripts/dev/wasm-smoke.mjs build/refresh-wasm-20261010/vio_engine.js build/refresh-20261010/wasm-smoke.json
python3 scripts/dev/build-provenance.py --build build/refresh-wasm-20261010 --artifact build/refresh-wasm-20261010/vio_engine.js --output build/refresh-20261010/wasm-manifest.json
cmake --build build/mobile-web-native --parallel 2
ctest --test-dir build/mobile-web-native --parallel 2 --output-on-failure --no-tests=error
```

- 로컬 작업 브랜치: `codex/refresh-wasm-20261010`; 소스와 artifact·검증 기록을 분리 커밋한 뒤 `main`에 `--ff-only` 병합
- 최종 checkout: `main`; 원격 push는 요청 범위에 없어 수행하지 않음
- 검증 한계: 이번 bounded desktop 검사는 실제 모바일 발산 해결·현장 GT 정확도·발열·장시간 성능·production 합격 근거가 아님
