# 선택형 수치 검증 solver profile

- 목적: production 기본 Ceres 양수 종료 기준 유지; full native/browser 수치 비교에만 `max10_zero_positive_tolerances` 명시 선택
- 소유 파일: `web/js/vio-worker.js`, `web/js/test-tumvi-app.js`, `tests/browser/refactor-validation.mjs`, 신규 `tests/browser/benchmark-profile-contracts.test.mjs`
- Worker: configure 성공 뒤 `benchmark_zero_positive_tolerances === true`만 활성화; 나머지는 setter 존재 시 명시 false. true 요청의 setter/getter 부재·실제 getter 불일치는 configure 실패
- replay: `benchmarkZeroTolerances=1` 선택; requested bool과 engine 실제 getter를 export에 분리 기록. 실제 옵션 수치는 bounded diagnostic capture 근거 사용; ordinary row의 tolerance 추정 금지
- runner: explicit numerical profile 선택 필수 기록. full room1 3쌍·room4만 true / 최대10iteration / 비구속10s cap; paced/loss는 false / TUM 기본0.1s / 최대10iteration /150feature. main live mock 설정 유지
- 비교: 입력 byte·state/reason/solver·epoch·init·coverage·사전 오차 cap 유지; 실제 iteration·termination 변경/정규화 금지, 이전 실패 기록·contract 보존
- 검증: 실제 Worker dispatch fixture의 기본 false·true→기본 재configure·API 부재·getter mismatch, 실제 replay start configure 요청·query/export. 제품 estimator 정확도와 구분
- 명령: `node --test tests/browser/benchmark-profile-contracts.test.mjs`; `node --check web/js/vio-worker.js`; `node --check web/js/test-tumvi-app.js`; `node --check tests/browser/refactor-validation.mjs`; 소유 파일 `git diff --check`
- 실행 gate: source 동결·fresh ABI native/WASM/SAN·bounded option trace·새 artifact/실제 HTTPS identity·parent performance ready 이후에만 full replay/latency
- 제한: TUM desktop0.1s budget은 실제 phone60ms/thermal 검증 제외; mode의 max10은 최대값이며 실제 완료 iteration 보증 제외

## Source 검증 근거

- valid RED: 신규7개 profile 계약 실패, `build/refactor-evidence/benchmark-profile/contracts-red.log` 보존
- GREEN: 신규8개 + 기존 Worker/transport/mobile70개 = **78 pass /0 fail /0 skip /0 TODO**, `contracts-all-green.log`
- syntax: Worker·replay·runner·신규 test4개 parse 통과; runner serialization/SO(3)/percentile3개 self-test 통과. ESLint·TypeScript typecheck 구성 없음
- 동결: `build/refactor-evidence/benchmark-profile/source-frozen.json`, SHA256 `324acf6766719caf00dc18db5f8d83b56637b3271bd7e5de5b2e3f5415f3edbe`; preflight 선택 mode 명시 보완 포함
- 실제 engine 옵션 f/g/p=0 확인: fresh native/WASM bounded diagnostic lane 후속 gate; 위 JS fixture는 product estimator/수치 parity 근거 제외
- full replay·paced·loss·actual HTTPS mock: parent performance ready 이전 미실행
