# 브라우저 계약 진단

- 실행: `node --test tests/browser/contracts.test.mjs`
- 제품 소스 직접 import/VM 실행; 생성 코드·dataset·runtime 변경 없음
- 일반 테스트: 기존 정상 동작의 회귀 기준
- `TODO` 테스트: 현재 재현 가능한 결함 계약; 테스트 실패를 숨긴 acceptance pass로 해석 금지
- TODO 해결 조건: 제품 수정 후 TODO 제거, 동일 계약의 정상 통과 확인
- 브라우저/WASM 실측: `npm --prefix scripts/dev run browser:baseline`
- 실제 TUM replay: `BROWSER_AUDIT_REPLAY_FRAMES=600 BROWSER_AUDIT_REPLAY_TIMEOUT_MS=180000 BROWSER_AUDIT_OUTPUT=build/audit-browser/tum-active npm --prefix scripts/dev run browser:baseline`
- fresh 소스 비교: `BROWSER_AUDIT_WASM=fresh`; isolated `build/audit-wasm/vio_engine.js` 사전 빌드 필요
- replay timeout: 기본60000ms; `BROWSER_AUDIT_REPLAY_TIMEOUT_MS=180000`으로 긴 처리 허용
- 16bit PNG의 browser8bit decode3표본: `node tests/browser/image-decode.mjs`; 출력 `build/audit-browser/image-decode/report.json`
- camera pose TUM export: `python3 tests/browser/export-trajectory.py build/audit-browser/tum-active/browser-baseline.json build/audit-browser/tum-active/camera.tum`
- 독립 정확도 scorer: `python3 scripts/evaluation/audit_metrics.py build/audit-browser/tum-active/camera.tum assets/datasets/tum/dataset-room1_512_16/mav0/mocap0/data.csv --estimate-frame camera --config config/tum_vi_room1.yaml --output build/audit-browser/tum-active/metrics.json`; coverage 계산 시 실제 JSON `replaySummary.count`를 `--expected-frames`로 추가
- export timestamp: audit에서 제출한 dataset camera timestamp; engine 자체 pose timestamp 부재로 측정 시각 추정 대용
- JSON 경로: 기본 `build/audit-browser/browser-baseline.json`; `BROWSER_AUDIT_OUTPUT`로 변경
- frame sample 수: `BROWSER_AUDIT_FRAMES=60` 기본; synthetic은 초기화·정확도 근거 제외
- 실제 device camera/IMU, iOS Safari, Android Chrome, 열화·background복귀·장시간 drift: 별도 실기기 검증 필요
