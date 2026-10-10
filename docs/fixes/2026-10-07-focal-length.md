# 2026-10-07 초점 거리 범위와 FOV 오판 수정

## 수정 계획

- 증상: Z Flip 6 Chrome 초기화 직후 pose 발산 보고; 현재 calibration provenance의 `ultrawide detect 100°` 경로 확인
- 확인된 코드 결함: `focusDistance.min < 0.05`를 100° FOV 근거로 사용; 0 또는 작은 값에서 명시한 `focalLengthFactor`까지 덮어쓰기
- 수정 파일: `web/js/app.js`의 `estimateFocalLength()`만 변경; `tests/browser/focal-length-contracts.test.mjs` 추가
- 삭제 경로: 초점 거리 capability에서 렌즈 종류·FOV를 추론하는 분기; URL fx·track FOV·zoom·명시 factor·기존 default 경로 보존
- 회귀 기준: `focusDistance.min=0`, `0.001`, `0.02`, fixed-focus 범위에서도 100° 자동 선택 없음; 명시 factor·URL fx·track FOV 기존 우선순위 유지
- 검증 명령: `node --test tests/browser/focal-length-contracts.test.mjs`; `node --test tests/browser/mobile-source-contracts.test.mjs tests/browser/imu-clock-contracts.test.mjs`; `bash scripts/dev/check.sh syntax`
- 범위 제한: 새로운 기기 calibration·렌즈 선택·회전·IMU·C++/WASM 변경 없음; 실제 렌즈 FOV·실기기 발산 해결·독립 GT 정확도 미확인

## 근거

- [W3C MediaStream Image Capture](https://www.w3.org/TR/image-capture/#focus-distance): `focusDistance`는 렌즈의 초점 거리 설정; 보통 최적 초점까지의 거리(m); FOV 또는 렌즈 종류 표식 없음
- [Chromium Camera2 upstream 구현](https://github.com/chromium/chromium/blob/main/media/capture/video/android/java/src/org/chromium/media/VideoCaptureCamera2.java): Android diopter를 웹의 m 단위로 변환; fixed-focus 또는 누락된 최소 초점 거리에서 0 가능. 현재 사용 중인 Chrome 바이너리의 해당 구현과 일치 여부는 미확인
- 기존 함수 직접 실행: 480×640에서 `focusDistance.min=0` → `fx=268.51188197672957`, `ultrawide detect 100°`; `focalLengthFactor=0.8`도 동일 값
- 기존 default69° 경로: 480×640에서 `fx=465.60288917518244`; 100° 경로는 그 값의 약57.67%. 두 값 모두 실측 기기 calibration 근거 제외

## 검증 결과

- 수정 전 신규 회귀: 5개 중3통과·2실패, exit1; `focusDistance.min=0`에서 100° 경로 선택 및 명시 factor0.8의 예상512 대신268.51188197672957 반환 확인
- 수정 후 `node --test tests/browser/focal-length-contracts.test.mjs tests/browser/mobile-source-contracts.test.mjs tests/browser/imu-clock-contracts.test.mjs`: 28개 통과·실패0·skip0·TODO0, exit0; 신규 focal 회귀5개 포함
- `bash scripts/dev/check.sh syntax`: exit0; web JS·개발 MJS parse, shell syntax·Python compile 범위. 별도 ESLint·TypeScript typecheck 구성 없음
- 단순화: 초점 거리 capability로 FOV를 결정하던 분기 삭제; 명시 factor 복구, URL fx·track FOV 우선순위 회귀 확인
- 기본69° 선택 시 기존 `heuristic_unverified` provenance 유지; 기기별 실측 FOV·보정 profile 근거 없음
- 실제 휴대전화: 위 결과는 함수·JS 계약 검증. 수정 후 활성 페이지 재시작·실제 렌즈 확인·발산 재검증·독립 GT 정확도는 별도 수집 필요
