# 2026-10-07 실제 video callback 시각·frame 연계 수정

## 수정 계획

- 실제 Chrome154 관측: `video.currentTime=1335.548207`, callback `mediaTimeS=2314658.08417`; 두 값의 origin 차이
- 같은 관측의 DOM 시각: `captureTime=1337504.805ms`, `presentationTime=1337508.335ms`, callback 주변 `performance.now=1337516.710ms`
- 확인된 결함: `mediaTimeS == video.currentTime` 비교 실패로 callback metadata 무시; 실제 기록558개 frame 모두 `canvas_read_arrival` 선택
- 수정 범위: `web/js/app.js`의 video callback·processLoop frame ID/시각 선택·reset 필드만 변경; 신규 `tests/browser/video-frame-contracts.test.mjs`
- frame 기준: rVFC 사용 시 단조 증가 `presentedFrames`; 다른 media origin 사이의 equality 제거; raw `mediaTime`은 provenance로 보존
- 시각 기준: 0 이상·callback arrival보다1ms 이상 미래가 아닌 유효 DOM `captureTime` 우선, 이후 유효 `presentationTime`, 이후 callback arrival
- 연계 한계: rAF에서 읽은 픽셀과 최신 presented-frame callback의 best-effort 연계; 실제 exposure와 정확히 같은 frame이라는 보장 제외, `physicalExposureVerified:false` 유지
- fallback: rVFC API 미지원 시 기존 `video.currentTime` dedup·canvas arrival 경로
- reset/resume/rotation: cached timing·소비한 frame ID 초기화; 다음 callback 전 이전 metadata 재사용 제외
- 검증: 서로 다른 media origin, frame ID 중복·증가, 유효 과거 captureTime, 미래/음수 시각 fallback, reset, API 미지원 회귀 → syntax·관련 JS 계약
- 보존: K·gyro/body·renderer·camera 선택·solver parameter·WASM799·원본 기록

## 공식 근거

- [rVFC 정의·metadata](https://wicg.github.io/video-rvfc/#video-frame-callback-metadata): `presentedFrames`는 composition 제출 frame 수이며 단조 증가; `captureTime`, `presentationTime`은 DOMHighResTimeStamp, `mediaTime`은 media PTS초
- [rVFC 소개](https://wicg.github.io/video-rvfc/#intro): callback은 rAF 직전 실행, canvas copy와 화면 frame 연계는 best-effort; callback 지연/누락 가능
- 물리 노출 시각·EIS latency·완전한 camera/IMU 동기·독립 GT 정확도는 위 API 값만으로 확정 불가

## 검증 결과

- 수정 전 기존 inline callback와 실제 `processLoop()` 회귀: 9개 중1통과·8실패, exit1; 실제 값 fixture에서 예상 `1337.504805s` 대신 arrival `1337.516710s`, 반복 frame ID도 currentTime 변화마다 중복 처리
- 수정 후 `node --test tests/browser/video-frame-contracts.test.mjs tests/browser/camera-selection-contracts.test.mjs tests/browser/focal-length-contracts.test.mjs tests/browser/mobile-source-contracts.test.mjs tests/browser/imu-clock-contracts.test.mjs`: 48개 통과·실패0·skip0·TODO0, exit0; 신규 video frame 회귀9개 포함
- `bash scripts/dev/check.sh syntax`, 신규 test `node --check`, `git diff --check`: exit0
- 단순화: 다른 media origin 값의 equality 분기 삭제; supported callback 경로와 legacy 경로에 각각 명시적인 frame 기준 적용
- 검증 범위: 실제 App callback 등록 함수·processLoop를 실행하는 mock 계약; Chrome154의 수정 후 실제 필드 readback·새 동작 기록·발산 해결은 별도 확인 필요
- 정확도 한계: `video_capture_time`은 browser metadata provenance; `association=latest_presented_frame_callback`, `physicalExposureVerified=false`. 실제 physical exposure·센서 동기·독립 GT 정확도 보증 없음
