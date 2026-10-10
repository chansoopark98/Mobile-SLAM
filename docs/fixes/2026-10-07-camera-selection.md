# 2026-10-07 명시적 후면 카메라 선택

## 수정 계획

- 증상: Z Flip 6 Chrome의 기본 `environment` stream은 HAL Camera 2 사용; focal1.7mm·vendor 수평FOV103°. 기본69° heuristic과 차이
- 수정 범위: `web/js/camera.js`의 초기 stream 선택·zoom 설명, `tests/browser/camera-selection-contracts.test.mjs` 신규 회귀
- 선택 규칙: `?camera=N` → 먼저 enumeration → `videoinput`의 정확한 Chrome label `camera N, facing back`과 일치하는 단일 장치에 직접 요청; 후면 카메라 번호와 enumeration slot 구분
- 기본 규칙: parameter 없음 → 기존 `environment` 선택 경로 유지; Camera 0을 보편적인 메인렌즈로 가정하지 않기
- 하드웨어 근거: 실제 HAL Camera 2의 conflicting device 목록에 Camera 0 포함; 후면 stream 동시 open 전제 제거
- bootstrap 규칙: label이 모두 비어 있을 때만 임시 permission stream → 재enumeration → 해당 bootstrap 중지 → 선택 카메라 open; bootstrap은 초기화 단계에서 새로 연 소유 stream만 정리
- 초기화 보호: 이미 소유한 stream이 있으면 새 요청 전에 오류; 기존 tracking stream·다른 stream 자동 중지 제외
- 실패 정리: 임시 bootstrap은 enumeration 성공/실패 모두 중지; 선택 stream의 device/facing 확인 실패는 해당 stream 즉시 중지
- 실패 기준: 잘못된 번호·누락·중복·전면 label, 결과 facing/device 불일치; 다른 장치로 조용히 fallback 금지
- profile 계약: 현재 schema에 lens binding 없음; `cameraProfile`과 명시 camera 선택의 기존 API 조합 허용, 호출자가 실제 선택 렌즈와 보정 profile 일치 확인. 기존 pixel domain·dimension·orientation 검사 및 unverified provenance 유지
- 보존: 영상 해상도·frame rate·rotation·K·distortion·IMU·C++/WASM·기존 데이터/서비스; deviceId는 요청에만 임시 사용, 별도 저장·로그 제외
- 검증: 신규 회귀 실패 확보 → 수정 → camera/focal/mobile/IMU 계약 및 `bash scripts/dev/check.sh syntax`, `git diff --check`

## 공식 API·검증 한계

- [W3C Media Capture and Streams](https://www.w3.org/TR/mediacapture-streams/): `enumerateDevices()`·`deviceId` exact constraint로 장치 선택; 장치 label과 가용성은 권한·browser 상태에 따라 변경 가능
- `zoom=1`은 선택된 장치의 배율 요청; 물리 메인렌즈 선택 증거 제외
- [W3C stabilization constraint 제안](https://github.com/w3c/mediacapture-image/issues/310): 현재 표준 Media Capture/Image Capture에 Android EIS 해제용 constraint 없음; 가짜 `videoStabilization` flag·전역 Chrome flag 추가 없음
- 실제 기기에서 `getSupportedConstraints()`·track capabilities·HAL metadata를 함께 확인; JS 요청 성공과 EIS OFF 확인 구분
- [Android video stabilization](https://developer.android.com/reference/android/hardware/camera2/CaptureResult#CONTROL_VIDEO_STABILIZATION_MODE): EIS는 영상 warp·crop 변경 가능. 렌즈 선택만으로 고정 camera–IMU 모델·실제 정확도 보장 불가
- lens 선택 이후 calibration provenance는 기존 `heuristic_unverified`; HAL active camera 번호·출력 해상도·초기화/pose/발산을 새 trial에서 별도 검증

## 결과

- 하드웨어 배타 사용 회귀: 변경 전11개 중2통과·9실패, exit1; 동시 camera open을 금지한 fixture에서 `fixture exclusive camera hardware busy` 재현
- 수정 후 `node --test tests/browser/camera-selection-contracts.test.mjs tests/browser/focal-length-contracts.test.mjs tests/browser/mobile-source-contracts.test.mjs tests/browser/imu-clock-contracts.test.mjs`: 39개 통과·실패0·skip0·TODO0, exit0; camera 선택 회귀11개 포함
- `bash scripts/dev/check.sh syntax`, 신규 test 파일 `node --check`, `git diff --check`: exit0; JS parse와 lint/typecheck 구분
- 실제 `Camera.initialize()` 실행 mock 검증: default 요청1회·enumeration 없음; visible label camera0는 요청1회로 직접 open; hidden label은 요청2회이며 bootstrap 중지 후 선택 camera open
- failure 검증: 누락/전면/중복 label·잘못된 query·enumeration 실패·선택 camera open 실패·잘못된 facing/device·비어 있는 video tracks; 각 소유 임시 stream 정리, 이미 존재하는 stream에 대한 초기화 거부 후 stream 보존 확인
- 단순화: `zoom=1`을 메인렌즈 선택으로 표현하던 설명/로그 수정; 기본 선택·calibration/profile API 변경 없음
- 실제 Z Flip 6: 수정 후 HAL active camera0 여부·영상 품질·EIS 상태·pose 발산은 별도 trial 필요. 신규 회귀는 실기기 tracking/정확도 증거 제외
