# Z Flip 6 실기기 발산 재현·수정 계획

- 대상: `SM-F741N`, Android16/API36, Chrome154.0.8037.126, 현재 무선 ADB `192.168.0.126:43371`
- baseline: `build/dev-android/divergence-observation.json` 398/398 포즈·44~159m/frame 이동; 입력 수정 후 `active-guarded-observation.json` 237/876 fresh·최대306.57m·guard reset2·초기화 timeout2
- 사용자 동작: 매우 부드러운 천천한 이동; 실기기 independent GT 없음
- 확인한 입력: Generic Sensor 미래 clock, 평균 gyro 오차 주입, 활성 Camera2 초광각·EIS ON, 69° 가정과 실제 HAL103° 차이, 매우 흐린 grayscale frame
- 보존: 기존 dirty source·TLS·WASM·NAS dataset·서비스·기존 기록; 실험 출력은 local `build/`

## 실행 순서·소유

1. Leader: 실기기 native baseline·source/dependency/served fingerprint; 제한된 입력 image/IMU 기록과 수치 요약
2. Camera lane: 명시 camera 선택 지원; `camera 0, facing back` 요청 → 실제 HAL active ID·영상 선명도·안정화 metadata 확인. zoom1을 메인렌즈 선택 근거로 사용 금지
3. Numerical lane: 초기 scale 정렬의 rank·유효성 회귀 → rectangular·column-scaled rank-revealing solve; finite·strict-positive scale·실패 전파
4. Leader: 별도 native/WASM build → native 기존·신규 회귀 → 동일 입력 desktop/Worker 비교 → 검증 candidate 승격 → 실제 phone 재기록
5. Reviewer: 공개 pose·초기화 상태·실행 metadata·검증 주장 독립 검토

## 수정 파일

- Camera: `web/js/camera.js`, camera 선택 관련 app 호출, 신규 browser 회귀
- Numerical: `src/frontend/initialization/initial_alignment.cpp`, 관련 header, `tests/test_numerical_contracts.cpp`
- Leader: `scripts/dev/android-snapshot.mjs`, 제한된 exact-input recorder/replay 개발 도구, runbook·증거 문서
- Worker5m fence는 명백한 발산 공개·상태 전파 차단; 정확도·초기화 개선 근거로 사용 금지

## 수락·실패 조건

- Camera 선택: 실제 요청한 후면camera와 HAL active ID 일치, 선언된 pixel domain·K·crop·rotation·distortion·EIS 한계 기록
- 입력 보존: grayscale/IMU·timestamp 순서와 accepted/drop 구분; 원본 image·IMU가 없는 numeric observation을 완전 replay로 보고 금지
- 정렬 회귀: constant velocity+rotation과 stationary의 scale nullspace 거부; 독립 excited trajectory의 scale2.3·gravity·velocity 회복; 0·NaN·실패 solve는 초기화 성공 제외
- Native/WASM: 현재 baseline 회귀 보존, 새 source와 artifact SHA 연결, 현재 loader를 새 source라고 주장 금지
- 실기기: fresh coverage·재초기화/guard reset·pose jump·정지/이동 구간별 drift를 분리. 포즈 0·reset 반복·숨긴 발산은 성공 제외
- 수치 rank 통과만으로 noise·실기기 scale 정확도 보장 금지; 관측 가능한 excitation·실측 calibration·독립 GT 필요
- 임계값 완화·평활화·임의 scale/속도 cap으로 문제 은폐 금지
