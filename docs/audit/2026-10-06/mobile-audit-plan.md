# 모바일 pose 불안정 진단 도구 실행 범위

- 문제: PC dataset replay와 별개로 실기기 camera pose가 크게 튀는 증상
- 변경 범위: 새 audit page /관측 module /safe local server /mock 계약 테스트; 기존 App·Camera·IMU·VIOWrapper·Worker 수정 제외
- 방식: same-origin iframe에 기존 `/index.html` 로드, 앱과 동일한 `?v=11` module URL의 prototype을 관측; algorithm 입력·return값·worker message 불변
- 순서: recorder 계약 테스트 작성 → bounded recorder 구현 → iframe 실제 module hook → 안전한 로컬 export → Chromium mock 실행·report 검사
- 기록: raw DeviceMotion/Generic sensor timestamp·arrival, bias 보정 전/후 device IMU, worker 전달 VIO IMU, camera metadata/rotation/crop, calibration/solver API args, accepted/dropped/completed frame, output pose·jump m/deg, reset/visibility/orientation, source/artifact hash
- 제한: 기본60초 /최대120초, channel별 최대10000 samples, log1000; raw image·video·device ID·track label·사용자 자격증명 저장 제외
- no upload: iframe의 기존 sendBeacon log 전송을 dev page에서 차단; export는 local Blob download, 서버 POST/PUT/PATCH/DELETE405
- 실행: 기존 Start를 iframe 안에서 사용자 직접 클릭하여 camera/IMU permission 유지; audit 상단 기록 Start/Stop/Export; Stop에서 sensor/camera/worker도 종료
- 성공 조건: 정상 앱 경로와 같은 module identity, input/output buffer·함수 return 불변, 제한값 준수, no network writes, reset 전후 jump 구분, timestamp provenance 명시, source hash와 environment export
- 검증: analytic m/deg jump·non-finite, bounded counts, transform 관측·transfer 원본 보존, reset marker, generic timestamp/arrival, Chromium fake-camera+mock sensors와 JSON local export
- 실패 조건: hooks가 다른 module instance에 설치, queue 결과를 실제 sensor capture 시각으로 오기, export 민감 metadata 포함, recording stop 후 sample 추가, 기존 로그/인증서/아티팩트 덮어쓰기
- 실기기 미접속 한계: recorder 동작 검증과 모바일 pose 문제 해결을 구분; 재현 JSON 확보 후 calibration/time/axis 가설 검증
