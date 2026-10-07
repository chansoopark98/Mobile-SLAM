# 실제 모바일 pose 불안정 — 입력 관측 도구

- 사용자 증상: 개발 PC 결과 대비 실제 모바일 camera pose가 크게 튀고 불안정
- PC TUM calibrated replay와 모바일 live camera/sensor 입력·calibration 계약의 차이를 먼저 분리
- 현재 상태: 관측 도구 VERIFIED(Chromium fake-camera/mock-sensor); 실제 phone 원인·문제 해결 UNVERIFIED
- 기존 `App`, `Camera`, `IMU`, `VIOWrapper`, `Worker`, C++·WASM 파일 변경 없음

## 확인된 현 구현과 원인 가설

| 구분 | 코드 근거 | 의미·검증 필요 |
|---|---|---|
| VERIFIED 코드 | `web/js/app.js:288` | 기본 rear FOV69° 가정; `:205` URL/track/zoom/focusDistance heuristic 대안. 실제 기기 렌즈의 K/calibration 증거 없음 |
| VERIFIED 코드 | `web/js/app.js:683`, `:685` | `t_ic=[0,0,-0.02]`, 화면 방향별 고정R, 중심 principal point, PINHOLE2·distortion 입력 생략(0 default). 실제 camera-IMU calibration과 구분 |
| VERIFIED 실행·코드 | `web/js/app.js:592` | `enableLandscapeCrop()` 호출 현재 주석 처리; default live path 480×640 portrait /crop=`none`. Camera v9 crop 설명은 활성 호출 근거 제외 |
| VERIFIED 코드 | `web/js/camera.js:150`, `:532`; `web/js/app.js:654`, `:1310` | rotate detection/CPU draw/canvas conversion, processScale K scaling. 초기 even rounding과 orientation 재설정 rounding 차이 존재 |
| VERIFIED 코드 | `web/js/imu.js:370`, `:389`, `:445` | IMU가 센서 sample timestamp 대신 callback `performance.now()` 사용 |
| VERIFIED 코드 | `web/js/imu.js:361` | Generic accel/gyro 독립 event + 최신 반대 센서값 조합 +dedup; pairing 시각 동기화와 구분 |
| VERIFIED 코드 | `web/js/imu.js:94`, `:455`; `web/js/app.js:915` | iOS 가속도 sign 보정, DeviceMotion deg/s→rad/s, device→VIO 축변환 `[x,-z,y]` |
| VERIFIED 코드 | `web/js/app.js:873` | video `presentationTime` 사용; 물리 capture 시각과 구분. sensor arrival timestamp와 시간 offset/jitter 검증 필요 |
| VERIFIED 코드 | `web/js/app.js:784`, `:805`; `web/js/imu.js:129` | camera/configure async 및 DeviceMotion precheck 이후 권한 요청; iOS transient activation 유지·permission 실패 가능성은 실기기 가설 |
| VERIFIED 코드 | `web/js/imu.js:219`, `:237` | 정지 calibration gravity 경고/gyro clamp 후에도 calibrated=true. 움직이는 calibration의 영향은 실기기 가설 |
| VERIFIED 코드 | `web/js/app.js:1136`; `web/js/vio-wrapper.js:332` | 거절 frame도 FPS 집계 가능; result frame ID·timestamp 부재로 pose-age 검증 불가 |
| VERIFIED 코드 | `web/js/vio-worker.js:108`, `:240` | IMU0.5s stale cutoff, frame-gap1.5s guard/reset. pose jump와 reset·tracking loss를 함께 조사 |

- 위 가정들의 실제 기기 오차 크기·주된 원인 순위는 미확정
- 가설 검증 순서: camera/IMU permission·존재 → 정지 bias/gravity·axis → timestamp/paired sensor lag → 실제 K/R/t calibration → frame/queue/result age → 추정기 수치·reset 정책
- 정확도 threshold를 만족했다고 해석 가능한 실제 device ground truth 없음

## 관측 방식

- 신규 `/audit-mobile.html`의 same-origin iframe으로 기존 `/index.html` 실행
- 기존 app import 문장에서 실제 Camera/IMU/VIOWrapper URL 추출; 동일 `?v=11` module instance prototype hook
- 동일 engine 입력·buffer 내용·resolved return값 유지; VIO/permission hook은 원래 Promise 반환, Camera/calibration async 관측은 추가 continuation 포함. Diagnostic copy 후 기존 transfer 실행
- 원본 이미지·영상 저장 없음: camera metadata와 grayscale byte count·capture duration만 기록
- raw DeviceMotion /raw Generic sensor /bias 전후 device IMU /flush 시 device IMU /실제 VIO 전달값 분리
- video callback metadata: `mediaTime`, `presentationTime`, `captureTime`, `receiveTime`, callback/arrival 시각; 미제공값 null
- permission wrapper/native request entry/result/error/resolve 시각과 `userActivation` snapshot
- configure/solver/tracker/PnP args·success, rotation/crop/track settings·capabilities, source/artifact SHA256
- frame attempt/accepted/dropped/completed, pose16개값·finite, translation jump m·rotation jump deg, wrapper reset generation, tracking interruption
- result association은 제출 FIFO 대용값; engine frame ID·pose timestamp 부재 명시. `physicalCaptureToPoseMs=null`
- `metadata.clocks`: 부모 recorder `atMs`와 iframe hook `performance.now` 각각의 `timeOrigin` 및 clock-domain label 포함; epoch mapping 가능, 실제 hardware capture 시각 보증 제외
- jump marker0.5m/20°는 탐색용, 실제 움직임·정확도·validity 판정 기준 제외
- 기존 remote `sendBeacon` 차단, 서버 write methods405; local Blob JSON 다운로드만 지원

## 실행·사용

```bash
# 실제 phone: 접근 hostname/IP가 포함된 신뢰 가능한 기존 certificate 사용
python3 scripts/dev/serve-mobile-audit.py --host 0.0.0.0 --port 8766 \
  --cert /absolute/trusted-cert.pem --key /absolute/key.pem
# phone browser: https://<reachable-host>:8766/audit-mobile.html

# localhost 검증 전용 self-signed cert; 기존 인증서 경로 변경 없음
python3 scripts/dev/serve-mobile-audit.py --generate-dev-cert --host 127.0.0.1 --port 8766
node tests/browser/mobile-audit-browser.mjs

node --test tests/browser/mobile-audit-contracts.test.mjs
python3 tests/browser/mobile-audit-server.test.py -v
```

- self-signed cert: `build/dev-mobile-audit/certs/`, localhost SAN /7일 /key0600; 실제 phone에서 신뢰되는 TLS 구성 증거 제외
- 페이지: 앱 열기 → 기록 시작 → iframe의 기존 Start 직접 클릭 → camera/IMU 권한 허용 → 정지 구간·재현 동작 → 기록·센서 종료 → JSON 내보내기
- 기본60초 /최대120초, channel별10000 /전체40000 rows /WASM log1000; 한계 후 기록·sensor·worker 종료
- Stop: camera/IMU stop, worker dispose, hooks 복원, iframe unload; 재시작은 앱 열기부터 수행
- schema: `mobile-slam-observation-v1`; raw images/video/device ID/group ID/track label/credentials 제외
- 키·인증서 dot directory, secret suffix, percent-encoded variant, traversal, 숨김/외부 symlink 정적 접근403

## 검증 결과

| 검사 | 실제 결과 |
|---|---|
| Recorder analytic | 4/4 PASS: count/duration 한계·stop·numeric snapshot·5m/90° pose jump |
| Safe server fixture | 4/4 PASS: public JS, decoded hidden path, traversal, secret/outside symlink denial; test sentinel만 사용 |
| Existing app browser integration | Chrome144.0.7559.96, HTTPS secure context, source/module SHA 일치, iframe 기존 Start 실행 |
| Data path | fake-camera480×640 portrait/crop none, raw/mock Generic+Motion, device→VIO `[0,9.81,0]`→`[0,0,9.81]` 관측 |
| Permission | mocked grant·denied·async/sync NotAllowedError·inactive activation 보존·기록 |
| Local export/stop | JSON 실제 download, 중지 후 row 증가0, pageerror0 |
| Network writes | browser non-GET/HEAD request0; server POST/log405 |
| Private HTTP | GET4 +HEAD4 decoded key/traversal route 모두403 |
| Mobile viewport | 412px 화면 controls/한국어 설명 읽힘; local screenshot 기능 배치 QA92. 참조 이미지 부재로 fidelity 판정 제외 |

- 이번 mock: frame attempt13 /accepted13 /completed12 /pose0; 정지 synthetic 센서이므로 추정 정확도·초기화 성공 근거 제외
- observer push/copy1218회 총9.060ms /단일최대0.060ms(desktop mock); listener·wrapper·summary·GC 전체 overhead 제외
- UI summary는 internal state 계산;1초마다 전체 recording deep clone 없음. Deep clone은 최종 export 전용
- observer 자체가 mobile timing/GC·iframe lifecycle에 영향을 줄 수 있음; 동일 실기기에서 normal app vs audit app A/B 필요
- 실제 Android/iOS permission, raw hardware timestamp 의미, axis/sign, intrinsic/extrinsic, thermal, pose jump 해결: UNVERIFIED
- full image/IMU replay recorder가 아님; 이미지 없이 source-clock·calibration·입출력 계약을 관측하는 dev tool

## 산출물

- 신규 도구: `web/audit-mobile.html`, `web/js/audit-mobile.js`, `scripts/dev/serve-mobile-audit.py`
- 신규 테스트: `tests/browser/mobile-audit-contracts.test.mjs`, `mobile-audit-server.test.py`, `mobile-audit-browser.mjs`
- evidence: `mobile-audit-contracts.log`, `mobile-audit-server-contracts.log`, `mobile-audit-browser.log`, `mobile-audit-browser-evidence.json`, `mobile-audit-mock.json`, `mobile-audit-mock.png`, `mobile-audit-visual-verdict.json`
- 남은 범위: 실제 phone에서 수동 재현 JSON, normal/audit A/B, 독립 calibration·clock·pose ground-truth 평가 후 scoped core 수정
