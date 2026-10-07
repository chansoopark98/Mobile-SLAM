# Mobile browser cleanup·검증 계획

- 소유: `web/js/{app,imu,camera,orientation,renderer,test-tumvi-app,audit-mobile}.js`, 필요한 `web/{index,test-tumvi}.html`, `tests/browser/mobile-source-contracts*.mjs`
- 기존 dirty source·generated WASM·TLS·dataset·서비스 보존; 다른 lane 파일 수정 제외
- 삭제/통합: accel+gyro callback별 중복 emission·4ms dedup, callback 시각의 sensor/capture 명칭, permission 이전 DeviceMotion 대기, rAF마다 같은 pose append, replay의 처리 후 고정 delay
- 유지: device→body `[x,-z,y]`, 기존 iOS acceleration sign·deg/s→rad/s, calibration heuristic fallback·PnP 기본 off; physical 검증 없이 축/중력 규약 변경 금지
- Generic IMU 정책: gyro source timestamp에 한 번 emission; 새 gyro는 다음 accel callback까지 보류 가능; 두 source finite/readiness·시간 차 `≤1.5/requestedHz` 검증, 최신 accel hold 명시. interpolation/hardware synchronization 주장 제외
- 시간축: `Sensor.timestamp`의 time-origin 상대 ms→초; DeviceMotion event 상대 ms 또는 legacy epoch ms를 `performance.timeOrigin`으로 매핑; arrival 별도. video presentation/callback 시각 proxy 표시; 노출 시각 생성 금지
- calibration: optional measured JSON profile의 drawImage source pixel/frame·provenance·K·SO(3)·distortion 검증; 명시적인 0/90/180/270° pixel rotation→crop→pixel-center resize로 K·camera/body extrinsic 일치
- renderer: frame/session/engine epoch로 새 fresh valid pose만 표시; reset/loss에서 pose·map 비움; smoothing 추가 제외
- replay: room1/room4 route, exact frame bound, source-time deadline paced/max-speed 구분; raw result·poseTimestamp·solver·count dump, reset/run cancellation
- old-fail 회귀: delayed callback sensor time, stale/missing pair, high-rate no double count, ring overflow, iOS activation, K rotation/crop/rounding, lost/epoch render
- 명령: `node --test tests/browser/mobile-source-contracts*.mjs`; 기존 `mobile-audit-contracts`·`browser/contracts`; `bash scripts/dev/check.sh syntax`; isolated Playwright 실제 앱/관측/replay 후 evidence 기록
- 한계: mock/desktop replay는 실제 iOS/Android sensor·capture 시각·camera calibration·thermal·field GT·실제 pose jump 개선 증거 제외
- API 근거: [Generic Sensor](https://www.w3.org/TR/generic-sensor/), [Device Motion](https://www.w3.org/TR/orientation-event/), [video callback](https://wicg.github.io/video-rvfc/)


## Camera profile 입력 계약

- URL: `/?cameraProfile=/public/device-camera.json`; 동일 origin의 허용 static JSON 파일. 선택 crop `crop=landscape_4_3`, 기본 none
- `schema=mobile-slam-camera-profile-v1`, `pixelFrame=drawImage`, `orientationType=portrait-primary|portrait-secondary|landscape-primary|landscape-secondary`, 비어 있지 않은 `provenance`
- `width,height`: browser `drawImage` pixel domain에서 직접 보정한 크기. manual rotation이면 native video dimensions, browser가 이미 회전한 출력이면 그 출력 크기. 치수·orientation 불일치는 오류; 새 FOV 가정으로 대체 제외
- `fx,fy,cx,cy`: finite, focal positive, integer pixel center 좌표 `0..width-1/height-1`; center-preserving resize: `c'=(c-crop+0.5)*(output/cropSize)-0.5`
- `r_ic`: 9개 row-major `R_B_C`, B는 기존 VIO fixed body `[x,-z,y]`; finite·orthogonal·det+1. `t_ic`: 같은 body frame에서 camera center의 3개 m
- `modelType=0`: Kannala-Brandt, `distortion=[k2,k3,k4,k5]`; `modelType=2`: PINHOLE, `distortion=[k1,k2,p1,p2]`. Worker API slot 이름은 공통 `k2,k3,k4,k5`; PINHOLE의 tangential vector `[p2,p1]`도 image rotation과 함께 회전
- explicit rotation `none/cw/ccw/half`→crop→실제 반올림한 output size 순서. `R_B_Cnew=R_B_Cold*Q^T`, lever arm `t_ic` 보존
- UI provenance: profile 입력 `supplied_calibration_unverified`; fallback `heuristic_unverified`. 파일 형식 검증·사용자 provenance 문자열은 현장 calibration 정확도 보증 제외

## Replay 소비 계약

- `/test-tumvi.html?dataset=room1&frames=2821&speed=-1`; room4 `dataset=room4&frames=2228`
- `speed=-1`: max-speed serial; `speed=1/2/5`: CSV 시각 기반 absolute deadline, busy arrival drop; `speed=0`: 수동 step
- parity pilot solver override: `solverTime=<seconds>&iterations=<integer>&features=<integer>`; 기본값 0.1s/10iter/150feature. 양 경로의 설정과 실제 termination 비교 필수
- IMU: cursor0부터 `<=frame` + first future bracket1회; 이전 전달 bracket이 아직 future면 신규0; carry는 Engine 소유
- `inputHash=1`: per-frame 실제 decoded grayscale SHA-256·hash 소요시간. parity provenance 전용 선택; paced timing은 hash 비활성 기본값으로 별도 실행
- 각 row의 IMU cursor start/end·first/last timestamp·sample count: 전송된 batch의 실제 범위; 미래 bracket 중복 확인 가능
- `window.__tumviReplay.export()` / JSON 버튼: `mobile-slam-replay-v2`, `completed/failed`, raw per-frame result·engine actual pose timestamp·epoch·solver·queue/copy/decode/roundtrip, 입력/accepted/completed/dropped/error/fresh pose counts
- 정확한 bound는 앱 내부에서 종료; runner의 Pause race로 N+1 frame 확대 제외. reset 중 decode 완료는 run generation으로 폐기
- CSV HTTP 오류·nonfinite/duplicate/out-of-order input·decode 실패·worker timeout·timestamp mismatch는 실패 기록/중단. 0 pose는 tracking·accuracy 통과 근거 제외

## 실행 근거

- `build/refactor-evidence/mobile/contracts-red.log`: 최초 6개 fixture 실패
- `contracts-before-calibration.log`: 추가 K/profile 2개 실패; `contracts-before-replay.log`: 추가 replay 3개 실패
- `node --test tests/browser/mobile-source-contracts.test.mjs tests/browser/mobile-audit-contracts.test.mjs`: 현재 source 17 + 기존 recorder4 = 21 pass /0 fail /0 skip /0 TODO
- `bash scripts/dev/check.sh syntax`: 통과; JS parse 검증, ESLint/TypeScript typecheck 아님
- isolated dev Playwright `node tests/browser/mobile-source-contracts-browser.mjs`: actual Chromium fake camera·mock source clock/permission/axes/profile(회전+crop+resize 실제 configure)·observer matching/paused epoch, 결과 `browser-contracts.json`
- test HTTP `127.0.0.1:8767`: 별도 read-only audit server + 보존된 active artifact. 최종 fresh artifact·real7002 HTTPS 검증과 구분; 소유 process PID1316952를 command 확인 후 종료
- visual verdict: `.omx/state/mobile-refactor/ralph-progress.json`; functional viewport QA, 제공된 디자인 reference 부재로 fidelity 판정 제외

- 최종 local mock 실행: 5/5 경로(Android grant, iOS grant/deny/error, supplied profile), pageerror0; source clock·actual activation·pause/reset epoch·duplicate reply 무집계·계약 수치 검증
- profile 실제 호출 golden: raw640×480 → CCW480×640 → centered480×360 crop →322×240. `fx=410*322/480`, `fy=400*240/360`, `cx=330.5*322/480-0.5`, `cy=279.5*240/360-0.5`; transformed tangential `[-0.004,0.003]`, unchanged body lever arm
- actual canvas byte oracle: none/cw/ccw/half 4/4; 412px/1280px replay·384px live iframe horizontal overflow0, status bottom≤viewport height
- 최종 source hash: `build/refactor-evidence/mobile/source-manifest.json`; generated WASM·원본 dataset 변경 없음
- 최종 fresh candidate/real7002: parent 통합 runner 소유. local mock 결과를 fresh artifact 또는 trusted HTTPS/phone 증거로 보고 제외


## 실제 same-origin profile gate — 준비

- test-only `MOBILE_SOURCE_REAL_PROFILE=1`: `web/public/mobile-slam-camera-contract-<pid>.json` 고유 synthetic fixture를 exclusive create, 실제 GET/HEAD+SHA/JSON·no-store/COOP/COEP·strict TLS·server health 확인
- Playwright route interception 제외; 실제 `/public/` 응답을 Start에서 읽고 configure K/R/t/왜곡 golden 확인. fixture·필요 시 새 빈 public directory는 `finally`에서 제거
- root의 새 candidate 승격·7002 readiness 신호 이후, performance 시작 이전 실행. 현재 준비 단계; 아직 실제 실행 통과로 집계 제외
- 명령: `MOBILE_SOURCE_REAL_PROFILE=1 MOBILE_SOURCE_TEST_ORIGIN=https://dev.serdic.com:7002 MOBILE_SOURCE_TEST_OUTPUT=build/refactor-profile-same-origin BROWSER_EXECUTABLE=build/refactor-validation/shared/chrome-strict-resolver.sh node tests/browser/mobile-source-contracts-browser.mjs`
- wrapper: actual Chrome + `MAP dev.serdic.com 127.0.0.1`, SSL ignore 없음; system trust/hostname 유지. 실제 phone calibration·field GT 판정과 구분

- 최종 diagnostic truth 수정: actual `params`의 fx/fy/cx/cy/R/t 로그 사용; supplied profile에 default FOV focal의 scaled-from 문구 제외. engine/configure 결과·C++ unchanged
- 진단 수정 후 source contracts21/21 + syntax/diff pass, app SHA256 `097641e47e1bec8823046c27ec6937d0f21a4bed9117b6488eb905033f14a25f`; parent 최종 web snapshot/fresh actual mock에서 확인


## run-4 상태 — profile proof 대기

- 판정: **FAILED before downstream mock**, 첫 paired full room1의 미리 선언한 cap 초과. `build/refactor-validation/run-4/early-stopped.json`; frozen_contract=false
- 실제 paired2821 frames·init45/45·fresh2776/2776·coverage차0. translation p95 `0.001472714440022603m` > 독립 cap `0.001m`, max `0.0018837114973963708m`; rotation p95 `0.008300623454160053°`/max `0.010768801178835°`
- solver4 differences: frame1105 iterations6/11·CONVERGENCE/NO_CONVERGENCE, frame1106 iterations6/5, frame1107 iterations7/6. `pilot-1/early-comparison.json`, `first-solver-fork.json`
- 원인 분리: 첫 관측 error cliff1094에서 `4.603545118854902e-7m`→`0.0005041658983308039m`, 당시 solver11/NO_CONVERGENCE 양쪽 동일. 후속 solver1105 분기를 최초 원인으로 단정 제외; `pre-solver-error-growth.json`
- 위 failure는 현재80 native/SAN·bounded520 검증을 폐기하거나 full gate 합격으로 바꾸지 않음. cap 완화·GT tuning 없음; parent/Build owner의 기존1110 진단 진행
- run-4 mock `browser-contracts.json` 없음. real-public GET/HEAD·TLS·configure·temporaryProfileRemoved 증거 **UNVERIFIED/PENDING**; 준비된 fixture mode를 실제 실행 통과로 집계 제외
- 이 문서 갱신에서 code/browser/test/replay 실행0. 소유 runner 종료·실제7002 서비스 유지는 root/Transport 기록; 검토자의 서비스 제어 없음
