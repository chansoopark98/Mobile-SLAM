# Z Flip 6 실제 지연·Lost 측정

- 날짜: 2026-10-12, Asia/Seoul. 기록 시작 14:40:50, 길이 30.039725초
- 기기: SM-F741N, Android 16/API36, Chrome 154.0.8037.126, 무선 ADB
- 코드: `98b396c`, 시작 working tree clean, 제품 코드·알고리즘 변경 없음
- 카메라: `?camera=0`, 480×640, 설정 30fps, 회전 none, Mobile Default
- 실제 C++ readback: LK21/3, feature120/min_dist30/F1, solver60ms/10iter, PnP OFF, rotation rescue OFF
- IMU: Generic Sensor 미래 시각 검출 후 DeviceMotion fallback. raw gyro 사용, 후보 bias 미적용
- 보정: heuristic_unverified, fx=fy=465.602889px, 69° 가정, 왜곡0·2cm lever arm 가정. 실측 calibration 아님
- 실행 서버: 기존 `node server.js`, PID2553417, direct/unlabelled. ops helper의 stopped는 이 수동 서버 상태를 나타내지 않음
- 실제 전화 GET loader SHA256: `1066a22d647fd4d170eb3dba5bd66ba0b49463143a6dc5ee8cccbad07b68e0fc`. JS6개 local/served hash 일치
- 서버 health는 external-paired로 표시했지만 실제 활성 loader는 기존 manifest의 embedded WASM. health mode 표기와 실제 artifact provenance 구분

## 측정 결과

| 항목 | 결과 |
|---|---:|
| Worker 결과 / fresh pose | 480 / 447 (93.125%) |
| 처리율 | 기록시간 기준 15.98fps, input span 기준 16.04fps |
| 입력 frame 간격 p50/p95/max | 64.025 / 98.965 / 178.110ms |
| 엔진 시간 p50/p95/p99/max | 29.545 / 62.985 / 84.388 / 126.980ms |
| Worker queue p50/p95/p99 | 0.270 / 1.111 / 2.925ms |
| IMU bracket 대기 p50/p95/p99/max | 0 / 25.791 / 34.299 / 41.005ms |
| 수신 시 fresh pose age min/mean/p50/p95/p99/max | 43.620 / 74.457 / 68.030 / 117.896 / 129.750 / 196.215ms |
| IMU 전송 수 / 관측 cadence | 1249 / 약41.6Hz |
| IMU 시각 간격 p50/p95/max | 22.350 / 51.861 / 72.765ms |
| Lost 결과 / 연속 구간 | 33 / 9 |
| 기록 overflow | result0, IMU0 |
| 기록 중 client/engine epoch 변화 | 0 / 0 |

- Lost 이유: insufficient_current_features27, missing_bracket3, invalid_or_uncovered_imu_interval3
- 480결과 모두 initialized=true, 기록 중 engineEpoch1 유지. 9개 Lost 구간 뒤 fresh pose 재개
- 최대 fresh-to-fresh 출력 공백: 856.760ms. 연속 fresh pose의 최대 회전 변화43.316°/66.830ms, 이동 변화0.552m/64.875ms 관측. 물리 이동량·정확도 근거 아님
- UI pose age는 rAF 소비 시 한 번 계산되는 문자열. 위 수치는 `result.arrivalMs - poseTimestamp×1000`이며 UI 값·물리 노출 지연과 구분

## 동작과 실패 경로

- 채팅 안내와 실제 동작 구간이 일치하지 않음. IMU 관측상 처음20초는 저회전, 후반10초는 큰 회전. 계획한 마지막 정지10초 확보 판정 제외
- 0–20초: 결과353, Lost5(IMU 사유), 엔진 p95 약37.2ms, 검출 특징점 중앙값120
- 20–30초: 결과127, fresh99, Lost28. 엔진 평균47.769/p95 80.055ms, 검출 특징점 중앙값85, gyro 최대101.026°/s
- 시각 사유27건 전부 후반 회전 구간. 해당 실패행 특징점 중앙값95, 21/27행은80개 초과
- `insufficient_current_features`는 optimizer의 currentVisualFactors<2 조건. detector featureCount와 다르므로 검출 특징점 부족·영상 blur로 원인을 확정하지 않음
- missing_bracket3건: Worker40ms deadline 종료, engineProcessingMs=0. IMU transport overflow/stale/capacity/order/lossResets는0
- invalid_or_uncovered3건: index41/257/263, 영상 시각70.148765/82.704195/83.046485초. 직전 영상 이후 실제 IMU sample0개, 다음 raw sample은 영상 이후. 코드상 endpoint 보간은 가능하지만 preintegration이1단계로 구성되는 경로와 일치
- 1단계 factor의 공분산은 rank 부족 가능. 기존 수치 guard와 whitening을 제거하는 수정은 부적절. 실제 거부 slot/단계 수는 이번 결과에 없으므로 위 연결은 코드·IMU 시각으로 재구성한 추론
- 참조: `src/backend/optimizer.cpp`, `include/backend/factor/integration_base.h`, `include/backend/factor/covariance_whitening.h`, `tests/test_numerical_contracts.cpp`

## 기록 이후와 검증 한계

- 14:41:50 snapshot은 Tracking. 14:44:30 앱 정지 전 snapshot에서 engineEpoch5·Initializing 관측
- clientEpoch2는 동일. 기록 종료 이후 엔진 epoch1→5 변화4회는 확인하나, 상세 구간·reset trigger 미기록. 30초의 reset0을 전체 세션 안정성으로 확대하지 않음
- 기록 후 앱 camera/IMU 정지, recorder hook 제거 확인. 기존 서버·ADB 연결 보존
- 첫 시도는 앱 중지 상태여서 결과0·실패 파일 보존. 앱 재시작 및 Tracking 확인 후 실제30초 기록
- 숫자·IMU 전송 기록이며 grayscale/video 없음. 동일 영상 replay·correspondence·blur·독립 GT·ATE/RPE·발산0 검증 불가
- IMU 전송값 복사·5회 CDP snapshot의 관측 부하 포함. capture/read/render 세부 지연 및 frontend/backend 시간은 분리 측정되지 않음
- 처리 결과 fresh 비율은 수락된 입력 결과의 분모. 카메라 전체30fps frame coverage와 다름

## 후속 수정·검증 순서

1. PC로 전체 image/IMU·source timing·결과를 보존하는 별도 시험. 전화 화면 타이머로 구간 표시, 수집 오버헤드 별도 비교
2. LK/F inlier, 정상 depth/current observation, optimizer visual factor 수와 frontend/backend 시간을 같은 frame에 계측
3. 회전 시 visual factor 소실 및 엔진 tail 비용부터 개선. 특징수·해상도·solver를 일괄 축소하지 않음
4. 실제 sample 위상과1-step factor 충돌을 독립 수치 회귀로 재현. 보간 sample 복제·공분산 epsilon·guard 제거로 성공 처리하지 않음
5. 같은 source/artifact·입력·설정으로 native/WASM 및 TUM-VI/독립 fast-motion GT 회귀. phone A/B에는 실제 Lost·reset·fresh coverage·pose age/처리 간격을 함께 평가

## 증거

- PC 보존본: `/home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-12-ZFlip6/latency-143050/`
- 원본: `build/dev-android/snapshot-2026-10-12T05-40-50-044Z.json`
- 통계: 같은 basename의 `-analysis.json`, `-episodes.json`
- preflight: `build/dev-android/preflight-2026-10-12T05-36-56-569Z.json` (사용자 page reload 이전)
- postflight: `build/dev-android/postflight-2026-10-12T05-44-30-597Z.json` (측정과 동일 timeOrigin 확인)
- 기록 명령: `node scripts/dev/android-snapshot.mjs --samples 5 --record-seconds 30`
- 제품 코드 수정·추가 build·정확도 test 실행 없음. 이번 결과는 실제 기기의 bounded numeric 관측
