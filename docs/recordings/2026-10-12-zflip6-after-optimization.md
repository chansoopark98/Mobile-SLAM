# Flip 6 최적화 후 실제 재시험

- 날짜: 2026-10-12, Asia/Seoul. 20:59:15 시작, 30.031325초
- 판정: **처리율 증가·IMU 구간 오류0 확인. 회전 Lost·출력 불연속이 남아 실기기 수락 조건 미충족**
- 코드: 미커밋 최적화 후보, HEAD98b396c. 실제 제공/전화 GET WASM SHA256 `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`
- 기기: SM-F741N, Android16/API36, Chrome154.0.8037.126. ADB `192.168.0.141:36885`, CDP9222
- 입력: camera0, 480×640, 설정30fps, DeviceMotion, Mobile Default, rotation rescue ON. 실측 calibration 없음
- 동작: 화면 타이머0–10초 정지/10–20초 평소 Lost 동작/20–30초 정지 안내. 실제 IMU에서는20–22초에도 회전 잔여,23초부터 저회전

## 전후 관측

- 수신 결과 총량: 기존480/fresh447, 수정609/fresh576
- 공정 시간창: `startedAtMs <= inputTimestamp×1000 < stoppedAtMs`. 시작 전에 캡처된 기존1개/수정2개 결과 제외
- 아래 비율의 분모는 시간창 안의 수신 완료 결과이며 전체 camera30fps source frame coverage가 아님

| 항목 | 기존 | 수정 후 |
|---|---:|---:|
| 시간창 결과 / fresh | 479 / 446 | 607 / 574 |
| 처리율 | 15.95fps | 20.21fps |
| fresh 비율 | 93.11% | 94.56% |
| Lost 프레임 / 연속 구간 | 33 / 9 | 33 / 6 |
| 영상 제약 부족 Lost | 27 | 33 |
| IMU interval / bracket Lost | 3 / 3 | **0 / 0** |
| 수신 pose age p50 / p95 | 68.02 / 117.92ms | 74.97 / 120.53ms |
| 엔진 시간 p50 / p95 | 29.54 / 63.05ms | 33.52 / 69.77ms |
| IMU 대기 p50 / p95 | 0 / 25.80ms | 0 / 12.68ms |
| 기록 중 epoch 변화 | 0 | 0 |
| recorder result/IMU overflow | 0 / 0 | 0 / 0 |

- 처리율 관측 증가 약26.8%, fresh 비율+1.45percentage points. **pose age·엔진 p95 지연 감소는 확인되지 않음**
- 수정 기록의 IMU factor: 모든609행에서10개 수락, rejectedImuFactors0. ring overflow/stale/capacity/order/lossResets0
- 실제 제출 IMU: 기존 약41.58Hz → 수정42.52Hz, 수정 source 간격 p50/p95/max22.98/47.20/68.28ms. 하드웨어 원시 sampling rate·실제 exposure 동기 증명 아님
- 수정 bracket 대기 최대47.07ms, budget 상한80ms; 이전40ms보다 늦은 실제 bracket 수락 관측
- 수정 UI 상태 갱신 기준 pose age 평균94.10/p95 128.67ms. 기존에는 동일 per-frame UI 시각이 없어 전후 비교 불가

## 남은 Lost와 병목

- Lost33개: `currentVisualFactors=0` 29개, `=1` 4개. Ceres·marginalization은 해당 행에서 미실행
- Lost 때 tracker feature 중앙값53, eligible bank track80, immature track146. 현 프레임까지 이어진 유효 성숙 track이 부족
- optimizer의 invalidDepthFeatures/projectionRejects/rejectedImuFactors는0. 이 수치는 upstream 삼각측량 실패·track 삭제 횟수를 포함하지 않음
- 최대 fresh 출력 공백 약632.84ms. 기록 중6개 Lost 구간 모두 이후 fresh 출력 재개, 엔진 epoch2 유지
- 새 품질 경고: 유효 pose576개 중 `tracking:acc_bias_threshold`159개. 회전 후 정지 구간에도 지속. finite/fresh pose를 물리 정확도로 판정하지 않음
- 인접 fresh 출력에서 위치 변화1.072m/37.545ms, 회전 변화44.697°/22.785ms 관측. 주변 gyro와의 불일치 정황이며 독립 GT 없는 수치 불연속으로 기록
- 현재 stage 전체609행 p50/p95: frontend10.82/41.38ms, backend21.61/42.45ms, IMU0.105/0.235ms, Worker queue0.145/0.863ms
- Ceres p50/p95 20.26/29.96ms, marginalization p95 16.01/p99 36.04ms. 서로 다른 stage의 p95를 더하여 전체 latency로 해석하지 않음
- copy 소유 이전은 매칭608행에서 추가 frameCopyMs0. main snapshot pixel read p50/p95 약16.93/23.50ms; source→submit proxy31.98/44.48ms, 수신→UI 상태 갱신16.28/32.81ms

## 비교·보존 한계

- 기존 회전은 마지막10초, 수정 회전은 중간10초에 집중. 수정 회전 최대178.03°/s vs 기존101.03°/s로 강도가 높음
- 동일 자세·이동·조도·온도·영상이 아니므로 전후 차이를 코드 변경의 독립 효과 또는 정확도 회귀로 단정하지 않음
- 새 recorder는 sourceTiming/UI hook·clone·타이머 DOM 갱신 부하가 추가됨. numeric 기록이며 영상·독립 GT 없음
- 측정 시작은 이미 initialized/Tracking. 준비 preflight의 epoch1→측정 시작 epoch2 변화는 측정 전 구간으로 원인 미기록
- 종료 후 async effective-config 조회는 CDP timeout. 이후 Awake·ADB/CDP 응답을 확인하고 async 조회를 제외한 동기 stop으로 복구
- 21:05:54 stop snapshot은 visibility=hidden, epoch3/Initializing. 기록 이후 epoch2→3 변화 원인·시점은 미기록; 30초 reset0을 전체 세션 안정성으로 확대하지 않음
- 최종 app.running=false, imuRunning=false, recorder hook 제거 확인. 서버·ADB 연결 보존
- 원본 SHA256: `11f0bba39fe08871a3bd9c26ab3c72df406f9c330e7a9e3e5fcfa91904ade07d`
- PC 보존본: `/home/park-ubuntu/Documents/Mobile-SLAM-recordings/2026-10-12-ZFlip6/optimized-205915/`
- 원본: `build/dev-android/snapshot-2026-10-12T11-59-15-089Z.json`, 같은 basename의 `-analysis.json`, `-details.json`, `-comparison.json`
- 전후 원본을 독립 재계산한 리뷰와 코드의 factor membership 조건 대조. 이번 시험에서 제품 코드·threshold 변경 없음

## 다음 검증 대상

- 회전 시 장기 track 소실→새 track 성숙 대기→current factor0/1 경로를 영상·IMU 전체 입력으로 재현
- tracking ID/age, LK/F inlier, 삼각측량 실패·삭제, 현 프레임 factor 연결을 같은 timestamp에 기록. stale pose·guard 완화로 Lost를 숨기지 않음
- bias 경고·pose 불연속은 실제 K/extrinsic·영상 안정화·sensor/image timing과 함께 검증. 현재 numeric 입력만으로 단일 원인 확정 불가
- 후속 개선과 실제 near-zero Lost 판정은 추가 원본 image/IMU 재시험 및 독립 기준 확보 후 수행
