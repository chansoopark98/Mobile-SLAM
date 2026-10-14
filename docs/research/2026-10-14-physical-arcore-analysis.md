# 실기기 ARCore 기록 무결성·재생 분석 — 2026-10-14

- 대상: 사용자 허용 범위에서 저장한 `SM-F741N`, Android SDK 36의 실측 30초와 해당 MP4 재생. 원본 파일·제품 알고리즘 수정 없음.
- 분석기: [`analyze_physical.py`](../../scripts/dev/references/arcore/analyze_physical.py). 기존 `analyze.py`의 분위수·공개 pose 비교 함수 재사용; 실기기 파일 구조·manifest·PGM·수명주기 검증만 추가.
- 근거: [`analysis/all-three-verified.json`](../../build/physical-arcore-20261014/analysis/all-three-verified.json), [`captures/`](../../build/physical-arcore-20261014/captures/). 각 capture의 `manifest.json` 생성 완료 후 분석.
- 수락 범위: 저장 무결성, 공개 SDK 추적 상태, 시간·pose 표현 일관성, 반복 재생 관측. 독립 GT 없음; ATE·RPE·실제 위치 정확도·private solver 시간 미검증.

## 실측·재생 2회

| 항목 | physical-live-01 | physical-playback-01 | physical-playback-02 |
|---|---:|---:|---:|
| 저장 frame / TRACKING | 607 / 601 | 616 / 606 | 609 / 599 |
| TRACKING 비율 | 99.011532% | 98.376623% | 98.357964% |
| 첫 TRACKING index / 첫 frame 대비 source 시간 | 6 / 0.248706055s | 10 / 0.448570899s | 10 / 0.448570899s |
| 초기화 후 비추적 episode | 0 | 0 | 0 |
| camera timestamp span | 28.227616597s | 28.460975814s | 28.427827333s |
| 관측 frame 빈도 `(N−1)/span` | 21.468338Hz | 21.608535Hz | 21.387494Hz |
| image / 외부 IMU 저장 | 606 / 27,062 | 615 / 0 | 608 / 0 |
| 종료 summary / Session close 경과 | 30.234864051 / 30.479676499s | 29.829054364 / 29.947156499s | 29.771438895 / 29.853046083s |
| `Session.update` p50 / p95 / p99 | 3.519739 / 4.480568 / 4.952851ms | 3.612631 / 4.844271 / 5.422885ms | 3.670781 / 4.928636 / 5.690613ms |
| pointcloud count p50 / max | 85 / 203 | 83.5 / 200 | 84 / 202 |

- 3회 모두: summary·JSONL row·manifest 실제 파일 수/크기/SHA-256 일치, camera timestamp 유일·단조 증가, setup/cleanup 오류 없음, Session close 확인.
- 실측: 첫 6frame `PAUSED`, 나머지 601frame `TRACKING`. 첫 CPU image는 `NotYetAvailableException`; 나머지 606개 PGM 640×480 payload 길이·SHA-256 검증 통과.
- 관측 frame 빈도에는 이미지 파일 저장·JSON 기록·렌더링·SDK 호출이 함께 영향을 줌. 카메라 고유 frame rate, 엔진 최대 처리량, 누락 frame 수를 이 값만으로 확정할 수 없음.
- `Session.update`는 BLOCKING API 호출 시간. 센서 획득부터 pose 전달까지의 지연이나 내부 최적화 시간으로 해석하지 않음.
- 실측 camera 간격 p50/p95/p99: 33.315325 / 66.632649 / 66.706206ms; arrival 간격: 46.120000 / 54.141654 / 61.737051ms.
- 실제 CPU image timestamp − 공개 AR frame timestamp: 실측 p50 **+1.994077ms**, 범위 **−2.943422..+2.799020ms**; 첫 재생 p50 **+1.878134ms**. 두 timestamp를 동일 시각으로 덮어쓰지 않음.
- 공개 pose: translation 대 matrix 최대 차 0m; quaternion norm² 최대 오차 `2.9140e-7`, quaternion 회전 대 column-major matrix 최대 차 `2.3321e-7`, 회전 직교성 오차 `4.4822e-7`, determinant 오차 `5.9758e-7`.
- 이 pose 표현 일치는 내부적으로 일관된 출력이라는 뜻이며 외부 공간에서의 정확도 증명이 아님.

## 센서·기록

| Android sensor | count | source Hz | source interval p50 / p95 / p99 | interval 표준편차 |
|---|---:|---:|---:|---:|
| Accelerometer, type 1, m/s² | 13,532 | 461.340852 | 2.166458 / 2.166667 / 2.209688ms | 0.007068ms |
| Gyroscope, type 4, rad/s | 13,530 | 461.269894 | 2.166458 / 2.166667 / 2.209688ms | 0.039189ms |

- 센서별 timestamp 유일·단조 증가. 최장 source 간격: accel 2.216354ms, gyro 6.651303ms.
- source 간격과 callback arrival 간격 분리: arrival 최장 간격은 각각 125.661042 / 126.874583ms. 실제 센서 source timestamp와 callback 묶음 도착을 혼동하지 않음.
- SensorManager 관측 IMU가 ARCore 내부 소비 IMU와 바이트·calibration·필터 측면에서 동일하다는 근거 없음. Playback은 기록 내부 IMU만 사용하며 외부 IMU observer가 꺼져 있음.
- 실측 MP4: **31,422,921 bytes**; SHA-256 `8a822bdd97a09ebdad0986469493d02407241304d4ce2be90c2d4a4b33adaea0`.
- 재생은 별도 MP4를 생성하지 않음. 실제 source MP4 전후 hash 검증은 상위 [실기기 실행 보고서](2026-10-14-physical-arcore.md)에 연결.

## 실측 대 첫 재생

- 공개 AR frame timestamp 정확 교집합: **5개**; 양쪽 TRACKING 교집합 **1개**. 전체 궤적을 이 연결만으로 비교할 수 없음.
- CPU image timestamp 정확 교집합: **453개**; 실제 PGM 바이트 hash 일치 **0/453**. 기록·복호화 경로 영향 가능성이 있으나 이 분석만으로 원인 확정 불가. 실측과 재생의 영상 바이트가 같다고 주장하지 않음.
- CPU image timestamp로 연결한 양쪽 TRACKING: **449개**. 연결된 pose에는 각 AR frame/image timestamp 차가 남아 있으며 보간하지 않음.
- 정렬 없는 원시 world translation 차 p50/p95/p99: **0.015965 / 0.031009 / 0.033168m**; rotation 차: **0.420671 / 0.904892 / 0.954136°**.
- 세션별 world 원점·축 기준이 다를 수 있음. 위 값은 출력 간 차이이며 GT 기반 ATE, RPE, 절대 정확도가 아님.

## 동일 MP4 반복 재생

- Playback 01 ↔ 02 CPU image timestamp 정확 교집합 **452개**, 실제 PGM 바이트 hash **452/452 일치**, 해당 timestamp의 tracking state 불일치 0.
- 이미지 timestamp로 연결한 양쪽 TRACKING **443개**. 정렬 없는 translation 차 p50/p95/p99 **0.009095 / 0.009841 / 0.010051m**, rotation 차 **0.104142 / 0.285140 / 0.369198°**.
- 공개 AR frame timestamp 정확 교집합은 **13개**; 양쪽 TRACKING **3개**에서만 원시 pose 차 0. 서로 다른 timestamp 연결법의 결과를 혼합하지 않음.
- 공통 영상 바이트가 같은 구간에서도 관측한 frame 집합·image/frame 시간차·세션 실행은 다름. 전체 내부 IMU 바이트 동일성·private solver 결정성·GT 정확도 근거로 확대하지 않음.
- 실측 ↔ 두 번째 재생 결과도 `all-three-verified.json`의 별도 comparison에 보존.

## 가상환경 실패와 구분

- 기존 [emulator v3](2026-10-13-arcore-runtime-verification.md): TRACKING 0, pointcloud 0, record camera 간격 p50 **0.100003ms**.
- 이번 실측: TRACKING 601, pointcloud 최대 203, camera 간격 p50 **33.315325ms**. 실제 기기에서 SDK가 추적한다는 직접 실행 근거 확보.
- 입력 장면·센서·runtime 환경이 다른 실험이므로 성능 우열이나 emulator 실패 원인의 단독 증명으로 확대하지 않음.

## 재현·회귀

```bash
python3 scripts/dev/references/arcore/analyze_physical.py \
  build/physical-arcore-20261014/captures/physical-live-01 \
  build/physical-arcore-20261014/captures/physical-playback-01 \
  build/physical-arcore-20261014/captures/physical-playback-02 \
  --output build/physical-arcore-20261014/analysis/new-comparison.json
python3 -m unittest discover -s scripts/dev/references/arcore -p test_analyze_physical.py -v
```

- 출력은 새 repository `build/` 경로로 제한. 기존 출력, capture 내부, `/mnt/backup/SLAM` 및 `MOBILE_SLAM_DATA_ROOT` 쓰기 거부.
- 잘못된 입력은 `validation: FAIL`과 비정상 종료로 기록. 빈 frame·누락 파일·manifest/hash 불일치·비유한 수·중복 timestamp·summary/row 불일치·pose 표현 오류를 정상 분석으로 취급하지 않음.
- 독립 값 기반 회귀 **8개 통과**: 90° 회전·20Hz 입력·추적 전이, 누락/손상 파일, 빈 frame, quaternion/matrix 불일치, 중복 timestamp, PGM payload 손상, 비유한 JSON, manifest/summary/lifecycle/pose/IMU JSON의 list·scalar·null 입력 거부.
- CLI guard **3개 통과**: overwrite 거부·canonical 경로 거부·missing input 실패. [`cli-guards.json`](../../build/physical-arcore-20261014/analysis/cli-guards.json); 예상 실패 [`expected-missing-input-failure.json`](../../build/physical-arcore-20261014/analysis/expected-missing-input-failure.json) 보존.
- 한계: 단일 기기·단일 짧은 동작, GT 없음, 열·장시간·다기기·고속 회전·암부·가림 이후 재지역화 미검증. 실측 첫 5초/마지막 5초의 실제 정지 여부는 사용자 지시만으로 확정하지 않음.

- Cleanup pass 2: 유효 JSON `[]` row에서 `AttributeError`와 0-byte 출력이 재현됨. JSON 객체 타입 검증 추가; 기존 실패 자료 보존, 수정 후 [FAIL JSON](../../build/physical-arcore-20261014/analysis/pass2-malformed-fixed.json)·exit 1 확인. [회귀 red](../../build/physical-arcore-20261014/analysis/analyzer-pass2-red.log) → [8개 green](../../build/physical-arcore-20261014/analysis/analyzer-pass2-green.log), 3개 capture 재분석 PASS.
