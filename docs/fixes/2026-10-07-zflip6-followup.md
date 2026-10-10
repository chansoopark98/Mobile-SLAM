# Z Flip 6 보존 기록 후속 진단·수정

- 날짜: 2026-10-07, Asia/Seoul
- **현재 판정: 초기화·기록 결함 수정과 solver 진단 보강 완료. 실기기 발산은 미해결**
- 적용 범위: 소스·개발 도구·격리 native/WASM. 운영 HTTPS7002와 전화에는 새 candidate 미배포·추가 측정 없음
- 보존: 기존 미커밋 변경, PC 보존본663파일, grayscale600장, dataset·TLS·서비스. 전체 archive/hash 재검증 불일치0
- 기록 배경: [실기기 인계](../recordings/2026-10-07-zflip6.md)

## 확인한 원인과 최종 조치

| 확인 사항 | 근거 | 반영 상태 |
|---|---|---|
| 실패한 초기화가 gyro bias와 preintegration을 변경 | 독립 정지+0.2rad/s bias 입력에서 alignment=false인데 Bg=0.1999966. 다음 interval 기준 불일치로0.2181812 추정 | 후보 복사본에서 계산, 공통 bias 기준 재적분, gyro 보정 후 기존0.25 excitation 검사, 정렬 성공 시에만 반영 |
| 첫300장 제한으로 후반 문제 영상 누락 | 2차 마지막 영상18.775790초, 최초fresh pose41.116735초. 193개pose 대응영상0개 | bounded queue→PC 연속저장. 이미지·timestamp/hash receipt 동기화 후 ACK, 최종record 동기화 후 phone 복구 객체 해제 |
| 실제 최적화 보정이 장기간 중단 | 1차 native253solve 중194회에서 실제trial10개 모두 실패. 선택 계측에서 finite inverse-depth<=1e-10 거부2,139회, 최솟값-45.05749; other0·target0 | `solver_candidate_evaluation_failed` 품질 경고와 real/accepted/domain-failure trial 수 기록. 기존 유효 수치해·prior 유지 |

- **Solver 경고는 부정확한 포즈의 충분조건이 아님**: calibrated room1에서도433행에서 같은 경고 발생, 전체 궤적 정확도 유지
- 계측 한계: 선택된 inverse-depth/quaternion 및 target-depth guard만 분류. 그 전후 모든 비유한 입력·residual·Jacobian 거부 총계는 미계측; 같은 trial의 중복 원인 배제 불가
- 1차 발산 경로의 관측 근거이지 모든 물리 원인의 확정 근거는 아님. K·왜곡·camera/body 외부정합·EIS/OIS·물리 exposure/IMU 동기 미검증
- 영상 누락 때문에 2차 후반 발산의 동일입력 재생 불가. 기존 기록을 수정하거나 빠진 영상을 복원한 것으로 취급하지 않음

## 측정 기록 재해석

| 지표 | 1차 | 2차 |
|---|---:|---:|
| accepted frame |558|847|
| 새 capture와 일치하는 result |557|846|
| 이전 lifecycle result |1|0|
| 종료 시 in-flight result |1|1|
| fresh pose |384|193|
| 저장 영상과 겹치는 fresh pose |186|0|
| 저장 영상 source span |32.067780초|18.733525초|

- 2차10–30초 acceleration964행 모두 `[0.2,-9.8,0]m/s²`, gyro p95≈0.001745rad/s
- 15.078085초·30.180865초 초기화timeout: 거의 정지한 입력과 일치. 초기화 결함으로 단정 제외
- gyro>0.05rad/s 첫 시각38.502510초; 마지막50–60초 gyro중앙값0.399883rad/s. 화면 지시 구간을 실제 정지 GT로 사용 불가
- [독립 분석·명령](../../build/zflip6-followup/record-analysis/report.md), [수치 JSON](../../build/zflip6-followup/record-analysis/record-analysis.json)

## 최종 검증

| 검증 | 결과·한계 |
|---|---|
| Native |10 CTest suites·115/115 test PASS·skip0. Numerical47개 포함 |
| 독립 회귀 |초기화3개·실제Ceres termination1개 RED→GREEN. 정상 수렴·예산 종료·기존 genuine unusable rollback 보존 |
| JS |148/148 PASS·skip/TODO0. 새 recorder13개·replay9개 포함 |
| Python exporter |독립 analytic3/3 PASS |
| Syntax·정적 확인 |JS/Python/shell parse·`git diff --check` 통과. genuine clangd 영향 source error0; unused-include warning1. ESLint/TypeScript/clang-tidy 전체 통과 주장 제외 |
| WASM |격리 최신빌드·실제 Chrome API/reset smoke PASS; source/dependency/artifact hash 기록 |
| Worker |보존1차/2차 각300/300 결과, transport loss·missing bracket0. 원래 phone scheduling 재현과 구분 |
| Calibrated browser |요청150·실제150frame·105pose·초기화45. 기존151frame 실행과 공통150frame의 pose 배열 최대차0; pause 시점의 기존 harness 1frame 차이 구분 |
| Calibrated full native |room1 2,821frame·2,776pose·초기화45·engineEpoch1·tracking 이탈0. baseline과 pose 배열 최대차0 |

- Room1 independent GT: camera→body, one-to-one timestamp, SE(3) scale=1
- GT 대응 **2719개**, 위치ATE RMSE **0.05316804m**, 회전APE RMSE **1.203101°**
- 최종 source의 전체room1 결과는 baseline과 동일. 실제 모바일·장기 온도·실측 정확도 검증 대체 근거 제외
- [전체room1 비교](../../build/zflip6-followup/room1-accepted-comparison.json), [GT 지표](../../build/zflip6-followup/room1-accepted-metrics.json)
- [최종 검증 집계](../../build/zflip6-followup/final-verification.json)

### 같은 보존 입력의 최종 결과

| 실행 | 기존 | 최종 | 해석 |
|---|---:|---:|---|
| 1차 native pose / 최대norm |253/300 ·107.283930m|동일|trajectory 변화0, warning194행 |
| 1차 Worker pose / 최대norm |247/300 ·245.772364m|동일|trajectory 변화0, warning171행 |
| 1차 Worker guard reset |1|1|발산 해결 근거 없음 |
| 2차 Worker pose / timeout |0/300 ·1|동일|저장된 초기 정지 구간만 재생 |

- Native tracking min_dist20·solver10s와 실제 mobile Worker min_dist30·solver0.06s·5m guard의 조건 차이 유지. native와 Worker 수치 직접 동일성 비교 제외
- 보존1차 gyro만 고정Chrome154 축 순서로 변환한 derived 입력 사용; 영상·시각·acceleration 불변, 원본 보존
- 2차 재생은 원본 입력 사용. 원래 live IMU 도착순서·시각이 없어 future bracket을 미리 공급하는 desktop 순차 재생
- [Native 비교](../../build/zflip6-followup/native-minute1-comparison.json), [Worker 최종 비교](../../build/zflip6-followup/browser-accepted-comparison.json)

## 제외한 후보

| 후보 | 관측 결과 | 결정 |
|---|---|---|
| Ceres inverse-depth lower bound |phone253→117pose·최대107.28→325.81m, room1 prefix255→176pose |제품 미반영 |
| 갱신실패→unusable→prior폐기 |full room1 2776→2312pose, ATE0.05317→0.14347m, tracking 이탈208회. engineEpoch는1 유지 |철회. warning만 공개하고 기존 수치 상태·prior 유지 |
| log inverse-depth manifold |독립209checks PASS이나 phone24pose·최대1227.895m, room1 prefix7pose |제품 미반영 |

- 실패 후보의 낮은 공개norm·낮은coverage를 정확도 개선으로 보고하지 않음
- `browser-minute*-final`, `native-room1-full`, hash`4c0e03…`는 **철회 후보**. 채택 결과는 `*-accepted` 경로
- [후보·실패 근거](../../build/zflip6-followup/estimator-analysis/result.md), [변경 전 계획·결정 기록](../../build/zflip6-followup/plans-and-decisions-before-final.md)

## 변경 파일·단순화

- `src/frontend/initialization/initial_alignment.cpp`, 관련header: 계산 후보·실패 상태 보존·공통bias 기준·보정 후 excitation
- `src/frontend/initialization/initializer.cpp`, 관련header: 중복 excitation wrapper와 보정 전 gate 제거
- `src/backend/optimizer.cpp`, 관련header: 기존 수치 흐름 유지, 실패trial 진단·실제Ceres test seam
- `tests/test_numerical_contracts.cpp`: 독립 물리 입력·실제Ceres 회귀
- API 변화: `VisualIMUAlignment`의 map 인자를 mutable reference로 명시. 저장소 호출부 검증 완료; 외부 const-map 호출부는 조정 필요
- `scripts/dev/android-input-record.mjs`, `tests/browser/android-input-record.test.mjs`: 연속저장·bounded 메모리·ACK/복구·partial 명시
- `scripts/dev/replay-android-input.mjs`, `tests/browser/android-input-replay.test.mjs`: PGM/hash·원본double시각·연속prefix·IMU bracket 검사
- `scripts/dev/check.sh`: browser-contracts가 모든 `*.test.mjs` 실행. 이전2파일 한정 목록 제거
- `scripts/dev/README.md`, 이 문서, 기존 인계문서: 재현·현재 판정 연결
- 추가 제품/개발 dependency0, 새commit0, dataset·운영 서비스 변경0

## 재현·빌드·배포 구분

```bash
bash scripts/dev/check.sh browser-contracts
bash scripts/dev/check.sh syntax
ctest --test-dir build/zflip6-followup-native --output-on-failure --parallel 2
node scripts/dev/replay-android-input.mjs \
  --record build/zflip6-followup/minute1-axis-fixed/record.json \
  --input-dir build/dev-android/controlled-minute-1/mav0/cam0/data \
  --loader build/zflip6-followup-wasm/vio_engine.js \
  --output build/zflip6-followup/new-phone-replay
```

- 최종 WASM SHA-256: `1099e09da3f298799b1aa16194f301c31da6975782446902497fe4bc3fd22cb5`
- [WASM provenance](../../build/zflip6-followup/wasm-provenance-accepted.json), [native provenance](../../build/zflip6-followup/native-provenance-accepted.json)
- 시작source1197파일 fingerprint·미커밋diff 보존. native baseline은 시작 tracked source119파일을 별도복원·hash확인 후 fresh compile
- 기존 기록의 native 최대89.82m는 이번동결 baseline에서 재현되지 않아 비교 기준에서 제외; 새 baseline107.283930m 사용
- 운영7002: 확인시 PID1636898, 기존 deployment`mobile-divergence-v2-20261007`, loader`f480f9…`; 신규candidate 자동승격 없음
- [개발 명령·기록/복구 절차](../../scripts/dev/README.md#grayscaleimu-입력-보존과-오프라인-재생)

## 남은 필수 입력

- 준비된 Z Flip6의 약1분 재측정: 새로운 recorder로 초기화 이후·발산 구간까지 영상/IMU 확보
- `--frames 24`는 총24장 제한이 아닌 phone 대기queue 크기. raw queue≤16MiB, PC로100ms마다 배출; 실제폰 처리량·관측오버헤드 미검증
- 재생 `--frames 2000`, raw prefix≤512MiB. recording partial·replayReady·missing payload·pending result 먼저 확인
- 실측 camera intrinsic/distortion·camera/body extrinsic·EIS/OIS 조건·sensor/image 시각 검증 및 독립 이동거리/GT 필요
- 이번 증거만으로 residual발산의 단일 물리 원인·수정 후 현장 정확도 판정 불가
