# 기록 기반 복구: 성능·프레임 분석

- 날짜: 2026-10-12, Asia/Seoul
- 상태: **전화 numeric 분석·동일adapter A0 재빌드·numeric parity 완료. A2 목표 fixture가 GT/coverage gate 실패하여 ABBA 보류; 검증된 후속 후보 대기**
- 계약: [개발 계획](2026-10-12-recorded-recovery-plan.md), [시험 명세 T5](2026-10-12-recorded-recovery-test-spec.md)
- 제품 변경: `tests/audit_dataset_replay.cpp`의 additive 측정 adapter만 수정. estimator·tracker·입력·solver·threshold 변경 없음
- 증거: [`build/recovery-20261012/performance/`](../../build/recovery-20261012/performance/)

## 최신 전화: 동일 시간창607행

- 원본: `build/dev-android/snapshot-2026-10-12T11-59-15-089Z.json`, SHA256 `11f0bba39fe08871a3bd9c26ab3c72df406f9c330e7a9e3e5fcfa91904ade07d`
- 원본609수신 결과에서 `startedAtMs <= inputTimestamp*1000 < stoppedAtMs`를 적용한607행. fresh574, Lost33
- 각 영상 timestamp의 이전 결과 timestamp부터 현재 timestamp까지 기록 IMU를 선형 보간; gyro norm 사다리꼴 적분/구간 길이를 `gyroMeanDps`로 정의
- 이는 angular path의 upper-bound 근사이며 net orientation·물리 exposure 정렬·독립 GT가 아님. 처음1행의 IMU 구간 coverage 부족은 null로 보존

| 분류 | 행 수 | gyroMean 중앙값 | frontend p95 | engine p95 | Ceres 표본 수 |
|---|---:|---:|---:|---:|---:|
| 전체 시간창 | 607 | 3.215°/s | 41.5665ms | 69.7665ms | 574 |
| fresh | 574 | 3.057°/s | 29.5355ms | 70.6435ms | 574 |
| Lost | 33 | 96.246°/s | 66.2250ms | 67.5020ms | 0 |
| bias 경고 fresh | 159 | 3.139°/s | 24.8485ms | 68.9215ms | 159 |

- Lost33행 current factor0은29행, factor1은4행. 해당 행 Ceres·marginalization 미실행
- 같은 구간 Pearson: gyroMean↔frontend `r=0.7889`, gyroMean↔current factor `r=-0.8969`, 각각606쌍. 연속 프레임의 자기상관·공통 motion·frame gap 영향이 있어 인과·독립 표본 검정으로 확대 금지
- 23초 이후 gyroMean<10°/s인164fresh행 중 bias 경고93행. 낮은 gyro를 완전 정지 또는 bias 오류 확정으로 해석하지 않음
- 원래609행의 frontend p95 **41.377ms**와 이607행의 **41.5665ms**는 분모가 다름. engine p95와 stage p95를 합산하지 않음
- 최신 numeric에는 영상·feature ID/age·삭제/삼각측량 이유·GT 없음. current factor의 나이는 **UNAVAILABLE**; tracker feature 수나 historical bank 수로 대체하지 않음
- 재현: `python3 build/recovery-20261012/performance/analyze_recorded.py --output NEW_ANALYSIS_DIRECTORY`
- 결과: [프레임 CSV·요약](../../build/recovery-20261012/performance/recorded-analysis-v3/analysis.json)

## 과거300frame prefix 재생

- 기존 replay 결과를 읽어 분석; 이번 lane에서 새 SLAM 실행 없음
- minute1 axis-corrected/current ON:270/300fresh, current 부족1·final-current 부족1, bias 경고26
- minute1 원래 기록/OFF:239/300fresh. 잘못된 gyro axis·다른 rescue 설정의 역사 control이며 동일 실험으로 속도 향상 비교 금지
- minute2 OFF/ON: 각각0/300fresh, initialization timeout1. 성공 coverage0을 정확도 통과로 보고하지 않음
- minute1 frame82는 OLD/current support0, frame98은 NEW/초기 current support 이후 final-current 부족. 두 유형을 하나의 NEW starvation으로 합치지 않음
- actual browser pipeline timing과 native membership를 frame index 및 timestamp 허용오차1e-6s로 연결. native 상세 진단 ON 시간은 controlled diagnostics OFF 성능 비교에 사용하지 않음
- 별도 triangulation 시간은 기존 pipeline API에 없어 **UNAVAILABLE**. backend에서 다른 stage 시간을 빼서 생성하지 않음
- 현재 두 prefix는 최신30초 영상 또는 약41.12초 발산 구간의 완전 replay가 아님. 실측 calibration·physical GT·기록 전 estimator state 부재 유지

## A2 당시 동일 adapter·A0 source 보존 (역사 기록)

- adapter SHA256: `47b832f6180500fe825bae63a6d27274ad1bc05847bd9e22c6074231767a4100`
- `std::chrono::steady_clock::is_steady` static assertion 추가; 기존 `process_ms` stopwatch 종료 뒤 `getPipelineDiagnostics()`를 읽고 `initialized`와 raw `pipeline`을 결과 JSON에 추가
- 고비용 feature/membership/입력 hash 진단은 기존 DIAGNOSTICS 옵션과 분리. 측정 stage의 합을 engine 전체 시간으로 대체하지 않음
- frozen A0 전체 source manifest: `2f143ca42c92c360552897f25ab6293603086f20e0b0f017b9f0fef64e283880`
- A0 source143entry 격리 추출; original manifest86files 확인. adapter를 제외한85개 product source는 원본과 byte 동일
- core manifest(adapter 제외): `6f796acf2caaae1de6a77a2e3470ba20609dbf6f6292154292ae7246a0fe4078`
- 동일 adapter를 반영한 A0 전체 source manifest: `e96cc34f74ff4151012d501aa09185ecb44169ff17e0b2a1cd369d93a5fc352b`
- [baseline source 확인](../../build/recovery-20261012/performance/baseline-source-verification.json), [준비된 configure/build argv](../../build/recovery-20261012/performance/baseline-build-commands.json). **A0 build 완료**, 실제 compile database의 core39records/34unique source를 target별 fullrecord multiset으로 원A0와 경로 정규화 비교하여 차이0. A2 동일 adapter·artifact 검증은 이후 결과로 기록

## A0 계측 parity 결과

- 원A0 SHA `43418010831c8b8747bf1e5c25ca88bc9815eb58132fb396c8449d27159bad7c` → 동일adapter A0 SHA `6720c2e185d0703057e7cbe470f1c3927474c16c9488c9ee705b9c67231f4070`
- configure/build exit0,2jobs. source/core/flags 확인: [A0 adapter manifest](../../build/recovery-20261012/performance/baseline-adapter-manifest-v2.json)
- 원본과 새 build의 native Ceres archive SHA `348d012e74faf65d2370616f9b49016540cc89e267528cb5f785dc5a5e5b3418` byte 동일
- clean101 fixed10s/10iter: 양쪽400입력·339fresh pose, status/reason/solver iteration/termination/epoch 차이0, pose matrix 원소 최대차이 **0.0**
- 기존 SE(3) trace-angle 비교의 최대1.71e-6°는 동일 matrix에 대한 부동소수점 roundoff; 원소 차이0을 함께 기록
- 초기 source-key dict 검사는5개 target variant를 덮어쓴 한계가 있어 폐기;39개 fullrecord multiset으로 재검증. 중복 source의 non-last variant flag 변경을 주입한 독립 selfcheck에서 차이 검출 확인
- 실제 stage 표본: engine/frontend/IMU400, backend399+null1, Ceres/marginalization339+null61, triangulation0/UNAVAILABLE
- [numeric parity](../../build/recovery-20261012/performance/adapter-parity-v1/pilot-verification.json), [stage 표본 수](../../build/recovery-20261012/performance/adapter-pipeline-samplecounts.json). 이 pilot의 시간은 후보 성능 비교 결과로 재사용하지 않음

## 고정 성능 절차

- 입력: 기존 clean101 v2·fast8809 각400frame, source-frame100–399 warmup300 별도 보고
- native profile: YAML, solver cap10s/10iterations, cvThreads1/seed0, PnP OFF, rotation rescue ON,120features, LK21/3·min20·iterations20/epsilon0.03; benchmark zero-tolerance OFF
- frozen A0 original binary↔동일adapter A0의 exact input/state/fresh timestamp와 기존 frozen parity pose 계약을 먼저 검사. mobile0.06s profile은 이 fixed-work 성능과 구분
- 각 fixture **A1→B1→B2→A2**, 두 paired controls/총4실행. 다른 build·SLAM replay 동시 실행을 `/proc`에서 검사하고 직렬 실행
- fixture·PGM payload·config·binary·adapter·parity evaluator·플랜 SHA와 확장된 argv·실제 readback·monotonic wall·GNU time RSS 보존
- frame별 full numeric diagnostics·hash I/O OFF. 각 stage 실제 측정 필드·null·sample count를 사용; Ceres not_run0은 null, final-cull 실패의 실제 Ceres 시간은 포함
- marginalization의 null/0/positive 표본을 분리. 별도 호출 flag가 없어0을 정확한 call count나 speed 이득으로 단정하지 않음
- mandatory all-frame paired median engine/frontend/backend p95≤A0×1.05, p99≤A0×1.10. unavailable stage는 gate 통과 아님
- init 전/후·warmup·fresh·Lost 분포와 matched fresh/same solver work 교집합은 보조 보고. 회복으로 늘어난 solver 작업량과 성능 회귀를 구별하되 mandatory gate 유지
- runner: [paired_native.py](../../build/recovery-20261012/performance/paired_native.py). 후보 binary·manifest 확정 뒤 `prepare`, root 실행 창에서만 `run`
- parser 독립 fixture400행: fresh397, 실제 Ceres398, marginalization positive198, 미지원 triangulation0표본; pre-solve0 제외·post-cull Ceres 보존 확인. [selfcheck](../../build/recovery-20261012/performance/parser-selfcheck.json)

## A2 목표 fixture 실패 뒤 읽기 전용 분석

- 대상: 기존 development churn12617,400source frame. 새 replay·benchmark·threshold 변경 없음; [A0/A1/A2 frame 분석](../../build/recovery-20261012/performance/a2-pilot-analysis-v1/summary.json)
- A0/A1:324fresh, warmup265/300(88.333%), global SE(3) scale1 ATE49.8351m/rotationAPE123.3536°. 두 실행의400행 상태·pose matrix exact 동일
- A2:274fresh, warmup215/300(71.667%), ATE0.317694m/rotationAPE13.2651°. coverage·translation·rotation·RPE gate 실패; 물리 정확도·near-zero Lost·성능 성공으로 채택 금지
- A2 first pose 차이: firstfresh frame41/2.05s, 원소 최대차이1.26549e-4. first solver state 차이:frame44/2.2s. 차이 발생과 불안정 확정을 구분
- 첫 factor-loss: A1 frame127–133(7행), A2 frame126–138(13행). frame125까지A2 prior available=true·solverusable=true이며 current2/invalidDepth94, A1은current6/invalid21
- A2 frame126 current1→Lost→prior release, frame127부터 incoming prior 없음. **factor 저하가 최초 prior release보다 먼저 관측**; release가 최초 원인이라는 증거 없음
- frame125 A2 member153개=성숙 valid59+성숙 unknown73+young unknown21; A1은성숙 valid132+young unknown21. [membership 분해](../../build/recovery-20261012/performance/a2-pilot-analysis-v1/membership-categories.json)
- frame138 A2 mature unknown63중 current observation24개·valid current0; frame140 mature unknown121중 current observation86개·valid current10. age8/9만이 아닌 성숙 deferred의 재삼각측량·기하 재평가 경로 확인 대상
- 회복 뒤 adjacent camera-relative GT RPE:139→140은0.14737m/7.25848°,140→141은0.17150m/6.17721°. 기존 global gate를 대체하지 않는 위치 확인용 offline 진단; GT는 estimator에 전달하지 않음
- 진단은frame90–329만 상세 기록되어 나머지 membership/prior 필드는 unavailable. A0에는pipeline이 없으므로A1값 복사·stage 역산 없음
- A2의 error 감소와 coverage 감소를 함께 보고. 미수락 후보의 diagnostics ON 시간을 처리 속도 개선으로 보고하지 않음; **ABBA 미실행 유지**

## 남은 검증

- A2 동일 adapter artifact·source manifest와 controlled ABBA gates
- 실제 phone 신규 기록·thermal·실측 calibration·independent GT. 현재 문서의 시간 상관을 모바일 latency 개선 또는 물리 정확도 완료로 보고하지 않음

## A3 Limited adapter 준비 (아직 재빌드·parity 미검증)

- 현재 adapter SHA256: `b24df8e5b686fa9d0dbbd1622c58d3d484773da15a49a29fb83a363732e97ee0`; 위 A2 당시 `47b832...` binary/parity는 최신 adapter 증거로 재사용하지 않음
- A0에 없는 새 enum/API를 참조하지 않고 numeric5=`tracking_limited`,6status slots를 예약. 원A0 core에서도 동일 adapter 컴파일 가능하도록 기존 getter만 사용
- Limited의 실제16원소 `T_W_C`를 normal pose와 함께 trajectory에 보존; 전체 pose/GT 분모 유지. normal·limited count/시간은 추가로 분리
- `initialized=engine.isInitialized()`, `poseFresh=engine.isPoseFresh()`, `engineFrameTimestamp=engine.getFrameTimestamp()`, `imuEndpointTimestamp=engine.getIMUEndpointTimestamp()` 직접 기록. 모든 getter는 processframe stopwatch 종료 이후
- pipeline v1/v2의 rawJSON을 보존; A0에 없는 quality/covariance/provisional 정보는0으로 만들지 않음
- 격리 A0의 adapter 제외85 product files 불변; core SHA `6f796acf2caaae1de6a77a2e3470ba20609dbf6f6292154292ae7246a0fe4078`, 새 전체 source SHA `0917129ac50b3ef4829e52cafec4fee90368f29fb9d62cabe170e0dd19d58b17`
- 남은 gate: root compile window → 원A0↔새SAMEadapter A0 fixed-work numeric parity 재실행 → A3 후보 GT/fullreview 통과 → quiet serialABBA2pairs/fixture. 상업 reference runtime 실행과 engine 성능 비교를 섞지 않음
