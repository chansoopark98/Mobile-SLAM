# Projective 회복 후속 개발 계획 검토

- 날짜: 2026-10-12, Asia/Seoul
- 담당·범위: GPT-6 Astra Ultra, **개발 계획 검토만**. 제품 개발·구현 후 코드/수학/결과 검토는 Sol Ultra 담당
- 최종 판정: **APPROVE — 승인된 순서로 개발 착수 가능**. R1–R5 반영 확인, 미해결 계획 blocker0
- 대상: [후속 개발 계획](2026-10-12-rotation-recovery-followup-plan.md), [후속 시험 명세](2026-10-12-rotation-recovery-followup-test-spec.md)
- 최초 동결 SHA256: plan `1fccbf307dab3eef68abc54de71d3a5e13e4bbb6724094b2e08c4759de132233`, spec `cedfe4ebc5b16c5b17ac81489c5af990a0bde55cd55bd25da8e7d2ebf3e54613`
- 확인 근거: AGENTS.md·기존 승인 계획·A1/A2 결과·후속 1차 연구·고정 T의 projective solver/prior probe·계획에 관련된 optimizer admission/final/prior 경계
- 변경 범위: 본 문서와 `build/recovery-20261012/a3-plan-review/`. 제품·기존 dirty 변경·데이터·서비스·빌드 보존. 새 solver 실행 또는 구현 수락을 수행한 리뷰가 아님

## 1차 수정 요구·해결 대상

| ID | 계획의 결함·영향 | 필요한 최소 수정·시험 |
|---|---|---|
| R1 | plan62의 후보 상대 회전은 IMU·incoming prior를 함께 사용한 solve의 결과. 같은 IMU prediction과 독립이 아니므로 `IMU covariance + visual conditional covariance`를 곧바로 χ²(3) NIS 공분산으로 쓰면 공유 noise·bias·anchor 상관이 누락됨 | 비교하는 두 추정량·접공간·선형화점·noise와 state의 공분산을 고정. 같은 접공간의 차이에서는 `S=Caa+Cbb−Cab−Cba` 또는 해당 혁신 Jacobian의 동등한 joint covariance가 필요. 독립성에 의존하는 다른 innovation을 선택하면 독립성을 증명. 비확률 consistency gate로 선택할 경우 χ²/NIS confidence 주장 없이 계산·수락 기준을 후보 실행 전에 동결. 동일 IMU를 공유하는 합성 control과 wronggyro/wrong prior counterexample 추가 |
| R2 | relative position/rotation 3σ의 기준 pose·covariance 종류·3σ 요약식이 미정. 직전 frame만 비교하거나 window gauge를 hard certainty로 넣으면 누적 drift를 숨길 수 있고, rank deficient normal의 pseudoinverse는 관측 불가 방향에 분산0을 줄 수 있음 | 마지막 검증 metric 시점에 대한 누적 uncertainty 또는 동등하게 누적을 보존하는 명시적 기준 선택. free T/V/Ba/Bg·nuisance를 조건부로 고정한 covariance와 marginal covariance 구분. `3*sqrt(lambda_max(C))` 등의 고정 식·gauge 제거·rank/nullspace 처리 규정. 관측 불가 방향은 unavailable/실패 처리하며 숫자0으로 통과 금지. reference가 window에서 제거되거나 Lost→fallback 재진입해도 uncertainty/timer 초기화0; 실제 metric 복귀 또는 정식 재초기화만 종료 |
| R3 | final projective support의 `재투영 불일치·robust reject`가 수치적으로 정의되지 않았고 기존 Cauchy loss는 binary outlier 제거가 아님. fallback의 실제 accepted IMU guard도 독립적으로 명시되지 않아 current 부족 분기로 invalid IMU가 우회할 여지가 있음 | 모든 사용 view와 actual current row에 적용할 raw normalized pixel-equivalent L2 기준·robust row/weight·rank 계산 집합을 동결. 기존 1.5px 기준 재사용 가능하며 metric κ 기준과 분리. 모든 fallback에서 accepted IMU=10, rejected IMU=0, endpoint/SPD/finite 검사 요구. current2D≥2라도 50px mismatch·IMU9/invalid covariance·stale input이면 거부하는 부정 fixture 명시 |
| R4 | H4는 full free state가 필요하다는 방향만 있고 입력·observable 방향·정량 oracle/tolerance가 미정. 결과를 본 뒤 성공을 정의할 여지가 있으며 pure rotation의 미관측 bias/scale을 GT 값과 일치시키는 잘못된 oracle도 가능 | 후보 실행 전에 analytic excitation→weak/pure rotation fixture, noise·finite prior·초기 오차·free parameter 목록·expected observable/nullspace·각 numeric tolerance를 JSON으로 동결. GT는 입력 생성/평가만 사용. 모든 T/V/Ba/Bg가 실제 free인지 검사; 미관측 방향은 GT 회복이 아닌 정보/rank 보존으로 판정. noise calibration을 주장하면 별도 반복 noise 검증. 실패 fixture·기준 보존 |
| R5 | 기존 승인 명세의 같은 frozen churn은 fresh coverage A0 대비 최소+5pp·actual current 부족과 Lost 구간 감소를 요구. 새 계획의 ≥A0(265/300)는 이 기준을 완화함 | 기존 기준을 warm≥280/300(93.3333%)로 복구하고 actual current-support 부족 frame·Lost 구간 수 감소도 유지. root가 기존 +5pp가 계속 binding임을 확인. 원래80% 절대 cap 또는 A0 무회귀만으로 통과 금지 |

- R1–R4는 이미 선택한 quality gate의 계산·수명·통과 조건을 고정하는 수정. R5는 기존 승인 수락 조건 복구
- κ≤10000·모든 view 1.5px·churn/TUM/fast GT·coverage·성능 cap 완화는 해결책에서 제외

## 수용 가능한 설계

- 최소 A1 healthy path와 조건부 rescue 분리는 A2의 전면 strict bank 회귀를 격리하는 선택. 기존 healthy factor 순서·수치 동등성과 persistent provisional age 우회 방지를 함께 요구
- free nuisance ρ는 finite translation parallax를 설명하며, ρ0 고정 모델의 1.00855° bias control을 유지. 고정 T probe는 full VIO의 translation/bias 안전성을 증명하지 않음
- unknown ρ의 다음 solve/map/PnP 저장 금지와 canonical ρ0 prior는 near-zero T에서 arbitrary ρ가 만드는 translation 정보 반례를 직접 다룸. canonical policy를 전역 FEJ consistency 정리로 부르지 않는 범위도 적절
- raw rank-aware QR nuisance 제거, OLD/NEW outgoing provisional ρ block0, admission mask·1000 slot prefix·주소/epoch 보존, provisional 관측의 prior 소비 후 은퇴 계약 유지
- prior 소비 ledger는 실제 prior에 들어간 residual과 source/target 관측에 연결되어야 함. NEW가 이전 prior만 줄이는 경우 새 관측을 소비하지 않고, prior 생성 실패는 ledger를 commit하지 않는 Q5/Q6 확인 필요
- metric 승격도 provisional 기원 이력 정책을 지우지 않으며, 같은 solve에서 승격/소비/retire가 일어날 때 factor membership과 map publication 순서를 Sol 구현 리뷰에서 확인
- fallback 합계 solver iteration10·mobile Ceres0.06s, same-frame rollback·prior release 계약 유지. `tracking_limited`도 같은 full-body GT/coverage 분모에 포함
- 선택 K는 성공한 rescue에서만 effective OLD를 적용하고 prior/slide가 같은 결정을 사용해야 함. 독립 GT·coverage·속도 gate 실패 시 기각

## 고정 수락·한계

- churn12617 A0/A1: warm265/300, ATE49.835m, rotationAPE123.354°. A2: warm215/300, ATE0.31769m, rotationAPE13.265°의 기각 증거 보존
- A3: churn warm≥280/300·reset0·translationATE≤0.25m·rotationAPE≤5°·기존 RPE cap, actual current-support 부족 frame·Lost 구간 수 감소. normal/limited 전부 같은 평가에 포함
- TUM room1/room4 전체·ON/OFF matched baseline별 translation/rotation RMSE 증가≤5%·coverage 감소≤1pp. 기존 fast seeds와 source/평가기 동결 뒤 신규 확인 seed 구분
- serial ABBA의 actual frontend/backend/engine p95≤A0×1.05·p99≤A0×1.10. 같은 adapter SHA·A0 수치 parity·build provenance 후 측정
- native/WASM·실제 Worker/App·sanitizer·browser·clangd·Sol 독립 구현/결과 검토와 새 artifact/served hash는 승인 이후 실행 gate
- C1 capture는 별도 opt-in, sync 기본 유지. desktop pixel/GT 성공과 latency 실패를 A3 회복 또는 속도 증거로 사용하지 않음
- 실제 Flip6·실측 calibration·field GT·열 검증은 없음. 계획 승인만으로 물리 추적 개선이나 후보 채택을 주장하지 않음

## 재검토 기록

- 최종 검토 revision: plan94행/SHA256 `d0b8d42545e574ed1db0515d11f6c7689dbce57016b851ef808d5cceea1775b2`, spec91행/SHA256 `ac104666e0004fcf4a1ecd0c1b49b1b939c8aab20944b673645333ad7e4d53dc`. Sol의 동결 통지 이후 실제 파일을 다시 읽고 SHA 일치 확인
- R1 해결: `rotationConsistencyScore=eᵀ(Q_IMU,R+H_visual,R⁻¹)⁻¹e≤16.2663`을 확률 해석 없는 개발용 가중 점수로 고정. 두 추정량의 상관을 명시하고 innovation covariance·NIS·χ² confidence 주장을 제거. 실제 raw visual relative rotation rank3·SPD 요구와 e/weight/선형화점/단위 oracle 유지
- R2 해결: 마지막 검증 metric anchor의 mean/Pmm·Pkk·Pkm·nullbasis 보존, 실제 IMU F/Q propagation과 joint relative Jacobian으로 누적 covariance 계산. `3sqrt(lambda_max)`의 위치0.25m·회전5도·총2.0s 동결. gauge fix·pseudoinverse0·Lost/slide 재진입으로 신뢰도를 만들어내는 경로 금지. 실제 오차 상한이라는 주장도 제거
- R3 해결: final projective의 current/모든 사용 view raw pixel-equivalent L2≤1.5px·forward h·Cauchy IRLS row 정책 명시. actual accepted IMU10/rejected0·endpoint·SPD·finite를 fallback 자체에도 요구. 1.4/1.6px·IMU9/10·invalid covariance 부정 fixture 추가
- R4 해결: full free T/V/Ba/Bg fixture의 analytic 운동·유한 prior·초기 오차·noise·단위와 observable 오차 tolerance를 후보 코드 전에 JSON으로 동결. 실제 analytic symmetry/COD로 rank를 정하고 총 null6 같은 일반화는 금지. 미관측 방향은 GT 근접성으로 채점하지 않음
- R5 해결: 같은 frozen churn의 warm≥280/300(A0+5pp)와 actual current-support 실패·Lost 구간 수 감소를 계획·명세 양쪽에 복구. 80% 또는265/300만으로 합격할 수 없고93.33%를 near-zero Lost라고 표현하지 않음
- 최초 plan/spec 스냅샷·REVISE 기록과 최종 검토 manifest: `build/recovery-20261012/a3-plan-review/`. 최종 검토는 문서·기존 증거 확인이며 새 제품 실행 결과 없음

## 개발 인계

- 추가 사용자 확인 없이 승인된 T0→D0/D1/D2→V0→Sol 독립 리뷰를 진행할 수 있음. H4의 fixture/rank/oracle 동결이 제품 코드보다 먼저이며, 실패 결과 뒤 tolerance 수정으로 통과시키지 않음
- 구현 리뷰에서 old graph의 마지막 metric snapshot 확보, reference covariance 접공간·cross term, actual prior residual/관측 소비 ledger의 원자적 commit, same-solve 승격/marginalization 순서를 확인. 위 계약을 구현할 수 없으면 임의 근사로 대체하지 않고 해당 설계만 재검토
- 실패 후보는 보존·기각하며 κ/재투영/GT/coverage/성능 gate를 완화하지 않음. free-ρ 계산 가능성·계획 승인과 full VIO 회복·물리 Flip6 개선·배포 수락은 각각 별도
- 승인 이후 알고리즘·quality 계산·prior 정책·수락 기준·범위가 실질 변경되면 변경된 부분을 새 계획 검토 대상으로 처리
