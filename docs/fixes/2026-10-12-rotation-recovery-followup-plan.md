# 약한 parallax의 projective 회복 후속 개발 계획

- 날짜: 2026-10-12, Asia/Seoul
- 상태: Sol Ultra 설계안. Astra Ultra **개발 계획 리뷰 전**; 제품 구현 금지
- 순서: 기존 연구·GT counterfactual → 본 계획/[시험 명세](2026-10-12-rotation-recovery-followup-test-spec.md) 동결 → Astra 계획 리뷰 → Sol 개발·테스트·수정·구현 리뷰·문서
- 모델: 연구·분석·설계·개발·검증·구현/결과 리뷰 GPT-6.1 Sol Ultra; 개발 계획 리뷰만 GPT-6 Astra Ultra
- 목표: depth가 불확실한 올바른 2D 관측을 일반 projective 제약으로 사용하고, metric landmark와 구분하여 회복. 고정 churn GT/coverage 실패를 기준 완화 없이 해결
- 별도 범위: 기존 승인 계획의 성숙 track defer 실험과 다른 **새 A3 projective 모델**. A0/A1/A2 source·실패·원본·서비스·TLS·기존 미커밋 작업 보존, dependency 추가0

## 선택 근거·기준

- 기준은 HEAD가 아닌 dirty A0. A1은 공통 slot/canonical ray/재앵커/unknown apply 수정과 기존 age 정책 보존; frozen source `2abf20b2…b5ce`, native `ede92663…f6aa`
- A0/A1 고정 churn12617:400frame/fresh324, warmup265/300(88.33%), ATE49.835m/APE123.354°, 35factor실패/7구간.400개 상태·pose 원소 차이0
- A2 frozen core `86bde6fc…b79e7`, native `ed3a9c7a…fd5a4`:fresh274, warmup215/300(71.67%), ATE0.31769m/APE13.265°로 **기각**. 기존 계획/실패 증거 덮어쓰기 금지
- A2 rank 거부3543 중GT rotation에서도3505(98.93%) 거부: 실제 weak parallax. frame125 mature unknown73; frame138 current unknown24/valid0. age7 이하도 보류하는 구현은 승인 정책과 일치
- frame300:119LK→119F→114mask, GT-visible112 중111개≤1.5px, current3D0. descriptor 추가보다 depth를 요구하지 않는 관측 모델을 우선
- frame127: F에서 제외된 GT-visible 대응 및 잔존 대응이약51px 오대응. 이 구간의 정당한 Lost를 새 상태 이름·threshold 완화로 숨기지 않음
- 자료: [후속 1차 연구](../research/2026-10-12-rotation-recovery-followup.md), [기록 분석](../research/2026-10-12-recorded-diagnosis.md), [구현 리뷰](2026-10-12-recorded-recovery-implementation-review.md)

## 최소 선택·제외

- 선택: **A1 정상 경로 + 같은 frame의 조건부 일반 projective solve**. 기존 ProjectionFactor의 free nuisance inverse depthρ를 사용; 새 flow/descriptor·solver 교체·RD-VIO 전체 구조 이식 없음
- 건강한 A1 경로는 추가 provisional factor 없이 같은 순서·설정으로 실행. A1 pre-solve current 부족 또는 final-current 실패 시에만 fallback 검토; 새 frame/시각·epoch를 만들지 않음
- 첫 실패 시 recovery mode에 진입하여 현재 unknown과 mode 중 생성된 새 track을 persistently provisional로 표시; age7 이하가 되어도 metric 인증 우회0. mode 종료 전 기존 legacy finite current≥2도 κ/fit/innovation을 재검증하여 잘못된 normal 복귀 방지; 건강한 mode 밖에는 추가 gate0
- A2의 모든 young/deferred를 영구적으로 엄격한 metric bank로 바꾸는 정책을 기본 경로에 이식하지 않음. 새 provisional 기원 track의 metric 승격에는 아래 κ/재투영 기준 유지
- 기각: κ10000/1.5px 완화, INIT_DEPTH 대입, finiteT에서ρ0 고정으로 pure rotation 가정, 임의 depth/covariance ridge, stale pose/IMU-only 성공 처리, F gate 완화
- 조건부 후속: 단일 후보의 실제 Jacobian/rank/GT 실패가 확인될 때만 structureless square-root 모델 또는 제한된 mature ID 복구를 별도 계획으로 검토. 현재 코드에 미리 추가하지 않음

## 관측 모델·검증된 경계

- `R_WC=R_WB R_BC`, `C_WC=P_WB+R_WB t_BC`, `h_j=R_Cj_Ci f_i+ρ_i t_Cj_Ci`, `r_j=√Ω(π(h_j)−f_j.xy)`; canonical `f.z=1`
- ρ0에서 projection은 유한, `J_translation=0`, `Jρ=Dπ(h)t`. T0 또는 t∥bearing이면 Jρ0; finiteT에서는ρ를 free로 최적화하여 실제 parallax를 설명
- [Civera TRO2008](https://webdiis.unizar.es/~jcivera/papers/civera_etal_tro08.pdf), [Montiel RSS2006](https://www.roboticsproceedings.org/rss02/p11.pdf), pinned Basalt는 inverse-depth/homogeneous 표현 근거. 해당 논문의 전체6D state·covariance나 Basalt의 방향2+inverse-range3D state를 구현했다고 주장하지 않음
- pinned Ceres2.2.0/DENSE_SCHUR/DOGLEG12-case probe: pureT0 freeρ0 정상, weak finiteT freeρ orientation 오차≈2e-9°;ρ0 고정은1.00855° bias. numerical feasibility 증거이며 full VIO 정확도 증거 아님
- nearT1e-12에서 freeρ는[-119,390]으로 임의 값. 이를 이어 쓰면 translation Jacobian norm236196·prior block norm1.389e10. 양수·finite·Ceres usable만으로 depth/translation 정보 인증 금지
- DOGLEG의 내부 D는 step regularization. 앱 ridge 추가0, D를 observation/covariance/permanent prior에 전달0. signedρ trial은 허용하되 최종 물리 support와 별도

## slot·상태·commit 계약

- 공통 구조 slot·1000cap prefix·ID 순서 유지. solve 시작에 `metric`, `provisional`, `excluded` admission mask를 고정하고 factor/final/apply/marginalization까지 유지; list reorder/중간 erase0
- `estimated_depth/solve_flag1`은 metric 인증만 표현. nuisanceρ는 optimizer local buffer에만 존재하고, unknown은 매 solve 시작ρ0. audit의 last-fitρ는 solver 입력/다음 frame/prior/map에 재사용하지 않음
- actual Ceres current residual을 `currentMetricFactors/currentProjectiveFactors`로 분리. final signedρ<0·pole/nonfinite·뒤쪽 bearing·재투영 불일치·robust reject는 current support에서 제외
- metric 승격: finiteρ>기존1e-10, source/모든 view cheirality, κ≤10000, 모든 view normalized pixel-equivalent L2≤1.5px, usable/final-current 검사 후만. fx/fy는 실제 configure 값; fisheye의 실제 distorted pixel 오차와 구분
- metric map·getSolvedFeatures/PnP·export는 인증된 positive depth만 사용.ρ0·uncertified 양수ρ·negative trial·audit 값을 metric point로 만들지 않음
- fallback commit은 actual IMU factor10/rejected0·endpoint coverage·SPD/finite 및 실제 현재 visual support·아래 rank/consistency/품질 gate를 모두 요구. 실패하면 기존 Lost·prior release·window advance 유지
- 최종 projective support는 현재 view와해당factor에사용한모든view의raw pixel-equivalent L2≤1.5px, forwardh.z>기존1e-10을요구. Cauchy는binary reject가아님; 이gate를통과한row만current/rank에포함하고기존Cauchy IRLS weight로가중, covariance/rank에rawρ0 Jacobian사용
- 정상 retry와 fallback의 **합계** iteration≤10·Ceres 시간≤기존 profile cap(모바일0.06s). rollback된 동일 state·입력에서 남은 budget만 사용; 두 번의 별도0.06s 허용 금지
- incoming prior는 window가 이동하기 전 같은 frame의 retry에서만 유효. 최종 실패 뒤 기존 prior release 유지; 이동한 slot의 옛 prior 보존0

## prior·anchor·관측 이력

- uncertifiedρ는 prior 생성 **전 다시ρ0**에서 raw residual/Jacobian을 평가. 최적화된 임의ρ의 큰 Jtranslation을 영구 prior로 가져오지 않음. 이는 명시된 국소 linearization 정책이며 전역 FEJ 정확성 정리로 과장하지 않음
- dropped nuisanceρ는 기존 raw QR의 rank-aware nullspace로 제거: `P=I−Jρ Jρ⁺`; zeroJρ이면P=I, nonzero two-view이면1차원 nuisance 제거. DOGLEG D·가짜epsilon·역행렬 강제0
- 5-case raw QR/COD probe 차이1.2–3.8e-16;ρ0 unknown prior의 translation block0. finiteT에서 parallax nuisance를 제거한 결과와 fixedρ0 회전 factor는 다름
- prior에 포함된 **provisional 기원 track의 관측은 소비 표시 후 은퇴**하여 future raw factor와 이중 사용하지 않음. prior 생성 실패이면 소비하지 않음. 건강한 기존 A1 metric track 이력 정책은 변경하지 않음
- OLD/NEW outgoing prior에는 provisionalρ block 잔존0. 주소·ID·drop/remap 검사 필수. ρ block 보존이 필요해지면 본 후보 중단 후 별도 reparameterization 계획
- uncertified localρ는 window 이동 전0/unknown으로 폐기. OLD/NEW의 새 anchor는 남은 미소비 관측으로 구성; 음수 depth placeholder를 world point로 변환하지 않음
- homogeneous 검증식: `h_new=R_newᵀ[R_old f_old+ρ_old(C_old−C_new)]`, `ρ_new=ρ_old/h_new.z`, `f_new=h_new/h_new.z`;ρ0에서1/ρ 사용0. metric 인증 이후만 기존 opticalZ reanchor 사용

## 시각·관성 품질 gate와 출력

- 기존 LK/F/rescue 기준 유지. 현재 bearing은 기존 ID correspondence에서만 구성; gyro 예측으로 관측을 만들어 넣지 않음
- pre-solve고정bias의actual IMU prediction과후보body상대R에대해 `e=Log(ΔR_IMUᵀ R_prevᵀ R_candidate)`, `rotationConsistencyScore=eᵀ(Q_IMU,R+H_visual,R⁻¹)⁻¹e≤16.2663`. H_visual,R은current/previous상대회전의canonicalρ0 rawvisual정보에서nuisance/gauge제거후계산; rank3/SPD아니면실패. 두추정량은상관되어있으며합행렬은**명시적weight**, innovation covariance/NIS/χ²확률이아님
- uncertainty는**마지막검증metric시각m부터현재k까지누적한IMU/model propagation-only**. immutable anchor mean/Pmm·currentPkk·crossPkm·nullbasis를보존; `Pkk←F Pkk Fᵀ+Q`, `Pkm←F Pkm`, `Σrelative=L [[Pkk,Pkm],[Pkmᵀ,Pmm]] Lᵀ`, L은T_Bm_Bk의relativeposition/rotation Jacobian. 실제preintegration F/Q·단위·cross항사용
- seed는첫rescue진입시lastmetric까지의실제normal raw graph/prior에서재구성·source기록. gaugefix를물리certainty로seed0하지않음; pseudoinverse는observable subspace만사용하고targetL이남은nullspace를소거하지못하면UNAVAILABLE. projectiveρ/D/correlatedvisualupdate로propagationcov축소0
- Lost·fallback실패/재진입·OLD/NEW reference marginalization은anchor/timer/crosscov초기화0. 마지막metric이후첫Lost의경과도포함. 보존된anchor는uncertainty기준이며옛slotprior를optimizer에재삽입하지않음
- fallback은현재visualorientation rank3·실제finalcurrentprojective support≥2를모두요구; rank tolerance는machineepsilon×matrixdimension×σmax로동결. metric최소2guard를silently재정의하지않고별도branch로기록
- full metric support 회복 전 `tracking_limited` 상태: current timestamp의 실제 갱신 state, rotation visual support와 IMU translation support·conditional covariance·지속 시간·factor 수를 명시. 기존 정상 Tracking과 구분
- 초기후보제한: lastverifiedmetric부터limited/Lost합계≤2.0s, position `3√λmax(Σp)≤0.25m`, rotation `3√λmax(Σθ)≤5π/180rad`. 이는IMU/model개발budget이며실제오차상한/전화confidence보증아님. metric인증current≥2+정상guard회복때만종료; fresh/gyro-only로재시작0
- 제한 초과·covariance unavailable·innovation 실패이면 Lost. 현재 회전만 디버그로 보존할 수 있어도 stale/full pose를 성공으로 재발행하지 않음
- 평가기는 normal+limited를 모두 같은 full-body ATE/APE/RPE·400/300 분모에 포함. limited 출력 제외·rotation-only 평가만으로 합격시키지 않음
- 진단 호환: A1/A2 eligibleFeatures는 structural slot 수이고 invalidDepth에는 inactive도 포함. 새 schema version과 명시된 admission/count 필드로 비교; historical count를 current support로 대체0

## 구현·소유·실험 순서

| 단계 | Sol Ultra 소유 범위 | 완료 근거 |
|---|---|---|
| P0 | 본2문서·Astra 계획 리뷰 | scope·rank/prior·품질·state 경계 해결 |
| T0 | backend lane: projection/numerical/engine 계약 tests·격리 math probe | full T/Bias free·prior·negative·wrong correspondence red 확보 |
| D0 | 같은 backend lane: feature_manager.h/.cpp, optimizer.h/.cpp, estimator.cpp, initializer.cpp 필요한 범위 | A1 normal 동등·세 mask·localρ·metric promotion/소비 이력 |
| D1 | 같은 lane: solver_diagnostics.h, vio_engine.h/.cpp, wasm binding 필요한 범위 | limited/current schema·finite covariance/innovation·budget·epoch 계약 |
| D2 | 독립 web lane: worker/wrapper/app·관련 browser tests | 새 상태를 명시·timestamp/freshness·quality·모든 출력 평가 |
| V0 | data/performance lane: 기존 격리 replay/GT/actual stage harness | 고정 target·TUM·공개/새 heldout·actual stage gates |
| R0 | 독립 Sol 구현·수학·결과 리뷰; root 통합/빌드/승격 | gate 전부·한계·source/artifact/served hash |

- 다른 lane 변경 되돌리기0. 기본 build jobs 총2, benchmark 중 build/replay 동시 실행0, dataset read-only
- native VIOSystem·common result와replay status consumer도limited를명시지원; D1backend가필요한common_types/vio_system을소유하고performance lane는audit_dataset_replay adapter만소유. schema변경adapter는A0/A3양쪽동일SHA+새parity부터검증; unknownstatus를normal로강등0
- 후보 baseline A1, A2 기각값은 별도 ablation. A3 freeρ+canonicalρ0 prior/소비정책 vs **기각 control** fixedρ0 finiteT·unknownρ carry 비교. 정상A1와 fallback 동작을 분리
- 선택적 K ablation: 성공한 projective rescue가 currentmetric<2이고 selector가NEW이면 OLD로 관측 이력 보존. true finiteT에서5–10view 누적이κ기준을 넘는지 먼저 확인; 기본A3가 duration/depth회복에 실패할 때만 시험. healthy selector 변경0, requested/effective margin을한commit결과로공유하여 priorOLD/slideNEW 불일치0
- 고정 churn→unoccluded base/phone prefix→full TUM·기존 fast101/707/2203/8809→source 동결→신규 heldout. 시험 결과 보고 임계값·입력·warmup·GT association 변경0
- 성능: 같은 adapter/source/flags/입력의 serial ABBA, 실제 frontend/backend/engine p95≤A0×1.05·p99≤A0×1.10. 추가 LK trial/새 frontend cost 없음; 속도 향상은 실측 전 주장0
- 합격: 기존모든GTcap, churn warmup≥280/300(93.3333%, A0+5pp)·actualcurrent-support실패와Lost구간수각각A0보다감소·reset0, translationATE≤0.25m/rotationAPE≤5°/기존RPE, phoneprefix무회귀·전체검증. 80%절대cap/265개무회귀만으로합격0;93.33%를near-zero Lost로부르지않음
- 중단: free T/Bias probe에서 false translation/bias confidence, arbitraryρ prior정보, nullspace/gauge 손상, 관측 이중 사용, quality 상태 숨김, GT/coverage/속도 gate 실패. frozen 실패 보존 후 새 계획; cap 완화로 수리0
- 승격은 통과 후 fresh artifact/manifest/실제 TLS GET 연결·원자적 backup/rollback. 물리 Flip6·실측 calibration·fieldGT·열은 별도 미검증
