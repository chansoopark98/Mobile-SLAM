# 기록 기반 회복 구현·수학·결과 독립 리뷰

- 날짜: 2026-10-12, Asia/Seoul
- 담당: GPT-6.1 Sol Ultra. Astra는 승인된 개발 계획 리뷰만 담당
- 상태: frozen A2 수학·코드 검토 완료, **제품 후보 A2 기각**. Target GT/coverage 실패로 WASM·배포 수락 불가; A3 별도 개발 계획 필요
- 기준: [승인 계획](2026-10-12-recorded-recovery-plan.md), [테스트 명세](2026-10-12-recorded-recovery-test-spec.md), [Astra 계획 리뷰](2026-10-12-recorded-recovery-plan-review.md)
- 범위: 제품 read-only; 본 문서와 `build/recovery-20261012/review/`만 write. 다른 lane의 진행 중 파일·기존 변경·원본 데이터·서비스 보존

## 수학·backend 검토표

| 항목 | 독립 검토 내용 | 상태 |
|---|---|---|
| 구조 slot | count/get/set/clear/setup/filter/marginalization의 feature ID·1000cap prefix 동일, erase 시점 보존 | 대기 |
| admission | solve 시작의 mask를 factor/final/current/apply/OLD prior 전체에 사용; finite-positive inactive도 제외 | 대기 |
| deferred | marked9→8→7에서도 geometry 검사, untouched rho0·status 보존, 실제 admission+usable+final 후만 pending 해제 | 대기 |
| canonical ray | ingestion positive scale invariance·z1, pixel/velocity 단위 보존, malformed reject | 대기 |
| young geometry | 3×3 bearing condition≤10000, 모든 view opticalZ 양수·Euclidean 재투영≤1.5px, pure rotation unknown 유지 | 대기 |
| OLD/NEW anchor | old/new pose 덮어쓰기 전 획득, R_WC/T_BC 방향·opticalZ·nonzero extrinsic·ray canonical 계약 | 대기 |
| prior | NEW pose9 drop/10→9, young inverse-depth 잔존0, rejected update prior release, parameter 주소 수명 | 대기 |
| final cull | initial current67→final 부족 같은 실패 차단, slot 고정, invalid landmark만 실패, mask 확대0 | 대기 |
| map/PnP | 미확정·inactive·negative/nonfinite/behind-camera 배제; known solved 순수회전과 unknown depth 분리 | 대기 |
| oracle | G3 analytic eigenvalues·G6 전체 경로·T1 실제 NEW20/20, 구현과 독립 기대값·red→green 근거 | 대기 |

## capture 검토표

- C1 첫 안정 snapshot: Worker `923f3fc5…698c3`, App `173d987f…0f595`, Camera unchanged. before-await capture·bounded ownership·identity/세대 fence·전송 후 drop·sync default 정적 검토에서 blocking finding 없음
- actual desktop Chrome144.0.7559.96:16개 color/number·portrait/landscape·rotation/crop pixel oracle, draw/crash/hang·busy/config/generation/stop 계약 근거 수용. 실제 estimator GT·ABBA 시간·phone target 미검증으로 기본 승격 불가
- 추가 capture 독립 리뷰 P2 두 건 수정 확인: pending을 constructor 전 close하여 peakLive3→2, 재진입 후 기존 inFlight를 지키며 posted `[1,3,2]`→`[1,3]`·peakRemote2→1·staleReply1→0. Worker SHA `6447aff5…df5ad`, independent before/after edge proof 및11개 Node 회귀 수용. 최종 actual-browser refresh·fallback/restart 회귀는 대기
| 항목 | 독립 검토 내용 | 상태 |
|---|---|---|
| frame identity | rVFC await 전 constructor, 해당 VideoFrame pixels만 처리, timestamp provenance 유지 | 대기 |
| ownership | 최대 converting1/pending1, overwrite/error/stop/reconfigure close, detach 이후 sender 접근0 | 대기 |
| lifecycle | generation/config/geometry/seq fence, stale/duplicate0, transferred failure drop·late reread0 | 대기 |
| fallback | readiness/probe 전 sync, 지원 실패 sync, target phone 미검증이면 opt-in/sync 기본 | 대기 |
| pixel oracle | numbered/color/rotation/noncenter crop·Worker 지연 시 전체 gray byte 동일 | 대기 |

## 결과·성능 검토표

- adapter 정적 검토: `47b832f6180500fe825bae63a6d27274ad1bc05847bd9e22c6074231767a4100`의7개 추가행 확인. process 타이머 종료 뒤 getter, 85개 frozen core source SHA+adapter 별도 검증 통과. **빌드·수치 parity·시간 gate는 아직 대기**
- stage semantics: frozen/current SolverDiagnostics의 미실행 Ceres/marginalization은 quiet_NaN→JSON null. 별도 triangulation timer 없음. review의 초기 default0 가정은 source 확인 후 철회; 실제 null/count 기준 적용
- A0 adapter 빌드/pilot: artifact `6720c2e1…1f4070`, core compile34/34 flags 동등·Ceres archive byteSHA 동일.400input/339fresh·state diff0·pose matrix exact0 확인; trace-angle의약1.7e-6°는 동일 matrix roundoff로 별도 표시. ABBA 후보 비교는 아직 대기
- A0: HEAD가 아닌 `build/research-20261012/baseline/`의 미커밋 core·artifact·입력 SHA 대조
- adapter: 동일 adapter SHA의 frozenA0 core/A2 core, fixed-work numeric parity 후 실제 stage getter 획득; 없는 stage는 UNAVAILABLE/null, process_ms 분해 금지
- 통계: 직렬 ABBA 두쌍·동일 mode/budget/threads·init전후·p50/p95/p99·sample count; 상세진단 ON과 timed run 분리
- 회복: phone prefix OLD current 부족·NEW final 부족을 각각 비교, GT 없는 jump 감소를 metric 정확도로 표현 금지
- 독립 GT: 공개 fast seeds·고정 churn12617·정책 동결 후 신규 확인 seed, 전체400/warmup300·body SE3 scale1·rotation/RPE/coverage 보존
- 회귀: full TUM room1/room4 ON/OFF profile별5% RMSE·1pp coverage gate, native/WASM 경로·source hash 분리
- 완료 근거: native 전체 discover/각 assertion·영향 sanitizer·browser TODO/skip0·clangd·fresh WASM/actual Worker·승격/GET hash
- A0 churn12617 독립 재계산: source400·fresh324·warmup265/300(88.33%)·current 부족33/final 부족2·7구간·reset0·timestamp 단조/unique400 확인. 기존 GT evaluator 재실행 exit1, translation ATE·rotation APE·RPE gate 실패 재현; coverage 통과가 정확도 통과를 뜻하지 않음

## findings·판정

- frozen A1 source87개·binary SHA 일치 독립 검증. common structural/capped prefix·canonical ray·old9/new10 pose 사전 보존·unknown masked apply 정적 계약 확인; 기존11+47·새A1계약4 pass, A2기대2 red
- A1의 `prepareOptimizationParameters()`가 marginalization에서 admission mask를 다시 계산함. A2의 pending/finite-positive inactive에서는 승인된 solve mask 수명 보장이 필요하여 구현 lane에 capture-at-setup/reuse 및 OLD prior ID 테스트 요구. A2 완성 checkpoint에서 판정
- A2에서 mask capture를 최초 setup으로 이동하여 위 문제 해결. frozen87source/binary 일치, projection22/22·numerical47/47. pending aging·canonical opticalZ·all-view 기하·final cull/map/PnP·NEW pre-copy pose 정적 검토에서 추가 구현 결함 없음
- P1 추가 혼합 OLD→NEW→OLD 실제 slide·prior·reject test23/23 통과. 초기 cost54.776 실패는 fixture의 IMU 시작 끝점 미설정으로 확인; 실제 processIMU로 seed 후 cost1e-12 기대값 유지. 제품 core/native SHA 불변, 실패·수리 기록 보존
- A2 target:274/400fresh·warmup215/300(71.67%)·ATE0.31769m/APE13.265°. A0/A1 대비 coverage−16.67pp 및 고정 GT cap 실패. 단위 통과·오차 감소를 수락으로 확대하지 않음
- GT counterfactual: rank 거부3543 중3505(98.93%)가 GT 회전에서도 κ>10000. frame300은119LK/119F/114mask·GT-visible112 중111개≤1.5px인데 current3D0. age-pending 보존은 승인 정책대로 동작했으며, weak parallax를 숫자 guard 완화로 해결할 근거 없음
- frame127의 F 거부/잔존 대응은약51px 오대응. 추가 projective 경로에도 실제 시각 제약·gyro/IMU consistency 필요; F threshold 완화·gyro-only 결과 성공 처리 금지
- 고정 churn12617 A1도324/400·warmup265/300·GT 실패. A0/A1의 상태·pose 직접 비교 결과는 `review/a0-a1-churn-comparison.json`; 한 fixture의 동등성은 전체 TUM 검증을 대체하지 않음
- T0 checkpoint: `RecordedRecoveryContracts`6개 기대값 정적 검토 수용 및 frozen source에서6/6 red raw log 확인. 원인: young slot 제외·scale ray map/residual·NEW opticalZ 불일치·unknown 실패삭제·young depth 미활성·aged degenerate 삭제. 기존 projection11/11·numerical47/47 통과, 기존 assertion 변경 없음
- T0 당시5개 주요 제품 파일 SHA가 baseline manifest와 일치함을 독립 확인. 테스트 source SHA `4b1bba53ea621b326a750bfecfd958bd03699e45b91af40d7dce22e94630a078`; 원본 `build/recovery-20261012/t0/` 보존. red→green 검증의 red 근거 수용
- T0 추가 coverage 대기: 현재 deferred test의 회전은 identity이므로 실제 비항등 rotation+extrinsic의 young triangulation, M4/G4·실제 NEW20·G6 Jacobian/final/PnP 전체 경로를 A2에서 확인. 구현 lane에 인계
- 현재 finding 없음은 검토 미실시 상태를 뜻함. 안정화 checkpoint의 source SHA·exact line·재현/테스트 근거를 연결한 뒤 APPROVE/REQUEST CHANGES 판정
- 발견 건은 수학/인덱스·capture/수명·성능/증거로 분리하고, P1/P2/P3·구체 영향·수정 요구·해결 근거 기록
- 물리 Flip6 실시간 정확도·열·실측 calibration·fieldGT는 이번 desktop/합성 리뷰 통과와 별도
