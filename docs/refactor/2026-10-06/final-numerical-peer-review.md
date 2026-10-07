# 최종 수치 경로 독립 검토

- 판정: **APPROVE — 승인된 수치 수정·R3 회귀 범위**. 이번 조회에서 새 P0/P1 blocker 발견 없음; 최종 R5 runtime parity·전체 dataset·현장 정확도 승인과 구분
- 검토자: engine lane owner / Sol6.1 Ultra. 자신이 수정한 engine·tracker·transport를 독립 승인 대상으로 사용하지 않음
- 방식: read-only source·PRD/test spec·기존 실제 로그/manifest 검토. 이번 검토에서 build·test·profiling·서비스·dataset 변경 없음; 이 문서만 작성
- 근거: `.omx/plans/{prd,test-spec}-mobile-slam-refactor-7002.md`, `docs/refactor/2026-10-06/numerical-lane.md`, 수치 source/header·`tests/test_numerical_contracts.cpp`

## 확인 결과

| 경계 | 확인 내용 | 판정·근거 |
|---|---|---|
| IMU interval·covariance | positive dt·sample buffer 길이·sum_dt coverage·finite state 검사; 0/1 sample·rank deficiency·nonfinite/asymmetric covariance 거부 | `IntegrationBase::hasUsableInterval`, `covariance_whitening.h`; rounding-sized LLT pivot 거부, jitter·covariance inverse 없음 |
| Whitening | `Σ=LLᵀ`, `W=L⁻¹` triangular solve; factor constructor에서 immutable W 계산, residual/Jacobian 동일 whitening | `IMUFactor`·`IMUFactorPnP`; independent Mahalanobis regression으로 `rᵀΣ⁻¹r` 보존 확인 |
| Solver usability | 필요한 IMU interval10개·현재 visual residual·finite parameter·unit quaternion 확인; usable summary·finite initial/final cost 필요 | invalid coverage/solve에서 원래 state 적용·새 prior 생성 제외; usable `NO_CONVERGENCE` budget 종료 유지 |
| Prior·reset | rejection 때 stale prior/주소 해제, reset에서 prior/loss/window/feature/image/IMU state 초기화·generation 증가 | `Optimizer::reset/releaseMarginalizationPrior`, `Estimator::clearState`; usable prior 생성→reset 제거·unusable rollback 회귀 있음 |
| Geometry·depth | finite ray·positive optical Z/inverse depth, finite projected residual/Jacobian; 실패 depth를 기본값·clamp로 정상화하지 않음 | Projection/Perspective factor·FeatureManager; 유효 golden과 zero/negative/NaN 실패 구분 |
| PnP | preintegration `unique_ptr`, 현재 image에서 freshness 초기화; backend timestamp anchor·고유 대응6개·2D 분포·유효 IMU·현재 visual residual 필요 | solved anchor 없는 warmup/empty frame에서 old pose 성공 재사용 제외; warmup slot 복사·현재 solve 후에만 pose 사용 |
| 회전·초기화 | propagation/preintegration에서 가속도·covariance 회전 이전 local quaternion normalize; gravity iteration별 A/b 초기화 | 공용 deltaQ/legacy manifold 일괄 교체 없음. independent midpoint/SO(3), small-step bias truncation, known gravity/scale 회귀 범위 적절 |
| 진단·resource | measured feature/bias/relative motion warning과 numeric usability 분리; image/IMU history4096·기존10초 interval resource 경계 | 절대100m·1mZ를 보편 physical reset threshold로 연결하지 않음; overflow 이유·generation 노출 |

## 실제 검증 근거·범위

- `build/refactor-numerical/fresh-native.log`: NumericalContracts **18/18**. 새 실행을 가장하지 않고 기존 로그 확인
- `build/refactor-numerical/sanitize-complete/compile-manifest.json`: repository numeric20 TU의 실제 compile argv 기록. 별도 `build/refactor-numerical/source-manifest.json`의 numerical-owned19개 source/header SHA를 현재 소스와 대조, 불일치 없음
- `build/refactor-numerical/sanitize-engine/totalorder-test.log`: numeric18 포함 전체 **37/37**, 실제 sanitizer test exit0 기록. Repository30 TU fresh instrumentation, 이전 repository static library 제외; 고정 third-party Ceres/OpenCV/gtest 자체 instrumentation은 범위 밖
- flags: `-O1 -g1 -fsanitize=address,undefined -fno-omit-frame-pointer`, `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1`, `UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1`; suppression·leak disable 없음
- legacy local Jacobian: Projection/Perspective/IMU/PnP의 nonidentity `J_factor·J_plus` finite difference 확인. Bias/covariance의 일반적 exact derivative 또는 formal ambient manifold/Minus 검증으로 확대하지 않음
- known-gravity fixture: configured norm9.81007·scale2.3·기존4회 refinement·1e-5 bound; 회귀 기대값을 완화하는 방향의 수정 없음
- `build/refactor-diagnostic/after-total-order-60-comparison.json`: 동일60 inputs, 양쪽 init45·fresh poses15·coverage0.25, state mismatch0, translation max4.8074e-10m·rotation max3.8182e-6°. 짧은 일치 근거이며 full room1/room4 판정 제외

## clangd ownership 경고

- `build/refactor-evidence/clangd-final-optimizer.json`: compiler errors0, `bugprone-unused-return-value` severity2 하나 (`optimizer.cpp:182`, 의도적 `loss_function.release()`)
- Pinned Ceres2.2.0 primary source `include/ceres/problem.h:126-131`: 기본 `loss_function_ownership=TAKE_OWNERSHIP`, shared pointer는 destructor에서 한 번만 삭제
- 실제 `internal/ceres/problem_impl.cc`는 loss pointer별 reference count 보관·unique key 삭제. `addFeatureFactors`는 residual이 하나 이상 등록된 뒤 release; zero factor는 local unique_ptr이 삭제
- marginalization은 별도 `marginalization_loss_` 소유, prior 해제 후 loss reset. Ceres 임시 Problem의 loss를 prior가 빌리는 경로와 구분
- 결론: 이 경고에서 double-free/leak 결함 근거 없음. 경고 억제·style-only source 변경·불필요 재빌드 권고 없음

## 승인 한계·후속 gate

- **R5 대기**: full room1/room4·3회 pilot tolerance·최종 source/artifact/served hash·actual Worker·paced 결과는 leader/build owner의 현재 통합 결과로 판정
- 아래 추가 검토 이전 source에는 native4-thread partition accumulation과 WASM sequential accumulation 분기가 존재. 당시 CV threads1은 backend reduction 순서를 바꾸지 않았으며, 이 차이는 아래 승인된 portability 수정에서 제거
- low-level `ResidualBlockInfo::Evaluate`의 cost Evaluate 반환값 전파·prior 자체의 일반적 invalid-input/conditioning은 별도 방어 계약. 현재 Optimizer는 추가 전 factor 거부·accepted solve만 marginalize하며, 이번 검토에서 실제 caller-chain failure 재현 근거는 없음; arbitrary invalid helper 사용까지 안전하다고 확대하지 않음
- formal manifold rewrite·큰 bias correction의 exact Jacobian·sensor noise identification·observability·실제 phone/field GT·thermal·production 정확도 미검증
- 위 runtime/연구 한계는 승인된 작은 결함 수정의 보류 사유로 사용하지 않음. 현재 숫자·회귀 범위만 승인, 전체 SLAM/현장 품질 완료 주장 제외

## Marginalization portability 추가 독립 검토

- 판정: **APPROVE — 승인된 first-seen layout·ordered assembly 수정**. 새로운 blocker 발견 없음; 이 검토에서 build/replay/performance 실행 없음
- 소유 경계: 수치 담당자의 `marginalization_factor.h/.cpp`·자체2개 oracle만 검토. Engine에는 실제 constant를 읽는 진단 property 한 줄만 추가, 다른 기능 변경 없음
- Layout: 새 `parameter_block_order_`에 residual/parameter 첫 등장 순서 보존; map은 주소→size/index/data lookup 용도. Dropped first·kept next partition과 `getParameterBlocks`의 반환 순서도 같은 semantic vector 사용
- Assembly: native/WASM 동일 factor vector 순서·동일 i/j loop로 A/b 누적. Pthread round-robin·역순 partial-sum join·obsolete helper만 삭제; `eps=1e-8`·Schur/eigen algebra·residual·manifold·noise·solver stopping 설정 변경 없음
- `.at` 사용은 잘못된/missing mapping의 silent null insertion을 제거. 기존 active caller의 drop-set/shift mapping·one-shot MarginalizationInfo 수명 계약 유지
- 실제 red→green: `build/refactor-diagnostic/marginalization/{red.log,green.log}`의 targeted **2/2 red → 2/2 pass**. 최신 source/test SHA `source-frozen.json`
- 독립 정수 J/r oracle: full H=`[[3,0,2],[0,4,-1],[2,-1,4]]`, b=`[1,10,1]`; A 제거 후 Hkeep=`[[4,-1],[-1,8/3]]`, bkeep=`[10,1/3]`, bound2e-12. 세 allocation/address layout에서 같은 semantic indices·kept block order 확인; eigenvector sign/basis 자체 비교 제외
- IEEE754 order stress: `((2^54+1)-2^54)+1=1`의 순서대로 누적; 기존4-partial reverse join은0. 실수 exact sum2에 대한 정확도 주장과 구분된 portability oracle
- 진단: `Engine.getFeatureDiagnostics().params.marginalization_assembly_threads=backend::factor::kMarginalizationAssemblyThreads(1)`. Outer factor assembly 수이며 CV/Ceres/BLAS thread 수와 구분
- Engine source freeze SHA: `src/vio_engine.cpp`=`b19be7538721953af9263c7b9a5d5c3823b002d82faaca083a6c92d91f8a9952`
- **후속 필수 gate**: MarginalizationInfo class layout 변경 후 전체 transitive native/WASM/SAN 재빌드. 이전37-case/18-case binary를 새 header의 전체 검증으로 재사용하지 않기. Bounded520 비교 후 full3 pilot·room4·최종 source/artifact hash 판정은 leader/build owner 후속

## 선택 구간 conditioning 진단·연결 검토

- 판정: **APPROVE — logging-only API/wire**. 수학 원인 확정·parity cap 통과 승인과 구분; 새 build/replay/SAN은 이 검토에서 실행하지 않음
- 근거 snapshot: `build/refactor-diagnostic/conditioning/source-frozen.json`의 numerical6 files, `engine-wire-source-frozen.json`의 Engine wire. Engine source SHA=`88740d53ae25f6dc3951c7793eda98d4311614957f6bb542216318d078766974`
- API: existing Engine `setDiagnosticCapture`가 tracker·Estimator→Optimizer에 같은 bool 전달; Engine recreation/reset 후 `setParameter` 다음 기존 flag 전달. 새 binding·algorithm·solver stopping 설정 없음
- 기본/빈 값: Optimizer 기본·disable·reset cache는 `{"enabled":false}`. Engine backend string이 빈 경우 JSON `null`로 삽입; 첫 solve 전/warmup/no-current-update/reset에서도 malformed `"backend":}` 생성 경로 없음
- 순서: setup/prepare parameters→before-solve incoming prior/semantic layout/current gradient→factor membership→실제 Ceres summary/iterations→gauge application→actual marginalization operation→outgoing next-frame prior. Incoming cached operation에는 생성 frame timestamp, outgoing에는 next-frame 사용 목적 명시
- 변이 제외: 진단은 actual eigenvalues/eigenvectors·원래 Amm/Schur 결과를 읽고 별도 stream/local matrix에 norm/reconstruction 값을 계산. 원래 A/b·eigen cutoff1e-8·manifold·noise·gauge branch·Ceres stopping/residual 결정 변경 없음
- bounds: matrix별65536 cells 초과 시 values=null+truncated, vector/feature/layout1000 rows·solver100 iteration rows·message4096 chars. nonfinite 숫자=null·nonfinite flag, quote/backslash/control character escaping 확인
- OFF: snapshot/eigen residual 계산·row serialization은 capture 조건 내부. 기존 수치 처리 path는 유지; 진단 중 성능 지표·실행 시간 개선 주장 제외
- cache lifecycle: setter 호출은 snapshot cache를 초기화. Native/JS range helper는 상태 변화 때만 호출(`[1080,1110)` 시작true·끝false), 반복true 호출로 previous operation을 잃지 않는 점 build owner 확인
- **현재 검증 경계**: 정적 source/JSON assembly 검토만. Fresh native/WASM 전체 ABI compile·actual default/armed-before-solve/warmup/reset JSON parse smoke→selected1110 replay는 parent/build owner 후속. Logging-only 변경에 이전 SAN 숫자를 최신 검증으로 확대하지 않음
