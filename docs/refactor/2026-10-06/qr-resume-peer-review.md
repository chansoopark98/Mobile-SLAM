# Square-root prior 재개 독립 검토

- 판정: **APPROVE — 동결된 QR 수학·실패 처리·진단 계약의 정적 검토 범위**. 새 P0/P1 blocker 없음; R5 replay·배포·현장 정확도 승인과 구분
- 검토자: QR review lane / Sol6.1 Ultra. 제품·test·dependency·dataset·TLS·서비스 수정 없음; 이 문서만 작성
- 방식: PRD/test spec·execution plan·numerical lane·이전 peer review·현재 checkpoint·동결 5개 파일과 기존 로그 조회. 이 검토에서 build·test·replay·profiling 실행 없음
- 최종 동결: `build/refactor-diagnostic/square-root-prior/source-frozen.json`, SHA256 `74a931a0b1274152aacb7f87cc214aad273ab7eaf9763fdef64e6546e4331aaf`; 5개 source/test SHA를 실제 파일과 대조, 모두 일치

## 수학·순서

| 경계 | 확인·판정 |
|---|---|
| 입력 | 기존 robust loss 가중 arithmetic·local pose 6열·factor vector 행 순서 보존. 첫 등장 semantic block 순서로 dropped→kept columns 배치; 주소는 lookup 용도 |
| Dropped QR | `ColPivHouseholderQR(D)`의 implicit `Qᵀ[K\|r]`; D의 column permutation은 column space를 바꾸지 않으므로 K에 permutation 적용 불필요. `rank q`만큼 앞 행 제거; dropped dimension m만큼 무조건 제거하는 경로 없음 |
| Rank 계약 | local Eigen3.4 `ColPivHouseholderQR.h`의 실제 기본값 `epsilon·diagonalSize·maxPivot` 확인. `setThreshold` 없음; q=0·m=0에서 K 보존, rank 부족에서 q행만 제거 |
| Kept 압축 | 남은 K의 unpivoted HouseholderQR와 동일 `Qᵀr`; upper triangular 결과만 기존 n×n J에 복사, 첫 `min(rows,n)` residual 보관·부족한 행 zero padding. kept mode cutoff 없음 |
| Constant | n행 아래 residual-only 성분은 additive cost constant로 제외. H/g 보존 계약이며 eigenvector 부호·임의 QR basis 동등성 또는 원래 절대 cost 동등성으로 확대하지 않음 |
| 빈 경계 | n=0 또는 inputRows=0은 `marginalization_empty_system` 실패·빈 prior. m=0은 dropped QR 생략; all-zero D의 q=0·all-zero K의 zero J 경로는 정적으로 유효 |
| prior 입력 | prior 생성에 normal equation·pseudo-inverse·eigen cutoff 사용 없음. Gram/eigen은 opt-in diagnostic-only |

- 독립 analytic oracle 유지: `A=2^22`, `s=2^-8`, rows `[A,-A/2,-A/2]`, `[0,s,-s]`, r=`[1,3]`; H=`s²[[1,-1],[-1,1]]`, g=`3s[1,-1]`. uniform scales `2^-16/1/2^16`, duplicate dropped columns·zero dropped rank·no-drop·orthogonal mixed rows 확인
- 기존 5×3 oracle 보존: semantic indices·kept address order, H=`[[4,-1],[-1,8/3]]`, g=`[10,1/3]`, bound `2e-12`; 삭제·완화 없음
- FP stress 입력 `[2^54,1,-2^54,1]`·과거 red/green/before-files 보존. 이전 ordered normal fold1은 실수 정답2와 다른 구현 계약; QR 전환에 맞춰 row 보존·주소별 동일 결과·H oracle·명시적 IEEE backward-error budget으로 재계약
- FP exact-real reference는 `uint64_t` binary 정수식으로2 계산; WASM의 long double precision 차이 제거. budget `128·eps/(1-128·eps)·Σ|r|`은 큰 cancellation 입력의 floating 오차 한계이며, gradient를 정밀하게2 복구했다는 근거로 사용 제외

## 자원·실패·lifecycle

- Pre-evaluation bound: ambient factor J/r·saved parameter doubles + dropped/CPQR copies·kept transform/reduced/QR·output·scratch·capture-on Gram/eigen 합산. size_t 곱/합 overflow를 검사한 뒤 **8,000,000 double cells** 초과 시스템 전체 거부; factor/prior 일부 truncation 없음
- 환산: 64,000,000 bytes = 약61.04 MiB. allocator metadata·factor private buffers·총 RSS·물리 phone 성능은 이 한도의 보장 범위 제외
- `raw_jacobians=nullptr` 기본값·snapshot unique ownership 확인. false CostFunction Evaluate·pre evaluation/QR bad_alloc은 explicit failure/reason·빈 J/r; 초기 단계 거부 시 평가 buffer 할당 전 종료
- Optimizer 두 perform 경로: candidate RAII → pre/evaluate/QR → address remap → 성공 시 prior 교체. 실패는 새 candidate와 이전 prior·parameter block vector 해제 후 반환; remap/record의 bad_alloc도 prior 해제
- 성공한 현재 solve의 apply 결과와 usable=true 유지; 이후 Estimator window advance 전에 위 prior 해제 완료. 다음 solve의 prior 없음 anchor 경로 유지
- Estimator는 optimizer quality warning과 measured failure reason을 병합; 기존 warning 덮어쓰기 없음. Solver stopping·manifold·noise·depth guard·GT·parity cap 변경 없음
- 실제 전역 OOM 복구·factor private allocation·모든 예외 지점의 fault injection은 이 정적 승인 범위 제외. 전체 프로세스 memory 회복을 claim하지 않기

## 진단·증거·후속

- 기본 OFF: Gram/eigen·timing snapshot·JSON 행 처리 제외, `{"enabled":false}` 유지. ON: `square_root_qr`, `rankUsedToFormPrior:false`, `normalMatricesDiagnosticOnly:true`, 실제 q·inputRows·workspace overflow·failure reason 노출
- matrix limit65536 cells·vector limit1000 rows·solver iteration limit100·message limit4096 유지. 초과는 null/truncated·nonfinite 숫자는 null; rank spectrum은 diagnostic-only 명칭
- timing label `assemblyAndQrMilliseconds`: pre evaluation·최종 spectrum/JSON 제외. total marginalization latency·peak RSS 지표로 사용 제외
- Ceres 추가 iteration fields `validStep`, `nonmonotonicStep`, `linearSolverIterations`는 실제 summary 관측만; solve 옵션 변경 없음
- 조회한 owner 로그: `green-final-integer-oracle.log`의 targeted `NumericalContracts.Marginalization*` **11/11**, caller Optimizer/Estimator compiler syntax exit0. 이 reviewer의 신규 실행 결과로 보고 제외
- targeted binary는 해당 filter만 새 owned object 사용; 이전 cached ABI의 Optimizer/Estimator runtime 검증으로 확대 제외. `MarginalizationInfo` class layout·Evaluate/marginalize 반환 계약 변경으로 **모든 transitive native/WASM/SAN rebuild 필수**
- 통합 owner 후속: fresh native/WASM/full SAN → actual selected1110 incoming/outgoing gauge·first solve branch → 기존 cap의 full3 pilot/room4·paced/loss/browser gate. short/static 합격으로 full parity·원인 유일성·정확도 향상 판정 제외
- physical phone·field GT·thermal·production/full SLAM 품질 미검증 유지

## Opt-in benchmark API·Worker/replay 추가 정적 검토

- 판정: **APPROVE — 동결된 benchmark control·기본값·lifecycle·truthful profile 연결**. 새 P0/P1 blocker 없음; 이 추가 검토도 source/log 조회와 문서 작성만 수행, build·test·replay 실행 없음
- Numerical manifest `build/refactor-diagnostic/square-root-prior/benchmark-source-frozen.json` SHA256 `1e8ead59b9e292041879d0cad794b5049dcf8d743ada2b73e00aa390564e044b`: 신규7개 파일+기존 QR2개 hash 일치
- JS manifest `build/refactor-evidence/benchmark-profile/source-frozen.json` SHA256 `324acf6766719caf00dc18db5f8d83b56637b3271bd7e5de5b2e3f5415f3edbe`: Worker/replay/runner/test4개 hash 일치. 이전 runner metadata manifest는 superseded

| 경계 | 확인·판정 |
|---|---|
| 기본/옵션 | Engine·Optimizer per-instance bool 기본false; 전역 config 변경 없음. true에서 Ceres function/gradient/parameter tolerance만0, false에서 기존 기본값 유지. iterations/time·factors/loss/manifold/QR·usability·pose/GT/caps 변경 없음 |
| lifecycle | Engine setter가 현재 Estimator에 전달, reset/reconfigure로 새 Estimator 생성 시 Engine flag 재적용. Optimizer reset에서 mode 유지; explicitfalse로 해제·새 Engine은false. WASM setter/getter binding 존재 |
| Worker | `===true` 요청만 활성화; omitted/false/nonboolean은false setter로 persisted mode 해제. true 요청에서 setter/getter 부재·readback mismatch는 configure 실패·configured=false. legacy 기본 경로의 API 부재는 실제 profile=null로 남기며 benchmark 성공으로 주장 제외 |
| 실제 profile | Worker result는 Engine getter, replay export는 실제 frame row의 bool 사용. request를 actual로 복사하는 경로 없음; ordinary replay의 actualOptions=null과 bounded diagnostic 필요 명시 |
| 실험 분리 | paired numerical 요청은 explicit env/CLI/query → API readback 확인, max10/time10s. paced/loss는 benchmark=false·max10/time0.1s. 기본 profile의 proxy latency와 benchmark numerical work를 한 지표로 합치지 않기 |
| 실제 solver | `Summary.iterations.size`, termination/message·iteration0 행 유지. diagnostic actual f/g/p·maxIterations/maxTime 기록; count/종료값 강제 보정 없음. `max10_zero_positive_tolerances` 식별자는 harness profile이며 실제 최대 횟수는 maxIterations 필드로 확인 |
| 기존 gate | full pilot3·room4의 동일 입력·pose/coverage·cap·기존 scorer와 compare 호출 보존. 실제 frame별 profile 확인 추가; 현재1110의 numeric PASS·strict iteration FAIL 기록을 이전 R5 합격으로 소급 변경하는 경로 없음 |

- 과학적 판단: 원 계획의 fixed-iteration 의도와 실제 `max_num_iterations=10`+기본 조기수렴 옵션의 차이를 explicit benchmark로 통제하는 방식은 정당. 관측된 기본값 실행을 보존하고 새 통제 profile의 전체 결과를 판정할 때 cap 완화·출력 조작에 해당하지 않음
- pinned Ceres2.2 source 확인: tolerance0에서도 gradient/step/cost_change가 정확히0이면 실제 convergence 가능; 최소 trust-region radius·invalid step·time 종료 유지. max10은 정확한10회 보장이 아니며 summary11행은 초기행+10 trial 가능. 실제 종료 원인과 count semantics 유지 필수
- 조회한 owner 회귀: `benchmark-green.log` focused **13/13**·3개 boundary compiler syntax exit0; JS `contracts-all-green.log` **78/78**, fail/skip/TODO0. Fresh Optimizer/Marg/Test filter와 Worker fixture 근거이며 Engine lifecycle·실 WASM·전체 ABI runtime 통과로 확대 제외
- 필수 후속: fresh transitive native/WASM/SAN·실 Engine/module의 default→enable→reset→reconfigure→disable getter/옵션 smoke → short300 first253 원인·benchmark1110 → 같은 cap의 full3/room4·별도 default paced/loss/actual browser 검증
- 내부 floating branch 또는 조기수렴 횟수 자체의 bitwise 일치만으로 production 품질 결함을 판정할 근거 없음. 실제 usable/fresh/state/pose/cap 실패가 있으면 원인 해결 필요; field accuracy·기본 solver convergence·물리 latency를 benchmark 합격으로 보증하지 않기

## Room4 frame1973 보충 semantic 판정

- 판정: **APPROVE — 확인된 exact-zero function stop에 대한 보충 numeric/current-usable/method 판정만**. 원래 frozen strict summary equality는 **FAIL 유지**; room4 정확도·production·전체 목표 완료 승인 제외
- 증거: `build/refactor-diagnostic/benchmark-profile/room4-first1973/{actual1973-solver,causal1973-proof-and-proposal,comparison,commands,primary-source-preparation,calibration-source}.json`. Reviewer는 기존 actual 자료·pinned source만 조회; 새 build/test/replay 실행 없음
- 적용 artifact: old candidate `fce25012d61a0d5965e4d5afadda25aecf96c219b30451e0394575c3caa10da1`, compiled source manifest `11ca788ff68b39653984b2d384c672820fcf698cfc0a621d273a49311ae9075c`. 실제 JS hash 일치 확인; 이후 수정 중인 source 또는 새 후보의 승인으로 확대 제외
- 원래 frozen contract SHA256 `4f82ccdfbebd48504d5de8320c5704511009d11b48b7f1e6478ada084cb2db77` 재확인, 변경 없음. `run-5/room4/parity.json`의 status=fail·원본10/11 counter·CONVERGENCE/NO_CONVERGENCE 보존

| 경계 | 실제 근거·판정 |
|---|---|
| Native1973 | usable=true, actual f/g/p0·max10/time10s. Message `Function tolerance reached. \|cost_change\|/cost: 0.000000e+00 <= 0.000000e+00`; finite positive cost59681.54675572614. Source guard `fabs(cost_change)>0`와 finite message로 actual double cost change=0 확인 |
| Native count | 저장된 행0..9 총10행, 마지막 gradient0.0017964945468520455·step3.6657097232797772e-6. Source 순서상 trial10을 계산·평가한 뒤 function stop으로 append 전 반환; attempted10은 source에서 유도, 해당 미저장 trial의 step/gradient 직접 측정 제외 |
| WASM1973 | usable=true, actual f/g/p0·max10/time10s. 실제 message `Maximum number of iterations reached. Number of iterations: 10.`; 저장 행0..10 총11행. Trial10 cost change6.548361852765083e-11·gradient0.0023064151822609347·step2.557228708623317e-6 |
| 재현 | prefix1978·capture1968..1977, original input prefix SHA `ffe4efef0539de5fbf2b077c5c6facca1647c3954a6d6ad42d7ed367f6da9cea`. 각 플랫폼 pose가 기존 full run prefix와 bitexact; fresh pose timestamp/state/reason/counter 재현 |
| 입력/metadata 한계 | 원래 플랫폼별 K의1ULP 차이 보존·재현. WASM warmup0..75의 getter -1 대 ordinary replay null timestamp는 no-fresh-pose schema 차이로 기록; 유효 pose 시간의 mismatch 없음 |
| 숫자/상태 | full2228 입력·2148 pose·init76·동일 coverage, 기존 pose caps 유지. Full room4 translation p95=2.2579987787668658e-6m/max=2.7497002279158254e-6m, rotation max=4.182619578303747e-6°. 추가 mismatch는 frame1973의 solver count·termination 두 metadata field뿐 |

- 적용 기준은 실제 trace 확인 전 고정: 같은 artifact/input/profile, finite exact-zero tolerance stop의 실제 message, 다른 쪽의 actual max-budget 종료, 양쪽 usable/fresh/finite, 기존 pose/coverage/state/epoch cap 보존. Time/min-radius/NaN/failure 또는 모든 CONVERGENCE↔NO_CONVERGENCE를 동일 예외로 묶는 판정 제외
- Pinned Ceres `Minimize`: parameter/function checks는 `HandleSuccessfulStep`·summary append 이전 반환; gradient check는 append·max-iteration check 이후. 이번 차이는 exact-zero **function** stop이며 gradient0·stationary optimum·현장 convergence 주장 제외
- 보충 표기: **numeric/current-usable/method parity PASS WITH PROVEN FUNCTION-STOP EXCEPTION; strict solver summary equality FAIL**. 한 frame의 한 summary row 차이와 termination 차이를 최종 보고에 명시; 원래 strict 결과 소급 PASS·counter 보정·cap 완화 제외
- 정확도는 별도 심각한 미해결 결과: 기존 `run-5/room4/native/metrics.json`의 SE(3) scale1 translation ATE RMSE **143.367042953646m**, 1s translation RPE RMSE **5.5498739442460066m**, Sim(3) diagnostic scale **0.0016633312522461622**. Metadata 예외로 accuracy 통과 불가
- default profile A/B·초기화/추적 결함 원인·별도 승인된 새 수정·새 artifact의 전체 회귀/GT 검증 대기. 이 판정으로 QR 또는 benchmark가 큰 GT 오차의 원인이라는 결론도 제외

## Initializer velocity·첫 constant target 추가 source 검토

- 판정: **APPROVE — 동결된 두 최소 결함 수정의 source·oracle 범위**. 새 P0/P1 blocker 없음; 이 reviewer의 신규 build/test/replay 실행 없음. 제품 source/test 수정 없이 이 문서만 추가
- Initializer manifest `build/refactor-diagnostic/initializer-velocity/source-frozen.json` SHA256 `f0a5d187c14172f33449b587f31ac745d6d1c9e41f30c520e1df6f0abc16c3d9`; 3개 source/test hash 실제 파일과 일치
- Blank manifest `build/refactor-evidence/blank-transition/source-frozen.json` SHA256 `d5eaac204d03d968da0281fd8fbe66b97b677cb7d71a5a6642fa67dc5a9e3da2`; cpp `7901617616f836b7dab62f00af93e269450f6a831a1151b5ddbd2efa39345ea5`·test `2f86d7ecc2e5525edb521af511b46de9d30dd6c8bc2492479b0bdd8e820fa7ee` 포함 모든 listed source hash 일치. Header는 이전 snapshot과 동일

| 경계 | 확인·판정 |
|---|---|
| Alignment layout | 실제 LinearAlignment와 RefineGravity는 map의 모든 image row마다 velocity 3열 배치. `visualInitialAlign`의 every-image `image_index`와 key-only window `kv` 분리가 정확하며, 같은 image R로 body velocity를 world V로 변환 |
| Initializer 범위 | source diff는 counter·loop increment·x segment index만 변경. scale/g/pose/depth/gyro/noise/stopping/QR·caps 유지. Header friend는 tests에서만 정의된 private actual-path seam이며 class field/public runtime API/dependency 추가 없음 |
| 독립 physical oracle | analytic excited P·dP/dt·d²P/dt²·nonidentity R·g·scale2.3·nonzero lever arm, 실제 2kHz raw IMU IntegrationBase·alignment·repropagate·transfer 사용. 11 contiguous keys control와 13 image rows/nonkeys1·3 →11 selected keys 비교; expected V는 x/transfer loop 복제가 아닌 analytic derivative |
| Constant 경계 | raw grayscale `min<max`만 기존 LK/detector 경로 허용. 정확히 constant인0/127/255만 새 거부 경로; nonconstant blur·낮은 contrast threshold 도입 없음. CLAHE 이전 raw predicate로 처리 결과의 인공 gradient와 구분 |
| Tracker lifecycle | 기존 `pruneTrackedPoints({})`가 point/history/normal/velocity/ID/count 배열 함께 비움, pending n_pts 별도 비움. 표준 image/time/pyramid finalization·undistortedPoints의 map clear/copy로 두 normal map 비움. reset·n_id 재설정·calibration 변경 없음 |
| Recovery | 다음 textured backend frame에서 신규 track count1·이전 nextID 이상 ID, 다음 실제 LK에서 배열 정렬·유효 점 유지. Engine의 기존 empty_features·pose/map invalidation boundary 사용; 주입 pose·timestamp 보정·새 reset 정책 없음 |
| 성능/경계 한계 | `minMaxLoc`는 매 frame 추가 O(width·height) 스캔; constant branch만 드물게 실행. CLAHE/pyramid를 constant frame에도 기존대로 진행. 추가 비용의 실제 p95/throughput은 새 runtime 측정 전 미검증 |

- 조회한 Initializer 실제 RED: 13-row의 selected velocity10개에서1.2054450105186394~2.9211123705268776m/s 오류, P/R/g·11 contiguous key control 통과. GREEN focused2/2·NumericalContracts32/32 owner 로그 확인; empty FeatureManager·제공된 SfM poses 조건으로 image matching/SfM 정확도 검증 제외
- 조회한 Tracker 실제 RED→GREEN: constant0/127의 CLAHE0/1·constant255의 CLAHE0 × backend/track-only2 = **10 transition**. 255+CLAHE1 seed는 target 전 LK track0으로 해당 전환 회귀 제외, 범위 명시. Actual Engine127 path의 기존4 feature/initializing →0/empty_features·다음 관측 recovery 확인
- 위 로그는 scoped fresh TU·고정 native archive 재사용 근거; 전체 transitive native/WASM/SAN·실 Worker 첫 blank·새 candidate room1/room4 검증으로 확대 제외
- 후속 필수: 새 source/artifact manifest·fresh 통합 회귀/SAN/WASM → 기존 full3/room4/default·benchmark·paced/loss/actual browser 검증. 기존 frame/evidence/caps·strict FAIL·보충 semantic 판정 보존
- 두 unit fix는 actual room4의143m ATE 원인·유일 원인·정확도 개선을 입증하지 않음. Room4 결과를 보고 선택한 variant는 adaptation 결과로 표시; independent held-out/field/production 정확도 미검증 유지
