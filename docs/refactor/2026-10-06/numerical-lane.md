# 수치·solver·reset lane

- 소유: `include/src/backend/{estimator,optimizer}`, IMU factor·integration, `include/src/frontend/{pnp_frontend,feature_manager,failure_detector}`, `src/frontend/initialization/initial_alignment.cpp`, 독립 `tests/test_numerical_contracts.cpp`
- 기존 dirty tracked/untracked 수정 보존; 다른 lane의 engine·tracker·CMake 변경 되돌리기 금지
- 변경 전 실패 회귀: 0/1 sample·singular/nonfinite covariance, inverse-depth0/NaN, PnP anchor/최소 대응 부족, propagation의 비단위 quaternion, gravity known truth
- 수정: explicit covariance inverse 삭제; finite symmetric SPD·positive pivots·시간 coverage 검사 후 factor별 immutable whitening 한 번 계산. optimization·marginalization 동일 rejection 사용
- solver: usable summary·finite cost/parameters·현재 visual observation·필요 IMU interval 확인. budget NO_CONVERGENCE 허용; 실패 결과 적용·prior 갱신 금지
- lifecycle: optimizer prior/parameter block 해제, estimator current-image usability·reset generation, PnP preintegration RAII·현재 solve 성공·anchor·최소 대응, invalid geometry 명시 거부
- propagation: 해당 두 경로에서 delta quaternion normalize; 공용 deltaQ/manifold convention 유지
- failure: feature/bias/relative motion 진단 기록; 절대100m·1mZ를 보편 reset gate로 연결 금지
- gravity: 독립 known truth 회귀를 먼저 실패 확인한 뒤 iteration마다 normal equation 초기화; replay 영향은 통합 lane 근거로 별도 기록
- 검증: numerical regression target·기존 integration/sliding-window regression, ASan/UBSan, local factor Jacobian `J_factor J_plus` finite difference. 빌드 owner에 target 등록 요청, 새 build 출력·기본2 jobs
- 한계: formal manifold 전환·observability·실제 phone/field GT·production 정확도는 별도 미검증

## Marginalization portability 추가 cleanup 계획

- 승인: run3의 frontend100frame exact·pose drift62 이후 원인 경계 수정. 새 소유 파일 `include/backend/factor/marginalization_factor.h`, `src/backend/factor/marginalization_factor.cpp`, 기존 `tests/test_numerical_contracts.cpp`
- 실제 red 먼저: 독립 scalar J/r로 H/b·Schur oracle, semantic first-seen role A/B/C·allocation/address 순서 독립성 확인. eigenvector 부호/임의 basis 대신 `JpriorᵀJprior`, `Jpriorᵀrprior` 검증
- Layout: lookup map 유지, first-seen semantic block vector로 dropped→kept partition·parameter block 반환 순서 고정. raw pointer hash 순회가 column 의미를 결정하는 경로 제거
- Assembly: Native4 round-robin factor partition·join3→0와 WASM single vector-order 분기 제거. 양 경로 같은 ordered factor accumulation; caller 확인 후 pthread branch·obsolete helper struct만 삭제
- 공유 진단: `backend::factor::kMarginalizationAssemblyThreads=1`, outer assembly의 실제 상수. Ceres/CV/BLAS thread 수와 구분; Engine/Replay owner가 기존 globalNUM_THREADS call site 연결
- 변경 제외: residual/factor 정보·eps1e-8·noise·manifold·solver tolerance·parity cap·GT 선택·원본/서버 변경
- 검증: targeted `NumericalContracts.Marginalization*` red→green만 이 lane 실행. fresh native/WASM/fullSAN·bounded520frame·full3pair/room4는 parent/Build owner가 순차 실행

- 실제 old-fail **2/2**, `build/refactor-diagnostic/marginalization/red.log`: allocation3배치 모두 role B/C layout2/1, semantic Schur matrix/gradient oracle 불일치; native4-part gradient0 대 선언된 IEEE factor fold1
- 실제 new-pass **2/2**, `green.log`: first-seen A/B/C·dropped/kept B/C·독립 `Hkeep=[[4,-1],[-1,8/3]]`, `bkeep=[10,1/3]` tolerance2e-12; ordered IEEE stress도1
- stress oracle 한계: `(2^54+1)-2^54+1`의 순서가 고정된 floating fold 결과1 검증. 정확한 실수 합2 또는 물리 정확도 증명으로 표현 제외
- 구현: map은 lookup만 사용, first-seen vector로 layout/반환 순서 고정, 양 플랫폼 같은 factor/block 누적. legacy pthread branch·partial matrix copies·thread helper struct 제거
- frozen source/test SHA256·API·검증 범위: `source-frozen.json`. header `ec1e7b0b5eabfa9ae3a7e6c461c1d59113cc5e8b9433c2f39587ddd19c705dfa`, cpp `00bcac8f8ca0beb574968eb5b0fbe0fa3da7d8620cf26641bf5dc26cb43bf25b`
- `MarginalizationInfo` class 크기 변경: 기존 cached Optimizer object가 이를 할당하므로 full old-linked numerical suite 실행 제외. parent가 모든 transitive dependency를 fresh rebuild 후 실제 SAN/native/WASM gate 판정
- run3 인과 한계: frontend0–99 exact, pose drift62·solver479가 기존 trace 근거. 위 두 unit 회귀는 실제 portability fork를 닫는 근거;0.515m 장기 disparity 해결 판정은 새520/full paired 결과 필요

## Run4 conditioning 진단 계획

- 승인 범위: Estimator/Optimizer/SolverDiagnostics·MarginalizationInfo에 opt-in bounded JSON 관측만 추가. EPS·matrix arithmetic·stopping·manifold·noise·cap 변경 없음
- API: `setDiagnosticCapture(bool)`, `const std::string& getBackendDiagnostics() const`를 Estimator→Optimizer 전달. Engine owner가 기존 diagnostic flag·getter에 연결
- 기본 OFF: 관측용 matrix/eigen 계산·row serialization 제외. 활성 frame1080–1110에서만 incoming prior/이전 actual marginalization snapshot, residual membership·실제 Ceres iteration/termination·gauge branch·outgoing next-frame prior 저장
- 인과 태그: incoming snapshot은 solve 전, cached marginalization은 생성 frame tag, outgoing은 다음 frame의 입력. frame1094 solve 뒤 matrix를 same-frame 원인으로 오인하는 설명 제외
- 상한: feature1000·iteration100·matrix65536cells, 명시 truncated/nonfinite. cache는 frame마다 교체·reset/disable 제거, raw image·새 공유 framework/의존성 제외
- prior: semantic block role/column layout·JᵀJ/Jᵀr, 실제 Amm/Schur dimensions/norm/symmetry/eigen info/rank/near-EPS/eigen·pseudo-inverse residual. 기존 실제 eigendecomposition 결과 재사용
- solver: summary.message·initial/final cost·iteration cost/cost change/gmax/step/relative decrease·실제 usable, gauge origin/optimized pitch·yaw difference·singularity branch
- parent/Planner가 fresh ABI dependency/native/WASM+smoke와1110 진단 실행. 이 lane의 self build/performance replay 제외; fullSAN은 최종 behavior 수정 후

- 최종 API·6개 ABI source hash: `build/refactor-diagnostic/conditioning/source-frozen.json`. scoped whitespace·정적 data-flow 확인만 수행; 이번 logging source의 실제 compile/smoke는 parent 기록 필요
- JSON: `backend.incomingPrior`의 current-parameter JᵀJ/Jᵀr와 cached previous creator timestamp, `visualMembership`, `solver`, `gaugeApplication`, `outgoingPriorForNextFrame`. outgoing cached operation의 role은 생성 frame 기준, incoming role은 window shift 이후의 현재 parameter 기준
- Marginalization spectrum은 기존 actual eigenvectors/eigenvalues를 재사용. raw matrix·signed eigenvalues·actual eps retained rank·norm/symmetry/eigen info·Penrose/reconstruction residual, dimension×epsilon×spectral scale은 **진단 값만**이며 cutoff로 사용 제외
- matrix 상한은 **각 matrix65536cells**, status/feature/eigen rows1000·iteration100·message4096. 초과 시 explicit truncated, nonfinite 수치는 null과 flag. raw image 저장·root service 변경 없음
- 실제 parameter-relative step ratio는 Ceres iteration row에 없어 `parameterRelativeStepAvailable=false` 표기; final norm에서 과거 ratio를 만들어내기 제외. actual termination message·step norm·cost change·gmax는 기록
- 수학 counterexample: HadamardQ/2와 `H=Q·diag(0,0,2^42,2^44)·Qᵀ`는 exact rank2·known-null residual0. NumPy double eig의 null λ≈−0.00103957/+0.00140578에서 fixed absolute1e-8은 rank3; machine epsilon×norm≈0.00390625. **product Eigen/실제1094 원인 재현은 아님**
- local Eigen3.4 `SVDBase.h`의 default rank criterion은 dimension×machine epsilon을 largest singular value에 곱하는 방식. 현재 온라인 [Eigen 문서](https://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)는 nightly5로 redirect돼 actual3.4 source와 구분
- [Square Root Marginalization 논문](https://arxiv.org/abs/2109.02182)은 normal-equation 형성 없이 QR로 prior를 갱신하는 방법을 제시. 문헌은 conditioning 조사 근거이며 현재 프로젝트의 root cause·수정 검증을 대신하지 않음

## Square-root prior 수정 계획

- 실제 결함: 1093 outgoing→1094 incoming snapshot 동일. uniform translation gauge의 `GᵀHG`가 Native `[0.00042145,0.00191649,0.00344151]`, WASM `[0.00003069,0.00004287,0.00467266]`; 상대 IMU/projection 정보의 zero gauge에 허위 정보 존재
- 인과 한계: 1093 양쪽 Schur rank75, 1094 trial2 Native candidate evaluation 실패·WASM 성공. prior 결함은 독립 증명됐지만 해당 domain 실패의 유일 원인·장기 parity 해결 주장은 실제 재실행 전 제외
- 소유 파일: MarginalizationInfo header/cpp·기존 NumericalContracts; Optimizer iteration의 `step_is_valid`, `step_is_nonmonotonic`, `linear_solver_iterations` 관측 추가. allocation 실패 caller는 승인받은 최소 prior release/qualityReason 처리만 추가
- red 먼저: binary known J/r의 weak contrast·uniform translation nullspace·uniform scale, duplicate/zero/no dropped rank. `JpriorᵀJprior`·gradient는 analytic reference, eigenvector basis/부호 검증 제외
- 수학: `D=J_dropped`, `K=J_kept`, CPQR의 rank `q`를 사용해 `Qᵀ[K|r]`의 첫 q행만 제거. `Kᵀ(I-DD⁺)K`, `Kᵀ(I-DD⁺)r`와 동등; rank deficient에서 m행을 무조건 제거하는 경로 제외
- rank: existing Eigen3.4 CPQR machine-relative 기준 `min(rows,m)·epsilon·maxPivot`; uniform scale 불변. kept Jacobian에 absolute eigen cutoff 없음
- 압축: 남은 K의 HouseholderQR와 같은 Q를 residual에 적용, 기존 n×n Jacobian/n residual로 유지·부족한 행은 zero padding. 미관측 residual-only constant 행 제거. dense rows×rows Q·normal equation·pseudo-inverse로 prior를 형성하는 경로 삭제
- 자원: integer-overflow 검사 후 주요 dense workspace **8,000,000 double cells** 한도. 초과는 explicit failure, factor/prior 일부 truncation 금지; 실제 inputRows/droppedRank/workspace estimate/timing availability 기록. 이 한도는 총 RSS·제품 phone 성능 보장이 아님
- 실제 pre-solve feature에서 계산한 row 상한: frame1093 약908×148 inputs1.075MB, frame1094 약1346×198 inputs2.132MB. QR 복사·transform은 추가 임시 메모리, `O(h m²+h mn+h n²)`; timing/peak RSS는 fresh native/WASM replay owner가 측정
- 기존 회귀 유지: 5×3 semantic/address Schur oracle. FPstress의 입력 `[2^54,1,-2^54,1]`·과거 red/green은 보존; old IEEE normal fold1 lock을 QR truth로 강제하는 contract는 명시적으로 교체. exact real gradient2와 선언된 floating backward-error 범위·semantic row/address portability 확인
- 진단: Gram/eigen 계산은 capture ON에서만 수행, eigen rank/cutoff는 prior 형성에 쓰지 않는 diagnostic-only로 표기. default OFF cache/flag/bounds 유지
- 검증: 이 lane은 targeted marginalization red→green·source freeze만 실행. parent의 fresh all-ABI native/WASM/SAN·selected1110·full3/room4·paced gate 전 장기 closure/물리 정확도 주장 제외

## 실행 결과

- VERIFIED: fresh native `NumericalContractsTest` **18/18**. 로그 `build/refactor-numerical/fresh-native.log`
- native 기록 시점: 최종 의도적 소유 이전 void cast 표기 전. 현재 최종 source의 검증은 아래32개 sanitizer; native candidate의 최종 통합 rebuild/CTest/hash는 build owner 기록으로 대체
- VERIFIED: 수치 경로·의존 repository source 20개를 새 instrumented object로 빌드, **ASan+UBSan 18/18**, leak 검출·첫 오류 중단 활성. 로그/실제 argv `build/refactor-numerical/sanitize-complete/{test.log,compile-manifest.json}`
- 환경: GCC11.4.0, C++17, pinned Ceres2.2.0, 기존 Eigen/OpenCV. sanitizer flags `-O1 -g1 -fsanitize=address,undefined -fno-omit-frame-pointer`
- VERIFIED: engine owner의14개 회귀 object와 결합, **ASan+UBSan32/32**. common core28개+test2개=30 repository TU 모두 새 instrumented object, 이전 repository static library 사용 제외. 로그/객체 hash/실제 link argv `sanitize-complete/{combined-test.log,combined-link-manifest.json}`; engine compile argv·source hash `sanitize-engine/compile-manifest.json`
- sanitizer 경계: pinned Ceres/OpenCV/GoogleTest 자체 instrumentation 제외. native adapter 별도 경로는 이32개 검증의 범위 제외
- VERIFIED: clangd23.1.0의 실제 optimizer LSP/clang-tidy probe, error0·warning1. `build/refactor-numerical/clangd-optimizer.json`; Ceres가 이미 소유한 loss의 `unique_ptr.release()` 반환값 무시 경고. 의도적 소유 이전을 명시했으며 경고 억제 설정 변경 제외
- 기존 결함 보존: 첫 독립8개 중7개 실패 `build/refactor-numerical/red.log`; preintegration 정규화 전 `dv=(0.375,0.25,0)`/`dp=(0.09375,0.0625,0)` 대 기대 `(0.4,0.2,0)`/`(0.1,0.05,0)` 실패 `integration-rotation-red.log`
- 추가 실제 UB: `FeaturePerId` 이동 시 초기화되지 않은 `is_outlier/is_margin` bool 읽기. 실패 `sanitize/ubsan-uninitialized-feature-red.log`, field 기본값 명시 후 재검증 통과
- 폐기한 검증 구성: 새 `IntegrationBase`와 기존 cached `Frame` object 혼용에서 class size10520/10512 불일치. `sanitize/partial-link-abi-rejection.log` 보존; 이를 제품 결함·sanitizer 통과로 보고 제외. fresh object로 전체 수치 의존성 재빌드
- source/binary SHA256: `build/refactor-numerical/source-manifest.json`; 원본 데이터·서비스·TLS·다른 lane 수정 변경 없음

## 변경·삭제

- IMU: covariance inverse·Evaluate마다 decomposition 삭제. `Σ=LLᵀ`, `W=L⁻¹` triangular solve, finite/symmetry·rounding보다 큰 양의 pivot·실제 positive dt/coverage 검사. factor 생성 시 immutable whitening 캐시; optimization/marginalization 같은 거부 기준
- solver: summary usability·termination·iterations·initial/final cost·IMU/visual/current-frame residual 수 노출. 사용할 수 있는 `NO_CONVERGENCE` 유지; unusable 결과 적용·새 prior 생성 제외, window advance 전 기존 prior/주소 목록 해제
- gauge: 유효 prior가 없는 문제의 첫 pose 고정. legacy `J_factor·J_plus` projection/perspective/IMU/PnP pose Jacobian의 nonidentity local finite difference 통과
- reset: optimizer prior/주소·loss 소유 해제, estimator window·feature·IMU·image history·진단·current-image usability 초기화와 generation 증가
- geometry: finite ray·positive optical Z/inverse depth 검사, 잘못된 triangulation·depth shift는 실패로 분류; NaN/behind-camera를 clamp·기본 depth로 정상화하는 경로 삭제. solver 중 비유한 residual/Jacobian 명시 거부
- PnP: preintegration `unique_ptr`, 현재 anchor·6개 이상 고유 대응·2D 분포·유효 IMU 확인, 현재 solve 성공만 pose 유효. warmup propagation의 다음 슬롯 state 복사, 기존 config solver budget 사용, blank/invalid frame에서 과거 hasPose 재사용 제거
- rotation: estimator/PnP local increment와 preintegration의 가속도·covariance 회전 전 quaternion 정규화; 공용 deltaQ·manifold·exp-map 변경 없음. independent 2-step angle/acceleration·SO(3)·finite SPD covariance 통과
- bias 선형화: 50×0.005s·bias 변화 약1e-4, 위치2e-7m/속도2e-6m/s/회전2e-7rad truncation budget 통과; exact finite-step derivative 주장 제외
- gravity: 반복별 A/b 초기화만 수정. configured gravity norm9.81007·scale2.3 known-truth, 기존4회 반복·1e-5 오차 bound 유지. room1 replay 영향은 통합 lane 근거 필요
- quality: feature/bias/relative motion threshold crossing을 `qualityReason`으로 기록; field calibration 없는 universal reset policy 제외. 절대100m 위치 reset 삭제,1mZ threshold를 새 lifecycle gate로 연결 제외
- history: 4096개 image/per-frame IMU resource budget, 초과 시 명시 reset generation·reason. 기존10s 유효 interval 또는4096 sample보다 커질 non-keyframe merge는 oldest keyframe retirement 선택

## 재현·남은 한계

```bash
cmake --build build/refactor-native --target test_numerical_contracts --parallel 2
ctest --test-dir build/refactor-native -R '^NumericalContractsTest$' --output-on-failure
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  build/refactor-numerical/sanitize-complete/numerical-engine-sanitized
```

- formal ambient manifold/Minus/prior 전환, 전체 bias·covariance의 exact Jacobian, 센서 noise identification·observability·field accuracy는 미검증
- synthetic current-frame solve·reset·수치 회귀와 실제 phone·장기 tracking·독립 GT 정확도 구분
- scope의 replay·fresh WASM 영향은 통합 owner의 room1/room4·parity 결과와 함께 판정; 이 문서의18개 수치 회귀만으로 해당 gate 확대 불가
