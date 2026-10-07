# 최종 refactor 독립 검토

- 최종 판정: **APPROVE — R0–R6 refactor·실제 HTTPS7002·bounded virtual validation·첫 locked room2 native 확인 범위**. 새 P0/P1 blocker0. 아래 실제 source/runtime 근거에 한정; physical phone·full SLAM·production 정확도/성능 승인 제외
- Source/core SHA `b936982e5fea0c8ad0fb77e3e8b311770998763ff77ef2ef4bb082945866674b`, embedded JS `9973d58eca6f95fcf5fcaf2d553d02bc2215f419e4ba1fc1cc94ae417eb3014d`, native replay `03d06e68d1db9bfc79f56593a2b4bae74601daf89d7a3757a9d3cb5f8c990e23`; 최종10-suite native/SAN·module·run6·room2에서 같은 고정 identity
- Root의 후속 mandatory STANDARD88 cleanup·postcheck·delivery report는 이 승인 뒤 수행 가능. runtime source/config/artifact 변경 발생 시 해당 검증·source review 재개; 이 문서가 미래 다른 artifact를 승인하는 효력 없음
- 검토자: qr_completion / Sol6.1 Ultra. 추가 agent/model/Astra 호출0. 이번 검토의 product/test 수정·build·replay·서비스 조작0; 이 문서만 작성
- 적용: code-review·mobile-slam-validation의 source/evidence·severity 기준. source 검토와 owner 실행 근거, 과거 candidate와 최종 candidate, dataset과 실기기 검증 구분
- self-review 제외: 이 검토자가 작성한 QR/benchmark/initializer velocity는 독립 `qr-resume-peer-review.md`의 승인과 최종 runtime 증거를 인용. 자신의 구현을 독립 승인으로 표현 제외

| 범위 | 현재 판정 | 근거·후속 |
|---|---|---|
| q-sign 최소 수정 | **APPROVE — 정적 수학/범위**, 새P0/P1 발견0 | `factor-sign-source-frozen.json` SHA871e9a71… 실제4hash 일치; correlatedSPD·rawO_R/J→W 순서·π cut·Utility 무변경 독립 확인. fresh nativeNumerical34 GREEN; freshSAN102/10·run6runtime GREEN |
| SFM finite/cheirality/summary 경계 | **APPROVE — 정적 source/범위**, 새P0/P1 발견0 | freezee05e73c7… 실제3hash·test source 불변 확인. OLD6:4defectFAIL/2validPASS·exit1. fresh nativeSFM6 GREEN; freshSAN102/10·run6runtime GREEN; -1 Summary는 source inference만 |
| Calibration3 translation 상수 | **실제 causal A/B VERIFIED**, 최종 candidate 전체 승인과 구분 | 동일 compiled26ee/native53b3/WASM12af +동일4b8a raw 입력. native/WASM ATE.051298642/.051298417m, oldpred corrected evaluator169.80781m. metadata inverse에서 상수 도출, GT로 선택0 |
| Initializer image row→key window V | 독립 source 승인 인용 | `qr-resume-peer-review.md:90` 추가 검토: physical real-path RED10 velocities1.205–2.921m/s→owner Numerical32green; 최종native/SAN102·runtime에포함 |
| QR·allocation·benchmark profile | 독립 source 승인 인용 | 같은 peer 문서의 weighted row/implicitQ/rankq·kept no-cut·8M cells·failure prior release·per-instancefalse/actual options/count semantics 승인. 최종source affectedABI closure 재빌드·102native/SAN 실제완료 |
| Engine/Tracker·native adapter | 최종candidate 회귀/runtime VERIFIED | `engine-lane.md`, constant transition 독립 peer. IMU96rate/phase·rawray/gap/epoch/stale invalidation·source/served hash 동시 확인 |
| Worker/wrapper/mobile/server | 최종actualWorker/mock/HTTPS VERIFIED | transport/mobile/server lane. request/epoch/heap/future carry·actual public HTTPS/mobile mocks·owned lifecycle·trustedTLS·새hash 필요 |
| 승인된 전체 범위 | **APPROVE** | freshsource/기능·SAN·strict paired·defaultpaced/loss·realHTTPSmock·첫 locked room2 실제근거 아래확인. 물리/production/fullSLAM과구분 |

- q-sign source scope: `include/backend/factor/{integration_base,imu_factor,imu_factor_pnp}.h`, `tests/test_numerical_contracts.cpp`; frozen manifest SHA256 `871e9a715385294d2b4154f214d4712cb4db20563b4b2909ea6130ee5aa623d9`
- `canonicalOrientationErrorSign(e)`은 w≠0에서 shortest sign, w==0에서 first nonzero xyz positive tie. q/−q에서 sign과vec 동시 반전으로 같은 rawO_R 선택; exactπ에서 미분 가능 주장 없음
- IntegrationBase의 rawO_R만2·sign(e)·vec(e). P/V/Bias·covariance·normalization·기존 guards 변경 없음
- Main: pose_i O_R/O_R, speedbias_i O_R/gyro-bias, pose_j O_R/O_R. PnP: pose_i O_R/O_R, bias_i O_R/gyro-bias, pose_j O_R/O_R. **모든 computed rawO_R block에 동일 sign 후 기존W**; 나머지O_R zero blocks는zero 유지
- CorrelatedSPD는 W가R행을 다른 whitened행에 섞는 구조. W후 일부행 sign 처리·residual-only 수정 경로 없음
- 독립 cost oracle: rawPx1/Raxis.2/Vy−.3, L_Raxis,Px=.5에서 cost1+(0.2−0.5)^2+.09=1.18. 기존 한쪽q flip1.58은 오류; 새4representative 동일1.18 요구. exactπ x/y/z positive-axis tie cost1+(2−.5)^2+.09=3.34
- FD 보호: awayπ nonidentity right-Plus pose_i/j와 초기 main speed/bias9열·PnP bias6열,4qrepresentatives. exactπ는 cost-only; nonidentity 곱의 nearπ5.55e-17을 binary-exactw0으로 오인하지 않는 별도identity input
- `Utility::positify/Qleft/Qright`는 no-op/기존linear algebra 그대로. `include/utility/utility.h` SHAda35ccd7de1bb502bfa760980fdbde29d673910135a9b42c2db0fa5163d16c14, old archived source와byte 동일
- 정적 승인 한계: large theta의 unnormalized1차 bias correction/exact generalJacobian·formal ambient manifold/Minus·π derivative·actualwindow/trial sign incidence는 이번 수정 검증 제외. 기존 근사를 새로운 exact derivative 통과로 확대 제외
- q-sign correctness와 실제169m drift 원인 분리: published Eigen 대표값 crossing첫664는proxy이며 actualCereserror-q 관측 아님. calibration-only native/WASM physical gain은 별도의 measured ablation

- Final build closure: sourceb936982e…·web287c10de…; native50/SAN50/WASM29 repository object 중 q/SFM affected **23/23/13 fresh**, **27/27/16 unchanged retained**. Builder의 source/dependency/flags/object hash 근거로 분리; 모든50TU를이번에 다시compile했다는 주장 제외. 새datafield/layout0이며 staleaffectedconsumer 없이 closure 확인이 필요
- 조회한 freshnativeactual:102discovered/executed/passed,10CTest suites,skip/fail0·12.16s. Suite counts: Integration3/SlidingWindow5/Trajectory12/Config7/Measurement5/Parity3/Engine21/Numerical34/SFM6/NativeAdapter6
- 조회한 실제 WASM module smoke:9973d58e…·mandatorybindings/defaultinvalidpose/diagnosticdefaultOFF/warmup-resetJSON/benchmarkdefaultfalse·enable-reset-reconfigure-disable PASS. scope는 loader/publiccontract이며 tracking/GT/served hash 승인 제외
- SFM 동결: `build/refactor-diagnostic/room4-drift/sfm-fix/source-frozen.json` SHA256 `e05e73c7cb4290262d81cd554dfe24d94e4f50c716958dc6c93c300c7c7ff549`; header/cpp/test3 hash 일치·6test source는before와byte동일
- Triangulation: Pose/observation finite→design matrix finite→SVD→homogeneous finite/exactw!=0→finiteXYZ→양cameraZ>0. bool 성공일 때만 두기존caller의 state/position 저장. clamp·새EPS/minparallax/angle threshold 없음; 무한/뒤쪽점을 PnP/BA valid structure로 입력하는 이전 경로 차단
- Reprojection: 기존 wxyz QuaternionRotatePoint·translation·normalizedprojection 보존. cameraXYZ finite/positiveZ를 division 전에 검사하고 residual scalar finite만true; pinnedCeres double/Jet overload 사용으로 AutoDiff template 경계 유지. Jet derivative 모든성분의 일반 finite 보장이나 nearparallel conditioning 검증으로 확대 제외
- Reconstruction: actual NumResidualBlocks0이면 solve전false. Solve후 IsSolutionUsable·finite·nonnegative final_cost를 먼저 검사하고 기존 CONVERGENCE||final_cost<.02·1s budget 유지. FAILURE의 default−1 sentinel이 .02로 accepted되는 수식 경로 차단
- SFM 원본actual RED6은4defectFAIL/2goldenPASS, 실제 public construct와 직접factor에 대한 기대; Summary 값−1 자체는 capture되지 않았으므로 source-backed inference만 표기
- Independent 정상control:25nonplanarfront points·camera centers0/[1,0,0]·XYZ1e-7m·R/T1e-8, wxyzRz90°+translation residual[.1,.2]. Builder freshnativeactualGREEN6/6 확인(`final-physical/sfm6-green-excerpt.log`); reviewer는 테스트 실행0
- SFM 조치 범위는 initialization validity, calibration-only .0513m gain의 원인으로 주장 제외. Public construct/class fields/active SfM/PnP convention·IMU alignment·noise·solver/GT/caps 변화없음

- Accuracy causal 증거: `build/refactor-evaluation/calibration-only/causal-ablation-result.json`·paired metrics. 잘못된 t_ic[.0451759,+.0725159,−.0439599]→primary inverse[.0455748356497,−.0711618018380,−.0446812541171]
- corrected3t 이외 R/K/noise/solver/raw/scorer 변경0; legacycompiledcore에서 ATE169.747693m→.051298642m/native, WASM.051298417m. 기존 prediction의 body transform만 수정한169.807811m은 evaluator offset만으로 설명 불가를 실제 재평가로 확인
- 기존 output 기반 drift 보충: `build/refactor-diagnostic/room4-drift/first-drift-summary.json`. 첫 fresh body pose 기준 travel norm 대 GT norm(정렬/scale없이)은 잘못된 calibration에서1m 차이 첫frame394(+15.9s),10m 첫662(+29.3s), 최대bad398.18m 대GT2.253m. q proxy664 전에 이미심각한 metric divergence; canonicalization을 지배 원인으로 연결 제외
- corrected CAL-only travel norm max2.335m,1m gap0. 위marker는 시각적/진단 비교이며 기존 acceptance cap으로 대체하지 않음. 실제internalV/Bias/prior는 관측되지 않은 상태
- Room4는 diagnosis/selection/adaptation에 사용. 이 개선을 untouched held-out/physical phone/production 결과로 표현 제외. Quaternion/SFM runtime 변경은 해당 calibration-only ablation에 미포함

| 최종 gate | 현재 상태 | 확인할 실제 근거 |
|---|---|---|
| All-ABI source/native/WASM | **VERIFIED** | `final-physical/pre-build-source-and-ABI-closure.json`·3provenance, q/SFM/CAL/Tracker·native102/module·actualserved9973/coreb936. transitive23/23/13 fresh+27/27/16 verifiedretained |
| Native discovered suites | **VERIFIED102/102·10/10** | `build/refactor-evaluation/final-physical/{native-test-result,native-discovered-counts}.json`, actualCTest12.16s·skip0; Numerical34·SFM6 포함. 최종runtime와 별도 |
| ASan/UBSan/LSan | **VERIFIED102/102·10/10** | `final-physical/{sanitizer-test-result,refactor-sanitized-provenance}.json`, actualCTest176.61s·fail/skip0. affected23fresh+27verifiedunchanged, strictASan/UBSan/LSan; 모든thirdparty계측 주장 제외 |
| Full room1×3·room4 | **STRICT PASS** | run6 ALLexit0. room1×3:2821inputs/2776fresh/init45/drop0·state/reason/solver/epochdiff0; room4:2228/2148/init76/drop0·strictPASS. 이번candidate FUNCTION supplement 필요0, 과거run5strictFAIL보존 |
| Default paced/loss/mock | **VERIFIED** | source-time20Hz300input/286accepted/241fresh/14drop, benchmarkfalse. Loss350·blank100–111즉시null·recover121·gap/busy epoch2null, 실제HTTPSChrome permission/profile/cropgoldens/cleanup |
| New HTTPS7002 artifact | **VERIFIED RUNNING** | PID3002812/instancec98f10df76144a3c80311b09d1e0d5cf·chain3·HTTP34·actualGETSHA9973/healthb936. ordinary/Chrome127route +sameworkstationforced183.98.179.103 strictTLS; offLAN/phone미검증 |
| Locked room2 | **첫 nativeDEFAULT 확인완료** | protocolv2c98 firstlaunch1회·무튜닝/반복.2882processed/2838fresh/init44,2547uniqueGTassoc(89.7463%), ATE.066529314m·RPE.020884968m/.702677276°. WASMroom2미실행·productionthreshold없음 |
| Physicalphone·thermal·fieldGT·production | UNVERIFIED | 이번 desktop/dataset/mock 작업의 완료 주장 범위 밖 |

- 원래 strict solver summary equality 실패·발견된 이전오류·failed runs·backups·원본dataset·TLS·unrelated dirty work 보존. 비교 cap/GT/noise/tolerance를 결과에 맞게완화하지 않기
- R0 보존·변경소유: `build/refactor-baseline/{start-status,start-diff,source-fingerprints}`·owner source-freeze·`cleanup-report.md/closure-checklist.md` 근거. 시작전dirty/untracked·dataset/TLS/userlogs/vendor/generated/foreignservices는변경권한으로확대제외. 최종88scope STANDARD cleanup은이review후Root/owner확인으로닫기
- 단순화: 공통Engine rawnative adapter·단일 ordered IMU carry/queue, Worker request table·모든waiter cleanup, 공동covarianceWhitening·semanticordered QR prior·실패prior release, rawconstant/rayhistory 재사용, finiteSFM 경계. 추가DI/framework/model/dependency/GT hyperparameter 없이기존경계수정
- Source/LSP: final-closure clangd23.1.0 initializer/optimizer/SFM 실제3TU error0. OptimizerCeres loss ownership의기존unused-return warning1은전달소유권근거로해석; 모든lint/TypeScript/standaloneanalyzer 또는 warning0 통과주장제외
- License/모델/scope: pinnedVINS GPLv3 포함기존사용조건유지, 새license적합성/법률보장주장없음. 원요청Astra/ultra 계획검토1회기록·실행Sol6.1Ultra·추가Astra/model/agent호출0 유지
- 위 R0–R6 delivery 범위의 source/runtime 검토 APPROVE. field/physical phone·thermal·offLAN·fullSLAM/loop closure·production quality/latency·모든thirdparty계측·일반large-bias exactJacobian은 UNVERIFIED 또는 scope밖 유지


- Run6 actual: `build/refactor-validation/run-6/{validation-all,runner-completion,runner-identity,parity-contract,frozen-contract-sha256}.json`, runnerexit0/statusverified. Rootperformancegate는102native/SAN/module/LSP3/HTTPSidentity 후true, 검증중 다른compiler/replay/stagingCPU실행제외
- Strict contract SHA `53bba52381be83e932d9c9c3c94867c6e08436af4cb11d778f803feb7164ae76`, 2026-10-06T17:06:30.563Z **room4 전 동결**. 기존caps translationp95.001/max.01m·rotationp95.01/max.1°·init/coverage0차이 유지; 관측오차로caps확대0
- room1 세pair 동일: translationp95 5.48061758e-11/max1.80668682e-10m. within-platformtranslation0, rawacosrotationmetric self-floor 최대4.51774549e-6° 기록유지·0보정없음. room4p95 2.03820473e-11/max2.94184559e-11m; status/reason/solver/epoch모두diff0
- Image/F64 IMU·순서/firstfuturebracket/enginecarry·실제calibration/profile·seed0/CV1 일치: run6 pilot1–3/room4 `input-comparison.json` matchedtrue, frame2821/2228. Native/WASMsameinput parity를 독립물리정확도와혼동제외

| 최종 GT 관측 | Profile·coverage | SE(3) scale1 결과 | 해석 |
|---|---|---|---|
| room1×3 native/Worker | benchmarktrue/max10/time10s;2776fresh/2821,2719GTassoc | nativeATE.053166151m/APE1.203019476°;WASM동일범위 | 개발/선택 데이터,실기기/production아님 |
| room4 native/Worker | benchmarktrue/max10/time10s;2148fresh/2228,2108GTassoc | nativeATE.051330817m/APE1.251115244°;WASM동일범위 | diagnosis/adaptation데이터,held-out주장제외 |
| room2 first native | defaultfalse/.1s/max10;2838/2882(98.4733%),GT2547/2838(89.7463%) | ATE.066529314m/APE.898932969°,RPE1s.020884968m/.702677276° | first locked confirmation,291pose GT미연결·그구간정확도미측정,WASMroom2없음 |

- Room2 `protocol-v2.json` SHA `c98a336ab8e772abf13a07ff2cb22ac0c2d774d2f54d607e7c71b0c370476e17`는trajectory전고정; 준비문서의authorization=false/trajectory0은 당시역사기록으로보존. `first-native-default-launch.json`: output없는상태에서actual1회·coreb936/native03d06/configa69/scorer80414/source85match·raw e982ae3d…; `first-native-default-result.json`: processed2882/drop0/epochreset0/loss0·exit0/동일identity·무retune/반복
- Room2 GT one-to-one2547 unique associations, dtmax.006485939s≤고정.01s. 미연결291pose는score에포함/GT재사용/coverage완전으로표현제외. Sim3scale.981214550·ATE.062241821m는별도diagnostic, 주요SE3.066529314m 대체제외. 사전productionaccuracythreshold없음 → productionpass선언없음

| Default20Hz paced 초기화 후 | p50 | p95 | p99 |
|---|---:|---:|---:|
| Engine processing ms |29.0800|49.1950|62.0100|
| Request roundtrip ms |29.2949|49.4250|62.3298|

- paced300: accepted/completed286/fresh241/errors0/drop14(paced_worker_busy), wall14.980765s/dataset14.950629s. 초기화전45와후241, decode/copy/queue/IMUwait/engine/Worker/roundtrip/deadline 각각분리; poseAge0은input−posesensor timestamp차이, physicalcapture/render지연아님
- 기존oldpaced26.71/45.12ms와 source/calibration/geometry/profile조건차이 존재. 새속도gain·무drop지속20Hz·phonethermal/real-time보장주장없음; hashOFFsource-time desktop20Hz실험근거만
- Loss350input/350completed/drop0/fresh284. Blank100–111모두pose/time/null·map0·freshfalse,회복첫121. Gap/busy 결과 engineepoch2/frame_gap/nullpose/map0/falsefresh 확인; 과거run5row100의stale4feature/map157failure원본보존
- Replay6개eventinventory consoleError/pageError/failedRequest/badResponse0. Mock은grant/denied/error/sourceclock/axes/epoch·duplicateReply·crop/wxyz pixel golden 계약이며 짧은fakecamera가실제로trackingaccuracy를입증한근거로확대제외
- Real same-originprofile GET/HEAD200·HEADbody0/SHA d85f2dd4…·strictTLS1.3/systemtrust/configgolden·temporaryProfileRemovedtrue. `/public/...json`의realrequest이며routeintercept0; syntheticprofile는phonecalibration아님

- Actualserver: `server-final-9973-live-evidence.json`, `parent-live-verification.json`, run6preflighthealth에서PID3002812/UUIDc98f…/loader9973/sourceb936일치. Leaf+2intermediate chain3·strictSNI/systemtrust, HTTP34·favicon204,기존SSL/log/ownedbackup보존. Manualstop `bash scripts/ops/serve.sh stop`; 승인된요청대로서버leavealive
- Routing: run6 Chrome `MAP dev.serdic.com 127.0.0.1`·ordinaryOS existinghosts127override보존/수정0. 별도의sameworkstation forcedTCP183.98.179.103:7002는동일identity/verify0 확인. Legacypreflight publicDNS.verified만으로offLAN공개접속증명제외; 독립offLAN/phone은UNVERIFIED
- Resources: `owned-process-memory-summary.json` owner PID/startTicks/descendants1s sampling. maxima summedRSS1,424.699MiB는sharedpage중복가능; nativeHWM88.176–90.148MiB. Observer12.270804CPU/690.641819wall=1.7767%onecore,679samples·평균14.950/max33.268ms. 관측비용공개; 이값을WASMlinearheap/liveallocation/QR8Mcellsbudget/phonecapacity와등치제외
- Read-only reviewer는새CPUtests/replay/profiling·source/runtime/deploy조작0. 실제ownerlogs/artifacts·고정hash·firstlaunch근거를조회하고문서만갱신. Ownvelocity/QR/benchmarksource는이전독립peer승인인용유지


## 최종 STANDARD88 독립 closure

- 판정: **APPROVE — 문서·소유·보존·behavior lock·postcheck 범위**. 기존 THOROUGH R0–R6 승인 유지; 새 product/helper/test source 변경0, reviewer 추가 test/build/replay/service action0
- `ai-slop-cleaner` STANDARD writer=blank_transition_fix / 독립 reviewer=qr_completion. 계획17:28:59Z·behavior lock이 R0 delta proof/4 pass inspection 전에 작성된 chronology 확인
- unique88 scope/list216a78de…·before/after88 hash mismatch0. R0 overlap18은 task-local HEAD+startdiff 복원 hash가 R0 fingerprint18/18 일치; 기존27 excluded current hash 일치, owner-reported21 original-byte 한계 유지
- inherited optimizer TODO는 current415/R0before181 동일. eligible session delta 후보0으로 불필요 source 삭제·재명명·utility/threshold 변경0; 과거82/88 inventory·문서 byte snapshot·run3/4/5/SfM 실패 보존 확인
- 발견한 metadata-only README inventory stale hash1건은 writer가 current9cf9ee53…로 정정; priorc53/history 보존·changed-after-preparation=true. 실제source/behavior lock은 처음부터같았으며 source변경으로 집계제외. 정정 inventorySHAf688b116…·postplanSHAb1910150…와 실제88current/digest 재대조 mismatch0
- Root postcleanup 실제 CTest10/10·102 discovery·11.33s, JS78/Python7·fail/skip/TODO0·syntax0·88 scoped diff0. source85/web17/config1/validation35/registration2 및 native03d06/active9973 mismatch0 확인; source불변이므로 기존strictSAN102/run6/첫room2 추가반복 불필요
- 이번 reviewer 쓰기: 이 절의 review 결과만. cleanup writer 문서/metadata·source/artifact manifest·dataset·TLS·서비스 수정0. Root 최종 mode/service 인계 가능; physical/production/fullSLAM/offLAN·GTcoverage·original-byte 한계는 기존대로유지
