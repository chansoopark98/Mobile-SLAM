# projective 회복 후속 테스트 명세

- 날짜: 2026-10-12, Asia/Seoul
- 상태: [후속 계획](2026-10-12-rotation-recovery-followup-plan.md)의 Sol Ultra 명세, Astra 개발 계획 리뷰 전
- 보존: A0/A1/A2·원본·실패·고정 churn/공개 seeds 그대로. 새 출력 `build/rotation-recovery-20261012/`, source별 patch/artifact/입력 SHA·명령·raw outcome·skip/TODO 기록
- oracle: 구현을 복사하지 않은 analytic ray/pose/IMU·독립 Jacobian finite difference·COD/SVD·known physical GT. GT는 평가와 fixture 생성에만 사용

## T0 projective 수학·solver

| ID | 독립 입력·기대값 | 필수 결과 |
|---|---|---|
| H1 | nonidentity R_WB/R_BC·lever arm,ρ−0.2/−1e-12/0/1e-12/0.2 | 기존 general h residual·모든 local Jacobian FD 일치; pole/nonfinite/invalid ray 거부 |
| H2 | pure cameraT0·translation∥bearing·weak finiteT·nearT1e-12 | ρ0 Jtranslation0·Jρ=Dπt, rank0/1 정확. finiteT freeρ vs fixedρ0의1.00855° bias control 보존 |
| H3 | existing12-case Ceres probe: 고정T의 orientation/rho | 재현만으로 full VIO 통과 주장0; nearT arbitraryρ[-119,390]·parallel unobservableρ 분리 |
| H4 | **T/velocity/Ba/Bg 모두 free**의11frame analytic physical IMU+영상; pure cameraT0/nearzero/weak finiteT, metric prior유/무·초기오차 | 후보code전에fixtureJSON+analytic symmetry/독립COD rank/oracle동결. Ceres usable만통과0; 미관측값GT정답강요0·falseT/bias 자신감이면중단 |
| H5 | 잘못된 R prior·wronggyro·50px wrong correspondence·반복무늬 + 올바른 회전/weakT control | 잘못된 대응은 innovation/final support에서 거부, 올바른 실제 bearing 회복. F 완화0·gyro-only 성공0 |

- H4는 원본 sensor 단위/endpoint·현재 bias convention 유지, timestamp1:1, whitening/SPD guard 불변. 포즈/IMU 서로 불일치하는 시험으로 물리 정확도 주장 금지
- H4freeze: analytic excitation→weak/purephase의p/R/v/derivative·leverarm·IMU·landmarkray·dt/noise·finiteinitialprior mean/cov 또는prior없음을JSON기록. perturbP(.01,−.02,.015)m, R(.02,.01,−.015)rad, V(.02,−.01,.015)m/s, Ba(.01,−.01,.005)m/s², Bg(.001,−.002,.001)rad/s를동결; 후보결과로선택/수정0
- noiseless **observable 방향** oracle: P/V≤1e-5(m,m/s), rotation≤1e-6rad, Ba≤1e-4m/s²/Bg≤1e-5rad/s. nullspace `||Jn||/(||J||||n||)≤1e-10`, projectednormal/gradient차이≤1e-10, rankτ=ε×max(rows,cols)×σmax. 초기attitude/prior/gauge에따른전체expectedrank는독립analytic/COD로코드전freeze, 일반화한총null수상수0
- pureT0 canonicalρ0 visualRrank3·visualTrank0/Jρrank0 먼저증명; V0/Ba의6개null후보방향과추가roll/pitch/scale/gauge방향은조건별검증. 미관측bias/scale/translation은GT근접성대신null정보·relativecovUNAVAILABLE로판정; pose0gaugefix를물리certainty로취급0
- noisy case는고정development입력의오차/기각만보고하고statisticalconfidence calibration주장0. 확률calibration을추가하려면별도반복noise protocol/계획리뷰필요
- ρ는 signed trial domain free. no fake positive INIT_DEPTH·상하 clamp·rho ridge. 최종negative/pole/뒤쪽 bearing은 current physical support와 metric map에서 제외
- numerical nearT는 matrix 원소·raw info·rank로 검증; 동일 rotation matrix의 trace-angle roundoff를 실제 차이로 오해하지 않음

## T1 prior·관측 이력·anchor

| ID | 독립 검증 | 필수 결과 |
|---|---|---|
| Q1 | rawQR vs independent COD/SVD `Jposeᵀ(I−JρJρ⁺)Jpose` 5-case | 차이≤1e-10, zeroJρ P=I, nonzero2view nuisance1개 제거, 동일 residual normal/gradient |
| Q2 | unknown priorρ0 vs arbitrary nearT optimizedρ | canonical prior translation block0·carry control의1.389e10 실패 보존; solver D/epsilon prior전달0 |
| Q3 | finiteT·finiteGTdepth에서ρ0 marginal linearization+ρ elimination vs fixedρ0 rotation constraint | parallax nuisance 제거 여부·rotation bias·선형근사 범위 보고; 둘을 같은 모델로 주장0 |
| Q4 | 모든 dropped/kept pose gauge·relative yaw/position nullspace, sign quaternion, ambient/local Jacobian | raw data nullspace 보존, minnorm/Schur equivalence는 필요한 rank 조건 명시 |
| Q5 | 실제 OLD→NEW→OLD optimize/slide/prior + rejected frame + 다음 복구 | pose/속도 주소·epoch·timestamp·known map 일치, stale prior0·ρ prior block0 |
| Q6 | provisional 관측 prior흡수 성공/실패·metric 승격 전후 | 흡수된 observation ID는 미래 raw factor0, 실패 때 소비0; 과거 measurement 재사용으로 info 증가0 |
| Q7 | homogeneousρ0/finite/signed reanchor + nonzero lever arm·ray scale0.2/1/7 | ρ0에서1/ρ0, optical direction correct·finitefront h gate, certified metric만depth 변환 |

- prior는 선택한 canonicalρ0 policy의 국소 근사. 임의의 optimizedρ0 reset을 전역 FEJ theorem으로 표현 금지
- 새 관측 이력 소비 계약은 provisional 기원 track에 적용; A1 정상 legacy 경로의 변경0을 source·수치 ablation으로 확인

## T2 slot·commit·품질 상태

- 구조 slot0/7/8/9/10·관측1/2/3·cap999/1000/1001·unknown/positiveinactive/failed 혼합. metric/projective/excluded ID·순서·mask 동일
- mask 생성 후 status를 바꾸어도 add/final/apply/marginalization admission 추가0. 실패 landmark만 status2; unknown estimated_depth−1/solve0 보존
- 매 solveρ0 init·priorρ0 init·다음 solveρ0 init을 각각 검사. audit lastFitρ=390/−119를 넣어도 실제 next input/map/prior 변경0
- provisionalρ positive finite이지만 κ>10000인 case: projective support는 검증 가능, metric count/map/PnP/solve1은0. κ9990/10010·L2 1.4/1.6px·all-view behindcamera 경계 유지
- current count는 실제 slot10 residual 및 final retained ID. 역사 factor·feature count·nearzero rho·unknown map으로 support 생성0
- rawprojectivefinal L2 1.4/1.6px·current/과거view각각·forwardh.z경계, Cauchy큰잔차도양수weight임을독립확인. binarysupport는L2≤1.5gate만, rankrow는동일통과row×기존CauchyIRLSweight; robust비용감소만current성공0
- actualIMU9/10·rejected1/0·endpointgap·역순/NaN·nonSPD whitening·duration불일치부정입력: projective2D가충분해도fallbackfresh0. 기존IMU/finiteguard우회0
- 정상 pre-guard 통과 시 baselineA1과 같은 factor/solve/순서. normal failure 뒤 same-frame retry에서 state rollback·original prior/address·남은budget 검증
- recovery mode 중 unknown/new track은 persistent provisional; age9→8→7로 metric우회0. 기존 legacyfinite2개가 잘못된 depth/fit인 복귀 counterexample은 mode종료·timer reset 금지; mode밖 healthyA1 동일
- total iteration≤10, total Ceres≤profile cap; budget소진이면 Lost. 두 solver retry로0.06s×2 허용0
- actualvisualorientation rawrank3+currentfinalprojective≥2+rotationConsistencyScore≤16.2663+finitecumulativepropagationuncertainty+duration모두요구. correlatedjointR/IMU에NIS/χ²alarm probability주장0; e/weight/linearization/단위각각oracle
- current sourceTimestamp의 새 pose만 limited fresh. metricTranslationSupported/rotationVisualSupported·factor종류·covariance·limitedDurationS·reason 필드 누락은 fail
- covariance는lastverifiedmetric anchor에대해propagation-only15D jointPkk/Pmm/Pkm와nullbasis로계산. arbitraryρ큰Jtranslation/DOGLEG/correlatedvisual로축소0; relativeL이관측null을소거못하면pseudoinverse0을작은σ로보고0/UNAVAILABLE
- lastmetric이후Lost→fallback재진입·OLD/NEW·anchorwindow밖제거·반복성공limited에서도동일anchor/time/crosscov유지. finiteinitialV/Bprior유/무·globalgauge불변·crosscov생략실패·누적drift/초기snapshot오류부정oracle
- lastmetric부터limited+Lost2.0s 경계와 `3sqrtλmax(positionΣ)0.25m`, `3sqrtλmax(rotationΣ)5π/180rad` 경계양쪽; unavailable/비SPD/gauge미제거fail. 통계적confidence/실제오차상한주장0
- metric 인증 current≥2와 정상 guard 실제 복구 때만 limited 종료; stale pose를 normal 또는 limited로 재발행0
- schema version/호환: eligible는structural, inactive는invalidDepth에 포함될 수 있음. explicit actualmetric/projective counters와각분모 별도; oldnumeric 의미를 조용히 바꾸지 않음
- 실제native VIOSystem→adapter와browserWorker→Wrapper→App각경로에서limited/fullbody output보존·status분리 확인. fixedsize status-count배열OOB0/limitedpose누락0/unknownstatusnormal강등0; 새adapterSHA면A0원본parity다시수행

## T3 고정 replay·독립 GT

- 개발 churn12617 기본/60%occlusion48frame 파생·analytic IMU·GT·caps 고정. A0/A1/A2 실패를 baseline으로 보존; GT는 estimator가 읽지 않음
- A0/A1:324/400·warm265/300·ATE49.835m/APE123.354°. A2:274/400·warm215/300·ATE0.31769m/APE13.265°. 오차 한쪽 개선만 합격0
- A3gate: churn전체400/warm300분모, **warmup≥280/300(93.3333%=A0+5pp)**·actualcurrent-support실패수와Lost구간수각각A0보다감소·reset0·translationATE≤0.25m/rotationAPE≤5°/기존RPEcap. normal+limited모든fullbody포함;80%/265개만으론fail,93.33%를nearzero로표현0
- frame125/126 prior release 전후,127 잘못된 correspondence,138/140회복,300 올바른2D/current3D0의 membership+innovation+state를같은timestamp로 비교
- 선택K: 실제selectorNEW·성공projective/metric<2일때만effectiveOLD. 같은camera trajectory에서2viewκ61k→5–10view metriccert 여부·pureRnevercert·requested/effective margin·prior주소/이력소비 검증; 조건부OLD가GT/coverage/속도gate실패하면K기각
- unoccluded12617 원본·fast101/707/2203/8809 원래 frozen cap·calibration·threads·rotation ON 그대로. 전체·warmup·firstfresh·Lost/quality·reset·ATE/APE/RPE 보고
- full TUM room1 2821/room4 2228, ON/OFF 각 matched A0/A1: body frame·일대일 timestamp·단일 SE3 scale1, translation/rotationRMSE 각각≤baseline×1.05, coverage 감소≤1pp
- source·모델·임계값·평가기 동결 뒤 **새 heldout seed** 생성. 기존12617/8809는 heldout아님. 실패seed 보존, 성공seed만선택0
- phone prefix exactdouble·axisderivative·mobile0.06/10/features120/min30·동일currentpolicy. fresh/current/final실패·jump·bias·reset 무회귀, 가능한 손실 최소1구간 실제회복; GT없는 정확도 주장0
- 최신 numeric영상0·기존600PGM prefix 한계 유지. 최신Lost33·약41.12초 미보존발산의 완전재현 주장0

## T4 속도·경로·승격

- A1 정상 path numeric 동등성 먼저: 같은 입력/상태/epoch/reason/solver·pose tolerance, 실제 추가projectivefactor0. property/JSON 존재만으로 경로 실행 주장0
- freeρ fallback 추가비용은 실제 stagegetter의 같은adapter/flags/sourcehash serialABBA로 측정. frontend 추가LK0, current2D좋은lateframe과wrongmatchframe분리
- native fixed-work10s/10iter와mobile0.06s 별도. all-frame engine/frontend/backend p95≤A0×1.05·p99≤A0×1.10; matchedfresh/samework는 보조로만 사용
- 원A0↔instrumentedA0 pilot 400/339·pose원소0 parity 재사용은 SHA 일치 때만. compile39records/34uniquefiles의 fullrecord multiset·Ceres archive·adapterSHA확인
- triangulation timer 없음은UNAVAILABLE, Ceres/marginalizationnotrun은null/count. stagep95 합산·missing0 speedgain0
- 영향 unit·full native discover·ASan/UBSan/LSan·browser TODO/skip0·syntax·genuineclangd·freshWASM binding/Worker/App 실제 실행. 미실시를통과로기록0
- canonicalpixels·IMUbatch/시간/seed/calibration/실행profile·artifactSHA가동일한native/WASM 비교, limited reason/count/covariance도비교
- C1capture는독립opt-in/syncdefault; 별도capture합격으로A3GT실패상쇄0. benchmark중다른build/replay0
- 독립Sol구현/결과review 뒤모든gate통과만승격. raw실패·ablation·exactcommand/source/input/artifact/servedhash·물리phone미검증기록

## 중단·계획 재검토

- fullfreeT/Bias에서falseconfidence/pose drift, provisionalrho carry, nullspace손상·prior중복관측·negative metric map·timer/stale성공, 또는고정GT/coverage/speed실패면candidate기각
- κ/reprojection·GTcap·association/warmup·seed 기준을완화하여통과시키지않음. 새구조·프로비저널prior정책변경이필요하면Sol새계획+Astra계획리뷰후구현
- capture benchmark실행창동안추가Ceresprobe/build/replay금지; 현단계는문서작성만
