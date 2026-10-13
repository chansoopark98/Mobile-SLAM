# 기록 기반 visual factor 회복 수정 개발 플랜

- 날짜: 2026-10-12, Asia/Seoul
- 상태: Sol Ultra 작성; 연구·기록 분석 통합 후 Astra Ultra 검토 대상. 검토 해결 전 제품 코드 수정 금지
- 순서: 연구 → 기록·코드 분석 → 본 계획·[테스트 명세](2026-10-12-recorded-recovery-test-spec.md) 동결 → 개발 → 테스트 → 수정 → 결과 문서
- 실행: 일상적인 가역 결정은 자율 수행. 연구·분석·개발·테스트·수정·문서·구현 검토는 GPT-6.1 Sol Ultra; **개발 계획 리뷰만** GPT-6 Astra Ultra 전담
- 목표: 새 track의 실제 기하 관측을 현재 visual factor로 연결하고, 프레임 삭제 후 anchor/depth/index 계약을 보존. 녹화·합성 GT·TUM에서 수정 효과와 회귀를 분리 증명
- 제외: IMU 모델 재수정, guard 완화, stale pose 성공 처리, `INIT_DEPTH` 대입, 가짜 IMU, solver cap 축소, smoothing, PnP 기본 활성화, runtime dependency 추가, 무근거 전면 재설계

## 동결 기준·보존

- baseline: `build/research-20261012/baseline/manifest.json`, 2026-10-12T12:29:58Z의 **현재 미커밋 최적화 소스**. HEAD `98b396c29ce9f6994b738190094e6397a2c6ace8`만으로 재현 불가
- 보존: `source.tar.gz` 3,626개 entry, `git-diff.patch`, `git-status.txt`, 이전 검증, native 및 embedded WASM. 아카이브에 포함된 dependency/build byte와 실제 core source provenance는 구분
- baseline native SHA256: `43418010831c8b8747bf1e5c25ca88bc9815eb58132fb396c8449d27159bad7c`
- baseline `vio_engine.js` SHA256: `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`; manifest SHA256 `cfbb10bd0b1fafa223d2767a0bcfe8f48f1e3463b9e7e582446eb9b51bf8fd32`
- 기존 검증 재사용 조건: 해당 source·dependency·artifact·입력·실행 설정 SHA256 일치. 이전 통과 수를 새 후보 통과 수로 복사 금지
- 새 build·결과: `build/recovery-{native,wasm,sanitize}-20261012/`, `build/recovery-20261012/`. 기존 baseline·실패 후보 덮어쓰기 금지, build jobs 총2
- 데이터 원본 `/mnt/backup/SLAM`, 전화 보존본, TLS, 서버, ADB, 무관한 tracked/untracked 작업 보존. 조사에 불필요한 NAS 전체 스캔·다운로드 금지

## 현재 근거·불확실성

- [최적화 결과](2026-10-12-optimization-results.md), [실제 재시험](../recordings/2026-10-12-zflip6-after-optimization.md): 30.031325초 중 시간창 결과607/fresh574, Lost33/6구간, 엔진 p95 69.77ms, frontend41.38ms, Ceres29.96ms
- Lost33: `currentVisualFactors=0` 29개, `=1` 4개. 모든609행 IMU factor10개 수락·rejected0; Lost 행 Ceres/marginalization 미실행. 이번 1차 수정 대상은 visual support
- Lost 때 tracker 중앙값53, eligible bank80, immature146. 역사 track 수와 현 프레임에 연결된 잔존 factor 수는 다름
- `Estimator::solveOdometry`: triangulate → removeFailures → optimize. optimizer의 invalidDepth0/projectionReject0은 삭제된 upstream 삼각측량 실패가 없었다는 증거가 아님
- 여러 소비자가 `size>=2 && start_frame<WINDOW_SIZE-2` 사용. WINDOW_SIZE10에서 anchor8/9는 현재 관측이 있어도 제외
- `MARGIN_NEW`는 slot9를 지우고 slot10을9로 이동. 반복 시 새 track anchor9와 관측 수가 유지되어 연령만으로는 성숙 불가
- 영구 starvation 최소 fixture: **현 프레임까지 유지되는 성숙 track1개 + 새 stable ID≥20개 + OLD threshold 미만의 parallax**. 모든 성숙 ID가 없어지는 경우 `parallax_num==0`으로 OLD가 선택될 수 있으므로 그 경우만으로 영구 deadlock 주장 금지
- 보존 prefix의 frozen native 진단에서 current-support 실패6개는 모두 **OLD**. NEW starvation은 독립 재현된 잠재 결함이며 이번 phone Lost의 확정 원인 아님. 1차 admission은 OLD/NEW 양쪽의 관측 가능한 young track을 대상으로 함
- exact-double browser minute1 axis-corrected/current ON: fresh270/300, current 부족1·final 부족1, warning26, 최대 published step2.452m, reset0. 원래 잘못된 gyro를 그대로 재생한239/300·minute2 init_timeout은 별도 역사 control; GT 없음
- [동결 기록 분석](../research/2026-10-12-recorded-diagnosis.md): 개발 seed12617 원본359/400·warmup300/300 GT통과, 고정60% occlusion 파생324/400·warmup265/300(88.33%)·OLD factor실패35·Lost7구간·reset0, ATE49.835m/APE123.354°. 파생 GT실패 fixture를 그대로 보존·수정 효과 평가에 사용; held-out 아님
- `removeFront`는 anchor9가 삭제될 때 새 anchor의 깊이로 변환하지 않음. 기존 age 조건에서는 해당 depth가 solve되지 않았으므로 age 조건만 바꾸면 새 결함이 활성화됨
- `compensatedParallax2`의 `p_i_comp=p_i`: 실제 회전 보상 없음. keyframe churn 가설이며 별도 ablation 전 변경 근거로 사용하지 않음
- 최신 numeric 기록은 영상·GT 없음. 기존 600PGM은 두 accepted prefix이며 핵심 약41.12초 발산과 최신30초의 모든 이미지가 없음. 이를 완전 phone replay·현장 정확도·현재 Lost 완전 재현으로 보고 금지
- 잔여 bias 경고·출력 불연속에는 K/extrinsic·EIS/OIS·rolling shutter·노출/IMU 시차 영향이 미확정. young admission은 순수 회전의 깊이 비관측성이나 feature가 없는 blur를 해결하지 못함

## 연구에서 채택할 범위

- 상용 엔진: 공개된 입력·회복·품질 계약과 공개 구현을 구분. black box 성능 주장, 비공개 SLAM core의 추정 구현, 미확인 license를 이식 근거로 사용하지 않음
- Web engine: 현재 rVFC snapshot 소유권·zero-copy 전송·이벤트 제출은 이미 구현. Worker queue p95 0.863ms, main pixel read p95 23.50ms. capture Worker는 독립 latency 실험으로 허용하고 visual 회복 효과와 분리
- OpenVINS/Basalt: 기하 관측·조건수·재투영·keyframe 연결성 검사의 근거로 사용. rig별 depth/baseline 기본값은 Flip6 실측 기준으로 전용 금지
- RD-VIO 2D 회전/subframe, 성숙3D projection prior/reassociation, GPU fused capture는 별도 후보. 본 최소 후보가 실패하고 해당 원인이 기록으로 확인될 때 새 실험·계획 검토 후 진행
- 상세 근거: [상용 VIO](../research/2026-10-12-commercial-vio.md), [open VIO](../research/2026-10-12-open-vio.md), [web engine](../research/2026-10-12-web-engines.md). frozen 실제 C++ lifecycle probe는7개 독립 기대값 확인; 연구 microprobe와 제품 regression을 구분
- 소유 WASM의 raw WABT 검증은 SIMD opcode65,790개·nonshared memory256/512MiB·atomic0 확인. 이미 있는 SIMD를 새 최적화로 제안하지 않음; hotpath/phone JIT 속도·REA 전체 graph 성공은 미증명
- 연구의 pinned revision·공식 링크·license·채택/기각 근거는 `docs/research/2026-10-12-*.md`와 결과 문서에서 연결. 제품 runtime dependency 추가 없음

## 수학·데이터 계약

- 구조적 slot 조건: 관측≥2, `0<=start_frame`, `start_frame+size-1<=WINDOW_SIZE`. list 순서의 동일 feature가 count/getDepth/setDepth/clearDepth/optimizer/final filter/OLD marginalization에서 같은 slot 사용
- 양수 유한 ray는 공통 `FeaturePerFrame` 입력 경계에서 `f←f/f.z`, z=1로 정규화; zero/negative/nonfinite z 또는 나눗셈 후 비유한 값은 ingestion에서 거부. observation pixel·기존 normalized velocity는 별도 단위이므로 변경하지 않음. projection/final cull/map/PnP/initializer/reanchor는 이 canonical ray만 사용
- depth 유효성·solve status·현재 관측 유무를 구조적 slot 조건에 섞어 solve 도중 index가 바뀌게 하지 않음. 실패 표시 후 erase는 apply/marginalization 완료 뒤에만 수행
- `NUM_OF_FEATURES=1000` cap의 동일 prefix를 모든 소비자가 사용. 초과분의 depth round-trip·삭제가 prefix 인덱스를 변경하지 않음
- 삼각측량은 구조적으로 유효한 track을 후보로 순회. A2가 처음 만난 **미확정 anchor8/9**는 `geometry_pending` 표식으로 관리; OLD9→8→7 뒤에도 유지하여 legacy DLT로 우회 금지. true admission→usable solve→final 기하 통과가 모두 끝난 뒤만 해제. 기존 이미 solved 성숙 track 모델 변경은 A3로 분리
- young 기하 검사 초기 동결값: world bearing `b_k=R_WC,k normalize(f_k)`, `A=Σ(I-b_k b_kᵀ)`의 유한 고유값·`λ_min>0`, `κ(A)=λ_max/λ_min<=10,000`; 기존 DLT 결과의 모든 관측 camera depth `z_k>1e-10`, 유한 좌표, `sqrt((fxΔu)²+(fyΔv)²)<=1.5pixel`을 모든 view에 적용
- 10,000은 pinned OpenVINS의 condition 기본값, 1.5pixel은 현재 ProjectionFactor의 noise scale을 출발점으로 고정한 **후보값**. phone 적합성 검증값으로 주장 금지. 시험 결과를 본 뒤 임계값·warmup·seed·기준 변경 금지
- 반복·실시간 실행에 새 SVD framework를 추가하지 않음. 기존 DLT·Eigen 작은 고유값 계산 재사용. marked/deferred lifecycle만 추가 검사하고 기존 solved 성숙 경로 회귀를 측정
- metric depth를 결정할 수 없는 young track: `estimated_depth=-1, solve_flag=0` 유지, 관측 이력은 기존 window/ID/cap lifecycle 범위에서만 보존. factor·map·PnP solved landmark·fresh support에 포함 금지
- 구조적 slot table과 **solve 시작 때 고정한 admission mask**를 분리. addFeatureFactors→final cull→current support→apply→OLD marginalization 전체가 같은 mask 사용; positive depth 자체로 inactive slot 재활성화 금지. 최종 실패 mask는 admission의 부분집합만 만들고 새로운 factor를 추가하지 않음
- unknown rho0는 homogeneous Ceres **trial domain**과 구분. inactive deferred의 getDepth→apply 왕복은 depth/status를 보존하고 map/PnP 성공승격0; finite-positive-but-inactive slot도 factor/prior/current-support에서 제외. actual admitted landmark의 최종 음수·비유한·behind-camera는 기존 실패 처리 유지
- current support는 slot10의 **실제로 추가되고 final filter에도 남은 서로 다른 feature factor**만 집계. 기존 최소2 조건과 모든10IMU·SPD·유한 state guard 유지

## NEW anchor 변환

- `R_WC=R_WB R_BC`, `t_WC=p_WB+R_WB t_BC`, `p_W=R_WC,i(d_i f_i/f_i.z)+t_WC,i`
- old anchor9를 삭제하기 **전** old9/new10 camera pose를 저장; `p_Cj=R_WC,jᵀ(p_W-t_WC,j)`, 새 optical depth `d_j=p_Cj.z`
- 현재 구현은 slot10→9 copy 이후 removeFront 호출. caller가 old/new pose를 먼저 보존하고 명시 전달하거나 copy 전에 변환하여 덮어쓴 old9를 읽지 않음
- 유한 positive depth·새 anchor 관측의 재투영 일치가 확인된 solved landmark만 depth 보존. nonpositive·비유한·정의되지 않은 깊이는 unknown0으로 무효화하고 다음 관측에서 재삼각측량; 임의 depth clamp 금지
- 삭제 후 남은1관측은 구조적 optimizer slot에서 제외하되 유효 depth는 다음 관측의 초기값으로 보존 가능. old-anchor를 잃은 unknown은 unknown 유지
- NEW prior의 pose9 drop·pose10→9 매핑과 inverse-depth block 비잔존 여부를 진단·독립 테스트로 확인. depth block이 outgoing prior에 남는다면 단순 anchor 변환은 금지하고 해당 후보 중단; 범위를 넓혀 사전 검토
- OLD reanchor·initialization scale의 공통 membership 적용도 테스트. ray.z가1이 아닌 입력에서도 optical depth 의미 보존

## 구현 순서·소유권

| 단계 | 소유·파일 | 완료 근거 |
|---|---|---|
| R0 연구·분석 | 기존4개 연구 lane, 제품 write 없음 | 공식/pinned 근거·입력 manifest·원인/한계 문서 |
| P0 계획 검토 | Sol 본2문서; Astra 검토만 | 고위험 미해결0, ablation·수치 계약 동결 |
| T0 수학 회귀 선작성 | Sol Ultra backend lane: `tests/test_projection_trial_contracts.cpp`, `tests/test_numerical_contracts.cpp` | 현재 source에서 새 계약 red, 기존 suite baseline 보존 |
| D1 공통 membership·anchor | 같은 Sol Ultra backend lane: `include/frontend/feature_manager.h`, `src/frontend/feature_manager.cpp`, `src/backend/{estimator,optimizer}.cpp`, `src/frontend/initialization/initializer.cpp` | slot·NEW/OLD 수학·deferred 계약 green |
| D2 young 기하 admission | D1 동일 소유; diagnostic 필요 시 solver diagnostics/engine/binding까지 해당 lane 소유 | age8/9 actual current factors·기하 거부 이유 증명 |
| D3 capture 실험 | 독립 web lane: `web/js/{app,camera,capture-worker}.js` 및 별도 browser capture test; 기존 소유와 충돌 시 인계 | 아래 C1 identity·수명주기·실행 probe·시간 gate |
| V0a 공통 시간 adapter | Sol Ultra performance lane: `tests/audit_dataset_replay.cpp`, owned benchmark harness/build 결과만 | 동결A0 core·A2 core에 동일 adapter SHA, frozen binary 수치 parity 뒤 stage 실측 |
| V0 평가·수정 | 평가 lane: 명시한 `scripts/dev/` fixture/evaluation만; 제품 파일 충돌 시 root 인계 | 동결 A0/A1/A2 비교·GT·native/WASM·기록 검증 |
| V1 독립 리뷰 | Sol Ultra 제품 diff·수학·시간·결과 검토 | 재앵커/index/prior/coverage 주요 결함 해결 |
| F0 문서·승격 | root 산출물·manifest·결과 문서 소유 | 실패 포함 보고·실제 GET hash·rollback 보존 |

- 병렬 lane는 자기 파일만 수정하고 다른 lane의 tracked/untracked 작업을 되돌리지 않음. backend lane가 T0→D1→D2를 순차 소유; root는 통합·빌드·승격을 담당하고 제품 변경 필요 시 lane에 인계
- 제품 연결 변경은 membership·NEW reanchor·young admission을 서로 다른 불완전 상태로 배포하지 않음. PR/commit은 요청 범위에서만, Lore intent·Tested/Not-tested 보존
- 단순화: 중복 age predicate를 작은 공통 조건으로 통합; 새 feature repository·상태 framework·새 runtime 옵션 없음

## 동결 ablation·선정

| 후보 | 내용 | 결정 |
|---|---|---|
| A0 | 위 미커밋 baseline source/native/WASM | 모든 비교의 control |
| A1 | 동일 기존 age 정책 + 공통 구조 helper·NEW 수학 회귀 수리 | 동작 동등성/재앵커 단독 효과 검증용; 새 source build로 보존 |
| A2 | A1 + 기하 검증된 anchor8/9 + untouched deferred 보존 | 1차 최소 제품 후보 |
| A3 | A2 + 성숙 track의 저baseline/deferred 분리 | upstream 실패 삭제가 실제 dominant일 때만 별도 review·GT gate |
| K1 | 선택된 후보 + 회전 보상 keyframe parallax | R_WC 방향·회전 단독/translation oracle·held-out GT 증명 후만 시험 |
| C1 | 선택된 기하 후보 + VideoFrame→전용 capture Worker | 실제 target Chrome construct/transfer/draw/read probe·pixel identity·수명주기·직렬 timing 통과 시만 기본 선택 |

- trial별 별도 source patch/hash/build·설정 JSON으로 비교. 비교용 임시 flag는 반드시 실행 readback하고 최종 제품에는 불필요한 flag를 남기지 않음
- A1 동등성 → A2 target fixture → 기존 기록·TUM·fast GT 순으로 검증. A2가 수락되면 근거 없는 A3/K1 구현을 추가하지 않음
- 성능: 동일 fixture·seed·thread·solver budget, baseline/candidate 직렬 ABBA 2쌍. 고비용 numeric/membership JSON OFF 측정과 상세 진단 ON 분석을 분리; stage getter는 아래 동일 adapter로 별도 수집. warmup 포함·제외 p50/p95/p99·RSS 기록
- 기존 A0 native binary는 `process_ms`만 출력. stage 기준은 frozen source 격리 build+A2 build에 **동일 SHA adapter**를 연결하여 `getPipelineDiagnostics()` 실제 getter를 호출해 획득; A0 core source SHA 정확히 보존, adapter SHA 별도 기록. getter는 engine timed scope 밖에서 호출·buffer, 고비용 numeric diagnostics OFF. frozen binary와 fixed-work numeric parity 먼저 통과; baseline stage 없는 상태의 파생 추정·다른 run 혼합 금지
- 더 작은 대안은 동일 실제 WASM getter harness로 보존 A0/A2 loader를 각각 실행. 선택 경로·mode·getter workload를 비교 전 동결; 호출되지 않은 Ceres/marginalization stage는 null/count로 보고. comparable stage를 획득할 수 없으면 UNAVAILABLE이며 stage gate 통과 판정 금지
- 향상 주장: 같은 입력에서 current support·Lost/quality·GT가 함께 개선된 경우만. feature count 증가·유한 pose·unit branch green만으로 채택 금지

## 독립 capture 실험 C1

- rVFC callback에서 await 전에 `new VideoFrame(video,{timestamp:round(existingProxyMs*1000)})`, 즉시 transfer. 기존 source proxy timestamp 유지; 명시 timestamp는 metadata이며 physicalExposureVerified=false
- Worker OffscreenCanvas2D에서 기존 회전/crop와 integer gray77/150/29 경로를 재사용. Y plane/copyTo 또는 GPU gray 알고리즘 변경은 제외
- 최대 converting1+pending1 VideoFrame, overwrite/reject/error/stop/reconfigure에서 close. frameSeq·generation·config·geometry 일치하는 결과만 기존 pending-gray 제출; 구세대 결과가 기존 snapshot을 덮어쓰지 않음
- startup ready ACK·실제 construct→transfer→draw→getImageData probe 성공 뒤 활성화. 전송 전 초기화/생성 실패는 sync fallback; 전송 후 실패는 해당 요청 폐기, 나중 video를 과거 timestamp로 재캡처 금지. main/Worker clock origin 혼합 금지
- 기본 선택 gate: 독립 pixels oracle·busy/lifecycle·동일 accepted input GT 무회귀·actual Chrome probe·직렬 비교 모두 통과, main callback p95≥20% 감소 및 source→submit/engine/fresh-gap p95 악화≤5%. desktop/mock timing은 phone 향상으로 보고 금지
- 실패 시 synchronous 기본 유지·실험 patch/결과 보존. C1 통과 여부와 A2 기하 채택은 독립 결정, 한쪽 개선으로 다른 gate 실패를 상쇄하지 않음

## 수락·철회·후속 경계

- mandatory: [테스트 명세](2026-10-12-recorded-recovery-test-spec.md)의 수학/slot/GT/기록/native/WASM/도구 gate 모두 충족. 기존 native14suite·168tests는 기준 관측이며 새 discover 수 별도 기록
- TUM room1/room4 전체: A0 동일 profile별 body frame·1:1 timestamp·단일 SE(3) scale1. translation/rotation RMSE 악화≤5%, pose coverage 감소≤1percentage point; OFF와 현 App ON을 구분
- fast101/707/2203/8809와 신규 held-out 확인 seed: 기존 frozen acceptance·전체400/warmup300 분모 유지. 이전 seed8809는 이미 공개되어 이번 신규 held-out으로 재사용 금지
- 기하 GT recovery fixture: A0의 정확히 재현된 starvation 구간에서 actual current support 및 fresh coverage 개선·reset0·GT 통과. `A0 has no reproduced failure`이면 개선 판정 불가
- 기존 두 phone prefix: 동일 saved bytes/IMU/config/timestamps에서 current-feature loss·fresh coverage·jump·bias 경고 개선 또는 지표별 무회귀. baseline loss가 재현되면 최소1구간의 근거 있는 회복 개선 필요; 양쪽 무실패면 무회귀만 보고하고 신규 GT recovery 개선을 필수로 유지
- 성능 gate: paired median engine p95·frontend p95·backend p95가 각 A0 대비+5% 이내, p99+10% 이내; 일회 outlier면 동일 동결 순서1회 재검증, 반복 악화 시 후보 제외. 실제 phone latency 향상 주장은 새 물리 기록 전 보류
- 즉시 철회: NaN/Inf·negative/behind-camera map·slot mismatch·prior 주소 오류·unknown의 성공승격·IMU 수락 guard 완화·GT gate 실패·coverage 분모 변경·stale 성공표시
- 수정 한도는 오류의 근거로 결정. 후보 실패 시 소스 patch·원본 결과 보존, A0 복원, 동일 기준으로 수정→재평가. 시험 통과를 위해 threshold·GT association·warmup을 조정하지 않음
- 승격: 독립 리뷰 → fresh native/WASM source hash 일치 → isolated Worker/App smoke → manifest·artifact hash → 기존 deploy-candidate 원자적 승격·백업 → 실제 strict TLS GET·Worker readback. 시작시 health snapshot만으로 현재 제공 버전 판정 금지
- 문서 완료: source/artifact/input hash, 명령, 실패 후보, ablation, 품질·시간·coverage, 실제 phone 미검증 항목을 `docs/fixes/2026-10-12-recorded-recovery-results.md`에 기록
- 물리 수락: 새 image/IMU 원본과 같은 동작에서 Lost·manual reset·fresh gap·bias warning·jump·p95 age를 측정. 실측 GT 없는 near-zero Lost도 metric pose 정확도 또는 production 수락으로 확대하지 않음
