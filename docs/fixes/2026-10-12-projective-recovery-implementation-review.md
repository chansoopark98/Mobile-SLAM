# projective 회복 구현·테스트·결과 독립 리뷰

- 날짜: 2026-10-12, Asia/Seoul
- 담당: GPT-6.1 Sol Ultra. Astra 개발 계획 리뷰는 완료; 구현 리뷰에 Astra 사용0
- 상태: **H4 full-free solver의 고정10회 oracle 실패로 구현 gate 보류**. raw 수학 검토 통과와 solver 수락은 별도; helper/product source 작성0
- 승인: [개발 계획](2026-10-12-rotation-recovery-followup-plan.md) SHA `d0b8d42545e574ed1db0515d11f6c7689dbce57016b851ef808d5cceea1775b2`, [시험 명세](2026-10-12-rotation-recovery-followup-test-spec.md) SHA `ac104666e0004fcf4a1ecd0c1b49b1b939c8aab20944b673645333ad7e4d53dc`
- 범위: 아래 배정된 uncertainty helper·전용test·test등록만 제품 write; 나머지 제품은 read-only. 본 문서·`build/rotation-recovery-20261012/review/`에 근거 저장, 다른 lane 변경·원본·실패·서비스 보존
- 역할 변경: root가 본 Sol lane에 bounded `recovery_quality.h/.cpp`·전용test·test등록 작성을 배정. **이 helper는 내가 작성하고 자기 승인하지 않음**; backend 및 후속 독립 Sol 리뷰어/root·data가 검토. optimizer/engine/state-flow는 계속 read-only 독립 리뷰
- 기존 결과: A1 정상 경로의 고정 churn 동등성 확인; A2는 GT/coverage 실패로 기각. [기존 독립 리뷰](2026-10-12-recorded-recovery-implementation-review.md) 보존

## 구현 전 H4 동결 gate

| 항목 | 독립 검토 | 상태 |
|---|---|---|
| 입력 | analytic p/R/v/derivative·body IMU·nonzero extrinsic·source timestamp·회전/순수cameraT0/weakT/nearT | v3 actual IMU floor 독립 통과; v2 실패 보존 |
| free 상태 | T/V/Ba/Bg actual parameter 자유도·gauge/finite prior 선언; fixedGT translation probe와 구분 | 11pose/11speed-bias/8rho free·extrinsic만 constant 확인 |
| oracle | candidate code 전 analytic symmetry+독립 COD/SVD expected rank·observable 방향별 tolerance 동결 | raw12case 통과; fullsolver6case 모두 고정10회 실패 |
| nullspace | 미관측 V/Ba/scale/attitude/gauge를 GT 근접값으로 합격0; arbitraryρ에서 falseT/bias 정보 증가 차단 | raw analytic/COD 통과; no-prior relativeP UNAVAILABLE 확인 |
| uncertainty | last verified metric reference Pmm/Pkk/Pkm·nullbasis·relativeL·IMU F/Q·Lost/window 수명 | 대기 |
| 한계 | 고정T12-case와5-case prior probe만으로 full VIO oracle 통과 주장0 | 대기 |

## 안정화 source 검토표

| 항목 | 요구 계약 | 상태 |
|---|---|---|
| 정상 A1 | 정상 currentmetric≥2 경로 factor/order/pose·설정 동등, 추가projectivefactor0 | 대기 |
| pre-tri 삭제 | 정상 guard 전에 삭제되는 unknown 관측 staging; healthy pruned bank 동일·F/malformed ID 부활0 | 대기 |
| retry | 같은frame state/prior rollback·두solve 합계 iteration10/기존시간cap·최종실패 prior release | 대기 |
| 세 mask | 구조1000prefix·metric/provisional/excluded capture 고정·중간재정렬/erase0 | 대기 |
| localρ | finiteT에서free, solve시작/unknownprior/다음solveρ0; auditρ를state/map/prior로carry0 | 대기 |
| final support | current/사용allview rawL2≤1.5px·forwardh.z·signed/pole/finite; Cauchy와binarygate 구분 | 대기 |
| IMU | actual10·rejected0·endpoint/time/SPD/finite; 부족IMU를projective로우회0 | 대기 |
| prior | rawρ0 QR nullspace·D미전달·unknowntranslation정보0·provisionalobs소비/재사용0 | 대기 |
| anchor | OLD/NEW 실제margin/prior주소·homogeneousρ0·미소비관측/metric opticalZ | 대기 |
| 회복 상태 | unknown/newtrack persistent·age우회0·wronglegacyfinite2 복귀거부·certifiedmetric만종료 | 대기 |
| consistency | 정의된 correlated-estimate 비확률 score/weight·rank3·threshold, NIS확률주장0 | 대기 |
| uncertainty | 누적15D propagation-only·cross/null·relativeL range·Lost/reset수명·pinv0 허위확신0 | 대기 |
| output | actualnewtimestamp fullbody limited·종류/지원/cov/duration/schema·stale/gyro-onlysuccess0 | 대기 |

## 결과 수락 검토

- fixed churn12617 normal+limited **전체** full-body 출력,400/300분모. warmup≥280/300(93.3333%), actual current-support 실패·Lost 구간 각각A0보다감소·reset0
- ATE≤0.25m·rotationAPE≤5°·기존RPE, 공개 fast/seeds·source 동결 후 신규 heldout·TUM room1/room4 ON/OFF5%RMSE/1ppcoverage
- frame127 잘못된 대응과 frame300 올바른2D/no3D를 분리, actual visual support 없는 구간은 정당한 Lost 유지
- native VIOSystem/adapter·WASM Worker/Wrapper/App의 limited 출력 누락0, unknownstatus normal 강등0·fixed배열 OOB0
- 같은 adapterSHA/source/compile-record multiset·dependency·입력·serialABBA의 실제 stage, missing/null·표본수·init전후 구분
- sanitizer·전체native discover/assertion·browser TODO/skip0·clangd·freshWASM/actualWorker·artifact/served hash를 실제 실행으로 검증
- C1 capture는 pixel/lifecycle 수정 후에도 desktop latency gate 실패, sync 기본 유지. 별도실험으로 projective GT실패 상쇄0

## finding·판정 규칙

- H4 동결: v3 fixture SHA `5b7d03ce954afb5d6b5ecf93e699022aaf4f5cdfe6414a13715c30217efff762`, 독립 oracle SHA `38a7c810989179a6106c450477f61a1afa334ecfbc6706276e59f72f1e1e259c`. actual32kHz quadrature 수학 fixture이며 전화 IMU sampling 검증 아님
- H4 원본10회: finiteprior3case의 observable P/R/V/Ba/Bg 및 no-prior3case relativeR/Bg tolerance 실패. 제품 gate 보류; `replay/h4-independent-nonlinear-assessment.json`·원본 source/binary/results 보존
- root 승인 research-only500회 진단: physicalGT objective6.53e-8, finiteprior 상태 오차 고정 tolerance 안으로 수렴. 초기 IMU cost120323→최종 약1e-13–3.6e-12; conditioning/수렴 근거이며10회 gate 통과·cap 변경 근거 아님
- 부정 진단: 같은perturbed anchor의actual IMU rollout으로초기IMUcost≈2.8e-24가되어도10회oracle실패. DENSE_QR10 독립6case대조: DENSE_SCHUR10대비state원소차이최대2.21e-9·cost차이2.03e-7, 실패유지(`review/h4-qr-control-readback.json`); 단순초기IMU불일치나normal-equationroundoff만으로설명/수정완료주장0
- 수렴 원인 추론: pinned Ceres DOGLEG 최소μ1e-8; frozen canonicalJ의실제column-scaling식적용후최소양수normal eigenvalue는pure3.93e-10/weak3.46e-10. scaledD≈I인선형예에서약3.8%/3.3%만이동; late step 성공·radius유지와일치하나실제iteration재현아님(`review/h4-scaled-spectrum.json`). library/solver/cap변경채택0
- finite15D prior 독립 SciPy 차분4case: whitened local Jacobian 최대오차1.55e-7(기준1e-5), ambientLift×Plus 오차<1e-14. prior 좌표/부호 결함 발견0, `review/h4-initial-prior-review.json`; 장시간 진단을 구현 승인으로 승격0
- P2 data adapter(v2 SHA `f710db9e7f81db58ea2c9437f6ab484ca3d62a26a812667752ff59610af897d8`): 정상5.95s→Lost6.0s→첫Limited6.05s가anchor6.04s/duration0.01s를보고해도invalidContractRows0으로허용. 마지막검증metric시간을보존하지않아Lost경과누락검출불가; `review/adapter-anchor-lifetime-before.json` 재현·data owner수정요청, **해결 대기**
- 독립 uncertainty 준비 oracle:1D P/V/Ba subsystem의10×0.1s propagation·cross covariance를closed form과대조,오차<1e-14. 초기위치gauge는relativeL로상쇄, 미관측V0는시간×δV로남아UNAVAILABLE이어야함. 제품/H4통과증거아님
- helper pre-code API·15D/world/right/relative6/frame·rank/PSD·cross/null·attempt/prop시간·failure latch·수락budget·독립test목록을`review/helper-precode-contract.json`에동결. H4gate 전제품source0/build0
- 상대6D Jacobian을SciPy중앙차분과독립대조한최대오차5.05e-10(기준1e-8), world P/right rotation·anchor/currentcross부호확인. `review/relative-jacobian-oracle.json` 보존
- H4 계약 검토·raw proof 수용 전에 제품 수정 gate 통과 판정0. 안정화 source와 frozen artifact만 review
- 실제 contract 위반은 P1/P2·정확한 source line·영향·독립 재현·수정/재검증 상태 기록
- 단위 green·유한 Ceres·오차 일부 감소·coverage만으로 후보 수락0. 불합격 source/input/raw result 보존, 임계값 완화0
- 물리 Flip6 calibration·exposure 동기·열·fieldGT·실제 confidence calibration은 desktop/합성 결과와 별도 미검증
