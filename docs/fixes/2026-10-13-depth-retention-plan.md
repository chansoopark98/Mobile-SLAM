# Good history로 인증된 depth update 보존 계획

- 날짜: 2026-10-13, Asia/Seoul. Ralph iteration4
- 상태: **architect → critic 추가 계획 검토 전, 새 variant 구현 금지**
- 목적: 현재 outlier를 제외한 뒤에도 유효한 historical depth update까지 무조건 폐기하는 D2 분기를 수정
- 연결: `.omx/plans/prd-tracking-performance-20261013.md`, `.omx/plans/test-spec-tracking-performance-20261013.md` R1–R4

## 근거·대안

- D2 native16 suites/201 cases 통과. edge guard는 red 재현 후 수정
- healthy fast101 통과: warmup288/300, ATE0.04069m, rotation1.15096°, RPE0.03589m
- healthy fast8809 통과: warmup279/300, ATE0.16546m, rotation3.16875°, RPE0.07187m
- unoccluded12617: warmup270/300, ATE0.164m, rotation3.093°; **RPE0.100883m>0.10m 실패**, 계약에 따라 full matrix 중단. 실패 source/binary/raw 보존
- 가설: Ceres가 anchor/window pose와 함께 최적화한 candidate rho가 good history를 설명해도 old rho로 무조건 복구하면 유효 update를 버림
- 대안1 무조건 old rho: D2 control로 보존. 대안2 history가 독립 인증한 candidate만 유지: **선택**. 대안3 무조건 candidate 유지: bad current에 오염된 depth 위험으로 제외
- 모델 일관성 수정이며 RPE0.100883에 맞춘 threshold 조정이 아님. cap·분모·association 불변

## 수정 경계

- 구현2파일만: `src/backend/optimizer.cpp`, `tests/test_projection_trial_contracts.cpp`
- 이미 선택된 current-only bad-observation branch만 수정. pre-solve `solve_flag==1`·admitted positive depth·nonpending legacy·valid anchor 조건 유지
- current row flag=true를 먼저 적용. 그 row는 history 검사·current support·outgoing prior·future raw factors에서 계속 제외
- 최적화된 candidate rho가 finite/positive이고 기존 helpers로 다음을 모두 만족하면 **candidate 유지**: 실제 good views≥2, observable geometry κ≤10000, 모든 good view finite/forward/raw pixel L2≤1.5px
- candidate 인증은 현재 optimized camera poses와 실제 good history만 사용. flag/anchor/frame index·good count 계약은 D2 그대로
- history가 candidate를 인증하지 못하면 **old rho 복구 + D2 good-view cheirality guard**. old도 finite/forward 검사를 통과하지 못하면 기존 whole-feature failure
- weak/pure rotation history에 양수 candidate라는 이유만으로 metric 정보 추가0. unknown/young/solve_flag0/invalid 경로는 기존 whole failure
- helper·flag consumers·feature manager·estimator·D1·tracker·IMU 변경0. second solve·guess/ridge·bad row 해제·F/age/solver cap 변경0
- 최종 current<2이면 기존 rollback/prior release/Lost, conservative flags 유지. candidate 보존으로 current count를 늘리지 않음

## 독립 test oracle

| ID | 입력·직접 기대값 | 필수 결과 |
|---|---|---|
| R1 | known P_W·lateral motion·good history가 candidate depth를 정확히 설명, current ray만100px bad | optimized rho 유지, current flag/support 제외, bad prior row0 |
| R2 | pure rotation 또는 optical-axis translation으로 동일 bearing·history 비관측 | candidate finite/positive여도 old rho 복구, old cheirality 통과 |
| R3 | observable history가 old point를 설명, candidate는 good view에서 raw>1.5px | old rho 복구, bad row는 계속 제외 |
| R4 | optical-axis translation·candidate5m/old2m·good camera z>2m | history가 candidate를 인증하지 못함, old behind→whole failure |

- P_W/pose/ray와 rank 기대값은 구현을 복사하지 않고 직접 구성. helper 호출 결과를 test oracle로 삼지 않음
- 기존 restore-edge fixture는 R4처럼 history를 비관측으로 구성. 잘 관측된 candidate5m을 stale2m으로 버려야 한다는 oracle 폐기
- R1의 새 candidate-retention 행동만 red→green. R2–R4는 보존된 negative controls로 green 유지하며 불필요한 red를 만들지 않음. 실제 slot/ID·rho·flag·current count·raw prior row 검사
- O1–O8·unknown/young·invalid candidate/anchor·fallback cheirality·rollback 회귀도 유지

## 실행·수락

1. D2와 patch 직전 source/raw/hash 동결 → architect → critic 추가 계획 승인
2. optimizer-only controlled variant, R1–R4/O1–O8/unit·관련 sanitizer/clangd
3. sentinel fast101/8809/unoccluded12617/room1 ON. 실패하면 중단·원인 수정, full matrix 선행0
4. strict churn → 전체 fast/TUM ON/OFF → source/model/evaluator freeze 후 신규 heldout·fresh WASM/Worker/App·quiet ABBA
5. 독립 code review/architect·Ralph-owned cleanup 후 영향 regression 재실행

- strict churn≥280/300, current failure<35, Lost 구간<7, reset0, ATE≤0.25m/rotation≤5°/1s RPE≤0.10m/3° 유지
- full TUM 위치/회전 RMSE≤matched baseline×1.05·coverage 감소≤1pp, 기존 fast/unoccluded cap·전체 분모 유지
- 속도: 각 ABBA pair engine p95≥3% 감소·median≤0.97·p99≤1.10, 기존 stage/kernel gate 유지
- 실패 D2보다 나아진 것만으로 수락0. native/합성·WASM·실기기 증거 구분, PnP ON full-sequence/phone calibration·열·field GT 미검증 유지
