# 현재 관측 거부와 metric landmark 수명 분리 계획

- 날짜: 2026-10-13, Asia/Seoul. Ralph iteration3
- 상태: **architect → critic 재검토 전. 구현·GT 수락 문서가 아님**
- 목표: 현재 raw outlier를 support에서 제외하되, 이전에 유효했던 metric depth와 관측 이력을 보존
- 계획·시험: `.omx/plans/prd-tracking-performance-20261013.md`, `.omx/plans/test-spec-tracking-performance-20261013.md` O1–O8

## 실패 근거와 선택

- D0는 현재 reprojection>1.5px을 landmark 전체 inverse depth−1/solve fail로 바꾸어 apply→removeFailures→map/prior에서 이력을 잃음. support8→7은 올바르지만 전체 삭제가 필요하다는 근거는 없었음
- full fixed matrix 실패: fast101 warmup199/300·ATE1.874m/rotation49.97°, unoccluded12617 ATE0.441m/9.12°, fast8809 rotation5.14°, TUM room1 ATE0.0625m/1.62°. churn의 부분 개선으로 healthy 회귀를 상쇄하지 않음
- 대안1 전체 landmark cull: 위 GT 회귀로 기각. 대안2 해당 stored observation만 제외: **선택**. 대안3 robust loss만 사용: raw outlier가 support로 계속 집계되므로 기각
- 가설: 거부 단위가 너무 넓어 유효한 과거 정보가 사라짐. 관측 단위 거부의 GT/coverage 효과는 동일 데이터로 확인해야 함
- H4/LM·legacy-age relaxation·gyro warp 실패는 별도 보존. 이번 후보에 solver/F/threshold/age/feature 수 변경을 섞지 않음

## 수정 범위·보존

- 제품4파일: `include/frontend/feature_manager.h`, `src/frontend/feature_manager.cpp`, `src/backend/optimizer.cpp`, `src/backend/estimator.cpp`의 `getSolvedFeatures` 최소 소비 검사
- 회귀2파일: `tests/test_projection_trial_contracts.cpp`, `tests/test_geometry_reuse.cpp`. 기존 target 재사용, 새 dependency0
- root가 실패 D0 source/binary/GT/명령과 patch 직전 source/hash를 별도 archive/manifest로 보존한 뒤 수정
- 원본 dataset·phone 기록·D1 IMU/whitening/projection/SVD 재사용·provenance/tooling·다른 lane 변경 보존
- executor1명이 제품4파일 연결을 소유. 계획 architect/critic와 구현/결과 reviewer는 독립 agent. build jobs 총2, 성능 측정 중 다른 build/replay0

## 상태·실패 분류

- `FeaturePerFrame`에 `bool projection_outlier=false` 추가. 실제 `(featureID,frame)` 관측에 귀속되며 landmark 품질·깊이 인증과 구분
- true는 같은 관측 수명 동안 해제0. vector 복사/erase/OLD/NEW는 flag를 관측과 함께 이동. 다음 실제 frame에 append한 관측은 false이며 이전 true를 덮어쓰지 않음
- `isParameterCandidate`, getFeatureCount/getDepthVector/setDepth의 structural1000prefix·ID 순서·slot 조건 유지. good 관측 수로 slot 압축·vector 중간 erase0
- `isDepthAdmitted`는 기존 조건에 nonempty·front not flagged·finite/forward anchor ray를 추가. flagged front의 finite depth를 map용 metric으로 인증0
- **관측만 거부할 조건**: pre-solve `solve_flag==1`인 인증된 depth가 admitted/finite/positive, nonpending legacy 경로, 유효 anchor, candidate depth와 나머지 good 관측의 기존 finite/cheirality 검사 통과. 유일한 실패가 실제 current factor의 finite/forward raw pixel error>1.5px. 단순 positive/isDepthAdmitted인 `solve_flag==0`의 새 triangulation에는 보존 경로를 적용하지 않음
- 위 조건이면 current row flag=true, current count 제외, manager의 pre-solve `estimated_depth`로 rho 복구(`1/estimated_depth`). solve/geometry 상태·para slot 주소·ID 순서 보존. 추가 age/rank 문턱·guessed depth0
- invalid rho/world/anchor·nonfinite·behind camera·young geometry_pending의 기하 실패는 기존 whole-feature failure 유지. marked history가 있다는 이유만으로 깊이 무효화0
- 현재 outlier는 식별되기 전 trial solve에 영향을 줄 수 있음. second solve·extrapolation·smoothing을 추가하지 않고 full GT에서 그 영향을 검증

## 소비자·frame index

| 소비자 | 제외 규칙 | 유지할 계약 |
|---|---|---|
| `addFeatureFactors` | good anchor 확인, flagged target residual 제외 | structural feature_index 유지. `imu_j++`/frame 전진 후 skip하여 실제 pose pair 보존 |
| `filterFinalLandmarks` | 기존 flagged row는 skip/current0, 새 current outlier는 위 조건으로 flag | skip에도 실제 frame index 전진. vector 마지막 원소를 current로 간주하지 않음 |
| `addFeatureFactorsForMarginalization` | flagged raw row 제외, good history residual 유지 | pose/depth block 주소·drop/remap 유지. 역사 depth block의 prior 잔존 허용 |
| `hasObservableGeometry` | good anchor·실제 good views≥2, good row만 정보 누적 | raw row마다 frame 전진, κ≤10000·rotation 검사 불변 |
| `hasConsistentReprojection` | good anchor·good row만 검사 | fx/fy·1.5px·forward gate 불변. good ray를 잘못된 frame pose에 연결0 |
| `triangulateAcrossAllViews` | unknown에 good actual views≥2, flagged row DLT 입력0 | A rows=2×good count, SVD workspace key=good count−1, projection cache는 실제 anchor/view frame key |
| `getCorresponding`/parallax | pair 중 하나라도 flagged이면 제외 | 통과 pair만 분모 증가, bounds·연속 ID 계약 유지 |
| `Estimator::getSolvedFeatures` | 저장된 CURRENT row가 flagged인 track을 반환하지 않음 | 해당 frame의 PnP matching cache가 같은 bad ID를 재사용0. good current 또는 history-only의 기존 반환 계약은 보존 |
| depth apply/map/export | 역사 valid depth 보존, bad anchor admission0 | historical metric point 잔존 허용. flagged current는 visual support/raw measurement0 |

- prior 증명 대상은 **거부된 raw row contribution0**. good history가 있어 같은 landmark depth block이 prior에 남는 것은 허용. D0의 “depth slot 없음/map7” oracle는 교체
- incoming prior를 재분해하여 과거 미표시 outlier를 소급 제거하지 않음. 이번에 새 flag가 된 row는 outgoing prior와 future raw factors에서 제외. Lost/reset의 기존 prior release 유지
- flag는 nonlinear solver의 current rejection에서만 생성. clearState/reset/new track으로 관측 수명 종료. initializer 재진입 시 기존 marked 관측이 재사용되면 scope를 root에 보고하고 gate 실패
- 목표 profile은 PnP OFF. `getSolvedFeatures`의 current-cache 검사는 같은 frame의 matching 이전 getter 반환을 보수적으로 제한하는 최소 수리이며 Engine CPP 변경0. 별도 `getSlidingWindowMapPoints`는 historical metric landmark를 유지
- PnP ON의 full-sequence/GT는 미검증이며 이번 target acceptance에서 제외. getter 단위 검사로 PnP ON 정확도·tracking 개선을 주장하지 않음

## anchor·rollback 수명

- anchor는 vector front의 실제 관측. flagged anchor를 virtual ray로 바꾸거나 start_frame 변경 없이 중간 good row를 anchor로 선택0
- `removeBackShiftDepth`·`removeBack`·`removeFront`: erase 후 새 front가 flagged이면 depth fail/보수 제외. 기존 anchor가 bad이면 world point/reanchor 계산0
- 정상 good→good reanchor와 다른 flags의 이동은 기존 계약 유지. bad nonanchor 제거가 남은 flags를 해제하지 않음. empty/singleview 검사 전 front 접근0
- 최종 good current<2이면 기존 pose/speed/depth rollback·prior release·Lost. **bad flags만 보수적으로 유지**, 성공 pose 공개0
- 같은 stored bad ray를 수정하여 회복시키지 않음. 다음 실제 frame의 false/new good row가 strict current를 통과할 때 회복; 이전 bad row는 future/prior에서 영구 제외

## 실행·검증 순서

1. D0/current source 동결·manifest·plan/test SHA → architect → critic 독립 승인
2. O1–O8을 D0에서 red 확인. 관측ID/실제 pose pair·depth 보존·prior row·OLD/NEW·new append·good count/SVD key 검증
3. flag/consumer/lifecycle를 작은 patch로 연결. iterator/slot/order·D1 변경 보존
4. 영향 unit/geometry·sanitizer/clangd → healthy sentinel fast101/8809/unoccluded12617/room1 ON. 실패하면 원인 수정 후 반복
5. strict churn → full fast/TUM ON/OFF → source freeze 후 신규 heldout·fresh WASM/Worker/App → quiet ABBA. fixed-work10s/10iter와 mobile60ms/10iter 분리
6. 독립 code review/architect → Ralph-owned deslop → 영향 regression 재실행. 전체 gate와 미검증 조건 문서화

## 수락·실패 gate

- churn: warmup≥280/300, current failure<35, Lost 구간<7, reset0, body SE3 scale1 ATE≤0.25m/rotation≤5°/1s RPE≤0.10m/3°. 전체400/warmup300·association 불변
- healthy fast/unoccluded 원 caps, full TUM room1/4 matched 시작 dirty ON/OFF 위치·회전 RMSE≤1.05배·coverage 감소≤1pp. 실패 D0보다 나아진 것만으로 수락0
- D1 parity·속도는 별도: engine p95 각 ABBA pair≥3% 감소/median≤0.97·p99≤1.10, stage caps·targeted kernel≥10% 유지. missing stage를0으로 보정0
- 실패: bad row의 current/prior 재사용, frame/slot 오류, flagged anchor 인증, good count<2의 fake depth, 역사 정보 전체 삭제, rollback 중 flags 소실, GT/coverage/RPE/speed 미달
- 새 dependency·F/RANSAC/threshold·solver cap·feature 수·whole map cull·smoothing/extrapolation·age 완화0. 실패 source/input/output 보존. 전체 목표 미달이면 NOT_ACCEPTED
- native/합성·WASM·phone 증거 분리. 신규 실기기·실측 calibration·exposure/IMU 동기·열·field GT는 미검증이며 desktop GT로 대체0
