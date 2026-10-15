# 저특징·바닥 후보 plan-v2 독립 Critic 리뷰

- 판정: **REQUEST CHANGES — 수락 분모 1건만 남음**. 아래 R1을 고정하면 구현 경계에 추가 차단점 없음.
- 대상: `build/low-feature-20261014/plan-v2.md`, SHA256 `46251e8d3b69f2dbf926e0f581dfc74747096111a490dcd045ef364dd4283f27` 확인.
- 독립성: `architecture-review-v2` 및 다른 review 출력은 읽지 않음. 고정 v2, 직접 작성한 v1 문제 목록, 기존 source/analysis, 실제 floor fixture/report 및 native baseline receipt만 사용.
- 변경: 이 새 리뷰 문서만. 제품·계획·서비스·fixture 변경 및 시험 재실행 없음.

## C1–C5 반영 확인

| v1 항목 | v2 확인 | 판정 |
| --- | --- | --- |
| C1 원본 IMU gap | raw-left lineage, 원본→원본/bracket 검사, `.20s`를 `.05s` frame으로 분할해도 거부, 잘못된 bracket에서 재seed 금지, 기존 metric .5s 정책 유지 | 해소 |
| C2 seed/core interface | read-only `Estimator::getGravity()` 소유권, snapshot/frame/IMU endpoint 동일시각, publication P/R 일치, reset generation, PnP/quality 제거, 기본 side-channel 계산·표시, Worker 입력거부 시 명시 invalidate API | 해소 |
| C3 deadline/plane ownership | source .5s + display .1s의 최대 .6s 의미, Worker 단일 plane state, observed/retained schema, source timestamp TTL, fit skip/fail/모호성/error 전이, Wrapper/App 경계 | 해소 |
| C4 실제 기능/오차 수락 | 실제 floor-visible fixture 신규 getter→WASM→Worker→App, GT normal/offset gate, floor blank/TTL, prediction eligible100%와 .25m/5° 구간 gate | 수락 분모 R1만 남음 |
| C5 비간섭 | fixed-work old/new의 map/window/bias/prior/solver snapshot 및 read-only 호출 전후 비교, timer 제외, runtime wall-cap 차이 분리 | 해소 |

- native baseline receipt에 격리 source SHA `6def3e51872beb7151c2dc6efd7269a47cc1ac665cf7c30904d1925301004cf3`, 16/16 CTest·210 GoogleTest/skip0, immutable comparison artifact와 실패 시도 보존이 기록되어 있음. 이 리뷰에서 시험을 재실행했다는 의미는 아님.
- side-channel은 기본 표시하되 별도 prediction GT gate 실패 시 기본 표시 승격을 보류하도록 명시되어 있으므로, 미검증 translation을 무조건 성공으로 채택하는 문제가 해소됨.
- covariance·semantic classifier·새 estimator factor를 추가하는 방향으로 범위를 넓힐 필요 없음. B의 설계·기존 실패 보존·물리폰 미검증·서비스 사용자 소유는 유지.

## R1 — 5Hz 신규 관측과 20Hz retained frame의 분모를 분리

- v2는 최대5Hz fitting과 `accepted후보≥100frame`을 동시에 요구하지만, 해당100frame이 새로운 observed fit인지 retained를 포함한 present result인지 정의하지 않음.
- 실제 고정 fixture/report 확인:
  - `fixtures/floor-visible-clean101/input.json`:400 frames, source timestamp `0`부터 `19.95s`.
  - `fixtures/runs/floor-visible-clean101-map/report.json`의 `frameLedger`: 최초 fresh는 index43, `t=2.15s`.
  - 비간섭 조건을 지켜 동일 최초 fresh에서 시작하면 observed fit의 이론상 상한은 `floor((19.95−2.15)/0.2)+1 = 90`회. support 실패·Lost·실제 budget은 이 수를 더 줄일 수 있음.
- 따라서 `observed≥100`은 불가능하고, retained 결과를 포함해100으로 세면 같은 관측을 반복 표시한 수가 됨. 후자도 UI coverage 수치로 유효하지만 신규 관측/독립 GT 분모와 구분해야 함.
- 최소 수정 권고:
  - `observedFitCount`: 성공적으로 새로 fit한 source timestamp/sequence의 고유 수. 예: **≥40**을 사전 gate로 고정하고 해당 observed fit의 **≥90%**가 normal≤3°·offset≤.05m를 만족.
  - `candidatePresentFrameCount`: observed 또는 retained의 유효 결과를 담은 서로 다른 source frame 수. **≥100**을 기존 화면 coverage gate로 유지하되 observed/retained 수를 별도 보고.
  - retained 결과의 GT 오차도 보고하되 반복 표시를 새 fit 성공수에 더하지 않음. blank·expired·hard-reset 결과는 present count를 늘리지 않음.
  - 40 대신 다른 observed 하한을 정해도 되지만,5Hz source-time sampling/실제 fixture 길이와 양립하도록 **새 구현 결과를 보기 전에** 고정.
- 이 수정은 floor algorithm/threshold/실행 범위를 변경하는 요구가 아니라 새로 추가된 수치 수락의 정의를 완결하는 것.

## 재검토 범위

- R1의 observed/present 분모와 각 GT gate를 수정한 고정 계획만 재확인하면 됨.
- 그 외 C1–C5, 대안 A/B/C, source/display horizon, floor 의미/불확실성 범위는 재개방하지 않음.
- 구현 승인 이후 실제 회귀·sanitizer·WASM·Worker·표시 검증은 그대로 필요. 이 계획 리뷰는 제품 검증을 대신하지 않음.
