# 저특징 IMU 예측·바닥 후보 계획: 독립 Architect 검토 v2

- 날짜: 2026-10-14. 판정: **APPROVE — v2 계획에 따른 구현 진행 가능**.
- 고정 검토본: `build/low-feature-20261014/plan-v2.md`, SHA256 `46251e8d3b69f2dbf926e0f581dfc74747096111a490dcd045ef364dd4283f27`. 요청된 hash와 실제 파일 일치.
- 범위: 본인의 v1 지적사항과 v2 수정 계약을 제한적으로 재검토. Critic 문서/결과 열람·전달 없음. 이 문서만 작성; 제품/고정 계획/기존 dirty/서비스 변경 없음.

## v1 blocker 해소

| 항목 | v2에 반영된 계약 | 판정 |
| --- | --- | --- |
| B1 실제 raw IMU gap | 원본 raw-left/right와 interpolated endpoint 분리,0.20s gap을0.05s camera endpoint로 나눠도 거부, 부적합 coverage의 재seed 금지, metric0.5s 정책 보존 | 해소 |
| B2 Worker skip의 seed 계보 | `invalidateInertialPrediction()`이 shadow만 폐기, missing_bracket/입력거부처럼 engine 미호출 경로에도 전달, 다음 blank 재개 금지, invalidation API 없는 부분호환 WASM unavailable | 해소 |
| B3 floor 영상 통합 수락 | 고정 floor-visible-clean101에서 신규 current getter→WASM→Worker fit→App/Renderer, independent GT normal/offset gate, blank retention/expiry/epoch/reacquisition 검증 | 해소 |

- seed: actual gravity read-only getter의 파일 소유권, accepted P/R/V/Ba/Bg의 exact timestamp·published P/R 일치·generation 검사 명시.
- 실패: 허용 Lost reason에도 별도 qualityReason이 비어 있어야 한다는 계약과 motion-warning 음성 시험 추가.
- plane: finite current observation 교집합, +g 방향과 Z-up 검증, Worker 단일 state 소유, 명시 schema/source identity, fit 생략과 실패/모호성의 retention/clear 정책 추가.
- 표시: source horizon0.5s와 delivery/display age0.1s의 합산 상한0.6s를 명시하되 source cap 연장 금지. 기본 side-channel 계산/표시와 구형 WASM unavailable 정책 명시.
- 수락: clean synthetic prediction의 anchor-relative GT gate와 기존 visual/map/window/prior 비간섭 비교가 추가됨. 기존 실패군 보존과 실제 phone 미검증 경계 유지.

## 구현·증거 작성 시 유지할 의미

- floor `accepted 후보≥100 frame`은 실제 result frame의 candidate availability 수로 집계하고, `observed` fit 수와 `retained` 표시 수를 별도로 기록. 20초 fixture에서 초기화 후 최대5Hz fitting이면 독립 fit100회는 보장되지 않음. retained 결과를 새로운 관측100회로 보고하지 않음. plane error도 observed/retained를 구분해 같은 source를 반복 집계한 효과를 드러냄.
- physical gravity fixture의 `g_physical=[0,0,−9.81]`와 엔진의 `g_estimator=[0,0,+9.81]`를 테스트 입력 단계에서 명시적으로 변환. 두 관례의 이름만 같다는 이유로 그대로 복사하지 않음.
- 현재 request/epoch에 속한 실패만 predictor를 폐기. 취소된 과거 request가 새 epoch seed를 제거하지 않는 기존 identity 경계를 유지.
- 비간섭 snapshot을 위해 test access/진단 경로를 사용할 수 있으나, prediction/floor가 backend를 갱신하도록 설계를 바꾸지 않음. 기존 시간제한 solver와 fixed-work 비교의 구분 유지.

## 반론·tradeoff·종합

- 가장 강한 반론은 여전히 “표시 연속성이 visual tracking 회복을 뜻하지 않는다”는 점. v2는 이를 Lost 유지·positionUncertain·semantic unknown·독립 수락 결과로 드러내므로 범위상 수용 가능.
- 독립 shadow는 이중 적분/수명 시험 비용과 translation drift 위험을 감수하는 대신 optimizer/map/prior 오염을 막음. floor factor 직접 결합은 관측성·정보상관 근거가 없으므로 계속 제외하는 판단이 타당.
- 신규 설계 blocker 없음. **설계 승인과 제품 검증 통과는 별개**. v2의 red/green·fresh native/WASM·실제 Worker·통합 floor/prediction·비간섭 gate를 완료하기 전 기본 제품 승격/정확도 통과를 주장하지 않음.

- 이번 확인: 고정 v2 SHA256 및 내용 검토. 신규 알고리즘 실행 시험이나 서비스 작업 없음.
- 변경 파일: 이 문서1개.
