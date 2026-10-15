# 저특징 IMU 예측·바닥 후보 계획: 독립 Architect 검토 v3

- 날짜: 2026-10-14. 판정: **APPROVE — v3 계획에 따른 구현 진행 가능**.
- 고정 검토본: `build/low-feature-20261014/plan-v3.md`, SHA256 `ce6e91442165d9ba5e93258958963a2b24dc262d177693bcc06418e687c46108`. 요청된 hash와 실제 파일 일치.
- 확인: v2→v3 전체 diff는 상태 label과 floor 수락 분모 문장2곳뿐. 나머지 API·수명·raw IMU gap·비간섭·검증 계약 동일. Critic 결과 열람/전달 없음.
- floor 집계: 실제 `observed fit≥40회`, `observed+retained 표시≥100 frame`을 각각 요구. normal≤3°·offset≤0.05m의90% 수락 분모는 observed fit만 사용. retained 반복을 신규 관측으로 계산하지 않는 v2 검토 의견을 충족.
- 최대5Hz·20초 fixture에서40 observed fits는 시간 예산상 가능. 실제 충족 여부는 신규 current-point getter/Worker/fitter 통합 시험으로 판정하며 기존 legacy map 결과로 대체하지 않음.
- v2의 B1/B2/B3 해소 판정과 강한 반론/tradeoff/종합 유지: 짧은 관성 예측 표시와 기하 수평면 후보의 설계 승인이고, metric tracking 회복·semantic floor·실기기 정확도 승인은 아님.
- 신규 blocker 없음. 제품 승격 전 v3 전체 실행 검증 gate 필요. 이번 작업은 이 문서만 추가; 제품/고정 계획/기존 dirty/서비스 변경 없음.
