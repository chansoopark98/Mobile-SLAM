# 저특징·바닥 후보 plan-v3 독립 Critic 리뷰

- 판정: **APPROVE — 고정 계획의 구현 착수 가능**.
- 대상: `build/low-feature-20261014/plan-v3.md`, SHA256 `ce6e91442165d9ba5e93258958963a2b24dc262d177693bcc06418e687c46108` 확인.
- 독립성: Architect v3 및 다른 review 결과를 읽거나 전달받지 않음. 고정 v3 전체와 v2→v3 diff를 직접 확인.
- 변경: 이 리뷰 문서만. 제품·계획·서비스·fixture 변경 없음.

## 마지막 차단 항목 해소

- v2에서 남긴 R1은 다음처럼 명확하게 반영됨:
  - 성공한 새 **observed fit≥40회**.
  - **observed+retained 후보 표시≥100 source frame**.
  - GT 통과율의 분모는 **observed fit만** 사용; 그중≥90%가 normal≤3° 및 offset≤.05m를 만족.
  - 반복 retained 표시를 새 관측으로 세지 않음.
- 5Hz fitter와 약18초 initialized source 구간에 양립하는 계약이며, 새 관측의 정확도와 화면 유지 coverage가 분리됨.
- diff는 상태 라벨 v2→v3 및 위 분모 수정뿐. 이미 해소된 C1 raw bracket, C2 seed/gravity/interface, C3 deadline/plane state, C4 실제 E2E·prediction gate, C5 backend 비간섭 계약을 변경하지 않음.

## 승인 범위

- 마지막 clean visual anchor의 독립 IMU shadow 출력과 현재 관측점 기반 수평면 후보 구현.
- metric pose/status/map/prior와 비간섭, raw-gap·frame·epoch·hard-error fail-closed, 명시된 source/display TTL, semantic classification unknown 유지.
- 고정 baseline 대비 native/browser/WASM·sanitizer·실제 Worker·E2E 수락을 실행하고 원시 실패를 보존하는 다음 단계로 진행 가능.
- 이 승인은 설계/계획 승인. 구현 완료·새 테스트 통과·실기기 정확도·무기한 IMU 추적·semantic floor 인식 또는 서버 실행을 승인 결과로 주장하지 않음.
- 추가 계획 차단 항목 없음. 사용자에게 반복 실행 승인 질문을 할 필요 없음.
