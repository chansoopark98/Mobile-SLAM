# 실패한 current-observation 정책의 안전 격리 계획

- 날짜: 2026-10-13, Asia/Seoul. Ralph iteration5
- 상태: **architect·critic APPROVED, support_fix lane에서 iteration5 구현 중**. 승인 범위는 아래 instance ownership과 명시적 opt-in 격리. 구현·검증 완료를 뜻하지 않으며 원래 tracking 목표는 NOT_ACCEPTED
- 요청 범위: 현재 실험 코드를 삭제하지 않고 명시적 opt-in으로 분리. 새 model 탐색·GT gate 완화·배포0

## 근거·목표

- D0 whole cull·D2 old-depth restore·D3 history-certified candidate는 반복 GT gate 실패로 수락하지 않음
- D3 fast101 ATE0.0552m/rotation2.66° 통과, fast8809 ATE0.28067m>0.25m/RPE0.131m>0.10m 실패
- general H4 모델도 실패. ADB 연결 없음, 최신30초 numeric에는 영상/GT 없음. desktop 결과로 phone tracking 해결을 증명할 수 없음
- 선택: 현재 source의 실험정책1/2를 보존하고 default0을 시작 A2 legacy 동작과 맞춤. 실패 source/binary/raw/data/test·D1·다른 작업 보존

## 정책·API 계약

| 값 | 동작 | 사용 |
|---|---|---|
| 0 | 기존 시작 A2 legacy current acceptance | Engine/Optimizer/native argv/App/Worker 기본 |
| 1 | D2 current raw rejection + old-depth restore/guard | 명시적 실험 opt-in |
| 2 | D3 good-history candidate retention + D2 fallback | 명시적 실험 opt-in |

- **Config/g_config에 policy 필드 추가0**. 실험 활성화 권한은 instance API만 소유하며 일반 Config는 실험을 활성화할 수 없음
- VIOEngine이 selected int0, Optimizer가 effective int0을 각각 소유. Estimator forward setter/getter로 전달하며 getter는 각 instance 값을 반환
- VIOEngine typed setter/getter:0..2만 허용. invalid는 policy·epoch·profile·calibration·history를 바꾸기 전에 throw. 직접 Estimator/Optimizer setter도 범위 검증
- resetState는 새로 만든 Estimator/Optimizer에 Engine selected policy를 전달. reset은 선택 보존; configure/configureFromConfig는 Engine selected0/backend effective0으로 초기화
- 실제 policy 변경(0→1,1→2,2→0 등)은 fresh engine epoch와 빈 tracking/map/prior/observation history에서 시작. 선택된 profile/calibration 보존. 같은 값 setter는 무동작/epoch 불변
- optimizer의 current raw-outlier 분기는policy≥1에서만 실행. good-history candidate 인증은policy2, policy1은old restore. 기존 unknown/young/invalid·flag/consumer/anchor/lifecycle 계약 보존
- default0이 이전 policy의 flags/prior를 이어받지 않도록 epoch 분리. legacy수치경로를새threshold/model로바꾸지않음

## 소비·scope

- 제품: `include/vio_engine.h`, `src/vio_engine.cpp`, `include/backend/optimizer.h`, `include/backend/estimator.h`, `src/backend/optimizer.cpp`, `wasm/vio_bindings.cpp`. config.h 변경0
- 도구·회귀: `tests/audit_dataset_replay.cpp`, `tests/test_engine_contracts.cpp`, `tests/test_projection_trial_contracts.cpp`, `scripts/dev/wasm-smoke.mjs`, `scripts/dev/README.md`
- native replay: 마지막 optional arg policy0|1|2, Engine setter로 opt-in; 기존 argv는0. Engine own getter를 raw/summary에 저장, invalid는 처리/출력 변경 전 거부
- WASM 두 API binding과 직접 smoke. default/configure/reset/invalid/epoch/readback을 fresh artifact에서 확인
- App/Worker는 실험을 활성화하지 않음. UI/URL parameter 추가0, 기존 loader/serve 파일을 실패 runtime으로 승격0
- Projection `prepareScene`은 Estimator setter로2를 명시하여 O/R 의미 보존. default control은 Estimator0/support8, explicit1/2는7. g_config policy 사용0

## 필수 검증

1. failed D3/current source·binary·raw/hash 동결 → architect → critic 승인
2. engine/backend tests: default0·0..2 own readback·invalid-before-mutation·same-value epoch·reset selection 보존
3. 0→1/1→2/2→0에서 fresh epoch/history와 profile/calibration 보존; 실험 history 혼합0
4. Projection O/R tests 명시policy2 유지, policy0legacy support8/1·2support7 controls. 테스트 삭제/기대값완화0
5. native argv 생략/0 exact parity·1/2 readback·invalid early failure; source/adapter/artifact/input hashes 기록
6. default0은 frozen 시작 A2+허용 D1 finite guard와 matched pixels/IMU/config/seed/threads에서 status/reason/timestamp/pose exact parity. 유효 입력 원소 차이0, invalid finite-guard 차이는 별도 oracle
7. 영향/full native·sanitizer·browser/syntax/clangd·fresh isolated WASM smoke·실제 Worker default readback
8. 독립 code review/architect·Ralph-owned cleanup → 영향 regression 재실행

- configureFromConfig control: 이미 selected2/backend2인 Engine에 일반 Config 입력→실제 selected0/backend0. Config에는 policy 필드 자체가 없음
- A0/B1 serial interleave: A/B 각 selected/effective readback·epoch 기록. 다른 Engine의 configure/reset/setter가 상대 policy/backend/epoch를 바꾸지 않음. setter의 fresh epoch는 해당 Engine만 변경
- 이 검증은 policy instance 격리만 증명. 기존 g_config calibration의 일반 동시 실행 격리는 주장하지 않음

## 수락·한계

- safe containment 수락: API/lifecycle/argv/binding controls와 default0 parity 통과, 실험 활성화는 명시opt-in만, 실패 증거·테스트 보존
- 원래 strict churn≥280/300·ATE≤0.25m/rotation≤5°/1sRPE≤0.10m/3°·fullTUM/coverage/속도gate 변경0. 시작 A2 자체 실패도 보존
- default0 호환성 통과는 tracking/accuracy 문제 해결이 아님. 결과에 SAFE_CONTAINMENT_VERIFIED와 원래 목표 NOT_ACCEPTED를 구분
- hard missing evidence: phone image/IMU 동시기록·실측 calibration/시간정렬·독립 fieldGT·열·수락 가능한 general recovery model
- native/합성·WASM·phone 증거 분리. commit/merge/deployment0, 원본/서비스/TLS/실패 runtime 보존
