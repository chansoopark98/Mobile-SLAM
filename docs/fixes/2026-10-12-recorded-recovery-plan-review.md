# 기록 기반 회복 개발 계획 검토

- 날짜: 2026-10-12, Asia/Seoul
- 담당·범위: GPT-6 Astra Ultra, **개발 계획 검토만**. 제품 구현·구현 후 diff/성능 검토는 Sol Ultra 담당
- 최종 판정: **APPROVE — 개발 착수 가능**. 1차 REVISE의 R1–R4가 계획·테스트 명세에 반영되어 미해결 계획 blocker0
- 대상: [개발 계획](2026-10-12-recorded-recovery-plan.md), [테스트 명세](2026-10-12-recorded-recovery-test-spec.md)
- 1차 검토 revision SHA256: plan `78e448b5e854b60fe17aaf5ab0cdbfbe91a2ef3902a2667d0f4e54040f067497`, test spec `70ba6134b1214fb244b6218a214d5d9a09990ae1e9049069d080542d4311b2e0`
- 확인: 현 AGENTS.md, frozen baseline manifest, open VIO/web/commercial 연구 문서, 최신 numeric phone 보고서, 계획에 직접 관련된 feature/optimizer/slide/initializer 계약
- 변경 범위: 본 리뷰 문서와 `build/research-20261012/plan-review/review.json`. 제품 파일·기존 dirty 변경·데이터·서비스 보존. 실행 결과 수락 또는 물리 성능 검증을 수행한 리뷰가 아님

## 구현 전 해결 항목

| ID | 결함·영향 | 필요한 최소 수정·증거 |
|---|---|---|
| R1 | A2는 anchor8/9만 검사하지만, OLD slide 뒤 미해결 track이8→7이 되면 기존 DLT 경로로 우회할 수 있음. G4의 `anchor age로 geometry 거부 우회0`과 충돌. 단순 positive rho 검사도 미참여 track을 final/apply/marginalization에 다시 편입할 수 있음 | A2가 만든 deferred track의 기하 admission 의무를 age 변경 뒤에도 유지하는 수명 계약을 명시. 이미 solved인 기존 성숙 track의 수치 모델 변경은 A3로 계속 분리. 구조적 slot과 고정 admission mask를 구분하고 add/final/current-support/apply/OLD marginalization에서 동일 mask 사용. deferred9→8→7→유효 translation과 finite-positive-but-inactive/unknown rho0 fixture 추가 |
| R2 | T1의 매 frame0.09m 단조 translation은 adjacent-frame9pixel임. 실제 `addFeatureAndCheckParallax()`는 slot8과9를 비교하며 반복 NEW에서8은 고정되므로 성숙 track의 baseline이 누적됨. 이 값만으로 실제 NEW20회를 보장할 수 없음 | slot8과의 parallax도10pixel 미만으로 유지되는 고정 궤적·scene을 명시. 예: camera x=0↔0.09m 왕복, mature slot8 at0, young point약4m·좁은 FOV. 모든20회 실제 selector 결과·성숙 current ID·young condition을 기록. selector 강제 덮어쓰기 금지는 유지 |
| R3 | reanchor만 `d*f/f.z`로 바꾸면 optical-Z 계약이 전체 경로에서 성립하지 않음. 현재 ProjectionFactor·final cull·map/PnP는 `f/rho` 또는 `f*d`, target `f.head<2>()`를 사용하여 z1 ray를 전제 | finite positive ray를 공통 관측 저장 경계에서 z1로 정규화하는 등의 단일 표현 계약을 선택. 또는 모든 소비자 변경 범위를 명시하되 불필요한 factor/Jacobian 재작성은 피함. 동일 ray에 양의 scale을 달리한 입력으로 triangulate→factor→final/apply→OLD/NEW→map/PnP의 같은 metric point·residual을 검증 |
| R4 | 성능 담당 확인: 보존 native A0 출력은 `process_ms`만 있어 계획의 frontend/backend/triangulation 등 stage p95 control을 그대로 얻을 수 없음 | frozen A0 core/A2 core에 동일 test adapter를 연결하고 adapter SHA·core/build provenance·원래 A0 binary와 수치 parity를 먼저 확인하거나, 두 보존/후보 WASM의 실제 getter harness를 사용. 같은 측정 경로·계측 비용으로 stage를 직접 수집. total이나 다른 run에서 stage 통계를 추정하지 않음 |

- R1 추가 수치 계약: reprojection은 `sqrt((fx*Δu)^2+(fy*Δv)^2) <= 1.5pixel`처럼 norm을 고정. κ10,000·depth positive·all-view 검사·signed Ceres trial의 구분 유지
- R2는 synthetic phone 원인 주장과 무관한 fixture 정확성 문제. 구형 prefix의 OLD current-loss와 frame98의 NEW initial-support 이후 final failure를 분리 유지

## 수용 가능한 설계 범위

- frozen **미커밋 baseline**과 A0/A1/A2 ablation을 사용하고 HEAD 단독 재현을 주장하지 않는 기준은 적절
- 구조적 membership·1000 slot prefix·지연 삭제를 공통화하고, 활성 mask와 slot 순서를 분리하는 방향은 필요
- NEW pose9 덮어쓰기 전 old/new camera pose 확보, optical depth 재앵커, outgoing prior의 depth block 비잔존 확인은 필수. depth가 prior에 남으면 단순 수리 범위 중단 조건 유지
- IMU admission·최소 current support2·SPD·finite guard 유지, fake depth·stale pose·임계값 완화 제외는 타당
- 기존 성숙 track 전면 재설계·covariance/prior 표현 변경·회전 keyframe 정책·재매칭은 첫 후보에 추가하지 않음. A2 실패 원인이 입증될 때 후속 계획 대상으로 유지
- capture C1은 기하 A2와 별도 수락. 최신 계획의 전송 후 실패 frame 폐기·다음 신규 frame부터 fallback은 픽셀/시각 계약을 보존

## 검증·승격 조건

- 계산 가능성을 증명한 NEW microprobe7개를 최신 phone Lost33의 확정 원인으로 사용하지 않음. 최신 기록은 영상·독립 GT가 없으며 기존300+300PGM은 부분 기록
- T2는 고정 pixels/IMU·withheld GT에서 A0의 실제 원인을 먼저 재현. 미재현이면 `NOT_REPRODUCED`; unit 성공·factor 수 증가만으로 recovery 성공 판정 금지
- TUM room1/room4 전체, profile별 matched A0, body/SE(3)/scale1/1:1 association, translation·rotation RMSE+5% 이하·coverage−1pp 이내 조건 유지
- 기존 fast seeds와 동결 후 신규 확인 seed를 구분. 실패 seed·원본 trial·정확한 command/source/artifact/input hash 보존
- native와 실제 Worker/WASM 실행, fresh build provenance, 영향 sanitizer·browser 계약·clangd 및 skip/TODO 결과를 별도로 기록
- 순차 ABBA 성능 비교와 C1의 pixel identity·generation/order/backpressure/close·actual target Chrome gate 유지. desktop 통과를 물리 Flip6 지연·정확도 향상으로 확대 금지
- 본 계획 검토의 승인과 제품 후보의 수락은 별도. 승인 후에도 동결된 gate 실패 후보는 승격하지 않음

## 재검토 기록

- 최종 검토 revision: plan130행/SHA256 `14c8d47c55cb5e1104fb1721ba1b6f6afb560e954aa04dd8f37ae4686ca7d2be`, test spec122행/SHA256 `69dabf4dec8161bfadb155481bee385f0b4c5b0ce210afaa54f4e334f778e442`
- R1 해결: `geometry_pending`을9→8→7 뒤에도 유지, 실제 admission·usable solve·final 통과 뒤 해제. 고정 admission mask는 OLD marginalization까지 공통 적용. M4·G5 추가
- R2 해결: 고정 slot8 x0와 x0↔0.09m 왕복 궤적, 실제 selector NEW20/20 검증으로 교체. 강제 margin 선택 금지 유지
- R3 해결: `FeaturePerFrame` 공통 입력 경계의 canonical z1 ray, pixel observation/normalized velocity 보존. G6에서 positive scale0.2/1/7의 전체 경로 동등성 검증
- R4 해결: 동일 SHA adapter·A0 core 보존·원래 A0와 fixed-work 수치 parity 후 직접 stage 측정. WASM actual getter 대안과 null/sample count도 명시
- capture의 전송 후 실패 frame 폐기·다음 신규 frame fallback, current phone/구형 replay 구분, Sol 구현 후 검토 역할도 확인
- 현재 C1 물리 target Chrome gate는 root의 ADB 확인에서 `UNAVAILABLE`(device/mDNS 없음·제공 포트 연결 timeout). synchronous 기본 유지; desktop 실험 결과로 capture Worker 기본 승격 금지
- 검토 메타데이터: `build/research-20261012/plan-review/review.json`. 이 승인 이후 알고리즘·임계값·평가 기준·범위가 실질 변경되면 변경 부분을 새 계획 검토 대상으로 처리
- 남은 위험: young 관측의 기하 검사는 현 pose/calibration 오차를 사용하므로 phone Lost·bias·불연속 해소를 보장하지 않음. A0 원인 재현·독립 GT·TUM·실제 Worker·성능 gate와 물리 수락 경계를 유지
- 사용자 추가 승인 없이 승인된 개발·검증 단계 진행 가능. 본 agent는 개발 계획 검토 범위를 완료
