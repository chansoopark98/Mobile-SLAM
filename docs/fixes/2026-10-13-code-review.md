# 실험 policy 격리·기존 변경 독립 코드 리뷰

- 날짜: 2026-10-13, Asia/Seoul. Ralph iteration5
- 후속 확인: 중단 후 남은 결과와 동일 환경 exact 대조를 복원했다. 아래 리뷰 당시 `PENDING_QA`·15/16은 역사 상태이며, 재실행·새 수정·최종 Git 상태는 [Flip6 종합 결과](../research/2026-10-13-flip6-analysis-and-application.md)에 기록한다. 원래 tracking/GT 목표는 계속 NOT_ACCEPTED.
- 코드·architect 판정: **APPROVE**, 검토 범위의 CRITICAL0/HIGH0. 추가 필수 source 수정 없음
- 실행 수락: **PENDING_QA**. 아래 fresh native/sanitizer·default0 parity·WASM·post-cleaner 결과 전 SAFE_CONTAINMENT_VERIFIED 선언0
- 원래 tracking/accuracy/speed 목표: **NOT_ACCEPTED**. D0/D2/D3/general H4 실패와 고정 GT/coverage/RPE/성능 gate 유지; default0 호환성이 문제 해결 증거는 아님
- 리뷰어 제품 source 수정·heavy 실행·recursive delegation0. 외부 ask_codex 미제공으로 native 독립 리뷰. 이전 단계 리뷰는 `build/tracking-performance-20261013/review/code-review-before-containment.md`에 보존
- 비교 기준: `build/tracking-performance-20261013/baseline-source/`; HEAD의 기존 dirty 변경은 이번 변경으로 오인하지 않음. iteration5의11파일과 이전 D1/flag consumers/provenance/CMake delta를 함께 검토

## 정책·수명 판정

- Engine selected0·Optimizer effective0은 instance 소유. Estimator는 검증된 backend setter/getter를 전달. Config/g_config policy 필드·global policy write0, 일반 calibration의 동시 격리 주장0
- Engine setter는0..2 검증 후 같은 값이면 return. 실제 변경은 selected 갱신→resetState로 fresh epoch/backend/tracker/map/prior/IMU/flags를 만들고 선택값을 새 Optimizer에 전달. invalid-before-mutation·same-value no-op 구현 확인
- configure는 selected0으로 resetState, configureFromConfig의 일반 Config 대입은 policy를 덮어쓸 필드가 없음. resetState/Estimator reset/Optimizer reset은 명시 선택을 유지. backend 미생성은 effective−1로 unavailable을 구분
- resetState가 diagnostic/benchmark/execution/rotation 선택과 camera/tracker 설정을 보존하는 기존 경로 재사용. selected/effective 분리 readback으로 backend 값과의 불일치를 숨기지 않음
- optimizer current raw-outlier 검사만 policy≥1에서 실행, history candidate 인증은2만. 새 default0에서 bad-row flag가 생성되지 않으며 Engine policy 변경은 history를 먼저 분리.1/2의 실패 알고리즘은 명시 opt-in에 보존
- low-level Estimator/Optimizer setter는 내부 연결·전용 실험용이며 Engine 수명 전환을 대체하지 않음. 직접 backend를 바꿨을 때 selected/effective 불일치가 드러나고 Engine reset에서 own selection으로 복귀하는 control 포함

## retained 코드·정보 소비

- stored flag는 실제 관측 수명에 귀속. solve/OLD raw prior/DLT/rank/reprojection/parallax/correspondence 모두 실제 frame 전진 후 flagged row 제외; structural1000prefix/ID/slot 유지
- 관측-only 보존은 pre-solve solve_flag1·admitted metric·nonpending에 한정. unknown/young/invalid는 whole-feature failure.1은 old rho+good-view cheirality,2는 good-history certificate 통과한 candidate만 유지
- current<2 rollback/prior release/Lost와 보수적 flags 유지. OLD/NEW bad front는 보수 실패하며 virtual ray/reindex0. historical map과 current PnP getter를 분리; PnP ON full-sequence/GT 미검증
- D1 IMU correction/inverse-rotation 재사용과 local projection/형상별 SVD workspace 보존. nonfinite A 선거부·info/V/분모 검사로 이전 V 상속 차단. 유효 입력 수학 변화0이라는 주장은 fresh default0 replay로 최종 확인
- provenance는 실제 CMake source root·compile directory 상대 source·frozen tree Git 분리·명시 imported archive SHA/readback을 기록. 자동 archive 검색만으로 link 증명0
- 기각한 marginalization 계산 후보는 시작 source와 byte 동일로 복구된 상태 유지

## 도구·tests 검토

- native trailing policy는 정확한 문자열0/1/2만 허용하고 dataset/config IO·output 생성 전에 검사. Engine own selected/effective를 실제 readback한 뒤 effective-config/raw/summary에 기록. 생략과 explicit0은 setter no-op으로 epoch 변화0
- WASM 두 public bindings와 smoke는 default0·selected/effective diagnostics·range·epoch·same value·reset preserve·configure0 검사를 포함. 기존 WASM exception catching 설정 유지. App/Worker/Wrapper에 실험 활성화 API 호출 없음 확인; fresh 실제 실행은 QA 대기
- engine controls는 실제 prior/feature/input history를 만든 뒤 invalid/same/change/reset을 검증. A0/B1 interleave가 실제 backend support8/7 및 상대 epoch/prior를 확인. configureFromConfig의2→0 control 포함
- O/R regression은 explicit2로 보존. 별도 default0 support8 vs1/2 support7·old/candidate depth 차이를 확인하는 controls 추가; test 삭제·GT cap 완화0
- comparator 준비 selfcheck7개 통과 기록 확인: pose1e−12·solver semantic·nonzero/missing policy readback·cap·unknown semantic field 변경을 거부. 실제 replay parity 통과로 오인하지 않음
- 검토 시 containment/provenance-tests.log7/7, syntax log 오류0 확인. root의 최신 full CTest는15/16이며 EngineContracts의 새 policy-history fixture가 feature count0에서 실패(line1050); root/executor가 원인을 확인 중. fixture 수리·fresh 재실행 전 native green 또는 SAFE_CONTAINMENT_VERIFIED 선언0. sanitizer/나머지 QA도 별도

## 검토 시 SHA256·후속 수락

- src/vio_engine.cpp: `5378c3238f817f62268e73b0299ac31779c49f926fca0c7ececb667b3ceaba6b`
- src/backend/optimizer.cpp: `21ce2dd4ef4f9b91c69b350a4e80864879f13c5b2198a0476785c594faee87b7`
- include/backend/optimizer.h: `b211c1c5d4d4d55260a10e750b39edf311d1fa5f947f53dff120e43c673c61f8`
- include/backend/estimator.h: `371bc33f26d4896a75826cdf7d91d51abd8aa673a50b973d581ec54d01d6bf61`
- tests/audit_dataset_replay.cpp: `672034456ab679d5af140ad6fb16261bf2c24f007d53ab99cee5104d9e2e17fb`
- scripts/dev/wasm-smoke.mjs: `347077a17f0688a79e90e817d53efec564c4bccab5364a997f2331d3f0138324`
- 후속 QA: fresh full native/ASan·UBSan·LSan, source-pinned default0 matched replay exact semantic parity, 생략/0·1/2·invalid argv, fresh isolated WASM/actual Worker0, clangd·browser, Ralph-owned standard cleaner 후 fresh 영향 회귀
- SAFE_CONTAINMENT_VERIFIED는 위 QA의 실제 green 이후에만 표시. D1 kernel 개선을 engine/mobile 개선으로 확대0. 신규 phone raw image/IMU·calibration/time alignment·fieldGT/열·수락 가능한 general recovery model 부재를 명시, commit/merge/deployment0
