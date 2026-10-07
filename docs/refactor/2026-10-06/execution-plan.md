# Mobile-SLAM 리팩토링 실행 계획

- 기준일: 2026-10-06; mode: 범위가 고정된 병렬 구현 + leader 통합·완료 검증
- 승인 결과: 실제 리팩토링, `https://dev.serdic.com:7002/` 실행, virtual browser·calibrated dataset 검증
- GPT-6 Astra / ultra 계획 검토 **1회** 완료·ITERATE 보완 반영; 이후 개발·검증은 GPT-6.1 Sol / ultra
- PRD: `.omx/plans/prd-mobile-slam-refactor-7002.md`; 테스트: `.omx/plans/test-spec-mobile-slam-refactor-7002.md`
- baseline: `docs/audit/2026-10-06/`는 변경 전 결과. 새 pass/완료 체크를 과거 audit에서 복사 금지

## Cleanup 사전 계획·소유

| Lane | 단독 수정 파일/책임 | 삭제·통합 대상 | 필수 검증 |
|---|---|---|---|
| 1 Engine | `include/vio_engine.h`, `src/vio_engine.cpp`, `include/vio_system.h`, `src/vio_system.cpp`, `wasm/vio_bindings.cpp`, `include/frontend/feature_tracker.h`, `src/frontend/feature_tracker.cpp`; 해당 engine 회귀 | native 중복 orchestration은 공통 VIOEngine 위임·parity 후 삭제; 미래IMU 소유·상태/시간/epoch 한 경계 | calibration/IMU oracle·freshness·reset·camera/body·headless native |
| 2 Numerics | `include/backend/estimator.h`, `src/backend/estimator.cpp`, `include/backend/optimizer.h`, `src/backend/optimizer.cpp`, IMU factor/integration·PnP·initial alignment·failure detector·numeric 회귀 | stale prior/PnP 소유·불필요 inverse/unsafe propagation·활성 실패 검사; factor legacy manifold는 일괄 전환 금지 | covariance/depth/solver·gravity·SO(3)·reset/PnP·ASan/UBSan |
| 3 Transport | `web/js/vio-worker.js`, `web/js/vio-wrapper.js`, `tests/browser/contracts.test.mjs` | loader fallback은 사용처 확인 후만 삭제; duplicate future carry·waiter/cache/ring 중복 상태 정리 | TODO5 strict 전환·epoch/sequence·all settle·map0·subarray·overflow |
| 4 Live browser | `web/js/{app,imu,camera,orientation,renderer,test-tumvi-app,audit-mobile}.js`, `web/{index,test-tumvi,audit-mobile}.html`, live/mock browser 회귀 | callback time의 sensor/capture 명칭·숨은 profile fallback·UI count 중복 정리 | permission/activation·clock/axis/resample·K/rotation·pause/drop·paced replay |
| 5 Evaluation/Build | trajectory evaluator·config·measurement processor·`CMakeLists.txt`, `wasm/CMakeLists.txt`, shared source 목록·native/test helpers·`scripts/evaluation/`, `scripts/dev/{check,native-replay,browser-baseline}`·replay/evaluator tests | source 목록·native replay 임시 link 우회·약한 parity fixture 중복; lane1 core API 소비 | evaluator known-truth·CMake/CTest·exact input manifest·fresh flags/hash·room1/4 |
| 6 HTTPS/Ops | `web/server.js`, 서버 lifecycle/회귀·TLS metadata·runbook 서비스 부분 | 자기서명 자동 fallback·공개 private path·무제한/truncate 로그·포트 혼선 | real7002 trusted chain/SNI·static boundary·bounded log·PID/served hash |
| Leader | 계획/기준 manifest·lane 통합·최종 replay·fresh deploy·결과 보고 | 완료 조건/소유 충돌 해결; final pass evidence 한곳 | R0~R6 전체·서비스 생존·변경 파일/단순화/위험 |

- 모든 lane: 기존 변경과 다른 lane 공존, 되돌리기 금지. 공유 파일은 owner 요청 후 변경; 새 의존성·범위 확대는 leader 인계
- `src/vio_engine.cpp`·binding·VIOSystem은 lane1만 수정; CMake/config는 lane5만 수정. lane2가 resetGeneration/usable update API 제공 후 lane1 연결
- Engine 출력 `T_W_C` row-major4×4 유지; additive epoch/time/reason/solver API·transport epoch+sequence·engine reset epoch는 PRD 공유 계약 준수
- **Engine future IMU queue 소유**, Worker의 한 bracket은 한 번 전달·소비; independent integral와 sum_dt가 기존 future assertion보다 우선
- 새 테스트/등록 이름·CLI 변경은 해당 owner가 실제 명령까지 제공; 임시 TODO/empty fixture로 gate 종료 금지
- 삭제 전 call site/build target/source 목록 확인·회귀 확보; reference/dataset/generated artifact를 사용 여부 없이 삭제 금지

## 순차 의존성과 병렬 실행

1. R0: 최신 dirty diff·untracked list·source/dependency/기존 artifact hash·서비스 PID·7002 점유 보존. TLS는 metadata/public-key fingerprint만 기록
2. 문서 gate: 이 cleanup 계획·PRD·test spec 작성 완료 후 구현 시작. Astra 검토 반복/새 승인 대기 없음
3. 병렬1: lane5 evaluator/fixture·lane6 TLS/static/log/lifecycle; lane1/2/3/4는 각 독립 seam의 old-fail regression 먼저 확보
4. 병렬2: lane2 usable solve/resetGeneration·numeric 수정, lane1 configure/IMU/time/status, lane3 transport, lane4 live sensor/profile. 공유 API 변경은 owner간 먼저 전달
5. 연결: lane1 internal reset·fresh pose·VIOSystem 위임, lane3 engine epoch/time 소비, lane4 UI·live/mock 동작, lane5 same source/build/headless·replay
6. 통합 회귀: R1~R4 strict·sanitizer·syntax/C++ analysis; 실패 원인 수정·해당 회귀 재실행. 새 native/WASM candidate·source/flags/hash manifest
7. Replay: 같은 room1 development3회 pilot→numeric tolerance 동결→full room1·room4 native/실 Worker·WASM·independent scorer; max-speed와 bounded paced 실험 분리
8. 배포: manifest의 활성 artifact set 원자적 승격. 현재 single-file embedded WASM은 JS 한 파일·config/manifest 검증, 이전 artifact 보존→실7002 index/replay/audit·mock browser·TLS·served hash·PID 생존 확인
9. 인계: R gate evidence·변경 파일/삭제/통합·수치/coverage·남은 field/fullSLAM 한계·URL/로그/stop 명령 제공; 요청 서비스 실행 유지

- build 기본2 jobs, 최대4 권장. 최종 latency 측정은 다른 build/replay와 동시에 실행 금지
- NAS room4 archive는 읽기만, 검증된 local staging·명시한 dataset root만 공개. NAS 전체 static expose 금지
- 현실 phone 부재는 synthetic·dataset·HTTPS 작업의 blocker 제외; 물리 증상 해결 판정만 conditional gate
- PnP on/off opt-in 회귀와 ablation 보존; 한 development 결과로 기본 on 변경 금지
- no blind smoothing, covariance diagonal jitter·depth clamp·invalid rotation projection으로 실패 가리기 금지

## 완료 checklist

- [x] R0 새 작업 snapshot/provenance·기존 dirty/데이터/키/로그/타 서비스 보존
- [x] R1 C++ evaluator·parity fixture independent old-fail/new-pass, no empty success
- [x] R2 engine IMU/calibration·live clock/axis/profile/권한 strict 계약
- [x] R3 solver fresh usable update·resetGeneration/epoch·prior/PnP·numerics·sanitizer 실제 실행
- [x] R4 TODO5 제거 후 필수 assertions 통과, all waiters/epoch/sequence/map/ring/subarray/drop 검증
- [x] R5 공통 source·VIOSystem 위임·fresh WASM, full room1/room4 native/browser·동결 parity·SE(3) translation/rotation·coverage·paced 결과
- [x] R6 실제 dev.serdic.com:7002 trusted HTTPS·static/log boundary·browser Worker·served hash·소유 PID·서비스 유지
- [x] 변경 파일·단순화·검증 명령/숫자·실패/skip/TODO·조건부 미검증·URL/로그/종료 명령 최종 보고

- 완료 확인: 2026-10-07 KST. 실제 근거·설정·숫자·한계: `README.md`, `final-refactor-review.md`, `virtual-validation.md`, `runtime-status.json`; 이전 계획 원문은 `execution-plan-before-completion.md`에 보존
- 완료 후 한계: phone camera/IMU·thermal·실제 jump 지배 원인/현장 개선·독립 field GT·capture-to-display·full SLAM·상용 배포 조건은 확인된 범위만 보고
- Rollback: 마지막 합격 candidate/config/manifest로 전환·hash/affected replay 확인; Git reset·NAS/기존 파일 삭제 제외
