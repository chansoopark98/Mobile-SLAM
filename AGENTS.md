<!-- AUTONOMY DIRECTIVE — DO NOT REMOVE -->
YOU ARE AN AUTONOMOUS CODING AGENT. EXECUTE TASKS TO COMPLETION WITHOUT ASKING FOR PERMISSION.
DO NOT STOP TO ASK "SHOULD I PROCEED?" — PROCEED. DO NOT WAIT FOR CONFIRMATION ON OBVIOUS NEXT STEPS.
IF BLOCKED, TRY AN ALTERNATIVE APPROACH. ONLY ASK WHEN TRULY AMBIGUOUS OR DESTRUCTIVE.
USE CODEX NATIVE SUBAGENTS FOR INDEPENDENT PARALLEL SUBTASKS WHEN THAT IMPROVES THROUGHPUT. THIS IS COMPLEMENTARY TO OMX TEAM MODE.
<!-- END AUTONOMY DIRECTIVE -->

# Mobile-SLAM 작업 계약

- 목표·단계·수락 조건: `docs/refactoring-plan.md`
- 현재 코드·실행 근거: `docs/audit/2026-10-06/README.md`
- 개발 환경·명령: `scripts/dev/README.md`
- 재현 절차: `.agents/skills/mobile-slam-validation/SKILL.md`

## 변경 전

- `git status --short` 확인; 기존 tracked/untracked 변경·dataset·TLS 키·빌드·서비스 보존
- 원본 dataset home은 `/mnt/backup/SLAM`; `MOBILE_SLAM_DATA_ROOT`로 명시 변경, 원본은 읽기 전용 유지·실행 결과는 local `build/`에 저장
- 실제 알고리즘 변경 전 현 동작 회귀, source/dependency/build fingerprint, baseline 확보
- cleanup 계획에 수정 파일·중복/비활성 경로·검증 명령 명시; 작은 단위로 수정·검증
- 제품 runtime 의존성 추가 전 요청 범위 확인; 개발 의존성은 `scripts/dev/`로 분리
- 독립 병렬 작업은 파일 소유권 명시; 하위 에이전트는 다른 작업 변경을 되돌리지 않기
- native 하위 에이전트 모델은 현재 설정 상속; 지원되지 않는 과거 모델명 하드코딩 금지

## 구현·검증

- C++17; 기존 타입·utility·패턴 우선 재사용
- native `VIOSystem`과 browser `VIOEngine` 경로 분리 상태 전제; 한쪽 테스트를 다른 쪽 검증으로 보고하지 않기
- IMU: timestamp 초, 가속도 m/s², 각속도 rad/s; 영상과 같은 시간축 및 frame 경계 샘플 검증
- 좌표·calibration: camera/body/world 구분, extrinsic 방향·row/column-major·crop/회전 이후 K 확인
- frame 처리량, initialized coverage, pose coverage, reset 수, 출력 timestamp/pose age를 분리 기록
- 속도: 초기화 전/후, capture·copy·queue·frontend·backend·render, p50/p95/p99를 구분
- 정확도: independent GT, one-to-one timestamp association, body frame, SE(3) scale=1, translation/rotation ATE·RPE·coverage 명시
- Sim(3)는 scale 진단으로 별도 보고; zero pose·zero association·누락 rotation은 정확도 통과 근거 제외
- 기존 WASM을 새 소스의 빌드로 주장하지 않기; 별도 build 출력에서 검증한 뒤 source/artifact hash 기록
- 빌드 parallel jobs는 기본 2, 최대 4 권장; 데이터·서비스 변경은 범위와 소유 확인 후 실행

## 증거·문서

- 명령·환경·결과·검증 한계 포함; 설정 존재와 실제 호출 성공 구분
- 데이터셋/desktop replay 통과를 실제 모바일 실시간 성능·production 정확도로 확대하지 않기
- 실제 모바일 pose jump 진단은 `web/audit-mobile.html`과 runbook의 관측 절차 우선; numeric observation만으로 완전 sensor/image replay·현장 GT 검증 완료 주장 금지
- 한국어 개조식 요약; 숫자·식·버전·경로·URL·실패 원인 유지
- 새 commit은 intent 우선 + 필요한 git-native Lore trailers (`Tested`, `Not-tested`, `Constraint`, `Directive`) 사용
