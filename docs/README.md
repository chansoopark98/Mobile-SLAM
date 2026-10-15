# 문서 색인

현재 상태는 **[STATUS.md](STATUS.md)**에서 먼저 확인합니다. 이 파일은 문서를 어떤 순서로 읽을지, 그리고 어떤 문서를 아직 근거로 써도 되는지를 정리합니다.

## 디렉터리 규칙

| 경로 | 용도 | 파일 이름 |
|---|---|---|
| `docs/STATUS.md` | 현재 상태 (living) | 고정 |
| `docs/fixes/` | 수정의 계획, 리뷰, 결과 기록 | `YYYY-MM-DD-<topic>-{plan,review,results}.md` |
| `docs/research/` | 조사, 비교, 역분석 | `YYYY-MM-DD-<topic>.md` |
| `docs/recordings/` | 실기기 측정 기록 | `YYYY-MM-DD-<device>-<topic>.md` |
| `docs/audit/`, `docs/refactor/` | 2026-10-06 시점의 baseline과 리팩토링 증거 (역사 기록) | 날짜 디렉터리 |

- 큰 로그와 산출물은 `build/<lane>-YYYYMMDD/`에 두고, 문서에는 경로와 hash만 남깁니다.
- 새 결과 문서를 쓴 다음에는 `STATUS.md`의 해당 절을 갱신합니다.

## 기준 문서 (현재 유효)

| 주제 | 문서 |
|---|---|
| 장기 목표와 phase gate | `refactoring-plan.md` (P0–P6, G0–G6) |
| 개발 명령과 runbook | `../scripts/dev/README.md` |
| 개선 로드맵 | `research/2026-10-14-slam-improvement-plan.md` |
| 상용 SLAM 비교 (8th Wall, ARCore, VINS-Mono) | `research/2026-10-14-slam-comparison.md`, `research/2026-10-14-commercial-slam-lab.md`, `research/2026-10-14-physical-arcore*.md` |
| Flip6 분석 종합 | `research/2026-10-13-flip6-analysis-and-application.md` |
| tracking 회복 결과 (C3, 현재 TUM baseline) | `fixes/2026-10-13-flip6-tracking-recovery-results.md` |
| UI/replay 수정 (U/R) | `fixes/2026-10-14-reference-reviewed-runtime-fixes.md` |
| 서버 WASM 승격 | `fixes/2026-10-14-server-ready.md` |
| low-feature 계획(v3 승인)과 결과 | `fixes/2026-10-14-low-feature-plan.md`, `fixes/2026-10-14-low-feature-results.md` |
| 실기기 최신 측정 | `recordings/2026-10-12-zflip6-after-optimization.md`, `recordings/2026-10-12-zflip6-latency.md` |
| 수학과 입력 경로 감사 | `fixes/2026-10-09-mobile-web-system.md`, `research/2026-10-09/` |
| 다음 Flip6 측정 요구사항 | `fixes/2026-10-07-zflip6-followup.md` |

## 역사 기록 / 대체됨 (근거로 인용하지 말 것)

| 문서 | 이유 |
|---|---|
| `analysis-report.md` | 2026-03 분석. 문서 스스로 근거로 쓰지 말라고 밝히고 있음 |
| `audit/2026-10-06/` | 10-06 초기 baseline. 현재 상태가 아님 |
| `refactor/2026-10-06/` | 10-07 리팩토링 결과. `execution-plan*.md`, `runtime-status*.json`은 과거 기록이며 그 안의 PID는 지금 존재하지 않음 |
| `fixes/2026-10-07-mobile-divergence-plan.md` | `fixes/2026-10-07-zflip6-followup.md`로 대체됨 |
| `fixes/2026-10-10-mobile-rotation.md`의 "App 기본 OFF" | 10-12부터 App 기본값이 ON으로 바뀜 |
| `fixes/2026-10-12-*-plan*.md`, `*-test-spec.md`, `fixes/2026-10-12-recovery-performance.md` | A2 기각, H4 보류로 종결됨 |
| `fixes/2026-10-13-{observation-rejection,depth-retention,experimental-policy}-plan.md`, `fixes/2026-10-13-code-review.md` | 본문에 PENDING이 남아 있지만 실제 결론은 `research/2026-10-13-flip6-analysis-and-application.md`에 정리됨 |
| `research/2026-10-14-low-feature-*-review-v1.md`, `*-v2.md` | v3로 대체됨 |
| `../.omx/notepad.md`, `../.omx/plans/` | Codex 작업 메모 (10-06 기준). `prd-low-feature-20261014.md`의 "review pending" 표기는 실제와 다름 (v3 승인됨) |
| `recordings/2026-10-07-zflip6.md` | 측정값은 유효함. 단 설정과 loader 정보가 오래됨 |
