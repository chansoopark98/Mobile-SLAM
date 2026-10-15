# Mobile-SLAM 현재 상태 (Living Status)

- 최종 갱신: 2026-10-15 (low-feature opt-in 전환, `887ea58`)
- 갱신 규칙:
  - 작업 단위가 끝날 때마다 §1, §3, §6을 갱신합니다.
  - 숫자에는 출처 문서 경로를 붙입니다.
  - 과거 기록은 `docs/fixes/`와 `docs/research/`에 남기고, 이 문서에는 "지금"만 적습니다.

---

## 1. Git·작업 트리 상태

| 항목 | 상태 |
|---|---|
| 현재 branch | `low-feature-opt-in` (`main`에서 분기, `887ea58` + 이 STATUS 커밋). **main에 병합 안 됨.** `main`은 2026-10-15에 `codex/flip6-analysis-20261013`를 fast-forward 병합한 상태 |
| `origin/main` | `f210570`. 로컬 `main`이 **12커밋 앞서 있고 push되지 않음** (2026-10-15 기준, `git rev-list --count origin/main..main`) |
| 기타 branch | `codex/*` 4개: 모두 main에 병합됨(삭제 가능) |
| 미커밋 WIP | 없음. 2026-10-14 Codex WIP는 2026-10-15에 4개 커밋(`789ed3d`, `4d74630`, `04386e5`, `f76217d`)으로 정리됨. 내용은 §3. low-feature opt-in 전환은 `887ea58` |
| 활성 WASM | `web/vio_engine.js` sha256 `38837d4…` (6,044,038B). `5d48e4b` clean tree 기준, C3 OFF 빌드 |
| 소스와 WASM 일치 여부 | **불일치.** `f76217d`(low-feature)가 `src/vio_engine.cpp`, `include/vio_engine.h`, `include/backend/estimator.h`, `wasm/vio_bindings.cpp`를 바꾸고 `include/utility/inertial_prediction.h`를 추가함(`887ea58`에서 include만 보강). 활성 WASM은 그 이전 소스(`5d48e4b`) 빌드 |
| 검증 baseline (2026-10-15, `887ea58` tree) | `check.sh native` CTest 16/16 통과(`build/dev-native`), `browser-contracts` 353/353 (skip·todo 0, opt-in 테스트 4개 추가), `eval` 19 OK, `syntax` OK. **VERIFIED** — 단, WASM은 다시 빌드하지 않았고 Playwright 브라우저 스크립트·실기기 측정도 하지 않음 |
| low-feature 후보 WASM | `e7420b36…` (6,056,932B). `build/low-feature-20261014/` 안에만 있고 **승격 안 됨** |

---

## 2. 엔진 현재 상태

### 실행 경로

- Native: `tiny_vins_mono → VIOSystem → MeasurementProcessor → VIOEngine`. 2026-10-06 리팩토링 이후 core를 공유합니다.
- Browser: `App → Camera(rVFC 동기 capture)/IMU → VIOWrapper → Worker → VIOEngine (WASM)`.

### 기본값과 실험 옵션

| 항목 | 기본값 | 비고 |
|---|---|---|
| `rotation_rescue` | C++ OFF / **App ON** | `?rotation-rescue=0`으로 끔 (`web/js/app.js:702`) |
| observation policy | 0 | 1, 2(D2/D3)는 기각되었고 opt-in으로만 격리 |
| `MOBILE_SLAM_EXPERIMENTAL_ANCHOR_DEPTH` (C3) | OFF (CMake option) | raw stability FAIL로 기본값 채택 거부 |
| capture | sync rVFC | `?capture=worker`는 실험용. 지연 gate 실패 |
| PnP | OFF | |
| mobile solver | 0.06s / 10 iter / 120 features | `web/js/app.js:64-69`. 2026-03 메모리의 0.04/8/100은 **구버전** |
| camera calibration | `heuristic_unverified` | fx≈465.6, 69° FOV 가정. 실측값 없음 |

### 정확도 (SE(3), scale=1, body frame)

| 데이터 | room1 ATE / APE | room4 ATE / APE | 출처 |
|---|---|---|---|
| TUM VI baseline (C3 OFF, 10-13) | 0.05006m / 1.231° | 0.04690m / 1.099° | `docs/fixes/2026-10-13-flip6-tracking-recovery-results.md` |
| TUM VI C3 ON (참고용, 채택 안 함) | 0.04924m / 1.232° | 0.04659m / 1.041° | 같은 문서 |
| VINS-Mono 원본, 같은 입력 600 frame | ATE 0.01497m (현재 엔진 0.01452m) | — | `docs/research/2026-10-14-commercial-slam-lab.md` |

- TUM room2(held-out)는 10-07에 0.0665m를 1회 측정했습니다.
- EuRoC는 현재 엔진으로 평가한 적이 없습니다.

### 실기기 (Galaxy Z Flip6, 10-12 기준)

출처: `docs/recordings/2026-10-12-zflip6-after-optimization.md`

- 처리율 20.21fps, fresh 94.56%.
- **Lost 33 frame** (연속 구간 6개). 모두 current visual factor 0~1개 상태에서 발생했습니다.
- engine p95 69.8ms, pose age p95 120.5ms.
- 인접 프레임 간 점프가 최대 1.07m / 44.7°까지 관측되었습니다.
- 독립 GT, 실측 calibration, exposure timestamp는 모두 없습니다.

### 비교 기준 (ARCore)

- 같은 Flip6에서 ARCore는 607 frame 중 601 frame이 TRACKING이었습니다(99.0%).
- 출처: `docs/research/2026-10-14-physical-arcore*.md`

---

## 3. 2026-10-14 Codex 작업 (2026-10-15 커밋됨)

아래 그룹으로 커밋했습니다. U/R과 low-feature는 `app.js`·`renderer.js` hunk가 섞여 있어 한 커밋(`f76217d`)에 들어 있습니다.

| 그룹 | 파일 | 상태 |
|---|---|---|
| **U/R 수정** (UI freshness 500ms, 상태 문구 10Hz 제한, Worker error 표시, replay `--imu-policy future-bracket`) | `scripts/dev/synthetic-browser.mjs`, `scripts/dev/README.md`, `app.js`/`renderer.js`의 U hunk, 관련 browser 테스트, `docs/fixes/2026-10-14-reference-reviewed-runtime-fixes.md` | browser 297/297, 392 frame replay 일치 |
| **리서치·레퍼런스 도구** (8th Wall/ARCore 역분석, 상용 SLAM 비교) | `docs/research/2026-10-14-*`, `scripts/dev/references/{arcore,eighthwall}/`, `scripts/dev/analyze-slam-bottlenecks.py` | 문서와 분석 스크립트 |
| **서버 WASM 승격** (HEAD 기준 빌드) | `web/vio_engine.js`, `web/vio_engine.manifest.json`, `docs/fixes/2026-10-14-server-ready.md` | 승격 완료. 서버 시작은 사용자가 담당 |
| **low-feature core** (IMU shadow 예측 ≤0.5s, 바닥 후보 점 export) | `include/utility/inertial_prediction.h`, `include/vio_engine.h`, `src/vio_engine.cpp`, `include/backend/estimator.h`, `wasm/vio_bindings.cpp`, `tests/test_engine_contracts.cpp` | native 테스트 13개 추가 |
| **low-feature browser** (평면 fitter, amber frustum, 바닥 mesh) | `web/js/low-feature.mjs`, `vio-worker.js`, `vio-wrapper.js`, `app.js`/`renderer.js` hunk, `tests/browser/low-feature*.test.mjs`, `browser-source-loader.mjs`, `transport.test.mjs`, 그 외 소규모 테스트, android/wasm-smoke 스크립트 | browser 349/349 |
| **low-feature 문서** | `docs/fixes/2026-10-14-low-feature-{plan,results}.md`, `docs/research/2026-10-14-low-feature-*` | 설계 v3 승인됨(설계 승인일 뿐 구현 승인이 아님) |

### low-feature의 위험·미완 사항

출처: `docs/fixes/2026-10-14-low-feature-results.md`, 2026-10-15 WIP 분석

- **바닥 GT gate 미달**: 71/79 = 89.87%로 기준 90%를 넘지 못했습니다. 그래서 서버 승격을 보류했습니다.
- **기본 OFF로 전환됨** (`887ea58`, 2026-10-15). 켜는 방법:
  - App: `?low-feature=1` (다른 값은 모두 OFF)
  - Wrapper/Worker: `configure({ lowFeature: true })` (boolean `true`만 인정, 재configure 시 플래그가 없으면 OFF로 돌아감). Worker configure 응답의 `lowFeatureEnabled`로 활성값을 확인합니다.
  - 개발 도구: `synthetic-browser.mjs --low-feature`. `replay-android-input.mjs`는 플래그가 없던 과거 기록과 비교할 수 있도록 기록된 값이 없으면 ON으로 replay합니다.
  - C++ engine의 shadow 예측과 바닥 후보 점 export는 그대로 있고, Worker가 opt-in 없이는 읽지 않습니다.
- cache-busting 정리됨 (`887ea58`): `app.js`, `vio-wrapper.js`, `camera.js`, `imu.js`, Worker URL, `low-feature.mjs` import를 `?v=14`로 통일했습니다. `renderer.js?v=13`, `orientation.js?v=11`, `capture-worker.js?v=1`은 마지막 bump 이후 내용이 바뀌지 않아 유지했습니다. `test-tumvi-app.js`(데스크톱 도구)의 import는 버전 query 없이 그대로입니다.
- `inertial_prediction.h`에 `<cmath>`와 `<limits>`를 직접 include함 (`887ea58`).
- `vio-wrapper.js`의 `_invalidateLowFeature` 실패를 `catch(() => {})`로 무시합니다. 문서상으로는 Worker fence가 이를 보완합니다.

---

## 4. 유지해야 할 Gate

결과를 본 뒤 gate를 바꾸지 않습니다.

- **TUM**: baseline(§2) 대비 위치와 회전 각각 ≤1.05배, coverage 손실 ≤1%p. room4 회전이 0.918°(`docs/fixes/2026-10-12-optimization-results.md`)에서 1.099°로 악화된 상태는 실패 기록으로 보존되어 있습니다.
- **churn 합성 시나리오**: fresh ≥280/300, current-support 실패 <35, Lost 구간 <7, reset 0, ATE ≤0.25m, APE ≤5°, 1s RPE ≤0.10m / 3°. 현재 미달입니다.
- **기하 조건**: κ ≤ 10000, 모든 view에서 재투영 ≤1.5px, current support ≥2, cheirality.
- **성능 변경** (`docs/research/2026-10-14-slam-improvement-plan.md`): stage별 ABBA 측정에서 p95 ≥10% 감소, coverage 손실 ≤1%p.

---

## 5. 기각된 시도 (재시도 금지)

| 시도 | 결과 | 문서 |
|---|---|---|
| inverse-depth bound, unusable→prior 폐기, log-depth manifold | 10-07 기각 | `docs/fixes/2026-10-07-zflip6-followup.md` |
| A2 persistent geometry admission | churn ATE 0.318m / 13.3°로 기각 | `docs/fixes/2026-10-12-recorded-recovery-implementation-review.md` |
| H4 projective recovery | solver oracle 실패 | `docs/fixes/2026-10-12-projective-recovery-implementation-review.md` |
| capture Worker(C1) 기본값 채택 | 지연 gate 실패 | `docs/fixes/2026-10-12-capture-worker-review.md` |
| D0/D2/D3 observation policy | 기각 후 opt-in으로 격리 | `docs/fixes/2026-10-13-tracking-performance-results.md` |
| gyro patch warp | frame204에서 가려진 점을 잘못 채택(false ID) | `docs/research/2026-10-13-flip6-analysis-and-application.md` |
| C3 anchor-depth 기본값 채택 | reset 2→3, pose step 3.62m | `docs/fixes/2026-10-13-flip6-tracking-recovery-results.md` |
| Backend 수치 패치, solver <0.04s / <8 iter, Ba/velocity clamp | native 발산 (2026-03 실험, 저장소 밖 기록) | `CLAUDE.md` §2에 규칙으로 정리 |

---

## 6. 다음 단계 (우선순위 순)

1. **`low-feature-opt-in` 브랜치 병합과 WASM 재빌드**
   - opt-in 플래그와 cache-busting은 `887ea58`에서 끝났습니다(브랜치 `low-feature-opt-in`, main 미병합).
   - 병합 후 WASM을 다시 빌드하고 provenance를 기록합니다. 승격은 사용자가 요청할 때만 합니다. 승격해도 low-feature는 `?low-feature=1` 없이는 꺼져 있습니다.
2. **low-feature 바닥 gate 미달 원인 규명**
   - 평면 재추정 쪽을 조사합니다.
   - 89.87%를 90% 이상으로 맞추려고 gate를 완화하는 것은 금지입니다.
3. **개선 계획 실행** (`docs/research/2026-10-14-slam-improvement-plan.md`): 0A artifact 고정 → 1A camera/IMU 입력 계약 기록 → 1B live 스케줄 replay → 2 stage별 ABBA → 3 feature ID lineage 계측 후 descriptor 재연결 → 4 8th Wall 실기기 비교와 독립 GT.
4. **실기기 Lost(visual factor 0~1) 대응**
   - 일반적인 projective/structureless 회복 모델을 연구합니다.
   - `compensatedParallax2`가 실제로는 회전을 보상하지 않는 문제를 다룹니다. 회전 keyframe과 병진 keyframe을 분리하는 것이 후보입니다.
5. **새 Flip6 측정** (`docs/fixes/2026-10-07-zflip6-followup.md`)
   - 초기화부터 발산 구간까지 영상과 IMU를 연속으로 기록합니다.
   - 실측 K/extrinsic, EIS/OIS 조건, time offset을 확보합니다.
6. **장기 과제** (`docs/refactoring-plan.md`)
   - P2 contract(`config/validation/p2-contract.yaml`) 작성 — 아직 없음
   - field GT 확보
   - iOS
   - 30분 연속 실행
   - loop closure와 재지역화
   - GPLv3 provenance 판단
