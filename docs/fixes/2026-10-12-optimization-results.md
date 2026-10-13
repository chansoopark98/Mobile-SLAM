# Flip 6 회전 추적·IMU·프레임 제출 최적화 결과

- 날짜: 2026-10-12, Asia/Seoul. 기준 HEAD `98b396c`; 수정은 현재 working tree에 반영
- 입력 근거: [실기기30초 측정](../recordings/2026-10-12-zflip6-latency.md), [수정 개발 플랜](2026-10-12-optimization-plan.md)
- 가상 검증 판정: **수치·회귀·독립 합성·WASM·실제 App 검증 통과, 테스트 주소 반영 완료**
- 후속 실기기 재시험: **처리율15.95→20.21fps, IMU 구간 Lost6→0. 영상 제약 Lost33개·출력 불연속이 남아 수락 조건 미충족**. [실제 재시험 결과](../recordings/2026-10-12-zflip6-after-optimization.md)
- 보존: 원본 NAS/전화 기록·TLS·기존 서버, baseline source/native/WASM, 기각 후보와 실패 로그. 새 제품 dependency0
- 원본30초: 실효16fps, fresh447/480, Lost33(영상제약27/IMU6), 엔진 p95 62.985ms, 수신 pose age p95 117.896ms

## 반영한 수정

- 회전 추적: reverse/photometric 검증을 통과한 새 대응의 F inlier 지지가 기존 지지의2배를 넘을 때만 교체. 채택하지 않은 trial은 기존 대응과 RNG 상태 유지
- 회전 비용: ordinary majority가 충분하면 상세 진단 외의 불필요한 ray lift/project 생략. 새 후보 수가 기존 지지를 넘지 못하면 reverse/refit도 생략
- IMU: affine world acceleration의 위치 시간모멘트 `h²(u0/3+u1/6)`와 속도 `h(u0+u1)/2`를 preintegration·Estimator·PnP에 일치 적용
- IMU Jacobian: 실제 normalized quaternion increment의 right derivative와 F/V 일치. 회전행렬 재사용·고정 크기 임시 행렬 사용
- 단일 양수 적분 구간: 명시한 독립 endpoint noise 모델에서 full-rank 공분산 구성. finite/symmetric/positive pivot whitening 검사 유지
- bracket: 최근 유효 source 간격16개의 p95로 대기 시작 때 `clamp(2×p95,40,80)ms` deadline 고정. 실제 미래 sample 필요, invalid/order/gap/reset 처리 유지. 추가 지연 허용 상한40ms
- 웹 전송: 이미 소유한 전체 grayscale buffer만 직접 transfer. reusable camera buffer·partial view는 기존 복사 보호
- 제출: rVFC 완료/Worker 완료/단일 pacing timer에서 최신 소유 packet 제출. 33/50ms pacing·warmup·pixel/time·epoch/geometry·single in-flight 유지. UI/render는 rAF 유지
- 수명주기: stop/error의 rAF handle 취소와 generation 검사로 구세대 callback의 startup 침범·중복 루프 방지. 성공 응답의 Lost는 다음 프레임 진행, transport 오류의 즉시 제출은 억제
- 진단/재시험: frame별 IMU/frontend/backend/Ceres/marginalization 시간, 유효 factor와 탈락 사유 수. guard reset 이전의 `candidatePipeline` 별도 보존. 30초 화면 타이머와 정확히 매칭된 source/copy/roundtrip/UI 상태 시각 추가
- 기존 solver60ms/10iterations, feature120, 해상도480×640, LK21/3, 수치/cheirality/motion guard 수락 조건 유지

## 기존 정확도 수락

- 전체 TUM-VI, camera→body, 일대일 timestamp association, 단일 SE(3), scale=1. ATE/APE는 RMSE
- 기준: 기존 기본 OFF 대비 위치·회전 각각 악화≤5%, pose coverage 감소≤1percentage point. 신규 회전 ON 통합 빌드로 두 구간 통과

| 구간 | 기존 ATE m / APE ° | 수정 ATE m / APE ° | pose/전체, GT 대응 |
|---|---:|---:|---:|
| room1 | 0.05174545 / 1.16332455 | 0.05174830 / 1.16476544 | 2776/2821, 2719 |
| room4 | 0.04692050 / 0.91831158 | 0.04693203 / 0.91817288 | 2148/2228, 2108 |

- coverage 동일: room1 98.4048%, room4 96.4093%. 가장 큰 회전 RMSE 증가 약0.124%, 위치 증가 약0.025%
- 수정1초 RPE: room1 0.013873m/0.559229°(2655쌍), room4 0.013239m/0.664057°(2067쌍)
- 근거: `build/optimize-20261012/combined-room{1,4}-on-evaluation.json`, `verification.json`

## 회전·IMU 독립 입력

- 동결 fast101/707/2203, 알고리즘 동결 후 새 확인 seed8809. 독립 rendered pixels·analytic IMU·withheld GT, 20초/400frame, portrait480×640, IMU60Hz
- 고정 warmup `[5,20)` 300frame. 아래 native4개 모두 초기화 후300/300, 전체359/400(89.75%), epoch reset0. 전체 분모도 유지

| 입력 | 기존 기본 OFF warmup coverage | 수정 ATE m / APE ° |
|---|---:|---:|
| fast101 | 60.67% | 0.040274 / 0.676639 |
| fast707 | 60.00% | 0.053029 / 2.717056 |
| fast2203 | 74.33% | 0.031110 / 1.262106 |
| 확인 fast8809 | 새 확인 입력 | 0.034912 / 1.399364 |

- 실제 HTMLVideo→Camera→IMU→App→Worker→renderer: DeviceMotion fast101, Generic fast8809 각각 warmup300/300·reset0·GT gate 통과. 실제 readback solver0.06/features120/min_dist30/rotationRescue1 확인
- raw WASM fast8809와 native: 400개 input timestamp/pose-valid 동일, pose matrix 원소 최대 차이 `2.06165e-11`
- 저주파 수치 stress: clean101 영상/GT 불변, noiseless analytic IMU만10Hz. 기존0pose/400·invalid interval339 → 수정339pose/400, warmup300/300, ATE0.014774m/APE0.463934°. 실제 Flip6 sampling-rate 모사 아님
- 주의: 기존 기각된 실험 ON의 fast707 APE1.1165°보다 이번2.7171°는 악화. 해당 ON은 TUM gate 실패 후보이며 제품 기본값이 아니었음. 모든 실험 조건에서 이전 최선값을 동시에 개선했다는 주장 제외

## 처리 시간·자연 rAF

- 같은 WASM·독립 seed8809 입력의 순차 desktop 자연rAF3회 before/after. capture/sensor metadata는 mock, 관측 부하 포함

| 항목 | rAF 제출 대기 | 이벤트 제출 |
|---|---:|---:|
| 각 실행 warmup coverage | 98.0/97.67/99.67% | 100/99.33/100% |
| 실행별 pose-age p50 중앙값 | 49.305ms | 45.692ms |
| 실행별 pose-age p95 중앙값 | 82.494ms | 75.169ms |

- 관측 중앙값 변화: p50 −7.33%, p95 −8.88%. 모든 실행 source pixel/timestamp/hash·GT gate 통과, reset0. 모든 반복의 지연이 일관되게 감소한 것은 아님
- 추가 수명주기 수정 후 현재 App/Wrapper 해시의 자연rAF 확인: warmup299/300(99.67%), reset0, ATE0.029843m/APE0.732312°, GT gate 통과
- native clean400frame 직렬3회: tracking p50 중앙값8.820→8.649ms, p95 21.874→21.816ms. 큰 엔진 속도 향상으로 해석하지 않음
- IMU 단독 미세 benchmark 직렬14회: 300integration×60interval/반복, 중앙값83.179→86.234ms(+3.67%). 정확한 수치 모델의 추가 CPU 비용 존재
- 개선 확인 범위: 불필요한 복사·회전 예측·rAF 제출 대기 제거. 실제 Flip6 FPS/열/pose age 개선량은 아직 미측정
- 근거: `scheduler-performance.json`, `engine-performance.json`, `imu-performance.json`, `app-fast8809-final-natural-evaluation.json`

## 검증·기각 기록

- native: **168 tests / 14 CTest suites 통과**
- ASan/UBSan/LSan: 영향6suite **116 tests 통과**, skip/suppression0. 기존 sanitizer binary 보존 후 해당 cache의 source를 재빌드
- browser: Node **250/250**, TODO/skip0, exporter analytic Python3/3
- quick: JS/Python/shell syntax, Python evaluator9+synthetic19=28 통과. ESLint/TypeScript 설정 없음
- genuine clangd23.1.0: Estimator/Optimizer/Tracker/PnP/Engine5 source error_count0
- fresh WASM API/reset/pipeline smoke, 실제 Worker·실제 App·실제 HTMLVideo의 rAF 이전 제출/픽셀·시각 소유 검증 통과
- 독립 리뷰: IMU 모델/소유권/Worker deadline·복원·루프 수명주기 검토. guard reset 진단 소실과 구세대 rAF 문제 수정·회귀, 재검토에서 두 항목 해결 확인
- 기각: rotation drift-bound v2의 TUM room1 ATE+9.44% 보존·철회. 과반/mature/spatial threshold만으로 fixture를 분리하는 후보도 적용 전 기각
- 초기 실패 보존: baseline과 새 JS test 편집이 겹친2fail은 동결 source에서221/221 재확인; 기존 midpoint oracle2개는 새 affine 해석값으로 재유도, 허용 오차 유지; default OFF→App ON 정책 테스트 변경 기록 유지
- 종합 증거: `build/optimize-20261012/verification.json`. tracker 실패/격리 결과: `build/optimize-tracker-evidence-20261012/`

## 빌드·제공 상태

- native/WASM core source SHA256 동일: `2f143ca42c92c360552897f25ab6293603086f20e0b0f017b9f0fef64e283880`
- native SHA256: `43418010831c8b8747bf1e5c25ca88bc9815eb58132fb396c8449d27159bad7c`
- 활성 embedded WASM loader: `web/vio_engine.js`, 6,025,183bytes, SHA256 `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`
- 활성 sidecar: `web/vio_engine.manifest.json`; candidate는 `build/optimize-wasm-20261012/vio_engine.js`
- 이전 loader/manifest backup: `build/refactor-deploy/1791788586578864844/`; 원자적 승격 기록 `build/optimize-20261012/deployment.json`
- 기존 direct HTTPS7002 서버 재시작 없음. 실제 strict-TLS desktop GET의 App/Wrapper/Worker/loader hash가 local과 일치, 제공 중인 실제 Worker의 새 pipeline binding·rotation readback1 확인
- `__health__`의 artifactSnapshot은 서버 시작 시1066a22d… 기록이 남음. 현재 제공 버전 근거는 실제 GET 및 `served-browser-verification.json`, startup snapshot과 구분
- App SHA256 `6311a5a2e990ccd962d1f348bbf8e3c479bf1173f8a11e6a31b85b9efe0682c1`; Wrapper `4a18003a70890caef2bfd418ea3c2c03db5d64682f1ae82cf9c16384eb325206`

## Flip6 재시험

- 주소: `https://dev.serdic.com:7002/?camera=0`, **새로고침 후 Start**, 부드러운 이동으로 Tracking 확인
- App 기본 회전 복구 ON. 비교 control은 `&rotation-rescue=0`, ON 명시는 `&rotation-rescue=1`. C++/Worker의 생략 configure 기본 OFF 계약은 유지
- Tracking 준비 후 numeric 기록: 아래30초 화면 타이머 사용. 0–10초 정지/10–20초 평소 Lost가 발생하는 정도의 이동·회전/20–30초 정지. Lost 때 수동 reset 없이 유지

```bash
node scripts/dev/android-snapshot.mjs --samples 5 --record-seconds 30 --timer
```

- 비교: 실제 gyro/영상 시각, accepted/completed/coalesced, fresh/전체 source coverage, Lost/reset 이유, pipeline 단계·factor 수, IMU wait budget, pose age p50/p95/p99, 온도·지속 처리
- 재현 필요 시 별도 image/IMU host-stream 기록. numeric 기록은 영상이 없어 correspondence/blur를 재생할 수 없으며 두 recorder 동시 사용 금지
- 실측 K/extrinsic·노출/IMU offset·EIS/OIS·독립 GT 미확인. 실제 발산0/Lost≈0·물리 정확도·장기 열 안정성은 재시험 전 수락 판정 제외
- 같은 raw bracket을 공유하는 보간 endpoint의 실제 noise 상관은 기존 모델에서도 미반영. 이번 full-rank 증명은 명시한 독립 endpoint noise 모델 기준

## 변경 파일

- IMU: `include/backend/factor/integration_base.h`, `src/backend/estimator.cpp`, `src/frontend/pnp_frontend.cpp`
- 추적/계측: `src/frontend/feature_tracker.cpp`, `src/backend/optimizer.cpp`, `src/vio_engine.cpp`, `include/{vio_engine.h,backend/solver_diagnostics.h}`, `wasm/vio_bindings.cpp`
- 웹/산출물: `web/js/{app,vio-wrapper,vio-worker}.js`, `web/vio_engine.js`, `web/vio_engine.manifest.json`
- 도구/회귀: `scripts/dev/{android-snapshot,android-input-record,wasm-smoke}.mjs`, `scripts/dev/README.md`, `CMakeLists.txt`, `CMakePresets.json`, native/browser tests, 본 계획/결과 문서
- 단순화: 소유 packet의 중복 copy 제거, 프레임 제출 본문을 한곳에서 재사용, 필요 없는 reverse/refit/lift/project 생략. 새 dependency·일반 event framework 없음
