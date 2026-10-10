# PC·모바일 웹 입력/수학 진단과 수정

- 기준일: 2026-10-09, Asia/Seoul
- 요청: Z Flip 6 보존 기록 재분석, PC/모바일 경로 비교, 논문·공개 구현·상용 SDK 조사, 독립 가상 데이터 검증과 수정
- 실행 증거: `build/mobile-web-20261009/`; 원본 전화 기록·dataset·기존 변경 보존
- 관련 조사: [레퍼런스 비교](../research/2026-10-09/reference-comparison.md), [수학·시스템 감사](../research/2026-10-09/math-system-audit.md)
- 현재 범위: 재현한 입력·수학 결함 수정 및 분리 빌드 검증. Z Flip 6 발산 해결·현장 정확도·발열 성능의 합격 근거 없음

## 1. 확인한 원인과 적용 변경

| 원인 | 수정 | 반례·검증 |
|---|---|---|
| rVFC metadata 저장 후 rAF에서 가변 video를 다시 읽어 이전 시각과 새 pixels 조합 가능 | rVFC에서 소유한 grayscale·시각·geometry·설정·generation·Worker epoch packet 1개 유지. rAF는 동일 packet 소비 | 수정 전 자연 rAF source46 불일치 보존. 실제 HTMLVideo A40→B180 후 A/tA 제출, rAF 추가 video 읽기 0 |
| 밝기 차이를 영상 회전으로 추정, `none` 경로의 치수·crop과 K/extrinsic 불일치 가능 | 자동 회전 추정 제거, 실제 video 치수 사용, 명시 회전·중앙 crop·resize를 pixels/K/Ric/왜곡에 같은 순서 적용 | CPU/WebGL 실제 browser geometry 12개, 독립 projection 계약 |
| signed/zero inverse-depth solver trial을 물리 landmark 실패와 혼동 | `h=R_ji ray_i+ρ t_ji` homogeneous projection, 유한한 signed/zero trial 허용. 최종 positive depth·각 관측 cheirality는 feature별 검사 | 독립 차분 207개 geometry×19열, 최대 오차 1.014e-8; invalid 1개로 전체 solve 폐기 방지 |
| quaternion의 packed local Jacobian을 Ceres ambient Jacobian으로 사용; prior의 현재 상태 미분 누락 | right SO(3) Exp/Log, 실제 Plus/Minus Jacobian, 모든 pose factor의 ambient lift, prior residual의 현재 chain rule을 함께 수정 | 독립 finite difference, q/−q·π branch, gauge·prior 정보 보존. 고정 J0/r0와 현재 derivative 구분 |
| IMU bias 회전 보정의 근사 quaternion과 derivative 불일치 | unit Exp bias correction·right Jacobian을 native/PnP factor에 공통 적용 | 비영점 bias·회전의 local/ambient 차분; 원래 midpoint propagation과 covariance 값 유지 |
| 실제 manifold가 pose를 읽을 때 Ceres parameter 등록 전에 값 미준비 | body/extrinsic SO(3) 검증→pose 준비→parameter 등록 순서로 native/PnP 수정 | 실제 초기화 이후 Ceres abort RED 보존, 수정 후 전체 replay 통과 |
| PC 실행의 유실·시각·최종 설정을 core-only replay로 확인 불가 | 실제 `tiny_vins_mono --headless`, 전 frame ledger·effective config·raw input oracle·17자리 pose 로그 | PC entry와 direct native의 실제 gray/IMU bytes·66개 설정·시각·pose 비교 |
| YAML validator는 random-walk noise=0 허용, engine은 거부 | 기존 engine의 positive covariance 계약에 맞춰 YAML도 0 거부·실패 설정 rollback | ConfigValidation 11개; noise 값 재튜닝 없음 |

- 최종 landmark 제거 후 실제 추가된 현재 image residual support 재확인; 전부 사라진 frame을 정상 visual update로 수락하지 않음
- pixel packet 교체는 1개로 제한; null/throw·reset·stop·hidden/focus·orientation·reconfigure에서 폐기·callback token 무효화
- `physicalExposureVerified=false` 유지: callback 안의 pixels/metadata 결합과 실제 노출 시각 정확도는 별도
- 미사용 `ProjectionFactor::check` 삭제; SO3/ambient lift 공통 helper 재사용. 수치 계산 순서 변경을 위한 추가 cleanup 제외

## 2. 실제 PC와 모바일 웹 차이

| 항목 | PC 실제 실행 | 모바일 웹 실제 App |
|---|---|---|
| 경로 | `tiny_vins_mono → VIOSystem → MeasurementProcessor → VIOEngine` | `App → Camera/IMU → VIOWrapper → Worker → VIOEngine` |
| 엔진 | 현재 `VIOSystem`도 공유 `VIOEngine` 호출 | 같은 C++ core의 WASM 빌드 |
| 입력 | 보정된 파일의 gray·CSV timestamp·IMU bracket | HTMLVideo/readback, API sensor event, 비동기 queue·rate limit·warmup |
| 보정 | YAML K·왜곡·camera→body extrinsic | 제공 profile 또는 `heuristic_unverified`; pixel 변환과 함께 보정 변환 |
| 시각 | 파일에 고정된 acquisition 시각 | capture/presentation/arrival 출처·origin 확인 필요; 물리 camera/IMU offset 자동 보정 없음 |
| 설정 | legacy replay min_dist20·solver cap10s; YAML 모드 별도 제공 | mobile_default min_dist30·solver cap0.06s·motion guard; F threshold engine 기본1, URL로 변경 가능 |
| 공개 계측 | 전체 frame ledger·config·input hash 추가 | 실제 C++ effective-config RPC 추가; typed response·epoch·설정 readback |
| 라이브러리 | OpenCV 4.5.4 | OpenCV 4.5.5; 차이를 기록한 뒤 같은 입력 수치 비교 |

- PC/direct 검증: 400frame, IMU 4,001행 중 마지막 영상까지 3,992행 전달. 남은 9행은 마지막 영상 이후 tail
- first pose 3.05s, fresh 339개, 고정 `[5,20)` coverage 100%, reset 0
- gray/IMU input oracle·설정·출력 source timestamp 일치; 위치 최대차 0m, 회전 최대차 1.708e-6°
- native/WASM 동일 입력: first initialization·pose coverage 일치, 위치 최대차 약 1.44e-11m
- App가 실제 받아들인 pixels·IMU batch 경계·effective config를 다시 Worker에 전달하는 별도 재생으로 adapter와 core 차이 분리
- 합성 acceptance의 live URL은 명시 `fth=1`; 현재 engine 기본값도1. App header의 오래된 기본5 주석 정정, URL 없는 실제 readback도 별도 확인. solver/feature profile 차이를 숨긴 상태로 PC/WASM 정확도 비교하지 않음
- 근거: `pc-adapter/final-clean101/comparison.json`, `phase-b-browser-raw-clean101/native-comparison.json`, `phase-b-browser-captured-clean101/live-comparison.json`

## 3. 수학·알고리즘 검증

- pose: camera→body→world 구성·역변환, 회전/crop 이후 K, 비영점 lever arm, quaternion 정규화·표현 불변성
- solver: inverse-depth trial 정의역과 최종 물리 유효성 분리, 현재 frame support·feature ID·prior parameter address 보존
- derivatives: projection·perspective·IMU·PnP IMU·marginalization의 ambient/local 계약 동시 검증
- preintegration: endpoint bracket·timestamp 초·specific force m/s²·gyro rad/s, bias relinearization, covariance finite/PSD와 sampling-rate 의미 구분
- 초기화: 정지·순수회전·일정속도-only 음성 사례는 400frame 처리 후 pose 0개가 기대 결과; 정상 입력 pose 0개는 실패
- marginalization: frozen prior J0/r0 유지, 현재 local-coordinate residual의 chain rule, q/−q 및 π cut의 deterministic convention
- 정밀도 평가: 독립 body GT, 일대일 source association, 전체 유효 pose 단일 SE(3)·scale=1, 위치/회전 ATE·RPE·coverage. Sim(3)는 scale 진단만 사용
- 검증 한계: 유한한 회귀·수치 차분·생성 분포. 모든 수학의 형식 증명이나 모든 motion의 관측가능성 보증 아님

## 4. 보정된 실데이터 회귀

- 공통 조건: full sequence, 같은 rig YAML, PnP0, seed0, OpenCV thread1, solver iterations10/cap10s, 같은 tracker·GT association
- 수락 기준: 초기 baseline 대비 위치/회전 RMSE 5% 초과 악화 없음, pose coverage 1percentage point 초과 감소 없음

| 시퀀스 | baseline 위치 ATE | 수정 후 위치 ATE | baseline / 수정 후 회전 APE | fresh pose |
|---|---:|---:|---:|---:|
| TUM-VI room1 | 0.05316804m | 0.05174410m | 1.20310° / 1.16332° | 2,776 / 2,821 동일 |
| TUM-VI room4 | 0.05131678m | 0.04692050m | 1.25120° / 0.91831° | 2,148 / 2,228 동일 |

- 두 시퀀스 수락 통과: `phase-b-control-gate.json`
- Phase A만의 중간 결과도 보존; 최종 수치를 특정 한 수정의 독립 효과로 해석하지 않음

## 5. 독립 가상 데이터

- generator: NumPy/SciPy analytic 6-DoF trajectory·ray-cast nonplanar texture/occlusion. estimator 코드를 GT 생성에 사용하지 않음
- 공통: 20초·영상20Hz·400frame, full-output 단일 SE3, warmup 분모 `[5,20)` 300frame 고정
- 정상: IMU200Hz, known K/extrinsic, global shutter. 모바일 모사: IMU40/60Hz, 0.1m/s²·0.1deg/s 양자화, 명시 noise/bias/random walk
- 개발 seed101/202, 독립 확인 seed707. 설정·scene·임계값을 실행 전에 저장, 결과를 본 뒤 재튜닝 없음
- landscape320×240 및 portrait480×640; 실제 휴대전화 noise·calibration 추정치 아님
- native 개발 7종 통과: clean101/202·mobile40/60 정확도, stationary/pure-rotation/constant-velocity 초기화 음성 사례
- clean101 native: ATE0.01424749m, APE0.439281°, RPE1s0.01163065m/0.262875°, warmup 이후100%
- 선택 PnP 경로도 별도 실행: 335fresh, ATE0.01362380m, APE0.379715°, warmup 이후100%
- browser 실제 HTMLVideo·drawImage·App·IMU·Worker·Three renderer 실행; 가상 sensor/camera API 경계만 대체
- Generic Sensor/DeviceMotion 비교, 축·deg→rad·clock 변환 포함; DeviceMotion 역변환 수치 오차 최대1.39e-17
- Phase B 초기 7개 browser GT 통과는 Phase C 이전 증거. 이후 자연 rAF에서 발견한 source46 불일치를 숨기지 않고 반례와 수정·반복 수락을 별도 기록
### 최종 App snapshot 검증

- 자연 rAF·20Hz source 400frame 독립3회(`r2/r3/r4`) 모두 pixels/timestamp/generation/epoch/geometry 일치·GT 수락 통과
- r1은 다른 browser 실행과 겹친 부가 결과로 보존; 독립3회의 대체 근거로 사용하지 않음

| 자연 rAF 반복 | accepted/400 | fresh | 고정 구간 coverage | 위치 ATE | 회전 APE | 초기화 이후 pose age p95 |
|---|---:|---:|---:|---:|---:|---:|
| r2 | 381 | 333 | 100% | 0.016028m | 0.695414° | 39.19ms |
| r3 | 379 | 333 | 99.33% | 0.011505m | 0.598983° | 50.37ms |
| r4 | 381 | 332 | 100% | 0.014423m | 0.437220° | 42.14ms |

- 각400snapshot, capture 실패·geometry 거부0. coalesced11/12/11, warmup5, Worker busy drop3/4/3; 누락 frame도 고정 분모에 포함
- 초기화 이후 engine p95 22.36/24.00/23.90ms. snapshot read p95 약1.34–1.49ms, 소유 buffer 복사·검사 약0.08–0.09ms
- 위 latency는 desktop wall clock·관측 부하 포함. 실제 phone 성능·thermal 또는 카메라 노출 latency의 증거 아님
- 확인 seed707 실제 App portrait: clean354fresh·ATE0.005938m·APE0.334237°, mobile60 354fresh·ATE0.006669m·APE0.332474°
- 두 portrait 경우 warmup 이후300/300·초기화2.30초·reset0. 가상 시계 직렬 처리이므로 pose-age/처리율 실시간 주장 제외
- Generic/DeviceMotion 입력 경로 통과. Generic accepted395frame 및 자연 r4 accepted381frame의 raw 재생은 effective params·state·초기화 동일, 모든 pose matrix 원소 최대차0
- 근거: `phase-c/` 각 `report.json`·`evaluation.json`, `browser-captured-{generic,native-r4}-clean101/live-comparison.json`
- 최종 리뷰에서 오래된 F threshold·rotation·noise 주석만 정정. executable text 동일 비교와 이전 App hash를 `cleanup/comment-only-equivalence.json`에 보존
- 주석 정정 후 현재 hash로 URL override 없는400frame와 자연 rAF 부가400frame 재검증 통과. 기본 F threshold readback1·setFThreshold 호출0, 부가 rAF warmup 이후100%·ATE0.012131m
- 최종 browser13개 실행·GT gate 통과: [요약](../../build/mobile-web-20261009/phase-c/browser-acceptance-summary.json), [전체 증거](../../build/mobile-web-20261009/phase-c/browser-final-summary.json). 잘못된 기본값5 이름의 초기 폴더도 실제 readback1·무 override로 설명하고 보존

### 최초 확인 seed707: native 결과

| 조건 | fresh/400 | 첫 pose | 고정 구간 coverage | 위치 ATE | 회전 APE |
|---|---:|---:|---:|---:|---:|
| clean320×240 | 329 | 3.55s | 100% | 0.010532m | 0.697176° |
| clean480×640 | 359 | 2.05s | 100% | 0.007368m | 0.244103° |
| mobile60·480×640 | 359 | 2.05s | 100% | 0.009272m | 0.396318° |

- 세 조건 reset0·수락 통과. 적용 profile46필드·fixture1,239파일 사후 hash 일치
- clean/mobile portrait 영상400장 동일, sensor noise/rate 차이만 존재. 결과 후 scene·임계값 변경 없음
- 근거: `confirmation-native/confirmation-summary.json`, `synthetic-design/confirmation-fixtures.json`

### 개발 seed101: 입력 오류 민감도

| 주입 조건 | fresh/400 | 고정 구간 coverage | 위치 ATE | 회전 APE |
|---|---:|---:|---:|---:|
| IMU 시각+30ms | 339 | 100% | 0.016854m | 1.241855° |
| IMU 시각−30ms | 339 | 100% | 0.015644m | 1.138874° |
| 적용 fx/fy+10% | 339 | 100% | 0.031404m | 1.704589° |
| Gaussian blur σ3 | 339 | 100% | 0.016182m | 1.076188° |
| 5–10초 무특징 영상 | 223 | **61.33%** | 0.051943m | **6.304127°** |

- 무특징 조건: **coverage·회전 수락 실패**, empty_features102회·insufficient_current_features15회, 10.8초 fresh 출력 재개
- 나머지 네 조건은 이 motion·scene에서만 기준 이내. 해당 시각/보정 오류의 일반 허용 범위 또는 자동 감지·보정 기능을 입증하지 않음
- 모두 400개 physical source ID·시각 보존, 원본 GT에 단일 SE3·scale1 적용, reset0. 잘 나온 구간만 정렬하지 않음
- derived-manifest의 최초 scorer 거부 보존. 추정 결과 변경 없이 동일 SHA의 원본 physical GT/spec로 재평가
- 근거: `sensitivity/README.md`, `sensitivity/results-scored.json`

## 6. 원래 Z Flip 6 기록의 판단

- [원본 보존·인계](../recordings/2026-10-07-zflip6.md)의 60초 로그와 첫300장 영상은 서로 다른 범위
- 독립 GT·실측 K/extrinsic·물리 exposure/IMU offset 없음; 당시 EIS/OIS ON 관측과 heuristic K 사용
- 동일 prefix stress: 시작 baseline253pose/max‖p‖107.28m → Phase A265/100.35m → 최종 math265/356.33m
- **기록 재생의 큰 발산은 미해결이며 최종 norm은 악화**. GT 없는 norm을 ATE로 바꾸거나, calibrated/synthetic 통과로 실기기 개선을 주장하지 않음
- 가능한 물리 원인: unknown camera/IMU delay, 실측과 다른 K/extrinsic, 안정화·crop·sensor 품질. 현재 자료로 개별 원인 확정 불가
- 수학 결함 수정의 타당성과 원래 전화 입력에 대한 robustness는 서로 다른 평가 축. 발산을 clamp·smoothing·pose 숨김으로 감추지 않음

## 7. 레퍼런스와 현재 구조 차이

- [VINS-Mono 고정 commit](https://github.com/HKUST-Aerial-Robotics/VINS-Mono/tree/90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d) 원문 확인. 각 프로젝트 commit·LICENSE·파일별 SHA는 [비교 문서](../research/2026-10-09/reference-comparison.md) 참조
- VINS-Mono/Fusion·OpenVINS: extrinsic/time calibration·관측성·noise 계약 비교. 현재 fixed extrinsic/global-frame-time factor에는 online delay·rolling-shutter 상태 없음
- ORB-SLAM3·VINS pose graph와 달리 현재 핵심은 local sliding-window VIO. loop closure·persistent map·global relocalization 미구현
- ARKit/ARCore: native camera calibration·sensor integration·tracking API와 일반 웹의 high-level sensor/camera 제약 차이. 비공개 solver 내부를 분석 완료로 표시하지 않음
- 상용 SDK: 공개 기능·라이선스만 확인. 8th Wall MIT framework의 공개와 SLAM engine source 공개는 다른 범위
- 수정 순서 판단: 이번 입력/수학 경계 수정→장치별 보정·시각 provenance 확보→독립 timing/readout 평가→필요한 time-offset/rolling-shutter 모델→장기 relocalization/loop closure
- native-backed WebXR는 지원 capability 확인 후 별도 pose source로 비교 가능한 대안. 자체 WASM 결과와 혼합 금지

## 8. 실행·빌드·인계

- native146tests/13suites, browser196contracts＋exporter3tests, evaluator27tests, sanitizer99tests, genuine clangd6source error0
- ESLint/TypeScript 설정 없음: syntax 검사 결과를 lint/typecheck 통과로 표시하지 않음
- candidate WASM: `build/mobile-web-wasm/vio_engine.js`, SHA256 `7b37399960ff4397344d27a0e0ebf2f4f47acb51448b4c9b49d586be3ae5b0d8`
- candidate native·PC·WASM의 source/dependency/compile flags: `final-native-manifest.json`, `final-pc-manifest.json`, `final-wasm-manifest.json`
- 서비스 재시작·WASM 교체 없음. HTTPS7002의 loader는 기존 `f480f9f9ada9c108b293ae973f4b66e6468085db0461b101072dbc1c33ad1cba`
- 서버는 workspace JS를 매 요청 읽으므로 변경한 App/Camera/Worker JS는 현재 GET에 반영됨. 전체 배포가 불변이라는 주장 제외; `final-served-hashes.json`에 실제 GET hash 보존
- 최종 서비스는 구 WASM + 현재 JS이며, 새 수학 엔진의 검증 결과는 분리 candidate에 해당
- 기존 데이터·기록·TLS·서비스·선행 dirty 변경 보존, 새 runtime dependency·commit 없음
- 재현 명령: [개발 runbook](../../scripts/dev/README.md)

## 변경 파일·증거 색인

| 변경 묶음 | 파일 |
|---|---|
| 모바일 pixels·보정·세션 | `web/js/app.js`, `web/js/camera.js` |
| 실제 C++ 설정 조회 | `web/js/vio-wrapper.js`, `web/js/vio-worker.js` |
| pose·factor·prior | `include/utility/so3.h`, `src/backend/factor/{pose_local_parameterization,projection_factor,perspective_factor,marginalization_factor}.cpp`, `include/backend/factor/{integration_base,imu_factor,imu_factor_pnp}.h` |
| 최종 유효성·Ceres 등록 | `src/backend/optimizer.cpp`, `include/backend/{optimizer,estimator}.h`, `src/frontend/pnp_frontend.cpp` |
| PC 입력·출력·설정 | `src/{vio_system,tiny_vins_mono}.cpp`, `include/vio_system.h`, `src/config/config_manager.cpp`, `src/utility/test_result_logger.cpp`, `tests/audit_dataset_replay.cpp` |
| 독립 생성·평가·동등성 | `scripts/dev/{generate-synthetic-vio,evaluate-synthetic-vio,pc-entry-check}.py`, `scripts/dev/synthetic-browser.mjs` |
| 회귀 연결·문서 | `CMakeLists.txt`, `CMakePresets.json`, `scripts/dev/{check.sh,README.md}`, 관련 native/browser/Python tests, 이 보고서·research·원본 인계 문서의 후속 링크 |

- 이번 변경 전체 목록: `cleanup/scope-final.json`; 시작 상태와 비교한 `final-source-manifest.json` hash 기준
- 전체 검증 색인: `final-verification.json`; 최종 리뷰 결과·서비스 GET hash·고정 dataset/합성/민감도 원본 연결
- 시작 증거: `baseline/status.txt`, `baseline/changes.patch`, `baseline/source-manifest.json`, immutable baseline native/WASM
- 실패/기각 증거도 유지: 초기 자연 rAF association 실패, 초기 Ceres 등록 순서 abort, source timestamp 1ULP 불일치, manifest schema 거부, 전화 prefix·무특징 조건 실패
