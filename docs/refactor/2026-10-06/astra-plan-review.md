# Mobile-SLAM 실행 계획 검토 — Astra 1회

- 검토일: 2026-10-06
- 판정: **ITERATE — 아래 실행 범위·경계·통과 조건 보완 후 즉시 구현 진행**
- 검토 역할: 사용자 지정 GPT-6 Astra / ultra의 단일 계획 검토; 제품 코드·서비스·데이터 변경 없음
- 후속 구현·개발·통합 검증: 사용자 지정 GPT-6.1 Sol / ultra
- 기준: HEAD `b2826c037a50036d286329b1a56daa1a75ba57a3` + 기존 tracked/untracked 변경; Git HEAD만으로 현재 소스 식별 불가
- 작성 범위: 이 문서만. 이전 audit의 실행 결과 재사용과 이번 코드 조회를 구분; 이번 검토에서 build·실기기·성능 실험 미실행
- 읽은 자료: `AGENTS.md`, `docs/refactoring-plan.md`, `docs/audit/2026-10-06/{README,estimator,browser,native,mobile-audit,datasets,architecture}.md`, `.omx/plans/{prd,test-spec}-mobile-slam-production.md`

## 1. 실행 전 보완 사항

| 우선순위 | 기존 계획의 공백 | 필요한 보완 |
|---|---|---|
| P0 | 현재 승인 범위가 여전히 진단·도구·계획 | 최신 사용자 승인인 실제 리팩토링·7002 HTTPS 실행·가상/browser 검증으로 실행 범위 갱신. 기존 audit 완료 체크리스트를 구현 완료로 재사용 금지 |
| P0 | 실제 phone 기록이 P1 전체의 선행조건처럼 배치 | 재현된 결함 수정·독립 synthetic 회귀·dataset replay·HTTPS 시작은 즉시 진행. 실제 mobile 원인 확정·현장 개선 판정만 phone/GT 증거에 의존 |
| P0 | P2 tolerance·input manifest 미정 | development room1에서 native/WASM 동일 입력 pilot 후 비교 기준 고정. 이후 room4 실행. 기준 없는 parity를 합격으로 보고하지 않기 |
| P0 | server 작업·공개 경계가 계획에 없음 | 실제 인증서 chain·7002 listener·정적 파일 허용 경로·로그 보존/상한·공개 artifact hash 검증을 별도 구현 lane으로 추가 |
| P1 | P1~P6 전체를 한 번의 완료로 묶음 | 이번 완료: 결함 수정, 공통 실행 경로, fresh 배포 artifact, 실제 HTTPS 서비스, native/browser 실행 근거. G3/G4 현장 품질·G6 full SLAM·상용 출시 판정은 별도 미검증 항목 |
| P1 | 장기 연구용 A/B·field GT·3분할이 모든 수정의 관문 | 엔진 교체·튜닝·production 판정에는 유지. 명확한 array/reset/NaN/evaluator 결함 수정에는 작은 회귀와 영향 범위 replay 우선 |

- 위 보완은 기존 사용자 승인 안에서 처리 가능; 재승인 대기 불필요
- 기존 방향 중 유지: 평가 신뢰성 우선, A의 작은 수정부터 진행, native/browser 결과 구분, NAS 읽기 전용, camera/body 의미 보존, source→artifact provenance
- “실제 모바일 pose 점프 해결”, “production-ready”, “전체 SLAM 완료”는 이번 가상 검증으로 성립하지 않는 주장

## 2. 권장 구현 순서

| 순서 | 실제 코드 작업 | 종료 근거 |
|---|---|---|
| 0 | 최신 dirty snapshot·기존 artifact·사용자 교체 TLS 파일 보존; 작업별 변경 파일/회귀 명시; 결과는 새 `build/refactor-*` 경로 | source/diff/dependency/flags/input/artifact manifest, 기존 서비스 PID·7002 점유 확인 |
| 1A | C++ evaluator의 SE(3)·rotation RPE·matched timestamp·빈 결과 실패 수정, parity fixture의 enum/layout/alias·headless cwd 수정 | 90° rotation, doubled scale, unmatched prefix, duplicate match, 0 pose/0 pair 독립 fixture 통과 |
| 1B | 7002 server의 TLS chain·정적 경로·로그 수명 수정 | 실제 7002 TLS 접속, 안전한 공개 경로 fixture, 기존 로그 보존; 최종 candidate 승격 전까지 baseline/candidate 구분 |
| 2A | configure 입력 복사/검증, IMU 경계·future carry, finite/positive depth, 유효 covariance factor, reset prior·PnP 소유권 수정 | IMU 적분 oracle·alias·0/1 sample·repeated reset 회귀, ASan/UBSan |
| 2B | wrapper/Worker의 frame/session/time 계약, ring overflow·subarray·waiter·empty map·stale reply 수정; live sensor 시각/축/profile provenance | 기존 TODO 5건이 실제 필수 assertion으로 전환, live adapter·시간축·pause/rotate 테스트 |
| 3 | solver 결과·fresh pose·loss/recovery를 연결; 외부/내부 reset 일관화; propagation의 비단위 quaternion 수정 | unusable solve/blank frame/gap/old result가 tracking 성공으로 노출되지 않는 end-to-end 회귀 |
| 4 | 동일 source 목록·`VIOEngine` 입력 계약 재사용; native `VIOSystem`의 추정 orchestration을 공통 engine에 위임 | GUI 없는 native 실행, camera pose consumer 보존, native/WASM 동일 입력 비교 통과 후 중복 삭제 |
| 5 | fresh WASM 별도 build, 실제 compile SIMD/undefined-symbol 설정 검증, browser가 candidate hash를 로드하도록 연결 | native·실제 Worker replay, initialized pose·coverage·SE(3) 수치, hash 일치 |
| 6 | 검증된 candidate를 7002에 제공; HTTPS 앱·replay·모바일 관측 페이지 실제 실행 | hostname/SNI/chain·HTTP/Worker/network/console·artifact hash·서비스 생존 확인 및 재현 명령 |

- 1A·1B 및 2A·2B는 소유 파일을 나누어 병렬 가능; 최종 성능 측정은 다른 build/replay와 겹치지 않게 순차 실행
- `RefineGravity`의 반복 normal equation 누적은 별도 원인 후보: known gravity/scale fixture 후 작은 수정과 room1 ablation. upstream 동일성만으로 정답 판정 금지
- PnP는 기본 off 유지. 소유권·유효성 결함은 수정하고 opt-in 경로를 시험하되, 한 600-frame 결과만으로 기본 on 전환 금지
- 알려진 내부 결함을 기록만 하거나 새 설정 파일만 만드는 것으로 단계 종료 처리 금지

## 3. 파일 소유와 공유 인터페이스

| Lane | 단독 수정 소유 | 인접 lane에 제공할 계약 |
|---|---|---|
| Core | `include/src/backend`, `include/src/vio_engine.*`, PnP·feature depth·초기화 수학 관련 파일, `wasm/vio_bindings.cpp`, 해당 C++ 회귀 | calibration 입력 layout, IMU boundary 소유, frame 결과·reset epoch·solver 진단 |
| Browser | `web/js/{imu,camera,app,vio-worker,vio-wrapper,renderer,test-tumvi-app}.js`, 해당 browser 회귀 | source timestamp/arrival 구분, frame ID·epoch, queue/drop, live profile, paced replay |
| Evaluation | `include/src/utility/trajectory_evaluator.*`, `scripts/evaluation/`, replay/evaluator tests·결과 scorer | body pose 변환·unique association·실제 SE(3) RPE·coverage·실패 exit |
| Build/Native adapter | root/WASM CMake, `src/vio_system.cpp`, `src/utility/measurement_processor.cpp`와 대응 header, build/replay runner | 공통 source 목록·headless 경로·exact input/artifact manifest; Core API 변경은 요청 후 반영 |
| HTTPS | `web/server.js`, server lifecycle script·server 회귀·runbook의 서비스 부분 | 7002·인증서 chain·허용 정적 root·로그 상한·PID/종료 계약 |
| Leader | 실행 계획·PRD/test spec 갱신, 기준 동결, 통합·최종 replay·배포 판정 | 같은 파일/빌드 디렉터리 동시 수정 방지, 기존 사용자 작업 보존 |

- Core의 `include/src/...` 표기는 각각 `include/...`, `src/...` 소유를 의미
- `src/vio_engine.cpp`·binding·CMake를 여러 lane이 동시에 편집하지 않기; 한 lane에서 API 제공, 다른 lane은 소비
- 모든 child에 기존 작업 공존·되돌리기 금지·파일 범위·필수 회귀를 명시
- 최소 공유 결과 의미: `frameId`, session/generation, image timestamp, 실제 pose timestamp, pose frame, fresh/valid, state/reason. 현재 16-element pose buffer와 공개 API는 compatibility 경계로 유지 가능
- result는 성공·무효 입력·0 IMU·exception 등 모든 반환 경로에서 동일 요청 식별자를 보존
- 내부 estimator reset도 session 변화로 전달; wrapper reset만 세면 새 좌표 원점의 jump를 실제 이동으로 오인
- 같은 요청 epoch의 새 result만 busy 해제·pose/map 갱신. dispose/reset/error 시 모든 pending waiter 종료

## 4. 수학·시간축에서 반드시 지킬 조건

### 좌표·calibration

- 기존 의미: `T_W_C = T_W_B · T_B_C`, `R_W_C = R_W_B R_B_C`, `p_W_C = p_W_B + R_W_B t_B_C`
- `R_IC`/`t_IC` 명칭 변경보다 실제 방향 보존 우선. `configure()`는 입력을 임시 소유 값으로 복사한 뒤 global config에 대입; row-major 계약과 Eigen column-major caller 오류는 별개로 수정
- nonidentity·noncommuting rotation 및 nonzero lever-arm fixture 필수. 90° body 회전에서 body 원점은 고정되고 camera 위치는 회전한 lever-arm만큼 변화
- 배열 finite, `RᵀR≈I`, `det(R)≈+1`, 양의 focal length·유효 크기 검증. 잘못된 calibration을 조용히 identity·SO(3) projection으로 고쳐 성공 처리 금지
- K는 실제 pixel crop·resize·rotation을 따라 변환. resize rounding과 pixel-center convention까지 fixture로 고정
- camera ray/inverse depth는 optical Z 기준 유지. z≈0·negative depth·NaN은 factor 진입 전에 거부; 분모 clamp로 잘못된 geometry를 정상 관측으로 만들지 않기
- mobile FOV69°·고정 extrinsic은 기존 fallback의 provenance로 표시. 측정 profile로 재명명하거나 metric 정확도 보증에 사용 금지

### IMU 경계와 clock

- image 시각 `t_k` 사이 적분 범위는 `[t_(k-1), t_k]`; elapsed time만 맞추고 측정값 경계를 틀리는 수정도 실패
- 첫 seed sample, 이전 image에서 보간한 endpoint, 원래 future sample의 값·시간을 구분. future carry 소유는 engine 또는 adapter 한 곳에 고정
- 경계 sample의 보간 재사용은 허용하되 동일 시간 구간의 중복 적분은 금지. 여러 future sample·한 frame 동안 신규 sample 0개·비동기 camera/IMU phase 포함
- 현재 Worker의 정상 test `Worker preserves future IMU beyond one interpolation boundary`는 첫 future sample 소비를 기대. 기존 assertion을 그대로 보호하면 결함을 고정할 수 있으므로 독립 적분 fixture로 대체/보강
- `VIOEngine::processIMUData()`는 현재 image 시각으로 cursor를 옮기면서 원래 future 측정값을 `prev_*`로 저장. 다음 구간의 시작 측정이 image endpoint와 불일치하는 경로까지 검증
- out-of-order/duplicate 거부 시 integration cursor를 과거로 이동시키지 않기. ring overflow·stale cutoff·buffer truncation은 lost interval과 reason/drop count를 남기고 정상 연결된 preintegration으로 위장하지 않기
- Generic accel/gyro는 별도 timestamp의 두 스트림. 단순 callback dedup 제거만으로 동기화 완료 처리 금지; 정해진 시각으로 resample하거나 각 stream age/보간 정책을 명시
- sensor API timestamp·event timestamp·callback arrival·video presentation timestamp를 구분. clock origin mapping·단위 확인 없는 `event.timeStamp / 1000` 일괄 치환 금지
- physical exposure가 없는 camera timestamp는 presentation/arrival proxy. 알려지지 않은 offset을 GT trajectory 정렬로 online 보정 금지

### Covariance·회전·solver

- 0/1 integration covariance는 singular할 수 있음. `LLT.info()==Success` 단독 판정 금지; finite·양의 분해 pivot·적분 시간 coverage·사용 가능 factor 확인
- SPD covariance는 `Σ=L Lᵀ`, whitening은 `L⁻¹ r`와 같은 metric 보존 방식으로 계산 가능. explicit inverse 후 NaN만 제거하거나 임의 diagonal jitter로 정보를 만들어 넣는 변경 금지
- 유효 IMU factor 누락을 무조건 skip하고 “정상 metric tracking”을 계속 출력하지 않기. 필요한 구간·관측이 부족하면 deferred/degraded/lost 상태와 이유를 노출
- sample count ≥2만으로 covariance·observability 합격 판정 금지; count는 필요조건 후보, 실제 수치 유효성과 시간 구간은 별도
- propagation에서 `deltaQ(...).toRotationMatrix()`의 비단위 quaternion 사용 수정. 단순 finite 검사는 SO(3) 위반을 잡지 못함
- 공용 `Utility::deltaQ`는 bias correction·analytic Jacobian에도 사용. propagation의 normalization/exp-map 개선을 전역 helper 변경과 묶지 않기; 바꾼 적분 모델의 독립 angle/acceleration oracle 필요
- Ceres `NO_CONVERGENCE`는 시간/iteration budget 종료로 사용 가능한 해일 수 있음. 로컬 pinned Ceres 2.2.0 `internal/ceres/solver_utils.h`도 이를 `IsSolutionUsable()`에 포함
- `IsSolutionUsable()` + finite parameter/cost + 필요한 residual/관측 + 올바른 gauge/anchor를 확인. usability는 정확도·기하 비퇴화 증명과 별개
- unusable solve에서는 결과 적용·marginalization 갱신 금지. 이전 추정값 보관은 가능하지만 새 image의 tracking pose로 timestamp만 바꾸어 반환하지 않기
- PnP는 `matched.empty()`와 `hasPose()`만으로 새 pose 판정 불가. 현재 frame solve 성공·backend anchor·사전 최소 대응/분포를 확인

### Reset·manifold·초기화

- reset은 sliding window, marginalization prior/parameter blocks, image/feature state, preintegration, PnP·solved-feature cache, IMU cursor, output timestamp·session까지 하나의 lifecycle 계약
- “prior를 적용하지 않는 flag”만 추가하고 오래된 pointer를 남기는 우회보다 소유 객체 reset/해제 우선
- 기존 `FailureDetector` 임계값을 그대로 연결하면 실제 동작/장거리 이동도 reset 가능. feature·innovation·bias·relative motion 근거와 회귀로 연결; 절대 `|position|>100m`를 보편 divergence 기준으로 확대 금지
- `PoseLocalParameterization`과 factor의 7-column tangent Jacobian은 함께 묶인 legacy 관례. 정상 quaternion manifold 하나만 교체하면 effective derivative가 달라짐
- 우선 검사: `J_factor · J_plus`와 `d r(Plus(x,δ))/dδ`의 local finite difference. 정식 manifold 전환 시 projection·IMU·PnP·marginalization·Minus까지 함께 변환/검증
- manifold 전체 전환은 이번 결함 수정의 무조건 선행조건에서 제외. 해결되지 않은 formal manifold 계약은 명시하고 수학 전체 정합성 완료 주장 제한
- gravity refinement는 iteration별 tangent basis에 맞는 A/b 재구성·conditioning·positive scale을 독립 known-truth로 확인. 실제 mobile 초기화 가능성/metric scale은 별도 관측성 문제

## 5. 이번 deliverable의 필수 통과 조건

| Gate | 실행·기대 결과 | 실패 처리 |
|---|---|---|
| R0 보존·빌드 | 새 native/WASM build, source/diff/flags/dependency/artifact hash; 기존 작업·NAS 원본 보존 | stale binary·artifact hash 불명은 실패 |
| R1 평가 | 90° relative rotation → 90°, doubled scale → SE(3) nonzero/Sim(3) scale0.5, unmatched prefix에도 동일 matched RPE, one-to-one·empty failure | no-pair0·TODO/skip·진단 exit0으로 대체 금지 |
| R2 sensor/calibration | 60/100/200Hz IMU × 10/20/30/60Hz camera, phase·future carry·duplicate·gap; analytic constant/ramp 측정·90° extrinsic·K resize/rotate | 적분 cursor/coverage/축/레이아웃 불일치 실패 |
| R3 lifecycle/numerics | 0/1 sample·bad covariance·NaN depth·blank/blur·solver unusable·reset/reinit·old result·PnP 반복 on/off; sanitizer 실제 실행 | NaN/crash/leak·stale pose를 fresh로 반환·prior epoch 혼합 실패 |
| R4 transport | 기존 TODO 5개 포함 strict tests; all waiters settle, 정확한 subarray bytes, bounded ordered ring, map0 cache 제거, frame/session/time 보존 | 숨은 drop·infinite waiter·stale reply 반영 실패 |
| R5 same-input replay | room1 2,821 frame 전체 native/실제 Worker·WASM, room4 2,228 frame 전체; calibration·IMU 정책·decoded image·counts·run 설정 일치 | 한 경로만 성공·no initialization·0 pose/0 match·누락 frame 설명 없음 실패 |
| R6 HTTPS | 실제 7002에서 index/replay/audit·JS/WASM 정상 로딩, 인증서 검증·후술 공개 경계·live process/artifact 확인 | config 파일·curl `-k`·임시 다른 포트 실행으로 대체 금지 |

- R2 synthetic의 시간 합은 double 정밀도와 fixture 시간 규모에 맞춘 tolerance 사전 명시; 예: 상대 시각 사용 시 `abs(sum_dt-Δt) ≤ 1e-9 s`
- R3 정규화된 회전은 SO(3) 오차와 determinant를 별도 검사; quaternion sign ± 동등성 유지
- R5는 초기화·fresh pose·GT-associated·tracking coverage와 state/reason·reset 수를 별도 보고. invalid를 숨기기 위한 기존 broad parity threshold 유지 금지
- room1은 이미 열람·튜닝된 development 입력. room4는 selection/validation 입력으로 표시; 동일 TUM 장비/room 계열을 새로운 mobile domain으로 주장 금지
- room4 검증된 archive는 NAS에서 읽고 local staging만 사용. 공개 server에서 NAS 전체를 expose하지 않기
- deterministic parity: 고정 seed·native OpenCV threads=1·고정 iteration 수·충분한 solver 시간 상한. 실제 종료조건/iteration 수가 달라지면 원인 구분; 시간 budget 효과와 수학 parity 혼합 금지
- 같은 room1 development pilot 3회에서 반복 변동 확인 → translation/rotation p95/max·init frame 차이·coverage 차이 허용치를 manifest에 동결 → room4 실행. 미고정은 R5 parity `unverified`
- safety 수정 전/후 coverage 감소는 stale 출력 제거의 결과일 수 있음. 감소 구간·원인 공개 및 목표 회복 검토; 그 자체를 성능 개선으로 포장 금지
- 정확도 변경 비교는 같은 입력·scorer·body frame·alignment·eligible 분모 사용. 기존 native ATE0.555253m·1s RPE0.160278m/0.743111°는 room1 baseline이며 새 제품 허용 오차가 아님
- virtual 검증 2종 구분: calibrated dataset로 실제 estimator 초기화/추적 실행, fake camera/mock sensor로 live 권한·clock·orientation·pause 흐름 실행. 후자의 0 pose smoke는 전자의 대체 증거 제외
- 적어도 한 bounded paced replay에서 입력 주기·busy/drop·pose age·queue 확인. max-speed replay throughput을 20/30Hz live mobile latency로 재명명 금지

## 6. 7002 HTTPS 실행 gate

- 사용자 교체 파일: `assets/keys/serdic_com_{cert,chain_cert,root_cert}.crt`, `assets/keys/serdic_com.key`; 내용 출력·Git 추가·자동 교체 금지
- leaf와 key의 공개키 일치, 유효기간·SAN hostname·issuer chain 확인. 실제 인증서 체인을 읽어 leaf+필요 intermediate를 제공; root는 일반적으로 서버가 보낼 chain에 추가 불필요
- 제공 root로만 검증 성공한 결과와 클라이언트 기본 trust store의 검증 성공 구분. 외부 hostname/SNI의 실제 HTTPS 접속을 `-k` 없이 확인
- `0.0.0.0:7002` listener·소유 PID·명령·stdout/stderr 위치 기록. 현재 서버 기본2224 및 README8444 혼선을7002 runbook과 실제 실행으로 해소
- 기존 `web/server.js`는 시작 시 `logs/test-tumvi.log`를 truncate. 기존 로그 유지, append 또는 session별 새 파일, 저장량 상한 적용
- public static은 web assets와 명시한 dataset/candidate root만 허용. decoded traversal·dotfile·key/cert suffix·외부 symlink를 실제 경로 기준으로 거부; `web/.certs`와 `server.js` 등 비공개 파일도 차단
- `/log`는 개발 진단 필요 여부에 따라 기본 비활성 또는 body/entry/rate/total-size 상한. 원격 입력으로 무한 메모리·디스크 점유 불가, client payload를 terminal control code로 그대로 출력하지 않기
- GET/HEAD·지원 MIME·COOP/COEP·cache 정책 검증. HEAD에서도 private asset body·secret metadata 미노출
- secret sentinel fixture로 차단을 시험; 실제 private key response를 테스트 출력으로 수집하지 않기
- 사용자 확인된 forwarding/UFW 설정을 재설정하지 않기. 로컬 hostname/SNI 성공과 외부망7002 도달성은 별도 증거; 외부 관측점 부재 시 후자 미검증 표시
- 검증한 candidate artifact를 원자적으로 승격하고 마지막 합격본 보존; 실행 중인 다른 서비스 종료 금지. 완료 시 사용자 요청 서비스는 계속 실행하고 정확한 URL·종료 명령 제공

## 7. 이번에 줄일 설계·후속으로 남길 범위

- 유지할 단순화: 공통 source 목록, 단일 engine orchestration, API 경계에서 명시적 상태·time·epoch, 하나의 future carry 소유, 실제 사용하지 않는 loader fallback 제거
- 피할 확대: 새 runtime dependency, WebGPU/AI frontend, 즉시 upstream 교체, 모든 global config의 전면 DI 전환, generic event bus, 임의 smoothing으로 jump 가리기
- global `g_config`·static factor 정보는 현재 single-engine 제약으로 표시; 독립 다중 engine 지원이 필요해지기 전 대규모 객체화 불필요. 테스트 간 설정 오염은 별도 프로세스·명시적 설정 복원으로 방지
- 실제 사용처 확인 전 legacy generated bundle·reference·dataset 삭제 금지. JS만 삭제하고 빌드 대상/consumer를 남기는 중간 상태 금지
- full image+raw IMU recorder는 실제 phone 동일 입력 재현에 필요한 후속 기능. 현재 metadata observer를 완전 replay recorder로 명칭 변경 금지
- 전체 Ceres manifold 재작성, temporal-offset 추정, rolling-shutter factor, loop closure·persistent map·global relocalization은 이번 안전성 수정에 끼워 넣지 않기; 별도 수락 조건 필요
- GPL 차용 provenance는 확인된 배포 검토 항목으로 유지. root MIT 표기를 근거로 상용 배포 허용 결론 금지; 이번 코드 검토에서 법적 허용 여부 미판정

## 8. 최종 보고에서 가능한 주장

- 가능: “확인된 evaluator·IMU 경계·reset·Worker 결함 수정, fresh native/WASM build와 실제 7002 HTTPS 서비스 실행, 명시한 room1/room4 및 virtual browser 계약 검증 완료” — 각 문장은 실제 통과 로그/수치가 있을 때만 사용
- 필수 병기: 변경 파일·삭제/통합 경계, strict test의 pass/fail/skip/TODO 수, sanitizer 범위, 입력 count·pose/GT coverage·SE(3) translation/rotation, process/Worker/paced 지연, source/served artifact SHA-256
- 미검증 유지: 실제 Android/iOS sensor·camera timestamp/calibration·thermal, 사용자 phone jump의 지배 원인 및 현장 개선, 독립 field GT 오차, 물리 capture-to-display latency, full SLAM·상용 출시 적합성
- 운영 인계: 인증서 SAN과 일치하는7002 접속 URL, process/PID·로그·종료 명령, 현재 배포 hash, phone 관측 페이지 URL
- 완료 판단: R0~R6의 해당 구현·실행 근거 확보. field 장비 부재를 이유로 가능한 리팩토링·서버 실행을 중단하거나, 반대로 virtual 성공을 field 완료로 확대하지 않기

## 근거 위치

- 계획 범위·미동결 기준·P1~P6: `docs/refactoring-plan.md` §4~5·§11, `.omx/plans/prd-mobile-slam-production.md`
- 수치/좌표/reset 근거: `docs/audit/2026-10-06/estimator.md` E01~E10 및 수학 주의점
- Worker·live sensor·virtual 한계: `docs/audit/2026-10-06/browser.md`, `mobile-audit.md`
- 경로 분리·source/SIMD·SLAM 범위: `docs/audit/2026-10-06/architecture.md`
- 현재 결과·dataset 범위: `docs/audit/2026-10-06/{README,native,datasets}.md`
- 이번 추가 코드 조회: `src/vio_engine.cpp`, `src/backend/{estimator,optimizer}.cpp`, IMU factors·integration·pose manifold, `src/frontend/initialization/initial_alignment.cpp`, `web/js/{imu,vio-worker,vio-wrapper}.js`, `tests/browser/contracts.test.mjs`, `web/server.js`
- pinned API 확인: `build/_deps/ceres-solver-src/include/ceres/{types,solver,manifold}.h`, `internal/ceres/solver_utils.h`; 외부 최신 SDK 추정 없이 현재 build source 계약 확인
