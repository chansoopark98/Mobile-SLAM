# 추정 속도·정확도·추적 소실 시험 결과

- 날짜: **2026-10-13**, Asia/Seoul. Ralph **iteration5**, 원래 목표 **NOT_ACCEPTED / 추적 놓침 미해결**
- 후속 확인: 중단된 작업의 native209·sanitizer157·WASM·동일 환경 default0 exact 근거를 복원하고 재검증했다. 아래 `PENDING`은 당시 상태로 보존하며 [Flip6 종합 결과](../research/2026-10-13-flip6-analysis-and-application.md)가 이후 실행·분석·Git 상태를 정리한다. 역사적 mismatch/GT 실패 receipt와 임계값은 변경하지 않았다.
- D0·D2·D3는 독립 GT gate 실패로 채택하지 않음. 실패 정책1/2를 명시적 opt-in으로 격리하고 default0 호환성을 확인하는 단계
- **최종 containment 검증 PENDING**: native·sanitizer·WASM·default0 parity·독립 review/cleanup 후 회귀 결과 대기. 아래 역사적 통과 수를 최종 source 통과로 사용하지 않음
- 실기기 연결·신규 phone GT 없음. commit·merge·push·배포 없음. 기존 dirty 작업·dataset·TLS·서비스·실패 source/binary 보존
- 종합 근거: [evidence-summary.json](../../build/tracking-performance-20261013/evidence-summary.json), [격리 계약](2026-10-13-experimental-policy-plan.md)

## 비교 기준·수락 조건

- 기준 source: `build/tracking-performance-20261013/baseline-source/`의 **시작 dirty A2 동결본**. Git HEAD 전체 diff를 Oct13 신규 변경으로 집계하지 않음
- intake1287파일·source/dependency/compile/binary hash 보존: [source fingerprint](../../build/tracking-performance-20261013/intake/source-before.sha256.json)
- 원본 dataset `/mnt/backup/SLAM` 읽기 전용, 실행 결과는 `build/` 아래. frozen pixels·CSV·IMU bracket·timestamp·GT·config 검증 후 실행
- churn 분모400frame, warmup `[100,400)` **300frame** 고정. 목표: fresh≥280/300, current-support failure<35, Lost 구간<7, reset0
- 정확도: 독립 GT·일대일 timestamp association·body-frame·단일 SE(3) scale1. ATE≤0.25m, rotation APE≤5°, 1s RPE≤0.10m/3°. Sim(3)는 scale 진단만 사용
- 입력 pose `T_W_C`를 frozen evaluator에서 `R_WB=R_WC R_BCᵀ`, `p_WB=p_WC−R_WB t_BC`로 body 변환. [evaluator:132–155](../../build/research-20261012/replay-diagnosis/frozen-browser/scripts/dev/evaluate-synthetic-vio.py)의 `--estimate-frame camera`는 입력 frame 지정
- full TUM은 matched A0·시작 dirty source 대비 위치/회전 RMSE≤1.05, coverage 손실≤1%p. 낮은 cost·실행 exit0·unit 통과를 GT 통과로 승격하지 않음

## 반영 범위·안전 격리

| 정책 | 동작 | 활성화 경로 |
|---|---|---|
|0|시작 A2 legacy current acceptance|Engine/Optimizer/native argv/App/Worker 기본|
|1|D2 current observation raw rejection·old-depth restore/guard|명시적 실험 API/native optional arg|
|2|D3 good-history candidate depth 인증·D2 fallback|명시적 실험 API/native optional arg|

- 구현 계약: Engine selected 값, Optimizer effective 값을 각 instance가 소유. Estimator forwarding. **Config/g_config policy 필드0**; 일반 calibration/config가 실험을 활성화하지 않음
- API: `setExperimentalObservationPolicy(int)`·`getExperimentalObservationPolicy()`. C++ effective getter는 backend 부재 시−1. WASM은 setter/getter 두 binding, effective는 diagnostic capture가 켜진 실제 backend params로 확인
- configure/configureFromConfig는0, reset은 선택 보존. 실제 변경은 해당 Engine의 fresh epoch·빈 history에서 시작하고 profile/calibration 보존; 같은 값은 epoch 불변. invalid0..2 범위 밖 입력은 mutation 전 거부
- native replay 마지막 optional arg `EXPERIMENTAL_OBSERVATION_POLICY=0|1|2`; 생략0. selected/effective raw·summary readback. App/Worker/UI/URL에 활성화 경로 추가0
- D2/D3 소비 guard: immutable observation flag, 실제 frame slot 유지, residual/prior/triangulation/correspondence에서 bad row 제외. valid historical landmark 유지, bad anchor·nonfinite·cheirality·weak geometry는 보수적으로 실패
- D1 유지: IMU residual/Jacobian 공통 보정·회전 재사용, DLT projection·관측 수별 SVD workspace 재사용, `allFinite()` guard. 이전/미초기화 SVD V 사용 방지
- provenance 유지: 실제 CMAKE_HOME_DIRECTORY·compile variants·imported Ceres archive·source root 기록. 사용하지 않는 ROOT 전역/mock 제거; 새 제품 dependency0
- 수정 파일 묶음: `include/backend/factor/{imu_factor,integration_base}.h`, `include/frontend/feature_manager.h`, `src/frontend/feature_manager.cpp`; `include/backend/{optimizer,estimator}.h`, `src/backend/{optimizer,estimator}.cpp`, `include/vio_engine.h`, `src/vio_engine.cpp`, `wasm/vio_bindings.cpp`
- 도구·회귀 묶음: `CMakeLists.txt`, `CMakePresets.json`, `scripts/dev/{build-provenance.py,wasm-smoke.mjs,README.md}`, `tests/{audit_dataset_replay.cpp,test_engine_contracts.cpp,test_projection_trial_contracts.cpp,test_imu_reuse.cpp,test_geometry_reuse.cpp,test_build_provenance.py}`. 기존 수정과의 세부 구분은 intake 기준
- 위 API/lifecycle **최종 검증은 PENDING**. default0 parity 통과도 A2의 기존 tracking/GT 실패를 해결했다는 뜻이 아님

## D0 whole-landmark 제거: 기각

- profile:480×640, feature120, LK21/3·20iterations, F1px, rotation rescue ON, RNG0/OpenCV1thread, solver0.06s/10iterations, PnP OFF. zero-tolerances OFF
- final current raw reprojection1.5px 초과 시 landmark 전체 제거. 오염 current support를 차단했으나 historical landmark까지 잃어 held-out·full TUM 악화

| churn 지표 | 시작 A2 | D0 | strict 판정 |
|---|---:|---:|---|
|전체 fresh|274/400|306/400|부분 개선|
|warmup fresh|215/300|247/300|≥280 실패|
|current/final failure|84/1|53/0|current<35 실패|
|Lost 구간/reset|5/0|5/0|해당 지표 유지|
|body ATE RMSE(m)|0.317694|0.136576|≤0.25 통과|
|body rotation APE(°)|13.265090|1.852646|≤5 통과|
|1s translation RPE(m)|0.135548|0.115352|≤0.10 실패|

- [baseline churn](../../build/tracking-performance-20261013/evidence/baseline-churn/evaluation.json), [D0 churn](../../build/tracking-performance-20261013/evidence/current-gate-churn/evaluation.json), [D0 full suite](../../build/tracking-performance-20261013/evidence/final-suite.json): synthetic6개 중5개 GT_FAIL, TUM room1/room4 ON/OFF4개 matched A0 위치·회전cap 모두 실패
- fast101 ATE0.043353→**1.874498m**, rotation1.547380→**49.973399°**. 무가림12617 ATE0.095282→0.440855m, rotation2.412985→9.120875°. fast8809 rotation5.136010°>5°
- D0 source/binary는 `rejected-d0-source/`, `evidence/final-*`에 보존. default에 채택하지 않음

## D2·D3 sentinel: 관측 단위 거부도 기각

- D2: 잘못된 current row만 제외하고 기존 rho를 복원. 남은 good history·cheirality guard 유지
- D3: good history≥2·condition≤10000·raw≤1.5px·forward geometry 조건을 만족하면 Ceres candidate rho 유지, 실패 시 D2 fallback. 같은 window pose의 영향은 독립 GT 인증과 다름

| 후보/입력 | warmup/300 | body ATE(m) | rotation(°) | 1s RPE(m) | 원 evaluator |
|---|---:|---:|---:|---:|---|
|D2 fast101|288|0.040688|1.150955|0.035890|GT_PASS|
|D2 fast8809|279|0.165462|3.168746|0.071869|GT_PASS|
|D2 무가림12617|270|0.164009|3.092978|**0.100883264**|GT_FAIL|
|D3 fast101|292|0.055194|2.662747|0.037404|GT_PASS|
|D3 fast8809|270|**0.280673846**|3.636140|**0.131030175**|GT_FAIL|

- [D2 sentinel summary](../../build/tracking-performance-20261013/evidence/d2-sentinel-summary.json), [D3 sentinel summary](../../build/tracking-performance-20261013/evidence/d3-sentinel-summary.json). body metrics 원문은 각 adjacent `evaluation.json`
- GT_PASS는 원 evaluator의 해당 sentinel 판정. 이번 strict280/300·full TUM·전체 수락과 구분. D2 unoccluded 실패·D3 fast8809 실패 시 계획된 sentinel 중단; 이후 full TUM/churn 성공 주장 없음
- `d2-failed-sentinel-source/`, `rejected-d3-source/`에 source·binary·hash 보존. 정책1/2는 실험 opt-in만 허용
- regression은 expected7/actual8 current support red, flagged row의 prior/geometry 소비, old-rho cheirality, weak-depth fallback을 보존. 테스트 삭제·분모·threshold 완화0

## D1 속도: kernel 개선, 전체 G5 실패

- [직렬 ABBA](../../build/tracking-performance-20261013/performance/abba-flagsfixed-v1/summary.md):8회·3,200frame,4pair·1,600행 pose16원소/status/reason/epoch/timestamp/IMU/solver exact 차이0
- 같은 frozen A2 core·OLD optimizer body에 D1만 연결. solver10s/10iterations, feature120/LK21/3·20, rotation ON/PnP OFF, cvThreads1/seed0, detail/hash diagnostics OFF. D0/D2/D3 수치 정책 제외

| engine p95(ms), 각 arm2회 median | baseline | D1 | median B/A | 판정 |
|---|---:|---:|---:|---|
|clean101|20.761225|20.430254|0.984058|약1.59% 감소, 3% 목표 실패|
|fast8809|29.526197|29.542683|1.000558|약0.056% 증가, 목표 실패|

- primary는 **각 pair≤0.97 및 run median≤0.97**. engine/frontend/backend p95≤1.05·p99≤1.10 safety는 통과; 전체 **G5 FAIL**
- [microbench-v3](../../build/tracking-performance-20261013/imu-reuse/microbench-v3/result.json): DLT95.708→82.305ms/300회(**14.00%**, 통과), IMU264.673→257.752ms/100000회(**2.62%**,10% 목표 실패)
- DLT3182 finite feature exact byte parity. malformed178 중17 nonfinite feature만 명시 거부,161 unchanged. IMU1092 evaluation exact. [geometry v3](../../build/tracking-performance-20261013/imu-reuse/geometry-parity-v3.json), [IMU parity](../../build/tracking-performance-20261013/imu-reuse/parity.json)
- clean backend18.998→18.726ms/Ceres8.600→8.400/marginalization12.259→12.036; fast backend25.013→24.957/Ceres12.611→12.537/marginalization16.889→16.872. stage percentile 합산0
- 최초 compile drift23/39를 timing 전에 차단. proxy 수정 후 actual39records multiset·같은 Ceres archive·adapter hash 확인. [보존 실패](../../build/tracking-performance-20261013/performance/flags-drift-preserved.json)
- desktop 두 synthetic fixed-work 결과. phone latency·thermal·WASM 속도·capture/copy/queue/render 전체 개선은 미검증

## 추가로 시험하고 기각한 방향

- H4: full free-state·고정 cap·독립 oracle120solve. LM near-translation/prior0·pure-camera-rotation/prior0 relative rotation/Bg 실패. [gate CLOSED](../../build/tracking-performance-20261013/h4/final-evidence.json); solver usable·작은 cost로 수락하지 않음
- ordinary LM churn: warmup238/300·ATE0.215666m·RPE0.075410m. coverage 실패. legacy-age admission:275/300·ATE0.451364m·RPE0.259223m. 성숙 gate 완화 기각
- gyro patch warp:312개2D 실험, frame127 정확 대응42개 회복했으나 frame204 실제 가려진 ID2774를 reverse/F 통과 후 오채택. [재현](2026-10-13-frontend-recovery.md). 제품 tracker 반영0
- marginalization Qᵀ 적용:46fixture 중4개 strict byte parity 실패, 최대차1.6263e−19. [기각·원본 복구](../../build/tracking-performance-20261013/perf-marginal/rejection.json). timing/adoption0

## 상용 엔진 조사·프로젝트 적용

- [ARCore·ARKit·8th Wall 비교/공식 URL/라이선스](../research/2026-10-13-commercial-tracking.md): 원문24파일 획득·MIT source5개 구조 분석(24파일에 포함). proprietary SLAM binary 다운로드·역컴파일0, 비공개 복구 알고리즘 재현 주장0
- ARCore image pose/표시 orientation·capture clock provenance·CPU/GPU/crop K 구분, ARKit limited/relocalization lifecycle, 8th Wall local descriptor→기하 검증 구조를 설계 조건으로 반영
- 적용: current 관측과 historical landmark lifecycle 분리·consumer guard·selected/effective 상태 readback·실패 모델 opt-in·source/input/build provenance. vendor threshold·floor prior·SDK projection matrix 복사0
- 후속 model 조건: 실제 pixels·IMU 시간 정렬·실측 calibration·independent field GT, ID lineage·projected local search·깊이 관측가능성·bounded recovery. 현재 guard만으로 회복 모델이 입증되지는 않음

## 검증 상태·남은 한계

- D3 역사적 source: native16/16 suites, sanitizer8/8 suites 통과. [native](../../build/tracking-performance-20261013/d3-tests.log), [ASan/UBSan](../../build/tracking-performance-20261013/sanitizer/d3-tests.log). 위 sentinel GT 실패와 별개
- sanitizer Ceres157members/157compile records instrumented 확인. system OpenCV/YAML은 prebuilt로 미계측. source·build 경로별 범위 유지
- **최종 containment PENDING**: native 전체/ASan·UBSan/default0 matched replay/API/native argv/WASM smoke·실제 Worker readback/browser·Python/clangd/독립 review·cleanup 후 회귀. 중간 결과를 final로 승격하지 않음
- active `web/vio_engine.js`·manifest 교체0. isolated D0/D3 artifact는 보존한 미승격 실험 빌드. 새 source와 active loader의 동일성 주장0
- 보존 phone minute1 prefix는 OBSERVATIONAL, minute2 initialized pose0으로 UNKNOWN. 최신30초 numeric에는 전체 영상/GT 없음; ADB 연결 없음. phone 실시간 정확도·추적 놓침 해결 판정 불가
- 남은 위험: 시작 A2의 GT/coverage 실패, 정책1/2 회귀, 일반 recovery model 부재, PnP ON 전체 회귀 미검증, 기존 global calibration 동시 격리 미검증. 정책 instance 격리는 calibration thread safety의 증거가 아님

## 재현

```bash
ctest --test-dir build/tracking-performance-20261013/candidate-native --output-on-failure -j2
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 ctest --test-dir build/tracking-performance-20261013/sanitizer/native --output-on-failure -j1 -R '^(IntegrationBaseTest|EngineContractsTest|NumericalContractsTest|PipelineContractsTest|ProjectionTrialContractsTest|MathGeometryContractsTest|IMUReuseTest|GeometryReuseTest)$'
node --test tests/browser/*.test.mjs
python3 -m unittest discover -s tests -p 'test_build_provenance.py' -v
python3 build/tracking-performance-20261013/evidence/run-case.py --help
node scripts/dev/wasm-smoke.mjs build/tracking-performance-20261013/wasm-final/vio_engine.js
git diff --check
```

- 실행 argv·input/config/evaluator/source/binary SHA는 각 `execution.json`·stage manifest에 저장. 재현 명령의 실행 가능성과 final 수락 결과를 구분
