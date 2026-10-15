# 모바일 웹 SLAM 개선 실행 계획 — 2026-10-14

- 출발점: [3종 비교·새 대조 실험](2026-10-14-slam-comparison.md). 구현 전 분석 계획이며 아래 제안값은 새 성능 달성 결과가 아님.
- 목표: 입력/추정/표시를 구분해 실제 phone의 fresh metric tracking·정확도·pose age를 함께 개선. 8th Wall의 모바일 웹 구조를 주 reference로 사용.
- 원칙: 빠른 실패·stale pose를 성공으로 세지 않기, 원본입력/실패후보 보존, timestamp/pixel/epoch 소유권 유지, 단일 변경 A/B, 수식·관측가능성 우선.
- 즉시 채택 금지: 실패한 C3/D2/D3 기본ON, gyro warp false-ID 허용, 단순 current-support threshold 완화, arbitrary depth/pose smoothing으로 Lost 숨김.
- 최초 비교 단계에서는 제품 변경을 하지 않았음. 사용자 후속 요청에 따라 추가 바이너리 분석 → 다각도 리뷰 → 통과한 범위의 구현·테스트로 진행. 아래 실행 계약과 review receipt를 기준으로 상태 갱신.

## 선택과 대안

| 방향 | 장점 | 위험·판단 |
| --- | --- | --- |
| A. 현재 엔진의 관측/입력→기하→성능을 분리 개선 | 기존 strict evaluator/회귀 유지, 원인별 검증 가능 | 단기 전체 recovery 해결 보장 없음. **우선 선택** |
| B. 8th Wall 방식 GPU frontend/descriptor 경로를 연구 prototype으로 비교 | 웹 특화 capture/feature 설계 직접 비교 | OpenCV KLT와 byte/feature 의미가 달라짐; latency·false matching·device GL 변동. A의baseline 이후 독립 실험 |
| C. 상용 SDK 또는 다른 엔진으로 교체 | 특정 기기/지원 조건에서 제품 경험 확보 가능 | native/웹·scale·calibration·API·지도 계약 차이, 동일 입력/GT 우위 미검증. 지금 교체 결정 근거 부족 |

- ADR: A를 먼저 수행하고 B를 제한된 입력·경계에서 비교. C는 A/B 수락 결과와 제품 요구를 근거로 별도 결정.
- 주요 tradeoff: frame 분산/GPU화는 main CPU를 줄여도 pose age를 늘릴 수 있음. 엄격한 geometry gate는 잘못된 metric depth를 막지만 continuity를 떨어뜨릴 수 있음. 어느 한쪽 숫자만 최적화하지 않음.

## 0. baseline과 진단 표시 고정

### 0A. 비교 artifact 고정

- 파일: `web/vio_engine.manifest.json`, `scripts/dev/build-provenance.py`, 기존 deploy 도구. 현재 source86파일·새WASM38837d4…·active140e29a…를 별도 baseline으로 유지.
- 수락: 실행 보고서에 JS/Worker/engine SHA, effective parameters, C3/policy/rescue readback, source/SDK versions 모두 존재. 설명하지 않은 source mismatch0.
- 기본값 승격 조건: build/smoke 외에 고정 synthetic/TUM/phone replay 비악화 증거. 현재fresh53/300을 알고 있으므로 최신 소스라는 이유만으로 자동 deploy 금지.

### 0B. stale UI와 실패 전파

- 파일: `web/js/app.js::_consumeResult`, `web/js/vio-wrapper.js::_workerFailure`, `web/js/renderer.js`; 회귀는 `tests/browser/` 기존 패턴 재사용.
- 방향: result dedup와 상태/age 갱신 분리. Worker failure 시 즉시 표시 stale/error로 전환. raw estimator poseFresh, metric state, wall age를 별도 표기.
- 수락: frozen clock으로 동일result가5초 후에도 age0/Tracking으로 남는 현재 red 재현을 green으로 변경. Worker error→null 후 한 UI tick 이내 stale/error. 명시Lost/epoch/reset/late-old-result 보호 유지.
- age를 이유로 metric pose 데이터를 삭제하거나 최신 timestamp로 바꿔 새 pose처럼 만들지 않음. 화면 만료 기준은 설정/명시된 계약으로 관리하며 정확도 scorer와 분리.
- follow alpha=.08은 외부 관찰 camera 동작. raw-view 선택과 시간 기반 follow를 분리 검토; frustum trajectory/estimator pose 값 불변 회귀.

## 1. 실제 입력과 replay 계약

### 1A. camera/IMU 입력 계약

- 파일: `web/js/camera.js`, `web/js/imu.js`, `web/js/app.js` camera profile, 기존 Android recorder.
- 기록: rear camera identity, resolution/crop/rotation, K/distortion, camera↔body extrinsic, exposure/EIS 여부, sensor source/arrival/dispatch, image source/presentation/arrival, request epoch/index.
- Generic Sensor 미래 clock 거부·DeviceMotion fallback·bounded pair buffer·bracket 검사 유지. browser API rate 설정을 실제 hardware sampling rate로 쓰지 않음.
- 수락: 원본 input 전부 단조/유일 시각, ordered sample 및 앞/뒤 bracket 검증, pixel SHA와 frame identity 연결100%, late/drop/overflow를 숨기지 않는 기록. 실제 물리 offset/exposure를 측정하지 못하면 `unverified` 유지.
- 같은 장면에서8th Wall과 비교하더라도 원점/scale/카메라 세션이 다르면 상태·반응 실험으로 제한. 정확도 비교는 독립GT·시간·좌표·scale 계약 성립 후 수행.

### 1B. replay 두 용도 분리

- 파일: `scripts/dev/synthetic-browser.mjs`, `scripts/dev/replay-android-input.mjs`, 기록 metadata.
- 수학적 입력 재생: frame endpoint를 포함하는 IMU를 먼저 공급. 동일 frame/IMU/profile에서 estimator 상태 비교.
- 실시간 스케줄 재생: frame 대기 중에도 원래 arrival 시각에 후속 IMU delivery 가능해야 함. `preSubmitCount→await frame→next batch`를 live schedule parity라고 명명하지 않음.
- 필수 fixture: 이번392개 frame, pre-submit bracket부족50개, 원live347fresh, 직렬실패299fresh, future-endpoint347fresh. 이 실패를 삭제하거나 분모를 축소하지 않음.
- 수락: frame-in-flight 상태의 후속 IMU가 전달되어50개 인위적 `missing_bracket` 제거; fixture의 source/frame/IMU identity 불변. wall latency 평가는 별도 clock/스케줄 provenance 검증 후만 허용.

## 2. 병목을 줄이되 pose age와 기하를 보존

- 선행: 단계1 입력 계약, 고정 source/profile, 동일 기기/입력, 직렬 ABBA 반복. 초기화 전/후·Lost/Tracking·capture/render/recorder 상태를 분리.
- 수락 제안: 개선 대상 p95≥10% 감소, total source→fresh pose p95≥5% 감소; p99≤baseline×1.05, fresh coverage 감소≤1%p, GT 기존 gate와 reset/false association 비악화. 특정 sub-kernel 개선만으로 전체 수락하지 않음.

| 후보 | 실제 수정 경계 | 필수 측정·실패 조건 |
| --- | --- | --- |
| capture admission/copy | camera rVFC와 owned pending snapshot | pixels/timestamp를 다른 순간에 묶으면 즉시FAIL. main callback/GC/readback·source→submit age 동시 측정 |
| GPU pyramid/gray prototype | 개발용 alternate frontend, 제품 기본 경로 유지 | 8th Wall PBO/deferred readback 참고. 소프트웨어GPU 결과를 physicalGPU 성능으로 확대 금지; iOS fallback과 frame age 필요 |
| frontend stage | KLT pyramid·reverse·F·detect 호출별 instrumentation | keypoint budget/ID/영상/threshold 고정, inlier·false-match·current factor 및 stage시간 기록 |
| backend stage | triangulation·factor assembly·Ceres·QR·slide/repropagation | Ceres max time를 전체deadline으로 취급 금지. 코드 재사용 최적화는 exact/수치독립 회귀 먼저 |
| IMU 비용 세분화 | propagation vs tmp/window preintegration vs IMUFactor vs slide merge | 기존 imuMs0.235ms만 보고 backendIMU 전체 비용0으로 단정 금지. 해당 부분 비중이 작으면 후순위 |
| 결과 serialization/render | map copy/structured clone/GPU update/DOM | 크기 상한 이미 있음. 실제 profile 없이 새로운 복잡한 buffer pool/공유메모리 추가하지 않음 |

- 기존 capture-worker 실패(source→submit p95+9.63%, fresh-gap+11.66%)와 D1 전체G5 실패를 회귀로 유지. “Worker/GPU이므로 빠르다”는 채택 기준이 아님.

## 3. visual-support 소실과 재연결

- 파일: `src/frontend/feature_tracker.cpp`, `feature_manager.cpp`, `src/backend/optimizer.cpp`, `estimator.cpp`, `src/vio_engine.cpp`.
- 먼저 동일 frame별 ID lineage를 기록: detected→LK→reverse→F→age/history→DLT rank/condition→cheirality/reprojection→pre-solve current→post-solve current→map/prior membership.
- 수락: Lost 직전 feature가 사라졌는지, 보이지만 depth가 불확실한지, 잘못 연결됐는지 구분할 수 있어야 함. tracker53/eligible80을 현재metric 지지로 계산하지 않음.
- descriptor/local-map 재연결 후보는 image warp·gravity-informed descriptor·guided search를 별도 통제. 8th Wall 공개 GORB/image-target 경로는 비교 설계 근거이며 그대로 복사할 private world algorithm이 아님.
- false-ID sentinel: 과거 frame204/ID2774의 실제 가려진 점 오채택0. 가림·반복무늬·순수회전·저baseline에서 근거 없는 metric depth 승격0.
- 기존 gate 유지: churn400/warmup300 기준fresh≥280/300, current-support failure<35, Lost 구간<7, reset0; ATE≤0.25m, rotation APE≤5°, 1s RPE≤0.10m/3°. TUM A0대비 translation/rotation≤1.05, coverage 감소≤1%p. 기준은 기존 작업 gate이며 신규 후보 통과 결과가 아님. churn의reset0과 raw phone prefix의baseline 비악화 기준을 혼합하지 않음.
- raw phone prefix·축보정 sensitivity control 모두 검사. corrected만 좋아지고 raw reset/jump가 나빠진 C3는 default OFF 유지. bias warning·guard가 숨긴 candidate jump까지 기록.
- 현재 factor 수≥2는 충분조건이 아님. motion observability/rank/innovation·calibration 검증과 함께 평가. 제한된 회전-only 결과를 내더라도 metric pose/지도·prior에 무근거 값 편입 금지.

## 4. 실제 모바일 웹 최종 검증

- 같은 물리 기기·렌즈·조명·동작 protocol로 자체 후보와8th Wall을 측정. 각 세션 입력 차이와 OS camera 접근 차이를 기록.
- fresh coverage, initialized time, Lost episode/reacquire, raw/display pose age, p50/p95/p99, CPU/GPU/thermal/battery, capture→render texture/pose skew를 분리.
- 외부GT가 없으면 상대 출력 안정성·실행/상태 관측까지 판정. ATE/RPE/metric 우열은 주장하지 않음.
- ARCore는 native 기준선으로 별도 record. Session.update API 시간과 웹전체engine/capture시간 직접 비교 금지.
- 장기·다기기 시험 전 단일30초 결과로 production 수락하지 않음. feature/SDK API 존재를 실제 사용·정확도 완료로 처리하지 않음.

## 실행 순서와 산출물

1. **0A/0B**: 버전 혼선과 표시 오류 해소. 작은 diff·기존 browser 회귀, estimator 불변 확인.
2. **1A/1B**: 물리 입력과 replay 신뢰성. raw/schedule/매칭된 수학적 replay 별도 artifact.
3. **2**: 실제 비용이 큰 한 단계씩 ABBA. pose identity/GT/age gate가 없으면 채택 보류.
4. **3**: 대응·기하·recovery 연구. 앞단계 계측을 사용하여 실패를 분해하고 sentinel 먼저 실행.
5. **4**: 합격 후보의 actual mobile8th Wall 비교와 독립GT 평가. 배포·field acceptance 별도 기록.

- 독립 병렬 가능: input/replay, UI contracts, readonly algorithm analysis. capture·worker·estimator 공유파일 수정은 소유권을 나누고 같은baseline으로 통합.
- 검증 명령: JS `node --test tests/browser/*.test.mjs`, Python analyzer회귀, `bash scripts/dev/check.sh syntax`; C++ 변경 시 해당 native suite·sanitizer·격리 WASM·actual Worker/GT replay를 추가. 모든변경에 무조건 전체고비용 suite를 반복하는 방식은 피함.
- 실패 조건: false correspondence 증가, depth 관측 불가능 상태의 metric 승격, prior/epoch/pixel identity 손실, accuracy/coverage 악화, total pose-age 악화. 원본/실패 로그 유지 후 후보 분기를 중단·기본값 복귀.

## 후속 실행 계약 — 상세 RE 반영·3개 관점 리뷰 통과

- 요청: 8th Wall·ARCore 바이너리 내부 추가 분석 및 계획 반영, 다각도 리뷰 통과 후 개발·테스트.
- 선행 자료: [8th Wall 심화](2026-10-14-8thwall-deep-runtime.md), [ARCore 심화](2026-10-14-arcore-deep-runtime.md). 두 심화 문서와 적용 표를 검토한 architecture/correctness·regression/evidence·scope/standards의 U/R APPROVE를 확보. 승인 당시 문서와 SHA는 `build/slam-revision-20261014/review-gate/`에 고정하고, 이후 구현/검증 결과는 별도 기록.
- 판단 대상: 전체 장기계획 중 재현된 결함과 독립 기대값이 있는0B·1B 우선 실행. 물리calibration·GPU frontend·지도재연결은 해당 입력/기하/성능 gate 없이 기본 채택하지 않음. 성능/추적 모델은 reviewer가 제시한 근거 있는 추가 변경만 편입.

### U — 화면 freshness/error 계약

- 소유 파일: `web/js/app.js`, `web/js/renderer.js`, 관련 `tests/browser/` UI 계약. Wrapper의 estimator 결과·map payload는 변경하지 않음.
- result identity dedup는 pose/map/trajectory 반영에만 사용. 동일 result에서도 age/IMU 상태 표시 갱신; **동일 result의 주기적 text 갱신만 최대10Hz**로 제한하고 새 result·error/stale 전이는 즉시 반영.
- 표시 가능한 pose: 기존 finite/state/epoch/fresh 검사와 별도로 `0 <= now-poseTimestamp*1000 <= 500ms`. **500ms는 display 만료 정책**으로, 같은 App의 기존 IMU freshness500ms와 맞춘 보수적 상한; 측정된 정확도·SDK 내부 상수·성능 목표가 아님.
- stale/new-old/null/failure에서는 현재 frustum을 숨기고 `Stale`/`Waiting`/`Worker error` 상태를 구분. historical trajectory/map은 일시적 expiry로 지우지 않음. 명시Lost/reset/epoch변경의 기존 clear 정책 유지. timestamp/status/poseFresh 원본 mutation0.
- 현재 Worker가 종료/부재(`worker=null`)이고 configured=false이며 실제 failure metric이 있는 경우 다음 UI tick 즉시 error. `workerErrors`는 누적값이므로 이것만으로 현재 failure를 판단하지 않음. 과거failure→복구→Worker를 보유한 정상reset/configure 대기에서는 Waiting/reset으로 처리하고 예전error로 분류하지 않음.
- 회귀: 최초 valid→같은result100ms/500ms/501ms/5초, 새sequence의오래된pose, 음수/NaN age, worker_failure/null, failure→복구→정상reset 대기, 재개 새result, dedup append1회, Limited/Unknown/Lost/epoch/reset 유지. 기존 fixture의 오래된 가상clock은 현실적인 시각으로 명시 수정하고 기존 의미 assertion 유지.

### R — 수학적 replay와 과거 pre-submit count 분리

- 소유 파일: `scripts/dev/synthetic-browser.mjs`, `tests/browser/synthetic-input-contracts.test.mjs`, 개발 runbook.
- raw 기본 IMU 공급은 `future-bracket`: 기존 `imuBatchRanges` 재사용, frame마다 필요한 뒤쪽 endpoint까지 미리 공급. captured input의 pre-submit counts가 존재해도 누락된 live 후속 도착을 재현했다고 가정하지 않음.
- 과거 동작은 명시적 `--imu-policy captured-counts`로만 유지. captured count가 없으면 거부하고, 결과에 `pre-submit-only / not-live-schedule` 표시. input JSON·pixels·IMU bytes/순서·effective profile 불변, sample 중복 전송0.
- 기존 malformed counts 검증 유지; policy는 raw에만 적용. live 모드에 captured-counts 요청은 실행 전 거부. report에 선택한 정책·실제batch·미검증 schedule 한계 저장.
- strict recorded-arrival replay는 arrival metadata 없이는 구현했다고 주장하지 않음. 이번 변경은 수학적 replay의 인위적bracket실패를 제거하는 범위.
- 회귀: 독립 known endpoint/count fixture, exact timestamp·float경계·연속frame·잘못된policy·counts없음·오래된 captured option. 실제392행 fixture는 기본경로347fresh/6current-failure/0missing, 이전299fresh/50missing은 명시diagnostic선택으로 보존. 기본경로의 pose/state/timestamp/epoch가 frozen future-bracket control과 exact 일치해야 함.

### 검토·개발·최종 검증 순서

1. 심화 RE의 관측/가설/미확인 분리, 실행 계약에 구현 근거 연결.
2. 독립 architecture/correctness, regression/evidence, scope/standards 검토. **각 lane APPROVE** 전 U/R product edit 금지. blocker는 계획 수정 후 재검토.
3. U/R 독립 파일 소유 executor에 위임. 먼저 기존 결함 red, 최소수정 green. 새 dependency·estimator/SDK/WASM 변경·배포0.
4. 전체 browser 계약·syntax·분석기 검사, 실제 captured replay·fresh source WASM의 synthetic App 회귀. UI stale행을 estimator Lost로 바꾸거나 accuracy 분모에서 제거하지 않음.
5. writer와 다른 컨텍스트에서 standards/intent 두 축 최종 리뷰. 결과·소스hash·실패 receipt·한계 기록 후 완료.

### 추가 RE 관측 → 채택/보류 판단

| 새 관측 | 계획 반영 | 구현 범위/검증 한계 |
| --- | --- | --- |
| 8th Wall 원래 projection 함수는 zero/negative depth에도 boolean1 반환; nonfinite 가능 | solver 반환·finite·cheirality·current support를 분리 유지 | 현재 geometry gate 완화와 C3 기본ON은 이번 채택 대상에서 제외 |
| TranslationStability는 dt 없는 `0.1·(C−2C₂+C₁)`; 같은 가속도30/60Hz 잔차4배, 불균일간격 일정속도에도nonzero | frame-step regularizer와 물리 IMU acceleration 구분 | jump 감소만 보고 smoothing/regularizer 기본 추가하지 않음; 향후 variable-dt/GT/지연 연구 gate 필요 |
| scalar Scale3d는 negative scale/zero observation에도 호출 가능 | responsive scale·scalar alignment·metric scale 검증 별도 | metric fresh 판정을 vendor status/함수 성공으로 대체하지 않음 |
| ARCore frame timestamp·tracking state/reason은 별도 cached-field 경계 | 결과 객체 존재/과거Tracking과 현재 화면freshness 분리 | U의 실시간age·현재failure·stale pose 표시 gate. 500ms 값은 우리UI정책이며 ARCore상수가 아님 |
| ARCore future-image clock 검사·역순 buffer 데이터 거부·IMU 초기시각 검증 내부함수 | clock provenance/monotonic/bracket 계약 유지 | 역순 buffer의 전체 데이터 종류를IMU로 단정하지 않음. R의futureendpoint 수학적 replay와 실제arrival schedule을 구분; 기존 미래clock 거부를 제거하지 않음 |
| sensor 재설정의 mutex/condition wait, image/pose/intrinsics별별도 dispatch | Session.update/공개getter 시간과 full sensor/solver 시간을 같은 metric으로 취급하지 않음 | ARCore빠른getter를 이유로 우리의IMU 계산을 병목이라고 단정하지 않음 |
| 4-byte Camera_release가 atomic refcount 감소·destructor 경로로tail-call | 짧은export·decompiler표면으로 no-op/자원무소유라고 결론내리지 않음 | 현재browsertrack/VideoFrame/분석사본 수명 보존; zero-copy/버퍼재사용 변경은 별도ownership gate |

- 위 ARCore는 실제 설치 ARM64 service에 대한 정적code/xref 관측, 8th Wall은 isolated original-code 성분 실행. 전체solver/runtime thread/실기기정확도 검증으로 합산하지 않음.

### 후속 실행 결과

- **U/R 구현·테스트 완료**: [상세 결과](../fixes/2026-10-14-reference-reviewed-runtime-fixes.md).
- 구현 전 3개 관점 APPROVE, 구현 후 Intent/Standards 두 축 PASS. 승인 당시 계획/리뷰는 `build/slam-revision-20261014/review-gate/`에 불변 snapshot으로 보존.
- browser297/297·exporter3/3 통과. raw default는 frozen392행×19필드 exact 일치,347fresh·current failure6·missing0. 명시 captured-counts는 과거50missing 실패를 같은392행으로 재현.
- 현재sourceWASM/기존activeWASM 각각 새 App의 실제 desktop browser 실행 확인. 현재source의400frame 합성 GT는해당 gate PASS. 입력 채택 집합이 다른 live 간 지표 차이를 수정의 정확도 향상으로 해석하지 않음.
- 0A의 provenance 고정은 완료했으나 active artifact 승격은 하지 않음. 1A 물리 calibration·2 성능·3 visual-support/recovery·4 실기기 최종 수락은 해당 증거가 남아 있으며, U/R 완료로 대체하지 않음.
