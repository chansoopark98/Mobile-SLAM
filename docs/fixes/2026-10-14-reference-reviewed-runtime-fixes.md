# 레퍼런스 심화 역분석·리뷰·웹 런타임 수정 결과

- 날짜: 2026-10-14, Asia/Seoul. 기준 HEAD `5d48e4bd7466842dad98928fb1f12988488402df`.
- 요청 순서 수행: **8th Wall/ARCore 추가 RE → 개선 계획 반영 → 3개 관점 독립 리뷰 → 승인 U/R 구현 → 테스트 → 2개 축 독립 코드 리뷰**.
- 구현 범위: U 표시 freshness/error, R 수학적 raw replay의 IMU 공급 정책. 전체 SLAM 정확도·성능 개선 완료와 구분.
- 제품 C++·Worker/Wrapper·IMU 입력 코드·원본 SDK·활성 WASM 변경0. 신규 dependency·commit·merge·push·deploy0.
- [개선 계획](../research/2026-10-14-slam-improvement-plan.md), [종합 비교](../research/2026-10-14-slam-comparison.md), [최종 통합 검증](../../build/slam-revision-20261014/verification.json).

## 1. 추가 바이너리 분석

| 대상 | 새로 수행한 분석·실험 | 확인한 의미·한계 |
| --- | --- | --- |
| [8th Wall](../research/2026-10-14-8thwall-deep-runtime.md) | 실제 WASM 내부 projection/translation-stability/scale 함수3종, **79/79** residual/Jacobian/domain 검사. Export section만 추가, code/data 등 동일, import259개 중 호출0 | projection 성공값과finite/positive-depth는 다른 조건. stability는 dt 없는frame-step2차차분. scalaralignment와metricVIOscale 구분 |
| [ARCore ARM64 service](../research/2026-10-14-arcore-deep-runtime.md) | 실제설치ELF35,801,624B의 API25+내부13+callee8, **46함수·200명령**. GNU/radare bounded46/46 일치, 독립decoder2/2 | cachedtimestamp/state, 별도camera/display/sensorpose·CPU/GPU K, imageclock·IMU초기시각·genericbuffer역순검사, sensorlock/wait, atomicreferencecount |

- 새8thWall stability 성분: `r=0.1·(C−2C₂+C₁)`. 동일3m/s²의30/60Hz 입력에서 잔차4배, 일정속도라도 불균일간격이면nonzero. 물리IMU가속도factor로 바로 이식하지 않음.
- Snavely zero/negativeZ 및 scalar negative/zero-observation에서도 boolean1 반환 가능. caller의geometry·rank·scale 검증을 대체하지 못하므로 기존 metric gate 유지.
- ARCore의4-byte `Camera_release`는no-op이 아니라 atomiccounter감소/destructor로tail-call. 짧은함수/문자열/수치offset만으로 기능·ownership·속도를 단정하지 않음.
- ARCore는 정적코드 경계,8thWall은 격리성분 실행. private전체solver·실제frame에서의branch선택·기기성능 검증 아님.

## 2. 계획 리뷰와 승인 범위

- 승인시점 snapshot/해시: [review-gate/approval.json](../../build/slam-revision-20261014/review-gate/approval.json). gate 생성 뒤 구현 시작.
- [Architecture/correctness](../research/2026-10-14-slam-revision-review-architecture.md): **APPROVE**. 누적workerErrors만으로 정상reset을실패로오인하는 계약 수정 후 승인.
- [Regression/evidence](../research/2026-10-14-slam-revision-review-verification.md): **APPROVE**. 500/501ms,10Hz,실패→복구→reset,392행exact·과거실패보존 조건 명시.
- [Scope/standards](../research/2026-10-14-slam-revision-review-scope.md): **APPROVE**. ARCore의genericbuffer역순거부를IMU전체로잘못귀속한 문구 정정. U/R을실기기/전체tracking승인으로확대하지 않음.
- 보류 유지: GPUfrontend·descriptor재연결·stabilityregularizer·관측gate변경·C3기본ON. 해당기하/false-ID/GT/latency조건미충족 상태에서 채택하지 않음.

## 3. U — 화면 freshness/error

- 변경: `web/js/app.js`, `web/js/renderer.js`.
- pose/map/trajectory 반영은 result identity당 1회. 같은 결과라도 age·IMU 상태를 갱신. 반복 text는 최대 10Hz, 새 result/error/stale 전이는 즉시 반영.
- 표시 age는 `0..500ms` 포함, 501ms부터 Stale. **500ms는 UI 정책**이며 vendor 상수·estimator validity·정확도 threshold가 아님.
- 임시 stale/null/failure에서는 현재 frustum만 숨기고 과거 trajectory/map 보존. 명시 Lost/Unknown/invalid Limited·reset·epoch 전환의 clear 동작 유지.
- 실제 Worker 부재 + configured=false + failure metric에서 Worker error. 누적 오류 이후 복구된 Worker의 정상 reset 대기는 Waiting이며 과거 error를 다시 표시하지 않음.
- Wrapper의 pose/status/timestamp/poseFresh·지도 payload를 수정하지 않음. follow-view alpha=.08도 유지. 체감 follow delay를 해결했다고 주장하지 않음.
- red: 27검사 중 21PASS/6FAIL. green: 관련 UI/video/geometry **78/78**, skip/TODO0. 실제 `VIOWrapper._workerFailure`와 reset 경로 포함. [receipt](../../build/slam-revision-20261014/ui/receipt.json).

## 4. R — raw replay의 미래 IMU endpoint

- 변경: `scripts/dev/synthetic-browser.mjs`, `scripts/dev/README.md`.
- raw 기본 `future-bracket`: 기존 `imuBatchRanges`를 재사용하여 필요한 frame 뒤쪽 sample을 먼저 전송. captured pre-submit count만으로 live 후속 delivery까지 재현했다고 가정하지 않음.
- 과거 배치 진단은 명시 `--imu-policy captured-counts`로 유지. 유효 counts 필수. 잘못된 policy/비정상 counts/live 모드의 명시 policy는 실행 전 거부.
- report에 requested/selected 정책, 실제 batch, `not-live-schedule` 한계를 기록. source JSON·pixels·IMU 값/순서·effective profile·WASM 불변. 실제 arrival 시간표를 재현하는 기능은 아님.
- red: 21검사 중 17PASS/4FAIL. green: **21/21**. 정확한 float endpoint·carry·중복 전송0·잘못된 요청·기존 membership 회귀 유지.

| 실제392영상재생 | 기본future-bracket | 명시captured-counts진단 |
| --- | ---: | ---: |
| 완료 |392|392|
| fresh |347|299|
| missing_bracket |0|50|
| current/final support실패 |6/0|2/2|
| frozencontrol 대비19필드exact차이 |0|0|
| 원본IMU1201개중전송prefix |1199|1198|

- 전송되지 않은 tail은 원본에 보존, sample 중복 전송0. count 차이는 정책에 따른 공급 경계이며 원본 데이터 삭제가 아님.
- [default대조](../../build/slam-revision-20261014/replay/default-exact-comparison.json), [과거실패대조](../../build/slam-revision-20261014/replay/captured-exact-comparison.json), [implementation receipt](../../build/slam-revision-20261014/replay/implementation-receipt.json).
- 이 수정은 개발 재생기가 만들던 인위적 bracket 실패를 제거. 실제 phone의 IMU source Hz·latency·current visual support를 개선한 결과가 아님.

## 5. 통합·실제 브라우저 검증

| 검증 | 결과 | 범위 |
| --- | --- | --- |
| 변경전browser계약 |285/285|기존baseline|
| 변경후browser계약 |**297/297**,fail/skip/TODO0|기존계약+새UI/replay회귀|
| exporterPython |**3/3**|GT출력좌표/시각검사|
| syntax·diff whitespace |PASS|JSparse/Pythoncompile; ESLint/TS typecheck로명명하지않음|
| ARCoredecoder |2/2|독립ARM64주소decodercontrol|
| 8thWall추가성분 |79/79|원래WASM계산/경계·Jacobian|
| 현재sourceWASM+새App/Worker |400계획/392완료/**352fresh**,warmup297/300,reset0,pageerror0|naturalrAF·가상DeviceMotion·desktopSwiftShader|
| 기존activeWASM+새App |120계획/108완료/54fresh,postwarmup20/20,reset0,pageerror0|활성구형artifact와JS호환실행;전체GTgate아님|

- 현재 source WASM은 이전 단계의 고정 `38837d4…`, active는 `140e29a…`. 이번 JS 수정으로 WASM을 재빌드하거나 승격하지 않음.
- 현재 source App 합성 GT: SE(3) scale1, ATE **0.0461957m**, rotation APE **1.85369°**, 1s RPE **0.0268945m/0.858752°**, 원 evaluator PASS. [평가](../../build/slam-revision-20261014/app-current-fast707/evaluation.json).
- 이전 live와 accepted 집합이 3개씩 다름: 이전에만 13/41/45, 이번에만 21/46/47. 따라서 ATE/fresh 차이를 이번 UI 수정의 정확도 향상이라고 주장하지 않음. 고정 입력 동등성의 직접 증거는 위 392행 raw control.
- 각 실행의 JS/Worker/loader/input hash·effective profile·source ledger·오류·분모는 [현재 App report](../../build/slam-revision-20261014/app-current-fast707/report.json), [active 호환 report](../../build/slam-revision-20261014/app-active-clean120/report.json)에 보존.
- 구현 후 독립 review: [Intent축 PASS](../research/2026-10-14-slam-revision-code-review-intent.md), [Standards축 PASS](../research/2026-10-14-slam-revision-code-review-standards.md). 두 축에서 확인된 수정 필요 결함0. 작성자와 검토자를 분리.

## 6. 변경·재현·남은 범위

- tracked 변경7개: 위 App/Renderer2개, synthetic runner/runbook2개, `tests/browser/{mobile-source-contracts,synthetic-input-contracts,video-frame-contracts}.test.mjs`3개.
- 단순화: 기존 frame identity·renderer·IMU batch helper 재사용. 새 queue·estimator abstraction·runtime SDK/dependency 추가0.
- 추가 기록: 심화 RE 문서2개, review 문서5개, 본 결과와 갱신 plan, 8th Wall probe1개·ARCore 분석/decoder control2개. 기존 연구 자료와 실패 evidence 보존.

```bash
bash scripts/dev/check.sh browser-contracts
bash scripts/dev/check.sh syntax
python3 scripts/dev/references/arcore/test_binary_boundaries.py
node scripts/dev/references/eighthwall/binary-deep-residual-probe.mjs \
  --input build/8thwall-deep-20261014/binary/survey/xr-slam/target.wasm \
  --output build/slam-revision-20261014/eighthwall/new-run
node scripts/dev/synthetic-browser.mjs \
  --input build/slam-comparison-20261014/replay/current-fast707-live/captured-input.json \
  --loader build/slam-comparison-20261014/build/wasm/vio_engine.js \
  --output build/slam-revision-20261014/replay/new-default --mode raw --frames 392
# 과거실패진단: 위명령에 --imu-policy captured-counts,새output사용
```

- 미해결: 실제 phone에서의 Lost/발산·calibration/exposure·descriptor 재연결·기하 관측·전체 solver 성능. 기존 H4/D0/D2/D3/C3 실패 및 field GT 부재는 유지.
- 향후 순서: 물리 입력/기하 lineage와 실제 cost 계측을 확보한 뒤 GPU frontend·recovery 후보를 기존 gate로 검증. 이번 표시·재생 수정을 전체 tracking 해결로 확대하지 않음.
