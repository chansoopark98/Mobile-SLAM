# U/R 실행 계약 독립 검토 — 범위·표준·완료 기준

- 검토자 역할: 계획 작성·제품 구현과 분리된 scope/standards reviewer.
- 대상: [개선 계획](2026-10-14-slam-improvement-plan.md)의 후속 실행 계약 U/R.
- 검토 범위: 현재 코드·재현 근거→수정 범위의 연결, 기존 보호 유지, 완료 주장 한계, AGENTS 및 검증 계약.
- 제품·계획 파일 수정 없음. 검토 문서만 작성. 계획의 추가 RE→채택/보류 표와 failure→복구→정상reset 구분 보완까지 재검토.
- **최종 verdict: APPROVE — U/R 실행 계약에 한정. 미해결 blocker 0.** 신규 8th Wall/ARCore 심화 문서·원시 결과와 수정된 RE→행동 표까지 검토.

## 기준별 검토

| 기준 | 판단 | 근거 |
| --- | --- | --- |
| 사용자 의도 충족 | 타당 | 추가 RE→다각도 review→승인 범위 implementation/test 순서가 명시됨; 분석만 하고 끝내는 계약이 아님 |
| 즉시 의미 있는 수정 | 타당 | U는 실제 stale 표시 오류, R은 recorded pre-submit membership을 live schedule로 잘못 소비하는 재생 오류를 해결 |
| 장기 작업 범위 | 타당 | calibration/GPU frontend/local-map recovery는 separate gates로 유지; U/R 통과를 mobile tracking/accuracy 전체 수락으로 주장하지 않음 |
| 기존 회귀·데이터 보존 | 타당 | original input/pixels/IMU/profile 불변, 실패299fresh/50missing receipt 및 captured-counts diagnostic 경로 보존 |
| 표준·구조 | 타당 | U는 App/Renderer, R은 dev replay/test/runbook 독립 ownership; 기존 imuBatchRanges 재사용; 신규 runtime dependency·estimator/WASM 편집 없음 |
| 관측과 정책 | 타당 | 500ms는 display expiry policy. ARCore/8th Wall 내부 임계치·정확도·처리시간으로 포장하지 않음 |
| 검증 비례성 | 타당 | UI frozen-clock 단위 경계 + 기존 browser contracts + 실제 captured replay + fresh-source synthetic App 회귀; C++ 미수정에 무관한 전체 native rebuild 반복 요구 없음 |
| 실패 후보 승격 | 차단됨 | capture-worker/GPU/C3/geometry rescue를 검증 없이 기본 적용하지 않음; false-ID/metric-depth 보호 유지 |

## U: 승인 가능한 계약과 최종 확인 항목

- 현재 `_consumeResult`는 null/동일 result key에서 즉시 return(`web/js/app.js:1333–1337`). 실제 Wrapper failure는 `_invalidate`로 latest cache를 비우고 configured=false 및 worker terminate(`vio-wrapper.js:296–302`). 기존 격리 probe에서 stale label/frustum 잔류를 재현했으므로 수정 동기가 직접적임.
- pose identity dedup를 geometry에만 적용하고 age/status를 독립 갱신하는 방향은 원본 pose/state를 조작하지 않고 표시 오류를 수정.
- 시간 경계 `0<=age<=500ms` 포함 조건이 명확. frozen clock의 500/501ms, NaN/negative age, 동일 sequence/new-old sequence, failure/null/resume으로 독립 기대값 정의 가능.
- historical trajectory/map을 보존하면서 current frustum을 hide해야 함. expiry마다 기존 `clear()`를 호출하면 trajectory/map까지 지우므로 새 hide 경로 또는 동등한 최소 분리가 필요. 최종 테스트는 frustum false뿐 아니라 history buffer/count 불변도 확인.
- Worker error metric은 cumulative일 수 있음. 최종 계획은 현재 worker=null + configured=false + failure metric을 함께 판정하며, 복구 후 Worker를 보유한 정상 reset/configure 대기에서 과거 error로 오분류하지 않는 회귀도 포함. 새 실패는 다음 visible UI tick에서 표시하고 UI throttle 때문에 100ms 늦게 전환되지 않도록 확인.
- Lost/reset/epoch는 기존 clear 정책 유지. Limited의 원본 quality/metric 의미와 evaluator population은 유지. UI expiry를 engine Lost 또는 poseFresh=false로 덮어써서는 안 됨.
- 10Hz status throttle은 새로운 성능 가설을 증명하는 최적화가 아니라 UI 갱신 과부하 제한. raw IMU 표시/age도 결과가 없는 동안 갱신 가능해야 함.
- 기존 0.08 follow-view alpha를 이번 U에서 제거하지 않는 것은 blocker가 아님. 이는 observer camera의 UX 동작이고 이번 결함은 stale/error contract. 최종 보고에서 **엔진 latency 또는 follow-view lag가 해결되었다고 주장하지 않을 것**.

## R: 승인 가능한 계약과 최종 확인 항목

- 현재 `replayIMUBatches`는 `input.capture.imuSubmissionCounts` 존재만으로 이전 membership을 자동 적용(`scripts/dev/synthetic-browser.mjs:71–73`). raw loop는 해당 batch 전송→frame→await completion(`:220–232`)이라 live frame-in-flight 후속 IMU가 재현되지 않음.
- 기본 future-bracket은 기존 `imuBatchRanges`의 단조 cursor/재사용 endpoint 계약(`scripts/dev/replay-android-input.mjs:100–108`)을 사용. 새 sensor arrival scheduler를 허위로 합성할 필요가 없음.
- strict recorded arrival에 필요한 metadata 부재를 명시하고 mathematical replay로 이름을 제한한 점이 타당. 해당 변경이 source timestamps/physical input을 개선하거나 phone sampling을 재현하는 것은 아님.
- captured-counts는 explicit diagnostic으로만 허용. missing counts/invalid policy/live mode 요청을 시작 전에 거부하고, future-bracket에서도 malformed recorded counts 검증은 기존대로 유지.
- 동일 input JSON file/pixel SHA/IMU bytes·순서/profile/seed·cvThreads/loader·Worker SHA를 비교해야 함. 정책만 다른 A/B를 위해 input에서 count 필드를 삭제한 복제 파일을 새 기본 입력처럼 만들지 않을 것.
- 단위 fixture는 expected endpoint/count를 독립적으로 써야 함. `imuBatchRanges`를 기대값 생성에 다시 호출하여 자기검증하는 테스트는 부족.
- 실제392행 전체 분모 유지. 기본347fresh/6current-failure/0missing 및 diagnostic299fresh/50missing 기대가 frozen control과 일치해야 함. count가 맞아도 pose/state/timestamp/epoch가 다르면 exact parity gate 실패.
- 이전 실행이 증명한 live347fresh와 mathematical347fresh 일치는 해당 입력/설정의 control 결과. schedule/latency/hardware exposure/metric accuracy 일치로 확대 금지.

## 최종 구현 리뷰의 범위

- 변경 파일만 staging/commit 대상; 기존 연구 자료·dataset·artifact/service 보존. commit을 수행한다면 사용자 AGENTS의 Lore intent/trailer 계약 적용.
- JS syntax를 ESLint/TypeScript 결과로 부르지 않음. 실제 프로젝트에 없는 도구 gate를 새 dependency 설치로 만들어내지 않음.
- 모든 기존 browser 테스트의 red/green 수·TODO/skip을 구분. 테스트 clock 변경은 현실적인 frozen time으로 바꾸되 기존 의미 assertion을 제거하지 않음.
- 실패 로그와 fresh-source baseline53/300 같은 기존 품질 한계 유지. UI 수정 후 screenshot이 좋아져도 estimator coverage/accuracy 개선으로 보고하지 않음.
- 완료 선언: **U 화면 상태 contract + R replay policy contract 구현·지정 검증 완료**. 전체 장기계획, physical8th Wall parity, SLAM 정확도 목표, push/deploy/field acceptance는 별도.

## 심화 RE 확인과 최종 verdict

- [8th Wall 심화](2026-10-14-8thwall-deep-runtime.md) 검토 완료. 원시 `build/slam-revision-20261014/eighthwall/residuals-v2/result.json`도 확인: count79/passed79/pass=true, invokedImports0, non_export_sections_unchanged=true, Code section SHA `6ff897c9bfb8709f4b792d1595eae175e4bc5947f09d16c011273e94be00a809`.
- projection/negative-depth/nonfinite 경계와 boolean success를 분리하고, TranslationStability가 dt 없는 frame-step 차분임을 명시. U/R에서 geometry gate 완화·임의 smoothing 도입을 제외한 결정과 일치.
- scalar scale helper를 VIO metric initialization으로 일반화하지 않고, 79조건을 camera tracking 성공으로 합산하지 않음. source 식·binary 함수 산술·실제 tracker branch 활성도를 구분한 보고 범위가 적절.
- [ARCore 심화](2026-10-14-arcore-deep-runtime.md) 및 `verification.json`/`analysis-final/summary.json` 확인: input SHA `7c32d45c2ef1ebaf92deae86a7ba31b0dd00d39bde09ab0890535ff396d62050`, source_unchanged=true, bounded46/46 inventory 일치, final_commands200/모두exit0, decoder_control_pass=true.
- cached timestamp/state와 새로운 estimation 구분, camera/display/sensor pose·CPU/GPU intrinsics별 dispatch, sensor mutex/condition wait, tail-call atomic reference release를 관측 경계로 유지. getter latency를 estimator 전체 성능으로 확장하지 않아 U/R scope와 일치.
- 근거 문구 보완 요청 1건: 계획의 RE 표에서 `역순IMU 거부`를 `timestamp buffer 역순 삽입 거부`로 정정. ARCore 문서는 `0x8fa000`의 데이터 종류 전체를 IMU로 특정하지 않는다고 명시. 의미상 U/R 설계 변경은 필요 없음.
- 정정 확인 완료: 계획 표현이 `역순 buffer 데이터 거부·IMU 초기시각 검증`으로 변경되고 buffer의 data kind가 미확인임을 추가 명시. 미해결 근거 귀속 문제 없음.
- **APPROVE**: U와 R은 현재 재현 결함에 직접 대응하고 기존 estimator/input/geometry·GT 계약을 보존하는 최소 실행 범위다. 다른 review lane까지 승인한 뒤 구현 진행 가능.
- 승인 대상은 계획과 acceptance contract. 구현 완료·테스트 통과·장기 mobile accuracy·SDK 동등성·배포 승인으로 읽으면 안 됨.

- 검토 시 계획 SHA256: `8624759dd1d5c69b7f76c40ad6220f92c5192d15619ce65a9ef8532063af22b5` (이후 review-status 기록 변경은 별도).
