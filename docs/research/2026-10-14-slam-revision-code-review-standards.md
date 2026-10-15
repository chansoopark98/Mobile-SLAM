# 최종 코드 검토 — 저장소 표준·소유권·유지보수

- **판정: PASS. 검토한 diff에서 수정이 필요한 확인된 결함 0건.**
- 축: 저장소 계약, runtime 데이터/자원 소유권, 오류 수명, 기존 구현 재사용, 의존성·범위, 유지보수. 사용자 의도/전체 성과 수락은 별도 intent review 축.
- 읽은 지침: root `AGENTS.md`, 사용자 제공 작업 계약, `.agents/skills/mobile-slam-validation/SKILL.md`, 승인된 U/R 계획·pre-review receipt. `web/`, `scripts/`, `tests/`에서 추가 AGENTS 파일 없음.
- 검토한 실제 tracked diff: 7개 파일. `5d48e4bd7466842dad98928fb1f12988488402df` 대비 U/App/Renderer와 R/dev harness 및 회귀·runbook.
- 제품 코드·계획 편집 없음. 이 독립 review 문서만 작성. 하위 위임 없음.

## 코드 판단

| 기준 | 판단·구체적 근거 |
| --- | --- |
| 입력/estimator 소유권 | U는 결과를 읽어 표시 여부만 계산. result.poseFresh/statusCode/timestamp/pose/map payload를 수정하지 않음. Worker·wrapper·camera·IMU·C++·WASM tracked diff 없음 |
| 결과 dedup | key/epoch는 pose/map/trajectory 갱신에 계속 사용. 시간/상태 계산은 그 바깥으로 이동. 동일 결과에서 trajectory를 반복 append하는 구조를 새로 만들지 않음 |
| 만료와 clear 분리 | `Renderer.hideCameraPose()`는 frustum visible만 false로 바꿈. finite pose의 일시 expiry/null/failure에서 history 보존; explicit invalid/Lost 및 epoch/reset의 기존 clear 유지 |
| 오류 수명 | worker null/configured false/cumulative workerErrors를 함께 검사. 과거 failure 이력이 남아도 정상 Worker가 존재하는 reset 대기를 error로 판정하지 않음. `_workerFailure`를 수정하지 않고 기존 lifecycle 상태를 이용 |
| UI 주기 | 기존 RAF `_consumeResult` 경로 재사용. 새 timer·poller·subscription 없음. 동일 status의 반복 문자열 갱신은100ms 간격, 새 result 및 상태 전이는 즉시. geometry render cadence나 follow alpha는 변경하지 않음 |
| 시간/좌표 단위 | now ms와 poseTimestamp seconds를 명시적으로 ×1000하여 비교. 500ms는 주석으로 display policy 한정. engine source timestamp·IMU bracket·좌표계/K 변경 없음 |
| API 배포 일관성 | renderer import query를 v11→v12로 갱신하여 새 `hideCameraPose()` API와 이전 cached renderer의 충돌 위험을 줄임. Renderer의 기존 clear/pose/map API 변경 없음 |
| 재사용 | R은 검증된 `imuBatchRanges`를 그대로 호출. 새 endpoint slicing 알고리즘·arrival scheduler·queue/engine wrapper를 추가하지 않음 |
| 입력 검증 | `capturedIMUCounts`에 기존 count validator를 모아 default 진입과 diagnostic에서 재사용. false/null/0 등 이전 truthy 검사로 통과하던 malformed field도 fail-closed. count 부재만 undefined로 구분 |
| CLI 오류 경계 | 명시 policy는 raw 전용이며 unknown/live policy를 input/output/browser를 열기 전에 거부. missing captured counts도 browser launch 전 계산 단계에서 거부 |
| 데이터 불변 | batch 정책 선택은 frames/IMU/capture metadata를 수정하지 않음. raw entry가 미리 계산한 batch를 page.evaluate로 전달. 각 sample 전송 구간이 명시되고 원본 파일을 재작성하지 않음 |
| 결과 해석 | requested/selected policy, actual row batch start/count, mathematical 또는 pre-submit-only/not-live-schedule 한계 기록. recorded arrival를 구현한 것처럼 이름 붙이지 않음 |
| 의존성·범위 | package/lockfile/build configuration 변경0. 제품 runtime dependency 추가0. R은 `scripts/dev/`와 기존 test/runbook에 한정. active artifact/SDK 배포 변경0 |
| 유지보수 | 작은 display hide method와 두 bounded policy/count helper만 추가. 범용 상태관리 계층·buffer pool·shared-memory·새 추상화 없음. 기존 파일별 문체·테스트 fixture 패턴을 따름 |

## 회귀 근거의 질

- UI receipt `build/slam-revision-20261014/ui/receipt.json`: red27개 중6실패 → 관련78개 green, fail/skip/TODO0. 새 테스트가 stale/error 현재 결함을 먼저 드러낸 기록 보존.
- frozen-clock500/501ms·같은result age 증가·미래/비정상시간·error/null/복구/reset·raw field 불변·history 보존을 독립 기대값으로 확인. renderer hide 직접 method 검사와 실제 `VIOWrapper._workerFailure` 연결 검사도 있음.
- 기존 test clock3곳은 pose timestamp1s에 now1s를 맞춘 변경이며 기존 loss/epoch/Limited assertion 유지. video callback fixture에는 새 no-result 상태 갱신이 필요로 하는 실제 IMU diagnostics 모양을 추가; 제품 경로 mock 대체 확대 없음.
- R receipt `replay/implementation-receipt.json`: red21개 중4실패 →21개 green, fail/skip/TODO0. expected batch는 fixture 숫자로 직접 선언하여 구현 helper로 기대값을 생성하지 않음.
- binary64 exact/인접 endpoint, 이미 공급된 future endpoint의 zero-count 다음 frame, 연속/prefix frame, malformed counts, deprecated policy 이름, live-mode 사전 거부를 검사.
- actual captured replay: 기본392행/347fresh/6current-support 실패/0missing; diagnostic392행/299fresh/50missing 유지. 두 `*-exact-comparison.json`의 comparedFields19개 exactMismatchCount0. source/IMU/PGM/profile/loader parity 기록 확인.
- 원본 IMU1201행 중 필요한 prefix1199/1198만 소비하고 trailing2/3행을 보존한 정정 receipt 확인. 모든 source행을 억지로 소비하게 하거나 실패50행을 분모에서 제거하지 않음.
- 각 lane receipt의 현재7개 파일 SHA를 독립 계산하여 모두 일치 확인. 이 검토에서 동일 focused suite를 중복 재실행하지 않음; 상위 통합 lane이 전체 browser/syntax 및 actual App 회귀 수행.

## 새 RE 도구의 경계 확인

- `scripts/dev/references/arcore/binary-boundaries.py`: ARM64 input SHA pin, 새 output directory `exist_ok=False`, canonical dataset 하위 출력 거부, stdout/stderr/hash receipt. 별도 assembler control 및 원본 불변 receipt는 기존 심화 검토에서 확인.
- `scripts/dev/references/eighthwall/binary-deep-residual-probe.mjs`: 원본 WASM SHA pin, existing output 거부, 새 분석 사본의 export section만 추가, 예상하지 않은 import를 trap. 제품 runtime loader로 연결하는 diff 없음.
- 이 두 도구의 핵심 소유권/입출력 범위만 이번 표준 축에서 재확인. 전체 RE 소스에 대한 별도의 완전 보안 감사로 주장하지 않음.

## 남은 제한과 최종 통합 조건

- 상위 lane의 전체 browser 계약·actual App/Worker 통합 결과는 이 검토 시 별도 진행 대상. 본 PASS를 그 실행 완료로 바꾸어 보고하면 안 됨.
- 500ms는 우리 display 정책이며 phone latency 목표 또는 ARCore/8th Wall threshold가 아님. 현재 follow alpha=.08, engine 성능·정확도·capture 경로는 이번 수정으로 개선되었다고 주장할 수 없음.
- expiry 보존 대상은 유효했던 historical map/trajectory. 비정상 producer pose/nonfinite timestamp 등의 기존 invalid-result clear와 temporary expiry를 구분한 코드/테스트 확인.
- field/GT/장시간 thermal·다기기 수락, deploy/push/commit은 별도 상태. 제품 SDK/원본 dataset에 쓰기 발생 근거 없음.
- source 변경 후에는 아래 SHA와 review 범위를 다시 맞춰야 함. 코드 수정이 없다면 focused 검사를 반복하는 대신 이미 확보한 실행·통합 결과를 최종 보고에 연결할 것.

## 검토 snapshot

- tracked diff SHA256: `2ed99058c1821d37889a93a99ab5fc853f19b107c9a2ab3cb262210368d699d0`.

| 파일 | 검토 SHA256 |
| --- | --- |
| `scripts/dev/README.md` | `f734f61bfb083558c6e176e3628a16366727781970281f8c357feecf9fb34676` |
| `scripts/dev/synthetic-browser.mjs` | `a778285bf32482fe61a31a0e5607b33c0f9a1f8bc07d8b9b4d38662e2c9ad615` |
| `tests/browser/mobile-source-contracts.test.mjs` | `f60118989ab3a572a29af3f7fec699c7fd63ebab244f47cc74f21961b1302820` |
| `tests/browser/synthetic-input-contracts.test.mjs` | `a5a38922858c56bfc8d784887ed5dcc49e3ab50e02bf0a5d53e984b0bd340d99` |
| `tests/browser/video-frame-contracts.test.mjs` | `4c96fb65ae7f6670b7c9bb0ef64627d1f1fb4990e6202247e59c22851d40b069` |
| `web/js/app.js` | `c8454d99aea7c75b6c6552919cdbcad5b6f14bf0d488520e7c7103fd73ccc481` |
| `web/js/renderer.js` | `ebbf1c75e2411170178502e07bd984965db807c9a8b8bb287862f6eb3c7e97e6` |
