# 후속 U/R 실행 계약 독립 검토 — architecture·correctness

- 날짜: 2026-10-14, Asia/Seoul.
- 검토 대상: [개선 계획](2026-10-14-slam-improvement-plan.md)의 장기 단계와 후속 실행 계약 U/R. 구현 승인 범위는 U/R만이며 장기 알고리즘·GPU·실기기 개선 전체가 아님.
- 최종 판정: **APPROVE — U/R 범위 한정**. 두 심화 보고서·근거 JSON·수정된 실행 계약과 채택/보류 matrix 확인 완료. 남은 blocking finding0. 구현 후 검증은 아래 수락 사례에 따라 별도로 수행.
- 방법: 기존 App/Renderer/Wrapper, synthetic raw runner·IMU batch helper, browser 회귀를 읽어 계약과 실제 수명을 대조. 제품·계획 수정 없음. 리뷰 파일만 작성.

## 발견 후 해결된 blocker

- 최초 U는 누적 `workerErrors>0 && configured=false`로 현재 Worker failure를 판정했다.
- `web/js/vio-wrapper.js:250–259`의 정상 reset도 configured=false인 대기를 만든다. 과거 failure에서 이미 복구한 뒤 reset하면 예전 오류가 다시 표시될 수 있음.
- root 수정 확인: 현재 worker 부재/종료(`worker=null`), configured=false, 실제 failure signal을 함께 사용. Worker를 보유한 reset/configure 대기는 Waiting/reset. `failure→복구→정상reset 대기` 회귀 추가.
- 현재 계약에서 위 blocker는 해결됨. 구현에서는 `workerErrors` 단독 판정을 다시 도입하지 않아야 함.

## U: 표시 freshness의 책임 범위

- 500ms는 표시 만료 정책으로 한정. estimator validity·IMU bracket budget·sensor 시각 보정·metric accuracy gate로 전파하지 않음.
- age는 동일 App monotonic clock의 `performance.now()`와 `poseTimestamp*1000`으로 계산. Wrapper의 epoch time을 쓰는 `sentAtMs`/roundtrip과 혼합하지 않음.
- 기존 state/finite/fresh/epoch/현재 Limited support 조건 뒤에 display age 검사를 적용. false metric pose를 화면 정책으로 승인할 경로가 없음.
- 동일 result의 pose/map/trajectory 적용은 한 번만 수행하고 age/상태는 계속 관측. 시각 갱신과 estimate 재처리를 분리하는 설계가 현재 dedup 결함에 맞음.
- temporary stale/null/failure에서 현재 frustum만 숨기고 historical trajectory/map 보존. 명시 Lost/reset/epoch 전환에서는 기존 clear가 우선. Wrapper payload·point count·pose timestamp·fresh flag 변경 없음.
- 현재 Renderer `clear()`는 trajectory/map/frustum 전부 지운다(`renderer.js:295–308`). 따라서 temporary hide가 기존 `clear()`를 잘못 재사용하지 않는지 별도 검증해야 함.
- UI text steady-state rate 제한과 새 result/error/stale 전이 즉시 반영을 구분. 표시 refresh가 estimator 입력 주기나 sensor 전달을 제어하면 안 됨.

### U 수락 사례

| 사례 | 요구 결과 |
| --- | --- |
| 같은 valid result, age100/500/501/5000ms | 100·500에서 표시,501·5000에서 숨김; pose/map/trajectory append는 최초1회; 원본 객체 불변 |
| 새 sequence지만 오래된 pose / 미래 pose / NaN timestamp | 새 sequence만으로 재승인하지 않음; frustum hidden·Stale/Waiting 원인 표시 |
| actual `_workerFailure` 이후 null | 다음 활성 UI tick Worker error·frustum hidden; historical history의 일시 만료 삭제 없음 |
| 과거failure→정상 재설정→정상reset 대기 | 누적 error counter로 Worker error가 재등장하지 않음; 새 valid result로 정상 복귀 |
| explicit Lost·Unknown·invalid Limited·epoch/reset | 기존 invalid-state clear·metric/position-uncertain 의미 보존; 이전 epoch map 연결 없음 |
| 임시 stale 뒤 새 valid pose | 최신 frustum 복구·한 번 append; map/trajectory 기존 이력과 새 값의 중복 없음 |

- 기존 `mobile-source-contracts.test.mjs`의 기본 now5000ms/poseTimestamp1s는 이미4초 오래된 pose이다. 기존 dedup·Lost·Limited·epoch 검사에서 현실적인 clock fixture로 바꾸는 것은 새 계약에 필요한 변경. 기존 assert 삭제·스킵·완화로 green을 만들면 안 됨.
- 별도 age source test의 now5000/pose4.8s는200ms 기대값 그대로 유지. Generic Sensor 미래 시각·source provenance assertions는 변하지 않아야 함.

## R: 수학적 입력과 도착 일정의 분리

- 현재 `synthetic-browser.mjs:71–73`은 captured pre-submit count가 있으면 자동 우선 사용한다. live의 frame-in-flight 후속 IMU가 count에서 빠져 있어 raw의50개 missing bracket을 만들었으므로 기본 정책 교정 근거가 명확함.
- future-bracket 기본은 기존 `imuBatchRanges`(`replay-android-input.mjs:100–108`) 재사용. cursor가 단조 증가하여 sample 중복 전송을 피하며 필요한 endpoint를 제공하는 수학적 제어 입력이다.
- 과거 동작은 explicit captured-counts diagnostic으로 유지. 실패 산출물·입력의 count 필드는 제거하지 않음. report의 선택 policy·batch provenance가 source capture metadata와 혼동되지 않아야 함.
- 이 변경은 실제 sensor rate 증가·IMU 실시간 지연 개선이 아니다. 입력 sample의 값·순서·pixels·calibration/profile·estimator 코드 불변 조건으로 판정.
- live에서 captured-counts를 거부하고 invalid policy/누락 counts를 사전 거부하는 것은 의도하지 않은 live schedule 변경을 막음.

### R 수락 사례

| 사례 | 요구 결과 |
| --- | --- |
| captured count가 존재하는 raw default | future-bracket 선택; source count metadata를 변경하지 않음 |
| explicit captured-counts | 원래 prefix-count delivery 및 not-live-schedule 표기; 기존50 missing 실패 진단 재현 |
| counts 없음/invalid enum/live captured-counts | browser 실행 전 명시 실패; 원본/기존 output 보존 |
| frame와 IMU 동일 timestamp·직전/직후 float 경계 | 기존 helper의 endpoint 정책에 맞는 독립 기대 batch; array slice/중복/누락 오류 없음 |
| 연속 frame가 이미 보낸 future sample을 공유 | 필요한 zero-size batch 허용; sample 재전송0 |
| 고정392-image 실제 frozen 입력 | 기본347fresh/6current-support failure/0missing; explicit diagnostic299fresh/50missing을 별도 기록 |
| frozen future-bracket control 비교 | sourceIndex, input/pose timestamp, status/reason, fresh/valid, pose16값, engineEpoch exact 일치; wall timing exact 요구 없음 |

- 이전 read-only 검토에서 마지막 identity 필드9종을 원본392행 전체에 직접 대조하여 차이0 확인. 단순 fresh 개수 일치보다 강한 기준이 이미 존재한다.

## 심화 바이너리 근거와 적용 한계

- [8th Wall 심화](2026-10-14-8thwall-deep-runtime.md) 및 `residuals-v2/result.json` 확인: 79조건,3 residual, non-export section 불변, trap import0.
- Snavely가 negative/zero depth에서 boolean1을 반환하고 nonfinite residual도 생성하는 것은 raw residual의 domain 계약 관측이다. 전체 vendor solver의 잘못된 point admission을 증명하지 않는다.
- current positive-depth·finite/rank/current-support gate를 완화할 이유가 없으며 U/R은 그 경계에 손대지 않는다. Scale3d scalar·responsive unit·metric 초기화도 분리 유지.
- TranslationStability는 frame-step 위치 두 번째 차분. dt-normalized IMU factor·화면EMA와 동일하지 않으며 가변 dt/drop의 지연·정확도 검증 없이 기본 채택하지 않는 장기계획과 일치.
- [ARCore 심화](2026-10-14-arcore-deep-runtime.md), `arcore/verification.json`, `analysis-final/summary.json` 확인. cached timestamp/state 복사·camera/sensor/display 독립 dispatch·sensor 설정 wait·reference-count release는 narrow static 관측이며 전체 estimator 실행 증거가 아님.
- public frame timestamp의 timebase 미정의 계약은 내부 CLOCK_BOOTTIME 호출 발견으로 바뀌지 않는다. U의 App clock 사용과 R의 명시적 timestamp/bracket 계약 유지가 적절함.
- 내부 constant-value IMU extrapolation branch의 존재는 무제한 extrapolation·미래 timestamp 덮어쓰기·bracket 검사를 제거할 근거가 아님. U/R에는 그런 변경이 없어 sensor 안전 경계를 유지한다.
- ARCore 역순 삽입 거부 `0x8fa000`의 buffer 데이터 전체를 IMU로 특정하지 않는 원문 한계를 action matrix도 유지해야 한다. root가 `역순 buffer 데이터 거부·IMU 초기시각 검증` 및 종류 미확인 범위로 수정한 것을 확인. 이 attribution 지적도 해결됨.
- release export의 짧은 크기를 no-op으로 오인하지 않는 resource 해석이 맞음. JS temporary hide·history preservation은 reference/VideoFrame 소유권 변경과 분리한다.

## 승인 경계

- U/R의 좁은 결함 교정 외에는 구현 승인하지 않음: 새 smoother, geometry threshold 완화, descriptor/relocalization, C3/D2/D3 default ON, SDK 교체, WASM 배포, 신규 실기기 수집.
- 기존 strict churn280/300·current-support<35·Lost episode<7·reset0, GT·TUM·false-ID gate는 보존. UI stale를 estimator Lost로 변조하거나 정확도 분모에서 제외하면 실패.
- 구현 후 별도 검증 필요: red→green, 기존 browser 의미 assertion, 실제392 raw replay·synthetic App/Worker 실행, 원본/제품 WASM hash 불변. 이 사전 검토는 테스트 완료·phone performance·정확도 개선을 주장하지 않음.

## 최종 review snapshot

- `docs/research/2026-10-14-slam-improvement-plan.md` SHA256 `8624759dd1d5c69b7f76c40ad6220f92c5192d15619ce65a9ef8532063af22b5`.
- `docs/research/2026-10-14-8thwall-deep-runtime.md` SHA256 `9c37a0aaeebfc1281833b0951644b58825dd752fb14a8fd5c42284c5577dd558`.
- `docs/research/2026-10-14-arcore-deep-runtime.md` SHA256 `e20d904c44e0649b675dfdeef414df757b6f1ad45e73b778b36a87481bb8bcc6`.
- 승인 의미: 위 snapshot의 U/R 계획은 architecture·correctness·sensor/geometry 경계 측면에서 구현 가능. 제품 테스트 완료·성능 개선·배포 승인을 대신하지 않음.
