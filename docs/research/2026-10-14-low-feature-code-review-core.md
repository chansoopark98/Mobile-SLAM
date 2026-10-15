# 저특징 구현 독립 코드 리뷰 — core·평면 모듈

- 판정: **APPROVE — 검토한 소스 범위에서 차단 결함 없음**.
- 범위: `include/utility/inertial_prediction.h`, `include/vio_engine.h`, `src/vio_engine.cpp`, `include/backend/estimator.h`, `wasm/vio_bindings.cpp`, `web/js/low-feature.mjs`의 제품6파일 및 관련 집중 시험.
- 기준 계획: 고정 plan-v3 SHA256 `ce6e91442165d9ba5e93258958963a2b24dc262d177693bcc06418e687c46108`.
- 독립성: 다른 최종 리뷰 문서를 읽지 않음. 지정 소스·계획·시험을 직접 대조하고 아래 시험을 별도 실행하여 판정.
- 변경: 이 문서와 `build/low-feature-20261014/reviews/core-*` 증거만 추가. 제품·계획·서비스 변경 없음.
- actionable finding: **CRITICAL0 / HIGH0 / MEDIUM0 / LOW0**. 재현된 결함이 없어 수정 요청·inline finding은 없음.

| 확인 경계 | 소스 근거·판정 |
| --- | --- |
| clean seed의 시각·gravity·extrinsic | [finishInertialFrame](../../src/vio_engine.cpp#L76): image/IMU endpoint/seed timestamp·reset generation·실제 공개 camera P/R 일치, clean tracking/usable update, frozen 실제 `getGravity()` 및 extrinsic 복사. PnP/quality warning에서 seed 제거 |
| 원본 IMU gap | [processIMUData](../../src/vio_engine.cpp#L281): raw-left timestamp와 원본 right bracket의 간격을 검사. 보간 endpoint로 큰 gap을 나눠도 coverage=false. 기존 metric gap 정책은 유지 |
| 제한 전파·실패 수명 | [InertialPrediction](../../include/utility/inertial_prediction.h#L8): frozen bias/gravity, SO(3) exponential, affine acceleration time moment, .5초/5m/유효 회전·finite gate. invalid 상태는 새 seed 전까지 전파하지 않음 |
| hard error·configure·reset | [processFrame wrapper](../../src/vio_engine.cpp#L371)에서 모든 결과를 finish gate에 통과시키고 exception에서 clear. 실패 configure/PnP 설정/reset 및 공개 invalidate API도 shadow를 폐기 |
| backend 비간섭 | predictor는 독립 state를 보유하며 estimator/window/optimizer/map에 write-back하지 않음. 공개 getter는 기존 pose/status/map을 변경하지 않음. 집중 시험에서 blank 전파의 두 engine window/bias/preintegration 상태 exact 비교 통과 |
| 현재 point support | [getGroundSupportPoints](../../src/vio_engine.cpp#L124): clean visual·유효 gravity/+Z 정합·현재 tracker ID/track count/전체 observation finite와 기존 admitted/nonpending/solved/non-outlier feature 교집합. 최대512점, ID 중복 억제 |
| WASM 출력 범위 | [getGroundSupportPoints_wrapper](../../wasm/vio_bindings.cpp#L63): positive capacity를512로 제한한 뒤 heap range/alignment 검사. unavailable prediction은 native 문자열 `"null"`로 일관됨 |
| prediction protocol | [validateInertialPrediction](../../web/js/low-feature.mjs#L33): 정확한 schema/키 집합·Lost whitelist·quality·epoch·frame timestamp·source horizon·homogeneous SO(3) 검사. metric/visual support로 재분류하지 않음 |
| 평면·거짓 floor 방지 | [fit](../../web/js/low-feature.mjs#L101): 최대512 unique points·64 hypotheses, tilt/clearance/support/spread/residual gate, centered refit 후 재검사, 경쟁 plane ambiguity 거부. semantic은 unknown |
| 평면 상태/소유권 | [GroundPlaneTracker](../../web/js/low-feature.mjs#L143): 최대5Hz 새 fit, source timestamp/sequence 유지, Lost에서 새 fitting 없음,1초 expiry 및 quality/epoch/error clear. 반환 배열을 복사하여 외부 mutation이 보관 상태를 바꾸지 않음 |
| transport 회복용 clean helper | [isCleanVisualContext](../../web/js/low-feature.mjs#L28): 기존 SO(3)/clean context에 fresh/valid/exact pose timestamp를 모두 추가 요구. 실패·Lost·잘못된 pose가 정상 visual 회복으로 판정되지 않음 |

- 원본 IMU gap은 shadow publication/seed에 적용되며 기존 metric backend를 재설정하는 새 정책으로 확장되지 않았음.
- floor 출력은 bounded geometry의 후보이며 table/raised-floor semantic 구분을 생성하지 않음. retention은 source timestamp를 바꾸지 않아 새 관측으로 집계되지 않음.
- finite/heap 검토는 현재 binding·Worker의512점 입력 경계를 전제로 수행. 실제 Wasm 경계 실행은 상위 E2E 검증에서 별도 확인 필요.

## 직접 실행한 증거

```bash
build/low-feature-20261014/native/test_engine_contracts \
  --gtest_filter='EngineContracts.Inertial*:EngineContracts.GroundSupport*' \
  --gtest_output=xml:build/low-feature-20261014/reviews/core-native-tests.xml
node --test tests/browser/low-feature.test.mjs
node build/low-feature-20261014/reviews/core-clean-context-probe.mjs
```

- 집중 native **13/13 PASS**, failure/error/disabled0. 원본 gap 보간 반례·failed configure·PnP·quality·seed time/gravity·lever arm·5m/.5초·support cap·비간섭 포함.
- 최종 pure module **28/28 PASS**, fail/skip/TODO0. 평면 positive/negative·중복점·semantic ambiguity·source TTL·epoch·array ownership·SO(3)·신규 clean helper 포함.
- 별도 clean helper probe **14/14 PASS**. 정상1건, stale/future timestamp·fresh/valid false·frame mismatch·warning·Lost·미초기화·transport/error·zero/reflection/shear pose13건.
- 로그: `core-native-tests.log/.xml`, `core-js-tests-final.log`, `core-clean-context-probe.mjs/.json`.
- source/binary provenance: `core-source-before.json`, `core-source-after.json`, `core-source-final.json`. 리뷰 중 root 소유 helper+해당 unit만 추가되어 최종 소스로 재확인·재실행. C++/bindings/계획/native 시험 binary hash는 첫 검토부터 동일.
- 최종 pure module SHA256: `b197b6b9b7a2b2c709a6c2d92ff38aebfad8bda2fea407849e7a59d09984b79e`.

## 검증 한계

- 이 리뷰는 전체 native/browser 회귀·새 sanitizer·실제 fresh WASM ABI/Worker·floor/prediction E2E 수락을 대신하지 않음. 해당 결과가 준비되기 전 전체 제품 완료로 해석하지 않음.
- 집중 native 일부 시험은 알려진 내부 상태를 준비하는 성분/수명 시험. 실제 영상 초기화 후 shadow seed 생성·GT 정확도는 실제 E2E 증거가 필요.
- fixed-work old/new 전체 trajectory·map/prior snapshot 비교, 실제 phone latency/thermal/field GT, semantic floor 인식은 이 리뷰에서 새로 실행·입증하지 않음.
- 구현 단계가 끝난 뒤 root는 계획의 observed≥40/present≥100/GT≥90%, prediction eligible coverage·.25m/5° 및 전체 비간섭 gate를 별도 판정해야 함.
