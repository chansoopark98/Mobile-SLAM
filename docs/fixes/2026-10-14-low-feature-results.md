# 저특징 IMU 예측·바닥 후보 구현 결과

- 상태: 구현·독립 코드 리뷰 통과. **body 기준 floor GT71/79=89.87%로90% gate 미달, 서버 파일 승격 보류**. 평면 재추정의 원인을 추가 검증 중.
- 범위: 마지막 정상 시각 추적 이후 짧은 IMU 예측과 현재 영상의 유효 3D점 기반 수평면 후보. 기존 SLAM optimizer·feature tracker·지도·metric pose 정책 변경 없음.
- 설계: [승인 계획](2026-10-14-low-feature-plan.md), [reference 비교](../research/2026-10-14-low-feature-references.md), [core 분석](../research/2026-10-14-low-feature-core-analysis.md), [브라우저 분석](../research/2026-10-14-low-feature-browser-analysis.md).

## Reference 반영

| 확인한 근거 | 구현 결정 |
| --- | --- |
| 8th Wall 공개 source의 assumed ground/initial depth와 detected/estimated hit 구분 | 원점 grid·고정 카메라 높이를 검출된 바닥으로 사용하지 않음. 실제 현재 관측점의 지지 범위만 후보 표시 |
| ARCore 실제 ARM64 바이너리의 IMU 초기시간·마지막 IMU 이후 query 경계 | 원본 sample gap·frame timestamp·epoch 확인. bracket 실패 후 과거 anchor 재사용 금지 |
| ARCore plane 종류, ARKit horizontal alignment와 별도 classification | 수평면과 semantic floor를 구분. 화면에 `바닥 후보(수평면, 분류 미확인)` 표시 |
| ARKit limited 상태에서 plane 갱신 제한 | Lost에서 새 평면 fitting 중단. 이전 후보는 원래 source timestamp로 최대1초 보관 |

- ARKit engine binary·private source는 확보하지 못했으며 Apple 공식 문서만 확인. ARKit 내부 알고리즘을 역분석·복제했다는 주장은 없음.
- 새 `.5초/.1초/5m`, plane gate는 우리의 engineering policy. vendor의 비공개 threshold를 복원한 수치가 아님.

## 제품 변경

- `include/utility/inertial_prediction.h`: 마지막 clean backend state의 독립 P/R/V·bias·gravity·extrinsic 복사, SO(3) gyro 전파와 affine acceleration 적분.
- `src/vio_engine.cpp`, `include/vio_engine.h`, `include/backend/estimator.h`, `wasm/vio_bindings.cpp`: strict seed·원본 IMU gap 검사, shadow invalidate/JSON getter, 현재 finite 전체 observation과 solved/admitted point 교집합 최대512점 export.
- `web/js/low-feature.mjs`: 공유 schema/pose/epoch 검사, 최대64회 RANSAC·centered LS, 수평면 support·spatial spread·경쟁 평면 거부, Worker 소유5Hz/1초 수명.
- `web/js/vio-worker.js`, `vio-wrapper.js`: 새 metadata 전달, missing bracket/잘못된 입력/전송 오류 시 무효화, 처리 중 과거 응답 차단 및 새 clean visual 이후 회복.
- `web/js/app.js`, `renderer.js`: amber IMU frustum, 분류 미확인 수평면 mesh, retained 색상 구분. 매 RAF 예측100ms·평면1000ms source age 검사. 예측은 trajectory/map/follow-view pose에 기록하지 않음.
- `scripts/dev/{synthetic-browser,replay-android-input,android-input-record,android-snapshot,wasm-smoke}.mjs` 및 관련 tests: 별도 통계·metadata·ABI 검사. 기존 GT population은 visual pose 그대로 유지.
- 단순화: Worker/Wrapper가 같은 pure validator를 사용하고 평면 상태는 Worker 한 곳에서 소유. 새 의존성 없음. 기존 U/R의 visual500ms·중복 상태10Hz·future-bracket 기본값 유지.

## 재현·검증

- 증거 root: `build/low-feature-20261014/`. 승인 고정안 SHA256 `ce6e91442165d9ba5e93258958963a2b24dc262d177693bcc06418e687c46108`.
- 새 WASM: `wasm/vio_engine.js`는 이 증거 root 아래 별도 build의 파일. SHA256 `e7420b3699d075ad7123e32a0e8a5039a28e6a4008abda2d13e940b84b9ce93b`,6,056,932bytes, embedded single-file.
- source manifest SHA256 `2780bae16ca788895bb9d60fb19350dd5c5da908cdaf9ac06ccbecd7897f7448`; build 전후85 source·2,660 dependency 파일 hash 변경0. `wasm-evidence/{prebuild,provenance,build-receipt,smoke}.json`.
- 전체 browser **349/349 PASS**, skip/TODO0. 별도 trajectory exporter **3/3 PASS**. `browser-final.log`.
- JS/shell/Python syntax PASS, `git diff --check` PASS. JS 프로젝트에는 별도 TypeScript/ESLint 설정 없음. clangd23.1.0 실제 C++ compile DB 검사 error0/symbol39, `clangd-probe.json`.
- WASM 신규4개 ABI·미초기화·heap range/alignment/cap·invalidate epoch 보존 검사 실제 Chrome PASS. 초기화/정확도 증거는 아래 E2E와 구분.
- 실제 Renderer known-axis reference 이미지와 pixel difference0. prediction의 궤적·지도·관찰 카메라 비간섭, retained opacity·reset 확인. `ui/renderer-check.json`, `ui/visual-verdict.json`.

| 실제 Worker/WASM 제어 실험 | 결과 |
| --- | --- |
| 기존388/new e742,6개 조건×400frame | 시각 pose·status·reason·epoch·solver·current factors·기존 map XYZ/count **2,400frame exact**. pipeline timer만 제외 |
| floor-visible control | visual357/400, floor observed79/retained254/present333. observed GT normal≤3°·offset≤.05m **71/79=89.87%, FAIL** |
| 0.25초 blank floor | 예측5/5. anchor-relative GT 오차 최대 .005683m/.013515° |
| 2초 blank floor | 최초 .5초 예측10/10, 최대 .013177m/.027026°. 이후30frame 예측0 |
| blank floor 보관 | 원래 sourceTimestamp6.95 유지, 새 fit0,1초 초과 제거 |

- floor GT gauge: 기존388 control의357개 visual camera pose를 calibration T_B_C로 body pose로 변환 후 SE(3),scale1 정합. 동일 변환을 모든 floor arm에 적용하며 plane 자체에 맞춰 재정합하지 않음.
- 실패 보존: control 평면8회 normal 기준 초과. 카메라 pose로 계산한 예비72/79는 body pose 변환 후71/79로 정정; 두 원시 결과 보존. blank 전환 이후 평면 GT 통과율78.67%/71.23%, 해당 궤적 GT FAIL. 기존 fast707의3개 arm도 GT FAIL. 예측/retained plane을 visual coverage에 합산하지 않음.
- 위 예측 수치는 합성 입력의 단기 anchor-relative 오차. 절대위치·물리폰 정확도·장기 IMU-only SLAM 보증 아님.

## 남은 한계

- 고속 이동의 blur·잘못된 correspondence·장기 feature 부족에 따른 metric Lost 자체는 남음. IMU 예측은 마지막 clean anchor부터 최대 .5초이며 이후 숨김.
- 바닥과 탁자의 semantic 구분, IMU 기반 새 바닥/scale 측정, plane factor를 통한 optimizer 보정은 미구현.
- frame 처리 시점의 예측이며 별도60Hz pose producer가 아님. 실제 모바일 capture/thermal/지연·현장 floor GT는 미검증.
- 서버 실행·중지·재시작은 사용자 담당. 이번 작업은 소스·검증된 파일 준비까지 수행.
