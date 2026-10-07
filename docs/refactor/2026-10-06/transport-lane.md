# Transport lane 수정·회귀 계획

- 소유: `web/js/vio-wrapper.js`, `web/js/vio-worker.js`, `tests/browser/contracts.test.mjs`, 신규 `tests/browser/transport.test.mjs`
- 기준: 기존 dirty 파일 보존; engine/binding·live app·IMU/camera 파일은 해당 owner만 수정
- 중복: RPC별 단일 pending 슬롯 → request ID로 조회하는 한 helper; 단일 waiter 슬롯 → 모든 waiter의 개별 deadline·정리
- 경계: 모든 요청/응답 `requestId`, `clientEpoch`, `sequence` 보존; reset/configure/load/dispose의 이전 요청·캐시 정리; 늦은 응답은 busy·캐시 수정 금지
- 시간: frame 입력 `inputTimestamp`/`timestamp`와 실제 engine `poseTimestamp` 분리; 빈 pose의 timestamp는 null; queue/processing/roundtrip은 관측 가능한 clock으로만 기록
- IMU: 선언 count·view 범위·finite·단조 증가 검증, 정확한 subarray bytes, overflow read cursor bounded, stale/overflow/invalid/order/capacity drop 별도 count
- 소유: Worker에서 미래 bracket 최대 하나를 한 번 소비; Engine만 future carry 보존. Worker 단독 ring 테스트는 적분 정확도 근거 제외
- 출력: 내부 engine epoch·tracking loss·map0에서 pose/map 무효화; error/noIMU/invalid 이미지도 ID·frame time·이유 보존
- loader: 실제 module Worker consumer 기준 ES module 로딩; 생성 classic artifact 삭제는 범위 제외
- 순서: 기존 Node baseline → TODO5 strict red → correlation/race/ring 회귀 → wrapper pending 정리 → worker validation/state 결과 정리 → 전체 green·syntax
- 명령: `node --test tests/browser/contracts.test.mjs tests/browser/transport.test.mjs`; `node --check web/js/vio-wrapper.js`; `node --check web/js/vio-worker.js`; `git diff --check`
- 증거: `build/refactor-evidence/transport-{baseline,strict-red,green}.log`; physical phone·fresh WASM·engine 적분 oracle는 상위 통합 gate
- 의존성: 신규 runtime dependency 없음; 새 public API는 additive `getMetrics()`; 기존 API return 종류 유지

## 실행 결과

- 기존 baseline: 4 pass / 5 TODO; TODO 제거 strict red: 4 pass / 5 fail / 0 TODO
- 최종: `node --test tests/browser/contracts.test.mjs tests/browser/transport.test.mjs` → 49 pass / 0 fail / 0 skip / 0 TODO
- heap 경계 red: 기존 helper에서 신규 bounds assertion 실패; 최신 heap view·read/write bounds 수정 후 green
- syntax: 소유 JS2개·Node test2개 `node --check` 통과; 소유 파일 `git diff --check` 통과. ESLint·TypeScript typecheck는 구성 없음
- 전체 `git diff --check`: 기존 `wasm/libs/ceres_build/**/progress.make` 공백 오류 존재; 소유 밖 생성 파일 보존
- 추가 Worker 실행: 실제 `worker_threads`에서 제품 Worker module dispatch·controlled engine fixture 로딩, exception, async init/reset race, default-less artifact 거부 검증. 실제 browser·fresh WASM estimator 검증과 구분
- 단순화: RPC6종 pending 슬롯 → request table1개; waiter 전체 Set; 반복 response helper1개; 무효 importScripts fallback·중복 frame-gap reset·noIMU 선제 skip·반복 frame 진단 상태 제거
- IMU 통계: `received`, `accepted`, `overflow`, `stale`, `capacity`, `invalid`, `order`, `reset`, `invalidBatches`, `gapCount`, `lastGapSeconds`; Worker 수명 누적, reset/configure에서 loss 통계 유지
- frame 통계: `framesSubmitted`, `framesCompleted`, `framesDropped`, `framesTimedOut`, `framesCancelled`, `frameDropReasons`; ACK 대기 reset은 frame gate, 오류·교체·종료는 모든 pending settle
- 결과: `requestId/clientEpoch/sequence`, `engineEpoch`, `inputTimestamp/timestamp`, 실제 `poseTimestamp`, `poseValid/poseFresh`, `reason`, `poseFrame=camera`, solver 진단, `frameCopyMs`, `workerQueueMs`, `workerProcessingMs`, `engineProcessingMs`, `roundtripMs`, `poseAgeSeconds`
- timing: queue/roundtrip은 양쪽 `performance.timeOrigin + performance.now()` 차이; processing/copy는 동일 surface monotonic duration. `poseAgeSeconds`는 입력과 pose sensor 시간 차이이며 capture-to-display 지연과 구분
- 제한: engine 자체 미래 bracket 적분 oracle·fresh native/WASM·실 browser/HTTPS·calibrated replay·field 정확도는 각 owner/leader 통합 gate. Worker fixture의 pose는 정확도 증거 제외
- live 추가 계약: raw 마지막 IMU timestamp가 frame보다 앞선 경우 최대40ms bracket 대기; 독립 IMU message에서 wake. deadline에는 `missing_bracket`·null pose, reset/configure/dispose/new epoch에서 wait 취소·이전 응답 무효화
- `imuWaitMs` 별도 기록; `workerProcessingMs`에서 bracket 대기 제외. prepared dataset future bracket·engine carry가 구간을 덮으면 wait0. actual Worker late bracket/deadline/reset-during-wait3개 회귀 추가
- 실행 재현: configure 뒤 `setExecutionParams(0,1)` additive API가 있는 새 엔진에 적용, `executionSeed`·실제 `cvThreads` 출력. optional getter 부재는 null로 표시
- unique IMU loss: overflow/stale/capacity 손실의 count·timestamp min/max·원인 latch; affected frame 직전 engine reset1회, ordered recent/future는 새 초기화에만 사용. 결과 `imu_transport_loss`·null pose/map/freshfalse; duplicate/order rejection만으로 reset 금지
- endpoint 관측: 실제 `getIMUEndpointTimestamp()`를 reset 전에 기록; seeded endpoint disconnected=true, getter -1은 endpoint 없음, getter 부재는 null/`unavailable`. frame 입력 timestamp로 적분 endpoint 추정 금지
- unique-loss 회귀: 700sample/0.3495s·capacity188 손실의 기존 false tracking fixture red2건 → green; actual Worker established pose 이후 capacity loss·epoch/new null result까지 검증. 기존 future/bracket wait/cancel 모두 유지
- allocation 시 heap growth: `_malloc` 이전에 F64/U8 type 분류. allocation이 heap view identity를 바꿔 F64를 U8로 처리하던 독립 fixture red1건 → green; source line 이동으로 수정
