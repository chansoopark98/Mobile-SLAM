# 저특징 IMU 예측·바닥 후보 계획: 독립 Architect 검토 v1

- 날짜: 2026-10-14. 판정: **REQUEST CHANGES — 아래 B1/B2/B3 반영 후 재검토**.
- 고정 검토본: `build/low-feature-20261014/plan-v1.md`, SHA256 `529b165709e1c70b9942e149516a5601f64da056ab150532151a9a0ecf26b7bc`. `docs/fixes/2026-10-14-low-feature-plan.md`와 byte hash 동일.
- 범위: 계획·reference 분석·현재 C++/Worker/Wrapper/App·시험 fixture의 읽기 전용 검토. 이 문서만 신규 작성. 제품 코드/고정 계획/기존 dirty 파일/서비스 변경 없음. Critic 결과를 받거나 전달하지 않음.
- 결론: 독립 shadow state와 geometric plane consumer를 분리하는 선택 B는 타당. 현 계획은 **관측 가능한 metric 추적을 회복하는 알고리즘이 아니라, 짧은 관성 예측 표시와 현재 지도점의 수평면 후보 표시**를 구현한다. 이 구분을 그대로 유지하면서 수명 경계와 통합 수락 조건을 보완하면 구현 가능.

## B1 — 실제 IMU gap과 보간된 endpoint dt를 구분해야 함

- 계획 P의 “연속 sample gap≤0.1s”와 “이미 보간된 feedIMU 구간 재사용”만으로는 실제 sensor gap 상한을 보장하지 못함.
- 근거: [feedIMU](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:175)는 `timestamp-current_time_`만 받음. [processIMUData](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/vio_engine.cpp:222)는 future raw sample을 보존하고 image timestamp마다 endpoint를 보간한 뒤 `current_time_`을 전진시킴.
- 반례: raw IMU t=0.00,0.20이고 image t=0.05,0.10,0.15,0.20이면 각 shadow 적분 dt는0.05. 실제 raw gap0.20을 정책이 거부해야 하지만 단순 `feedIMU dt≤0.1`은 모두 허용. 기존 endpoint를 기준으로 남은 right-bracket span만 재검사해도 이미 분할된 gap의 provenance가 사라질 수 있음.
- 필수 수정: predictor 전용으로 **원본 인접 raw sample timestamp 및 해당 interpolation의 원래 left/right bracket**을 유지·검사. source gap>0.1s인 구간과 그 구간 안의 보간 endpoint는 seed/전파에 사용하지 않음. queue에 먼 미래 gap이 있다는 이유만으로 그 이전 정상 구간을 소급 폐기할 필요는 없으나, gap을 가로지르는 첫 prediction 전에는 반드시 폐기. metric backend의 기존0.5s 정책·반환값·epoch는 변경하지 않음.
- 수락: 같은 raw0.20s gap을 camera endpoint0/1/여러 개로 분할해도 모두 prediction 없음; raw gap0.1s±1e−9 경계, carried future sample, anchor가 gap 중간인 경우, 다음 clean anchor 재시작을 native+실제 WASM에서 검사.

## B2 — Worker에서 건너뛴 frame의 폐기가 core seed까지 전달되어야 함

- 계획은 missing bracket 이후 shadow를 폐기한다고 하지만 신규 core API는 getter만 정의. Worker의 null 출력만으로는 이 불변식을 구현할 수 없음.
- 근거: [Worker frame branch](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/web/js/vio-worker.js:598)는 bracket 실패 때 `engine.processFrame()`을 호출하지 않고 `emptyResult('missing_bracket')`를 반환. 따라서 core는 해당 실패를 알지 못하고 마지막 clean anchor를 유지할 수 있음. 다음 blank frame에 IMU가 도착하면 같은0.5s horizon 안의 오래된 seed로 prediction이 재개될 수 있음.
- 필수 수정: 기존 metric 상태를 reset하지 않는 predictor-only invalidation API를 core/binding에 추가하고, Worker의 missing bracket·validation failure·해당 lifecycle의 processing exception에서 호출. 또는 Worker의 sticky quarantine이 **해당 실패 이후의 새로운 clean visual anchor**를 인증하기 전까지 재개를 막도록 정확히 정의. core의 “shadow 폐기”까지 보장하려면 전자가 단순함. null packet 하나를 보내는 것과 seed invalidation을 구분.
- 취소된 과거 request가 새 epoch의 seed를 폐기하지 않도록 호출 전 client/engine epoch identity 확인. Wrapper timeout/transport failure는 현재 `_invalidate`·새 client epoch 경로와 함께 고정.
- 수락: clean anchor→Worker missing_bracket(엔진 호출0)→IMU 정상+blank는 계속 prediction 없음; 이후 clean visual→다음 blank만 재개. seed 폐기 전후 metric pose/status/reason/epoch/prior는 baseline과 동일. old request cancellation이 새 seed를 제거하지 않는 음성 시험 포함.

## B3 — floor 기능에 영상부터 표시까지의 양성 통합 수락을 추가해야 함

- 계획의 명시적 end-to-end gate는 blank 구간 prediction을 다룸. floor는 직접 pointcloud unit과 getter/renderer의 분리 계약만으로도 통과할 수 있어, 실제 이미지→admitted current points→WASM export→plane fit→renderer 경로가 항상 null인 구현을 막지 못함.
- 필수 수정: 독립 floor geometry/GT가 있는 floor-visible 영상 fixture 최소1개를 고정하고 실제 신규 WASM/Worker를 거쳐 **current-support point count>0 및 accepted plane>0**을 입증. plane normal/offset은 engine gauge와 GT gauge를 명시적으로 맞춘 뒤 평가. pixel/IMU/GT/profile/source/artifact hash 보존. 같은 경로에서 blank 전환 시 sourceTimestamp 갱신0·retained 표시·1초 만료·epoch 폐기도 확인.
- 검토 중 발견한 `build/low-feature-20261014/fixtures/floor-map-analysis.log`에는 `floor-visible-clean101`의 기존 exported-map 분석이 있음. 이는 시험 가능성의 근거이나, 아직 없는 신규 **current-only getter와 최종 제품 fitter/renderer**의 통합 증거가 아님. 기존 map 전체를 신규 getter oracle로 대신하지 않음.
- 실제 phone/현장 정확도까지 이 gate로 확대하지 않음. 독립적인 가상 영상 통합 수락이면 이번 구현 검증에 충분.

## 구현 계약에 명시할 보완사항

- **Seed API/소유권:** [Estimator::g_](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/include/backend/estimator.h:86)는 private. P lane에 `include/backend/estimator.h`의 읽기 전용 gravity copy getter 또는 accepted snapshot getter 추가를 명시. estimator 수학 변경은 불필요. [getLatestVINSResult](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:503)는 slide 후 WINDOW_SIZE−1을 읽으므로 `result.timestamp==image_timestamp==current_time_`, same generation/epoch를 seed에서 실제 assert/gate. raw previous acc/gyro도 그 endpoint의 값이어야 함.
- **Gravity 부호:** 엔진의 전파식은 `a_world=R*(a_meas−Ba)−g_estimator`, 초기화된 `g_estimator`는 +Z 방향. fixture `imu-analytic-cases.json`의 `gravityWorld=[0,0,−9.81]`는 물리 중력이며 `a_world=R*f+g_physical` 관례. oracle loader에 `g_estimator=−g_physical` 변환을 명시해야 stationary 시험에서 부호를 뒤집지 않음. state 이름을 둘 다 `gravityWorld`로 묵시적으로 재사용하지 않음.
- **SO(3) 및 수치 모델:** 별도 shadow에서 exact exponential을 쓰는 선택은 허용. 기존 backend의 normalized first-order `deltaQ`와 predictor가 수치적으로 동일하다는 주장은 금지. constant angular rate 독립 oracle 및 일반 변동 회전의 convergence error를 분리. affine world acceleration moment는 endpoint world acceleration을 선형으로 보는 이산 모델이며, 회전 중 실제 연속 specific force를 exact하게 적분했다는 뜻이 아님.
- **실패 quality:** [solveOdometry](/home/park-ubuntu/park/SOLUTION/Mobile-SLAM/src/backend/estimator.cpp:395)는 optimize 실패 때도 별도 `qualityReason`에 motion/bias 경고를 넣음. Lost의 `last_reason_`에는 이 경고가 붙지 않을 수 있음. fallback whitelist 외에 `last_solver_quality_reason_.empty()`를 요구하고 경고가 있으면 shadow 폐기. `insufficient_final_current_features`+`relative_translation_threshold` 조합을 별도 회귀로 고정. previsual shadow를 쓰므로 실패 solver rollback state를 seed로 삼을 필요는 없음.
- **Current point provenance:** `getSolvedFeatures()`의 back observation이 현재 프레임인지 ID 교집합만으로 추정하지 말고, 실제 finite `image_data`에 제출된 ID와 매칭하는 최소 경계를 권고. tracker ID는 있으나 current observation이 nonfinite라 backend에 제출되지 않은 경우를 거부하는 시험 필요. getter packet은 exact request/input/engine frame timestamp와 epoch로 감싸고, old WASM의 일반 map fallback은 만들지 않음.
- **Plane state 소유자:** plane fitting/retention을 어느 계층이 소유하는지 확정. Worker에서 current points를 읽는다면 거기서 최대5Hz fitting/retention 후 결과를 보내고 Wrapper가 검증하는 구조가 자연스러움. App RAF는 sourceTimestamp 기준 expiry만 재검사. 두 계층이 별도 TTL·fit state를 보유하지 않음.
- **5Hz 사이 상태:** 재검출을 건너뛴 frame에서 sourceTimestamp 유지; 최신 visual result라는 이유로 plane 나이를 갱신하지 않음. 새 visual point set이 plane gate를 실패할 때 기존 plane을 제거할지 retained로 유지할지도 단일 정책으로 고정. 4corner mesh는 support extent를 표현하며 무한 ground나 semantic floor label로 확대하지 않음.
- **표시/측정 분리:** prediction anchor horizon과 display source age를 매 RAF 독립 검사하는 선택은 적절. 원래 visual trajectory append0, GT scorer 포함0, floor/PnP/prior feedback0을 유지. 별도 prediction GT report는 preblank에 고정한 alignment와 actual compared sample count를 사용. 100ms display cap 때문에 WASM에서 available해도 표시0일 수 있으므로 두 수치를 분리.

## 가장 강한 반론·tradeoff·종합

- **강한 반론:** 현재 사용자 문제는 고속/무특징 환경의 추적 단절인데, 이 계획은 visual support를 회복하지 않음. 초기 velocity/bias가 잘못되면0.5초 shadow도 잘못된 위치를 그릴 수 있고, geometric horizontal surface는 table일 수 있음. 따라서 제품이 단순히 계속 움직인다는 이유로 “Lost 해결·바닥 인식 정확도 개선”을 주장하면 계획의 핵심 경계를 위반.
- **Tradeoff:** B는 backend 오염을 막고 짧은 표시 연속성을 제공하지만, 별도 적분과 lifecycle 계약/시험 비용이 필요. orientation-only 표시보다 위치 drift 위험이 크고, optimizer에 floor factor를 넣는 C보다 추적 회복 능력이 작음. 반면 C는 높이·독립 관측성·정보상관을 입증하지 못한 현 상태에서 더 큰 오류를 만들 수 있음.
- **종합:** 선택 B 유지. 원래 metric Lost를 유지하면서 사용자에게 관성 예측/position uncertain과 수평면 후보/분류 미확인을 명시하는 범위는 실행 가능. B1/B2로 shadow 계보와 raw timing을 닫고 B3로 기능의 실제 연결을 확인한 뒤 구현. field tracking/semantic floor는 후속 독립 수락 대상으로 유지.

## 검증 범위

- 실행한 확인: `git status --short`, 고정 계획2개 SHA256 비교, C++/bindings/Worker/Wrapper/App 및 관련 분석·fixture의 targeted read.
- 새 알고리즘이나 신규 API는 아직 구현 전이므로 unit/native/WASM/phone 실행 PASS를 주장하지 않음. 검토 목적의 서비스 시작·중지0.
- 변경 파일: 이 검토 문서1개. 기존 dirty 변경·고정 계획 보존.
