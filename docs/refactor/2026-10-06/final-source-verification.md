# 최종 소스 검증

- 검토일: 2026-10-06, 사용자 지정 GPT-6.1 Sol / ultra, 추가 Astra 검토 없음
- **소스 안전·lifecycle·total-order·bounded diagnostic 판정: APPROVE — 두 P1 repair와 최초 sorting 분기 수정 재검토 완료**
- **R5 parity 판정: SHORT DIAGNOSTIC PASS / FULL GATE PENDING — 수정 뒤60frame 결과 확인, full room1 3쌍·동결·room4 필요**
- **통합 runtime 판정: 대기**. root가 기록할 R5 full replay·고정 parity·R6 실제7002 JSON/해시/서비스 확인 전 완료 서명 제외
- 검토자 작성 범위: 이 문서만. 제품 source·빌드·서비스·dataset·TLS 변경 없음; profiling 중 테스트/빌드/CPU 작업 실행 제외
- 독립성: 검토자는 numerical lane 작성자. numerical18·결합ASan/UBSan32 결과는 구현 lane 근거로만 인용; 자기 작성 수치 코드의 독립 architect approval로 표현 제외
- 이번 독립 대상: engine/tracker/native adapter, evaluator/config/CMake, Worker/wrapper/mobile, HTTPS/ops/deployment와 공유 인터페이스
- 기준: `build/refactor-baseline/{start-status.txt,start-diff.patch,source-fingerprints.json,original-engine/}`와 현재 dirty 소스. 기존 작업이 포함된 working tree를 Git HEAD만으로 동일시 제외
- 범위/검토 계약: `docs/refactor/2026-10-06/{execution-plan,astra-plan-review}.md`, `.omx/plans/{prd,test-spec}-mobile-slam-refactor-7002.md`

## 발견·수정 기록

| 우선순위 / owner | 근거·실패 경로 | 최소 수정·수락 근거 |
|---|---|---|
| **P1 / Engine** | `src/frontend/feature_tracker.cpp:235`·240: `rejectWithFundamentalMatrix()`의 두 `liftProjective` 결과를 finite/positive optical-Z 검사 없이 나눗셈 후 RANSAC에 전달. `detectAndTrack()`의 `undistortedPoints()` 호출은 RANSAC 뒤218행이라 뒤 guard로 앞 scoring 보호 불가 | F 입력의 두 ray를 나누기 전 유효성 판정, invalid 대응을 같은 mask로 제거 후 충분한 유효 대응만 RANSAC. NaN/zero/negative-Z가 F 입력에 들어가지 않고 정상 geometry 유지하는 독립 fixture; clamp·camera model·manifold 변경 제외 |
| **P1 / Engine, 위 수정과 같은 경계** | 212–218행에서 `prev_pts/prev_undistorted_pts` 저장 후 새 `cur_pts`의 ray guard 실행. 378–381행은 current/next/ID/count만 필터. current invalid ray가 제거되면 retained previous 배열이 다음 LK status보다 길어질 수 있음. 22–35행 `filterByStatus`는 vector 길이만큼 `status[i]`를 무조건 읽어 다음 frame에서 범위 밖 접근 가능 | 유효성 mask 적용 시 해당 대응의 retained parallel vector까지 동기화. 선택 ray만 NaN/0/negative로 반환하는 Camera 경계 fixture → 다음 tracking frame까지 실행, vector/ID/count 대응 유지·ASan 오류0. 새 안전 계층이나 전체 tracker 재작성 제외 |
| **P1 / Transport** | `web/js/vio-worker.js`의 `appendIMU()` overflow, `drainIMUToWasm()` stale/capacity drop은 숫자만 기록. `processFrame()`은 retained batch를 그대로 Engine에 제공. 빠른 timestamp packet에서 unique interval을 일부 버려도 retained 첫 sample의 gap이0.5s 미만이면 Engine이 연결된 구간으로 적분·fresh tracking 반환 가능 | overflow/stale/capacity의 새 interval loss를 flag로 기록, 해당 interval을 정상 preintegration으로 받아들이기 전 engine lifecycle 무효화/명시 reason·epoch/cache 전달. duplicate/order counter와 unique interval loss 구분. sub0.5s high-rate truncation 입력으로 invalid pose+epoch+reason 독립 회귀; 실제 센서 속도·현장 정확도 주장 제외 |
| P2 / Mobile docs·HTTPS namespace | mobile lane profile 예시 `/profiles/device-camera.json`과 mock `/profiles/fixture.json`은 route interception에서 제공. 실제 server 허용 정적 JSON은 `/public/*.json`; `/profiles/`는403. 현재 mock만으로 실제 정적 profile 제공까지 검증 완료 표현 불가 | 예시·fixture를 실제 허용 `/public/...` namespace에 맞추거나 제한된 profile route를 명시 지원. 최종7002에서 실제 공개 file 경로의 GET/HEAD·configure 호출 확인 |

- 위 지적: 정적 control/data flow 근거. 검토자가 profiling 중 새 실행으로 crash/실제 phone 발생 빈도를 재현했다는 주장 제외
- Engine P1 두 행은 하나의 ray/parallel-vector 경계로 묶음. blocker는 Engine 경계와 Transport interval-loss 경계 **2개**
- owner와 root에 전달 완료. source freeze·현재 pilot 기록은 leader가 조정; 검토자가 frozen source를 수정하거나 과거 결과를 새 source 결과로 변경 제외

## Repair 확인

- **Engine CLOSED**: 두 ray의 finite/positive-Z·나눗셈 뒤 float 표현 범위 검사 후 F 입력 구성, `pruneTrackedPoints()`에서 previous/current/next normalized point·velocity·ID·track count 동기화, 짧은 historical vector와 status 경계 보장. invalid point를 clamp하거나 model/manifold 변경하는 방식 제외
- Engine owner 실제 old-fail: ASan previous16/status8 heap-OOB·F invalid ray retained25 대 valid24, `build/refactor-evidence/engine-red/ray-validity/red.log`
- Engine owner 독립 boundary fixture와 다음 frame 포함 fresh instrumented 실행: Engine17+Numerical18 **35/35**, `build/refactor-numerical/sanitize-engine/rayguard-test.log`. Numerical18은 검토자 자기 작성 회귀; Engine의 새 boundary fixture는 다른 owner가 작성·실행
- **Transport CLOSED**: overflow/stale/capacity loss의 count/reason/최초·최종 timestamp latch, retained tail admission 전 engine reset·epoch 기록, affected frame `imu_transport_loss`·null pose/map·fresh=false. 새 raw tail/future는 새 initialization의 입력으로 유지. duplicate/order만으로 reset하는 정책 제외
- Transport owner 실제 sub0.5s fixture:700개/.3495s batch의 capacity188 손실 old-fail→invalid pose/new epoch, strict Node **48/48**. 검토자는 수정 소스의 latch→reset→admission→invalid result 흐름을 재확인; 해당 실행은 owner 근거
- 남은 source blocker **0개**. profile `/public` 문서 경로 교정은 소규모 인계 항목; actual profile static GET/HEAD와 configure의 최종 HTTPS 근거는 R6 검증에서 별도 유지
- **runtime 승인 제외**: superseded room1 pilot은 실제 frontend/pose/solver parity 불일치로 실패 기록 유지. source safety approval이 R5 parity·R6 전체 완료를 대체하지 않음

## Parity 진단 추가 기록

- 대상: superseded `build/refactor-validation/run-2/pilot-1/{native,browser}`. exact raw gray/IMU packet hash·init45·fresh2776 일치와 pose/solver 불일치 구분
- 첫 **관측된** frontend 분기: frame4 tracked count native148/WASM147, frame5 148/147, frame7 146/147. frame4는 Estimator window10 도달 전이라 Ceres/initial alignment 호출 이전. 최초 좌표·ID 분기는 더 이른 frame일 수 있으며 아직 stage trace 미확보
- first pose45의 약0.01786m 차이·frame46 convergence6 대 NO_CONVERGENCE11은 이미 서로 다른 frontend 입력 뒤의 결과. Ceres Schur 옵션 차이만을 최초 원인으로 단정 제외
- static weight 검사: focal=(fx+fy)/2, projection sqrt-info=focal/1.5, 표시 값약127.3172614011366. Native config restore 이전 setParameter와 browser configure에서 같은 calibration/noise 사용; 표시 precision 차이 약1e-13 외의 cached-weight mismatch 미발견
- **확인된 공통 소스의 ordering seam**: `FeatureTracker::setMask()`는 track count만 comparator로 사용하고 이후 greedy spatial suppression. equal-count 순서는 표준에서 고정되지 않아 서로 다른 C++ standard library에서 selection/RANSAC correspondence 순서 차이 가능
- 독립24개 equal-count fixture, 제품 변경 없음: native GCC11/libstdc++ order `12 23 22 ...`, WASM/libc++ order `0 1 2 ...`; nearby group survivors native `[12,23,20,17,0,11,8,5]` 대 WASM `[0,3,6,9,12,15,18,21]`. source/log `build/refactor-diagnostics/sort-order/{sort_order.cpp,native.log,wasm.log}`
- 같은 결과를 실제 record type `pair<int,pair<cv::Point2f,int>>`·20px squared-distance suppression으로 재확인. 각 플랫폼 OpenCV header 사용, library/product object 재빌드 제외. `sort_order_actual_type.cpp`, `native-actual.log`, `wasm-actual.log`
- 위 fixture는 라이브러리별 실제 tie fork를 증명; **actual room1 첫 분기의 단독 원인 증명은 아님**. frame0–5 CLAHE hash→GFTT points→LK coordinates/status→sorted IDs/mask survivors/F 입력과 keyframe 선택을 순서대로 비교 권고
- root/Engine/Build owner에 deterministic secondary ID/pixel ordering·narrow60frame 검증 권고 전달. 자동 source 변경·GT tuning·cap 확대·전체 pilot 재시작 제외
- **actual first fork 확인**: owner의5-frame trace를 독립 재조회. `build/refactor-diagnostic/native-before-total-order/results.json`, `wasm-before-total-order.json`
- frame0 CLAHE/GFTT141/final normalized rays exact. frame1 CLAHE·LK raw141·border status141·mask_input141·base mask image까지 exact; `mask_sorted` 첫 row native `[473.4546203613,353.7262573242,id97,count2]` 대 WASM `[331.8595581055,425.0899658203,id70,count2]`에서 최초 분기
- 그 뒤 mask_output138/138의 ID/좌표·GFTTnew12/12 content가 달라지고 frame4 survivor count148/147로 확대. 이 recording의 최초 파이프라인 분기 원인은 equal-track-count sort order; OpenCV version·Ceres·cached weight를 첫 원인으로 단정하는 가설은 해당 trace와 불일치

## Total-order·진단 최종 확인

- comparator: track count descending → stable ID ascending → finite pixel X/Y 순서. longest-track 우선 유지, 같은 우선순위의 library-dependent permutation 제거. field GT·오차 cap·noise/solver tuning·manifold 변경 없음
- owner 독립 golden: 가까운 equal-priority ID5/2 중2 선택, 더 오래 tracked ID9는 nearby ID1보다 우선. `tests/test_engine_contracts.cpp`의 `EqualPriorityMaskUsesStableIdsWithoutChangingLongerTrackPriority`
- diagnostic: Engine/Tracker의 `diagnostic_capture_=false` 기본값, 비활성 getter는 `{"enabled":false}`. 활성화 때만 image hash·point/mask stage 처리, 매 frame 이전 문자열 교체, reset/disable에서 cache 제거
- 각 point/status stage 최대1000개, camera parameter snapshot16개 상한, 원래 count와 제한된 rows 구분. 이미지 입력은 기존 engine 크기/총 pixel bound, 원시 image payload 대신 명시된 `fnv1a64` hash. 원래 input SHA-256·build provenance와 혼동 제외
- execution profile: active 요청1은 실제 getter1, reset에 seed/thread profile 재적용, explicit profile destructor에서 scheduler0 해제. global OpenCV config/thread control·한 synchronous engine 전제 유지; 독립 concurrent profile 지원·LSan suppression 제안 제외
- 검토자는 해당 diff·60개 실제 diagnostic rows를 read-only로 재조회; runtime test/build·source 수정 실행 제외. final SAN/changed-files cleanup·배포 source hash는 owner/leader 별도 기록
- 수정 조건: total-order + Native Ceres Schur ON의 consistency alignment. 같은 입력60frame의 frontend 모든 stage 차이0을 독립 확인, init45·fresh15/15·state/reason/solver/relative epoch 차이0
- `build/refactor-diagnostic/after-total-order-60-comparison.json`: translation p95 **4.749270733680702e-10m**, max **4.807429385971528e-10m**; rotation max **3.818191820832846e-6°**
- 이60개 결과는 이전 first-fork와 초기 backend 회귀의 closure 근거. full room1/room4·실제 Worker 전체 replay·장기 performance·독립 phone/field accuracy의 완료 근거로 확대 제외

## 확인된 소스 연결

| 경계 | 정적 확인·판정 |
|---|---|
| Calibration / pose | `configure()`가 alias 입력을 먼저 소유 값으로 복사, finite/focal/SO(3)/det+1 검증. `T_W_C=T_W_B·T_B_C`와 row-major4×4·body lever arm 유지. invalid configure 뒤 Worker configured=false로 frame 차단 |
| IMU / clock | Engine 한 곳에서 future queue·원래 right bracket·보간 endpoint 소유. 신규0 batch는 이미 가진 future bracket까지만 처리; 미래 bracket 없는 구간에서 ZOH 이동 제외. duplicate/out-of-order 검사는 cursor/queue 변경 전. Worker40ms 기다림이 sensor source time을 생성하지 않음 |
| Freshness / lifecycle | frame 진입 시 pose/solver 표시 무효화, current estimator/PnP usability만 publish. blank feature는 성공 pose 반환 제외. estimator generation 변화에서 engine epoch·tracker·IMU carry·PnP/map cache 재설정. Worker clientEpoch+sequence, wrapper requestId+epoch+sequence 연관, stale reply는 busy/cache 갱신 제외 |
| Solver / covariance | numerical lane에서 `IsSolutionUsable()`·finite cost/parameter·현재 관측과 factor coverage 확인, budget `NO_CONVERGENCE` 허용. invalid IMU whitening은 optimization/marginalization 진입 제외. prior 해제·PnP RAII·feature bool 초기화는 구현 회귀 근거. formal ambient manifold/Minus 전환 완료 주장 제외 |
| Execution profile | explicit seed/thread profile에서 active OpenCV threads1, destructor의 `setNumThreads(0)` scheduler 해제. 실제 LSan 근거를 옵션 억제·미검증 workaround로 바꾸는 제안 제외 |
| Native adapter | `VIOSystem`은 raw decoded image+ordered IMU만 공통 `VIOEngine`에 전달. result logger/viewer는 camera pose, body viewer는 동일 extrinsic inverse. 최초 raw packet에 seed+future bracket 한 번, 다음 packet은 이미 보낸 미래 bracket 중복 전달 제외. camera/IMU ns 변환은 browser와 같은 `Number(ns)*1e-9` |
| Evaluation | C++ metric SE(3) scale1 기본·Sim(3) 진단 분리, matched time/pose·one-to-one GT, 실제 SE(3) rotation RPE, 빈 결과 invalid/NaN·save 거부. 독립 Python scorer의 body 변환·translation/rotation ATE/RPE·coverage 사용; C++ translation ATE만으로 rotation ATE까지 측정 완료 표현 제외 |
| Build / parity | native/WASM `cmake/SLAMCoreSources.cmake` 공통 목록, headless viewer 선택, 실제 core SIMD/no-fast-math·undefined-symbol 요구. OpenCV native4.5.4/WASM4.5.5 차이 공개. generated 파일 존재·link 성공만으로 새 artifact served 주장 제외 |
| Masks / inputs | 선택된 TUM YAML의 `fisheye=1`, 빈 external mask path에서 양 경로 같은95% generated circle. custom file mask는 native 전용이라 일반 custom-mask parity로 확대 제외. runner는 실제 gray/Float64 IMU byte hash·K/distortion/extrinsic/noise/solver/tracker 조건과 frame counts 비교 |
| Parity / timing | room1 full2,821 frame3쌍+within-platform 반복 후 contract 동결, room4 full2,228 frame 전 해시 재확인. 사전 translation p95≤0.001m/max≤0.01m·rotation p95≤0.01°/max≤0.1°와 state/reason/solver/epoch/coverage exact 요구. 결과 기반 cap 확대 제외. paced20Hz와 max-speed·hash 시간 별도 |
| Mobile | Start activation 안에서 native permission 호출, Generic accel/gyro source timestamp·pair skew 정책과 DeviceMotion origin/단위/축/중력 provenance 명시. video presentation/canvas arrival proxy를 exposure로 표기 제외. supplied profile은 drawImage domain·orientation·crop·pixel-center resize·tangential distortion·`R_B_Cnew=R_B_Cold Qᵀ`·lever arm 보존; fallback은 heuristic_unverified |
| HTTPS static / TLS | GET/HEAD 전용·remote log405, 허용 extension/path·decoded traversal/dot/private/outside symlink 차단, 열린 FD realpath 재검사. leaf/key/SAN/current validity/ordered intermediates/system trust, root 제외, 자기서명 fallback 없음. dataset은 local room1/room4 map과 checkout boundary로 제한 |
| Ops / deployment | start7002 authorization은 실제 deployed JS hash·single-file embedded flag·external WASM null 검증. embedded mode의 legacy external `.wasm`·alias 차단. active loader 원자적 승격과 이전 file backup. 정확한 PID/start ticks/uid/exe/cwd/argv·trusted health instance 확인 후 소유 종료,1MiB 로그 뒤 drain/discard·기존 로그 보존·자동 restart 없음 |

## Runtime 완료 조건

- 현재 문서: 소스 판정과 필요한 수정 근거. R0~R6 전체 완료 기록으로 사용 제외
- Engine/Transport blocker 수정 시 affected regression·sanitizer·source manifest·fresh artifact·served hash의 새 근거 필요. 기존 pilot은 기존 source/build와 함께 보존; 새 source 결과로 재명명 제외
- R5: 같은 실제 입력으로 room1 pilot3쌍·동결 contract·room4, nonzero initialization/fresh pose/GT association, 전체 분모·reset/reason·metric scale1 정확도와 paced count/latency 실제 JSON 필요
- R6: 실제7002 hostname/SNI/system trust·index/replay/audit·actual Worker·mock·served loader hash·health startup hash·소유 PID/명령·서비스 생존 실제 근거 필요
- profile route와 failure injection이 mock response interception으로만 통과했다면 실제 server 제공/실 Worker 경계는 별도 표기
- 물리 phone sensor/image/exposure/thermal·현장 jump 지배 원인/개선·독립 field GT·full SLAM·상용 적합성은 이번 source/synthetic/dataset/virtual gate로 확정 제외
