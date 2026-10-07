# 최종 가상 검증 계획·실행 계약

- 소유: `tests/browser/refactor-validation.mjs`, 이 문서, `build/refactor-validation/` 새 evidence
- 실행 조건: leader `performance_ready` 신호 이후; native/WASM/sanitizer build·다른 replay와 latency 측정 동시 실행 금지
- 서비스: 기존 owner가 연 실제 `https://dev.serdic.com:7002/`; Chrome hostname→127.0.0.1 resolver만 사용, TLS 검증 활성. 별도 공개 서버/다른 서비스 종료 없음
- 입력: room1 2,821 / room4 2,228 frame, native actual OpenCV gray·ordered `[ts,ax,ay,az,gx,gy,gz]` byte oracle, camera/IMU/GT/config CSV SHA-256
- 설정: TUM cam0 calibration/noise, single future bracket·engine carry, seed0·native threads1, LK21/3·20iter/.03·min20·edge0·F1, PnP0/freq3, solver10iter/10s nonbinding cap
- 비교: full room1 native/browser 3쌍 순차 → 입력 hash·fresh pose·state/reason·solver iterations/termination·relative epoch 확인 → 수치 계약 동결 → full room4
- 실행 전 독립 cap: translation p95≤0.001m / max≤0.01m; rotation p95≤0.01deg / max≤0.1deg; init frame·fresh coverage·state/reason/solver 차이0. 관측 오차로 cap 확대 금지
- 타이밍: max-speed와 source-time absolute deadline 20Hz paced 별도. hash 계측은 parity에만 적용, paced에서는 해제. queue/copy/decode/engine/processing/roundtrip·pose sensor age 분리
- 가상 live: Android Generic/iOS DeviceMotion mock permission grant/deny/error, activation·source timestamp/late callback·축, pause/resume·rotation·drop/gap, actual Worker·fresh WASM textured→blank→recover
- HTTP: index/replay/audit, COOP/COEP/cache/MIME/HEAD, served source/bundle SHA vs fresh provenance·health snapshot, strict local SNI + workstation public DNS 접근 결과 별도
- 정확도: actual engine poseTimestamp, camera `T_W_C` row-major16 → camera TUM; 독립 scorer body conversion·SE(3)1 translation/rotation ATE/RPE·GT coverage, Sim(3) scale 별도
- 실패: zero initialization/pose/association, 입력 hash 누락/불일치, 필수 frame 누락, 새 console/page/network error, cap/state 불일치에서 pass 금지. evidence 보존 후 owner/leader 인계
- 제한: desktop fake camera/sensor·calibrated dataset 검증; 실제 phone exposure/thermal/clock/calibration, jump 원인·field GT·full SLAM·상용 배포 검증과 구분
- 실행 전 상태: runner syntax·자체 quaternion/SO(3)/percentile3개 검사 통과; 실제 R5/R6 통과 근거는 아직 없음

## 실행 명령

```bash
node tests/browser/refactor-validation.mjs --phase self-test
MOBILE_SLAM_PERFORMANCE_READY=1 MOBILE_SOURCE_REAL_PROFILE=1 node tests/browser/refactor-validation.mjs --phase all --benchmark-zero-tolerances 1 --output build/refactor-validation/NEW-RUN
```

- 지원 phase: `preflight`, `pilot`, `room4`, `paced`, `loss`, `mock`, `all`; `room4`는 앞선 pilot의 frozen contract와 SHA 필요
- 구분 실행 시 같은 output 아래 phase별 로그/보고서 분리; 기존 native/output 로그 덮어쓰기 거부, 재시도는 새 output 경로 선택
- root 신호 없이 환경변수 설정·성능 실험 시작 금지. native binary·provenance·fresh candidate 위치는 CLI override 가능
- full paired room1/room4: 명시 선택 `max10_zero_positive_tolerances`, f/g/p=0, 최대10iteration·10s cap. 실제 Ceres iteration/termination 그대로 비교. production 기본 positive tolerance 변경 제외
- paced/loss: benchmark false, TUM 기본0.1s·최대10iteration·150feature; main live mock은 제품 자체 budget 유지. 수치 비교용10s cap을 production latency budget으로 해석 제외
- 추가 Android 검사: DeviceMotion permission shim 없는 Generic `Permissions.query` deny, 실제 Start activation·센서 미시작·screen orientation 이벤트의 epoch/cache 무효화

## 첫 실제 실행·superseded 기록

- 실제 service: PID1520254, `https://dev.serdic.com:7002/`, strict Chrome·workstation public DNS TLS 검증 성공; 후보 SHA `b813c97cf636fad4eee68bd5e6240caae2b063b662544d294441f2800514e9fb`
- 최초 preflight 실패: optional `/favicon.ico` 404 console event에 Playwright response event 부재. exact console location URL+status로 expected negative 판정 보완; 다른 console/page/network 오류 필수 실패 유지
- 첫 full pair: native/browser 각 room1 2,821 frame / 2,776 fresh pose / init45, browser drop0; 전체 decoded gray·IEEE Float64 IMU byte hash 일치
- 분류: `build/refactor-validation/run-2/superseded.json`; 독립 검토의 FeatureTracker finite/positive-Z guard 결함과 IMU unique-loss invalidation 보완 전 후보. 최종 parity/accuracy 완료 근거로 재사용 금지
- 중단: 소유 runner1530733·하위 native/isolated Chrome만 종료, second native pilot partial 보존, frozen contract 없음. HTTPS service·다른 서비스 보존
- 새 실행: guard/endpoint getter fresh build·hash·source/provenance·service health refreeze 후 새 output 경로에서 full3쌍부터 재시작; 수치 cap 변경 없음

## Run-3 실제 결과·R5 실패

- 후보 SHA `882791a5f876cfda36d413058ea83725e6f73663e974e641ae85da8ad4f75d12`; source manifest `5624dc8c9ff4fa6a02b4abf77a8bf38467aee7bf4a89f059f516ddf2dd92110c`; actual HTTPS PID1810169
- strict preflight·public DNS·실제 served source/bundle hash·GET/HEAD·COOP/COEP/cache·index/replay/audit 통과; favicon204, 새로운 network/page/console 오류 없음
- full room1 세 쌍: 양쪽 2,821 input / 2,776 fresh pose / init45 / fresh coverage98.404820985%; browser drop0; full decoded gray·Float64 IMU byte/settings 모두 일치
- 반복성: native 내부3회·browser 내부3회 pose translation 차이0, state 차이0; rotation acos roundoff floor max약4.52e-6deg
- native/browser 차이: solver iterations171 / termination42 frame 항목 차이, status·reason·relative epoch·fresh coverage 차이0; first iteration divergence frame479(8 vs9, 둘 다 CONVERGENCE), first termination divergence frame726
- 수치: translation p95 0.408633993m / max0.515326159m; rotation p95 0.245749535deg / max0.657387124deg → 독립 사전 cap 실패
- 첫 위치 차이: >1e-8m frame62; >1e-6m frame94; >0.001m frame448; >0.01m frame725. Feature count는 전체2,821 frame 일치; full frontend coordinate/stage equality는 이 결과만으로 확인 불가
- 판정: **R5 FAILED**, frozen contract 생성 없음; room4·paced·loss/recovery·real public profile mock 미실행. 자료 `build/refactor-validation/run-3/{freeze-failure-analysis,first-divergence-details}.json`
- 조치: 소유 runner42577 정상 오류 종료·profiling 중지, 결과 보존. 허용 cap 확대 없이 owner의 solver/marginalization 조건 진단 후 새 후보·새 output에서 재검증

## Run-4 첫 full pair·사전 cap 실패

- 후보 SHA `b75645443e52c8b51a0b78116ecad2f9cd02e4d0d94b7e64d6af2897f6008bff`; actual service PID1947692, strict preflight·public DNS·source/bundle·app hash 통과
- 첫 room1 full pair: 각2,821 frame / 2,776 fresh / init45 / browser drop0 / 입력·설정·feature count 일치
- 차이: translation p95 0.001472714m / max0.001883711m; rotation p95 0.008300623deg / max0.010768801deg
- 판정: translation p95의 사전0.001m cap 초과, solver 항목4개 불일치 → **R5 FAILED**, 개선 폭으로 pass 처리 금지
- 분기: frame1105 native6/CONVERGENCE vs browser11/NO_CONVERGENCE; frame1106 iterations6 vs5; frame1107 iterations7 vs6. 이전 frame479·725의 큰 분기는 닫혔지만 full 조건은 미충족
- root 승인 early check: 첫 쌍 비동결 compare 후 소유 runner1952906·하위 native/isolated Chrome만 중단, partial second native 보존; service1947692 유지·frozen contract 없음
- 근거: `build/refactor-validation/run-4/pilot-1/{early-comparison,first-solver-fork}.json`, `run-4/early-stopped.json`; room4·paced·loss·public profile mock은 아직 미실행

## Run-5 수치 비교·독립 기본 runtime 검증

- 실제 HTTPS: PID `2449945` / instance `41fe340bca3a44ad8405987db68b53a3`; embedded loader SHA256 `fce25012d61a0d5965e4d5afadda25aecf96c219b30451e0394575c3caa10da1`, source manifest `11ca788ff68b39653984b2d384c672820fcf698cfc0a621d273a49311ae9075c`
- 모든 phase: strict Chrome TLS/SNI·served source/bundle SHA·GET/HEAD·COOP/COEP/cache·세 페이지·workstation public DNS 확인. 외부 네트워크/실제 phone 접근 근거와 구분
- full 수치 비교: 명시 benchmark true / 최대10iteration /10s cap; 실제 Ceres summary row/termination 보존. paced/loss는 actual benchmark false / TUM0.1s /최대10iteration; main mock은 기존 앱 설정
- room1 개발 3쌍: 각각2,821 input /2,776 fresh /init45 /browser drop0, decoded gray·Float64 IMU bytes·설정 일치; state/reason/solver/relative epoch 차이0
- room1 cross: translation p95 `0.0000746661m` /max `0.000332278m`; rotation p95 `0.000562735deg` /max `0.00229368deg`. 사전 cap 내부; 관측 오차로 cap 확대 없음
- 반복: native/browser 각 플랫폼 translation 차이0·state 차이0; rotation acos roundoff floor max `4.52e-6deg`
- 동결: `2026-10-06T14:36:59.191Z`, **room4 시작 전**; `parity-contract.json` SHA256 `4f82ccdfbebd48504d5de8320c5704511009d11b48b7f1e6478ada084cb2db77`
- room4: 각2,228 input /2,148 fresh /init76 /browser drop0, 입력·설정 일치. translation p95 `2.258e-6m` /max `2.750e-6m`; rotation p95 `3.818e-6deg` /max `4.183e-6deg`
- **room4 strict FAILED**: frame1973의 실제 solver iterations native10 vs browser11, termination `CONVERGENCE` vs `NO_CONVERGENCE`; 그 외 상태·epoch·coverage 차이0. Internal Ceres 종료 메시지는 full replay capture 범위 밖; 원인 추정과 구분
- ALL runner 정상 오류 종료1·소유 Chrome 종료; 서비스 유지. `validation-all.json` FAILED·기존 contract·실패 자료 보존. 이후 phase는 parent 승인으로 별도 실행, 전체 pass로 합산 제외

| 독립 GT / body SE(3), scale1 | room1 native | room4 native |
|---|---:|---:|
| fresh pose / input | 2,776 /2,821 | 2,148 /2,228 |
| one-to-one GT association /fresh | 2,719 /2,776 | 2,108 /2,148 |
| translation ATE RMSE, m | 0.800450622 | 143.367043 |
| rotation APE RMSE, deg | 5.78040915 | 173.210263 |
| 1s RPE pairs | 2,655 | 2,067 |
| translation RPE RMSE, m | 0.178709133 | 5.54987394 |
| rotation RPE RMSE, deg | 0.721480623 | 0.830856399 |
| diagnostic Sim(3) fitted scale | 0.634966238 | 0.00166333125 |

- scorer: max association dt0.01s·RPE delta1s/tolerance0.05s·actual engine poseTimestamp·camera→body 변환. Rotation APE는 동일 position-fit SE(3) rotation 사용; Sim(3)는 진단 전용
- room4의 큰 절대 오차 관측; micrometre native/browser 일치만으로 정확도 통과/개선 주장 제외. Browser GT 수치도 근접; `room4/{native,browser}/metrics.json`
- 독립 paced **PASS**: 20Hz /300 input /294 accepted+completed /6 `paced_worker_busy` drop /249 fresh /init45 /epoch change0; `inputHash=0`, sensor pose age0
- paced 초기화 후 p50/p95/p99, ms: engine `26.71/45.12/54.31`, Worker `26.78/45.23/54.43`, roundtrip `26.95/45.51/54.60`, deadline→result `27.97/47.36/57.71`. Decode/copy/queue 별도 `paced-20hz/summary.json`; 실제 exposure·capture-to-render·phone thermal 검증 제외
- 독립 loss **FAILED**: 실제350frame /drop0 /error0. 첫 blank100에서 current poseTimestamp의 fresh valid pose·tracking·feature8/map157; blank101–111에서 `empty_features`·null pose/time/map0, recovery fresh 시작121. stale timestamp 증거와 one-frame loss-detection transition 구분
- loss 후반 fresh229개; gap/busy probe는 blank assertion 이전 실패로 **미실행**. `textured-blank-recover/{results,failure,summary}.json`, `validation-loss.json` 보존
- 독립 mobile mock **PASS**: actual Chrome5개 grant/deny/error/profile 시나리오 + 별도 Android Generic `Permissions.query` denial/orientation. 실제 Start activation·source clock/axis·pause epoch·pixel rotation golden·configure322×240/K/extrinsic/distortion 확인
- 실제 same-origin 임시 JSON: GET200/HEAD200/body0, SHA256 `d85f2dd4184d6f47b0710cd326a5ca8defab4f905f7ea23ad0bc1da4c94f48e2`, MIME JSON·COOP/COEP/no-store·default certificate verification/TLS1.3, `temporaryProfileRemoved:true`
- generic deny: accel/gyro permission query 모두 Start activation true, IMU emission0, rotation epoch2→3·cache null/map0. `mock/browser-contracts.json`, `mock-android-generic-denial/result.json`
- error 관측: preflight/replay/paced/loss는 console/page/request/HTTP events 필수 assertion; mobile mock은 pageerror·특정 실제 profile HTTP assertion 범위. Mock 전체 console/request event inventory 근거는 별도
- 메모리: verified PID/start ticks의 1s VmRSS/VmHWM 샘플. full room1/room4 native HWM 약87.7–90.7MiB; 최대 summed owned RSS `1512.18MiB`는 shared page 중복 가능. WASM linear heap capacity·실사용·QR workspace와 동일시 제외
- observer 비용: full wall607.6s /observer CPU24.6s, sample 평균약40ms/max88ms; 독립 phase는 stat discovery+소유 descendant 상세 읽기로 별도 비용 기록. `owned-process-memory*-summary.json`
- QR 선언 cap: combined dense workspace8,000,000 doubles =64,000,000 bytes /61.035MiB. allocator·factor private buffer·전체 RSS 제외; 실제 WASM capacity/selected QR workspace는 build lane capture와 구분
- 변경·검증: Worker/replay opt-in profile2개·runner·신규 profile test; 신규8+기존70=78 strict PASS·syntax4. `benchmark-validation-lane.md`, `build/refactor-evidence/benchmark-profile/source-frozen.json`
- 현재 판정: **전체 strict 검증 미완료**. room1 development/freeze·paced·mock 통과와 room4 solver strict·blank transition 실패 분리; physical phone·field GT·full SLAM·production/off-LAN 검증 제외
- 후속 bounded room4 trace: 같은 `fce250…` artifact의 frame1973에서 native actual message `Function tolerance reached. |cost_change|/cost: 0.000000e+00 <= 0.000000e+00`와 WASM actual max10 종료 확인. Native의 function stop은 summary append 이전 반환; trace 없는 trial의 step/gradient 직접 측정으로 확대 제외
- [보충 독립 판정](qr-resume-peer-review.md): **numeric/current-usable/method parity PASS WITH PROVEN FUNCTION-STOP EXCEPTION; strict solver summary equality FAIL**. 원본 counter·termination·contract·room4 정확도 실패 보존; 모든 convergence/max-budget 차이를 일괄 예외 처리하는 기준 제외

## Run-6 최종 실제 browser 검증 — ALL strict PASS

- 완료: `2026-10-06T17:09:28.312Z` /KST2026-10-07 02:09; runner exit0, 소유 isolated Chrome·CPU measurement 종료. 서비스 lifecycle 변경 없음
- 적용: 실제 HTTPS PID `3002812` /instance `c98f10df76144a3c80311b09d1e0d5cf`; embedded JS SHA256 `9973d58eca6f95fcf5fcaf2d553d02bc2215f419e4ba1fc1cc94ae417eb3014d`, source manifest `b936982e5fea0c8ad0fb77e3e8b311770998763ff77ef2ef4bb082945866674b`
- 실행 전 parent exact true gate·fresh native/full SAN102 tests/10 suites·API/LSP/HTTP 근거 확인; 이 validation lane의 build/SAN 실행 결과로 확대 제외. 실행 전/후 native/WASM artifact·모든 provenance source hash 불변 재확인
- 사전 [판정 정책](run-6-assessment-policy.md) SHA256 `4834e6ccc9662887cb862a605dfc8422b7d73de9f3b79e454243ecc9295bb900` 유지; 원래 cap·raw Ceres count/termination·comparer/runner assertions 변경 없음
- Native/browser full room1 3쌍: 각각2,821 input /2,776 fresh /init45 /browser drop0, 모든 decoded gray·F64 IMU bytes·설정 일치, state/reason/solver/relative epoch·init/coverage 차이0
- room1 cross translation p95 `5.48062e-11m` /max `1.80669e-10m`; rotation p95 `3.81819e-6deg` /max `4.18262e-6deg`. 실제 정확도 수치가 아니라 동일 입력 cross-platform 수치 일치 지표
- Within-platform3회: translation 차이0·state/solver 차이0; rotation acos roundoff max `4.51775e-6deg`
- 실제 freeze: **room4 시작 전** `2026-10-06T17:06:30.563Z`; contract SHA256 `53bba52381be83e932d9c9c3c94867c6e08436af4cb11d778f803feb7164ae76`. Translation p95≤0.001/max≤0.01m, rotation p95≤0.01/max≤0.1deg 사전 cap 그대로
- Full room4 `development_after_repairs`: 양쪽2,228 input /2,148 fresh /init76 /browser drop0, byte/settings 일치; translation p95 `2.03820e-11m` /max `2.94185e-11m`, rotation max `4.18262e-6deg`; **strict state/reason/solver/epoch 차이0, PASS**
- Final artifact에서 새 summary mismatch 없음; supplemental FUNCTION-stop exception·count normalization 적용 없음. Old run-5 strict FAIL·old artifact의 보충 판정·old wrong calibration/metrics 그대로 보존

| 최종 numerical profile GT / body SE(3), scale1 | room1 native | room4 native |
|---|---:|---:|
| fresh/input | 2,776/2,821 | 2,148/2,228 |
| one-to-one GT association/fresh | 2,719/2,776 | 2,108/2,148 |
| translation ATE RMSE, m | 0.0531661512 | 0.0513308174 |
| rotation APE RMSE, deg | 1.20301948 | 1.25111524 |
| 1s RPE pairs | 2,655 | 2,067 |
| translation RPE RMSE, m | 0.0139185529 | 0.0145527803 |
| rotation RPE RMSE, deg | 0.559451650 | 0.622106868 |
| diagnostic Sim(3) fitted scale | 1.00391195 | 0.984357067 |

- Browser GT metrics도 근접. Actual poseTimestamp·camera→body·dt≤0.01s·RPE1s/tolerance0.05s; numerical profile true/최대10iteration/10s cap. 별도 default room4 약0.0513166m 관측과 이 표의 profile 수치 구분
- Default paced **PASS**: actual profile false /TUM0.1s·최대10iteration·150feature /hashOFF /source-time20Hz300; 286 accepted+completed /14 `paced_worker_busy` drops /241 fresh /init45 /epoch change0
- 초기화 전 engine p50/p95/p99 `16.11/23.46/28.21ms`, roundtrip `16.52/23.98/28.52ms`; 초기화 후 engine `29.08/49.20/62.01ms`, Worker `29.17/49.27/62.12ms`, roundtrip `29.29/49.43/62.33ms`, deadline→result `31.03/49.97/65.22ms`
- 초기화 후 decode p95 `0.535ms`, frame copy `0.300ms`, Worker queue `0.195ms`, IMU wait0. Pose sensor age0은 CSV input/pose 시간 차이; 실제 exposure/capture-to-render latency와 구분. 14/300 drop 관측; 지속적 실제 phone20Hz 통과 주장 제외
- Default loss **PASS**: actual350 input/accepted/completed /drop0/error0 /284 fresh. 모든 blank100–111 `empty_features`·invalid/nonfresh·null pose/time·map0; recovery fresh 시작121
- Actual gap/busy **PASS**: +2s gap의 새 frame accepted·즉시 중복 frame busy drop1; `frame_gap`, engine epoch2·initializedfalse·null pose/time/map0, solver0/not_run. Fake pose injection 없음
- 실제 mobile mock **PASS**: Chrome5개 Android/iOS grant/deny/error/profile + Generic `Permissions.query` denial/orientation. Start activation·source timestamps/축·pause/resume·pixel rotation·K/crop/resize/config322×240·extrinsic/distortion golden 확인
- Real same-origin JSON GET200/HEAD200/body0, MIME/COOP/COEP/no-store, SHA256 `d85f2dd4184d6f47b0710cd326a5ca8defab4f905f7ea23ad0bc1da4c94f48e2`, strict browser TLS1.3·PID3002812/fresh9973 snapshot 일치. Finally unlink 후 실제 disk 부재 확인
- Network inventory: full3 room1·room4·paced·loss의 console/page/request/HTTP error 각각0. 기존 mock은 pageerror·특정 profile HTTP assertion 범위; 전체 general console/request inventory로 확대 제외
- 접속 routing: Chrome explicit127 resolver; 일반 hostname도 기존 unrelated `/etc/hosts`의127 override. Raw preflight `publicDNS.verified`는 legacy hostname HTTPS 성공 필드이며 public DNS 증거로 해석 제외
- 별도 `network-routing-context.json`: hosts 보존·127 routing과 parent의 `183.98.179.103:7002` forced-public-TCP/system-CA/SNI actual GET/health 검증 연결. 모두 이 workstation의 관측; 독립 off-LAN/phone 접근 미검증
- Memory observer: verified owned PID/start/descendants·1s stat discovery/detail sampling, 679samples /wall690.64s /observer CPU12.27s(단일 core 약1.78%). Sample 평균14.95ms/max33.27ms; 측정 observer overhead 공개
- Native HWM88.18–90.15MiB; 최대 summed owned RSS1424.70MiB(shared page 중복 가능). 전체 RSS·WASM linear heap capacity·QR declared combined workspace8,000,000 doubles/61.035MiB 서로 다른 범위
- 최종 근거: `build/refactor-validation/run-6/validation-all.json` status `verified`, `parity-contract.json`, `room4/parity.json`, `paced-20hz/summary.json`, `textured-blank-recover/gap-drop.json`, `mock/browser-contracts.json`, `owned-process-memory-summary.json`
- 판정 범위: 실제 served final artifact의 calibrated dataset parity·bounded default runtime/transport·desktop fake sensor/camera 계약 통과. 실제 phone calibration/exposure/thermal·field GT·full SLAM/loop closure·production accuracy와 구분
