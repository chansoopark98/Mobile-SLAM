# 최종 closure 체크리스트

- 승인: [독립 THOROUGH review](final-refactor-review.md) **APPROVE — R0–R6·실제 HTTPS7002·bounded virtual·첫 locked room2 native 범위**
- STANDARD: [cleanup 보고](cleanup-report.md)4 pass·추가 product/helper/test source 수정0. [root postcleanup](../../../build/refactor-evidence/final-closure/postcleanup-verification.json)도 PASS; root 최종 문서·서비스 인계·mode 종료만 후속
- 고정: core `b936982e5fea0c8ad0fb77e3e8b311770998763ff77ef2ef4bb082945866674b`, web `287c10de8bd2d725b47e2b6d47c2749ba893174793b5c8d3820b31c1b4a8ec0e`, config `a69eb5d465b1246c72016d00de3d383ac97523644d1c5faa3bb33903d7a2a42e`, embedded JS `9973d58eca6f95fcf5fcaf2d553d02bc2215f419e4ba1fc1cc94ae417eb3014d`
- 이번 쓰기: closure/cleanup 보고·inventory·task-local evidence만. 코드·source/artifact manifest·root README/runtime-status/execution-plan·다른 review docs·dataset·TLS·서비스 변경0. author 새 tool probe/build/test/replay/agent/Astra0

## 소유·보존

- [정확한88개 파일](../../../build/refactor-evidence/final-cleanup/session-owned-files.txt), list SHA `216a78ded17b59fd5f53fb312a656b6eaf981b855becad33c230f322e65cd0d0`; [현재 inventory](../../../build/refactor-evidence/final-cleanup/session-owned-inventory.json)
- R0 dirty18개는 session delta만; [task-local R0 복원](../../../build/refactor-evidence/final-cleanup/standard-r0-delta-proof.json)18/18 before fingerprint 일치. 기존 작업 전체 정리·되돌리기0
- 기존 unchanged16·audit11 =27개 제외, owner-reported21개의 original-byte 증명 한계 유지. vendor/build/generated/dataset/TLS/private/user logs·타 서비스 제외
- old82 scope는 `session-owned-inventory-82-before-final-physical.json`에 보존. 과거 SAN-running/GT 미실행/room2 prep-only 문서는 `closure-checklist-historical-before-standard.md`에 보존
- [final freeze](../../../build/refactor-evaluation/final-physical/pre-build-source-and-ABI-closure.json)85core/17web/35validation, [root postcheck](../../../build/refactor-evidence/final-closure/postcleanup-verification.json)85core/17web/1config/35validation/2registration 모두 mismatch0
- source88개 STANDARD before/after 동일. 원본 없는 파일·pre-existing TODO/Utility·vendor whitespace의 광범위 cleanup 거부; 목록 중복0·stager 기존 소유만 갱신

## R0–R6 현재 근거

| Gate | 현재 실제 근거·판정 | 검증 한계 |
|---|---|---|
| R0 | 위 scope·delta·freeze·기존 데이터/키/로그/타 서비스 보존 | file-level session provenance 한계21개 유지 |
| R1 | independent evaluator/config/input oracle RED→GREEN·metric scale1·unique GT association, [native102](../../../build/refactor-evaluation/final-physical/native-test-result.json)10 suites PASS | 숫자는 실행·단위 oracle와 trajectory metrics를 구분 |
| R2 | Engine21·Numerical34·SfM6·NativeAdapter6 포함102 PASS, Node78/Python browser7·WASM public API/actual Worker·clock/K/permission/profile | fake sensors/desktop와 실제 phone 구분 |
| R3 | [strictSAN102/10](../../../build/refactor-evaluation/final-physical/sanitizer-test-result.json)fail/skip/empty0·176.61s; ASan/UBSan/LSan strict·suppression0, [독립 source/runtime 승인](final-refactor-review.md) | affected23/23/13 repository consumers fresh, unaffected27/27/16 retained; 모든50/50/29TU를 다시 compile했다고 표현 제외 |
| R4 | [run6 ALL verified](../../../build/refactor-validation/run-6/validation-all.json), actual loss350·accepted/completed350·drop/error0·fresh284; first constant target0/null/map0·recovery·gap/busy 계약 | old run5 first blank FAIL 보존; 새 artifact와 별도 |
| R5 | current9973 실제3 paired full room1·사전 cap 동결·[room4 strict PASS](../../../build/refactor-validation/run-6/room4/parity.json), defaultpaced/loss·[첫 locked room2](../../../build/refactor-evaluation/room2-confirmation/first-native-default-result.json)1회 완료 | numerical benchmark/default timing 분리; room4 adaptation, room2 metrics-only·무production threshold |
| R6 | [현재 server3002812](../../../build/refactor-orchestration/server-status.json)·9973/coreb936 served SHA·trusted TLS/SNI·GET/HEAD/static boundary·actual HTTPS/profile/mock·workstation public DNS | 독립 off-LAN/실제 phone 미검증; author 서비스 조작0 |
| STANDARD/postcheck | source diff0·4 smell pass; [root 후속](../../../build/refactor-evidence/final-closure/postcleanup-verification.json)10/10 CTest11.33s·JS78/Python7·syntax0·scoped diff0·source/artifact match | source 불변이라 fullSAN/run6/room2 반복 불필요; 새 artifact 변경에 승인 전이 제외 |

## 최신·과거 정확도 구분

| 실제 실행 | body SE(3), scale1 translation ATE RMSE | coverage/역할 |
|---|---:|---|
| FINAL run6 room1 benchmark |0.05316615124297205m| fresh2776/2821·unique GT2719/2776; development3쌍·frozen cap |
| FINAL run6 room4 benchmark |0.05133081739730758m| fresh2148/2228·unique GT2108/2148; adaptation validation·strict parity PASS |
| FINAL room4 default |native0.05131657175614425m/WASM0.05131696350241545m| max10·0.1s·benchmarkfalse; [별도 실제 결과](../../../build/refactor-evaluation/final-physical/room4-default/final-default-result.json) |
| 첫 locked room2 native DEFAULT |0.06652931381663713m| full2882·fresh2838·GT2547/2838=89.7463%; 첫1회·source/config/scorer 고정·tuning/repeat0 |
| CAL-only archived core26ee/WASM12af |native0.051298642382137244m/WASM0.05129841730761593m| primary t_ic3만 개입한 [실제 causal control](../../../build/refactor-evaluation/calibration-only/paired-metric-brief.json), FINAL 결과와 별도 |
| 과거 wrong calibration |143.408m/중간169.747m, correctedbody-only169.80781m| 원본 실패·raw trajectory 보존; 새 pass로 소급 변경0 |

- room2 1s translation/rotation RPE RMSE `0.0208849677m/0.7026772756°`, rotation APE0.8989329689°, diagnostic Sim(3) scale0.9812145499. 89.7463% GT association 밖 pose를 정확도 분모에 숨기거나 GT 재사용하지 않음
- run6 numerical profile은 opt-in max10·10s·f/g/p0, 실제 Summary count/termination 그대로. default paced는20Hz·300input/286accepted+completed/14busy drop/241fresh; physical capture-to-display/phone60ms/thermal과 별도
- run3/run4 cap FAIL, run5 room4 solver count10/11 strict FAIL, run5 blank100 FAIL, SfM OLD4defectFAIL/2goldenPASS 자료 보존. SfM negative -1 sentinel은 actual Summary 미capture의 source inference만

## 도구·후속

- [source contracts](../../../build/refactor-evidence/final-closure/source-contracts.json): Node78/Python browser7/syntax exit0; root postcheck도 동일PASS. syntax는 parsing, ESLint/TypeScript gate 미구성
- [tool evidence](../../../build/refactor-evidence/final-cleanup/tool-evidence.json): version/install probe는2026-10-06T07:42:55Z 기록, Playwright/Context7는 실제 호출 성공 기록. config/skill/package 존재와 실제 접속 구분
- 실제 clangd23.1 한정 TU compiler error0·intentional Ceres ownership warning1; standalone full scanner·새 lint/static test 실행0. [최종 review](final-refactor-review.md)의 source/security 오류 경계 확인과 구분
- 남은 범위: physical phone clock/camera/IMU·현장 pose jump 해결·field GT·thermal·독립 off-LAN·full SLAM/production, WASM room2 미검증. process-global config/CV RNG/thread single-engine 제약 유지
- root 최종 문서·서비스 생존 인계·mode lifecycle 종료 후 전달. 추가 source/promote/deploy/replay 없음; 서비스3002812 유지
