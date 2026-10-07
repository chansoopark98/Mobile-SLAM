# Mobile-SLAM 리팩토링 결과

- 갱신: 2026-10-07 KST; 리팩토링·실제 HTTPS7002·가상 검증·최종 정리·후속 검증 완료
- 범위: 공통 추정기·수치·센서/Worker 경계 수정, 재현 평가, 실제 HTTPS 7002, 가상 브라우저 검증
- 계획 검토: Astra / ultra 1회; 이후 개발·검증 GPT-6.1 Sol / ultra
- 보존: 시작 전 dirty 작업·NAS 원본·TLS·기존 로그·다른 서비스; [시작 snapshot](../../../build/refactor-baseline/start-status.txt)
- 장기 production·실기기·현장 GT·전체 SLAM 판정과 이번 실행 범위 구분

## 확인한 핵심 원인

- TUM VI cam0의 `T_cam_imu` 역변환 translation 오류: 기존 `[0.04517590,0.07251590,-0.04395990]` → 원본 calibration `[0.045574835649698026,-0.07116180183799704,-0.04468125411714437]` m
- 원본 room1/room4 calibration 동일; inverse 양방향 identity·native/browser 값 독립 회귀 4/4
- **Calibration-only 인과 비교**: 동일 compiled core·입력·solver에서 room4 body SE(3), scale1 ATE `169.747693 m → 0.051299 m`; WASM `0.051298 m`
- 입력 2,228·fresh pose 2,148·GT one-to-one 대응 2,108·init frame76 동일; 기본 profile false·solver0.1s/max10/features150
- 기존 예측을 올바른 body 변환으로만 재평가: ATE `169.807811 m`; 평가 변환만으로 설명되는 개선 제외
- [인과 비교](../../../build/refactor-evaluation/calibration-only/paired-metric-brief.json)·[원본 metadata 검증](../../../build/refactor-diagnostic/room4-drift/input-review.md)
- 위 비교는 archived core `26eebd…`/JS `12af…`; 이후 q-sign·SfM 수정까지 포함한 최종 후보의 결과와 별도

## 변경·단순화

| 경계 | 변경·보호 |
|---|---|
| 공통 core | native VIOSystem → VIOEngine 위임, CMake source 목록 공유, 중복 IMU/feature orchestration 삭제 |
| 입력·좌표 | calibration 복사·검증, camera/body·row-major·crop/rotation K 계약, 엔진 한 곳의 미래 IMU·정확한 frame endpoint 보간 |
| 수치 | SPD whitening 재사용, gravity/SO(3)·depth·usable solve·reset/PnP 수명 보호, 정상방정식 prior → bounded square-root QR |
| 초기화 | non-keyframe 포함 velocity row mapping, finite/positive 삼각측량·빈 reconstruction·unusable BA 거부 |
| quaternion | 동일 물리 회전의 q/−q 비용 불변, main/PnP raw O_R residual·Jacobian sign을 whitening 전에 함께 적용 |
| feature | equal-priority total order, invalid rays·평행 배열 정리, 첫 균일 target의 LK false track 제거·ID 보존 |
| Worker·센서 | bounded IMU ring, epoch/sequence/request 연관, stale·timeout·reset·dispose 처리, 정확한 subarray/heap-growth 복사, clock/axis/permission/profile provenance |
| HTTPS | 실제 TLS chain/SNI, 공개 경로 allowlist·private/traversal 차단, bounded 로그·소유 PID lifecycle·artifact backup/promotion |
| 개발·평가 | 독립 scale1/rotation/one-to-one scorer, source/dependency/build/input/served hash, Playwright·Context7 MCP·clangd·validation skill |

- [정확한 소유 파일·보존 경계](../../../build/refactor-evidence/final-cleanup/session-owned-files.txt)·[수정 계획](execution-plan.md)
- [독립 source 검토](final-refactor-review.md)·[baseline](../../audit/2026-10-06/README.md)

## 검증 상태

| 검증 | 현재 결과 |
|---|---|
| 최종 native | 실제 102/102·10 suites, skip0; Numerical34·Engine21·SfM6 포함 |
| JS·Python | JS78/78·skip/TODO0, browser Python7/7, syntax 통과; ESLint/TypeScript 판정 제외 |
| 최신 WASM | JS `9973d58eca6f95fcf5fcaf2d553d02bc2215f419e4ba1fc1cc94ae417eb3014d`, embedded6,009,632B; 실제 Chrome API/reset/profile lifecycle 통과 |
| 최종 SAN | ASan/UBSan/LSan 실제 102/102·10 suites,176.61s,skip0·suppression0 |
| 최종 궤적·runtime | Run6 ALL strict PASS: room1 전체3회·room4·default paced/loss/gap/busy/real HTTPS mock; room2 native 첫1회 완료 |
| HTTPS | 최신9973 배포·HTTP34·strict TLS/SNI/chain3·실제 GET SHA/source 검증 통과; PID3002812 실행 유지 |
| 최종 review/cleanup | 독립 THOROUGH APPROVE·STANDARD88개 범위 정리 완료, 추가 코드 수정0; 후속 CTest10/10·JS78·Python7·syntax·source SHA 검증 통과 |

- 최종 core source manifest: `b936982e5fea0c8ad0fb77e3e8b311770998763ff77ef2ef4bb082945866674b`
- affected transitive 객체 재컴파일 + 변경 없는 객체의 source/dependency/flags/object hash 확인; 모든 객체를 이번 pass에서 새 컴파일했다는 주장 제외
- [최종 정리](cleanup-report.md)·[후속 검증](../../../build/refactor-evidence/final-closure/postcleanup-verification.json)
- [실행 근거](../../../build/refactor-orchestration/evaluation-status.json)·[source 계약 로그](../../../build/refactor-evidence/final-closure/source-contracts.json)
- 이전 실패 보존: room4143/169m, run5 strict solver metadata 차이, 첫 blank loss 실패; [가상 검증 기록](virtual-validation.md)
- Strict 원본 counter·결과 유지; FUNCTION-stop 보충 판정은 새 artifact의 실제 causal capture 필요. [사전 정책](run-6-assessment-policy.md)

## 최종 궤적·처리 지연

- independent GT·camera→body·one-to-one association·SE(3) scale1; Sim(3)는 진단 전용

| Sequence / 설정 | 입력 /fresh /GT 대응 | 위치 ATE RMSE | 회전 APE RMSE |
|---|---:|---:|---:|
| room1 native·Worker 전체3회 /benchmark true·max10·10s cap | 2,821 /2,776 /2,719 | 0.05317m | 1.20302° |
| room4 native·Worker /benchmark true·max10·10s cap | 2,228 /2,148 /2,108 | 0.05133m | 1.25112° |
| room4 별도 기본 설정 /false·max10·0.1s | 2,228 /2,148 /2,108 | native0.0513166m /WASM0.0513170m | 약1.251° |
| room2 native 첫1회 /false·max10·0.1s | 2,882 /2,838 /2,547 | 0.06653m | 0.89893° |

- room2 protocol-v2는 첫 궤적 실행 전 동결; tuning·재실행0. GT 대응89.7463%, 미연결291포즈 구간 정확도 미측정; same rig/domain·WASM room2 미실행
- room1 3회·room4 strict 비교: image/F64 IMU/config·init·fresh·state/reason/solver·epoch 모두 일치, 사전 numeric cap 유지; 보충 예외 적용0
- 동결 contract SHA `53bba52381be83e932d9c9c3c94867c6e08436af4cb11d778f803feb7164ae76`; room4 실행 전 기록
- default 20Hz paced /300input /286completed /241fresh /14drop(4.7%). 초기화 후 engine p50/p95/p99 `29.08/49.20/62.01ms`, roundtrip `29.29/49.43/62.33ms`
- desktop source-time proxy·hashOFF·default false/0.1s; 속도 개선·무drop sustained20Hz·실제 phone SLA 주장 제외. 실제 capture/exposure/render latency 별도
- 첫 blank100–111 즉시 null pose/time/map0·freshfalse, textured 회복121. 실제 gap/busy·frame_gap epoch2·null/map0 검증 통과
- owned RSS sampling1s: 최대 summed RSS1,424.70MiB(shared page 중복 가능), native HWM88.18–90.15MiB. Observer CPU12.27s/690.64s ≈1.78% one core; sample mean14.95/max33.27ms
- [가상 검증 signoff](../../../build/refactor-validation/run-6/final-validation-signoff.json)·[room2 첫 확인](../../../build/refactor-evaluation/room2-confirmation/first-native-default-result.json)

## 서비스·재현

- 서비스: <https://dev.serdic.com:7002/>; PID `3002812`, instance `c98f10df76144a3c80311b09d1e0d5cf`; 관측 페이지: <https://dev.serdic.com:7002/audit-mobile.html>
- 실행·상태·소유 서비스 종료: `bash scripts/ops/serve.sh {start|status|stop}`; 최종 검증 뒤 서비스 실행 유지
- 소유 로그: `logs/https/20261006T164826765565Z-7002-c98f10df76144a3c80311b09d1e0d5cf.log`; 기존 로그·TLS 보존
- 기존 hosts override의 hostname→127.0.0.1과 `dig @1.1.1.1`+명시 public TCP183.98.179.103 검증 구분; 같은 호스트 두 경로 모두 HTTP200/TLSverify0, 독립 off-LAN 미검증
- 개발 명령·설정·실제 도구 호출 범위: [scripts/dev/README.md](../../../scripts/dev/README.md)·[도구 근거](closure-checklist.md)
- NAS canonical home `/mnt/backup/SLAM` 읽기 전용, staging·결과 local `build/`; NAS 전체 공개 제외

## 남은 판정 범위

- 실제 Android/iOS camera·IMU·intrinsic/extrinsic profile·현장 jump·thermal·capture-to-display latency: 미검증
- observation JSON은 완전한 image/raw-IMU replay 또는 independent field GT 대체 근거 제외
- room4는 원인 진단 후 development split; 추가 room2는 동결된 소스의 1회 확인·동일 rig/domain 한계 명시
- loop closure·영속 지도·global relocalization·formal manifold 전체 전환·상용 배포 라이선스: 별도 구현/판정 대상
