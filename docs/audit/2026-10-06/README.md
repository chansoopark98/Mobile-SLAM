# Mobile-SLAM 병렬 진단 결과

- 기준일: 2026-10-06, 기존 dirty working tree 기준
- 수행: 독립 병렬 에이전트로 구조·추정기·native·browser·개발 도구 점검, Planner → Architect → Critic 계획 검토
- 현재 단계: 진단·재현 도구 구축; 알고리즘 전면 수정 전 baseline 확보
- 사용자 핵심 증상: PC에서 정상으로 보이지만 실제 모바일 camera pose 급변·불안정
- 우선 후속 작업: 실제 모바일 입력 관측 → 동일 recording replay → 시간·calibration·좌표·tracking/reset 원인별 회귀
- 실제 모바일 검증: 현재 ADB 연결 기기 없음·실기기 관측 로그 미수집; 공개 dataset desktop 결과로 증상 해결 판정 불가
- 전체 계획: [refactoring-plan.md](../../refactoring-plan.md)

## 실제 실행 결과

| 경로·입력 | finite pose 출력 / 입력 frame | 초기화 | 위치 ATE RMSE, SE(3) | 1초 RPE 위치 / 자세 | 시간 p95 |
|---|---:|---:|---:|---:|---:|
| 새 native VIOEngine, room1 전체 | 2,776 / 2,821 | 2.250s | 0.555253m | 0.160278m / 0.743111° | engine 25.452ms |
| 새 native, 앞 600 frame, boundary bracket | 555 / 600 | 2.250s | 0.242430m | 0.075836m / 0.892351° | engine 27.136ms |
| 새 native, 앞 600 frame, image 시간 이하 IMU | 555 / 600 | 2.250s | 0.289261m | 0.079916m / 0.880487° | engine 26.071ms |
| 기존 배포 WASM, browser 앞 601 frame | 556 / 601 | frame45 | 0.282962m | 0.078070m / 0.881804° | Worker roundtrip 30.735ms |
| 새 WASM, browser 앞 601 frame | 556 / 601 | frame45 | 0.282962m | 0.078070m / 0.881803° | Worker roundtrip 28.750ms |
| 새 native, 앞 600 frame, PnP on | 551 / 600 | 2.250s | 0.129010m | 0.059828m / 0.826307° | engine 21.258ms |

- 측정 장비: 개발 PC; native cv threads=1, solver 0.1s/10회, 150 features; browser Chrome144, Playwright1.63.0
- Native 전체 입력: 141.004s, 실행 62.151s; process p50/p99 = 19.126/29.764ms
- 시간의 범위: native engine 내부 호출, browser submitted-frame → 완료 결과 roundtrip; capture·image decode·render·물리 센서 지연·thermal 제외
- 실행 방식: 최대속도 serial dataset replay; 실제 모바일 live 20/30Hz pacing 통과 근거 제외
- 입력 범위: native 전체와 browser 약30초는 길이·IMU 공급 경로 차이 존재; native↔WASM 동일 입력 parity 합격으로 해석 금지
- Browser 600-frame 요청은 관측 종료 시 601개 처리; 실제 처리 개수를 분모로 평가
- 모든 정확도: camera → IMU/body 변환, one-to-one GT association ≤10ms, scale=1 rigid SE(3); RPE interval 1s, tolerance50ms
- Pose 출력 수: engine 반환값·finite/SO(3) 관찰 기준; 실제 tracking 유지·물리 정확도는 별도 지표, self-reported lost=0만으로 추적 정상 판정 불가
- Native 전체: pose coverage98.4048%, GT association2,719/2,776; 자세 APE RMSE4.785254°
- Native 전체 Sim(3) 별도 진단: fitted scale0.833508, ATE0.528079m; metric 정확도 판정에 사용 제외
- 기존/새 WASM: 이 601-frame 범위에서 pose matrix 최대 차이3.126e−7; hash 차이를 모바일 불안정 주원인으로 단정 불가
- PnP: 한 30초 조건에서 속도·오차 개선 관측, pose4개 누락; 기본값 변경·일반화·실기기 안정성 판정 보류
- 공통 평가 cross-check: 기존 Python의 별도 body transform/rigid alignment/SE(3) RPE 수식과 1e−9 이내 일치; association helper 공유, 독립 field GT 검증과 구분

## 결함·수정 우선순위

| 우선순위 | 확인된 문제 | 근거·의미 |
|---|---|---|
| P0 | 모바일 callback 시간과 실제 sensor/capture 시간 구분 필요 | `imu.js`는 callback `performance.now()` 사용, live image는 presentation 시간; 실제 clock/offset/지연 측정 우선 |
| P0 | 측정 calibration 대신 FOV·extrinsic 경험값 | 기본69° FOV, `t_ic=[0,0,-0.02]`, 회전·crop·기기별 profile 검증 필요; 실제 jitter 주원인은 미확정 |
| P0 | 회전 RPE 상시0, 잘못된 timestamp index, Sim(3) scale 보정 | native 기존 평가 지표로 자세·metric scale 품질 판정 불가; independent scorer 추가 |
| P0 | parity calibration alias·저장 순서 훼손 | `RᵀR−I` norm0.04349 재현; 기존 비교 fixture 자체가 회전행렬 계약 위반 |
| P1 | feature loss 후 과거 pose를 valid로 반환 가능 | tracking·lost·stale timestamp 구분, FailureDetector 호출 연결 필요 |
| P1 | 내부 reset에서 marginalization prior 유지 | epoch·prior·PnP·map 상태를 함께 초기화하는 회귀 필요 |
| P1 | IMU0/1 sample covariance whitening NaN | factorization 유효성·camera 경계 interpolation·future carry 검사 필요 |
| P1 | Worker timestamp/IMU ring/subarray/waiter/map 계약 | 정상4개 + TODO 결함5개 재현; TODO를 품질 통과로 해석 금지 |
| P2 | capture/copy 비용, 완료 처리량과 UI counter 불일치 | mobile profiler·frame drop·pose age 계측 우선; 속도만으로 추정 품질 판정 금지 |
| P2 | native/browser orchestration 분리, full SLAM 기능 부재 | same-core replay 경계 확보 후 통합; loop closure/relocalization/map persistence 별도 단계 |

- 원인 구분: 위 코드 결함·입력 위험은 확인; 사용자가 경험한 실제 모바일 pose 점프의 지배 원인은 미확정
- 차용 코드 provenance: root MIT 표기와 GPLv3 reference 동일/차용 코드17개; 파일별 provenance·배포 조건 정리 필요

## 테스트 결과의 범위

| 검사 | 결과 | 한계 |
|---|---|---|
| Native fresh build | 성공, 194단계/81.27s | 제품 소스85개 기존 hash 유지 |
| 기존 unit | 24/24 통과 | literal config·비어있는 output 등 약한 assertion 존재 |
| 기본 CTest | 5/6 실행 파일 통과 | parity 상대 cwd 문제, dataset 복원 후에도 기본 gate 실패 유지 |
| repo cwd parity | 1/3 통과 | GTK GUI 오류2개, Qt offscreen 적용 불가 |
| GUI만 끈 복제 fixture parity | 3/3 통과 | engine172/native255 poses, 평균 위치 차이0.0958m/자세3.1233°도 broad threshold 통과 |
| ASan/UBSan 부분 검사 | 15/15 통과 | 전체 estimator/PnP/모바일 메모리 검증 제외 |
| Fresh WASM build + integration | 성공, 기존 assertion15/15 | 합성 입력은 초기화·pose0개도 통과 가능 |
| 실제 browser 합성60 frame | load/processing 성공, pose0개 | 정확도·수렴 증거 제외 |
| Browser 정상 계약 / exporter | 4개 / 3개 통과 | 별도 TODO5개는 확인된 결함 |
| 새 평가 regression | 9/9 통과 | analytic fixture; 공개 dataset 및 실기기 검증과 구분 |
| 개발 도구 | doctor/JS·shell parse/MCP/C++ LSP 성공 | clang-tidy/clang-format/TypeScript lint 미설치, 해당 통과 주장 제외 |

## 데이터·환경·보존

- 사용자 데이터 기준 경로: `/mnt/backup/SLAM`, 읽기 전용 사용; [datasets.md](datasets.md) inventory 참고
- TUM VI 7개 archive: room1~4·slides1·magistrale1의 CSV/calibration·image sample 확인, outdoors1 내용 미검증; room4를 작은 validation 후보로 분리
- EuROC MH01~05 nested ZIP, Redwood, TartanAir 확인; 현재 VIO 입력·GT 계약 충족 여부와 변환 필요성은 inventory에 분리
- Repo room1 fixture: 공식 TUM VI 자료를 checksum 확인 후 복구, video2,821/IMU28,122/GT16,541
- 출처·MD5·SHA256·CSV hash: [dataset-provenance.json](dataset-provenance.json), [공식 TUM VI](https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset)
- 공식 다운로드는 NAS 자료 부재 시 fallback; 기존 NAS·repo 자료 자동 덮어쓰기 없음
- 기존 tracked diff 보존: [preservation.json](preservation.json); 시작 binary diff SHA256와 종료 SHA256 일치(.gitignore의 이번 추가 제외)
- 원본 snapshot: `build/audit-baseline/initial-diff.patch`, [initial-status.txt](initial-status.txt)
- 기존 알고리즘·배포 JS·데이터·TLS 키·서비스 보존; 새 harness·문서·프로젝트 설정만 추가

## 상세 문서·재현 명령

- [구조](architecture.md), [수학·평가 계약](estimator.md), [native 실행](native.md), [browser 실행](browser.md), [도구 상태](tooling.md)
- [개발 runbook](../../../scripts/dev/README.md), [장기 계획](../../refactoring-plan.md), [프로젝트 AGENTS](../../../AGENTS.md)
- 정량 JSON: `native-room1-full-metrics.json`, `native-room1-600-metrics.json`, `native-room1-until-image-600-metrics.json`, `native-room1-pnp-600-metrics.json`, `browser-tum-active-metrics.json`, `browser-tum-fresh-metrics.json`, `evaluation-crosscheck.json`

```bash
bash scripts/dev/check.sh doctor
bash scripts/dev/check.sh quick
MOBILE_SLAM_COMPILE_DB=build/audit-native bash scripts/dev/check.sh clangd
bash scripts/dev/check.sh mcp
python3 -m unittest discover -s tests/browser -p 'test_*.py' -v
node --test tests/browser/contracts.test.mjs
```

- production 판정 미완료: 실제 mobile 입력·GT·지원 기기·허용 오차·지연·연속 실행·복구 기준 미확정
- 다음 수정 단계: 모바일 관측 도구와 공통 입력/평가 계약으로 실패를 재현한 뒤 원인별 최소 변경·회귀·실기기 재검증

## 실제 모바일 관측 도구

- 페이지: `web/audit-mobile.html`, 동일 module URL의 기존 live app을 same-origin iframe에서 관측
- 실행·phone HTTPS·기록·내보내기: [runbook](../../../scripts/dev/README.md)
- 검증·필드·한계: [mobile-audit.md](mobile-audit.md)
- 저장: 숫자·metadata만 로컬 JSON; parent/iframe performance timeOrigin, sensor/event/callback 시각, 적용 calibration·해상도·회전, frame accept/drop·Worker roundtrip, pose delta·reset·권한/activation 관측
- 현재 default camera: portrait, `enableLandscapeCrop` 호출 주석으로 crop `none`; 과거 v9 crop 설명과 실제 호출 경로 구분
- 제한: 기본60/최대120초, channel당10,000/전체40,000 samples, 영상·image·device ID·TLS 자료 저장 제외
- 직접 측정 제외: 물리 exposure/capture-to-pose 지연, 독립 engine timestamp·queue 시간 분해, 현장 GT 오차; 데이터셋 재생 가능한 완전 image/sensor recording과 구분
- Observer overhead: hook/copy·UI·iframe 영향 가능; 일반 앱과 관측 앱의 같은 device trial 비교 필요
- 기존 app의 iOS 권한 요청이 async 준비·대기 이후인 경로 확인; activation 상실 가능성을 관측하며 실제 iOS 실패 원인으로 단정 제외
- 이번 검증: PC HTTPS·mock sensor·실제 app hook·JSON download·종료·공개 asset/POST 차단; 실제 Android/iOS 관측·사용자 pose 점프 원인·개선은 미검증
