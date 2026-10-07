# Mobile-SLAM 장기 리팩토링 계획

- 기준일: 2026-10-06
- 목표: 모바일 웹 브라우저의 실시간 metric 위치·자세 추정, 반복 가능한 정확도·지연·안정성 검증
- 최초 요청 범위: 기존 코드·구조·성능 병렬 진단, 개발·테스트 도구 설정, 후속 구현 계획
- 이후 승인된 구현·HTTPS7002·가상 검증 완료 결과(2026-10-07): `docs/refactor/2026-10-06/README.md`; 아래 초기 진단/장기 production gate는 역사·후속 계획으로 유지
- 이번 단계 산출물: `docs/audit/2026-10-06/`, 프로젝트 도구 설정, 이 계획, PRD·테스트 명세
- 후속 알고리즘 수정: 회귀·평가 계약 확보 후 단계별 진행
- 보존 대상: 기존 미커밋 변경, 데이터셋, 참조 코드, 빌드 산출물, 기존 서비스
- 확인 필요 항목: 대상 기기·OS·브라우저, 허용 위치·자세 오차, 지연, 연속 실행 시간, 전체 SLAM 기능 범위
- 사용자 확인: PC에서는 좋아 보여도 실제 모바일 camera pose가 크게 튀거나 불안정; 현재 실제 phone 미연결·실패 원인 미확정
- 데이터 canonical home: `/mnt/backup/SLAM` 읽기 전용 inventory·manifest 참조; 원본 수정·이동·삭제 금지
- 판정 제한: 단위 테스트 통과·WASM 로딩·화면 이동만으로 production 품질 판정 불가

## 1. 현재 진단과 증거

| 항목 | 현재 확인 | 의미·후속 조치 |
|---|---|---|
| 코드 상태 | 시작 시 다수 tracked·untracked 변경; `initial-status.txt`, `build/audit-baseline/initial-diff.patch` 보존 | Git HEAD만으로 baseline 식별 불가; 실제 소스·설정·바이너리 fingerprint 필요 |
| Native 빌드 | 신규 audit build 성공; `native-build.log` | 현 소스의 native 컴파일 가능성 확인; 브라우저 정확도 증거와 분리 |
| Native 회귀 | 초기 fresh Release: 6 CTest 실행 파일, 27 GTest 중 24 통과·3 dataset fixture 실패; `native-ctest.log` | 복원 전 결과. 복원 후 parity 별도 실행은 1 통과·2 실패; `native-parity-restored-dataset.log`; 전체 회귀 통과 claim 금지 |
| Dataset 복원 | 공식 TUM VI room1 MD5 일치·SHA-256 기록, camera 2,821·IMU 28,122·GT 16,541; `dataset-provenance.json` | 현재 fixture 확보; 단일 development 진단 sequence이며 독립 최종 test 아님 |
| Native 경로 | `VIOSystem` 기반 실행 | 브라우저 경로와 동일 알고리즘 검증 필요 |
| Browser 경로 | `VIOEngine` + PnP 경로, Worker·WASM | native 실행 결과를 browser 결과로 대체 불가 |
| 정확도 평가 | 현재 native evaluator의 Sim(3) 정렬·rotation 0·시간 association 문제 | 기존 ATE 숫자를 metric 위치·자세 정확도로 채택 금지 |
| 독립 scorer | 이번 진단용 evaluator에 known-truth 회귀 9개 통과; `evaluation-regression.log` | native 기존 evaluator의 정확성·실제 VIO 정확도와 별도 증거 |
| 기존 테스트 | 0 pose·0 GT match를 충분히 배제하지 못하는 조건 | 실패를 통과로 기록할 가능성; 평가 계약 회귀 우선 |
| Browser smoke | 기존 15 assertion 통과; 별도 합성 60 frame은 초기화 0·pose 0·map 0 | 최적화·tracking·정확도 benchmark에서 제외 |
| Native calibrated replay | fresh VIOEngine 2,821 frame·2,776 valid pose(98.4048%), init frame 45 / 2.250179s | 독립 audit harness + bracket IMU + 검증된 calibration; 기존 parity test의 합격 대체 불가 |
| Browser bounded replay | active/fresh 각 601 frame·556 pose(92.5125%), init frame 45 | 실제 Worker·WASM replay 확인; workstation headless·단일 구간·repeat 미동결 |
| 실제 모바일 증상 | 사용자 보고: camera pose jump·불안정, PC 결과와 차이 | 직접 mobile camera·IMU·pose 기록 전 원인 확정 금지; P1 최우선 재현 대상 |
| Calibration | parity `.data()` column-major·destination alias 재현, `||RᵀR−I||` 0.04349; `estimator.md` E03 | 기존 parity 입력 훼손 확인; 별도 row-major buffer의 browser calibration까지 결함 전이 금지 |
| 복구·장기 안정성 | 이전 reset·recovery 관련 위험 | 손실·재초기화·좌표 연속성·메모리 누수 시험 필요 |
| 전체 SLAM | loop closure·영속 지도·재지역화 검증 부재 | 현재 VIO와 최종 SLAM 기능 gate 분리 |
| Build 신선도 | 신규 WASM build 성공, active artifact와 SHA-256 차이 | 신규 build와 실제 브라우저 active artifact를 구분; source → compile flags → 로딩 hash 연결 필요 |
| 배포 provenance | root MIT 표기와 compile 대상 GPLv3 reference 차용 관계; `architecture.md` | A/B 양쪽 동일 license gate; 상용 배포 허용 여부 미판정 |

- 진단 중 추가 결과: 날짜별 audit 보고서와 실행 로그를 우선 참조
- `docs/analysis-report.md`: 2026-03-10 과거 분석; 현재 구현·실행 결과의 대체 근거로 사용 금지
- 초기 dataset fixture 실패: 자료 복원 후 재실행 결과와 구분; 다운로드 성공만으로 trajectory 검증 완료 처리 금지

### 현재 측정값 — 제품 합격 기준과 분리

| 경로·범위 | 측정값 | 제한·근거 |
|---|---|---|
| Native 전체 room1 | input 141.004s / wall 62.151s, process p50/p95/p99 19.126/25.452/29.764ms | Desktop max-speed file replay, capture·live sensor·display latency 제외; `build/audit-baseline/native-replay/room1-bracket-full/summary.json` |
| Native 전체 room1 정확도 | SE(3) ATE RMSE 0.555253m, rotation APE RMSE 4.785254°, 1s RPE RMSE 0.160278m / 0.743111°, GT associated 2,719 | camera pose→body GT 변환, fixed scale 1; 단일 development sequence; `native-room1-full-metrics.json` |
| Browser 첫 601 frame 정확도 | SE(3) ATE RMSE active 0.282962m / fresh 0.282962m, 1s RPE RMSE 약 0.078070m / 0.881804° | 전체 room1·독립 final·실기기 결과 아님; `build/audit-browser/tum-{active,fresh}-600/metrics.json` |
| Browser Worker roundtrip | p95 active 30.735ms / fresh 28.750ms | 제출→result, capture·decode·render 제외; max-speed desktop; physical latency 또는 통계적 개선 claim 금지 |
| Active↔fresh 행태 비교 | 같은 bounded 입력 pose matrix 최대 절대 차이 `3.126e-7` | SHA-256은 다름; 미리 고정한 tolerance·repeats 없음; G2 합격·source/artifact 완전 parity 판정 불가 |

- 현재 결과: 독립 scorer 기반 정량 baseline 확보; G2·전체 production acceptance 미통과
- 단일 calibrated replay의 valid pose·coverage: 물리 정확도·tracking 상태 신뢰성·실기기 coverage와 구분
- Sim(3) fitted scale native 0.833508 / browser 약 0.892802: diagnostic 결과, metric SE(3) 수치 대체 금지
- 비교 제한: native 2,821 frame과 browser 601 frame 수치를 같은 범위의 개선 결과로 직접 비교 금지
- PC calibrated dataset 성능: 실제 모바일 jump의 원인 규명·개선 검증 대체 금지; mobile 동일 기록의 변경 전/후 비교 필수

## 2. 결정 원칙과 대안

### RALPLAN-DR

| 원칙 | 적용 기준 |
|---|---|
| 측정 신뢰성 우선 | GT·시간·좌표·coverage 검증 전 정확도 개선 수치 채택 금지 |
| 하나의 추정기 계약 | 동일 sensor replay를 native·WASM·browser에서 비교 가능 |
| 삭제·재사용 우선 | 중복 경로·숨은 fallback 제거; 기존 API·유틸 재사용 |
| 제한된 계산량 | bounded queue·window·map, profiler 증거 기반 최적화 |
| 독립 평가 | development·engine-selection validation·locked final test 3분할; GT로 online 추정기 보정 금지 |

| 우선순위 | 결정 요인 |
|---|---|
| 1 | 위치·자세·metric scale 정확도와 실패 검출의 신뢰성 |
| 2 | 모바일 browser sensor 가용성·timestamp·calibration 품질 |
| 3 | 연속 실행 지연·메모리·열화, 라이선스·WASM 배포 가능성 |

| 대안 | 장점 | 비용·위험 | 선택 조건 |
|---|---|---|---|
| A. 현 VIO를 단계별 복구·통합 | 기존 C++·WASM 투자 재사용, 작은 회귀 단위, 원인별 ablation 가능 | 서로 다른 실행 경로·누적 수학 오류 정리 필요; GPLv3 차용 provenance·full SLAM 추가 비용 | 평가·sensor 계약 복구 후 engine-selection validation 수렴 가능, 의존성·라이선스 사용 가능 |
| B. Pinned upstream 비교 후 WASM 후보 이식 | VINS-Mono native reference의 알고리즘·회귀 자산, AlvaAR WASM 후보의 browser 이식 구조 비교 | ROS·thread·sensor adapter 이식 비용, A와 동일 GPLv3 사용 조건, bundle·메모리 제약; AlvaAR가 VIO 동등 후보인지는 별도 판정 | 같은 engine-selection validation에서 A의 목표 달성 불가 또는 유지비 우세; 사용·배포·VIO 동등성 확인 |
| C. 제품 범위를 제한한 browser VO/VIO | 지원 기기·환경 축소로 측정 가능한 품질 기준 확보 | 일반적인 모바일 환경·전체 SLAM 요구 충족 범위 제한 | browser IMU·calibration으로 정한 목표를 달성할 수 없는 경우, 사용자 범위 결정 필요 |

- 우선 선택: A의 평가·입력 경계 복구까지 진행; 엔진 전체 교체 결정은 P2 종료 evidence 이후
- 공정 비교: 같은 engine-selection validation 입력·camera profile·시간 구간·좌표·장비·지연 정의 사용; locked final test는 엔진 선택에 사용 금지
- Pinned 비교 대상: 로컬 VINS-Mono `90dabb5ec79946ae42fd2e1e91d4e69aabe1e25d` native reference, AlvaAR `7796af500ee92001ac2a9888363ff64d7a3bee75` WASM feasibility candidate; 실제 실험 전 dirty status·파일 hash·dependency revisions 추가 고정
- B 역할 구분: VINS-Mono는 native reference, AlvaAR는 WASM feasibility candidate; native reference의 성공을 AlvaAR의 metric VIO 동등성·제품 적합성으로 전이 금지
- A/B 비교표 필수: 파일별 license/provenance·native/target peak memory·JS/WASM bundle bytes·build 재현성·필수 API/thread/ROS porting 작업·작업시간·정확도·coverage·latency. 미측정 cell은 `unverified`
- A 진단 예산: G1 완료 뒤 최대 3개 원인군, 원인군당 최대 2개 최소 수정 후보; 총 6개 후보 상한. 각 후보의 입력·반복·시간 상한은 P2 entry manifest에 사전 고정
- A→B 재평가 기준: 고정된 init·coverage·translation/rotation·memory/bundle gate 미달이 예산 소진 뒤 지속, provenance 사용 조건 불충족, 또는 동일 validation에서 B의 사전 고정 비용·품질 기준 우세
- Switch 결정: P2 시작 전 위 조건의 수치·비교 우선순위·예산을 동결; 결과 확인 뒤 기준 완화 금지. B도 기준 미달이면 C 범위 결정 evidence로 보고
- 라이선스 조사: 실제 참조 파일·의존성별 provenance와 사용 조건 확인; 인터넷 예제·복원된 proprietary 코드의 제품 편입 제외
- WebGPU·AI frontend·추가 엔진 도입: 기본 해법으로 가정 금지; profiler·ablation·배포 조건으로 판단

## 3. 필수 실행·데이터 계약

| 경계 | 필수 계약 | 실패 조건 |
|---|---|---|
| Camera | timestamp API·clock domain·provenance·physical exposure 여부·uncertainty, frame sequence·resolution·orientation·calibration ID; pixel format·stride | presentation/callback/arrival time을 physical capture time으로 표시, K와 해상도·회전 불일치 |
| IMU | monotonic 공통 시간축, accel `m/s²`, gyro `rad/s`, 축·좌표·중력 포함 여부 명시 | 단위·축 미확정, 비단조 timestamp·큰 gap·중복 이벤트 은폐 |
| 동기화 | camera/IMU clock mapping·offset·불확실성 기록; frame 구간에 맞춘 IMU drain | dequeue 시간으로 sensor time 대체, offset을 GT로 online 보정 |
| Calibration | K·distortion·camera model·`T_B_C`·센서 noise·profile 출처; 실제 배열 값 검증 | 임시 메모리 포인터·row/column order 오류, 출처 없는 mobile intrinsic 고정값 |
| 추정기 | 동일 `configure / input / output / reset` 계약; 입력 순서·thread 정책·seed | native·WASM에 별도 수학 경로·숨은 fallback 존재 |
| Pose | `T_W_B` 정의, 오른손 좌표계·m·quaternion `xyzw`, sample timestamp·state·session ID | 무효 pose를 유효 tracking으로 표시, reset 전후 gauge 변화 미표시 |
| 상태 | `initializing / tracking / degraded / lost / recovering / unsupported`; 원인 코드 | 0 pose·NaN·timeout을 성공으로 처리, tracking loss 표시 지연 |
| Queue | bounded 용량·drop 정책·기록; 최신 입력 우선 여부 문서화 | backlog 무한 증가, drop된 frame·IMU 은폐 |
| Artifact | source hash·dirty diff hash·compile flags·dependency versions·JS/WASM SHA-256 | 테스트 바이너리와 배포 artifact의 출처 불명·stale cache |

- Pose matrix 저장 순서와 JS 전달 순서: 한 곳에서 정의, golden rotation·translation fixture로 검증
- Camera rotation·device orientation·IMU body frame 변환: 조합별 synthetic fixture와 실제 sensor 기록 비교
- 추정기 confidence: 품질 지표와 calibration 근거 확보 전 probability 명칭 금지; feature 수·innovation·상태 등 관측 가능한 값 표시
- 센서 가용성 미충족: `unsupported` 또는 기능 제한 표시; metric VIO 품질 claim 금지
- Timestamp 구분: physical exposure start/midpoint·media presentation time·`requestVideoFrameCallback` callback time·main/Worker arrival time·pose publish·display presentation 각각 별도 필드
- Camera physical capture timestamp 불가 시: presentation-to-pose·arrival-to-pose proxy latency만 보고; capture-to-pose 합격 판정은 `unverified`
- Clock uncertainty: browser camera/IMU/main/Worker clock mapping의 offset·drift·timestamp quantization·jitter·rolling shutter exposure interval 기록
- 물리 지연 검증: 외부 계측 clock 기준 LED/화면 이벤트·high-speed 영상 또는 mocap motion event로 capture·render 시점을 대응; 계측 정확도·동기화 잔차가 latency gate보다 충분히 작은지 확인

## 4. 검증 gate와 성능 지표

### 고정 gate

| Gate | 통과 조건 | 실패 조건 |
|---|---|---|
| G0 재현성 | 새 native·WASM build, 로딩 artifact hash, 명령·버전·환경·입력 manifest 기록 | 기존 binary만 실행, build failure·stale module·필수 fixture 누락 |
| G1 평가 신뢰성 | 알려진 SE(3)·scale·rotation·timestamp fixture 기대값 통과; 별도 구현 cross-check; 0 pose·0 match 명시 실패 | Sim(3)로 scale 오류 은폐, rotation 0 고정, 잘못된 association·중복 match |
| G2 Replay | P2 entry manifest에 고정한 입력 counts·길이·반복·translation/rotation/init/coverage tolerance 만족 | tolerance 미설정은 `unverified`; VIOSystem 통과·초기화 이전 smoke로 합격 대체 |
| G3 정확도 | held-out test의 ATE·RPE·rotation·scale drift·coverage·실패율 모두 사전 기준 만족 | 실패 구간·reset·GT 없는 구간을 임의 제거, scene 평균만 보고 |
| G4 실시간 | 실제 지원 기기에서 timestamp provenance·외부 clock uncertainty와 capture-to-pose/display p50/p95/p99, backlog·drop·memory·열화 확인 | physical capture 불명은 proxy 결과로 분리; 데스크톱·processing time으로 제품 latency 대체 |
| G5 복구·지속성 | 센서 gap·blur·저텍스처·앱 background 후 상태 전이·복구·메모리 한도 검증 | NaN·crash·무한 queue·과거 pose 정체를 tracking으로 표시 |
| G6 전체 SLAM | loop closure·relocalization·map reuse 요구별 별도 held-out 검증 | VIO 실행만으로 전체 SLAM 완성 표시 |

### 정확도 보고 계약

- 위치: meter 기반 ATE RMSE·median·p95·max, SE(3) rigid alignment 기본
- Sim(3): scale ambiguity 진단용 별도 결과; metric VIO production gate 대체 금지
- 자세: quaternion 상대 회전 geodesic angle, degree 단위 RMSE·p95·max
- RPE: 고정 시간·거리 간격별 translation·rotation, 실제 pair 수·가용 구간 기록
- Scale: metric translation 대비 scale ratio·구간 drift; scale fitting 전 결과 포함
- Coverage: GT 가용 평가 구간의 eligible frame을 분모로 pose·match·tracking coverage 별도 보고
- 초기화: 시작부터 유효 tracking까지 시간·실패 횟수·입력 조건; 초기화 구간 제외 기준 manifest 고정
- 시간 association: 사전 tolerance, one-to-one match·중복 방지·잔차 분포; interpolation 정책 고정
- Reset: session·구간별 결과와 전체 정렬 결과 모두 기록; 구간별 새 origin으로 drift 은폐 금지
- GT 미제공·불확실 구간: 성능 로그는 유지, 정확도 평가 범위·불확실성 명시
- Aggregate: sequence별·device별 결과, 실패 포함 집계; 빈 결과·비유한 값은 합격 처리 금지

### P2 entry 동결 계약

- 소유: Leader + Native QA + Browser QA; `config/validation/p2-contract.yaml` 작성 예정
- 결과 관찰 전 고정: engine/source/artifact/input hash·camera profile·clock policy·coordinate·alignment·float policy·warmup·seed
- 입력량: camera frame count·IMU count·구간 시작/끝·총 길이·drop 허용량·replay wall-time 상한·반복 횟수
- 수치 허용치: native↔WASM translation p95/max `m`, rotation p95/max `degree`, init 시간차 `s`, init 최소 성공률, valid pose/tracking coverage 최소·차이 상한
- 비교 frame: 동일 sequence·sample timestamp association; `T_W_B` 또는 검증된 `T_W_C`를 한 기준으로 변환. metric scale fitting 금지
- 현재 값: 모두 미확정; `null`·단위 누락·입력 count 불일치·repeat 부족은 G2 `unverified` 또는 계약 실패
- 비교 수치는 development pilot·수치 정밀도 한계에서 제안, engine-selection validation 실행 전에 동결. Validation 결과를 본 뒤 tolerance 조정 금지
- 변경 필요 시: 새 contract version·근거 기록, 이전 결과 판정 보존, 새 validation 수행; locked final test에 적용할 기준은 final 실행 전에 고정

### 수치 목표 상태

| 항목 | 초기 검토 제안 | 확정 시점 |
|---|---|---|
| 지원 기기·브라우저 | 대표 Android Chrome·iOS Safari 실기기 각 1종 이상, 최소·목표 기기 구분 | 대상 기기 응답·sensor capability 조사 후 |
| 연속 실행 | 30분 sustained trial 제안; foreground·background/resume 구간 분리 | 제품 사용 시간 확인 후 |
| 유효 tracking coverage | GT 가용 구간 ≥95% 제안; 초기화·loss 별도 기록 | 독립 baseline·사용 환경 확인 후 |
| 출력 갱신 | 실제 camera input 대비 유효 pose 갱신율 보고; 초기 20Hz 검토 | 기기·해상도·목표 interaction 결정 후 |
| 지연 | capture-to-pose p95 ≤100ms 검토; display latency 별도 | sensor timestamp 접근성·기기 baseline 후 |
| 위치·자세 오차 | 절대 meter·degree 및 RPE 기준 미정; 환경·작업 허용 오차부터 정의 | 사용자 목표·GT 정확도·baseline 조사 후 |
| 메모리·bundle·열화 | 실기기 관측 후 상한 수치 결정; queue·window·map은 항상 bounded | P2 baseline 확보 후 |

- 위 숫자: 요구사항 확정치·현재 달성치·상용 표준 수치가 아닌 초기 검토 제안
- 확정 PRD: 지원 matrix·허용 오차·지연·coverage·복구 시간·연속 실행·메모리 한도·GT uncertainty 포함
- 성능 최적화 채택: 동결된 G2 tolerance·입력량·반복 gate와 동일 정확도·coverage·복구 기준 유지 + 목표 기기 latency·memory 개선 evidence

## 5. 단계별 구현·검증

| 단계 | 작업·소유 경계 | 필수 검증 | 종료·중단 기준 |
|---|---|---|---|
| P0 진단·도구 설정 | 현 상태 보존, 구조·수학·native·browser·도구 병렬 audit, provenance·라이선스 inventory | 신규 build·기존 test·browser smoke·MCP 실호출·도구 버전·로그 | 실행 가능/실패/미검증 분리 보고; 환경 실패에 대안 경로 기록 |
| P1 모바일 재현·평가·sensor 계약 | 실제 mobile observation/recording 먼저, stationary·축 회전·이동·drop·pause 진단; 동일 입력 replay, evaluator·timestamp·단위·좌표·K·reset 회귀·field GT feasibility | 실제 device capability·sensor/profile/state 기록, 변경 전/후 모바일 기록 replay, known-truth unit·외부 계측 pilot | G1 통과; mobile data 미확보 시 증상 원인·개선 미검증; GT infeasible이면 field 정확도·physical latency gate 보류 |
| P2 실행 경로 통합 | shared core→same-input adapters→VIOSystem delegation→중복 삭제; P2 entry contract 동결 | native↔WASM tolerance·input counts·repeats, initialization·map·pose, camera/body compatibility·recovery | G2 통과·engine-selection validation baseline 확보; 고정 A 예산·switch 기준 적용 |
| P3 정확도 복구 | init·preintegration·bias/gravity·marginalization·optimizer 상태를 실패 evidence 순서로 수정 | 독립 fixture·Jacobian·conditioning·sequence별 ATE/RPE/rotation/coverage, ablation | G3 사전 기준 통과; 최종 test 반복 튜닝·GT leakage 발생 시 판정 무효 |
| P4 모바일 실시간화 | frame capture·copy·queue·Worker scheduling·solver budget·SIMD 병목 개선 | profiler trace·동일 replay 정확도 회귀·실기기 sustained latency/memory/thermal | G4·G5 통과; 속도 향상과 정확도·coverage 악화 동시 발생 시 변경 폐기 |
| P5 전체 SLAM | 합의된 loop closure·relocalization·map persistence 기능, 지도 크기·version·좌표 정책 | repeated route·false loop·map reload·cross-session 실험 | G6 통과; 지원 안 되는 기능은 명시 |
| P6 출시 연구 판정 | 지원 matrix·field held-out·권한 실패·보안·provenance·배포 artifact 문서 | 사전 고정 test set·실기기·장기 실행·release 후보 hash | 모든 필수 gate + 라이선스 조건 충족 후 production-ready 판정 |

- 순서: P0 → P1 → P2 → P3/P4 비교 실험 → P5 → P6
- P3/P4 병렬 허용: 서로 다른 파일·실험 variant와 고정 evaluator 사용, 결과 통합은 순차
- 각 수정 단위: 실패 재현 → 독립 기대값 회귀 → 최소 수정 → unit/integration → 필요한 replay·browser 확인
- Cleanup: 작업 전 작은 계획, 기존 동작 보호, 삭제·공통 경계 재사용 우선; 새 runtime dependency는 별도 결정
- 이미 passing인 테스트 반복: 새 변경·새 실패·미해결 위험이 있을 때만 확대

### P1 첫 작업 — 실제 모바일 jump 분리

- 잠정 대상: Android Chrome·iOS Safari의 실제 capability 기준 별도 matrix; 구체 device/model·OS·browser·camera mode 응답 전 지원 확정 금지
- 소유: Browser QA가 dev-only observation recorder·기기 capability·camera/IMU/state 로그, Estimation이 좌표·단위·calibration·reset 분석, Native QA가 동일 입력 replay·지표 비교
- 현재 준비: dev-only observation recorder 구현·PC HTTPS/mock·로컬 JSON 다운로드 검증 완료; 실제 phone camera·sensor 기록·현장 GT·jitter/latency 실측 미확보
- Recorder 결과 구분: timestamp·metadata·pose만 있으면 observation, image+raw IMU+profile+input ordering이 있으면 deterministic replay. 관측 로그를 완전 replay로 표시 금지
- 기록 계약: device/browser·권한·센서 API 종류, 실제 resolution·crop/resize·orientation·K profile, raw accel/gyro·gravity 포함 여부·단위/축, sensor/presentation/arrival clock, frame sequence·drop·queue·pose age·state/reason·reset/session
- 구현 순서: capability 확인 → 변경 전 mobile observation → 필요한 image/IMU recorder → same-input native/WASM/browser replay → 원인별 작은 회귀·ablation → 같은 mobile 조건에서 변경 후 측정

| Mobile 진단 | 제어·기록 | 비교·실패 신호 |
|---|---|---|
| Stationary | 장치를 고정 지지대에 고정·정적 scene, 시작/초기화/유효 구간 구분 | pose translation/rotation jitter·drift·pose age·reset, gyro bias·accel gravity·sample interval; 정지 중 jump/무효 pose 추적 |
| 축별 회전 | body 각 축 회전, portrait/landscape 조합·camera crop 변화를 개별 기록 | axis sign·deg/rad·gravity 회전·`T_B_C`·lever-arm·출력 좌표 오류; 외부 계측 없는 수동 각도는 정확도 GT 아님 |
| Translation | 축별 이동·정지·재이동, 가능하면 외부 reference·검증된 거리 | scale·방향·init·tracking loss·pose jump; 계측 불확실성 없는 수동 이동으로 metric 합격 판정 금지 |
| Drop·gap | frame/IMU drop·delay를 각각 주입, overload 원시 순서 보존 | bounded queue·retained IMU 순서·gap/reason·stale pose·reset 전후 prior/session; drop을 정상 처리로 집계 금지 |
| Pause·resume | camera pause·tab background·권한 변경·장치 회전·resume | clock discontinuity·orientation/K 갱신·reset/recovery·stale map/pose·waiter 정체 |

- 실험별 길이·반복·계측 기준: recording manifest에 사전 고정; 현재 실기기 수치·통과 목표 없음
- Ablation 분리: latency/queue → orientation·crop-K → IMU clock → accel/gyro 단위·축·중력 → extrinsic → reset/prior. 한 번에 한 원인 후보 변경, 동일 모바일 기록 비교
- Mobile 기록 provenance 불충분: 후보 원인·재현 한계로 보고; 특정 알고리즘·sensor 오류 확정 금지
- Mobile 개선 판정: 동일 device·profile·동작·기록 범위의 jump/jitter·coverage·reset·pose age·GT 오차 비교; PC room1 통과로 대체 금지
- 허용 수치·장비 미정: 원인 분리·실행 evidence만 수집, production 합격 선언 금지

### 되돌릴 수 있는 통합 순서

1. 공통 CMake core source·추정기 입력/출력 계약 생성; 기존 VIOSystem·VIOEngine 진입 API 유지
2. Native dataset·browser adapter가 동일 camera/IMU/profile replay 입력 생성; 차이는 platform I/O에 한정
3. `VIOSystem`이 같은 core에 위임; viewer·logger·camera pose consumer의 이전 의미를 compatibility adapter로 유지
4. 같은 입력·pose·state·복구 parity gate 통과 후 사용처 없는 orchestration·source 목록·helper 중복 삭제

- Pose migration: 현재 consumer가 `T_W_C` 기대 시 `T_W_C = T_W_B · T_B_C` 변환; API field·문서에 frame 명시
- Lever-arm 회귀: `t_B_C ≠ 0` + 90° body rotation으로 camera 위치 변화와 body 위치 불변 확인; Three.js·trajectory logger·map consumers 각각 검사
- 경로 선택: release manifest의 `engine_path` flag로 legacy/shared-core variant 전환; 테스트되지 않은 자동 fallback 금지
- Rollback 자산: 마지막 합격 JS/WASM bundle·config·manifest hash 보존, 새 candidate 별도 경로 배포·cache version 분리
- Rollback 기준: P2 동결 tolerance·coverage/init 실패, camera/body 변환 실패, crash/NaN, queue/memory 상한 초과, 사전 latency regression 상한 초과
- Rollback 수행: path flag·bundle/config manifest를 마지막 합격본으로 전환 후 hash·smoke·affected replay 확인; Git reset·데이터 삭제 금지

## 6. 데이터·baseline·ablation

- Canonical 데이터 위치: 사용자 지정 `/mnt/backup/SLAM`; 읽기 전용으로 dataset·sequence·format·GT/calibration 가용성·checksum inventory
- 현 repository room1: 이번 audit용 local fixture; canonical NAS의 원본을 대체하거나 전체 데이터 준비 완료로 표시 금지
- Split 후보: NAS inventory에서 실제 GT·IMU·camera profile·license·sequence/session 확인 후 development/selection/final 후보 배정; 현재 후보 배정 보류
- Replay 접근: 원본 read-only 경로 직접 사용 또는 manifest로 검증한 별도 local cache; NAS 원본 이름 변경·재압축·덮어쓰기 금지

### Development / engine-selection validation / locked final test 3분할

| 구분 | 사용 | 관리 |
|---|---|---|
| 작은 deterministic fixture | 좌표·IMU 적분·time match·reset·이상값 unit regression | 코드와 기대값 검토; 현실 정확도 지표로 사용 금지 |
| 공개 development replay | EuRoC·TUM VI의 권한·calibration·GT 가용 구간 확인 후 튜닝 | dataset·sequence·checksum·split manifest; 자료 미확보는 blocked fixture로 기록 |
| Engine-selection validation | A/B 엔진 선택·hyperparameter 채택·trade-off 판단 | development와 sequence/scene·session 분리; tolerance·switch 기준·반복 횟수 사전 동결 |
| Locked final test | 엔진 선택·튜닝 종료 후 최종 정확도·안정성 판정 | selection에 사용 금지; split·criteria·artifact 잠금, 실패 sequence 포함 |
| 실제 모바일 개발 기록 | stationary·축 회전·translation·drop/pause, camera/IMU clock·orientation/crop-K·profile·state | P1 우선, observation과 완전 replay 구분, device/session별 split·보존 정책 |
| 독립 field 기록 | 다른 장소·조명·motion·기기·session + 외부 reference | 개발·선택·최종 3분할 적용, GT 정확도·동기화·좌표 보정 근거, 추정기 입력과 분리 |

- 다운로드 데이터: 공식 출처·사용 조건·크기·checksum·압축 해제 manifest 기록
- Dataset GT가 estimator 입력·scale 보정·온라인 parameter 선택에 섞이는 경로 금지
- 공개 dataset 성능만으로 실제 휴대전화 rolling shutter·IMU timestamp 품질을 대신 판정 금지
- GT 계측 방식: 요구 오차보다 충분히 작은 계측 불확실성·clock uncertainty 확보; 불충족 시 정량 합격 보류
- 분할 manifest: scene·device·recording session·sequence ID·checksum·GT 구간·split·허용 사용 목적 포함; 인접 구간 분할로 같은 session leakage 방지
- 최종 test 봉인: engine 선택·튜닝 종료 후 artifact·기준 고정. 결과 확인 후 수정 시 기존 판정 보존하고 새 final set 필요

### Field GT feasibility 작업 — P1

| 작업 | 장비·방법 후보 | 소유·산출물 |
|---|---|---|
| 계측 가능성·예약 | optical mocap+rigid marker 또는 surveyed AprilTag frame+동기화 외부 camera; 정확도·capture volume·occlusion 비교 | Native QA/평가 담당 + Leader; 장비 접근·비용·권한·수용 환경 조사표 |
| Rigid extrinsic | phone body↔marker `T_B_M` calibration; 여러 rotation·translation pose와 별도 검증 pose | Estimation + 평가 담당; 값·방법·원자료·translation/rotation uncertainty |
| 좌표 frame | mocap world 또는 surveyed tag world의 metric 좌표·scale·survey uncertainty | 평가 담당; `T_W_G`·계측 기록·검증 residual |
| Clock·latency | 공유 trigger/LED/high-speed event 또는 사전 clock offset/drift 측정; browser timestamp proxy와 물리 event 대응 | Browser QA + 평가 담당; offset/drift·quantization·jitter·association 잔차 |
| Uncertainty budget | GT position/rotation·survey·extrinsic·timestamp×motion·exposure/rolling-shutter 기여 | 평가 담당; 요구 오차·지연 대비 budget, 독립 hold-out pilot 결과 |

- Feasibility gate: 계측 uncertainty budget·가용 구간·clock 방법·occlusion 처리·원자료 확보 후 field accuracy/physical latency 시험 시작
- 장비·방법 불가 또는 uncertainty가 허용 기준보다 큼: G3 field·G4 physical capture 판정 `unverified`; proxy latency와 공개 dataset 결과만 보고
- AprilTag 후보: camera marker 추적 자체의 정확도·blind spot·tag survey·외부 camera calibration 별도 검증; 추정기 camera만 이용한 자기검증 금지

### Baseline

| ID | 경로 | 비교 목적 |
|---|---|---|
| B0 | 보존된 dirty 소스·현재 browser artifact | 사용자 문제의 원형; 현상 재현·변경 전후 비교 |
| B1 | 같은 소스 신규 native `VIOSystem` | 기존 native 경로 참고; browser 대리 합격 금지 |
| B2 | 같은 소스 신규 native `VIOEngine` | browser core와 같은 알고리즘 경로 기준 |
| B3 | B2 소스·설정의 신규 WASM + browser replay | 이식·sensor adapter·scheduling 영향 분리 |
| B4 | VINS-Mono pinned native reference, AlvaAR pinned WASM feasibility candidate | 알고리즘 참고·이식/교체 비용 판단; 각각의 기능 동등성·사용 조건·입력·평가 계약 확인 |

- Artifact 비교: build flags·SIMD·float policy·solver·seed·input 순서·calibration·resolution까지 manifest 포함
- Native 속도와 browser 속도: 동일 장비에서 이식 overhead 비교, 실제 target 기기에서 제품 latency 별도 측정
- B0 artifact 출처 불명: historical reference 표시, 정확한 source parity claim 금지

### Ablation matrix

| 실험 | 한 번에 바꿀 요소 | 관찰 값 | 채택 조건 |
|---|---|---|---|
| 시간·센서 | clock mapping, IMU drain, orientation 변환 각각 | init·RPE·rotation·coverage·time residual | 독립 sensor/GT 계약 통과 |
| Calibration | 기존 값 ↔ 검증된 profile | reprojection·scale·ATE·실패율 | GT로 profile tuning 금지, calibration provenance 확보 |
| Core 경로 | 기존 native ↔ VIOEngine, PnP on/off | init·scale·state·drift | 같은 입력에서 차이 원인 설명 가능 |
| Frontend | 해상도·feature 수·LK 조건 각각 | tracks·inlier·latency·loss·accuracy | 제약 환경별 정확도·coverage 기준 유지 |
| Backend | window·solver iteration·budget 각각 | solve p95·conditioning·ATE/RPE | bounded latency + 관측 가능한 실패 검출 |
| Runtime | SIMD·copy 제거·queue/drop 정책 각각 | stage latency·memory·drop·정확도 | artifact flags 확인, 회귀 통과 |
| SLAM | loop closure·relocalization 각각 | drift·false closure·recovery·map memory | 독립 route·재방문 test 통과 |

- 반복: 고정 seed·warmup 정책·run 횟수 사전 설정; 평균·분산·worst sequence 보고
- 비교: 하나의 변경만 채택한 variant와 baseline 비교; G2의 동결 tolerance·input counts·length·repeats 적용, 미설정·repeat 부족은 `unverified`; 개선 없음·불확실 시 유지보수 비용으로 판단
- Engine-selection: development→동결된 validation 결과로 A/B·variant 채택; locked final test 이용 금지
- 최종 test: 결과를 본 뒤 재튜닝한 경우 새 locked final set 확보 후 다시 판정

## 7. 개발·테스트 도구와 운영

| 도구 영역 | 필요한 capability | 검증 기준 |
|---|---|---|
| C++ | compiler·CMake·CTest·clang-format·clang-tidy·clangd 또는 대체 compile DB consumer | 버전·`compile_commands.json`·실제 command 성공; 미설치 도구는 대체·제한 기록 |
| 수치·메모리 | ASan/UBSan native build, debug assertions, finite/Jacobian tests | 실제 sanitized test 실행; sanitizer config만으로 완료 금지 |
| WASM | Emscripten·pinned OpenCV/Ceres/Eigen·fresh build·artifact fingerprint | core compile flags와 dependency flags 각각 확인; link `-msimd128`만으로 core SIMD claim 금지 |
| Browser | DevTools/Playwright MCP·camera/IMU replay·HTTPS development server | tool 연결·실호출·browser 실행·Worker/WASM console·network 확인 |
| 문서·외부 API | Context7·공식 upstream docs·연구 출처 탐색 capability | MCP ping만으로 만족 금지; 실제 lookup 결과·account/scope·제한 기록 |
| 평가 | deterministic replay·독립 GT cross-check·성능 trace·결과 schema | missing data·invalid pose·association 오류·artifact mismatch 실패 검출 |
| 자동화 | reproducible commands·pinned dev tools·CI 단계·로그 저장 | 깨끗한 환경 재현; dataset 대용량·비밀키를 Git에 포함 금지 |

- 설정 범위: 이번 진단·수정에 필요한 capability부터 구성; 설치 여부와 실제 사용 가능 여부 별도 기록
- 프로젝트 `.codex/config.toml`·개발 스크립트·local skill: 명령·입력·출력·실패 기준 포함
- 외부 plugin·MCP: 사용 계정·접근 범위·권한·네트워크 요구·availability 기록; 미접속 서비스로 도구 준비 claim 금지
- Skill: audit / replay / WASM artifact / browser sensor 확인 절차를 반복 가능한 형태로 유지
- 오래 걸리는 build·test: background 실행 + 로그·exit code·source fingerprint 저장; 별도 추측 ETA 사용 금지
- 서비스: 진단용 port·process owner·stop 방법 기록; 기존 서비스 일괄 종료 금지
- CI: 빠른 unit → 평가 계약 → 작은 native/WASM replay → browser replay → 조건부 전체 dataset·실기기 matrix
- 최종 변경 보고: 변경 파일·삭제/통합 경계·실행 명령·결과·잔여 위험·미검증 범위

## 8. 병렬 에이전트 소유·검토 경로

| Lane | 소유·담당 | 다른 lane과의 계약 |
|---|---|---|
| Leader | scope·snapshot·baseline·통합·최종 gate | 요구·결정·실패 원인 일관성 |
| Architecture | 모듈·실행 경로·dependency·license read-only audit | core/platform 경계·중복 삭제 후보 |
| Estimation | init·IMU·calibration·optimizer·recovery 수학 audit | 알고리즘 failure fixture·최소 수정 제안 |
| Native QA | native build·CTest·sanitizer·evaluator·dataset replay | artifact·command·정량 제한 |
| Browser QA | WASM·Worker·sensor·camera·replay·실기기 | same-core 비교·stage latency·state |
| Tooling | project MCP·dev tools·skills·reproducible runner | probe evidence·scope·version |
| Planning | 이 문서·PRD·test-spec | findings·acceptance·failure·순서 반영 |

- 동시 child agent 최대 6; 필요하지 않은 lane은 종료·재사용
- Native Codex subagents: 독립 read-only audit·좁은 파일 소유 수정에 우선 사용
- 수정 prompt: 소유 파일·완료 기준·검증·공유 코드 보호·충돌 보고 명시
- 같은 core 파일 동시 편집 금지; algorithm variant는 별도 실험 경로 또는 순차 적용
- 향후 OMX team: 여러 구현 lane의 지속 조율·shared blocker가 생길 때만 사용; tmux 실행 자체가 검증 증거는 아님
- 검토 경로: planner 초안 → architect trade-off·경계 검토 → critic acceptance·failure challenge → leader 반영
- 실행 검토: unit/integration evidence → verifier claim 검토 → 실제 browser/field gate → leader 완료 판정

## 9. 위험 pre-mortem

| 예상 실패 | 조기 신호 | 예방·복구 |
|---|---|---|
| 평가기가 좋은 숫자를 만들지만 실제 추정은 실패 | Sim(3) scale correction, 0 rotation, 낮은 coverage·reset 누락 | known-truth 기대값·독립 evaluator·one-to-one time match; raw trajectory·state 로그 보존 |
| 공개 dataset 성공 후 실제 모바일에서 불안정 | 불명확한 capture time·IMU interval, K mismatch, rolling shutter·thermal·drop 증가 | 지원 capability 확인·실기기 recorder·profile 검증·latency trace; 지원 범위 축소 결정 evidence |
| 전면 교체·최적화가 유지비·배포 위험 확대 | 엔진 경로 분기·proprietary 코드 유입·GPL 조건 미확인·stale WASM | provenance inventory·core 경로 통합·작은 단계·hash 연결; 실패 변경 단위별 되돌림 |

- 회귀 발생: 마지막 합격 artifact·manifest와 비교, affected gate 재실행; 데이터·기존 작업을 삭제하는 복구 금지
- 수렴 실패: 입력·평가·초기화·conditioning을 순서대로 분리; 임계값 완화로 통과 만들기 금지
- Hardware·GT 부족: 실행 성능·정량 정확도 판정 범위 제한 명시; production 합격 보류

## 10. ADR-001 — 평가·입력 경계를 먼저 복구

- 상태: revision 2 Architect APPROVE → Critic APPROVE, 계획 검토 완료; P2 tolerance·제품 목표 동결과 implementation gate는 별도
- Context: 서로 다른 native/browser 경로, 기존 accuracy evaluator 결함, 초기 dataset fixture 누락(현재 복원 완료), 실제 모바일 정량 요구 미확정
- Decision: P1 평가·sensor 계약과 P2 same-core replay를 먼저 구축; 엔진 전체 교체·AI/WebGPU 추가는 비교 evidence 뒤 결정
- Alternative: 검증된 upstream으로 즉시 교체; 폐기 이유는 구현 자체의 부정이 아닌 미해결 sensor·평가·라이선스·WASM 비용
- Consequences: 초기 가시적 기능 확장 속도는 느리지만 변경 원인·scale·시간·복구·latency 비교가 가능
- Revisit: 독립 replay의 init·coverage·ATE/RPE·rotation 목표 미달, architecture 유지비 과다, 배포 사용 조건 충돌
- Exit evidence: 확정 PRD·dataset split·평가 fixture·native/WASM artifact parity·실기기 baseline

### Architect 검토 반영 — revision 2

- 3분할 평가: development / engine-selection validation / locked final test, 엔진 선택에 최종 test 사용 금지
- P2 entry: translation·rotation·init·coverage tolerance와 입력량·길이·반복·시간 상한 동결, 미설정 `unverified`
- Timestamp: physical capture·presentation·callback·arrival 구분, proxy latency·외부 clock uncertainty 별도 판정
- Field GT: 장비 후보·`T_B_M`·좌표 survey·시간 방법·uncertainty budget 소유와 feasibility gate 추가
- 대안: pinned VINS-Mono native reference·AlvaAR WASM feasibility candidate, A/B 동일 GPL gate·bundle/memory/porting 비교·A 6후보 예산·switch 사전 동결
- 통합: shared core→adapter→VIOSystem delegation→삭제, camera/body·lever-arm compatibility·path flag·bundle rollback 기준 추가
- 검토 상태: revision 2 Architect APPROVE → Critic APPROVE; 최종 facts refresh는 설계 변경 없이 추가
- 최종 facts refresh: 공식 dataset 복원·calibrated full native replay·bounded active/fresh browser replay·SE(3) 수치·parity 결함 범위·현재 gate 제한 반영

### 사용자 steering 반영 — 모바일 재현·NAS

- `/mnt/backup/SLAM` read-only canonical 데이터 inventory·candidate split 입력으로 지정
- 실제 모바일 camera pose jump를 P1 첫 재현 대상으로 앞당김; Android Chrome/iOS Safari capability별 관측·기록
- Stationary·축 회전·translation·drop/pause 진단과 latency/orientation-crop-K/IMU clock-units/extrinsic/reset ablation 분리
- 현재 상태: device 미연결·정량 mobile 기록 미확보·실패 원인 미확정; dev-only recorder 준비와 현장 실행 구분
- 계획 원칙·엔진 선택 gate 유지; 제품 알고리즘 수정·production 합격 선언 없음

## 11. 이번 요청의 완료 기준

- [x] 현 상태·source fingerprint·dirty diff 보존
- [x] 구조·수학·native·browser·도구 병렬 audit 보고서·명령·로그 수집
- [x] 신규 native·WASM build와 실제 browser 로딩 확인; artifact hash·제한 기록
- [x] 기존 unit·통합 test 실행; 실패 원인과 algorithm 미검증 구분
- [x] 공식 dataset 복원·native full/browser bounded replay·GT 수치·평가 제한 기록
- [x] MCP·plugin·skill·개발 도구 설정과 실제 probe; unavailable 항목·대체 기록
- [x] 현재 baseline evidence와 production에 필요한 추가 evidence 분리
- [x] 계획·PRD·test-spec에 findings 반영, revision 2 architect → critic APPROVE

- 구현 착수 기준: P1의 작은 작업 단위·소유·기대값 회귀·실패 기준 확보
- 전체 프로젝트 완료 기준: 지원 기기·현장·허용 오차를 확정한 후 G0~G6 적용 범위 모두 통과
