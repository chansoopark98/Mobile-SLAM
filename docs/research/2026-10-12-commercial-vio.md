# 상용 모바일 VIO 공개 근거·REA 조사

- 조사일: **2026-10-12**, Asia/Seoul
- 담당: commercial research lane, GPT-6.1 Sol Ultra; 제품 코드 변경 없음
- 범위: ARKit·ARCore 공식 문서, `morluto/rea` 고정 소스·배포 CLI, 공개 논문, 프로젝트 소유 JS/WASM 정적 검사
- 결과: **상용 엔진의 비공개 solver·bias 추정·tracking threshold 복구 근거 없음**. 공개 실패 모드와 입력 계약, 회전 퇴화 대응 문헌을 후속 설계 근거로 채택
- 증거: [commercial build](../../build/research-20261012/commercial/), [다운로드 manifest](../../build/research-20261012/commercial/downloads.json), [primary documents](../../build/research-20261012/commercial/primary-documents.json)

## 현재 현상과 조사 판단

- 기준: [Flip 6 최적화 후 실제 재시험](../recordings/2026-10-12-zflip6-after-optimization.md), [기존 레퍼런스 비교](2026-10-09/reference-comparison.md)
- 공정 30초 시간창: 결과607, fresh574, Lost33; Lost의 current visual factor0은29행, factor1은4행
- Lost 때 tracker 중앙값53, historical eligible80, immature146; 각 Lost 행에서 Ceres·marginalization 미실행
- 모든609행의 IMU factor10개 수락·rejection0: packet admission 관측. camera/IMU 물리 동기·calibration·bias 정확성 증명과 구분
- 엔진 p95 **69.7665ms**, frontend p95 **41.377ms**, Ceres p95 **29.964ms**, 수신 age p95 **120.535ms**; stage별 p95 합산 금지
- 현재 영상·독립 GT 부재. 오래된600PGM은 해당30초 영상이 아닌 부분 기록
- 판단: 먼저 **현재 영상까지 연결된 track의 생존·성숙·삼각측량·factor 편입**을 확인. Lost 행에서 실행되지 않은 solver의 반복 수 확대는 직접 대응 근거 부족
- `tracking:acc_bias_threshold`159/576 valid pose: 유효 상태와 경고를 함께 보존. 원인 미확정 상태에서 bias clamp·경고 문턱 상향을 수락 근거로 사용하지 않음

## 공식 상용 문서에서 확인한 내용

| 대상 | 공개 근거 | 이 프로젝트에 적용할 설계 원칙 | 확인되지 않은 내용 |
|---|---|---|---|
| ARKit world tracking | camera feature의 시간 변화와 motion sensing을 결합하는 VIO. 빠른 움직임의 blur·큰 frame간 이동은 품질 저하 | visual support와 관성 prediction의 상태를 구별; 환경·motion 원인을 품질 상태에 기록 | KLT/descriptor 종류, 최적화 상태·bias noise·marginalization·실제 threshold |
| ARKit lifecycle | `notAvailable`, `limited(reason)`, `normal`; 초기화·feature 부족·relocalization 등 구분 | finite pose만으로 fresh 정확도 판정 금지; 회복·재초기화·epoch를 분리 기록 | 내부 state machine·relocalization solver 구현 |
| ARCore motion tracking | 영상 특징과 IMU 결합. 짧은 빠른 움직임에서 IMU 기반 pose 추정 후 visual tracking 재개; 지속 빠른 움직임은 tracking loss 가능 | 관성 전파를 출력하더라도 visual tracking으로 표시하지 않는 명시 상태 필요 | 무영상 propagation 시간 예산·covariance·내부 fallback 구현 |
| ARCore failure reasons | `EXCESSIVE_MOTION`, `INSUFFICIENT_FEATURES`, `INSUFFICIENT_LIGHT`, `BAD_STATE` 등 | feature starvation, motion, sensor/input 문제를 구분한 진단 필요 | 각 reason을 결정하는 수치 기준 |
| ARCore camera | CPU image와 GPU texture용 intrinsics 구분; `CameraIntrinsics` 값은 `Session.update()`마다 변경 가능 | 실제 estimator 픽셀 영역의 K·해상도·crop·회전·안정화 상태를 묶어 기록 | 임의 browser getUserMedia 프레임에 native SDK K를 그대로 적용할 근거 |
| ARCore EIS | gyro 기반 homography mesh와 별도 3D 좌표 변환으로 preview 보정 | 안정화된 preview 좌표와 estimator 관측 좌표가 일치하는지 검증 | 현재 Flip6 browser stream의 EIS/OIS 상태·매 프레임 보정값 |

- ARKit 원문: [Understanding World Tracking](https://developer.apple.com/documentation/arkit/understanding-world-tracking), [Managing Session Life Cycle and Tracking Quality](https://developer.apple.com/documentation/arkit/managing-session-life-cycle-and-tracking-quality). Apple 제공 Markdown을 별도 build에 보존; web fetch의 Markdown content-type 실패는 파일 조회로 보완
- ARCore 원문: [Fundamentals](https://developers.google.com/ar/develop/fundamentals), [rapid motion](https://developers.google.com/ar/develop/runtime), [failure reason API](https://developers.google.com/ar/reference/java/com/google/ar/core/TrackingFailureReason), [Camera](https://developers.google.com/ar/reference/java/com/google/ar/core/Camera), [CameraIntrinsics](https://developers.google.com/ar/reference/java/com/google/ar/core/CameraIntrinsics), [EIS](https://developers.google.com/ar/develop/java/electronic-image-stabilization)
- ARCore EIS 문서의 특정 orientation 지원 설명에는 과거 version 예고 문구가 남아 있음. 현재1.56.0의 지원 범위로 확대하지 않음
- ARKit: 이 조사 환경은 Linux; Mac/iPhone·ARKit 실행 binary 없음. 실제 API 호출·상용 estimator 실행·독립 정확도 측정 없음
- 상용 성능·안정성은 자체 엔진의 near-zero Lost 또는 정확도 수락 기준을 대신하지 않음

## REA 고정 소스·실행 확인

| 항목 | 고정값·실제 결과 |
|---|---|
| 공개 source | [`morluto/rea`](https://github.com/morluto/rea/tree/5190843f8f09ac17cbf79561bd1f708c509cda78), commit `5190843f8f09ac17cbf79561bd1f708c509cda78` |
| source archive | 8,629,622B, SHA256 `07ff498dfe6c5e3ee5ab662b9057ce62414f086438c4005d46758afe46693b71` |
| source license | [MIT](https://github.com/morluto/rea/blob/5190843f8f09ac17cbf79561bd1f708c509cda78/LICENSE); third-party 자료·외부 decompiler 권한 별도 |
| 실행 package | npm `rea-agents@6.4.0`, tar1,825,761B/SHA256 `59eb1604b8db75f522d4a975c36ce6b4a2ad465e43bad165f9a8f728391db9b8`; local prefix·package-lock·npm integrity 보존 |
| 설치 | `--ignore-scripts --omit=dev --no-audit --no-fund`, 144 packages; 전역 설치·agent registration·setup·재시작 없음 |
| 실제 성공·부분 결과 | `inspect-artifact` byte hash·artifact inventory 완료. JS directory 분석은 exit0·parse1파일/26,138AST nodes이나 **semantic worker heap exhaustion으로 PARTIAL** |
| 버전 차이 | 고정 source HEAD의 `inspect-wasm-artifact` 문서와 배포 npm6.4.0 기능 불일치. 실행은 **COMMAND_NOT_FOUND / exit1**, 실패 JSON 보존 |
| 환경 한계 | 현재 PATH에 Hopper/Ghidra/JADX 없음. source의 기능 목록을 실제 decompile 성공으로 보고하지 않음 |

- 기능 계약: [CLI and Evidence](https://github.com/morluto/rea/blob/5190843f8f09ac17cbf79561bd1f708c509cda78/docs/cli.md), [WASM artifact inspection](https://github.com/morluto/rea/blob/5190843f8f09ac17cbf79561bd1f708c509cda78/docs/wasm-artifacts.md)
- 결과: [REA artifact](../../build/research-20261012/commercial/rea-owned-artifact.json), [REA JS graph](../../build/research-20261012/commercial/rea-owned-js-analysis-dir.json), [REA WASM command failure](../../build/research-20261012/commercial/rea-owned-wasm-attempt.json)
- 첫 JS graph 명령에 파일을 직접 넣은 시도는 exit1; 별도1파일 directory 재시도는 exit0. 단, `limitations`의 heap exhaustion 때문에 전체 graph 완료로 판정하지 않음. modules0/findings0은 module·기능 부재의 증거가 아님; 실패·부분 결과 보존
- REA graph는 공급된 JS의 제한된 정적 관측. binary의 수치 알고리즘 복구·상용 solver 재현·runtime behavior의 증명 아님

## 프로젝트 소유 WASM 정적 검사

- snapshot JS: `web/vio_engine.js`를 한 번의 byte read로 복사, 6,025,183B, SHA256 `140e29a8d0717daca8b330d50849671ddb13403e0e82ccb9f8f221a764bf1595`
- 최초 base64 data URI 가정은 실패. 실제 glue는 `binaryDecode(StringLiteral)`; Babel AST로 literal을 찾고 glue의 byte 식만 재현하여 application 실행 없이 추출
- WASM: 5,347,031B, SHA256 `86206cfbd5edc6566ec1075794345c03f17018c3d3a0b339144fc49ae4c29f11`
- WABT: [공식1.0.42 Linux x64 release](https://github.com/WebAssembly/wabt/releases/tag/1.0.42), tar SHA256 `84895407a6bbb80e918f33b16b2fb2206021c150b6bc9ff6f761263a745ab131`; upstream checksum 일치, [Apache2 LICENSE](https://github.com/WebAssembly/wabt/blob/1.0.42/LICENSE)
- `wasm-validate --enable-all`: exit0. `wasm-objdump -h -x`: exit0, import61/export23, nonshared memory initial4096/max8192 pages = **256/512MiB**
- `wasm-objdump -d` stream 계수: SIMD 명령 **65,790개**, atomic 명령0. 전체 disassembly 대신 mnemonic 계수·도구 SHA·exit 기록
- 확인 범위: **이 snapshot에 SIMD opcode가 실제 존재**, 이 module의 shared memory/pthread import 없음
- 미검증: hotpath의 SIMD 실행 비중, phone JIT lowering, 실제 속도 이득, 전체 browser thread 수, thermal, runtime peak memory, 새 build source provenance, 신규 HTTP served identity
- 재현·결과: [추출 helper](../../build/research-20261012/commercial/inspect-owned-wasm.mjs), [snapshot manifest](../../build/research-20261012/commercial/mobile-slam-snapshot.json), [WABT summary](../../build/research-20261012/commercial/owned-wasm-summary.json)

## ARCore 자산·약관 경계

- 공식 Maven metadata: 현재 release **1.56.0**, `lastUpdated=20260831203928`; [Maven AAR](https://dl.google.com/dl/android/maven2/com/google/ar/core/1.56.0/core-1.56.0.aar) 356,212B, SHA256 `20a66fb176c62234879a94202f1424adf14f3cc1159aedbd247d3517b65f1f50`
- 공식 SDK tag `v1.56.0` commit `3abfeb18669c2cbb2d07057f135d117ee9d91826`; [LICENSE Section1](https://github.com/google-ar/arcore-android-sdk/blob/3abfeb18669c2cbb2d07057f135d117ee9d91826/LICENSE)은 AAR·공개 C header를 [ARCore Additional Terms](https://developers.google.com/ar/develop/terms)에 연결
- Additional Terms가 참조하는 [Google APIs Terms §4(a)(6)](https://developers.google.com/terms#section_4_prohibitions_and_confidentiality)에 reverse engineering/source extraction 제한 및 적용 법률 예외 문구 존재
- 적용 약관 링크 확인을 마치기 전 ELF 문자열·심볼·짧은 disassembly를 생성한 조사 순서 오류 기록. 해당 결과는 **제안·알고리즘 해석·제품 코드의 채택 근거에서 제외**, 별도 build에만 보존
- 약관 확인 후 proprietary 내부 분석 중단. 서비스 APK 취득·실행·hook·복호화·접근 우회 없음. 공개 SDK source의 Apache2 표기를 AAR 전체 허용으로 해석하지 않음
- 이 연구의 ARCore 설계 근거: 공개 API 문서·공식 실패 모드·입력 계약. proprietary algorithm recovered라는 주장 없음

## 추가 primary 문헌과 적용 후보

| 문헌 | 근거·가정 | 현 코드와의 차이·후속 후보 |
|---|---|---|
| [Hwangbo et al., IJRR2011](https://www.cs.cmu.edu/~myung/publication/ijrr11.pdf), DOI10.1177/0278364911416391 | 빠른 회전에 gyro 초기 warp, 8DoF affine+photometric patch model. 보정된 camera/gyro 가정 | 현재 `FeatureTracker::readImage`는 gyro로 point center를 seed하고 기존 pyramid에서 translation KLT 수행. **회전으로 변형된 patch** 검증은 별도 후보; CUDA 성능 수치를 WASM으로 전용 금지 |
| [RD-VIO v3](https://arxiv.org/abs/2310.15072v3), 2024-02-16 | IMU-PARSAC의3D–2D/2D–2D 두 단계, pure rotation 감지, deferred triangulation, rotation subframes, translation keyframe 유지 | historical bank는 많아도 current mature support가0/1인 현상과 가까운 설계 비교. bearing/rotation 제약과 metric depth 제약을 구분; paper의 ideal camera/IMU extrinsic 가정에 주의 |
| [Yang et al., full calibration v3](https://arxiv.org/abs/2201.09170v3), 2022-01-29 | IMU/camera intrinsic, extrinsic, time offset, rolling readout의 관측가능성·퇴화 분석. fully excited motion에서의 식별 가능성 | bias 경고를 상수 threshold로 숨기기보다 입력 보정과 residual·excitation 검증 우선. 정지/단축 회전만으로 모든 online calibration 상태를 풀지 않음 |

- RD-VIO/fullcalib PDF 다운로드는 각12MiB 상한을 초과하여 중단·실패 manifest 보존. RD-VIO arXiv HTML 및 fullcalib arXiv/저자 원문을 웹으로 검토; 로컬 PDF 보존 완료로 보고하지 않음
- 문헌은 각 연구 시스템의 근거; ARKit/ARCore가 동일 알고리즘을 쓴다는 증거 없음
- 제품 runtime 의존성·상용/타사 코드 복사 없음. 논문·SDK wrapper·가중치·데이터의 사용권 구분

## 후속 개발 우선순위·검증 조건

1. **current support 관측 경로**
   - 동일 frame timestamp에 LK 전/후·border·F inlier·ID/age·삼각측량 실패 이유·삭제/anchor 이동·current mature factor 편입 수 기록
   - 영상 전체+IMU+effective config+artifact hash 보존; recorder overflow와 누락을 별도 판정
   - historical eligible와 current-supported eligible를 같은 이름으로 합치지 않음
2. **회전 구간의 track·keyframe 보존**
   - 낮은 translation/parallax에서 새 depth의 강제 성숙·가짜 삼각측량 방지; translation-bearing anchor 유지 후보 검증
   - pure rotation의 visual bearing 제약은 metric translation/depth 제약과 별도 모델·상태로 설계
   - frame0/1 visual factor를 과거 factor 또는 propagated pose로 대체해 fresh tracking으로 계산하는 변경 제외
3. **회전 patch 모델 실험**
   - 기존 ordinary/gyro center-seed 경로와 gyro warp patch 후보를 고정 이미지·IMU에서 비교
   - bidirectional cycle, photometric residual, angular geometric consistency, current support를 동시에 평가; 한 지표만으로 adoption하지 않음
   - K/extrinsic/time offset 오차·motion blur·frame gap·rolling shutter·노출/조도 변화를 독립 sweep; 기존 정확도 gate 유지
4. **bias·동기·calibration 검증**
   - `|delta_theta| ≈ |omega| * |delta_t|`; 관측 최대178.03deg/s에서10ms offset은 약1.78deg. 실제 offset 측정값이 아닌 민감도 예시
   - 실제 최종 픽셀 K·camera/body extrinsic·clock provenance·EIS/OIS·readout 미확인 항목을 기록; 풍부한6DoF calibration과 withheld 검증 구간 분리
   - bias norm·update·IMU/visual residual과 loss 전후 변화를 비교. 경고 제거만으로 bias 정확도 판정 금지
5. **수락 조건**
   - 현재 baseline, full room1/room4, held-out fast rotation, 정지/저parallax, 잘못된 hint·동기 오차의 실패 보호 검증
   - independent GT의 one-to-one timestamp association·body frame·SE(3) scale1 translation/rotation ATE·RPE·coverage 유지
   - native와 browser WASM을 각각 검증; 처리율·fresh/initialized coverage·Lost·reset·p50/p95/p99 지연과 accuracy를 함께 보고
   - 이 연구 note는 구현·회귀·실기기 near-zero Lost·physical accuracy의 완료 증거가 아님

## 재현 명령·문서 검증

```bash
node build/research-20261012/commercial/rea-cli/node_modules/rea-agents/scripts/rea.mjs inspect-artifact "$PWD/build/research-20261012/commercial/mobile-slam-snapshot.js" --json
node build/research-20261012/commercial/rea-cli/node_modules/rea-agents/scripts/rea.mjs analyze-javascript-application "$PWD/build/research-20261012/commercial/owned" --json
node build/research-20261012/commercial/inspect-owned-wasm.mjs
build/research-20261012/commercial/wabt/wabt-1.0.42/bin/wasm-validate --enable-all build/research-20261012/commercial/mobile-slam-snapshot.wasm
build/research-20261012/commercial/wabt/wabt-1.0.42/bin/wasm-objdump -h -x build/research-20261012/commercial/mobile-slam-snapshot.wasm
```

- 문서 내부 local link12개 존재 확인; `git diff --check -- docs/research/2026-10-12-commercial-vio.md` exit0
- [종합 manifest](../../build/research-20261012/commercial/research-summary.json)에 성공·부분 결과·미지원·제외 자산 구분; 제품 코드·기존 기록·서비스 변경 없음
