# 상용 SLAM 직접 분석·가상 실행·공개 reference 비교

- 작업: **2026-10-13 시작 → 2026-10-14 완료, Asia/Seoul**. 기준 HEAD `2789f64534f5f8a5d6b4d65fc616a7bdeeb7613f`, 시작 working tree clean.
- 기존 수행에 대한 답: **부분 역분석·일부 가상 runtime 실행은 있었음. 상용 SLAM 내부 solver 전체를 복구하고 동등 입력으로 정확도를 비교한 단계까지 완료한 것은 아님.** 이전 REA inventory·SDK stub·독립 계약 fixture를 상용 엔진 전체 검증으로 계산하지 않음.
- 이번 결과: 저장소 내 역분석 도구 신규 설치, 실제 배포 WASM/ARCore service ELF 함수 분석, 원본 8th Wall SDK·ARCore SDK 직접 실행, VINS-Mono와 현재 엔진의 동일 영상/IMU byte replay 및 GT 비교 완료.
- 제품 estimator·기본 설정·served WASM 변경0. 원본 dataset·기존 서비스·SDK·실패 로그 보존. commit/push/deploy 미수행.
- 통합 증거: `build/commercial-slam-20261013/`. 경로 날짜는 시작일이며 각 manifest의 UTC 획득·실행 시각을 별도 기록.
- 후속 실기기 검증: [Galaxy Z Flip 6 실제 설치·30초 측정·MP4 재생2회](2026-10-14-physical-arcore.md). 아래 표는 최초 가상 환경 조사 범위이며, 이후 physical ARCore601/607 TRACKING 결과는 후속 문서에 별도 기록.

## 설치·코드베이스·직접 실행

| 대상 | 실제 수행·결과 | 판정 한계 |
|---|---|---|
| radare2·WABT | 공식 **radare2 6.2.4·WABT1.0.42** 다운로드 SHA256 확인 후 `scripts/dev/tools/re-lab/`에 신규 격리 설치. `radare2 -v`, `wasm-objdump --version` 직접 성공 | 글로벌 설치·제품 dependency 추가 없음. [설치 manifest](../../scripts/dev/tools/re-lab/installation.json) |
| 8th Wall 배포 code | 기존 pinned official standalone bytes를 새 hash로 확인, 원본 SDK 무패치·public API로 Chrome 실행5건. 400frame responsive599finite pose·`NORMAL`; absolute610finite pose·**모두 `LIMITED/INITIALIZING`** | nonrepeat callback은 각각296/309. callback599를 입력400의 fresh pose coverage로 계산하지 않음. responsive 단위·metric 정확도 미검증 |
| ARCore1.56.0 | 공식 HelloAR source를 별도 출력에서 offline build·설치. Session/update/image/K·record/playback 직접 호출. 최종 live30·record30·playback26/30frame, MP4 **1,238,271B** | 최종4모드 **TRACKING0·pointcloud0**, 정확도=null. camera clock·renderer 이상 보존. SDK 실행 성공과 추적 실패 분리 |
| VINS-Mono | pinned repo `90dabb5e…`·Docker image `0b13b81e…`에서 실제 tracker/estimator 실행. TUM-VI room1 첫600gray·5976IMU. 현재 엔진도 별도 native object 새 빌드·replay | 실제 image/IMU byte SHA parity PASS. ROS/direct cadence·Ceres·검출 mask·weight 차이 존재. 종료 SIGINT 후 upstream thread abort−6은 정상 종료 통과 아님 |
| ARKit/Vuforia | 공식 문서·공개 API/소스 비교, 현 SDK/runtime 존재 여부 확인 | Linux에 Apple runtime 없음; Vuforia 실제 SDK/header/library/key 미공급. 두 엔진 runtime·역분석·ATE 완료 주장 없음 |

- 재사용: 기존 VINS Docker/source·ARCore toolchain·8th Wall `runtime.js`/sensor adapter 사용. 새 runtime·fixture를 별도 제품 abstraction으로 추가하지 않음.
- 실제 source/SDK·실험 설정·binary fingerprint: [8th Wall 보고서](2026-10-13-8thwall-runtime.md), [ARCore 보고서](2026-10-13-arcore-runtime-verification.md), [VINS 보고서](2026-10-13-reference-execution.md).
- 논문·인터넷·공식 source: Context7 조회 후 pinned VINS/OpenVINS/Basalt source·저자 논문·SDK 공식 문서 **28파일/1,901,344B** 새 획득. 추가 8th Wall runtime 문서6개도 새 조회. [source/논문 비교](2026-10-13-engine-gap-analysis.md), [primary manifest](../../build/commercial-slam-20261013/research/primary-manifest.json).

## 실제 역분석 도구 결과

- 새 재현 wrapper: [re-lab](../../scripts/dev/references/re-lab/README.md). GNU readelf/objdump와 신규 radare2·WABT 사용. REA6.4.0은 기존 로컬 설치·inventory 증거를 구분해 유지; Ghidra/JADX 설치·실행을 주장하지 않음.
- 8th Wall `xr-slam.js` SHA256 `398f675c920892afd4521d5196df2b6b8a1b8a90b6a5c749cf5ad1a6e0a85270`, embedded WASM **3,826,095B**, SHA256 `780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422`.
- WABT validation·전 함수 disassembly·직접 callgraph·선택 함수3개 radare `pdf/agfj/pdc`·`wasm2c` 실제 실행 **14명령 exit0**. C lowering **65,370,247B** 및 선택 함수 C excerpt3개 생성; 원본 소스 복구 또는 C 재빌드 성공을 의미하지 않음.
- 정의 함수 **5,455개**, unique direct edge **14,776개**, unresolved indirect callsite **2,654개**. 직접 reachability는 import 포함·간접 호출 미해결 하한이며 runtime 실행 비율이 아님.

| 선택 함수 | WABT 명령 수 | radare 명령/블록 | 직접 도달 함수 수(import 포함) | 그 범위의 unresolved indirect site |
|---|---:|---:|---:|---:|
| `c8EmAsm_stageFrame` | 7,077 | 7,051/433 | 715 | 284 |
| `c8EmAsm_setEventQueue` | 209 | 209/11 | 173 | 149 |
| `c8EmAsm_processStagedFrame` | 988 | 986/37 | 1,409 | 560 |

- 바이너리 ABI 직접 확인: stageFrame의 인자 **4×i32+7×f32+6×f64**, event queue **2×i32**, processStagedFrame **인자0**. 입력 stage/event queue와 staged process 경계 존재. 이 ABI·C lowering만으로 상태 변수·camera/IMU 동기·solver 수식·threshold를 복원했다고 판정하지 않음.
- radare stageFrame은 non-existent branch 경고2개 및 WABT 대비26명령 차이; processStagedFrame은2명령 차이. **pseudo-decompiler 명령 exit0과 완전한 제어 흐름 복구를 구분**. 정확한 명령 inventory는 WABT에 근거하고 private 의미는 미확인 유지.
- [WASM 최종 summary](../../build/commercial-slam-20261013/re-lab/world-verified/summary.json), [명령·stdout/stderr hashes](../../build/commercial-slam-20261013/re-lab/world-verified/commands.json). script snapshot·도구/input/output SHA 포함. raw/C 결과는 ignored build에만 보존, 제품 source에 편입하지 않음.

| ARCore 분석 대상 | 직접 관측 | 의미·한계 |
|---|---|---|
| 기존 SDK client stub | `ArSession_update`13B, forwarding jump | client forwarding 경계만으로 VIO core 분석 완료 아님 |
| 이번 **service** `lib/x86_64/libarcore_c.so` | **47,269,520B**, SHA256 `45691ffe065cf7f9b8f8639a5fec42b052fdf5fb47e99a90abeac12cee8cced4` | emulator APK에서 실제 ELF 추출·snapshot, ELF 자체 Linux 로딩/실행 없음 |
| service `ArSession_update` | 1,908B, radare42blocks/366instructions/40callsites | pointer guard·내부 dispatch·status 경계 검사. 간접 dispatch의 private estimator까지 완전 추적한 것은 아님 |
| service `ArFrame_getTimestamp` | 685B,16blocks/162instructions/27callsites | public timestamp accessor 경계 확인. 센서 노출 시각의 물리적 정확도 증거가 아님 |

- ARCore 함수마다 `pdf/agfj/pdc`와 GNU `objdump --disassemble` 교차 출력, ELF headers 포함 **10명령 exit0**. radare relocation 적용 힌트와 원본 stderr 보존; pseudo-C의 복구 조건문·타입을 원본 의미로 확정하지 않음.
- [service ELF summary](../../build/commercial-slam-20261013/re-lab/arcore-service-final/summary.json), [명령 receipts](../../build/commercial-slam-20261013/re-lab/arcore-service-final/commands.json). 실제 SDK 실행 근거는 위 ARCore runtime 문서로 분리.
- 실패 보존: 첫 bootstrap `--version` 대신 radare `-v` 필요 오류, `world-v1`의 `anal.depth=0`에 의한 빈 출력·`wasm2c --enable-all` 제한 오류. 수정 후 빈 graph/ops·symbol 누락·nonzero exit를 거부하도록 검증. v1을 성공 결과로 덮어쓰지 않음.

## 동일 입력 공개 엔진 비교

- 실제 ROS bag decoded image와 현재 C++ `input-oracle.bin`에서 **600gray bytes/ordered5976IMU bytes 동일**. [입력 동등성](../../build/commercial-slam-20261013/open-reference/input-equivalence.json).
- body frame·one-to-one GT association≤10ms·SE(3) scale1. 공통555timestamp 집합도 별도 평가; zero pose·zero association 결과 제외.

| 지표 | VINS-Mono | 현재 VIOEngine native |
|---|---:|---:|
| 유효 pose/고정600분모 | 570/600,95.0% | 555/600,92.5% |
| 첫 pose 경과 | 1.500152s | 2.250179s |
| 전체 translation ATE RMSE | 0.014971m | 0.014515m |
| 공통555timestamp translation ATE RMSE | 0.015078m | 0.014515m |
| 공통555timestamp rotation APE RMSE | 0.415282° | 0.652218° |
| 공통555timestamp 1s translation RPE RMSE | 0.007593m | 0.008776m |
| 공통555timestamp 1s rotation RPE RMSE | 0.473935° | 0.321725° |

- [metric 결과](../../build/commercial-slam-20261013/open-reference/metrics-room1-600.json). 한30초 development sequence·최종1회 비교이며 엔진 순위·phone 정확도 수락 아님. 속도는 ROS0.5배속 publication과 native direct 호출이 달라 비교 제외.
- YAML 숫자가 같아도 VINS `FOCAL_LENGTH460`과 current190.975892로 keyframe threshold·vision weight가 다름. mask·초기 depth fallback·Ceres1.12/2.2도 다름. pixel/IMU byte parity와 전체 estimator 설정 동등성은 별도.

## 현재 프로젝트와의 차이·다음 실험

| 확인된 차이·공통점 | 현재 source·판단 | 후속 실험 |
|---|---|---|
| 회전 보상 parallax | `feature_manager.cpp::compensatedParallax2`는 현재 `p_i_comp=p_i`. 공개 reference의 bearing/baseline 선택과 비교할 구체적 차이 | 순수회전·작은 병진 fixture에서 raw/회전보상 parallax·mature support·keyframe reason 비교. 바로 threshold 완화하지 않음 |
| 시간 보정 | VINS 공개 td/row/readout factor와 달리 현재 projection에는 해당 상태 없음. clock 정규화·요청 timing identity는 이미 존재 | independent offset/jitter/rolling-shutter sweep, 실제 calibration/exposure 출처 검증 |
| map·recovery | ARKit 공개 relocalization 계약, VINS full pose graph와 current local VIO 범위 차이. 현재 producer `pipeline-v1`; JS limited consumer 존재는 limited recovery 완성 근거 아님 | reset epoch·false recovery·지도 재연결을 local VIO 회복과 분리 평가 |
| 이미 구현된 부분 | young geometry admission(condition≤10000/reprojection≤1.5px), square-root QR 주변화, request/sourceTiming association 존재 | 과거 문서의 미구현 설명을 현재 source에 적용하지 않음. 기존 기능 재도입 대신 실제 실패 membership을 비교 |
| 상용 output 계약 | 8th Wall responsive/absolute 상태·반복 callback, ARCore public state/record-playback·별도 service | `NORMAL`·finite pose·callback FPS를 current metric fresh coverage로 자동 치환하지 않음 |

- 파일·line·pinned source·원문 URL 및 OpenVINS FEJ/MSCKF/Basalt 비교: [engine-gap-analysis](2026-10-13-engine-gap-analysis.md). private 상용 solver의 Ceres/MSCKF/QR 사용 여부는 미확인.
- 이번 변경은 연구 도구·실행 근거·차이 분석. 알고리즘 수정은 baseline 대비 translation/rotation≤5% 증가·coverage 감소≤1percentage point 등 기존 gate와 원인별 독립 검증 후 별도 진행.

## 변경·검증·남은 범위

- 파일: `scripts/dev/references/re-lab/{bootstrap.py,analyze.py,analyze-elf.py,callgraph-control.wat,README.md}`; ARCore runner/analyzer2개; 8th Wall public probe/README; VINS replay/input-verify/evaluate wrapper4개/README; 개발 README와 연구 문서5개.
- 단순화: 기존 runtime·sensor adapter·pinned image 재사용. 새 제품 dependency/estimator abstraction 없음.
- 검증: 도구 checksum/version·WABT validation·WASM14/ELF10명령·script/input/output hash 교차 확인. 독립 reviewer가 direct graph 재계산·parser import/cycle/direct/tail/indirect control·경로 탈출 거부·기존 evidence 불변 확인.
- 실제 toolchain의 [알려진 WAT control](../../build/commercial-slam-20261013/re-lab/control/verification.json): defined2/import1/direct edge2/indirect site1·entry reachability3 확인, 기존 output·canonical dataset write·unsafe export 거부.
- 새 Python/MJS syntax·browser JS285/285·exporter3/3·기존 sensor adapter113checks·strict metric9/9 확인. ARCore summary 재분석도 기존 output 덮어쓰기 거부·원본 summary 보존 확인. 통합 syntax/경계 regression·문서 link/hash 최종 결과: [integration-verification.json](../../build/commercial-slam-20261013/integration-verification.json).
- Plain JS의 ESLint/TypeScript 설정 없음. syntax를 lint/typecheck 성공으로 명명하지 않음. 제품 C++ 수정이 없어 full native/sanitizer/WASM 재빌드·전체 room1/room4 재시험은 이번 연구의 수락 범위 아님.
- 미완료: proprietary solver 전체 수식·상태·threshold 복원/원본 재빌드, ARKit/Vuforia runtime, ARCore emulator 연속 추적, 상용/자체의 exact sensor+calibration+clock 입력 동등성, 실제 phone의 exposure/calibration·열·배터리·독립 GT 정확도. 자료·플랫폼·관측이 없는 항목은 null/미검증 유지.
