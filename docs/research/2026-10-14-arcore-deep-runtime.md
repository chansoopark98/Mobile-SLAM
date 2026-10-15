# 실제 ARM64 ARCore service: 출력·시간·센서·자원 경계 역분석

- 실행일: 2026-10-14, Asia/Seoul. **설치 service ELF의 내부 함수까지 분석**, SDK client stub 분석과 구분.
- 결과: public API25개, timestamp/sensor diagnostic에서 찾은 내부 함수13개, 직접 callee8개, 총 **46개 bounded 함수** 분석. 최종 **200명령 exit0**, GNU/radare bounded instruction inventory **46/46 일치**.
- 실제 확인: frame/state accessor의 저장값 복사, pose/intrinsics별 독립 dispatch, sensor 설정의 mutex/condition wait 경로, 초기 IMU timestamp 검증, image clock 비교, image/camera reference-count release.
- 미확인: `Session_update`의 virtual dispatch 이후 estimator 전체 연결, 세션별 실제 branch 실행, private optimizer/필터 수식, field accuracy·latency 개선. 문자열·파일명만으로 EKF/특정 solver를 판정하지 않음.
- 사용자 진술: 바이너리 분석 법적 검토를 받았고 전체 분석을 허용했다고 명시. 이 보고서는 해당 진술을 권한 맥락으로 기록하며 법률 판단을 제공하지 않음.
- 원본 APK·SDK·ELF·제품 코드 변경0, 신규 Android capture0. [기존 실기기 실행](2026-10-14-physical-arcore.md)과 이번 정적 분석을 혼합하지 않음.
- 최종 근거: [verification.json](../../build/slam-revision-20261014/arcore/verification.json), [public/internal summary](../../build/slam-revision-20261014/arcore/analysis-final/summary.json), [추적 callee](../../build/slam-revision-20261014/arcore/callees/summary.json), [bounded callgraph](../../build/slam-revision-20261014/arcore/bounded-callgraph.json).

## 입력·도구·재현

- 기존 SM-F741N에서 획득한 `com.google.ar.core` **1.56.262080393**의 base APK 재사용. 이번 작업에서는 ADB 수집하지 않음.
- APK:50,954,278B, SHA256 `905d63c2e97cef8df19c1621b517dea9b8775183916f16cb65ab64ba9fe5e3ff`.
- member:`lib/arm64-v8a/libarcore_c.so`, **35,801,624B**, SHA256 `7c32d45c2ef1ebaf92deae86a7ba31b0dd00d39bde09ab0890535ff396d62050`.
- [원래 획득 manifest](../../build/physical-arcore-20261014/device-service/manifest.json). x86_64 emulator service SHA`45691ffe…`·47,269,520B와 다른 ARM64 실제 설치물. 크기·명령 수로 두 아키텍처의 tracking 속도를 비교하지 않음.
- 도구: radare2 6.2.4, GNU `readelf`, `aarch64-linux-gnu-objdump/as/ld`. 실제 version·command·stdout/stderr SHA는 [analysis-final/commands.json](../../build/slam-revision-20261014/arcore/analysis-final/commands.json), [callees/commands.json](../../build/slam-revision-20261014/arcore/callees/commands.json).
- ELF 주소는 **ELF virtual address**, ASLR 이후 실제 device pointer가 아님. 무명 함수에 GNU가 표시하는 `가장 가까운 export+큰 offset`은 원래 함수명이 아님.
- helper는 입력 SHA를 고정하고 기존 output 및 canonical dataset 아래 출력을 거부. 새로운 버전은 주소·layout을 재검토해야 함.

```bash
python3 scripts/dev/references/arcore/binary-boundaries.py \
  --input build/physical-arcore-20261014/device-service/functions/target.so \
  --output build/slam-revision-20261014/arcore/analysis-final

python3 scripts/dev/references/arcore/binary-boundaries.py \
  --input build/physical-arcore-20261014/device-service/functions/target.so \
  --output build/slam-revision-20261014/arcore/callees \
  --follow-only --follow 0x1036aac --follow 0xe8218c --follow 0xe821c0 \
  --follow 0xe84450 --follow 0xf35000 --follow 0x114bbe8 \
  --follow 0x2042790 --follow 0x20422f0

python3 scripts/dev/references/arcore/test_binary_boundaries.py
```

## 접근법과 오류 통제

- ELF64 little-endian section metadata를 decode하고 `.text`만 조사. 인접한 ARM64 `ADRP`→64-bit `ADD immediate`의 **source register가 일치할 때만** 목표 주소 계산.
- 파일 offset→section virtual address 변환 후 선택 diagnostic 문자열의 시작 주소와 exact match. `.eh_frame`의 **63,908 FDE range**로 xref 소유 코드 범위를 찾음.
- 부호 있는 ADRP page displacement, immediate shift, 다른 목적 register를 처리. 비인접 주소 계산·literal load·동적 계산·문자열 중간 주소는 찾지 못하므로 hit0은 기능 부재 근거가 아님.
- 독립 assembler control2건: `.text=0x20000`, `.rodata=0x10000/0x30000`. 음수/양수 page delta, source-register 불일치 decoy, 중간 `nop` decoy를 검사하고 GNU disassembly와 target 주소 대조. **2/2 PASS**. [control 출력](../../build/slam-revision-20261014/arcore/decoder-control.stderr).
- FDE와 export size는 코드 범위 근거이고 C++ 원래 이름/의미의 근거가 아님. 분석 중인 함수 경계 안 raw operations를 `pDj <bytes>`로 읽어 GNU start/stop 범위와 대조.
- 첫 기존 `analyze-elf.py` 실행은 `ArFrame_destroy-pdf`가 **exit0/빈 stdout**을 반환해 중단. 실패 log를 [exports/commands.json](../../build/slam-revision-20261014/arcore/exports/commands.json)에 보존. 기존 wrapper 수정 없이 신규 bounded helper에서 tiny/tail 함수도 검증.
- radare `af/agfj/pdc`는 shared tail·다른 주소까지 따라가 FDE 밖 명령을 포함할 수 있음. 예: 초기화 guard `0x1942ff8`은 bounded **51명령**인데 CFG1101명령 중1050개가 범위 밖. CFG 명령 수를 원함수 크기로 보고하지 않음.
- 46함수 모두 `GNU count ×4 = byte bound`, radare raw count=GNU count. 이는 명령 inventory 검사이며 pseudo-C의 타입·분기·데이터 흐름 완전 복원을 뜻하지 않음.
- 동적 relocation 조사: `.rela.plt` **606개** 포함. `.rela.dyn`은 Android 형식 `LOOS+0x2`로 표시되며 전체 packed relocation/vtable 객체 연결을 복구하지 않음. 임의로 모든 virtual target을 연결하지 않음.

## 출력·pose·intrinsics 계약

| API | 시작 주소 | bytes / 명령 | 코드에서 직접 확인한 핵심 |
|---|---:|---:|---|
| `ArSession_update` | `0xea7c6c` |1016 /254| session/out_frame guard, session vtable slot`0x48` 호출, status 처리·관측 hook |
| `ArFrame_getTimestamp` |`0xe847a0`|572 /143| frame`+24`의64-bit 저장값을 output에 복사 |
| `ArCamera_getTrackingState` |`0xe5a9a0`|1084 /271| camera`+12`의32-bit 저장값 복사 |
| `ArCamera_getTrackingFailureReason` |`0xe5addc`|1084 /271| camera`+16`의32-bit 저장값 복사 |
| `ArCamera_getPose` |`0xe5ca3c`|620 /155| session vtable slot`0x70`을 통한 pose 출력 |
| `ArCamera_getDisplayOrientedPose` |`0xe5c5a8`|620 /155| 별도 slot`0x60`; getPose와 다른 경로 |
| `ArFrame_getAndroidSensorPose` |`0xe7dda4`|1136 /284| session slot`0x130`; camera pose와 다른 경로 |
| `ArCamera_getImageIntrinsics` |`0xe5b218`|556 /139| 내부 camera provider slot`0x90` |
| `ArCamera_getTextureIntrinsics` |`0xe5b444`|556 /139| 같은 provider의 별도 slot`0x98` |
| `ArCameraIntrinsics_getFocalLength` |`0xe5bad0`|728 /182| intrinsics`+0/+4` float32를 output2개로 복사 |
| `ArCameraIntrinsics_getPrincipalPoint` |`0xe5bda8`|728 /182| principal-point accessor를 별도 분석 |
| `ArCameraIntrinsics_getImageDimensions` |`0xe5c080`|1260 /315| dimension accessor·argument guard 분석 |
| `ArImage_getTimestamp` |`0xe8d26c`|1196 /299| image`+32`가 가리키는 객체의`+112`64-bit 값 복사 |

- [timestamp 원문](../../build/slam-revision-20261014/arcore/analysis-final/ArFrame_getTimestamp-objdump.stdout): `0xe84844 ldr x8,[x20,#24]`→`0xe84848 str x8,[x19]`. invalid session/frame branch에서 output pointer가 있으면 `0xe849d0 str xzr,[x19]`로0을 기록하는 경로도 존재.
- tracking state/reason accessor의 값 복사 경로는 새 estimation이나 timestamp freshness 보장과 다름. 저장된 상태를 성공적으로 읽는 것과 새로운 frame을 처리한 것은 분리해서 집계해야 함.
- pose를 읽기 전 session/camera/out_pose null guard는 확인했으나 해당 API 안에서 `tracking==TRACKING`을 검사해 pose 출력을 막는 경로는 식별하지 못함. SDK의 사용 계약은 caller가 상태를 확인하도록 요구.
- 공식 SDK header의 계약과 교차: [arcore_c_api.h](../../assets/references/ARCore/source/libraries/include/arcore_c_api.h), header SHA는 verification에 보존.
  - physical camera pose: image readout 좌표, 중앙 row의 exposure 중심 시점.
  - display-oriented pose: 논리 display orientation 기준; physical camera pose와 회전 차이가 있을 수 있음.
  - Android sensor pose: native device sensor 좌표이며 위치·회전 모두 camera와 다를 수 있음.
  - CPU image/GPU texture intrinsics는 각각 회전·crop 전 값이며 frame마다 바뀔 수 있음.
  - frame timestamp 단위는ns지만 **time base를 확정하지 않으며** CLOCK_BOOTTIME과 비슷할 수 있다는 계약. 아래 내부 clock 관측이 이 API 보장을 바꾸지 않음.
- 이 계약을 현재 프로젝트에 적용할 때에는 camera/body/display/texture pose를 각각 기록하고 extrinsic 방향·rotation/crop K 변환을 명시해야 함. 바이너리의 private object offset을 제품 코드에 복사하는 방식은 제안하지 않음.

## 시간·IMU 내부 코드

| 내부 FDE 시작 | bytes / bounded 명령 | 문자열→code 근거 | 직접 확인·한계 |
|---|---:|---|---|
|`0x17ec5dc`|2148 /537| future image / CLOCK_BOOTTIME| `clock_gettime`을 clock ID7과1로 여러 번 호출, 결과·timestamp 비교 및 오류 분기. 단일 세션에서 실제 offset 보정값은 미측정 |
|`0x1942ff8`|204 /51| 초기 timestamp가 마지막 IMU보다 앞선 경우의 오류| 입력 timestamp≤0 거부, 저장된 마지막 timestamp와 비교 후 더 작은 값 거부, 허용 branch에서 timestamp와 state block 저장 |
|`0x197fd1c`|1292 /323| 마지막 IMU 이후 요청: constant-value extrapolation diagnostic| 끝 record를 가리키는 pointer와 helper 호출 후 새 timestamp를 쓰는 branch 확인. 실제 query 빈도·허용 horizon은 미측정 |
|`0x1980228`|1196 /299| 동일 extrapolation diagnostic| 별도 compiled 함수에서도 해당 branch 참조. 모든 IMU 요청의 무조건 외삽이라고 해석하지 않음 |
|`0x1c1f330`|1376 /344| 동일 extrapolation diagnostic·IMU buffering source path| buffer 시간 범위 비교·record 탐색·경계 처리 코드. 파일명만으로 전체 estimator를 WLS라고 확정하지 않음 |
|`0x180ffe0`|1580 /395| query timestamp가 buffer 최신값보다 새로움| 다른 timestamp 조회 경계의 오류 branch. 위 외삽 경로와 같은 정책이라고 합치지 않음 |
|`0x8fa000`|2356 /589| buffer 최신값보다 오래된 데이터 삽입 거부| 두 timestamp 비교 후 오류 구성 branch. 이 함수의 데이터 종류 전체를 IMU로 특정하지 않음 |

- 모든 exact xref·파일offset·virtual address·FDE range: [strings/summary](../../build/slam-revision-20261014/arcore/analysis-final/summary.json).
- 초기화 시간 guard의 명령 근거:
  - `0x1942ff8 cmp x1,#0; b.le ...`.
  - `0x1943004 ldr x9,[x4,#24]!; cmp x1,x9; b.ge 0x1943050`.
  - 더 작은 timestamp는 `0x162133` diagnostic을 참조하고 공통 status helper로 tail branch.
  - 허용 경로는 `[object+8]`에 timestamp, state block을 저장. 같은 timestamp일 때는 별도 초기화 helper도 호출.
- clock 관측 근거: `0x17ec628 mov w0,#7`→`0x17ec62c bl clock_gettime@plt`; `0x17ec638 mov w0,#1`→`0x17ec63c` 호출. 이것만으로 browser `performance.now()`/Android sensor/camera clock 간 동기 수식을 얻은 것은 아님.
- 외삽 관측 근거: `0x1980034`가 diagnostic 주소`0x148158`를 계산. 이어 `0x1980060`에서 마지막 record 기준 pointer를 구성하여 helper 호출, `0x198006c`에서 timestamp 저장. data-field 단위 의미와 수치 보간 전부를 복원했다고 주장하지 않음.
- 현재 엔진의 우선 검증 후보: frame 경계 IMU coverage, 마지막 IMU age, query>last/역순/동일 timestamp 횟수, 초기화/reset epoch와 time history 재사용 여부. 외삽이나 timestamp 덮어쓰기를 즉시 제품에 도입할 근거로 쓰지 않음.

## 센서 queue·동기화

| 코드 범위 | 직접 관측 | 판단 |
|---|---|---|
|`0x17ed3ac`136B| `0x17ed3c4 bl ASensorManager_createEventQueue@plt`, 반환 null 검사, 성공 시32B wrapper 저장·실패 시 status 생성 | 실제 NDK event queue 생성 경로 존재 |
|`0x16b620c`780B| queue 상태·설정 비교, 설정 변경 loop, mutex/condition wrapper 직접 호출, vtable slot`0x28`의 rate 설정 후보 | 센서 설정은 동기화 경계를 가짐; 실제 event rate 단위/최종Hz는 미측정 |
|`0x16b6928`1500B| queue 생성·sensor 등록·event rate diagnostic, 여러 virtual calls | 관련 manager 경로. 각 virtual target 전체 미해결 |
|`0x16b7134`404B| event queue null guard diagnostic | queue 준비 상태가 호출 선행조건인 branch 존재 |
|`0x17f642c`1420B| sensor 등록 실패 diagnostic | 등록 실패가 별도 오류 경로로 처리됨 |
|`0x17f6b20`452B| queue sensor disable 실패 diagnostic | 종료/비활성화 경계 존재 |

- 직접 callee까지 추적: `0x16b6250→0x2042790→pthread_mutex_lock@plt`, `0x16b6264→0x20422f0→pthread_cond_wait@plt`.
- 따라서 단순 pthread 문자열 존재를 넘어 sensor 설정 함수에서 lock/wait 경로까지 연결. 실제 wait 시간·contention·thread priority·sensor delivery latency는 측정하지 않음.
- import에 pthread create/join/cond/mutex, `ASensorEventQueue_*`, camera/media/EGL/GLES 함수 존재. import 존재만으로 해당 phone run에서 모든 backend가 활성화됐다고 보지 않음.
- 현재 프로젝트 비교 시 camera frame 처리시간과 sensor delivery·queue wait·copy·render를 구분할 이유는 확인. 각 queue 구현·thread 수·priority를 ARCore와 같게 설정하는 성능 제안은 하지 않음.

## 자원·image acquisition·pause/resume

| API | 주소 / bytes | 직접 관측 |
|---|---|---|
|`ArFrame_acquireCameraImage`|`0xe7b3c8` /644| session slot`0x200` 결과를 helper`0xe8218c`에서 status/output으로 변환, 성공·nonnull 조건에서 image`+24`에 session 기록 |
|`ArImage_release`|`0xe8cc90` /148| image/session 관련 필드 검사·diagnostic 후 공통 release`0x1036aac` 호출 |
|`ArCamera_release`|`0xe4e3e8` /4| 공통 release로 단일 **tail branch**. 짧다고 no-op이 아님 |
|`ArFrame_destroy`|`0xe8442c` /36| null이면 return; 내부 destructor`0xe84450` 호출 후 size384 인수와 deallocation 경로로 tail branch |
|`ArSession_resume`|`0xea7974` /380| guard·hook 이후 session slot`0x30`, 공통 status helper`0xf35000` |
|`ArSession_pause`|`0xea7af0` /380| 별도 session slot`0x38`, 같은 status 변환 경계 |
|`ArSession_destroy`|`0xea78a8` /204| 별도 lifecycle 함수. pause와 같은 함수/작업으로 취급하지 않음 |
|`ArImage_getPlaneData`|`0xe8e844` /1564| image/plane/output 검사·별도 accessor 경로. raw field pointer를 frame pose와 같은 수명으로 가정하지 않음 |

- `ArCamera_release`는 여러 release export와 주소 alias를 공유. GNU가 같은 주소를 `ArAnchor_release`로 표시하는 것은 symbol alias 선택이며 분석 대상 변경이 아님.
- **실제 reference counting 확인**:
  - `0x1036acc`에서 counter 주소`object+8`, `0x1036ad0`에서 increment`−1`을 준비하고`0x2068aa0` 호출.
  - helper의 ARM64 LSE 경로는 `ldaddal`; 대체 경로는 `ldaxr→add→stlxr` retry loop. [명령](../../build/slam-revision-20261014/arcore/release-atomic-helper.txt).
  - 반환한 이전 counter가1이면 vtable`+8`의 target으로 tail dispatch;1보다 작은 경우 진단. 따라서 common release는 원자적 감소와 최종 수명 처리 경계.
  - pointer/finalizer 공유·다른 thread와의 실제 ownership topology는 별도 미확인.
- `acquireCameraImage`의 temporary result cleanup`0xe821c0`도 성공 tag에서 common release로 이어짐. API 호출1회와 실제 보유 참조 개수를 같은 수치로 계산하지 않음.
- 현재 프로젝트에 대한 안전한 비교 항목: acquire/release 수·동시 outstanding 수·고수위·에러 후 반환·stop/reset 후 자원 회수. ARCore의 private offset이나 객체 allocator를 복제할 필요는 없음.

## 적용 후보와 수락 한계

| 근거에서 도출한 비교 항목 | 현재 프로젝트에서 필요한 독립 실험 | 금지할 확대 해석 |
|---|---|---|
| timestamp/state는 accessor의 저장값 | 동일 timestamp 반복·stale age·fresh flag·epoch·tracking 상태를 함께 기록 | getter 성공/finite pose를 fresh tracking으로 집계 |
| camera/display/sensor pose 경로 분리 | known extrinsic·display rotation·frame association 검증 | sensor pose와 camera pose 직접 ATE 비교 |
| CPU/GPU K 별도·per-frame 계약 | crop/resize/rotation과 이미지 크기별 K 일치 확인 | texture K를 저장 PGM에 그대로 적용 |
| IMU 미래·역순·초기화 경계 | clock offset/jitter·coverage·endpoint age의 독립 sweep | ARCore diagnostic을 근거로 무제한 외삽 도입 |
| queue 설정 lock/wait·resource reference count | queue wait와 solver latency 분리, stop/reset 수명 회귀 | API wrapper 명령 수를 runtime 처리시간으로 환산 |

- 위 항목은 **설계/실험 후보**, 제품 변경 승인이나 성능 향상 실측이 아님. 기존 current-support·geometry admission·GT 기준을 완화하지 않음.
- 실제 device의 이전 TRACKING 수치는 기존 보고서의 실행 근거이며 이번 static 코드 경로에서 도출한 결과가 아님.
- 최신 ARM64 배포의 제한된 진단·resource·dispatch 경계를 확인했으나 ARCore 전체 소스 복원, optimizer state/noise/marginalization 식, private vtable 전체, 모든 backend의 scheduling을 완료했다고 주장하지 않음.
- 변경 파일: 이 문서, `scripts/dev/references/arcore/binary-boundaries.py`, `test_binary_boundaries.py`. 분석 원문·실패·snapshot은 ignored evidence에 보존. 제품 build/test/deploy 변경 없음.
