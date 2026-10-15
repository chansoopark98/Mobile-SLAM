# 8th Wall 배포 WASM: 모듈·호출 경로·잔차 함수 실험

- 실행일: 2026-10-14, Asia/Seoul. 원본 SDK 보존, 제품 알고리즘·배포 artifact 변경 없음.
- 사용자 제공 권한: 바이너리 엔진 분석에 대한 법적 검토를 받았으며 프로젝트 전체를 분석해도 된다고 명시. 이는 사용자 진술이며 이 보고서의 법률 판단이 아님.
- 결과: 원본 JS 5개에서 WASM 5개 추출, WABT 전체 disassembly·radare2 선택 함수 분석, Ceres 관련 문자열→메모리→함수→호출 경로 연결, RTTI/vtable 후보 복원, **원래 함수 code bytes를 실행한 잔차/Jacobian 6건 통과**.
- 범위 구분: 원본 SDK의 브라우저 실행은 별도 runtime 실험. 여기의 수치 실험은 export만 추가한 **격리 분석 사본**이며 전체 tracking 실행·정확도 근거가 아님.
- 증거 루트: [binary/](../../build/8thwall-deep-20261014/binary/), [전 모듈 결과](../../build/8thwall-deep-20261014/binary/survey/summary.json), [실제 명령·출력 SHA](../../build/8thwall-deep-20261014/binary/survey/commands.json).

## 원본·도구·재현

- 공식 [engine 저장소](https://github.com/8thwall/engine/tree/e1421ae742a9c5d654871367c64c1de3ad044caf) clone HEAD `e1421ae742a9c5d654871367c64c1de3ad044caf`.
- clone의 `HEAD:xr-standalone.zip` LFS pointer: **13,998,848B**, SHA256 `a2fc73ba450c1a96ee2a20db442f7fb328a3c8b76565e7e9d15b68cd75c86a93`.
- 기존 공식 archive SHA와 동일; ZIP 내부 파일 **13/13**이 기존 standalone 파일과 byte 동일. 기존 SDK를 재다운로드하거나 덮어쓰지 않음. [검증](../../build/8thwall-deep-20261014/binary/feature-provenance.json).
- 공개 source 비교: [8thwall/8thwall](https://github.com/8thwall/8thwall/tree/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd) HEAD `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd`. 이 source와 standalone binary의 전체 reproducible build 동일성은 미확인; 인터페이스·특정 수식 대응만 확인.
- 도구: **WABT 1.0.42, radare2 6.2.4, Node.js v22.21.1**, `c++filt`. [설치 SHA](../../scripts/dev/tools/re-lab/installation.json), [이번 version stdout](../../build/8thwall-deep-20261014/binary/survey/radare-version.stdout).
- 라이선스 구분: 공개 monorepo의 MIT와 standalone archive의 `XR ENGINE LICENSE AGREEMENT`는 별도. archive LICENSE와 현재 engine 저장소 LICENSE는 byte 차이가 있으므로 같은 문서로 취급하지 않음. 두 문서의 적용·권리 해석을 이 보고서에서 확정하지 않음.

```bash
# 신규 output만 허용. 원본은 읽기 전용으로 사용.
python3 scripts/dev/references/eighthwall/binary-survey.py \
  --input assets/references/8thwall/analysis-runtime/standalone \
  --output build/8thwall-deep-20261014/binary/survey

# 후속 script도 신규 output만 허용. CLI source와 실행 snapshot/SHA 보존.
python3 scripts/dev/references/eighthwall/binary-semantics.py \
  --survey build/8thwall-deep-20261014/binary/survey \
  --output build/8thwall-deep-20261014/binary/semantic-final
python3 scripts/dev/references/eighthwall/binary-residuals.py \
  --survey build/8thwall-deep-20261014/binary/survey \
  --xrefs build/8thwall-deep-20261014/binary/semantic-final \
  --output build/8thwall-deep-20261014/binary/residuals-final
python3 scripts/dev/references/eighthwall/binary-semantics-control.py \
  --output build/8thwall-deep-20261014/binary/control-final
node scripts/dev/references/eighthwall/binary-residual-probe.mjs \
  --input build/8thwall-deep-20261014/binary/survey/xr-slam/target.wasm \
  --output build/8thwall-deep-20261014/binary/residual-runtime-verified
```

- `binary-survey.py`는 기존 re-lab의 disassembly parser·직접 reachability·tool 환경을 재사용. 분석 script 3개 snapshot/SHA 및 명령별 stdout/stderr SHA 기록.
- 재사용 도구: `binary-survey.py`, `binary-semantics.py`, `binary-residuals.py`, `binary-semantics-control.py`, `binary-residual-probe.mjs`. semantic/residual 도구는 위 원본 WASM SHA에 고정되며 다른 입력을 거부. 새 binary 버전에는 함수 번호·layout을 다시 확인해야 함.
- raw disassembly·분석 사본·C++ type 문자열은 ignored `build/`에만 보존. 새 제품 의존성 없음.

## 5개 WASM의 역할·구조

| 모듈 | WASM bytes | 정의/import 함수 | 함수 export | 직접 edge / 간접 site | SIMD 명령 | 초기 메모리 / table entries |
|---|---:|---:|---:|---:|---:|---:|
| `xr.js` | 415,711 | 1,576 / 226 | 46 | 3,911 / 490 | 1,433 | 96MiB / 548 |
| `xr-slam.js` | 3,826,095 | 5,455 / 259 | 52 | 14,776 / 2,654 | 34,052 | 16MiB / 2,896 |
| `xr-face.js` | 5,554,862 | 6,550 / 232 | 34 | 15,476 / 7,971 | 119,138 | 16MiB / 3,562 |
| `media-worker.js` | 3,661,858 | 3,903 / 46 | 30 | 9,447 / 2,216 | 52,175 | 32MiB / 1,549 |
| `semantics-worker.js` | 3,711,387 | 4,151 / 31 | 31 | 9,122 / 6,859 | 93,127 | 96MiB / 1,799 |

- 함수 export 수는 memory/global/table export를 제외. edge는 unique 직접 caller/callee 쌍이며 간접 site는 미해결 `call_indirect` 위치 수.
- 각 모듈은 자체 nonshared memory 1개, 최대 **32,768pages = 2GiB**, 고정 크기 `funcref` table 1개. 초기/최대 선언은 실제 peak RSS·할당·성능 측정이 아님.
- 전체 5개 모두 atomic 명령 **0**, SIMD 명령 존재. `wasm-validate --disable-simd`는 전부 거부. 기본 feature validation은 5/5 성공; threads를 켜지 않은 validator가 별도 shared-memory atomic control을 거부함도 확인.
- 따라서 이 WASM들에 공유 메모리 threading 요구가 없다는 근거는 확보. JS Web Worker 병렬성·브라우저 내부 GPU 작업·다른 배포 variant의 thread 사용 여부로 확대하지 않음.
- 이름/debug/target-features custom section은 이 5개 section 목록에 없음. export 이름 외 내부 함수는 숫자 ID로 조사. [section 원문](../../build/8thwall-deep-20261014/binary/survey/xr-slam/sections.stdout).
- 잘못 사용한 `--disable-threads` 옵션 5건은 unknown option 실패로 보존. WABT는 threads가 기본 OFF이고 해당 disable flag가 없음. 수정 후 feature/control **7/7** 성공. [실패 포함 기록](../../build/8thwall-deep-20261014/binary/feature-provenance.json), [수정 확인](../../build/8thwall-deep-20261014/binary/feature-validation-corrected.json).

| 모듈 | export/import·compiled material 증거 | 판단 |
|---|---|---|
| `xr.js` | pipeline texture 준비·완료·release·stop reset, semantics render/cubemap, device/camera runtime export; WebGL imports | 카메라/texture pipeline과 렌더 연결 모듈. SLAM 전체가 이 작은 WASM에 들어 있다고 보지 않음 |
| `xr-slam.js` | stage/process, configuration, event queue, query, recenter; `decodeMeshJs`, GL/fetch/WASI imports; Ceres·MapBuilder·MapTracker·BundleAdjustment 문자열 | world/image tracking·query 경계. Ceres 연결을 아래 코드 경로까지 추적 |
| `xr-face.js` | face/mesh/ear model load, stage/process, framework data; TFLite/XNNPACK 및 Ceres type/path | 얼굴 model inference와 geometry 경로의 compiled material 존재 |
| `media-worker.js` | `postMediaError`, WASI I/O; OpenH264 encoder/decoder 문자열 | 미디어 처리 worker. 실제 codec 경로 선택·bitrate·프레임 품질은 미측정 |
| `semantics-worker.js` | model load/start/cleanup/process export; TFLite/XNNPACK path | segmentation/semantics worker 경계. 모델 연산별 runtime 실행·delegate 선택은 미측정 |

- 원문 목록: 각 모듈의 `imports.json`, `exports.json`, `selected-strings.json`. 문자열은 byte offset 포함. TFLite kernel 이름 전체가 실험에서 호출됐다는 뜻이 아님.
- 모듈간 JS-mediated 호출은 WASM 직접 graph에 나타나지 않음. 함수 ID는 각 모듈 내부 번호이므로 서로 같은 ID를 동일 함수로 취급하지 않음.

## 입력·구성·출력·reset 경계

| export | ABI | WABT / radare 명령 | 직접 도달 함수 수 | 미해결 간접 site |
|---|---|---:|---:|---:|
| `xr.getReadyInPipeline` | `(i32,i32)→void` | 116 / 116 | 6 | 0 |
| `xr.markTextureFilledInPipeline` | `()→void` | 23 / 23 | 2 | 0 |
| `xr.markStageFinishedInPipeline` | `()→void` | 23 / 23 | 2 | 0 |
| `xr.releaseInPipeline` | `(i32)→void` | 23 / 23 | 2 | 0 |
| `xr.resetOnPipelineStop` | `()→void` | 63 / 63 | 5 | 0 |
| `slam.configureXr` | `(i32,i32)→void` | 2,398 / 2,392 | 295 | 270 |
| `slam.setEventQueue` | `(i32,i32)→void` | 209 / 209 | 173 | 149 |
| `slam.query` | `(i32,i32)→void` | 4,201 / 4,167 | 372 | 275 |
| `slam.recenter` | `()→void` | 24 / 24 | 458 | 274 |
| `slam.engineCleanup` | `()→void` | 4 / 4 | 172 | 141 |
| `slam.prepareYuvBuffer` | `(i32,i32,i32,i32)→void` | 322 / 322 | 324 | 210 |
| `slam.processStagedFrame` | `()→void` | 988 / 986 | 1,409 | 560 |

- 표의 export는 `c8EmAsm_` 접두사를 생략. 직접 도달은 import 포함, 실행 coverage가 아닌 정적 하한.
- `setEventQueue(ptr,size)`와 `configureXr(ptr,size)`의 두 정수만으로 구조체 의미를 추측하지 않음. 공개 [tracking-controller.cc](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/app/xr/js/tracking-controller.cc#L515)에서 각각 Cap’n Proto `RequestPose`, `XRConfiguration` reader와 연결됨을 확인.
- 공개 source `stageFrame`은 camera texture·크기·회전, Euler/quaternion, video/frame/current time, GPS를 별도 입력으로 받음. 센서 queue를 frame 객체로 복사하고 GPU pyramid `DEFER_READ`를 호출. source의 시간 단위 변환은 영상/frame seconds→nanoseconds; 특정 phone에서 시간 정렬 정확하다는 증거는 아님.
- `processStagedFrame` source는 처리 가능한 pipeline front를 active로 옮기고, 불가하면 이전 active를 반환. JS에 `window._c8.reality`, `realitySize`, `realityTexture`를 전달. 반복 callback과 fresh pose를 분리해야 하는 코드 근거.
- `query`는 `XrQueryRequest`→engine query→`XrQueryResponse` byte buffer 경계. JS hitTest가 Cap’n Proto pointer/size로 호출·decode. 이를 frame pose accessor로 오인하지 않음.
- `recenter` source는 engine rebuild이며 texture pipeline stop reset은 별도 `xr.js` 경계. recenter와 전체 camera session restart를 같은 작업으로 취급하지 않음.
- `prepareYuvBuffer`는 별도 recorder YUV plane 변환 경로. 존재만으로 일반 SLAM 영상이 모두 CPU YUV 입력이라고 단정하지 않음.

## Ceres 문자열에서 실제 코드 경로까지

- 새 분석: WASM Data section 직접 decode → 문자열 파일 offset을 linear-memory address로 변환 → WABT `i32.const` exact xref → export의 직접 graph와 교차.
- 알려진 독립 WAT control: data address **1234**, 직접 root→child, 동일 string을 참조하는 indirect decoy, signed LEB 음수. **6/6** 확인, WABT section도 `init i32=1234`로 교차. [control](../../build/8thwall-deep-20261014/binary/control-final/verification.json).
- `xr-slam`: 관련 문자열401개 중 exact code reference182개; selected export 직접 경로와 만나는 문자열49개. `xr-face`:342/140/9. 모든 참조가 solver 본체는 아니므로 숫자를 복구된 알고리즘 개수로 사용하지 않음.
- 대표 연결: 파일 offset **3,339,776**의 `external/ceres/internal/ceres/solver.cc` → linear address **43,561** → 함수 **1051** → `processStagedFrame` 함수508에서 경로 `508→509→666→702→733→784→936→1051`.
- 함수1051에는 preprocessor/minimizer/problem_impl/gradient_checker 경로도 참조됨. 따라서 SLAM frame export에서 Ceres 관련 compiled routine으로 **정적 직접 도달 가능**함은 문자열 존재보다 강한 증거. branch 실행 여부·solver 옵션·최적화 성공은 이 그래프로 입증하지 않음.
- `xr-face`의 `processStagedFaceControllerFrame`도 함수630의 `ceres/solver.cc` 참조에 직접 도달. Ceres를 world tracking 전용 의존성으로 분류하지 않음.
- [전체 offset/xref 결과](../../build/8thwall-deep-20261014/binary/semantic-final/results.json), [함수1051 disassembly](../../build/8thwall-deep-20261014/binary/semantic-xrefs/xr-slam-func-1051.wasm.txt).

| compiled 설정 key | exact xref 함수 | getter 호출 직전 값 | 확인 한계 |
|---|---:|---:|---|
| `BundleAdjustment.cameraMotionWeight` | 1145 | f32 **10** | getter에 전달되는 상수; effective runtime override 미측정 |
| `BundleAdjustment.numPreviousKeyframesToAdjust` | 1145 | i32 **4** | 동일 |
| `BundleAdjustment.minNumPointsForKeyframeToAdjust` | 1145 | i32 **5** | 동일 |
| `BundleAdjustment.minNumKeyframesForPointToAdjust` | 1145 | i32 **2** | 동일 |
| `BundleAdjustment.numGlobalIterativeIterations` | 1145 | i32 **10** | 실제 반복 횟수 측정값이 아님 |
| `BundleAdjustment.useFastPointAdjustmentInGlobalIterative` | 1145 | i32 **0** | 런타임 Boolean 최종값은 미확인 |

- 함수1145의 첫 initialization guard와 string pointer/상수/call/store 순서를 직접 확인. WABT/radare **47/47명령** 일치. [원문](../../build/8thwall-deep-20261014/binary/semantic-xrefs/xr-slam-func-1145.wasm.txt).
- `PosePnP.solveOptimalPoseUseAnalytical` 등의 getter 함수1563도 frame process에 직접 도달. 이 경로만으로 세션마다 analytical/Schur 어느 branch가 선택되는지 판정하지 않음.

## RTTI·vtable·잔차 수식 연결

- C++ RTTI를 `c++filt -t`로 demangle. 문자열 address→aligned data pointer→type_info 후보→`[0,type_info,table slots...]` 후보→WABT element table의 function 번호 순으로 연결.
- vtable 의미 부여는 관측된 Itanium 계열 layout에 근거한 **후보 해석**. 마지막 slot을 `Evaluate` 후보로 선택한 뒤 실제 산술·수치 호출로 일부 교차 검증. RTTI가 전체 solver state vector 차원을 정의한다고 보지 않음.

| demangled residual type | 잔차 / parameter block 크기 | vtable address point | Evaluate 후보 / 명령 수 | frame process에서 constructor 경로 |
|---|---|---:|---:|---|
| `CameraMovement` | 6 / 6 | 295536 | 1589 / 327 | 함수1022·1030·1146 직접 도달 |
| `SnavelyReprojectionResidual` | 2 / 6,3 | 295684 | 1594 / 2390 | 함수1030 직접 도달 |
| `RotationTarget` | 3 / 6 | 295800 | 1607 / 108 | 함수1146 직접 도달 |
| `ZeroTarget` | 3 / 6 | 295896 | 1604 / 292 | 함수1146 직접 도달 |
| `PoseEstimationTranslationStability` | 3 / 6 | 295988 | 1601 / 1595 | 함수1146 직접 도달 |
| `Scale3dPoseEstimation` | 3 / 1 | 296104 | 1610 / 89 | 함수994 직접 도달 |

- 예: Snavely type 문자열295696 → type_info295780 → vtable295676/address point295684 → slots749/750/751 → functions1592/1593/**1594**. 함수1030이295684를 상수 참조하며 `508→509→666→702→733→784→938→992→1030`으로 도달.
- Evaluate 후보들은 주로 indirect dispatch 대상이므로 기존 직접 graph에서 누락. 이번 연결은 data/table 기반으로 간접 후보를 좁힌 것이며 모든 간접 호출 해결이 아님.
- 공개 [bundle-residual.h](https://github.com/8thwall/8thwall/blob/4edfa484936a2cc691366eeec5c6e58bb6d4c8bd/reality/engine/geometry/bundle-residual.h#L253)의 대응 수식:
  - CameraMovement: 회전3축 `rw·(camera-reference)`, 병진3축 `pw·(camera-reference)`.
  - RotationTarget: 처음3 parameter와 target의 차이. 후보1607의 scalar `f64.sub`3회와 SIMD/Jacobian 분기를 확인.
  - ZeroTarget: offset부터3 parameter를 잔차로 반환.
  - Snavely: angle-axis 회전·translation 이후 `scale·x/z-u`, `scale·y/z-v`. 공개 source 수식과 type 차원·binary 후보 대응; 전체 branch의 수치 동등성은 이번에 미실험.
  - TranslationStability: world camera position의 두 시점 차분 대비 변화에0.1을 곱하는 공개 수식. binary 후보의 모든 branch 복원·수치 확인은 미완료. pose output smoothing과 동일 기능이라고 단정하지 않음.
- [vtable·경로 결과](../../build/8thwall-deep-20261014/binary/residuals-final/residuals-symbol.json).

## 원래 WASM 잔차 함수의 수치 실행

- 독립 메모리에 object/functor/parameter/residual/Jacobian 영역을 배치. 포인터 layout은 해당 WASM load offset (`object+20`, double8-byte stride)·vtable 후보·공개 residual data layout을 확인한 뒤 구성.
- WASM import259개는 호출 즉시 실패하는 trap으로 연결. 세 잔차 함수6회 실행 중 **import 호출0**. camera/device/network·SDK 초기화·브라우저 lifecycle을 실행하지 않은 성분 실험.
- 원본 `xr-slam` WASM SHA256: `780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422`.
- 분석 사본 SHA256: `6894cb2fa1ae0f8443d572c2701ef664f246e58061a3501f1e0ba12a25b48502`. 변경은 Export section에 후보 함수3개를 추가한 것뿐; **원본 Code section SHA256 `6ff897c9bfb8709f4b792d1595eae175e4bc5947f09d16c011273e94be00a809` 동일**.
- 입력 camera `[1,2,3,4,5,6]`, reference `[0.1,0.2,0.3,0.4,0.5,0.6]`. Jacobian 없음/있음 각각 실행. 성공 반환1·잔차 오차<`1e-12`·Jacobian 정확 일치 **6/6 PASS**.

| 함수 | 추가 입력 | 실제 잔차 | Jacobian |
|---|---|---|---|
| CameraMovement1589 | `pw=0.2`, `rw=2` | `[1.8,3.6,5.4,0.72,0.9,1.08]` (부동소수점 허용오차 내) | 6×6 diagonal `[2,2,2,0.2,0.2,0.2]` |
| RotationTarget1607 | reference 처음3값 | `[0.9,1.8,2.7]` | 3×6 `[I₃ 0]` |
| ZeroTarget1604 | offset3 | `[4,5,6]` | 3×6 `[0 I₃]` |

- [재사용 실험 script](../../scripts/dev/references/eighthwall/binary-residual-probe.mjs), [전체 입력·예상·실제·hash](../../build/8thwall-deep-20261014/binary/residual-runtime-verified/result.json).
- 이 실험으로 해당 후보의 산술·Jacobian 동작과 layout 해석은 확인. 실제 프레임에서 그 잔차가 호출되는 빈도, weight 최종값, 최적화 convergence, track loss·복구·ATE/RPE는 미검증.

## 도구 신뢰도·남은 범위

- 1차 survey **54명령 exit0**. 후속 symbol-address 기반 분석18명령 exit0. WABT와 radare는 일부 함수에서 명령 수 불일치:
  - configure2398/2392, query4201/4167, face process17013/17022, semantics model load2261/2246.
  - 내부 Ceres 후보1051은7744/7484, translation-stability 후보1601은1595/566.
  - 수치 실행한1589/1607/1604는327/327,108/108,292/292 일치. 그러나 개수 일치만으로 제어 흐름·decompiler 정확도를 증명하지 않음.
- 첫 내부 함수 radare 시도는 WASM function body 시작의 local declarations를 opcode로 분석해1–4명령만 나옴. 실패 output·script·`residuals.json` 보존 후 `symbols.stdout`의 실제 `vaddr` 사용으로 수정. 최종은 **`residuals-symbol.json`**.
- 문자열/table/code xref와 실제 잔차 산술까지 확인했으나 최적화 state 전체·marginalization·IMU noise model·map/relocalization 전체 수식·모든 threshold를 복원한 것은 아님.
- binary compiled configuration은 공개 source와 범위가 다를 수 있음. source에 없는 문자열을 곧바로 미공개 알고리즘의 완전한 설명으로 대체하지 않음.
- 문서·신규 분석 도구5개만 tracked 후보. 제품 C++/JS 변경 없음; native/sanitizer/full WASM 제품 rebuild는 이번 정적·성분 분석의 검증 범위가 아님.
