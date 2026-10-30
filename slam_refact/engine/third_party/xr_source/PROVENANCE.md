# third_party/xr_source 출처 기록

## 2026-10-30 공개 트리 정리

- 원 출처·저작자 고지: [루트 README](../../../../README.md#external-code). 아래 이전 기록의 경로는 공개 경로 이름으로 갱신했습니다.
- `upstream_sha256`: 기존 source proof-base의 원래 bytes 해시 유지. `vendored_sha256`: 현재 프로젝트 파일 bytes 해시. 이름·주석 정리로 달라진 두 값을 같은 값으로 덮어쓰지 않습니다.
- 검증: `python3 slam_refact/engine/tools/check_vendor.py --upstream --data`. 원본 해시 확인 → 기존 patch 적용 → `public_source_bytes`의 정해진 이름·저자 연락처 주석 정리 → local bytes 일치를 검사합니다. 알고리즘 임의 변경은 허용하지 않습니다.
- 현재 MANIFEST 212행: 기존 206행 + 재생성 입력 6개(`source-input`, CMake 컴파일 목록에는 추가하지 않음).
- 추가 입력: `c8/camera/device-infos.{h,cc}`, `c8/geometry/intrinsics.cc`, `reality/engine/geometry/analytical-residual/{generate-pose-estimation,rotation,memoizer}.py`.
- device table 재생성: `python3 -I -B slam_refact/engine/src/frontend/gen_device_table_inc.py`는 프로젝트 안의 `xr_source/`를 기본 입력으로 씁니다. 출력은 기존 113 모델·99 초점거리 case·152 기기 항목과 byte 동일합니다.
- 원본 비교는 선택한 proof-base에서 analytic residual을 다시 생성해 비교합니다. 공개 정리는 생성된 수식을 바꾸지 않습니다.
- 아래의 “머리 주석·본문 불변” 및 파일 개수는 각 기록 작성일의 값입니다. 현재 공개 트리에는 이 절의 이름·주석 변경과 추가 입력이 적용되어 있습니다.


- 작성: WP0a, 2026-10-15. 근거 설계: `slam_refact/docs/design/architecture.md` §2.2, §5.3.
- 출처: `../../../../README.md#external-code` commit `4edfa484936a2cc691366eeec5c6e58bb6d4c8bd`.
  - 로컬 사본: `assets/references/xr_reference/official/xr_reference` (gitignore 대상, HEAD = 위 commit). **이 worktree는 편집된 상태입니다**(`git status`가 907개 파일을 보이고, 추적 파일 903개 중 870개는 정확히 한 줄 삭제입니다). R4(사용자 결정 2026-10-20: clone 작업 트리가 기준, 저작권 고지 파일은 만들지 않음)에 따라 `check_vendor.py --upstream`의 기본 비교 기준은 이 **현재 작업 트리**입니다(`--upstream-base worktree`, 기본값). 이전 방식(commit의 git 객체 + 저작권 줄 1개 제거 변환)은 `--upstream-base commit`으로 남겨 두었습니다(아래 "비교 기준"). 작업 트리 기준 분석: MANIFEST 196행(자체 patch 8행 제외) 전부 일치, 불일치 0.
- 파일 목록, upstream 경로, upstream sha256, vendored sha256, 적용 patch, role은 `MANIFEST.tsv`가 정본입니다. `upstream_sha256`은 "commit의 파일에서 저작권 한 줄을 뺀 것"의 해시입니다.
- 이 문서와 MANIFEST에는 출처만 적습니다(commit `5b8f6f5` 정책). **vendored 파일은 upstream commit의 파일과 byte 동일하지 않습니다**: reference runtime/Niantic 저작권 한 줄(`// Copyright (c) 20xx …`, Python 생성기는 `#`)을 뺐습니다. closure의 upstream 파일 192개 중 172개가 해당하고(1행 170개, 2행 2개), 나머지 20개는 commit 파일 그대로입니다(it-lane closure 리뷰 N-8, 2026-10-20: 작성 시점 값. 현재 `check_vendor.py --upstream` 출력은 vendored upstream 파일 195개 = commit 파일에서 저작권 줄을 뺀 것 175개 + commit 파일 그대로 20개). 그 한 줄 외에 머리 주석과 본문은 바꾸지 않았고, 본문 변경은 `patches/` 8개뿐입니다(아래 표, 모두 `check_vendor.py`가 재현). 그중 7개는 컴파일 호환·closure 보완이고, 8번째 `us015-image-world-tracker.patch`(2026-10-20)는 닫힌 바이너리 동작의 복원입니다(hook이 nullptr인 기본 경로는 공개 코드와 같음; 아래 표). `third_party/murmurhash/constexpr_murmur3.h`의 MIT 저작권 줄(Tamás Szelei)은 reference runtime 줄이 아니라서 남아 있습니다.

## 구성 (206개, `MANIFEST.tsv` 데이터 행; 2026-10-22 lighting 2개 추가 후)

| role | 수 | 내용 |
|---|---:|---|
| `tu` | 79 | `build_probe.sh`의 `C8_SRCS`(22) + `FEAT_SRCS`(18) + `GEO_SRCS`(12) 중 아래 두 항목을 뺀 50개 + image target core 11개(아래 "image target core") + face/ear 15개(아래 "face and ear") + sky 2개(아래 "sky") + lighting 1개(아래 "lighting") |
| `tu-wasm-only` | 1 | `c8/time/now.cc` (아래 "native 제외" 참조) |
| `generated` | 1 | `reality/engine/geometry/analytical-residual/pose-estimation.cc` |
| `header` | 117 | 52 TU + vendor probe의 include closure(85개). `third_party/cvlite/flann/*.h` 16개, `third_party/murmurhash/constexpr_murmur3.h`, `bzl/inliner/rules2.h`, `bzl/stamp/release-build-id.h` 포함. + image target core 헤더 11개 + face/ear 헤더 18개 + sky 헤더 2개 + lighting 헤더 1개 |
| `patch` | 8 | `patches/*.patch` |

최초(2026-10-15) 구성은 165개(tu 61, header 96, patch 6 등)였고(N-8 정정: 첫 commit `6827916`의 MANIFEST는 142개 = tu 50, header 85, patch 5, generated 1, tu-wasm-only 1이며 165개는 작성 시점의 오기), 이후 image target core(US-011)와 face/ear(US-018/019)가 늘렸습니다. (2026-10-20 정정, US-015 리뷰 F-2) header 수는 이전에 114로 적혀 있었으나 이 행의 내역 합(85 + 11 + 18 + 2)과 MANIFEST 모두 116이며, 그 2개 차이는 표기 오류로 확인했습니다(HEAD의 MANIFEST header 85개 대비 현재 추가된 31개 = image target 11 + face/ear 18 + sky 2; 다른 lane의 미기록 헤더는 없음). 이 정정 전 데이터 행 203개(78 + 1 + 1 + 116 + 7)에 US-015 patch 행 1개가 더해져 204개입니다. 아래 "image target core"의 22개 = TU 11 + 헤더 11입니다.

### 비교 기준 (RM-2, `check_vendor.py --upstream`)

- (R4, 2026-10-20) 기본 기준은 reference 사본의 **현재 작업 트리 파일 그대로**입니다(변환·git 없음; `upstream_sha256` = clone 파일의 sha256, 저작권 줄이 남은 파일은 FAIL). 아래는 `--upstream-base commit`(이전 기본값)의 설명입니다. 그 모드는 reference 사본의 **commit 4edfa484 git 객체**를 기준으로 하며(`git show <commit>:<path>`), commit이 사본에 없으면 worktree로 대체하지 않고 BLOCKED로 실패합니다.
- 각 파일의 기준 bytes = commit 파일에서 reference runtime/Niantic 저작권 줄(처음 3줄 안의 첫 `// Copyright (c) 20xx … reference runtime|Niantic`, Python은 `#`) 하나를 뺀 것. 이 변환 하나만 허용하고, 출력에 몇 개에 적용했는지 센 값을 보입니다(171 + 생성기 1 = 172개 적용, 20개 그대로; it-lane closure 리뷰 N-8, 2026-10-20: 작성 시점 값. 현재 `check_vendor.py --upstream` 출력은 vendored upstream 파일 195개 = commit 파일에서 저작권 줄을 뺀 것 175개 + commit 파일 그대로 20개).
- 확인 항목: (1) 기준 bytes의 sha256 = MANIFEST `upstream_sha256`; (2) 기준 + `patches/`(`PATCH_ORDER`) = vendored, byte 단위; (3) "include 줄만" 바꾼다고 적은 patch(`gcc-isnan-global.patch`, `face-ear-closure.patch`)의 hunk가 `#include` 줄(과 빈 줄)뿐인지; (4) `generated`는 commit의 파일들(`git archive`)로 생성기를 돌려 재생성.
- 이전 방식은 worktree 파일의 해시를 `upstream_sha256`과 비교했습니다. 사본이 편집돼 있으면 "commit의 파일"이 아니라 "편집된 파일"과 비교하게 되고, 깨끗한 clone에서는 전부 실패하며, 사본과 vendored를 함께 고친 변경은 잡지 못했습니다. 지금 191개(작성 시점 값; 현재 195개) vendored 파일의 worktree 사본은 새 기준과 모두 같아서(편집이 저작권 줄 삭제뿐이므로) 이번에 해시 값은 하나도 바뀌지 않았습니다.

### closure 계산 방법 (VERIFIED)

`include-closure-per-file.json`(정규식 기반, seed 8개 누락) 대신 컴파일러 의존성 출력으로 다시 계산했습니다.

```
g++ -std=c++17 -DJAVASCRIPT -DC8_NO_EXCEPT -MM <52 TU + probe_main.cc + parameter-data-shim.cc>
g++ -std=c++17 -DC8_NO_EXCEPT -MM <같은 입력>
em++ -std=c++17 -DJAVASCRIPT -DC8_NO_EXCEPT -MM <같은 입력>
include 순서: lane-B shim → overlay-cxx17 → gen → upstream root
```

세 결과의 합집합 중 upstream/overlay/gen 아래 파일이 137개(TU 52 + header 85)입니다. 증거: `build/slam-refact-native-wp0a/closure/deps-{gxx,gxx-nojs,em}.txt`.

## generated 파일

- `pose-estimation.cc`는 Bazel genrule `:pose-estimation-analytical-cc`의 결과입니다.
- 생성기: upstream `reality/engine/geometry/analytical-residual/generate-pose-estimation.py` (sha256 `20482eb1…f793`).
- 생성 환경: python 3.13.5 + sympy 1.14.0 (reference runtime pin은 sympy 1.11.1). 결과 sha256 `03be91a798d3c26edc5719c01c9921faea1e0cbff5721ecbb13d5e70ff5b26cb` = lane B 재생성본 (architecture §2.2 1).
- `check_vendor.py --upstream`이 생성기를 다시 돌려 byte 동일을 확인합니다(VERIFIED 2026-10-15).
- 기본값 `Bundle.useNewPoseEstimationAnalytic=0`에서는 호출되지 않습니다.

## patch (적용 순서 = `check_vendor.py`의 `PATCH_ORDER`)

| patch | sha256 | 대상 | 분류 (P-2) |
|---|---|---|---|
| `gcc-frame-point-comparator.patch` | `3c97c306…532e` | `frame-point.h` `comparator_{}` | ② GCC 호환 |
| `cxx17-constexpr-uninit.patch` | `253a4912…3233` | `hmatrix.h` `inv()` constexpr 제거, `gr8gl.h` `fastAtan2` 초기화 | ① C++17 port-down |
| `cxx17-set-contains.patch` | `97414cfb…bba0` | `pose-pnp.cc` `contains` → `count` | ① |
| `cxx17-erase-if.patch` | `d396d7c2…3425` | `global-matcher.cc` `std::erase_if` → erase-remove | ① |
| `libcxx-compat.patch` | `fa9023c6…748e` | 아래 표 | ③ libc++ 동등화 |
| `gcc-isnan-global.patch` | `632b1f82…51b6` | `image-targets-rigid-assembly.cc`: `#include <math.h>` 한 줄 추가(알고리즘 줄 변경 없음). 한정자 없는 `isnan`이 libstdc++의 `<cmath>`만으로는 전역에 없어 g++에서 컴파일되지 않음 | ② GCC 호환 (US-011) |
| `face-ear-closure.patch` | `0225848e…15c5` | include 두 줄만 추가(알고리즘 줄 변경 없음): `tracked-face-state.h`에 `c8/map.h`(원본은 capnp 의존 `scope-timer.h`를 통해 간접 include, shim에는 없음), `ears/ear-types.cc`에 shim `c8/geometry/intrinsics-clip.h`(class `Intrinsics` 부분집합) | ② closure 보완 (US-018/019) |
| `us015-image-world-tracker.patch` | `939ca095…cfd2` | `imagedetection/image-tracking-state.{h,cc}`, `detection-image-tracker.{h,cc}`: 바이너리 ImageTrackingState의 world tracker 멤버(`ImageWorldTrackerState`)와 hook 인터페이스(`ImageWorldTracker`), `updateTrackingState`의 VIO-on 세 분기(reset·994 결과·분실 frame pose), `DetectionImageTracker::setWorldTracker/worldTracker`(+76 / −8). hook이 nullptr(기본)이면 공개 코드와 같은 경로. 근거 `docs/spec/11-image-world-tracker.md` §9.3, 계약 FULL-12 | ④ 닫힌 바이너리 동작 복원 (US-015, 기본 off) |

앞의 네 patch는 lane B(`slam_refact/tools/re/public_build/patches/`)의 파일을 그대로 복사했습니다(RE-B §8).

### `libcxx-compat.patch` (WP0a 신규)

호출만 `slam_refact::libcxx::`(`third_party/libcxx_compat`)로 바꾸고 include 한 줄을 더합니다. 인자와 알고리즘 줄은 그대로입니다.

| 파일 | 호출 | 설계 목록(§5.3) |
|---|---|---|
| `reality/engine/features/gr8gl.cc:204` | `nth_element` (retainBest, Q3) | 있음 |
| `reality/engine/geometry/pose-pnp.cc:308` | `nth_element` | 있음 |
| `c8/random-numbers.h:38,47-50` | `shuffle`, `normal/uniform_real/uniform_int(u32,u64)` 분포 | 있음 |
| `reality/engine/features/descriptor-lsh-index.cc:92` | `shuffle` | 있음 |
| `third_party/cvlite/flann/lsh-table.h:365,459` | `shuffle`, `sort` | 있음 |
| `reality/engine/features/frame-point.h:64,66,69,77` | `push_heap`, `pop_heap`, `sort_heap` (`BestPointMatches`) | **추가** |
| `c8/geometry/two-d.cc:199` | `priority_queue` (`Poly2::convexHull`) | **추가** |

추가 두 건의 근거: `check_std_algos.py`의 전수 검사에서 나왔습니다. `BestPointMatches`의 비교자는 descriptor 거리(정수 Hamming)만 보므로 동점이 흔하고, heap 구현에 따라 남는 match와 순서가 달라집니다. `convexHull`의 `priority_queue`도 같은 각도의 점 순서가 heap 구현에 따라 달라집니다. 두 경우 모두 P-2 ③(libc++ 동등화)에 해당합니다.

## native 제외: `c8/time/now.cc` (review NI-1)

- 모든 `sr_vendor_*` 타깃은 native와 WASM 모두 `-DJAVASCRIPT`입니다. 이것이 검출을 단일 스레드로 만듭니다(`gr8gl.cc:93-97` `USE_MULTITHREADED_FEATURES=false`, `thread-pool.cc:27` `threadCount=0`).
- `c8/time/now.cc`는 `JAVASCRIPT`에서 `emscripten/emscripten.h`와 `emscripten_get_now()`를 써서 native에서 컴파일되지 않습니다.
- **excluded on native: unreferenced in closure; emscripten-only under JAVASCRIPT.** closure 안에서 `c8/time/now.h`를 include하거나 `nowMicros()`를 부르는 파일이 없습니다(grep, 2026-10-15). WASM 빌드에는 그대로 들어갑니다.
- 구현: `third_party/CMakeLists.txt`의 `if(NOT EMSCRIPTEN) list(REMOVE_ITEM …)`. MANIFEST role `tu-wasm-only`.

## image target core (full-parity PRD US-011, 2026-10-19)

- 대상: 공개 `reality/engine/imagedetection/`의 추적 본체 10쌍(`tracked-image`, `target-point`, `detection-image-matcher`, `detection-image-local-matcher`, `detection-image`, `image-tracking-state`, `detection-image-tracker`, `location`, `local-points-filter`, `image-targets-rigid-assembly`)과 `reality/engine/geometry/image-warp.{h,cc}`. 같은 upstream commit의 파일에서 저작권 한 줄만 뺀 것을 복사했고(위 정정), `image-targets-rigid-assembly.cc`에 `gcc-isnan-global.patch`(`#include <math.h>` 한 줄)를, `image-tracking-state.{h,cc}`와 `detection-image-tracker.{h,cc}`에 `us015-image-world-tracker.patch`(2026-10-20, US-015 닫힌 world tracker 복원, 기본 off)를 적용합니다.
- 제외(대체): `detection-image-loader.{h,cc}`(GL pyramid, `GlTexture`, capnp metadata)는 `src/image_target/target_loader.{h,cc}`(CPU pyramid 포트 위 재작성)로, `image-detection-request-executor.{h,cc}`(capnp 응답, ARCore/ARKit 분기)는 US-013에서 plain struct로 대체합니다. 공개 테스트 파일 5개(`*-test.cc`)는 모두 `tests/image_target/`에 이식했습니다(`detection-image-loader-test`, `detection-image-tracker-test`는 GL과 git-lfs 이미지 대신 CPU loader와 합성 이미지).
- closure (VERIFIED, 2026-10-19): `g++ -std=c++17 -DJAVASCRIPT -DC8_NO_EXCEPT -MM -Ixr_shim -Ixr_reference -Ilibcxx_compat/include <11 TU>`의 결과에서 기존 MANIFEST·shim에 없는 파일은 이 22개와 shim `reality/engine/api/reality.capnp.h`뿐입니다.
- 빌드: `src/image_target/CMakeLists.txt`의 `sr_vendor_imagedetection`(STATIC, `-w`, `_USE_MATH_DEFINES`는 공개 BUILD copts). 기존 `sr_vendor_*` 타깃과 world SLAM 경로는 바뀌지 않습니다.
- libc++ 민감 구문: `detection-image-tracker.cc`의 `std::sort`(비교자가 고유 이름까지 보는 전순서)와 테스트 전용 `IdGenerator`의 `uniform_int_distribution`(값을 뽑지 않음)은 `tools/std_algos_allowlist.tsv`에 근거와 함께 기록했습니다.

## face and ear (full-parity PRD US-018, US-019, 2026-10-19)

- 대상: 공개 `reality/engine/faces/`의 `face-geometry`, `tracked-face-state`, `face-detector-global`, `face-detector-local`, `face-tracker`; `reality/engine/ears/`의 `ear-types`, `ear-detector-local`; `reality/engine/deepnets/detection-anchor-nms`; `c8/geometry/`의 `face-types`, `facemesh-data.h`, `face-augmentation`, `mesh`, `mesh-types`, `box2`; 의존 헤더 `c8/color.h`, `c8/string-view.h`, `c8/string/join.h`, `c8/string/containers.h`; `pixel-transforms.cc`의 미해결 정의를 채우는 `c8/color-maps.cc`, `c8/pixels/packed.cc`. 같은 upstream commit의 파일에서 저작권 한 줄만 뺀 것을 복사했고(위 정정), `face-ear-closure.patch`(include 2줄)만 적용합니다.
- 제외(대체): GL ROI renderer(`faces/face-roi-renderer`, `face-roi-shader*`, `ears/ear-roi-renderer`)는 `src/face/face_roi_renderer.{h,cc}`(CPU rasterizer), capnp 메시지 채우기(`faces/face-messages`)와 JS glue(`reality/app/xr/js/facecontroller.cc`)는 `src/face/face_controller.{h,cc}`, `face_coordinates.{h,cc}`(orientation.cc의 FaceResponse 부분), 이벤트(`face-controller.ts`)는 `src/face/face_events.{h,cc}`로 대체합니다. TFLite/XNNPACK interpreter(`deepnets/tflite-interpreter`)는 shim이 같은 호출면을 `src/ml`로 실행합니다.
- 빌드: `src/face/CMakeLists.txt`의 `sr_vendor_face`(STATIC, `-w`)와 `sr_vendor_face_pixels`(OBJECT). 기존 `sr_vendor_*` 타깃과 world SLAM 경로는 바뀌지 않습니다.
- libc++ 민감 구문: `HashMap`(얼굴 id로 find/operator[]/erase만, 순회 없음)과 `tracked-face-state.cc` IPD의 `std::sort`(float 값 정렬)는 `tools/std_algos_allowlist.tsv`에 근거와 함께 기록했습니다. `ear-types.cc` `median`의 한정자 없는 `nth_element`(검사기 패턴 밖)는 n번째 순서 통계량 값만 쓰므로 결과가 구현과 무관합니다(value-only).
- 테스트: 모델·이미지가 필요 없는 공개 테스트와 정답 JSON 2개는 `tests/face/`에 이식했습니다(`docs/spec/08-face-ear.md` §5, §8). git-lfs 이미지·모델에 의존하는 `TestMesh`, `TestFaces`, `face-detector-*-test`, `ear-detector-local-test`의 가시성 assertion은 BLOCKED입니다.

## sky (full-parity PRD US-020, 2026-10-20)

- 대상: 공개 `reality/engine/semantics/semantics-classifier.{h,cc}`와 `reality/engine/deepnets/tflite-multiclassifier.{h,cc}`(RGBA/255 -> float32 NHWC 변환, 출력 [1,256,144,20]의 class 0 plane). 같은 upstream commit의 파일에서 저작권 한 줄만 뺀 것을 복사했고(위 정정), **patch는 없습니다**. 두 `.cc`는 `bzl/inliner/rules2.h`의 빈 `cc_library{}`/`cc_end()` 매크로를 그대로 컴파일합니다(이미 vendored).
- 제외(대체): GL cubemap renderer `semantics-cubemap-renderer.{h,cc}`와 셰이더 7개(`semantics/shaders/`), `semantics-*-shader`, `texture-projection-shader`, capnp(`api/semantics.capnp`), JS glue(`reality/app/xr/js/layers-controller.cc`, `src/layers-controller.ts`, `src/semantics/*`, `semantics-worker.{cc,js}`)는 vendoring하지 않고 `src/sky/`가 CPU 코드로 다시 구현합니다(`docs/spec/09-sky.md` §1).
- `tflite-multiclassifier.h`는 `#include "tflite-interpreter.h"`를 디렉터리 없이 씁니다. 이 파일을 고치지 않기 위해 `src/sky/CMakeLists.txt`의 `sr_vendor_sky`가 shim의 `reality/engine/deepnets/` 디렉터리를 include 경로에 둡니다. `tflite-multiclassifier.cc`가 포함하는 `tflite-model-operations.h`(원본은 `third_party/half`에 의존)는 shim의 빈 헤더가 대체합니다(아래 shim 표).
- 빌드: `src/sky/CMakeLists.txt`의 `sr_vendor_sky`(STATIC, `-w`)가 두 `.cc`와 shim `tflite-interpreter.cc`를 컴파일합니다. 같은 shim 구현이 `sr_vendor_face`에도 있으므로(Ceres를 끌고 오지 않으려고 sky 쪽에 따로 컴파일) **두 타깃을 한 실행 파일에 링크하면 `c8::TFLiteInterpreter` 심볼이 중복됩니다.** 웹 통합(US-022)이 face와 sky를 한 모듈로 묶을 때 한쪽만 남겨야 합니다.
- closure (VERIFIED, 2026-10-20): `g++ -std=c++17 -fsyntax-only -DJAVASCRIPT -DC8_NO_EXCEPT -Ixr_shim -Ixr_shim/reality/engine/deepnets -Ixr_reference -Ilibcxx_compat/include <2 TU>`가 새 파일 없이 통과합니다(기존 closure로 충분, 새 shim은 `tflite-model-operations.h` 하나).

## lighting (full-parity PRD US-005, 2026-10-22, 패키지 D)

- `reality/engine/lighting/lighting-estimator.{cc,h}` 2개를 patch 없이 byte 그대로 vendoring했습니다(`tu` 1 + `header` 1). 둘 다 clone 작업 트리 파일의 sha256이 `upstream_sha256`과 같고(`check_vendor.py --upstream --upstream-base worktree` PASS), `vendored_sha256`도 같습니다.
- 닫힘(closure): `lighting-estimator.cc`의 include는 `c8/pixels/pixel-transforms.h`, `pixels.h`, `c8/task-queue.h`, `c8/thread-pool.h`, `c8/stats/scope-timer.h`(shim)이고 모두 기존 MANIFEST 또는 shim에 있습니다. 새 의존 파일은 없습니다. `estimateExposureScore`는 이미 vendoring된 `c8/pixels/pixel-transforms.cc:1927`에 있습니다.
- 링크: `pixel-transforms.o`를 처음 끌어들이는 실행 파일은 `GRAYSCALE_RGB_256`, `decodeFloat`(`c8/color-maps.cc`, `c8/pixels/packed.cc`)가 필요합니다. 두 파일은 face 레인에서 이미 vendoring돼 있었으므로 `third_party/CMakeLists.txt`의 `SR_VENDOR_C8_SRCS`에 `lighting-estimator.cc`와 함께 추가했습니다(`sr_vendor_face_pixels` OBJECT와 같은 파일이며, 직접 link line에 오른 object가 archive member보다 먼저 정의를 채웁니다).
- 대체되는 코드: `src/engine/lighting.cc`의 `ExposureScore`가 직접 포팅한 `estimateExposureScore` 대신 `c8::LightingEstimator::estimateLighting`을 부릅니다. `lighting-request-executor.cc`는 capnp 의존이라 vendoring하지 않고 `EstimateExposure`(마지막 level 선택, 4채널 stride)가 계속 대신합니다.
- 비교 기준 `tools/check_vendor.py`의 `TU_8W`에 `lighting-estimator.cc` 1줄을 추가했습니다.

## shim (`third_party/xr_shim/`, 별도 MANIFEST)

include 경로에서 이 트리보다 앞에 둡니다.

| 파일 | 출처 (lane B) | 변경 |
|---|---|---|
| `c8/stats/scope-timer.h` | `slam_refact/tools/re/public_build/shim/c8/stats/scope-timer.h` | 없음 (no-op ScopeTimer) |
| `c8/geometry/intrinsics.h` | `…/shim/c8/geometry/intrinsics.h` | 없음 (capnp 없는 include만) |
| `parameter-data.cc` | `…/shim_src/parameter-data-shim.cc` | JSON 경로의 `throw std::runtime_error` → `C8_THROW` (`-fno-exceptions`, D-15) |
| `reality/engine/api/reality.capnp.h` | 새로 작성 (US-011). upstream 열은 schema `reality/engine/api/reality.capnp`(commit 위와 같음) | capnp 생성 헤더 대신 `CurvyGeometry`(schema :393-406)만 plain struct `Data` + `Builder`/`Reader`로 제공. `detection-image.{h,cc}`가 수정 없이 컴파일됨 |
| `reality/engine/api/face.capnp.h` | 새로 작성 (US-018). upstream 열은 schema `reality/engine/api/face.capnp` | `AttachmentPointMsg::AttachmentName`(schema :97-134, 36값)과 `DebugOptions::Reader`(mesh.capnp :21-25의 `getOnlyDetectFaces`)만 |
| `reality/engine/deepnets/tflite-interpreter.h`, `tflite-interpreter.cc` | 새로 작성 (US-018). upstream 열은 원본 `deepnets/tflite-interpreter.{h,cc}` | 같은 호출면(`typed_input_tensor<float>`, `Invoke`, `typed_output_tensor<float>`)을 `slam_refact::ml::Interpreter`로 실행. XNNPACK cache 크기·thread 수 인자는 받기만 함 |
| `c8/io/capnp-messages.h` | 새로 작성 (US-018) | 기본 `ConstRootMessage<T>::reader()`만(`face-tracker.cc:243-246`) |
| `c8/pixels/render/object8.h` | 새로 작성 (US-019) | `CAM_PROJECTION_NEAR_CLIP`/`FAR_CLIP`(object8.h :21-22)만 |
| `c8/geometry/intrinsics-clip.h` | 새로 작성 (US-019). upstream 열은 `c8/geometry/intrinsics.cc` | class `Intrinsics`의 `toClipSpaceLeftHanded`, `rotateCropAndScaleIntrinsics`, `toClipSpaceMatLeftHanded`, `cropAndScaleIntrinsics`(intrinsics.cc :129-193, :245-263, 같은 float 연산 순서). world SLAM shim `intrinsics.h`는 그대로 |
| `reality/engine/deepnets/tflite-model-operations.h` | 새로 작성 (US-020). upstream 열은 원본 헤더 | 빈 헤더. 원본은 `rotateSquareTensorFP16`/`rotateModel`(third_party/half 의존)을 선언하며 `tflite-multiclassifier.cc`는 include만 하고 호출하지 않음 |

## 빌드·검증 기록 (2026-10-15)

- 단일 스레드 결과 확인: lane B의 gr8 184/184 parity(S1 U4)와 `probe-cxx20/cxx17` native 실행은 **`JAVASCRIPT` 없이(멀티스레드 검출 경로)** 측정되었습니다. lane B의 WASM probe(`probe-wasm.out`)는 `-DJAVASCRIPT`입니다. 이 vendored closure를 `-DJAVASCRIPT`(단일 스레드)로 native g++에서 빌드한 vendor probe의 결과 38줄이 `probe-wasm.out`과 같습니다(`sr_tests_vendor` `VendorProbe.MatchesWasmLibcxxReference`, VERIFIED). 같은 closure를 em++로 빌드해 node로 돌린 결과도 38줄 모두 같습니다(VERIFIED).
- 대조 실험: `libcxx-compat.patch`만 뺀 closure로 같은 테스트를 빌드하면 `A.retainBest`, `A.detectAndCompute`, `A.kp[0]`, `B.gorb` 줄이 다릅니다(FAIL, 예상대로).
