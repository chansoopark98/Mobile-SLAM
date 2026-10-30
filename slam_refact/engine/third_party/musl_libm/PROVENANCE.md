# third_party/musl_libm 출처 기록

## 2026-10-30 공개 트리 정리

- 원 출처·저작자 고지: [루트 README](../../../../README.md#external-code). 아래 이전 기록의 경로는 공개 경로 이름으로 갱신했습니다.
- `upstream_sha256`: 기존 source proof-base의 원래 bytes 해시 유지. `vendored_sha256`: 현재 프로젝트 파일 bytes 해시. 이름·주석 정리로 달라진 두 값을 같은 값으로 덮어쓰지 않습니다.
- 검증: `python3 slam_refact/engine/tools/check_vendor.py --upstream --data`. 원본 해시 확인 → 기존 patch 적용 → `public_source_bytes`의 정해진 이름·저자 연락처 주석 정리 → local bytes 일치를 검사합니다. 알고리즘 임의 변경은 허용하지 않습니다.
- musl `COPYRIGHT` 원문과 각 소스의 FreeBSD/Sun 고지는 유지합니다. 기존 upstream 해시와 버전은 바꾸지 않았습니다.


- 작성: 2026-10-16 (precision lane). 근거 증거: `build/slam-refact-20261016/precision/`.
- 출처: 설치된 emsdk 5.0.0 (`~/emsdk`, emscripten `a7c5deabd7c88ba1c38ebe988112256775f944c6`)의 musl 1.2.5
  (`upstream/emscripten/system/lib/libc/musl`, `VERSION` = 1.2.5). 원본 xr-slam.wasm도 emscripten의 musl libm을 링크합니다.
- 라이선스: musl MIT (`COPYRIGHT` 원문). `src/*.c`의 머리 주석(FreeBSD/Sun 고지)은 그대로 둡니다.
- 파일별 upstream 경로와 sha256은 `MANIFEST.tsv`가 정본입니다(`tools/check_vendor.py --upstream`가 확인).

## 왜 필요한가

- host(g++/glibc) 빌드는 glibc의 `sinf`/`cosf`/`atan2f`/`log10f`를 씁니다. 이 함수들은 마지막 bit가 musl과 다르게 반올림되는 인수가 있습니다
  (무작위 인수에서 `sinf` 약 0.5 %, `cosf` 약 2 %, `atan2f`·`log10f`는 드뭄).
- VERIFIED: 원본 wasm의 461(`sinf`)/462(`cosf`)/843(`atan2f`)를 격리 실행한 결과는 이 vendored musl 코드와 300,000/300,000 일치하고, glibc와는 다릅니다.
- VERIFIED: 이 파일들을 링크한 native replay는 4개 exact-input bundle 전 구간에서 WASM(em++, musl) replay와 trace·내부 dump가 bit 단위로 같습니다.
  빼면 flip6-s3 trace frame 24(ReplaceYaw의 `atan2f`), 그 전에는 frame 20(`cosf`/`sinf`)에서 갈라집니다. `log10f`는 TrackPoints의 depth-certainty weight에서 필요했습니다.
- 2차 추가(2026-10-16 fix lane, 근거 `build/slam-refact-20261016/fix/libm/`): `acosf`, `logf`, `expf`, `pow`, `sin`, `cos`, `sincos`와 그 의존 파일.
  - VERIFIED: 원본 wasm에서 엔진 호출 지점이 부르는 함수를 확인했습니다. 993 `acosf`(926 MapTracker, 1266 FFT), 354 `logf`(1266), 1217 `expf`(1199 kernel),
    342 `pow`(1169 kalman 등), 898/899 `sin`/`cos`(1569 Ceres AngleAxisToRotationMatrix 등)이며, 각각 격리 실행 결과가 emsdk musl과 200,000/200,000 같습니다.
  - VERIFIED: 이 musl 소스로 빌드한 host sweep(11개 함수, 2,200,004개 결과)이 em++ 5.0.0 빌드(node)와 bit 단위로 같습니다. glibc 2.35는 `acosf` 1.9 %, `sin`/`cos` 약 3 %, `pow` 0.07 %에서 다릅니다(`logf`/`expf`는 이 sweep에서 차이 없음).
  - `pyramid_layout.cc`의 `1.0f / pow(1.44, i)`는 원본(459)에서 f32 상수로 접혀 있고 host·em++ 값과 같습니다(시험으로 고정).
  - 한계: `GetQuaternion`(sensor_intake.cc)은 원본에서 JS(V8 `Math.sin`/`Math.cos`)로 계산됩니다. V8 결과는 musl과 약 0.9 %에서 마지막 bit가 다릅니다(float 변환 전 double 값). 이 차이는 이 vendoring 범위 밖입니다.
- 증명되지 않은 다른 libm 함수(`acos`, `asinf`, `atan2`, `cbrt`, `cosh`, `exp`, `log`, `tanf` 등)는 vendoring하지 않았습니다.

## 구성

- `src/{sinf,cosf,sincosf,__sindf,__cosdf,__rem_pio2f,__rem_pio2_large,atan2f,atanf,log10f}.c`: musl `src/math/`의 원문(verbatim).
- `src/{acosf,logf,logf_data,expf,exp2f_data,sin,cos,sincos,__sin,__cos,__rem_pio2,pow,pow_data,exp_data,__math_*}.c`와 `src/*_data.h`: 원문(verbatim).
- `include/libm.h`: musl `src/internal/libm.h`에서 위 파일이 쓰는 부분만 남긴 발췌(수동). 추가: glibc 헤더에서 `M_PI_2`·`sincosf`를 보이게 하는 `_GNU_SOURCE`, musl `features.h`의 `hidden`, glibc `<math.h>`의 내부 `__sin`/`__cos` 선언과 겹치지 않도록 musl 커널 이름을 바꾸는 `#define`(코드는 그대로).
- `include/features.h`: musl `src/include/features.h` 발췌(수동). `*_data.h`가 `hidden`을 얻는 곳이며, host의 `features.h`를 `#include_next`로 이어 씁니다.
- 빌드: `third_party/CMakeLists.txt`의 `sr_musl_libm`(OBJECT, host 전용). `sr_vendor_c8`이 PUBLIC으로 링크하므로 엔진 실행 파일 안에서 이 정의가 공유 glibc libm보다 먼저 묶입니다.
  em++ 빌드는 emsdk의 같은 musl을 그대로 링크하므로 이 타깃을 만들지 않습니다.
- 시험: `tests/compat/musl_libm_test.cc` (`sr_tests_musl_libm`, label unit).
