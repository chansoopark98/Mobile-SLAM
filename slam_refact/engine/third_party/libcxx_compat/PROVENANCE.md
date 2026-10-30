# third_party/libcxx_compat 출처 기록

## 2026-10-30 공개 트리 정리

- 원 출처·저작자 고지: [루트 README](../../../../README.md#external-code). 아래 이전 기록의 경로는 공개 경로 이름으로 갱신했습니다.
- `upstream_sha256`: 기존 source proof-base의 원래 bytes 해시 유지. `vendored_sha256`: 현재 프로젝트 파일 bytes 해시. 이름·주석 정리로 달라진 두 값을 같은 값으로 덮어쓰지 않습니다.
- 검증: `python3 slam_refact/engine/tools/check_vendor.py --upstream --data`. 원본 해시 확인 → 기존 patch 적용 → `public_source_bytes`의 정해진 이름·저자 연락처 주석 정리 → local bytes 일치를 검사합니다. 알고리즘 임의 변경은 허용하지 않습니다.
- LLVM 라이선스 원문·SPDX와 알고리즘 소스는 유지합니다. 기존 upstream 해시와 버전은 바꾸지 않았습니다.


- 작성: WP0a, 2026-10-15. 근거: `architecture.md` §5.2–5.3, D-17 (후보 ① emsdk 5.0.0).
- 출처: 설치된 emsdk 5.0.0 (`~/emsdk`, emscripten `a7c5deabd7c88ba1c38ebe988112256775f944c6`)의 libc++.
  - `upstream/emscripten/system/lib/libcxx/readme.txt`: llvm-project **21.1.8** + emscripten-libs-21 patch.
  - 헤더 경로: `$EMSDK/upstream/emscripten/system/lib/libcxx/include/` (sysroot `cache/sysroot/include/c++/v1`과 대상 파일 동일).
  - `__sort` 정의: `system/lib/libcxx/src/algorithm.cpp`.
- 라이선스: Apache-2.0 WITH LLVM-exception. 원 SPDX 머리말을 각 파일에 유지하고 `LICENSE.TXT`(원문 그대로)를 함께 둡니다.
- 파일별 upstream 경로·sha256과 vendored sha256은 `MANIFEST.tsv`가 정본입니다.

## 만든 방법

1. **기계적 변환** `tools/libcxx_compat_transform.py <out>` (알고리즘 줄은 바꾸지 않음)
   - `_LIBCPP_BEGIN/END_NAMESPACE_STD` → `namespace slam_refact::libcxx`
   - 나머지 `_LIBCPP_*` 매크로 → `_SR_LIBCPP_*` (em++ 빌드에서 진짜 libc++ 매크로와 충돌하지 않게)
   - `std::__x`와 발췌한 공개 이름(`sort`, `nth_element`, …) → `::slam_refact::libcxx::x`
   - libc++ 내부 `#include <__…>` → 발췌 헤더 또는 `__config.h` 주석
2. **수동 편집** (MANIFEST `patches` = `transform+manual:…`, `check_vendor.py --upstream`가 diff 줄 수를 보고)
   - `__algorithm/sort.h`: `extern template … __sort<…>` 블록(라이브러리 인스턴스화)을 `src/algorithm.cpp`의 `__sort` 정의로 교체. C++20 전용 `ranges::less`는 `__config.h`에 발췌.
   - `__algorithm/shuffle.h`: debug 전용 `__libcpp_debug_randomizer`와 C++14 `random_shuffle` 삭제. **DECISION(§5.2)**: `uniform_int_distribution<ptrdiff_t>` → `uniform_int_distribution<__sr_wasm32_ptrdiff_t>`(= `int32_t`). 원본 wasm32의 ptrdiff_t 폭을 native에서도 씁니다.
   - `__random/{uniform_int,uniform_real,normal}_distribution.h`: iostream `operator<<`/`>>` 삭제.
3. **새로 쓴 파일**
   - `__config.h`: `<__config>` 대체 매크로(C++17, hardening NONE, debug randomization 없음)와, 발췌 알고리즘이 부르는 내부 helper의 발췌(`desugars_to`, `__less`, `ranges::less/greater`, `_IterOps<_ClassicAlgPolicy>`의 사용 멤버, `__countl_zero`/`__countr_zero`/`__libcpp_blsr`/`__bit_log2`, `__to_unsigned_like`, `__invoke`, `__identity`).
     - `__builtin_clzg/ctzg`(clang ≥ 19)는 `__builtin_clz*`/`__builtin_ctz*`로 같은 값을 냅니다(0 입력은 digits).
     - `__libcpp_is_contiguous_iterator`/`__unwrap_iter`: 포인터와 표준 라이브러리 `std::vector`/`std::string` iterator를 contiguous로 봅니다(INFERRED: 엔진이 넘기는 iterator 종류를 덮음). 그 밖의 iterator는 generic(비 branchless) 경로로 가며, libc++도 그런 iterator에서는 같은 경로입니다.
   - `queue.h`: `<queue>`의 `priority_queue`에서 쓰는 멤버(기본/비교자 생성자, `empty`, `size`, `top`, `push`, `emplace`, `pop`)만 발췌.
   - `algorithm.h`, `random.h`: umbrella.

## 공개 진입점

`sort`, `nth_element`, `partial_sort`, `make_heap`, `push_heap`, `pop_heap`, `sort_heap`, `min_element`, `shuffle`, `uniform_int_distribution`, `generate_canonical`, `uniform_real_distribution`, `normal_distribution`, `priority_queue`. 엔진(`std::mt19937`)은 표준이므로 발췌하지 않습니다.

## 검증 (2026-10-15)

| 확인 | 명령 | 결과 |
|---|---|---|
| native g++ 11.4 (libstdc++) vs em++ libc++ | `ctest` `LibcxxCompatGolden.MatchesEmsdkLibcxx` | 394/394 case digest 일치 (VERIFIED) |
| golden이 libstdc++와 구별되는지 | `LibcxxCompatGolden.GoldenDiscriminatesHostStandardLibrary` | libstdc++ `std::`는 314/394 case가 다름 (VERIFIED) |
| em++ 위의 compat | `tools/gen_libcxx_golden.sh --check tests/data/compat/libcxx-golden.json` | em++ libc++ 재생성 = golden, em++ compat = golden (VERIFIED) |

case 구성은 `tests/compat/libcxx_golden_cases.h`: 8 원소형(동점을 볼 수 있는 record, `±0.0f`가 섞인 float의 branchless 경로 포함) × 7 입력 패턴 × 27 길이(0–5000)에서 sort / nth_element / partial_sort / make+sort_heap / push+pop_heap, priority_queue, shuffle(mt19937 seed 5489, 길이 0–70·100–5000, 다음 raw 출력 포함), 분포 21종.

`normal_distribution`은 `sqrt`/`log`(libm)를 씁니다. 이번 golden에서는 glibc와 emscripten musl 결과가 같았지만, §5.1에 따라 libm을 쓰는 값의 bit 동일은 일반 보장이 아닙니다.

## libc++ 14 sort (2026-10-16, precision lane)

- `__algorithm/sort_llvm14.h`: libc++ 14 `std::sort` (introsort: 길이 ≤ 30 insertion sort, 1000 이상 median-of-5, pivot과 같은 원소 분할, `__insertion_sort_incomplete`, depth 소진 시 swap + sift_down heap sort)와 그 heap helper를 namespace `slam_refact::libcxx::llvm14`로 발췌했습니다.
  - 출처: libc++-14-dev `1:14.0.0-1ubuntu1.1` (`/usr/lib/llvm-14/include/c++/v1/__algorithm/{sort,sift_down,make_heap,pop_heap,sort_heap,partial_sort}.h`). 생성: `tools/gen_libcxx14_sort_excerpt.py` (알고리즘 줄은 그대로, 한정자·속성 매크로만 바꿈). MANIFEST의 upstream_sha256은 `sort.h`입니다.
  - `__algorithm/sort.h`의 공개 `sort`는 `llvm14::sort`를 부릅니다(수동 편집). 나머지 LLVM 21 내부 함수는 `nth_element`가 씁니다.
- 근거 (VERIFIED, `build/slam-refact-20261016/precision/`):
  - 원본 xr-slam.wasm의 f1555(SelectSubset 점수 정렬)는 위 libc++ 14 구조입니다. 격리 실행한 f1555는 libc++ 14 `std::sort`와 150/150 case에서 같은 순서를 냅니다(엔진 점수 18개, 동점이 많은 무작위·정렬·역순·organ pipe, 길이 0–3000).
  - LLVM 21 `sort`는 엔진 점수 18개 중 15개에서 원본과 순서가 다릅니다. 이 차이로 flip6-s3 exact-input replay의 subset이 frame 16에서 갈라졌습니다.
  - golden 비교: libc++ 14와 emsdk 5.0.0(LLVM 21)은 `sort`(30/56)와 `nth_element`(8/56)에서만 다르고, heap/partial_sort/shuffle/분포는 같습니다. `nth_element`는 원본과 대조하지 않았으므로 LLVM 21 그대로 둡니다(UNVERIFIED).
- 검증: `tools/gen_libcxx14_sort_golden.sh --check tests/data/compat/libcxx14-sort-golden.json` (libc++ 14 `std::sort` = compat `sort`, 56 case), GTest `LibcxxCompatGolden.SortMatchesLibcxx14`.
