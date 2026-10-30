# slam_refact engine — 제3자 코드 고지

이 디렉터리(`slam_refact/engine/third_party/`)의 코드는 **빌드 시 소스**입니다. 제품 `web/` runtime 의존성이 아닙니다(architecture.md D-12).
출처(저장소, commit, 경로, hash, patch)는 각 디렉터리의 `MANIFEST.tsv`와 `PROVENANCE.md`에 있습니다.
reference runtime 공개 파일 자체에 대해서는 repo 정책(commit `5b8f6f5`)에 따라 출처만 기록하고, 작성자 이름은 파일에 유지하고 원래 연락처는 루트 README의 외부 코드 고지에 모읍니다. 아래 제3자 라이선스 고지는 원문을 유지합니다.

| 제3자 코드 | 위치 | 고지 |
|---|---|---|
| FLANN (OpenCV `cvlite` 경유) — Copyright 2008-2009 Marius Muja, David G. Lowe; LSH: Vincent Rabaud. BSD License | `xr_source/third_party/cvlite/flann/*.h` (`any.h` 외 15개) | 각 파일 머리의 BSD 고지 원문 유지 |
| FLANN `any.h` — (C) Christopher Diggins 2005-2011, Pablo Aguilar 2005, Kevlin Henney 2001. Boost Software License 1.0 | `xr_source/third_party/cvlite/flann/any.h` | 파일 머리의 고지 원문 유지 |
| constexpr MurmurHash3 — Copyright (c) 2015 Tamás Szelei. MIT License | `xr_source/third_party/murmurhash/constexpr_murmur3.h` | 파일 머리의 MIT 고지 원문 유지 |
| LLVM libc++ (emsdk 5.0.0, llvm-project 21.1.8) — Apache License v2.0 with LLVM Exceptions | `libcxx_compat/` | 각 파일의 SPDX 머리말 유지, `libcxx_compat/LICENSE.TXT` 원문 |
| musl libc 1.2.5 libm subset (emsdk 5.0.0) — Copyright © 2005-2020 Rich Felker, et al. MIT License; FreeBSD msun / Sun Microsystems 고지 | `musl_libm/` (host 빌드 전용) | `musl_libm/COPYRIGHT` 원문, 각 소스 머리 고지 유지 |
| kissfft | vendoring 안 함 (Eigen `unsupported/Eigen/FFT`로 대체, S5 §7.3) | 해당 없음 |
