# Dependency license notices

These notices remain with the public source repository when local dependency checkouts, build directories and installed headers are excluded from Git.

| Dependency | Source/version | Preserved notices |
|---|---|---|
| OpenCV | 4.5.5 | `opencv/LICENSE`, existing installed SoftFloat/ADE/zlib notices and the leading OpenCV/FLANN source notices |
| Ceres Solver | 2.2.0, commit `85331393dc0dff09f6fb9903ab0c4bfa3e134b01` | `ceres/LICENSE`, miniglog source notice |
| Eigen | source version 3.4.0, local upstream checkout `3147391d946bb4b6c68edd901f2add6ac1f31f8c` | All upstream `COPYING.*` files; `COPYING.README` explains their scope |
| Boost Software License | Version 1.0, referenced by FLANN `any.h` | Original notice and the verbatim BSL-1.0 license stanza retained from the installed Boost 1.74 copyright file |

- `SOURCES.json` records each notice's source, version, copy mode and SHA-256.
- Full license files are copied verbatim. A `leading notice` contains only the original header comment; its program body is not copied. The Boost stanza retains its copyright-file formatting.
- The ADE notice is preserved from the existing OpenCV installation; the minimal WASM build disables the G-API module. Its presence does not claim that ADE is linked.
- New builds retain the notices supplied with their exact dependency sources. These copies identify the existing dependency versions and are not a claim that a new build has been performed.
