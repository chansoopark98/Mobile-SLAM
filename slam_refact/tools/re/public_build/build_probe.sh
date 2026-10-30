#!/usr/bin/env bash
# CMake/Bazel-free compile probe for public xr_reference/xr_reference@4edfa484 world-SLAM sources.
#
# Builds object files and static archives for the minimal closures, links probe executables and
# runs them on fixed synthetic input:
#   libc8core.a   : c8 math/geometry/pixels (+ parameter-data shim, no nlohmann/capnp)
#   libslamfeat.a : reality/engine/features (gr8gl CPU half, frame-point, descriptors, matchers)
#   libslamgeo.a  : reality/engine/geometry (bundle, residuals, pnp, epipolar) -> needs Ceres
#   libcvlite.a   : third_party/cvlite (OpenCV 3.2 fork) for the Gr8Cpu path only
# Probe variants:
#   probe-cxx20      : g++ -std=c++20, overlay-gcc (1 GCC compat patch)            [no cvlite]
#   probe-cxx17      : g++ -std=c++17, overlay-cxx17 (GCC compat + 3 port-down patches) [no cvlite]
#   probe-gr8cpu     : g++ -std=c++20, Gr8Cpu (cvlite FAST/Harris) path
#
# Generated sources: gen_cvlite.sh (cvlite genrules) and the sympy generator for
# analytical-residual/pose-estimation.cc (Bazel genrule), both written under <out_dir>/gen.
#
# Usage: build_probe.sh <out_dir> [jobs<=4]
set -euo pipefail

ROOT_REPO="$(cd "$(dirname "$0")/../../../.." && pwd)"
SRC_ROOT="${ROOT_REPO}/assets/references/xr_reference/official/xr_reference"
TOOL_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT="${1:?out dir}"
JOBS="${2:-4}"
if (( JOBS < 1 || JOBS > 4 )); then echo "jobs must be 1..4" >&2; exit 2; fi
mkdir -p "${OUT}"
OUT="$(cd "${OUT}" && pwd)"
CERES_SRC="${ROOT_REPO}/build/_deps/ceres-solver-src"
CERES_BUILD="${ROOT_REPO}/build/dev-native/_deps/ceres-solver-build"
CERES_LIB="${CERES_BUILD}/lib/libceres.a"
GEN_DIR="${OUT}/gen"

"${TOOL_DIR}/gen_cvlite.sh" "${GEN_DIR}" >/dev/null
# Bazel genrule :pose-estimation-analytical-cc (sympy code generator; reference runtime pins sympy 1.11.1).
mkdir -p "${GEN_DIR}/reality/engine/geometry/analytical-residual"
python3 "${SRC_ROOT}/reality/engine/geometry/analytical-residual/generate-pose-estimation.py" \
  "${GEN_DIR}/reality/engine/geometry/analytical-residual/pose-estimation.cc"
python3 -c 'import sympy; print("sympy", sympy.__version__)' > "${OUT}/sympy-version.txt"
"${TOOL_DIR}/apply_overlay.sh" "${OUT}/overlay-gcc" \
  "${TOOL_DIR}/patches/gcc-frame-point-comparator.patch" >/dev/null
"${TOOL_DIR}/apply_overlay.sh" "${OUT}/overlay-cxx17" \
  "${TOOL_DIR}/patches/gcc-frame-point-comparator.patch" \
  "${TOOL_DIR}/patches/cxx17-constexpr-uninit.patch" \
  "${TOOL_DIR}/patches/cxx17-set-contains.patch" \
  "${TOOL_DIR}/patches/cxx17-erase-if.patch" >/dev/null

C8_SRCS=(
  c8/hmatrix.cc c8/hpoint.cc c8/hvector.cc c8/quaternion.cc c8/c8-log.cc c8/exceptions.cc
  c8/string/format.cc c8/task-queue.cc c8/thread-pool.cc c8/time/now.cc
  c8/geometry/egomotion.cc c8/geometry/homography.cc c8/geometry/parameterized-geometry.cc
  c8/geometry/vectors.cc c8/geometry/device-pose.cc c8/geometry/two-d.cc c8/geometry/line.cc
  c8/geometry/interpolation.cc
  c8/pixels/gr8-pyramid.cc c8/pixels/pixel-transforms.cc c8/pixels/image-roi.cc
  reality/engine/binning/linear-bin.cc
)
FEAT_SRCS=(
  reality/engine/features/gr8gl.cc reality/engine/features/image-point.cc
  reality/engine/features/image-descriptor.cc reality/engine/features/frame-point.cc
  reality/engine/features/keypoint-queue.cc reality/engine/features/feature-scales.cc
  reality/engine/features/feature-manager.cc reality/engine/features/detection-config.cc
  reality/engine/features/descriptor-lsh-index.cc reality/engine/features/descriptor-bit-counter.cc
  reality/engine/features/global-matcher.cc reality/engine/features/local-matcher.cc
  reality/engine/features/point-descriptor.cc reality/engine/features/point-matches.cc
  reality/engine/features/gr8cpu-interface.cc reality/engine/features/gr8cpu-nop.cc
  reality/engine/tracking/ray-point-filter.cc reality/engine/tracking/rolling-lin-reg.cc
)
GEO_SRCS=(
  reality/engine/geometry/bundle.cc reality/engine/geometry/bundle-residual.cc
  reality/engine/geometry/bundle-residual-analytic.cc reality/engine/geometry/observed-point.cc
  reality/engine/geometry/epipolar.cc reality/engine/geometry/pose-pnp.cc
  reality/engine/geometry/ap3p.cc reality/engine/geometry/p2p.cc
  reality/engine/geometry/reprojection.cc reality/engine/geometry/homography-decomp.cc
  reality/engine/geometry/point-regression.cc
  reality/engine/geometry/analytical-residual/pose-estimation.cc
)
GR8CPU_SRCS=(
  reality/engine/features/gr8cpu.cc reality/engine/features/gr8.cc
  reality/engine/features/opencv-image-descriptor.cc c8/cvlite/convert.cc
)
CVLITE_SRCS=(
  third_party/cvlite/features2d/fast.cc third_party/cvlite/features2d/fast-score.cc
  third_party/cvlite/features2d/feature2d.cc third_party/cvlite/features2d/keypoint.cc
  third_party/cvlite/imgproc/filter.cc third_party/cvlite/imgproc/imgwarp.cc
  third_party/cvlite/imgproc/smooth.cc third_party/cvlite/imgproc/tables.cc
  third_party/cvlite/imgproc/thresh.cc
)
while IFS= read -r f; do CVLITE_SRCS+=("${f#${SRC_ROOT}/}"); done \
  < <(find "${SRC_ROOT}/third_party/cvlite/core" -maxdepth 1 -name '*.cpp' | sort)

# compile <variant> <std> <overlay> <extra-flags> <src>  -> echoes object path
compile_obj() {
  local variant="$1" std="$2" ov="$3" extra="$4" rel="$5"
  local obj="${OUT}/obj/${variant}/${rel%.*}.o" log="${OUT}/obj/${variant}/${rel%.*}.log"
  mkdir -p "$(dirname "${obj}")"
  local src="${SRC_ROOT}/${rel}"
  [[ -f "${GEN_DIR}/${rel}" ]] && src="${GEN_DIR}/${rel}"
  [[ -n "${ov}" && -f "${ov}/${rel}" ]] && src="${ov}/${rel}"
  local inc=(-I"${TOOL_DIR}/shim")
  [[ -n "${ov}" ]] && inc+=(-I"${ov}")
  inc+=(-I"${GEN_DIR}" -I"${SRC_ROOT}" -I/usr/include/eigen3 -I"${CERES_SRC}/include"
        -I"${CERES_BUILD}/include" -I"${CERES_SRC}/internal/ceres/miniglog")
  # shellcheck disable=SC2086
  if g++ -std="${std}" -O2 -w -fPIC -D_USE_MATH_DEFINES ${extra} "${inc[@]}" -c "${src}" \
      -o "${obj}" > "${log}" 2>&1; then
    printf 'OK\t%s\t%s\n' "${variant}" "${rel}"
  else
    printf 'FAIL\t%s\t%s\t%s\n' "${variant}" "${rel}" "$(grep -m1 error "${log}" | sed "s|${SRC_ROOT}/||g")"
  fi
}
export -f compile_obj
export OUT SRC_ROOT TOOL_DIR GEN_DIR CERES_SRC CERES_BUILD

STATUS="${OUT}/build-status.tsv"
: > "${STATUS}"
build_set() {  # variant std overlay extra -- files...
  local variant="$1" std="$2" ov="$3" extra="$4"; shift 4
  printf '%s\n' "$@" | xargs -P "${JOBS}" -I{} bash -c \
    'compile_obj "$0" "$1" "$2" "$3" "$4"' "${variant}" "${std}" "${ov}" "${extra}" {} >> "${STATUS}"
}
archive() {  # variant lib files...
  local variant="$1" lib="$2"; shift 2
  local objs=()
  for f in "$@"; do
    local o="${OUT}/obj/${variant}/${f%.*}.o"
    [[ -f "${o}" ]] && objs+=("${o}")
  done
  rm -f "${OUT}/lib/${variant}/${lib}"; mkdir -p "${OUT}/lib/${variant}"
  ar rcs "${OUT}/lib/${variant}/${lib}" "${objs[@]}"
}

PARAM_SHIM="${TOOL_DIR}/shim_src/parameter-data-shim.cc"
PROBE_MAIN="${TOOL_DIR}/probe/probe_main.cc"
GR8CPU_MAIN="${TOOL_DIR}/probe/probe_gr8cpu.cc"

for v in cxx20 cxx17; do
  std="c++20"; ov="${OUT}/overlay-gcc"
  [[ "${v}" == cxx17 ]] && { std="c++17"; ov="${OUT}/overlay-cxx17"; }
  build_set "${v}" "${std}" "${ov}" "" "${C8_SRCS[@]}" "${FEAT_SRCS[@]}" "${GEO_SRCS[@]}"
  archive "${v}" libc8core.a "${C8_SRCS[@]}"
  archive "${v}" libslamfeat.a "${FEAT_SRCS[@]}"
  archive "${v}" libslamgeo.a "${GEO_SRCS[@]}"
  inc=(-I"${TOOL_DIR}/shim" -I"${ov}" -I"${GEN_DIR}" -I"${SRC_ROOT}" -I/usr/include/eigen3
       -I"${CERES_SRC}/include" -I"${CERES_BUILD}/include" -I"${CERES_SRC}/internal/ceres/miniglog")
  if g++ -std="${std}" -O2 -w -D_USE_MATH_DEFINES "${inc[@]}" "${PROBE_MAIN}" "${PARAM_SHIM}" \
      "${OUT}/lib/${v}/libslamgeo.a" "${OUT}/lib/${v}/libslamfeat.a" "${OUT}/lib/${v}/libc8core.a" \
      "${CERES_LIB}" -lpthread -o "${OUT}/probe-${v}" > "${OUT}/link-${v}.log" 2>&1; then
    printf 'OK\t%s\tlink probe-%s\n' "${v}" "${v}" >> "${STATUS}"
  else
    printf 'FAIL\t%s\tlink probe-%s\t%s\n' "${v}" "${v}" "$(grep -m1 -E 'error|undefined' "${OUT}/link-${v}.log")" >> "${STATUS}"
  fi
done

# Gr8Cpu path (cvlite = OpenCV 3.2 fork). C++20 + overlay-gcc only.
build_set gr8cpu c++20 "${OUT}/overlay-gcc" "-D__OPENCV_BUILD -fexceptions" "${CVLITE_SRCS[@]}"
build_set gr8cpu c++20 "${OUT}/overlay-gcc" "" "${GR8CPU_SRCS[@]}"
archive gr8cpu libcvlite.a "${CVLITE_SRCS[@]}"
archive gr8cpu libgr8cpu.a "${GR8CPU_SRCS[@]}"
inc=(-I"${TOOL_DIR}/shim" -I"${OUT}/overlay-gcc" -I"${GEN_DIR}" -I"${SRC_ROOT}")
if g++ -std=c++20 -O2 -w -D_USE_MATH_DEFINES "${inc[@]}" "${GR8CPU_MAIN}" \
    "${OUT}/lib/gr8cpu/libgr8cpu.a" "${OUT}/lib/cxx20/libslamfeat.a" "${OUT}/lib/gr8cpu/libcvlite.a" \
    "${OUT}/lib/cxx20/libc8core.a" -lz -lpthread -o "${OUT}/probe-gr8cpu" > "${OUT}/link-gr8cpu.log" 2>&1; then
  printf 'OK\tgr8cpu\tlink probe-gr8cpu\n' >> "${STATUS}"
else
  printf 'FAIL\tgr8cpu\tlink probe-gr8cpu\t%s\n' "$(grep -m1 -E 'error|undefined' "${OUT}/link-gr8cpu.log")" >> "${STATUS}"
fi

sort -o "${STATUS}" "${STATUS}"
awk -F'\t' '{n[$2" "$1]++} END {for (k in n) print k, n[k]}' "${STATUS}" | sort
