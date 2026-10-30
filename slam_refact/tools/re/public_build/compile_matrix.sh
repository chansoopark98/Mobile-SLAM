#!/usr/bin/env bash
# Per-file standalone compile matrix for public xr_reference/xr_reference@4edfa484 sources.
#
# For every source in the list, run `-fsyntax-only` under several configurations: g++ 11 and the
# emsdk clang (host target) x -std=c++20/c++17 x {no shim, shim (no-op ScopeTimer, capnp-free
# intrinsics.h), shim + patch overlay (patches/*.patch: GCC compat, C++17 port-down)}. Records exit
# code and the first error line per (file, config) into <out>/compile-matrix.tsv and full logs
# under <out>/matrix-logs/.
#
# Usage: compile_matrix.sh <out_dir> [jobs<=4]
set -euo pipefail

ROOT_REPO="$(cd "$(dirname "$0")/../../../.." && pwd)"
SRC_ROOT="${ROOT_REPO}/assets/references/xr_reference/official/xr_reference"
TOOL_DIR="$(cd "$(dirname "$0")" && pwd)"
OUT="${1:?out dir}"
JOBS="${2:-4}"
if (( JOBS < 1 || JOBS > 4 )); then echo "jobs must be 1..4" >&2; exit 2; fi
mkdir -p "${OUT}/matrix-logs"
OUT="$(cd "${OUT}" && pwd)"

CERES_SRC="${ROOT_REPO}/build/_deps/ceres-solver-src"
CERES_BUILD="${ROOT_REPO}/build/dev-native/_deps/ceres-solver-build"
GEN_DIR="${OUT}/gen"

FILES=(
  reality/engine/features/gr8cpu.cc reality/engine/features/gr8cpu-interface.cc
  reality/engine/features/gr8cpu-nop.cc reality/engine/features/gr8.cc
  reality/engine/features/gr8gl.cc reality/engine/features/gr8gl-slow.cc
  reality/engine/features/gr8-feature-shader.cc reality/engine/features/gl-reality-frame.cc
  reality/engine/features/feature-detector.cc reality/engine/features/image-descriptor.cc
  reality/engine/features/image-point.cc reality/engine/features/frame-point.cc
  reality/engine/features/descriptor-lsh-index.cc reality/engine/features/descriptor-bit-counter.cc
  reality/engine/features/global-matcher.cc reality/engine/features/feature-manager.cc
  reality/engine/features/local-matcher.cc reality/engine/features/keypoint-queue.cc
  reality/engine/features/feature-scales.cc reality/engine/features/tracker-input.cc
  reality/engine/features/detection-config.cc reality/engine/features/point-descriptor.cc
  reality/engine/features/point-matches.cc reality/engine/features/tree-matcher.cc
  reality/engine/features/opencv-image-descriptor.cc reality/engine/features/dbscan.cc
  reality/engine/features/kmeans.cc
  reality/engine/geometry/bundle.cc reality/engine/geometry/bundle-residual.cc
  reality/engine/geometry/bundle-residual-analytic.cc reality/engine/geometry/observed-point.cc
  reality/engine/geometry/epipolar.cc reality/engine/geometry/pose-pnp.cc
  reality/engine/geometry/ap3p.cc reality/engine/geometry/p2p.cc
  reality/engine/geometry/reprojection.cc reality/engine/geometry/homography-decomp.cc
  reality/engine/geometry/point-regression.cc reality/engine/geometry/image-warp.cc
  reality/engine/geometry/orientation.cc
  reality/engine/tracking/tracker.cc reality/engine/tracking/ray-point-filter.cc
  reality/engine/tracking/tracking-sensor-event.cc reality/engine/tracking/fft-compute.cc
  reality/engine/tracking/rolling-lin-reg.cc
  reality/engine/pose/pose-request-executor.cc reality/engine/binning/linear-bin.cc
  c8/hmatrix.cc c8/hpoint.cc c8/hvector.cc c8/quaternion.cc c8/parameter-data.cc
  c8/c8-log.cc c8/exceptions.cc c8/string/format.cc c8/task-queue.cc c8/thread-pool.cc
  c8/geometry/egomotion.cc c8/geometry/homography.cc
  c8/geometry/intrinsics.cc c8/geometry/parameterized-geometry.cc c8/geometry/vectors.cc
  c8/geometry/device-pose.cc c8/geometry/worlds.cc
  c8/pixels/gr8-pyramid.cc c8/pixels/pixel-transforms.cc c8/pixels/image-roi.cc
  c8/pixels/pixel-buffer.cc c8/stats/scope-timer.cc c8/time/now.cc
)

CLANGXX="${HOME}/emsdk/upstream/bin/clang++"
OV_GCC="${OUT}/overlay-gcc"
OV_17="${OUT}/overlay-cxx17"
"${TOOL_DIR}/apply_overlay.sh" "${OV_GCC}" "${TOOL_DIR}/patches/gcc-frame-point-comparator.patch" >/dev/null
"${TOOL_DIR}/apply_overlay.sh" "${OV_17}" "${TOOL_DIR}/patches/gcc-frame-point-comparator.patch" \
  "${TOOL_DIR}/patches/cxx17-constexpr-uninit.patch" "${TOOL_DIR}/patches/cxx17-set-contains.patch" \
  "${TOOL_DIR}/patches/cxx17-erase-if.patch" >/dev/null

# config = compiler:std:variant, variant in {noshim, shim, shim+ovgcc, shim+ov17}
CONFIGS=(
  gcc:c++20:noshim gcc:c++20:shim gcc:c++17:shim gcc:c++20:shim+ovgcc gcc:c++17:shim+ov17
  clang:c++20:shim clang:c++17:shim
)

compile_one() {
  local rel="$1" cfg="$2"
  local cc std variant
  IFS=: read -r cc std variant <<< "${cfg}"
  local tag="${rel//\//__}.${cc}.${std}.${variant}"
  local inc=() src="${rel}" ov=""
  [[ "${variant}" == shim* ]] && inc+=(-I"${TOOL_DIR}/shim")
  [[ "${variant}" == *ovgcc ]] && ov="${OV_GCC}"
  [[ "${variant}" == *ov17 ]] && ov="${OV_17}"
  if [[ -n "${ov}" ]]; then
    inc+=(-I"${ov}")
    [[ -f "${ov}/${rel}" ]] && src="${ov}/${rel}"
  fi
  inc+=(-I"${GEN_DIR}" -I"${SRC_ROOT}" -I/usr/include/eigen3
        -I"${CERES_SRC}/include" -I"${CERES_BUILD}/include"
        -I"${CERES_SRC}/internal/ceres/miniglog")
  local cmd=(g++)
  [[ "${cc}" == clang ]] && cmd=("${CLANGXX}" --gcc-install-dir=/usr/lib/gcc/x86_64-linux-gnu/11)
  local rc=0
  (cd "${SRC_ROOT}" && "${cmd[@]}" -std="${std}" -fsyntax-only -w -D_USE_MATH_DEFINES "${inc[@]}" "${src}") \
    > "${OUT}/matrix-logs/${tag}.log" 2>&1 || rc=$?
  local first
  first="$(grep -m1 -E 'error' "${OUT}/matrix-logs/${tag}.log" | sed "s|${SRC_ROOT}/||g; s|${OUT}/||g; s|\t| |g" || true)"
  printf '%s\t%s\t%s\t%s\n' "${rel}" "${cfg}" "${rc}" "${first}"
}
export -f compile_one
export OUT SRC_ROOT TOOL_DIR GEN_DIR CERES_SRC CERES_BUILD CLANGXX OV_GCC OV_17

{
  printf 'file\tconfig\texit\tfirst_error\n'
  for f in "${FILES[@]}"; do
    [[ -f "${SRC_ROOT}/${f}" ]] || printf '%s\t-\tMISSING\t\n' "${f}"
  done
  for f in "${FILES[@]}"; do
    [[ -f "${SRC_ROOT}/${f}" ]] || continue
    for c in "${CONFIGS[@]}"; do printf '%s %s\n' "${f}" "${c}"; done
  done | xargs -P "${JOBS}" -n 2 bash -c 'compile_one "$0" "$1"' | sort
} > "${OUT}/compile-matrix.tsv"

echo "wrote ${OUT}/compile-matrix.tsv"
awk -F'\t' 'NR>1 && $3!="MISSING" {n[$2]++; if ($3=="0") ok[$2]++} END {for (k in n) printf "%s ok=%d/%d\n", k, ok[k], n[k]}' \
  "${OUT}/compile-matrix.tsv" | sort
