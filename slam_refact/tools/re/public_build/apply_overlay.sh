#!/usr/bin/env bash
# Build an include overlay of patched public xr_reference files without touching the reference tree.
#
# Copies every file touched by the given patches from the reference tree into <overlay_dir>
# (same relative path) and applies the patches there. Put <overlay_dir> before the reference
# root on the include path (-I overlay -I root); patched .cc files are compiled from the overlay.
#
# Usage: apply_overlay.sh <overlay_dir> <patch> [<patch> ...]
set -euo pipefail
ROOT_REPO="$(cd "$(dirname "$0")/../../../.." && pwd)"
SRC_ROOT="${ROOT_REPO}/assets/references/xr_reference/official/xr_reference"
OV="${1:?overlay dir}"; shift
rm -rf "${OV}"; mkdir -p "${OV}"
for p in "$@"; do
  for rel in $(grep -E '^\+\+\+ b/' "${p}" | sed -E 's|^\+\+\+ b/([^\t ]+).*|\1|'); do
    if [[ ! -f "${OV}/${rel}" ]]; then
      mkdir -p "${OV}/$(dirname "${rel}")"
      cp "${SRC_ROOT}/${rel}" "${OV}/${rel}"
    fi
  done
  patch -s -p1 -d "${OV}" < "${p}"
  echo "applied $(basename "${p}")"
done
