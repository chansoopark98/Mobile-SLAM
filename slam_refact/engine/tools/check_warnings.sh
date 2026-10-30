#!/usr/bin/env bash
# US-007 "zero warnings" check (architecture.md §2.5, gate B-1).
#
# Reads a clean-build log (g++/em++ output, e.g. `cmake --build ... 2>&1 | tee build.log`) and
# fails if any compiler warning points into engine-owned sources. Vendored code
# (third_party/**) is outside the warning policy and compiled with -w; warnings that point into
# it, or into system/dependency headers, are reported as a count only.
#
# Usage: check_warnings.sh <build.log> [engine_root]
set -euo pipefail
LOG="${1:?usage: check_warnings.sh <build.log> [engine_root]}"
ENGINE="${2:-$(cd "$(dirname "$0")/.." && pwd)}"
ENGINE="$(cd "${ENGINE}" && pwd)"
[[ -f "${LOG}" ]] || { echo "no such log: ${LOG}" >&2; exit 2; }

# Lines look like: /abs/or/rel/path.cc:12:34: warning: message [-Wflag]
mapfile -t ALL < <(grep -E '^[^ :]+:[0-9]+(:[0-9]+)?: warning:' "${LOG}" || true)
engine=()
other=0
for line in "${ALL[@]}"; do
  path="${line%%:*}"
  case "${path}" in
    /*) abs="${path}" ;;
    *) abs="$(cd "$(dirname "${LOG}")" && realpath -m "${path}")" ;;
  esac
  if [[ "${abs}" == "${ENGINE}/"* && "${abs}" != "${ENGINE}/third_party/"* ]]; then
    engine+=("${line}")
  else
    other=$((other + 1))
  fi
done
for w in "${engine[@]}"; do echo "ENGINE WARNING: ${w}"; done
echo "check_warnings: ${#engine[@]} engine warnings, ${other} outside engine sources (vendored/system, not gated)"
[[ ${#engine[@]} -eq 0 ]]
