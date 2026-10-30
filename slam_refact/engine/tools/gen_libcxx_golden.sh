#!/usr/bin/env bash
# Generate or check tests/data/compat/libcxx-golden.json (architecture.md §5.2-§5.3, §7.2).
#
# Builds tests/compat/gen_libcxx_golden.cc with em++ (emsdk 5.0.0, -std=c++20 like the original
# reference runtime toolchain) and runs it with node:
#   --impl=std     emsdk libc++ results          -> the golden file (--out)
#   --impl=compat  slam_refact::libcxx under em++ -> must equal the golden (--check), except the
#                  sort/ cases, which follow libc++ 14 (gen_libcxx14_sort_golden.sh)
# The native (g++/libstdc++) comparison is the GTest sr_tests_compat.
#
# Usage: gen_libcxx_golden.sh --out <json> | --check <json>   [--work <dir>]
set -euo pipefail
ENGINE="$(cd "$(dirname "$0")/.." && pwd)"
MODE=""; TARGET=""; WORK=""
while (( $# )); do
  case "$1" in
    --out) MODE=out; TARGET="$2"; shift 2 ;;
    --check) MODE=check; TARGET="$2"; shift 2 ;;
    --work) WORK="$2"; shift 2 ;;
    *) echo "usage: $0 --out <json> | --check <json> [--work <dir>]" >&2; exit 2 ;;
  esac
done
[[ -n "${MODE}" ]] || { echo "usage: $0 --out <json> | --check <json> [--work <dir>]" >&2; exit 2; }
WORK="${WORK:-$(mktemp -d)}"
mkdir -p "${WORK}"
# shellcheck disable=SC1091
source "${HOME}/emsdk/emsdk_env.sh" >/dev/null 2>&1
EMVER="$(cat "${EMSDK}/upstream/emscripten/emscripten-version.txt" | tr -d '"')"
if [[ "${EMVER}" != "5.0.0" ]]; then
  echo "emsdk ${EMVER} found; the golden is defined for emsdk 5.0.0 (architecture.md D-17)" >&2
  exit 1
fi
em++ -std=c++20 -O2 -fno-exceptions -fno-threadsafe-statics -ffp-contract=off -fno-fast-math \
  -Wall -Wextra -Werror -I"${ENGINE}/third_party/libcxx_compat/include" \
  "${ENGINE}/tests/compat/gen_libcxx_golden.cc" -sENVIRONMENT=node -sSTACK_SIZE=1MB \
  -o "${WORK}/gen_libcxx_golden.js"
case "${MODE}" in
  out)
    node "${WORK}/gen_libcxx_golden.js" --impl=std > "${TARGET}"
    echo "wrote ${TARGET} ($(grep -c '"digest"' "${TARGET}") cases)"
    ;;
  check)
    node "${WORK}/gen_libcxx_golden.js" --impl=std > "${WORK}/std.json"
    node "${WORK}/gen_libcxx_golden.js" --impl=compat > "${WORK}/compat.json"
    status=0
    if ! diff <(grep '"digest"' "${TARGET}") <(grep '"digest"' "${WORK}/std.json") > "${WORK}/std.diff"; then
      echo "FAIL: em++ libc++ results differ from ${TARGET} (${WORK}/std.diff)"; status=1
    fi
    # sort/ follows libc++ 14 (tools/gen_libcxx14_sort_golden.sh checks it); compare the rest.
    if ! diff <(grep '"digest"' "${TARGET}" | grep -v '"name": "sort/') \
              <(grep '"digest"' "${WORK}/compat.json" | grep -v '"name": "sort/') > "${WORK}/compat.diff"; then
      echo "FAIL: slam_refact::libcxx under em++ differs from ${TARGET} (${WORK}/compat.diff)"; status=1
    fi
    [[ ${status} -eq 0 ]] && echo "PASS: em++ libc++ and em++ slam_refact::libcxx both equal ${TARGET} ($(grep -c '"digest"' "${TARGET}") cases)"
    exit ${status}
    ;;
esac
