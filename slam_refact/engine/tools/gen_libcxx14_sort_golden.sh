#!/usr/bin/env bash
# Generate or check tests/data/compat/libcxx14-sort-golden.json: the "sort/" golden cases evaluated
# by libc++ 14 std::sort, the algorithm of the original xr-slam.wasm (third_party/libcxx_compat/
# include/libcxx_compat/__algorithm/sort_llvm14.h has the evidence). All other golden families stay
# on the emsdk 5.0.0 golden (gen_libcxx_golden.sh).
#
# Builds tests/compat/gen_libcxx_golden.cc natively with g++ against the libc++ 14 headers and
# library of libc++-14-dev (/usr/lib/llvm-14) and keeps the sort/ cases:
#   --out <json>    write the golden (--impl=std)
#   --check <json>  libc++ 14 std:: and slam_refact::libcxx (same build) must both equal <json>
set -euo pipefail
ENGINE="$(cd "$(dirname "$0")/.." && pwd)"
LLVM14="${LLVM14:-/usr/lib/llvm-14}"
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
[[ -f "${LLVM14}/include/c++/v1/__algorithm/sort.h" ]] || { echo "libc++ 14 headers not found under ${LLVM14} (BLOCKED)" >&2; exit 1; }
WORK="${WORK:-$(mktemp -d)}"
mkdir -p "${WORK}"
g++ -std=c++20 -O2 -fno-exceptions -fno-threadsafe-statics -ffp-contract=off -fno-fast-math \
  -nostdinc++ -isystem "${LLVM14}/include/c++/v1" -I"${ENGINE}/third_party/libcxx_compat/include" \
  "${ENGINE}/tests/compat/gen_libcxx_golden.cc" -o "${WORK}/gen_libcxx14" \
  "${LLVM14}/lib/libc++.a" "${LLVM14}/lib/libc++abi.so.1" -lpthread
# Keep the JSON header lines and the sort/ cases (the last kept case loses its trailing comma).
filter() {
  python3 -I -c '
import json, sys
d = json.load(sys.stdin)
d["cases"] = [c for c in d["cases"] if c["name"].startswith("sort/")]
d["case_count"] = len(d["cases"])
d["toolchain"] = d["toolchain"] + " + libc++ 14 (libc++-14-dev)"
print("{")
print("  \"format\": \"%s\"," % d["format"])
print("  \"impl\": \"%s\"," % d["impl"])
print("  \"toolchain\": \"%s\"," % d["toolchain"])
print("  \"cplusplus\": %d," % d["cplusplus"])
print("  \"sizeof_ptrdiff_t\": %d," % d["sizeof_ptrdiff_t"])
print("  \"case_count\": %d," % d["case_count"])
print("  \"cases\": [")
for i, c in enumerate(d["cases"]):
    print("    {\"name\": \"%s\", \"digest\": \"%s\"}%s" % (c["name"], c["digest"], "," if i + 1 < len(d["cases"]) else ""))
print("  ]")
print("}")'
}
case "${MODE}" in
  out)
    "${WORK}/gen_libcxx14" --impl=std | filter > "${TARGET}"
    echo "wrote ${TARGET} ($(grep -c '"digest"' "${TARGET}") cases)"
    ;;
  check)
    "${WORK}/gen_libcxx14" --impl=std | filter > "${WORK}/std.json"
    "${WORK}/gen_libcxx14" --impl=compat | filter > "${WORK}/compat.json"
    status=0
    diff <(grep '"digest"' "${TARGET}") <(grep '"digest"' "${WORK}/std.json") > "${WORK}/std.diff" ||
      { echo "FAIL: libc++ 14 std::sort differs from ${TARGET} (${WORK}/std.diff)"; status=1; }
    diff <(grep '"digest"' "${TARGET}") <(grep '"digest"' "${WORK}/compat.json") > "${WORK}/compat.diff" ||
      { echo "FAIL: slam_refact::libcxx::sort differs from ${TARGET} (${WORK}/compat.diff)"; status=1; }
    [[ ${status} -eq 0 ]] && echo "PASS: libc++ 14 std::sort and slam_refact::libcxx::sort both equal ${TARGET} ($(grep -c '"digest"' "${TARGET}") cases)"
    exit ${status}
    ;;
esac
