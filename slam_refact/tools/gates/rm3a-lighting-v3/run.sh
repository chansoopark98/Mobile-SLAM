#!/usr/bin/env bash
# rm3a-lighting-gate.v3: the registered first-run command (gate file section 6).
#
#   bash slam_refact/tools/gates/rm3a-lighting-v3/run.sh --designation-sha256 SHA256 --out NEWDIR [--check]
#   bash slam_refact/tools/gates/rm3a-lighting-v3/run.sh --null-mutation --out NEWDIR          (marker-free, no designation: the 7 null-run/mutation tests)
#
#   --designation-sha256  sha256 of the coordinator's designation file (exported as SLAM_REFACT_RM3A_V3_DESIGNATION_SHA256, the sr_held env value)
#   --out                 a NEW directory under build/slam-refact-rm3a-v3-20261023/runs/ (held.log, gtest-gates.log, result.json, SHA256SUMS)
#   --check               sr_held CheckRelease only (no marker, nothing runs)
#
# Exit codes: 0 PASS; 1 FAIL; 2 usage; 3 HELD (no marker: not a run); 4 INVALID after the marker.
# No original-SDK run, no browser, no network: the measurer is a native gtest on recorded inputs, so no lane-precheck / Chromium lock is needed.
# This script never creates, edits or removes a marker, manifest, designation or review file.  An accident guard, not an authentication.
set -uo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../../.." && pwd)"
snap="$repo/build/slam-refact-rm3a-v3-20261023"
prereg="$repo/build/slam-refact-full-20261019/preregistration"
bin="$snap/held/sr_tests_xr_api_evidence"
sha="" out="" check=0 nullmut=0
usage() { echo "usage: $0 --designation-sha256 SHA256 --out NEWDIR [--check] | $0 --null-mutation --out NEWDIR" >&2; exit 2; }
while [ $# -gt 0 ]; do
  case "$1" in
    --designation-sha256) [ $# -ge 2 ] || usage; sha="$2"; shift 2;;
    --out) [ $# -ge 2 ] || usage; out="$2"; shift 2;;
    --check) check=1; shift;;
    --null-mutation) nullmut=1; shift;;
    *) usage;;
  esac
done
[ -f "$bin" ] || { echo "precondition: $bin is not a file" >&2; exit 2; }
if [ "$nullmut" -eq 1 ]; then
  [ -n "$out" ] && [ -z "$sha" ] && [ "$check" -eq 0 ] || usage
  exec node "$here/run.mjs" --bin "$bin" --data "$snap/src/build" --null-mutation --out "$out"
fi
case "$sha" in *[!0-9a-f]*|"") echo "precondition: --designation-sha256 is not lower-case hex" >&2; exit 2;; esac
[ "${#sha}" -eq 64 ] || { echo "precondition: --designation-sha256 is not 64 hex digits" >&2; exit 2; }
export SLAM_REFACT_RM3A_V3_DESIGNATION_SHA256="$sha"
if [ "$check" -eq 1 ]; then
  exec node "$here/run.mjs" --prereg "$prereg" --gate rm3a-lighting-gate.v3 --env-var SLAM_REFACT_RM3A_V3_DESIGNATION_SHA256 --bin "$bin" --data "$snap/src/build" --check
fi
[ -n "$out" ] || usage
mkdir -p "$snap/runs"
node "$here/run.mjs" --prereg "$prereg" --gate rm3a-lighting-gate.v3 --env-var SLAM_REFACT_RM3A_V3_DESIGNATION_SHA256 --bin "$bin" --data "$snap/src/build" --out "$out"
