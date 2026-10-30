#!/usr/bin/env bash
# orient-us023-gate.v3: the registered first-run command (tests/engine/orient_gate_test.cc, test OrientGateHeld.Us023).
#
#   bash slam_refact/tools/gates/orient-us023-v3/run.sh --binary HELD_BIN --designation-sha256 SHA256 --out NEWDIR
#        [--gate orient-us023-gate.v3] [--prereg DIR] [--rec DIR] [--precheck]
#
#   --binary              the held copy of sr_tests_orient_gate_held (preregistration/orient-held-v3/); the gate pins its own
#                         executable (manifest held.pins), so any other binary is refused (HELD) before anything is compared
#   --designation-sha256  the sha256 of the coordinator's designation file: exported as SLAM_REFACT_ORIENT_REVIEWER
#   --out                 a NEW directory; run.log (this script's whole output, the gate's stdout/stderr and "exit=<code>") and SHA256SUMS
#   --gate                the designated gate "<gate_id>.v<N>" (SLAM_REFACT_ORIENT_GATE); default is the gate this directory is named for
#   --prereg / --rec      optional replacements of the preregistration directory / the recording root compiled into the binary
#   --precheck            only the side-effect-free check of the shared device (node sr_held.mjs check: no marker, no engine answer);
#                         nothing else is run and --out is not needed
#
# Exactly one real run per registration: the gate creates the one-run marker (O_EXCL) after every check that can refuse the run, so a
# refused start (HELD) can be repeated after the cause is fixed, and a start that created the marker cannot (protocol 12.6, 13.1).
# This script never creates, edits or removes a marker, manifest or designation file.
# Not to be run before the gate is registered, reviewed and designated: this file was only checked with `bash -n`.
set -uo pipefail
orig_args=("$@")

gate="orient-us023-gate.v3"
filter="OrientGateHeld.Us023"
usage() { echo "usage: $0 --binary HELD_BIN --designation-sha256 SHA256 --out NEWDIR [--gate ID.vN] [--prereg DIR] [--rec DIR] | --precheck ..." >&2; exit 2; }

binary="" sha="" out="" prereg="" rec="" precheck=0
while [ $# -gt 0 ]; do
  case "$1" in
    --binary) [ $# -ge 2 ] || usage; binary="$2"; shift 2;;
    --designation-sha256) [ $# -ge 2 ] || usage; sha="$2"; shift 2;;
    --out) [ $# -ge 2 ] || usage; out="$2"; shift 2;;
    --gate) [ $# -ge 2 ] || usage; gate="$2"; shift 2;;
    --prereg) [ $# -ge 2 ] || usage; prereg="$2"; shift 2;;
    --rec) [ $# -ge 2 ] || usage; rec="$2"; shift 2;;
    --precheck) precheck=1; shift;;
    *) usage;;
  esac
done
[ -n "$binary" ] && [ -n "$sha" ] || usage
[ -f "$binary" ] || { echo "precondition: binary $binary is not a file" >&2; exit 2; }
case "$sha" in *[!0-9a-f]*|"") echo "precondition: --designation-sha256 is not lower-case hex" >&2; exit 2;; esac
[ "${#sha}" -eq 64 ] || { echo "precondition: --designation-sha256 is not 64 hex digits" >&2; exit 2; }
case "$gate" in
  *.v[1-9]|*.v[1-9][0-9]*) ;;
  *) echo "precondition: --gate must be <gate_id>.v<N>" >&2; exit 2;;
esac

if [ "$precheck" -eq 1 ]; then
  [ -n "$prereg" ] || { echo "precondition: --precheck needs --prereg DIR" >&2; exit 2; }
  here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  version="${gate##*.v}"
  SLAM_REFACT_ORIENT_REVIEWER="$sha" node "$here/../../prereg/sr_held.mjs" check --prereg "$prereg" --gate "${gate%.v*}" \
    --version "$version" --env-var SLAM_REFACT_ORIENT_REVIEWER --pin "$binary"
  exit $?
fi

[ -n "$out" ] || usage
[ ! -e "$out" ] || { echo "precondition: $out exists (a new directory is required)" >&2; exit 2; }
mkdir -p "$(dirname "$out")" && mkdir "$out" || { echo "precondition: cannot create $out" >&2; exit 2; }
out="$(cd "$out" && pwd)"
log="$out/run.log"

env_args=("SLAM_REFACT_ORIENT_REVIEWER=$sha" "SLAM_REFACT_ORIENT_GATE=$gate")
[ -z "$prereg" ] || env_args+=("SLAM_REFACT_ORIENT_PREREG_DIR=$prereg")
[ -z "$rec" ] || env_args+=("SLAM_REFACT_ORIENT_REC_DIR=$rec")

{
  echo "# RUN ${filter} $(date -Is) gate=$gate reviewer=$sha"
  printf 'command: bash %q' "$0"; printf ' %q' "${orig_args[@]}"; echo
  echo "binary: $(sha256sum "$binary")"
  echo "cwd: $PWD host: $(hostname) loadavg: $(cut -d' ' -f1-3 /proc/loadavg)"
  env "${env_args[@]}" "$binary" "--gtest_filter=${filter}"
  rc=$?
  echo "exit=$rc"
  echo "# done $(date -Is)"
  exit "$rc"
} 2>&1 | tee -a "$log"
rc=${PIPESTATUS[0]}

(cd "$out" && sha256sum run.log > SHA256SUMS)
exit "$rc"
