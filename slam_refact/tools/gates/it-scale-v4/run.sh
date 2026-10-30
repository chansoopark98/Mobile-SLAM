#!/usr/bin/env bash
# it-scale-gate.v4: the registered first-run command (gate file section 7). Same accident-guard structure as it-scale-gate.v3 (it-scale-v2/run.sh).
#
#   bash slam_refact/tools/gates/it-scale-v4/run.sh --designation-sha256 SHA256 --out NEWDIR [--gate it-scale-gate.v4] [--prereg DIR] [--check]
#
#   --designation-sha256  sha256 of the designation file (exported as SR_ITS4_DESIGNATION, the sr_held env value)
#   --out                 a NEW directory (run.log of this script, seq-a/ seq-b/, pipe-seq-*/, verdict-*.json, result.json, SHA256SUMS)
#   --check               sr_held CheckRelease only (no marker, no seed read, nothing generated); --out is not needed
#
# Steps of a real run: (0) lane-precheck (blocked -> wait, never relax or kill; exit 6 after the wait limit, nothing created),
# (1) run.mjs: sr_held heldGate -> marker -> three sequences -> original SDK x5 each -> exact-input bundles -> engine arm 1 (GATE) -> REPORT runs (arm 0, pre-fix engine, cross pairs) -> positive controls (arm 1) -> judge -> final.
# BROWSER_EXECUTABLE other than /usr/bin/google-chrome is refused (exit 2) before any marker; Chrome and its SwiftShader libraries are sha256-pinned.
# Exit codes: 0 PASS; 1 some judged item FAIL; 2 usage; 3 HELD (no marker: not a run); 4 INVALID after the marker; 5 INSENSITIVE (no PASS claim); 6 lane-precheck blocked.
# This script never creates, edits or removes a marker, manifest, designation or seed file. An accident guard, not an authentication.
set -uo pipefail
orig_args=("$@")
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo="$(cd "$here/../../../.." && pwd)"
gate="it-scale-gate.v4"; prereg="$repo/build/slam-refact-full-20261019/preregistration"; sha="" out="" check=0
usage() { echo "usage: $0 --designation-sha256 SHA256 --out NEWDIR [--gate ID.vN] [--prereg DIR] [--check]" >&2; exit 2; }
while [ $# -gt 0 ]; do
  case "$1" in
    --designation-sha256) [ $# -ge 2 ] || usage; sha="$2"; shift 2;;
    --out) [ $# -ge 2 ] || usage; out="$2"; shift 2;;
    --gate) [ $# -ge 2 ] || usage; gate="$2"; shift 2;;
    --prereg) [ $# -ge 2 ] || usage; prereg="$2"; shift 2;;
    --check) check=1; shift;;
    *) usage;;
  esac
done
case "$sha" in *[!0-9a-f]*|"") echo "precondition: --designation-sha256 is not lower-case hex" >&2; exit 2;; esac
[ "${#sha}" -eq 64 ] || { echo "precondition: --designation-sha256 is not 64 hex digits" >&2; exit 2; }
case "$gate" in *.v[1-9]|*.v[1-9][0-9]*) ;; *) echo "precondition: --gate must be <gate_id>.v<N>" >&2; exit 2;; esac
[ -d "$prereg" ] || { echo "precondition: --prereg $prereg is not a directory" >&2; exit 2; }
prereg="$(cd "$prereg" && pwd)"
engine="$prereg/it-scale-gate.v4-held-v1/sr_e5_replay"
prefix="$prereg/it-scale-gate.v4-held-v1/sr_e5_replay_prefix"     # REPORT-only pre-fix engine control (git 19fd011, before US015-CC-1)
for f in "$engine" "$prefix"; do [ -f "$f" ] || { echo "precondition: $f is not a file" >&2; exit 2; }; done
export SR_ITS4_DESIGNATION="$sha"
if [ "$check" -eq 1 ]; then
  exec node "$here/run.mjs" --prereg "$prereg" --gate "$gate" --env-var SR_ITS4_DESIGNATION --engine "$engine" --prefix-engine "$prefix" --check
fi
[ -n "$out" ] || usage
[ ! -e "$out" ] || { echo "precondition: $out exists (a new directory is required)" >&2; exit 2; }
mkdir -p "$(dirname "$out")"
# (0) lane-precheck: a foreign build or headless Chromium blocks; wait (poll every 30 s, at most 6 h), never relax the thresholds
waited=0
until PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0 bash "$repo/slam_refact/docs/design/tools/lane-precheck.sh" >/dev/null 2>&1 \
      && [ "$(cut -d' ' -f1 /proc/loadavg | cut -d. -f1)" -lt 12 ]; do
  [ "$waited" -lt 21600 ] || { echo "lane-precheck still blocked after 6 h (nothing created)" >&2; exit 6; }
  sleep 30; waited=$((waited + 30))
done
mkdir "$out" || exit 2
out="$(cd "$out" && pwd)"
{
  echo "# it-scale-gate.v4 run $(date -Is) gate=$gate"
  printf 'command: bash %q' "$0"; printf ' %q' "${orig_args[@]}"; echo
  echo "cwd: $PWD host: $(hostname) loadavg: $(cut -d' ' -f1-3 /proc/loadavg) lane-precheck waited ${waited}s"
  echo "node: $(command -v node) $(node --version); python3: $(command -v python3) $(python3 -I -c 'import sys; print(sys.version.split()[0])')"
  echo "chrome: $(/usr/bin/google-chrome --version 2>&1 | head -1); numpy/Pillow: $(python3 -I -c 'import numpy, PIL; print(numpy.__version__, PIL.__version__)' 2>&1)"
  echo "engine: $(sha256sum "$engine")"; echo "prefix engine (REPORT control): $(sha256sum "$prefix")"
  # the output directory is created by run.mjs itself (a new directory is required there): use a sibling work name
  node "$here/run.mjs" --prereg "$prereg" --gate "$gate" --env-var SR_ITS4_DESIGNATION --engine "$engine" --prefix-engine "$prefix" --out "$out/gate"
  rc=$?
  echo "exit=$rc"; echo "# done $(date -Is)"
  exit "$rc"
} 2>&1 | tee -a "$out/run.log"
rc=${PIPESTATUS[0]}
(cd "$out" && find . -type f ! -name SHA256SUMS -printf '%P\0' | sort -z | xargs -0 sha256sum > SHA256SUMS)
exit "$rc"
