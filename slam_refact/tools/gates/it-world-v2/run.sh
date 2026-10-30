#!/usr/bin/env bash
# it-world-gate.v2: the registered first-run command (tools/re/iwt_oracle/it_world_gate.py registered, US-015, spec/11 section 9).
#
#   bash slam_refact/tools/gates/it-world-v2/run.sh --designation FILE --replay BIN --wasm WASM --out NEWDIR [--gate it-world-gate.v2]
#
#   --designation  the coordinator's designation file of this registration (the gate checks its sha256 against the manifest, then
#                  every pin it lists: replay, f994 wasm, make_cases.py, run_f994.mjs, compare.py, it_world_gate.py)
#   --replay       the pinned sr_iwt_replay (a held copy of the freeze build; it is NOT changed by the v2 tool patch)
#   --wasm         the pinned f994 wasm copy of the original (us015-re/oracle/xr-slam-f994.wasm)
#   --out          a NEW directory; run.log (this script's whole output and "exit=<code>"), gate/ (cases, oracle.json, engine.txt,
#                  engine.stderr [v2: the engine's stderr is kept], result.json), SHA256SUMS of every file
#   --gate         the designated gate "<gate_id>.v<N>" (default it-world-gate.v2); the runner reads that version's manifest, gate file,
#                  coordinator seed and creates that version's one-run marker (O_EXCL) after every other check
#
# One real run per registration. A refused start (HELD, exit 3) created no marker and can be repeated after the cause is fixed.
# This script never creates, edits or removes a marker, manifest, seed or designation file; the gate does that itself.
# The v1 registration is reproduced from preregistration/it-world-gate.v1.snapshot.tar, not by this script.
# Not to be run before the gate is registered, reviewed and designated: this file was only checked with `bash -n`.
set -uo pipefail
orig_args=("$@")

usage() { echo "usage: $0 --designation FILE --replay BIN --wasm WASM --out NEWDIR [--gate it-world-gate.v<N>]" >&2; exit 2; }
designation="" replay="" wasm="" out="" gate="it-world-gate.v2"
while [ $# -gt 0 ]; do
  case "$1" in
    --designation) [ $# -ge 2 ] || usage; designation="$2"; shift 2;;
    --replay) [ $# -ge 2 ] || usage; replay="$2"; shift 2;;
    --wasm) [ $# -ge 2 ] || usage; wasm="$2"; shift 2;;
    --out) [ $# -ge 2 ] || usage; out="$2"; shift 2;;
    --gate) [ $# -ge 2 ] || usage; gate="$2"; shift 2;;
    *) usage;;
  esac
done
[ -n "$designation" ] && [ -n "$replay" ] && [ -n "$wasm" ] && [ -n "$out" ] || usage
for f in "$designation" "$replay" "$wasm"; do
  [ -f "$f" ] || { echo "precondition: $f is not a file" >&2; exit 2; }
done
case "$gate" in
  it-world-gate.v[1-9]|it-world-gate.v[1-9][0-9]*) ;;
  *) echo "precondition: --gate must be it-world-gate.v<N>" >&2; exit 2;;
esac
command -v node >/dev/null || { echo "precondition: node is not on PATH" >&2; exit 2; }
[ ! -e "$out" ] || { echo "precondition: $out exists (a new directory is required)" >&2; exit 2; }
mkdir -p "$(dirname "$out")" && mkdir "$out" || { echo "precondition: cannot create $out" >&2; exit 2; }
out="$(cd "$out" && pwd)"
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
tool="$here/../../re/iwt_oracle/it_world_gate.py"
log="$out/run.log"

{
  echo "# it-world run $(date -Is) gate=$gate"
  printf 'command: bash %q' "$0"; printf ' %q' "${orig_args[@]}"; echo
  echo "cwd: $PWD host: $(hostname) loadavg: $(cut -d' ' -f1-3 /proc/loadavg)"
  echo "designation: $(sha256sum "$designation")"
  echo "replay: $(sha256sum "$replay")"
  echo "wasm: $(sha256sum "$wasm")"
  echo "runner: $(sha256sum "$tool")"
  echo "node: $(command -v node) $(node --version) $(sha256sum "$(command -v node)" | cut -d' ' -f1)"
  echo "python3: $(command -v python3) $(python3 -I -c 'import sys; print(sys.version.split()[0])') $(sha256sum "$(command -v python3)" | cut -d' ' -f1)"
  python3 -I "$tool" registered --out "$out/gate" --designation "$designation" --gate "$gate" --replay "$replay" --wasm "$wasm"
  rc=$?
  echo "exit=$rc"
  echo "# done $(date -Is)"
  exit "$rc"
} 2>&1 | tee -a "$log"
rc=${PIPESTATUS[0]}

(cd "$out" && find . -type f ! -name SHA256SUMS -printf '%P\0' | sort -z | xargs -0 sha256sum > SHA256SUMS)
exit "$rc"
