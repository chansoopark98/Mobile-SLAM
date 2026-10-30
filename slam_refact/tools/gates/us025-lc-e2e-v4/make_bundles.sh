#!/usr/bin/env bash
# us025-lc-e2e-gate.v4 copy of slam_refact/tools/harness/us026/make_bundles.sh (that file is pinned by other gates and is not modified). The ONLY change: divergence/make_variant_v2.py instead of make_variant.py (fail-closed
# monotonic input times: a violation writes no bundle and exits 21; options and output are otherwise the same).
# US-026 exact-input bundles (design §2.5, §3.2) from a run-reference.mjs --record-inputs --memprobe --memprobe-detail
# run under build/slam-refact-full-20261019/us026/runs/.
#
#   make_bundles.sh <scenario> <run label> [<variant>=<params> ...]
#
# 1. <out>/<scenario>/<run>: divergence/make_variant.py with the flags of divergence/build_exact_bundles.sh
#    (--capture-binding --capture-wallclock --queue-times, + --queue-linacc for scale 'absolute' as the absolute
#    precedent build/slam-refact-20261016/ablation/abs) plus --capture-id-seed (seed read from the run's memprobe
#    mtTrack.idRng). build_exact_bundles.sh itself is not called: its run and output paths are fixed to
#    build/slam-refact-2026101{5,6}. No pyramid injection (pyramid_model swiftshader-approx CPU model, the flip6-n1
#    precedent of docs/reports/parity.md:36).
# 2. <out>/<scenario>/<run>-<variant> for every <variant>=<params>: add_params.py (one engine-config.tsv params line).
#
# Base bundle: build/slam-refact-20261015/harness/bundles/<scenario>/s2-r1 when it exists (prepare_inputs.py, no K
# override); otherwise <base-root>/<scenario>/us026-base is made once with prepare_inputs.py from the run's
# engine-input-sensors.jsonl.gz (+ the scenario's derived-linear-acceleration.tsv.gz for scale 'absolute').
# Existing directories are never overwritten. Commands are appended to <out>/commands.txt.
set -euo pipefail
cd "$(dirname "$0")/../../../.."
[ $# -ge 2 ] || { echo "usage: make_bundles.sh <scenario> <run label> [<variant>=<params> ...]" >&2; exit 2; }
s=$1 r=$2; shift 2
# Gate runs (gate_e2e.mjs) override the three roots and name the pinned base bundle (US026_BASE: used as is, never built).
RUNS=${US026_RUNS:-build/slam-refact-full-20261019/us026/runs}
OUT=${US026_BUNDLES:-build/slam-refact-full-20261019/us026/bundles}
BASE_ROOT=${US026_BASE_ROOT:-build/slam-refact-full-20261019/us026/bundles-base}
run=$RUNS/$s/$r
[ -f "$run/recorded-inputs.jsonl.gz" ] && [ -f "$run/memprobe.jsonl.gz" ] || { echo "not a --record-inputs --memprobe run: $run" >&2; exit 2; }
mkdir -p "$OUT/$s"
log() { echo "$*" >> "$OUT/commands.txt"; }
abs=$(python3 -c "import json,sys; print(1 if json.load(open('slam_refact/tools/re/runtime/scenarios.json'))[sys.argv[1]].get('scale') == 'absolute' else 0)" "$s")
base=${US026_BASE:-build/slam-refact-20261015/harness/bundles/$s/s2-r1}
if [ -n "${US026_BASE:-}" ]; then
  [ -f "$base/engine-config.tsv" ] || { echo "US026_BASE is not a bundle: $base" >&2; exit 2; }
elif [ ! -d "$base" ]; then
  base=$BASE_ROOT/$s/us026-base
  if [ ! -d "$base" ]; then
    cmd=(python3 -I slam_refact/tools/harness/prepare_inputs.py --scenario "$s" --label us026-base --out-root "$BASE_ROOT"
         --sensors "$RUNS/$s/engine-input-sensors.jsonl.gz")
    [ "$abs" = 0 ] || cmd+=(--linear-accel "build/slam-refact-20261015/reference/runs/$s/derived-linear-acceleration.tsv.gz")
    log "${cmd[*]}"; "${cmd[@]}"
  fi
fi
exact=$OUT/$s/$r
if [ ! -e "$exact" ]; then
  cmd=(python3 -I slam_refact/tools/harness/divergence/make_variant_v2.py --base "$base" --out "$exact" --capture "$run"
       --capture-binding --capture-wallclock --queue-times --capture-id-seed)
  [ "$abs" = 0 ] || cmd+=(--queue-linacc)
  log "${cmd[*]}"; "${cmd[@]}" > "$exact.report.json"
  python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(sys.argv[1], json.dumps({k:d.get(k) for k in ('binding','bindingMonotonicConflicts','queueBinding','idGeneratorSeed')}))" "$exact.report.json"
else
  echo "skip $exact (exists)"
fi
for vp in "$@"; do
  v=${vp%%=*} p=${vp#*=}
  out=$OUT/$s/$r-$v
  [ ! -e "$out" ] || { echo "skip $out (exists)"; continue; }
  cmd=(python3 -I slam_refact/tools/harness/us026/add_params.py --base "$exact" --out "$out" --params "$p")
  log "${cmd[*]}"; "${cmd[@]}"
done
