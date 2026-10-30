#!/usr/bin/env bash
# Exact-input replay bundles (thresholds-registration.md Addendum 6 item 5) from run-reference --record-inputs runs:
# recorded binding + recorded timeNanos (wall clock) + recorded IMU queue times, pyramids from the given capture dirs.
#   build_exact_bundles.sh <scenario> "<run labels>" "<pyramid capture dirs (may be empty)>"
# Writes build/slam-refact-20261016/exact-input/bundles/<scenario>/<run>/ (contract inputs) and <run>-linacc0/
# (same + the LINEAR_ACCELERATION (0,0,0) events the original received). Existing dirs are never overwritten.
set -euo pipefail
cd "$(dirname "$0")/../../../.."
s=$1 runs=$2 pyrs=${3:-}
base=build/slam-refact-20261015/harness/bundles/$s/s2-r1
outd=build/slam-refact-20261016/exact-input/bundles/$s
mkdir -p "$outd"
pyr_args=()
for p in $pyrs; do pyr_args+=(--pyramids-from "$p"); done
[ ${#pyr_args[@]} -eq 0 ] || pyr_args+=(--capture-pyramids)
for r in $runs; do
  run=build/slam-refact-20261015/reference/runs/$s/$r
  for v in "" "-linacc0"; do
    extra=(); [ -z "$v" ] || extra=(--queue-linacc)
    out=$outd/$r$v
    [ ! -e "$out" ] || { echo "skip $out (exists)"; continue; }
    cmd=(python3 -I slam_refact/tools/harness/divergence/make_variant.py --base "$base" --out "$out" --capture "$run"
         "${pyr_args[@]}" --capture-binding --capture-wallclock --queue-times "${extra[@]}")
    echo "${cmd[*]}" >> build/slam-refact-20261016/exact-input/commands.txt
    "${cmd[@]}" > "$out.report.json"
    python3 -c "import json,sys; d=json.load(open(sys.argv[1])); d.pop('argv',None); print(sys.argv[1], json.dumps({k:d.get(k) for k in ('injectedFrames','stagedWithoutPyramid','binding','bindingMonotonicConflicts','queueBinding')}))" "$out.report.json"
  done
done
