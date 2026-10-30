#!/usr/bin/env bash
# US-025 LC-6b: exact-input engine bundles from one ORIGINAL run of a synthetic scene (same steps as e5/e5_pipeline.py stage 2, us026/make_bundles.sh).
#   lc6b_bundle.sh <scenario> <run label>
# Writes <D>/bundles/<scenario>/<label>-base (prepare_inputs), <label> (exact-input: --capture-binding --capture-wallclock --queue-times --capture-id-seed)
# and <label>-on (= <label> + params MapBuilder.loopCloseEnabled=b:1). Existing directories are never overwritten.
set -euo pipefail
cd "$(dirname "$0")/../../../../.."
[ $# -eq 2 ] || { echo "usage: lc6b_bundle.sh <scenario> <run label>" >&2; exit 2; }
s=$1 r=$2
D=build/slam-refact-pkgh-20261022/us025/lc6b
runs=$D/runs/$s/$r
H=slam_refact/tools/harness
mkdir -p $D/bundles/$s $D/logs
python3 -I $H/e5/prepare_bundle.py --scenarios-file $D/scenarios.json --runs-dir $D/runs --bundles-dir $D/bundles --scenario $s --label $r-base \
  --sensors $D/runs/$s/engine-input-sensors.jsonl.gz --drop-mask-from $runs/trace.jsonl > $D/logs/bundle-base-$s-$r.log 2>&1
python3 -I $H/divergence/make_variant.py --base $D/bundles/$s/$r-base --out $D/bundles/$s/$r --capture $runs --capture-binding --capture-wallclock --queue-times --capture-id-seed \
  > $D/logs/bundle-exact-$s-$r.log 2>&1
python3 -I $H/us026/add_params.py --base $D/bundles/$s/$r --out $D/bundles/$s/$r-on --params 'MapBuilder.loopCloseEnabled=b:1' > $D/logs/bundle-on-$s-$r.log 2>&1
echo ok $D/bundles/$s/$r $D/bundles/$s/$r-on
