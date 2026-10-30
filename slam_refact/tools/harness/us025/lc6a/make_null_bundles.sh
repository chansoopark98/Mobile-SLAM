#!/usr/bin/env bash
# US-025 LC-6a: exact-input replay bundles of the original lc=0 runs (null arm), made exactly like the LC-G bundles of the lc=1 runs
# (build/slam-refact-pkgh-20261022/us025/golden/bundles/commands.txt): divergence/make_variant.py on the gate base bundle of the scenario with the
# run's recorded inputs, wall clock, queue times and id generator seed. The base bundles are the ones the registered US-026 gate pinned.
#   make_null_bundles.sh <scenario> <label>   -> build/slam-refact-final-20261025/lc6a/bundles/<scenario>/<label>
set -euo pipefail
cd "$(dirname "$0")/../../../../.."
[ $# -eq 2 ] || { echo "usage: make_null_bundles.sh <scenario> <label>" >&2; exit 2; }
s=$1 r=$2
RUN=build/slam-refact-final-20261025/lc6a/orig/$s/$r
OUT=build/slam-refact-final-20261025/lc6a/bundles/$s
BASE=build/slam-refact-full-20261019/us026/gate-inputs/bundles-base/$s/s2-r1
[ -f "$RUN/recorded-inputs.jsonl.gz" ] && [ -f "$RUN/memprobe.jsonl.gz" ] || { echo "not a --record-inputs --memprobe run: $RUN" >&2; exit 2; }
[ -f "$BASE/engine-config.tsv" ] || { echo "missing base bundle $BASE" >&2; exit 2; }
mkdir -p "$OUT"
[ ! -e "$OUT/$r" ] || { echo "skip $OUT/$r (exists)"; exit 0; }
cmd=(python3 -I slam_refact/tools/harness/divergence/make_variant.py --base "$BASE" --out "$OUT/$r" --capture "$RUN" --capture-binding --capture-wallclock --queue-times --capture-id-seed)
echo "${cmd[*]}" >> "$OUT/commands.txt"
"${cmd[@]}" > "$OUT/$r.report.json"
python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(sys.argv[1], json.dumps({k:d.get(k) for k in ('binding','bindingMonotonicConflicts','queueBinding','idGeneratorSeed')}))" "$OUT/$r.report.json"
