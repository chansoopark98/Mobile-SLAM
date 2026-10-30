#!/usr/bin/env bash
# Divergence lane: parity_report.py (main checkout evaluator, thresholds-v1.json unchanged) over a run set.
#   parity_for.sh <runs root> <label> <out dir>
# e.g. parity_for.sh build/slam-refact-20261015/engine-runs wp5b-r2 build/slam-refact-20261016/divergence/parity/control-wp5b-r2
set -euo pipefail
cd "$(dirname "$0")/../../../.."
R=$1 L=$2 O=$3
[ ! -e "$O" ] || { echo "refusing to overwrite $O" >&2; exit 2; }
mkdir -p "$O"
inj=$L-inj
[ -d "$R/flip6-s2/$inj" ] || inj=${L}-inj
python3 -I slam_refact/tools/harness/eval/parity_report.py \
  --run tum-room1-full=$R/tum-room1-full/$L --run tum-room1-lowtex=$R/tum-room1-lowtex/$L \
  --run flip6-s2=$R/flip6-s2/$L --run flip6-n1=$R/flip6-n1/$L --run flip6-s3-lowtex=$R/flip6-s3-lowtex/$L \
  --injected flip6-s2=$R/flip6-s2/$inj --injected tum-room1-full=$R/tum-room1-full/$inj \
  --absolute flip6-s2-absolute=$R/flip6-s2-absolute/$L --absolute tum-room1-full-absolute=$R/tum-room1-full-absolute/$L \
  --module-results build/slam-refact-20261015/wp5b/module-results.json \
  --out-json "$O/parity.json" --out-md "$O/parity.md"
python3 -I slam_refact/tools/harness/eval/us011_table.py \
  --run tum-room1-full=$R/tum-room1-full/$L --run tum-room1-lowtex=$R/tum-room1-lowtex/$L \
  --run flip6-s2=$R/flip6-s2/$L --run flip6-n1=$R/flip6-n1/$L --run flip6-s3-lowtex=$R/flip6-s3-lowtex/$L \
  --out-json "$O/us011.json"
echo "parity_for.sh $*" >> build/slam-refact-20261016/divergence/commands.txt
