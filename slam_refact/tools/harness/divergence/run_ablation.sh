#!/usr/bin/env bash
# Ablation of exact-input bundles (diagnosis only). For each <scenario>/<run>: build variants with
# ablation_variants.py (one approximation, leave-one-out, all) and replay them with BIN, at most JOBS at a time.
# usage: run_ablation.sh OUT BIN JOBS scenario/run ...
set -euo pipefail
OUT=$1; BIN=$2; JOBS=$3; shift 3
R=$(cd "$(dirname "$0")/../../../.." && pwd)
cd "$R"
SEEDB=${SEEDB:-build/slam-refact-20261016/fix/round2/bundles-seed}
GATEB=build/slam-refact-20261015/harness/bundles
VARIANTS=${VARIANTS:-"E T B S P D TB TBSPD BSPD TSPD TBPD TBSD TBSP"}
mkdir -p "$OUT"
jobs_list=()
for sr in "$@"; do
  sc=${sr%%/*}; run=${sr##*/}
  for v in $VARIANTS; do
    a=$v; [ "$v" = E ] && a=
    bd=$OUT/bundles/$sc/$run/$v
    if [ ! -d "$bd" ]; then
      mkdir -p "$(dirname "$bd")"
      python3 -I slam_refact/tools/harness/divergence/ablation_variants.py --exact "$SEEDB/$sc/$run" \
        --gate "$GATEB/$sc/s2-r1" --out "$bd" --approx "$a" > "$bd.variant.json"
      echo "python3 -I slam_refact/tools/harness/divergence/ablation_variants.py --exact $SEEDB/$sc/$run --gate $GATEB/$sc/s2-r1 --out $bd --approx '$a'" >> "$OUT/commands.txt"
    fi
    jobs_list+=("$bd|$OUT/replays/$sc/$run/$v")
  done
done
for j in "${jobs_list[@]}"; do
  bd=${j%%|*}; rd=${j##*|}
  [ -d "$rd" ] && continue
  mkdir -p "$(dirname "$rd")"
  echo "$BIN --bundle $bd --out $rd" >> "$OUT/commands.txt"
  while [ "$(jobs -rp | wc -l)" -ge "$JOBS" ]; do wait -n; done
  ( "$BIN" --bundle "$bd" --out "$rd" > "$rd.log" 2>&1 || echo "FAIL $rd" >> "$OUT/failures.txt" ) &
done
wait
echo done
