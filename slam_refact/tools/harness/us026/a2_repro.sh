#!/usr/bin/env bash
# US-026 root-cause A-2 (REPORT tier, user-approved 2026-10-22): one extra original BA-on run of flip6-s2
# (same options and poke as us026-ba-e2e-gate.v1, NOT through the registered gate), its exact-input bundle, the
# engine BA-on replay with the maphash diagnostic, and the comparison. Output only under
# build/slam-refact-full-20261019/us026/rootcause-a2/<label>. Usage: a2_repro.sh <label>   (e.g. r1)
set -euo pipefail
cd "$(dirname "$0")/../../../.."
L="${1:?label}"
ROOT=build/slam-refact-full-20261019/us026/rootcause-a2
RUNS=$ROOT/runs; OUT=$ROOT/$L
[ ! -e "$OUT" ] || { echo "exists: $OUT" >&2; exit 2; }
mkdir -p "$OUT" "$RUNS"
BASE=build/slam-refact-full-20261019/us026/gate-inputs/bundles-base/flip6-s2/s2-r1
MH=build/slam-refact-rootcause-20261022/native/harness/slam_refact_replay_maphash
# 1. original run inside the Chromium lock, precheck first (load wait, never kill other processes)
flock -w 7200 build/c1-chromium.lock bash -c '
  until PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0 bash slam_refact/docs/design/tools/lane-precheck.sh --json >"'"$OUT"'/precheck.json" \
        && [ "$(cut -d" " -f1 /proc/loadavg | cut -d. -f1)" -lt 12 ]; do sleep 30; done
  node slam_refact/tools/re/runtime/run-reference.mjs --scenario flip6-s2 --label a2-'"$L"' --out-root '"$RUNS"' \
    --memprobe --record-inputs --memprobe-detail 12 --poke "{\"ba\":1}"' >"$OUT/run.log" 2>&1
RUN=$RUNS/flip6-s2/a2-$L
# 2. exact-input bundle (same flags as the gate's bundles/commands.txt) and BA-on params
python3 -I slam_refact/tools/harness/divergence/make_variant.py --base "$BASE" --out "$OUT/bundle-poke" --capture "$RUN" \
  --capture-binding --capture-wallclock --queue-times --capture-id-seed >"$OUT/bundle.log" 2>&1
python3 -I slam_refact/tools/harness/us026/add_params.py --base "$OUT/bundle-poke" --out "$OUT/bundle-poke-on" \
  --params MapBuilder.bundleAdjustmentIterativeEnabled=b:1 >>"$OUT/bundle.log" 2>&1
# 3. engine BA-on replay with the map-hash diagnostic
"$MH" --bundle "$OUT/bundle-poke-on" --out "$OUT/replay-on" >"$OUT/replay.log" 2>&1
echo "A2 $L DONE" | tee "$OUT/DONE"
