#!/usr/bin/env bash
# Accuracy-development replay: one scenario, every frame processed (ideal mode), extra prepare_inputs.py
# arguments passed through (--intrinsics, --params, --linear-accel, ...). Never overwrites a label.
#
#   bash slam_refact/tools/harness/run_acc.sh <scenario> <label> [prepare_inputs.py args...]
#
# Binary: $SLAM_REFACT_REPLAY (default build/slam-refact-native-acc/harness/slam_refact_replay).
# Engine options that are not bundle keys come from the environment of the replay process
# (SLAM_REFACT_ACCURACY=..., see engine/include/slam_refact/engine_config.h AccuracyConfig).
set -euo pipefail
cd "$(dirname "$0")/../../.."
[ $# -ge 2 ] || { echo "usage: $0 <scenario> <label> [prepare args...]" >&2; exit 2; }
scenario=$1 label=$2
shift 2
[[ $label =~ ^[a-z0-9-]+$ ]] || { echo "label must match [a-z0-9-]+" >&2; exit 2; }
E=build/slam-refact-20261015
bin=${SLAM_REFACT_REPLAY:-build/slam-refact-native-acc/harness/slam_refact_replay}
[ -x "$bin" ] || { echo "BLOCKED: $bin not built" >&2; exit 3; }
bundle=$E/harness/bundles/$scenario/$label
out=$E/engine-runs/$scenario/$label
[ ! -e "$out" ] || { echo "refusing to overwrite $out" >&2; exit 2; }
prep=(python3 -I slam_refact/tools/harness/prepare_inputs.py --scenario "$scenario" --label "$label" "$@")
"${prep[@]}" > /dev/null
mkdir -p "$(dirname "$out")"
"$bin" --bundle "$bundle" --out "$out" > /dev/null
{
  echo "# $(date -u +%Y-%m-%dT%H:%M:%SZ) cwd=$PWD"
  echo "SLAM_REFACT_ACCURACY=${SLAM_REFACT_ACCURACY:-} SLAM_REFACT_SURFACES=${SLAM_REFACT_SURFACES:-}"
  printf '%q ' bash slam_refact/tools/harness/run_acc.sh "$scenario" "$label" "$@"; echo
  printf '%q ' "${prep[@]}"; echo
  printf '%q ' "$bin" --bundle "$bundle" --out "$out"; echo
  echo "binary_sha256 $(sha256sum "$bin" | cut -d' ' -f1)"
} > "$out/commands.txt"
echo "$out"
