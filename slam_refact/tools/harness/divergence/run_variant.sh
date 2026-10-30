#!/usr/bin/env bash
# Divergence lane: run the diagnostic replay binary on a bundle with the FFT counter log.
#   run_variant.sh <bundle dir> <new out dir> [fft full-dump call range lo-hi]
# Paths are relative to the main repo root. Output dirs are never overwritten.
set -euo pipefail
cd "$(dirname "$0")/../../../.."
bundle=$1 out=$2 full=${3:-}
bin=${SLAM_REFACT_REPLAY:-build/slam-refact-native-div/harness/slam_refact_replay}
[ ! -e "$out" ] || { echo "refusing to overwrite $out" >&2; exit 2; }
mkdir -p "$(dirname "$out")"
export SR_DIV_FFT_LOG="$out-fft.jsonl" SR_DIV_MT_LOG="$out-mt.jsonl" SR_DIV_EPI_LOG="$out-epi.jsonl"
[ -z "$full" ] || export SR_DIV_FFT_FULL="$full"
"$bin" --bundle "$bundle" --out "$out" > "$out.log" 2>&1
{
  echo "# $(date -u +%Y-%m-%dT%H:%M:%SZ) cwd=$PWD bin=$bin sha256=$(sha256sum "$bin" | cut -c1-16)"
  echo "SR_DIV_FFT_LOG=$SR_DIV_FFT_LOG SR_DIV_FFT_FULL=$full $bin --bundle $bundle --out $out"
} >> build/slam-refact-20261016/divergence/commands.txt
tail -1 "$out.log"
