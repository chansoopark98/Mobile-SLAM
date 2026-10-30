#!/usr/bin/env bash
# US-025 LC-6a: engine replays of the exact-input bundles with slam_refact_replay_lc6a (dev/report, not a gate run).
#   run_engine_arms.sh <jobs 1..4> <label>=<bundle dir> [<label>=<bundle dir> ...]
# Each replay goes to build/slam-refact-final-20261025/lc6a/engine/<label> (never overwritten) with a log; a ".done" line with the exit status is appended
# to engine/status.tsv. At most 4 replays run at once (CLAUDE.md job limit); the engine is single threaded.
set -u
cd "$(dirname "$0")/../../../../.."
BIN=build/slam-refact-final-20261025/lc6a/bin/slam_refact_replay_lc6a
OUT=build/slam-refact-final-20261025/lc6a/engine
jobs=$1; shift
case $jobs in 1|2|3|4) ;; *) echo "jobs must be 1..4" >&2; exit 2;; esac
[ -x "$BIN" ] || { echo "missing $BIN (build_lc6a.sh)" >&2; exit 2; }
mkdir -p "$OUT"
one() {
  spec=$1; label=${spec%%=*}; bundle=${spec#*=}
  [ -e "$OUT/$label" ] && { echo "skip $label (exists)"; return 0; }
  start=$(date +%s)
  "$BIN" --bundle "$bundle" --out "$OUT/$label" > "$OUT/$label.log" 2>&1
  rc=$?
  printf '%s\t%s\trc=%s\t%ss\tload=%s\n' "$(date +%FT%T)" "$label" "$rc" "$(( $(date +%s) - start ))" "$(cut -d' ' -f1 /proc/loadavg)" >> "$OUT/status.tsv"
}
export -f one; export BIN OUT
printf '%s\n' "$@" | xargs -P "$jobs" -I{} bash -c 'one "$@"' _ {}
