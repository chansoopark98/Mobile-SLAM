#!/usr/bin/env bash
# us025-lc-e2e-gate.v1 detection power, ENGINE-level input mutants (protocol §4: input variations, no source touched): the LC-6a dev ON bundles are rebuilt from
# the OFF bundle with ONE loop-close threshold changed (WorldMaps.loopClosing* are engine settings read from the bundle's params line), replayed by the dev
# slam_refact_replay_lc6a, and compared with compare_us025.py `on`. The unmutated ON bundle is the control and must PASS. Dev data only (the LC-G runs).
#   bash mutants_params.sh <out dir (new)> [jobs 1..2]
# Mutants: D120/D40 distance threshold 80 -> 120/40, M30 matches required 75 -> 30, C10 pose points required 35 -> 10 (large); D79/M74/C34 boundary (one unit).
# Expected: the large ones FAIL where the unmutated run reached the threshold at all; boundary ones are reported either way (an undetected boundary mutant is
# a reported hole, not a detection). Runs: lc1-2 and hkd-7 (flip6-s3-lowtex), the two runs with the fewest attempts.
set -u
cd "$(dirname "$0")/../../../.."
OUT=${1:?usage: mutants_params.sh <new out dir> [jobs]}
JOBS=${2:-2}
case $JOBS in 1|2) ;; *) echo "jobs 1..2" >&2; exit 2;; esac
[ ! -e "$OUT" ] || { echo "refusing to overwrite $OUT" >&2; exit 2; }
BIN=build/slam-refact-final-20261025/lc6a/bin/slam_refact_replay_lc6a
G=build/slam-refact-pkgh-20261022/us025/golden
[ -x "$BIN" ] || { echo "missing $BIN" >&2; exit 2; }
mkdir -p "$OUT/bundles" "$OUT/replays" "$OUT/compare"
declare -A P=( [CTRL]="" [D120]="WorldMaps.loopClosingDistanceThreshold=f:120" [D40]="WorldMaps.loopClosingDistanceThreshold=f:40"
  [M30]="WorldMaps.loopClosingMatchesRequired=i:30" [C10]="WorldMaps.loopClosingPoseEstimationPointsRequired=i:10"
  [D79]="WorldMaps.loopClosingDistanceThreshold=f:79" [M74]="WorldMaps.loopClosingMatchesRequired=i:74" [C34]="WorldMaps.loopClosingPoseEstimationPointsRequired=i:34" )
jobs_file=$OUT/jobs.txt
: > "$jobs_file"
for run in lc1-2 hkd-7; do
  for m in CTRL D120 D40 M30 C10 D79 M74 C34; do
    b=$OUT/bundles/$run-$m
    params="MapBuilder.loopCloseEnabled=b:1${P[$m]:+;${P[$m]}}"
    python3 -I slam_refact/tools/harness/us026/add_params.py --base "$G/bundles/flip6-s3-lowtex/$run" --out "$b" --params "$params" > /dev/null || exit 3
    echo "$run $m" >> "$jobs_file"
  done
done
one() {
  run=$1 m=$2
  "$BIN" --bundle "$OUT/bundles/$run-$m" --out "$OUT/replays/$run-$m" > "$OUT/replays/$run-$m.log" 2>&1
  rc=$?
  python3 -B -I slam_refact/tools/harness/us025/compare_us025.py on --original "$G/runs/flip6-s3-lowtex/$run" --engine "$OUT/replays/$run-$m" --out "$OUT/compare/$run-$m.json" > "$OUT/compare/$run-$m.txt" 2>&1
  printf '%s\t%s\treplay_rc=%s\tcompare_rc=%s\t%s\n' "$run" "$m" "$rc" "$?" "$(head -c 300 "$OUT/compare/$run-$m.txt" | tr '\n' ' ')" >> "$OUT/results.tsv"
}
export -f one; export BIN OUT G
xargs -P "$JOBS" -L1 bash -c 'one "$@"' _ < "$jobs_file"
echo "done: $OUT/results.tsv"
