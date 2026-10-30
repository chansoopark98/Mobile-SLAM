#!/usr/bin/env bash
# Lane C (US-004) runbook: fixtures -> headless reference runtime reference runs (one browser at a time) -> analysis.
# Existing run labels are skipped (run-reference.mjs refuses to overwrite).  Usage: bash run_all.sh [scenario...]
set -euo pipefail
cd "$(dirname "$0")/../../../.."
T=slam_refact/tools/re/runtime
R=build/slam-refact-20261015/reference
F=$R/fixtures

[ -d $F/tum-room1-portrait-full ] || python3 $T/build_tum_fixture.py --output $F/tum-room1-portrait-full
[ -d $F/tum-room1-portrait-lowtex17-20 ] || python3 $T/build_tum_fixture.py --output $F/tum-room1-portrait-lowtex17-20 \
  --frames 480 --lowtex-start 17 --lowtex-end 20

run() {  # scenario label [flags...]
  local s=$1 l=$2; shift 2
  if [ -e "$R/runs/$s/$l/meta.json" ]; then echo "skip $s/$l"; return; fi
  echo "run $s/$l $*  load=$(cut -d' ' -f1-3 /proc/loadavg)"
  node $T/run-reference.mjs --scenario "$s" --label "$l" "$@" > "$R/runs/$s-$l.log" 2>&1 || echo "  exit $? (see $R/runs/$s-$l.log)"
}
mkdir -p $R/runs
SCENARIOS=${*:-tum-room1-full tum-room1-lowtex flip6-s2 flip6-n1 flip6-s3-lowtex}
for s in $SCENARIOS; do run "$s" r1; run "$s" r2; done
if [[ " $SCENARIOS " == *" tum-room1-full "* ]]; then
  run tum-room1-full profile1 --profile --profile-interval-us 500
  run tum-room1-full wrap1 --wrap
fi

TIC=$(python3 -c "import json;print(*json.load(open('$F/tum-room1-portrait-full/manifest.json'))['t_ic'])")
for s in $SCENARIOS; do
  case $s in
    tum-room1-full) python3 $T/analyze_reference.py --scenario-dir $R/runs/$s --gt $F/tum-room1-portrait-full/groundtruth-body.csv --t-ic $TIC > /dev/null ;;
    tum-room1-lowtex) python3 $T/analyze_reference.py --scenario-dir $R/runs/$s --gt $F/tum-room1-portrait-lowtex17-20/groundtruth-body.csv --t-ic $TIC > /dev/null ;;
    *) python3 $T/analyze_reference.py --scenario-dir $R/runs/$s --t-ic 0 0 -0.02 > /dev/null ;;
  esac
  echo "analysis: $R/runs/$s/analysis.json"
done
[ -e $R/runs/tum-room1-full/profile1/profile.cpuprofile ] && python3 $T/profile_table.py --run $R/runs/tum-room1-full/profile1 > /dev/null
[ -e $R/runs/tum-room1-full/wrap1/wasm-wrap.json ] && python3 $T/wrap_summary.py --run $R/runs/tum-room1-full/wrap1 > /dev/null
echo done
