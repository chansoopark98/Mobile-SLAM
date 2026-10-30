#!/usr/bin/env bash
# Record original absolute-scale runs with recorded inputs (MS-RT ablation), one browser at a time, each started
# only when the 1-min load average is < 12 and no other run-reference.mjs node process runs.
# usage: record_absolute_originals.sh LOG scenario:label ...
LOG=$1; shift
R=$(cd "$(dirname "$0")/../../../.." && pwd)
cd "$R" || exit 1
for sl in "$@"; do
  sc=${sl%%:*}; lb=${sl##*:}
  [ -d "build/slam-refact-20261015/reference/runs/$sc/$lb" ] && { echo "skip $sc/$lb (exists)" >> "$LOG"; continue; }
  while :; do
    l1=$(cut -d' ' -f1 /proc/loadavg)
    busy=$(pgrep -f 'node .*run-reference\.mjs' | wc -l)
    if [ "$busy" -eq 0 ] && awk -v l="$l1" 'BEGIN{exit !(l < 12)}'; then break; fi
    sleep 20
  done
  echo "start $sc/$lb $(date -Is) load $(cat /proc/loadavg)" >> "$LOG"
  node slam_refact/tools/re/runtime/run-reference.mjs --scenario "$sc" --label "$lb" --memprobe --record-inputs \
    > "build/slam-refact-20261015/reference/runs/$sc-$lb.log" 2>&1
  echo "end $sc/$lb rc=$? $(date -Is) load $(cat /proc/loadavg)" >> "$LOG"
done
echo DONE >> "$LOG"
