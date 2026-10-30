#!/usr/bin/env bash
# US-025 LC-6b: one ORIGINAL SDK run (run_lcgold_scene.mjs) of a synthetic scene under the precheck and the shared Chromium lock. Dev run, not a gate run.
#   bash slam_refact/tools/harness/us025/scene/lc6b-run.sh <scenario> <label> [run options...]      (scenario names: $D/scenarios.json)
set -u
cd "$(dirname "$0")/../../../../.."
[ $# -ge 2 ] || { echo "usage: lc6b-run.sh <scenario> <label> [options...]" >&2; exit 2; }
s=$1 label=$2; shift 2
D=build/slam-refact-pkgh-20261022/us025/lc6b
OUT_ROOT=$D/runs
LOG=$D/run-log.tsv
mkdir -p "$OUT_ROOT/$s"
now() { date +%Y-%m-%dT%H:%M:%S%z; }
started=$(now)
pre=$OUT_ROOT/$s/precheck-$label.txt
{ echo "started $started"; echo "loadavg $(cat /proc/loadavg)"; df -h / | tail -1; } > "$pre"
pct=$(df --output=pcent / | tail -1 | tr -dc 0-9)
[ "$pct" -le 97 ] || { echo "disk $pct% > 97%: stop" >&2; exit 4; }
deadline=$(( $(date +%s) + ${WAIT_MIN:-0} * 60 ))
while :; do
  ok=1
  PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=${MAX_BUILDS:-0} PRECHECK_MAX_DISK_PCT=97 bash slam_refact/docs/design/tools/lane-precheck.sh >> "$pre" 2>&1 || ok=0
  if [ $ok = 1 ] && [ -n "${LOAD_MAX:-}" ]; then
    awk -v l="$(cut -d' ' -f1 /proc/loadavg)" -v m="$LOAD_MAX" 'BEGIN{exit !(l < m)}' || { ok=0; echo "load >= LOAD_MAX $LOAD_MAX" >> "$pre"; }
  fi
  [ $ok = 1 ] && break
  if [ "$(date +%s)" -ge "$deadline" ]; then
    printf '%s\t%s\t%s\t%s\tPRECHECK-BLOCKED\t%s\n' "$started" "$(now)" "$s" "$label" "$*" >> "$LOG"; tail -12 "$pre" >&2; exit 3
  fi
  sleep 20
done
{ echo "go $(now)"; echo "loadavg $(cat /proc/loadavg)"; } >> "$pre"
# DBG=1: same chrome plus a loopback DevTools port and a debugger-enabled CDP client (lc-g §9: the 10/10 runs made that way did not spin after a loop-close
# success; plain runs spun in 6/10 and lose their dump). Both run inside the lock; the client exits when meta.json appears.
if [ "${DBG:-0}" = 1 ]; then
  port=${LCG_CDP_PORT:-9347}
  LCG_CDP_PORT=$port BROWSER_EXECUTABLE=$PWD/slam_refact/tools/re/runtime/us025/chrome-debug-wrapper.sh US025_SCENARIOS=$D/scenarios.json \
    flock build/c1-chromium.lock bash -c 'node slam_refact/tools/re/runtime/us025/cdp_arm.mjs "$0" 100000 "$1" > "$2" 2>&1 & arm=$!; shift; shift; node slam_refact/tools/harness/us025/scene/run_lcgold_scene.mjs "$@"; rc=$?; kill $arm 2>/dev/null; exit $rc' \
    "$port" "$OUT_ROOT/$s/$label/meta.json" "$OUT_ROOT/$s/cdp-arm-$label.log" --scenario "$s" --label "$label" --out-root "$OUT_ROOT" "$@"
else
  US025_SCENARIOS=$D/scenarios.json flock build/c1-chromium.lock node slam_refact/tools/harness/us025/scene/run_lcgold_scene.mjs --scenario "$s" --label "$label" --out-root "$OUT_ROOT" "$@"
fi
rc=$?
st=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('status'))" "$OUT_ROOT/$s/$label/meta.json" 2>/dev/null || echo NO-META)
printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$started" "$(now)" "$s" "$label" "rc=$rc" "$st" "$*" >> "$LOG"
exit $rc
