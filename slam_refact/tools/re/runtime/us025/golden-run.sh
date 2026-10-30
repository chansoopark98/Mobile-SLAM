#!/usr/bin/env bash
# US-025 LC-G dev/golden run wrapper (NOT a registered gate run): one ORIGINAL SDK run through run_lcgold.mjs with the resource precheck and the shared
# Chromium lock. Output root build/slam-refact-pkgh-20261022/us025/golden/runs (the only root run_lcgold.mjs accepts).
#
#   bash slam_refact/tools/re/runtime/us025/golden-run.sh <scenario> <label> [run_lcgold.mjs options...]
#
# - lane-precheck.sh with PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0; PRECHECK_MAX_DISK_PCT is only raised when the caller exports it
#   (dev/golden runs only; the override and the observed `df`/load are recorded next to the run in precheck.txt).
# - The browser runs under `flock build/c1-chromium.lock`.
# - Appends one line to build/slam-refact-pkgh-20261022/us025/golden/run-log.tsv.
set -u
cd "$(dirname "$0")/../../../../.."
[ $# -ge 2 ] || { echo "usage: golden-run.sh <scenario> <label> [run_lcgold.mjs options...]" >&2; exit 2; }
s=$1 label=$2; shift 2
OUT_ROOT=build/slam-refact-pkgh-20261022/us025/golden/runs
LOG=build/slam-refact-pkgh-20261022/us025/golden/run-log.tsv
mkdir -p "$OUT_ROOT/$s" "$(dirname "$LOG")"
now() { date +%Y-%m-%dT%H:%M:%S%z; }
started=$(now)
pre=$OUT_ROOT/$s/precheck-$label.txt
{ echo "started $started"; echo "loadavg $(cat /proc/loadavg)"; df -h / | tail -1; echo "PRECHECK_MAX_DISK_PCT=${PRECHECK_MAX_DISK_PCT:-<default 95>}"; } > "$pre"
# MAX_BUILDS (default 0): concurrent top-level builds tolerated by the precheck (other lanes build in this checkout; dev/golden runs only, recorded in precheck.txt).
# WAIT_MIN (default 0): keep re-checking every 20 s for up to this many minutes (other lanes build in the shared checkout); LOAD_MAX (default
# unset): additionally wait until the 1-minute load average is below it (the original SDK is load sensitive, us026/README.md NR-BA-2).
deadline=$(( $(date +%s) + ${WAIT_MIN:-0} * 60 ))
while :; do
  ok=1
  PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=${MAX_BUILDS:-0} bash slam_refact/docs/design/tools/lane-precheck.sh >> "$pre" 2>&1 || ok=0
  if [ $ok = 1 ] && [ -n "${LOAD_MAX:-}" ]; then
    awk -v l="$(cut -d' ' -f1 /proc/loadavg)" -v m="$LOAD_MAX" 'BEGIN{exit !(l < m)}' || { ok=0; echo "load $(cut -d' ' -f1 /proc/loadavg) >= LOAD_MAX $LOAD_MAX" >> "$pre"; }
  fi
  [ $ok = 1 ] && break
  if [ "$(date +%s)" -ge "$deadline" ]; then
    printf '%s\t%s\t%s\t%s\tPRECHECK-BLOCKED\t%s\n' "$started" "$(now)" "$s" "$label" "$*" >> "$LOG"; tail -12 "$pre" >&2; exit 3
  fi
  sleep 20
done
{ echo "go $(now)"; echo "loadavg $(cat /proc/loadavg)"; } >> "$pre"
flock build/c1-chromium.lock node slam_refact/tools/re/runtime/us025/run_lcgold.mjs --scenario "$s" --label "$label" --out-root "$OUT_ROOT" "$@"
rc=$?
st=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('status'))" "$OUT_ROOT/$s/$label/meta.json" 2>/dev/null || echo NO-META)
printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\n' "$started" "$(now)" "$s" "$label" "rc=$rc" "$st" "$*" >> "$LOG"
exit $rc
