#!/usr/bin/env bash
# US-026 dev run wrapper (design us026-design.md §2.7, §5.4): one ORIGINAL SDK run through run-reference.mjs with
# the resource precheck, the shared Chromium lock and a dev-run log line. DEV scenarios only.
#
#   bash slam_refact/tools/re/runtime/us026/dev-run.sh <scenario> <label> <purpose> [run-reference options...]
#
# - Refuses any scenario not in DEV_SCENARIOS (registration inputs must stay unobserved for BA / method N).
# - Runs `lane-precheck.sh` with PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0; on failure it prints the reason and
#   exits 3 without starting a browser (no threshold change, no process is killed).
# - The browser runs under `flock build/c1-chromium.lock` (one headless Chromium at a time).
# - Appends one line to build/slam-refact-full-20261019/us026-draft/dev-run-log.tsv (also for refused / failed runs).
# - Output root: build/slam-refact-full-20261019/us026/runs (run-reference.mjs --out-root).
set -u
cd "$(dirname "$0")/../../../../.."
DEV_SCENARIOS="flip6-n1 flip6-arcore-s3-absolute"
LOG=build/slam-refact-full-20261019/us026-draft/dev-run-log.tsv
OUT_ROOT=build/slam-refact-full-20261019/us026/runs
[ $# -ge 3 ] || { echo "usage: dev-run.sh <scenario> <label> <purpose> [run-reference options...]" >&2; exit 2; }
s=$1 label=$2 purpose=$3; shift 3
now() { date +%Y-%m-%dT%H:%M:%S%z; }
n=$(( $(wc -l < "$LOG") ))
runner_sha=$(sha256sum slam_refact/tools/re/runtime/run-reference.mjs | cut -d' ' -f1)
log() {  # log <started> <exit> <status> <run-reference options...>
  local t0=$1 rc=$2 st=$3; shift 3
  printf '%s\t%s\t%s\t%s\t%s\tdev\t%s\t%s\t%s\t%s\t%s\t%s\n' "$n" "$t0" "$(now)" "$s" "$label" "$runner_sha" "$*" "$purpose" "$rc" "$st" "$OUT_ROOT/$s/$label" >> "$LOG"
}
started=$(now)
case " $DEV_SCENARIOS " in *" $s "*) ;; *) echo "refused: $s is not a dev scenario ($DEV_SCENARIOS)" >&2; log "$started" 2 REFUSED-NOT-DEV "$@"; exit 2 ;; esac
if ! PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0 bash slam_refact/docs/design/tools/lane-precheck.sh; then
  log "$started" 3 PRECHECK-BLOCKED "$@"; exit 3
fi
flock build/c1-chromium.lock node slam_refact/tools/re/runtime/run-reference.mjs --scenario "$s" --label "$label" --out-root "$OUT_ROOT" "$@"
rc=$?
st=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1])).get('status'))" "$OUT_ROOT/$s/$label/meta.json" 2>/dev/null || echo NO-META)
log "$started" "$rc" "$st" "$@"
exit $rc
