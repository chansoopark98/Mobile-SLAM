#!/usr/bin/env bash
# us025-lc-e2e-gate.v2: the registered command (registration document section 11, "first real run command"). FORK of ../us025-lc-e2e-v1/run.sh (v1 itself forked from
# slam_refact/tools/gates/us026-ba-e2e-v2/run.sh). Changed against v1: gate version 2, the v2 names (designation variable, tool path, default output paths). Same steps, same refusals.
#   R1  every PRECHECK_* variable of the caller is REMOVED from the environment of every child (node, python, bash) and recorded (name and value) in the log
#       before it is removed (final-a-review-harness M-4: a caller's PRECHECK_MAX_DISK_PCT=97 must not silently relax the resource check)
#   R2  a REAL run is refused (exit 2, before any marker) when `git status --porcelain` of the freeze paths is not empty or git fails (M-5); gate_lc.mjs
#       checks the same thing again in code, so `node gate_lc.mjs run` called directly cannot bypass it
#
# NOT RUN by the lane that wrote it on any registration scenario. The coordinator runs it, from the repository root, after the independent pre-review, the
# PENDING-PIN fill (make_lc_manifest.mjs --engine-commit ...), the designation and a commit of this gate directory:
#
#   US025_PREREG_DIR=<preregistration dir> SLAM_REFACT_US025_LC_V2_DESIGNATION_SHA256=<sha256 of the designation file> \
#     bash slam_refact/tools/gates/us025-lc-e2e-v2/run.sh [--dry-run | --resume]
#
# (R3) an exit code 6 of one scenario (infrastructure failure after its marker) does NOT stop the other scenarios: each has its own marker.
# (R4) --resume continues after a stop: scenarios with a report.json are skipped (their report enters the verdict), scenarios with a marker but no report are NOT
#      restarted (that run was consumed: stop, coordinator decision + new version), scenarios without a marker are prechecked and run.
# (R5) a real run (also --resume) is refused (exit 2, before any marker) when / is more than START_DISK_PCT_MAX = 90 percent used (prereview C4; the in-run
#      precheck limit stays 95, so the gate has about 5 percent of the disk, roughly 90 GB, to grow while it runs).
# Optional: US025_OUT_ROOT (default build/slam-refact-pkgh-20261022/us025/gate-runs/us025-lc-e2e-gate-v2; must be below build/slam-refact-pkgh-20261022 and must not
# contain us025-lc-e2e-gate/<scenario> yet), US025_LOG_DIR (default <OUT_ROOT>/us025-lc-e2e-gate.logs).
#
# Order (everything goes through tee into $LOGDIR/run.sh.log, made read-only and summed at the end):
#   1 identity: command line, date, sha256 of this script / node / manifest / gate file, NAMES of the steering variables, the removed PRECHECK_* values,
#     git head, git status of the freeze paths
#   2 lane-precheck.sh (resources; informational; gate_lc.mjs repeats it before each marker and inside the Chromium lock before every original run)
#   3 `gate_lc.mjs precheck` for every registered scenario: every HELD check, no marker. Any refusal ends the script before a marker exists.
#   4 (not with --dry-run) `gate_lc.mjs run` for the scenarios in the manifest's order, one process each: marker (kind = scenario), output directory, the
#     original lc=1 and lc=0 runs (each under flock build/c1-chromium.lock, resources checked again inside the lock), exact-input bundles, the pinned engine replays,
#     the comparator, report, SHA256SUMS, read-only. The first non-zero exit other than 6 stops the script (5 = resources never ready: no marker; 3 = HELD; 6 = infrastructure
#     failure after the marker: that scenario's run is used, coordinator decision + new version, the other scenarios still run, R3).
#   5 sha256 of each output tree's SHA256SUMS, then `gate_lc.mjs verdict` (0 PASS, 1 FAIL, 4 BLOCKED/UNEVALUATED, 6 INFRA, 7 INSUFFICIENT, 8 INVALID).
# After it: the coordinator records first_real_run_at (registration document section 11), then `node gate_lc.mjs audit ...` (it also compares the pins now).
# Desktop headless Chromium (SwiftShader) and recorded inputs only; ephemeral 127.0.0.1 server inside the runner; nothing under web/.
set -u -o pipefail

GATE_ID=us025-lc-e2e-gate
GATE_VERSION=2
DESIGNATION_ENV=SLAM_REFACT_US025_LC_V2_DESIGNATION_SHA256
PRECHECK=slam_refact/docs/design/tools/lane-precheck.sh
TOOL=slam_refact/tools/gates/us025-lc-e2e-v2/gate_lc.mjs
FREEZE_PATHS="slam_refact/engine slam_refact/web-chunk slam_refact/web-smoke slam_refact/tools wasm src"

if [ -z "${US025_PREREG_DIR:-}" ]; then echo "run.sh: US025_PREREG_DIR is required (see the header)" >&2; exit 2; fi
if [ ! -f "$TOOL" ]; then echo "run.sh: run it from the repository root ($TOOL not found)" >&2; exit 2; fi
DRY=0
RESUME=0
START_DISK_PCT_MAX=90
case "${1:-}" in
  --dry-run) DRY=1 ;;
  --resume) RESUME=1 ;;
  '') ;;
  *) echo "run.sh: unknown argument $1 (only --dry-run or --resume)" >&2; exit 2 ;;
esac
OUT_ROOT="${US025_OUT_ROOT:-build/slam-refact-pkgh-20261022/us025/gate-runs/us025-lc-e2e-gate-v2}"
case "$OUT_ROOT" in
  *..*) echo "run.sh: US025_OUT_ROOT must not contain .." >&2; exit 2 ;;
  build/slam-refact-pkgh-20261022/*) ;;
  *) echo "run.sh: US025_OUT_ROOT must be under build/slam-refact-pkgh-20261022/ (run_lcgold.mjs writes only there)" >&2; exit 2 ;;
esac
LOGDIR="${US025_LOG_DIR:-$OUT_ROOT/$GATE_ID.logs}"
if [ "$DRY" = 1 ]; then LOG="$LOGDIR/run.sh.dry-run.$(date +%Y%m%dT%H%M%S).$$.log"
elif [ "$RESUME" = 1 ]; then LOG="$LOGDIR/run.sh.resume.$(date +%Y%m%dT%H%M%S).$$.log"
else LOG="$LOGDIR/run.sh.log"; fi

if [ -z "${US025_RUN_INNER:-}" ]; then
  mkdir -p "$LOGDIR" || exit 2
  set -o noclobber
  : > "$LOG" || { echo "run.sh: $LOG exists: a gate log is never reused" >&2; exit 2; }
  set +o noclobber
  US025_RUN_INNER=1 bash "$0" "$@" 2>&1 | tee -a "$LOG"
  rc=${PIPESTATUS[0]}
  chmod a-w "$LOG"
  sha256sum "$LOG" > "$LOG.sha256"
  echo "log: $LOG (exit code of the run: $rc, sha256 in $LOG.sha256)"
  exit "$rc"
fi

trap 'echo "## EXIT $? at $(date -Is)"' EXIT
step() { echo "## $(date -Is) STEP $*"; }

# R1: the PRECHECK_* variables are recorded, then removed from every child
mapfile -t PRE_VARS < <(env | cut -d= -f1 | grep '^PRECHECK_' || true)
CLEAN=(env)
for v in "${PRE_VARS[@]}"; do [ -n "$v" ] && CLEAN+=(-u "$v"); done
logged() { echo "## CMD: $*"; "${CLEAN[@]}" "$@" </dev/null; local rc=$?; echo "## RC $rc: ${1:-} ${2:-} ${3:-}"; return $rc; }

PREREG="$US025_PREREG_DIR"
MANIFEST="$PREREG/$GATE_ID.v$GATE_VERSION.manifest.json"
GATEMD="$PREREG/$GATE_ID.v$GATE_VERSION.md"
step "1 identity"
echo "command: $0 $*"
echo "cwd: $PWD  user: $(id -un)  host: $(hostname)"
echo "gate: $GATE_ID v$GATE_VERSION  prereg: $PREREG  out: $OUT_ROOT  logs: $LOGDIR  dry-run: $DRY"
echo "node: $(node --version) $(sha256sum "$(readlink -f "$(command -v node)")")"
echo "run.sh: $(sha256sum "$0")"
for f in "$MANIFEST" "$GATEMD"; do if [ -f "$f" ]; then sha256sum "$f"; else echo "MISSING: $f"; fi; done
echo "environment variable names (values are not printed): $(env | cut -d= -f1 | grep -E '^(SLAM_REFACT_|US025_|US026_|PRECHECK_|NODE_|BROWSER_|PLAYWRIGHT_|TRKPROBE_|ITPROBE_)' | sort | tr '\n' ' ')"
if [ "${#PRE_VARS[@]}" -gt 0 ] && [ -n "${PRE_VARS[0]}" ]; then
  echo "PRECHECK_* in the caller's environment (REMOVED for every child; recorded here because they are thresholds, not secrets):"
  for v in "${PRE_VARS[@]}"; do echo "  $v=${!v}"; done
else echo "PRECHECK_* in the caller's environment: none"; fi
if [ -n "${!DESIGNATION_ENV:-}" ]; then echo "$DESIGNATION_ENV: set, value sha256 $(printf %s "${!DESIGNATION_ENV}" | sha256sum | cut -d' ' -f1)"; else echo "$DESIGNATION_ENV: NOT SET (the run will be HELD)"; fi
echo "load: $(cat /proc/loadavg)"; echo "disk: $(df -h / | tail -1)"
DISK_PCT=$(df -P / | awk 'NR==2{gsub("%","",$5); print $5}')
if [ -z "$DISK_PCT" ] || [ "$DISK_PCT" -gt "$START_DISK_PCT_MAX" ]; then
  echo "disk usage of / is ${DISK_PCT:-unknown}% (start condition: <= ${START_DISK_PCT_MAX}%)"
  if [ "$DRY" = 0 ]; then echo "run.sh: a real run needs <= ${START_DISK_PCT_MAX}% disk usage (clean up first; deleting files needs the user's confirmation); nothing was run"; exit 2; fi
else echo "disk start condition ok: ${DISK_PCT}% <= ${START_DISK_PCT_MAX}%"; fi
echo "python3: $(command -v python3) $(python3 --version 2>&1)"; echo "node: $(readlink -f "$(command -v node)")"; echo "chrome: $(/usr/bin/google-chrome --version 2>&1)"
echo "git: $(git rev-parse HEAD 2>/dev/null || echo unavailable) on $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unavailable)"
# shellcheck disable=SC2086
FREEZE_DIRTY=$(git status --porcelain -- $FREEZE_PATHS 2>&1; echo "git-exit:$?")
if [ "$FREEZE_DIRTY" != "git-exit:0" ]; then
  echo "git status (freeze paths $FREEZE_PATHS) is NOT empty (or git failed):"; echo "$FREEZE_DIRTY" | head -20 | sed 's/^/  /'
  if [ "$DRY" = 0 ]; then echo "run.sh: a real run needs an empty freeze-path git status (commit or restore first); nothing was run"; exit 2; fi
else echo "git status (freeze paths): empty"; fi

step "2 lane precheck (informational; repeated by gate_lc.mjs before each marker and inside the lock)"
if [ -f "$PRECHECK" ]; then
  "${CLEAN[@]}" PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0 bash "$PRECHECK" || echo "run.sh: lane precheck does not pass now; gate_lc.mjs waits for it before each marker (thresholds unchanged)"
else echo "run.sh: $PRECHECK not found; nothing was run"; exit 1; fi

SCENARIOS=$(python3 -I - "$MANIFEST" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
for d in m['registered_run']['scenarios']:
    print(d['name'])
PY
) || { echo "run.sh: cannot read the registered scenarios from $MANIFEST"; exit 2; }
echo "registered scenarios:"; echo "$SCENARIOS" | sed 's/^/  /'
gate_args=(--prereg-dir "$PREREG" --gate-id "$GATE_ID" --gate-version "$GATE_VERSION" --designation-env "$DESIGNATION_ENV" --out-root "$OUT_ROOT")

step "3 precheck of every scenario that still has to run (no marker)"
PENDING_SCENARIOS=""
REPORTS=""
while read -r s; do
  rep="$OUT_ROOT/$GATE_ID/$s/report.json"
  marker="$PREREG/$GATE_ID.v$GATE_VERSION.first-run.$s.marker"
  if [ "$RESUME" = 1 ] && [ -f "$rep" ]; then echo "resume: $s already has a report ($rep); skipped, its report enters the verdict"; REPORTS="${REPORTS:+$REPORTS,}$rep"; continue; fi
  if [ "$RESUME" = 1 ] && [ -e "$marker" ]; then echo "run.sh: --resume: $s has a marker but no report: that run was consumed and is not restarted (coordinator decision + new version); nothing was run"; exit 3; fi
  logged node "$TOOL" precheck "${gate_args[@]}" --scenario "$s" || { echo "run.sh: HELD or failed for $s; no marker was created, nothing was run"; exit 3; }
  PENDING_SCENARIOS="${PENDING_SCENARIOS:+$PENDING_SCENARIOS$'\n'}$s"
done <<< "$SCENARIOS"
if [ "$DRY" = 1 ]; then echo "dry run: stopping before the first real run (no marker, no output)"; exit 0; fi

step "4 the runs (one process per scenario, once)"
ANY6=0
while read -r s; do
  [ -n "$s" ] || continue
  logged node "$TOOL" run "${gate_args[@]}" --scenario "$s" --run-log "$LOGDIR/$s.run.$(date +%Y%m%dT%H%M%S).log"
  rc=$?
  echo "scenario $s exit code $rc"
  if [ "$rc" -eq 6 ]; then
    echo "run.sh: $s ended with an infrastructure failure after its marker (exit 6); the other scenarios are still run (each has its own marker)"
    ANY6=1
  elif [ "$rc" -ne 0 ]; then echo "run.sh: stopped at $s (exit $rc); scenarios after it were not started (--resume continues)"; exit "$rc"; fi
  REPORTS="${REPORTS:+$REPORTS,}$OUT_ROOT/$GATE_ID/$s/report.json"
done <<< "$PENDING_SCENARIOS"

step "5 sums and the gate verdict"
for f in "$OUT_ROOT/$GATE_ID"/*/SHA256SUMS; do sha256sum "$f"; done
logged node "$TOOL" verdict --manifest "$MANIFEST" --reports "$REPORTS"
vrc=$?
echo "gate verdict exit code $vrc (0 PASS, 1 FAIL, 4 BLOCKED / UNEVALUATED, 6 INFRA, 7 INSUFFICIENT, 8 INVALID); any scenario with exit 6: $ANY6"
echo "next: the coordinator records first_real_run_at in the manifest, then: node $TOOL audit --prereg-dir $PREREG --gate-id $GATE_ID --gate-version $GATE_VERSION"
exit "$vrc"
