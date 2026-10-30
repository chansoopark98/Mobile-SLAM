#!/usr/bin/env bash
# us025-lc-e2e-gate.v4: the registered command (registration document, "first real run command"). FORK of ../us025-lc-e2e-v2/run.sh (v2 forked v1, v1 forked us026-ba-e2e-v2/run.sh).
# Changed against v2 (design us025-lc-e2e-v3-parallel-design.md): gate version 3, the v3 names (designation variable, tool path, default output paths); step 4 is ONE
# `gate_lc.mjs run-all` call for all pending scenarios (phase A original runs in a global queue over the slots, then phase B bundles/replays/comparator in a pool of J jobs);
# optional `--jobs J` (1..8, default 6; J does not change any output byte). The number of slots is the registered SLOTS of gate_lc.mjs and cannot be given here.
# Same steps, same refusals, same R1 / R2 / R3 / R4 / R5.
#   R1  every PRECHECK_* variable of the caller is REMOVED from the environment of every child (node, python, bash) and recorded (name and value) in the log
#       before it is removed (final-a-review-harness M-4: a caller's PRECHECK_MAX_DISK_PCT=97 must not silently relax the resource check)
#   R2  a REAL run is refused (exit 2, before any marker) when `git status --porcelain` of the freeze paths is not empty or git fails (M-5); gate_lc.mjs
#       checks the same thing again in code, so `node gate_lc.mjs run` called directly cannot bypass it
#
# NOT RUN by the lane that wrote it on any registration scenario. The coordinator runs it, from the repository root, after the independent pre-review, the
# PENDING-PIN fill (make_lc_manifest.mjs --engine-commit ...), the designation and a commit of this gate directory:
#
#   US025_PREREG_DIR=<preregistration dir> SLAM_REFACT_US025_LC_V4_DESIGNATION_SHA256=<sha256 of the designation file> \
#     bash slam_refact/tools/gates/us025-lc-e2e-v4/run.sh [--dry-run | --resume] [--jobs J]
#
# (R3) an exit code 6 of one scenario (infrastructure failure after its marker) does NOT stop the other scenarios: each has its own marker.
# (R4) --resume continues after a stop: scenarios with a report.json are skipped (their report enters the verdict), scenarios with a marker but no report are NOT
#      restarted (that run was consumed: stop, coordinator decision + new version), scenarios without a marker are prechecked and run.
# (R5) a real run (also --resume) is refused (exit 2, before any marker) when / is more than START_DISK_PCT_MAX = 90 percent used (prereview C4; the in-run
#      precheck limit stays 95, so the gate has about 5 percent of the disk, roughly 90 GB, to grow while it runs).
# Optional: US025_OUT_ROOT (default build/slam-refact-pkgh-20261022/us025/gate-runs/us025-lc-e2e-gate-v4; must be below build/slam-refact-pkgh-20261022 and must not
# contain us025-lc-e2e-gate/<scenario> yet), US025_LOG_DIR (default <OUT_ROOT>/us025-lc-e2e-gate.logs).
#
# Order (everything goes through tee into $LOGDIR/run.sh.log, made read-only and summed at the end):
#   1 identity: command line, date, sha256 of this script / node / manifest / gate file, NAMES of the steering variables, the removed PRECHECK_* values,
#     git head, git status of the freeze paths
#   2 lane-precheck.sh (resources; informational; gate_lc.mjs repeats it before each marker and inside the Chromium lock before every original run)
#   3 `gate_lc.mjs precheck` for every registered scenario: every HELD check, no marker. Any refusal ends the script before a marker exists.
#   4 (not with --dry-run) ONE `gate_lc.mjs run-all` for the pending scenarios (manifest order). Phase A: the Chromium lock (build/c1-chromium.lock) and the slot locks are held; the original
#     runs of all scenarios go through one queue over SLOTS slots (a marker per scenario right before its first original run; each run: lane-precheck, load rule, slot CPU rule, watchdog).
#     Phase B (after A ended, locks released): exact-input bundles, the pinned engine replays and the comparator in a pool of J jobs, report, SHA256SUMS, read-only per scenario.
#     Exit 6 = infrastructure failure after a marker (that scenario's run is used: coordinator decision + new version; the other scenarios are still finished); 5 = resources never ready
#     before a marker (no marker for that scenario); 3 = HELD; any other non-zero stops the script.
#   5 sha256 of each output tree's SHA256SUMS, then `gate_lc.mjs verdict` (0 PASS, 1 FAIL, 4 BLOCKED/UNEVALUATED, 6 INFRA, 7 INSUFFICIENT, 8 INVALID).
# After it: the coordinator records first_real_run_at (registration document section 11), then `node gate_lc.mjs audit ...` (it also compares the pins now).
# Desktop headless Chromium (SwiftShader) and recorded inputs only; ephemeral 127.0.0.1 server inside the runner; nothing under web/.
set -u -o pipefail

GATE_ID=us025-lc-e2e-gate
GATE_VERSION=4
DESIGNATION_ENV=SLAM_REFACT_US025_LC_V4_DESIGNATION_SHA256
PRECHECK=slam_refact/docs/design/tools/lane-precheck.sh
TOOL=slam_refact/tools/gates/us025-lc-e2e-v4/gate_lc.mjs
FREEZE_PATHS="slam_refact/engine slam_refact/web-chunk slam_refact/web-smoke slam_refact/tools wasm src"

if [ -z "${US025_PREREG_DIR:-}" ]; then echo "run.sh: US025_PREREG_DIR is required (see the header)" >&2; exit 2; fi
if [ ! -f "$TOOL" ]; then echo "run.sh: run it from the repository root ($TOOL not found)" >&2; exit 2; fi
ORIG_ARGS=("$@")
DRY=0
RESUME=0
JOBS=""
START_DISK_PCT_MAX=90
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run) DRY=1; shift ;;
    --resume) RESUME=1; shift ;;
    --jobs) JOBS="${2:-}"; case "$JOBS" in [1-8]) ;; *) echo "run.sh: --jobs must be 1..8" >&2; exit 2 ;; esac; shift 2 ;;
    *) echo "run.sh: unknown argument $1 (only --dry-run, --resume, --jobs J)" >&2; exit 2 ;;
  esac
done
if [ "$DRY" = 1 ] && [ "$RESUME" = 1 ]; then echo "run.sh: --dry-run and --resume together" >&2; exit 2; fi
OUT_ROOT="${US025_OUT_ROOT:-build/slam-refact-pkgh-20261022/us025/gate-runs/us025-lc-e2e-gate-v4}"
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
  US025_RUN_INNER=1 bash "$0" "${ORIG_ARGS[@]}" 2>&1 | tee -a "$LOG"
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
echo "command: $0 ${ORIG_ARGS[*]:-}"
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

step "4 the runs (ONE run-all process: phase A original runs over the slots, phase B bundles/replays/comparator in a pool; one marker per scenario, once)"
ANY6=0
ANY5=0
if [ -n "$PENDING_SCENARIOS" ]; then
  PENDING_CSV=$(echo "$PENDING_SCENARIOS" | paste -sd, -)
  jobs_args=(); [ -n "$JOBS" ] && jobs_args=(--jobs "$JOBS")
  logged node "$TOOL" run-all "${gate_args[@]}" --scenarios "$PENDING_CSV" "${jobs_args[@]}" --run-log "$LOGDIR/run-all.$(date +%Y%m%dT%H%M%S).log"
  rc=$?
  echo "run-all exit code $rc"
  if [ "$rc" -eq 6 ]; then
    echo "run.sh: at least one scenario ended with an infrastructure failure after its marker (exit 6); every started scenario has its own report (each has its own marker)"
    ANY6=1
  elif [ "$rc" -eq 5 ]; then
    # v4 (G-21): exit 5 means "a scenario had no marker (resources never ready)"; it says nothing about the scenarios whose marker WAS consumed. Those have their reports (INFRA or not): they enter the
    # verdict below, and the final exit code stays 5.
    echo "run.sh: run-all exit 5: at least one scenario got no marker (resources never ready); scenarios with a taken marker have their reports and enter the verdict; the exit code stays 5"
    ANY5=1
  elif [ "$rc" -ne 0 ]; then echo "run.sh: stopped (run-all exit $rc); scenarios without a report were not finished (--resume continues scenarios without a marker; a scenario with a marker and no report is consumed)"; exit "$rc"; fi
  while read -r s; do
    [ -n "$s" ] || continue
    rep="$OUT_ROOT/$GATE_ID/$s/report.json"
    if [ "$ANY5" = 1 ] && [ ! -f "$rep" ]; then echo "run.sh: $s has no report (no marker was taken for it)"; continue; fi
    REPORTS="${REPORTS:+$REPORTS,}$rep"
  done <<< "$PENDING_SCENARIOS"
fi

step "5 sums and the gate verdict"
if [ "$ANY5" = 1 ] && [ -z "$REPORTS" ]; then echo "run.sh: no scenario has a report; nothing to judge (exit 5)"; exit 5; fi
for f in "$OUT_ROOT/$GATE_ID"/*/SHA256SUMS; do sha256sum "$f"; done
logged node "$TOOL" verdict --manifest "$MANIFEST" --reports "$REPORTS"
vrc=$?
echo "gate verdict exit code $vrc (0 PASS, 1 FAIL, 4 BLOCKED / UNEVALUATED, 6 INFRA, 7 INSUFFICIENT, 8 INVALID); any scenario with exit 6: $ANY6"
echo "next: the coordinator records first_real_run_at in the manifest, then: node $TOOL audit --prereg-dir $PREREG --gate-id $GATE_ID --gate-version $GATE_VERSION"
if [ "$ANY5" = 1 ]; then echo "run.sh: exit 5 (a scenario got no marker); the verdict above covers the scenarios that have a report only"; exit 5; fi
exit "$vrc"
