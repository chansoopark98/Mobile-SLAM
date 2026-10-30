#!/usr/bin/env bash
# or1-chunk-replay-gate.v4: the registered command (gate document section 9, "first real run command").
# FORK of slam_refact/tools/gates/or1-chunk-replay-v3/run.sh (sha256 ef8b8f54...e10f). Differences from v3 (gate document section 9.3):
#   R1  version 4, designation variable SLAM_REFACT_OR1_V4_DESIGNATION_SHA256; the scenarios, their derived directories and the judge's expectations come from the manifest
#       (manifest.registered_run.scenarios, manifest.pinned.gate_data), not from a fixed list of bundles
#   R2  exit codes do not collide: 0 PASS, 1 FAIL, 2 precondition / dirty tree / PRECHECK environment, 3 HELD, 4 INVALID (incomplete or invalid measurement), 5 judge error,
#       6 infrastructure failure after a marker was claimed, 7 lane precheck does not pass. In v3 a precheck that did not pass and a gate FAIL were both exit 1.
#   R3  a REAL run is refused (exit 2, before any marker) when `git status --porcelain` of the freeze paths is not empty, and the same check is repeated before every scenario
#       (review M-5); the dry run only records it
#   R4  a REAL run is refused (exit 2) when any PRECHECK_* variable is set (review M-4: a caller could relax the lane-precheck thresholds without a trace); the dry run
#       records the variables and removes them for the precheck call
#   R5  the judge is slam_refact/tools/gates/or1-chunk-replay-v4/judge.mjs (INVALID is never PASS or FAIL, a tool crash is exit 5, the registered call counts and floors),
#       not `or1.mjs verdict`; a scenario whose process exits non-zero after its marker is recorded in run-status.tsv and ends the run: the scenarios after it are not started
#       and the judge reports them MISSING (the verdict is INVALID, never FAIL); a FAIL of a scenario that did run still decides FAIL (exit 1)
# Everything else is the v3 text.
#
# NOT RUN by the lane that wrote it. Lane check: `bash -n` and `--dry-run` only (--dry-run stops after the prechecks and creates no marker and no output).
# The coordinator runs it, from the repository root, after F3, the independent review and the designation:
#
#   OR1_PREREG_DIR=<preregistration dir> OR1_ENGINE_DIR=<pinned wasm dir> OR1_NATIVE_BIN=<pinned slam_refact_replay> OR1_OUT_ROOT=<NEW dir under build/> \
#   SLAM_REFACT_OR1_V4_DESIGNATION_SHA256=<sha256 of the designation file> \
#     bash slam_refact/tools/gates/or1-chunk-replay-v4/run.sh [--dry-run]
#
# Optional: OR1_GATE_ID (or1-chunk-replay-gate), OR1_GATE_VERSION (4), OR1_DESIGNATION_ENV (the NAME of the variable above), OR1_LOG_DIR (default <OUT_ROOT>.logs),
# OR1_PRECHECK (default slam_refact/docs/design/tools/lane-precheck.sh). The PRECHECK_* thresholds are NOT the caller's: a real run refuses them.
#
# What it does, in this order (everything goes through tee into $OR1_LOG_DIR/run.sh.log, made read-only and summed when it ends):
#   1 the command line, the date, the sha256 of this script / node / the manifest / the gate file, the NAMES of the steering environment variables and the sha256 of the
#     designation variable's value (never the value of any other), the load, git HEAD and the freeze-path git status
#   2 lane-precheck.sh (resources; read only)
#   3 `or1.mjs precheck` for every registered scenario: every HELD check, no marker. Any refusal ends the script before a marker exists.
#   4 (not with --dry-run) `or1.mjs run --kind gate` for the scenarios in the manifest's order, one process each, the exit code of each recorded in run-status.tsv.
#   5 sha256 of each output tree's SHA256SUMS, then the judge with its exit code.
# After it: the coordinator records first_real_run_at in the manifest, then `node slam_refact/web-chunk/tools/or1/or1.mjs audit --prereg-dir ... --gate-id ... --gate-version ...`.
# The run is a desktop node replay of recorded inputs; no browser (so no Chromium lock), no server, no network, nothing under web/.
set -u -o pipefail

GATE_ID="${OR1_GATE_ID:-or1-chunk-replay-gate}"
GATE_VERSION="${OR1_GATE_VERSION:-4}"
DESIGNATION_ENV="${OR1_DESIGNATION_ENV:-SLAM_REFACT_OR1_V4_DESIGNATION_SHA256}"
PRECHECK="${OR1_PRECHECK:-slam_refact/docs/design/tools/lane-precheck.sh}"
OR1=slam_refact/web-chunk/tools/or1/or1.mjs
JUDGE=slam_refact/tools/gates/or1-chunk-replay-v4/judge.mjs
FREEZE_PATHS="slam_refact/engine slam_refact/web-chunk slam_refact/web-smoke slam_refact/tools wasm src"

need() { if [ -z "${!1:-}" ]; then echo "run.sh: $1 is required (see the header)" >&2; exit 2; fi; }
for v in OR1_PREREG_DIR OR1_ENGINE_DIR OR1_NATIVE_BIN OR1_OUT_ROOT; do need "$v"; done
if [ ! -f "$OR1" ] || [ ! -f "$JUDGE" ]; then echo "run.sh: run it from the repository root ($OR1 or $JUDGE not found)" >&2; exit 2; fi
DRY=0
case "${1:-}" in
  --dry-run) DRY=1 ;;
  '') ;;
  *) echo "run.sh: unknown argument $1 (only --dry-run)" >&2; exit 2 ;;
esac
case "$OR1_OUT_ROOT" in build/*) ;; *) echo "run.sh: OR1_OUT_ROOT must be a path under build/" >&2; exit 2 ;; esac
LOGDIR="${OR1_LOG_DIR:-${OR1_OUT_ROOT}.logs}"
if [ "$DRY" = 1 ]; then LOG="$LOGDIR/run.sh.dry-run.$(date +%Y%m%dT%H%M%S).$$.log"; else LOG="$LOGDIR/run.sh.log"; fi

# R4: the PRECHECK_* variables of the caller (names and values: they are thresholds and test injections, not secrets)
PRECHECK_SET=$(env | grep '^PRECHECK_' | sort || true)
if [ "$DRY" = 0 ] && [ -n "$PRECHECK_SET" ]; then
  echo "run.sh: PRECHECK_* is set in the environment; a real run uses the lane-precheck defaults and refuses to start (nothing was run, no log):" >&2
  echo "$PRECHECK_SET" | sed 's/^/  /' >&2
  exit 2
fi

# the outer shell: everything the inner shell prints goes through tee into the log, then the log is summed and made read-only
if [ -z "${OR1_RUN_INNER:-}" ]; then
  mkdir -p "$LOGDIR" || exit 2
  set -o noclobber
  : > "$LOG" || { echo "run.sh: $LOG exists: a gate log is never reused" >&2; exit 2; }
  set +o noclobber
  OR1_RUN_INNER=1 OR1_RUN_LOG="$LOG" bash "$0" "$@" 2>&1 | tee -a "$LOG"
  rc=${PIPESTATUS[0]}
  chmod a-w "$LOG"
  sha256sum "$LOG" > "$LOG.sha256"
  echo "log: $LOG (exit code of the run: $rc, sha256 in $LOG.sha256)"
  exit "$rc"
fi

trap 'echo "## EXIT $? at $(date -Is)"' EXIT
step() { echo "## $(date -Is) STEP $*"; }
logged() { echo "## CMD: $*"; "$@" </dev/null; local rc=$?; echo "## RC $rc: ${1:-} ${2:-}"; return $rc; }
freeze_dirty() { git status --porcelain -- $FREEZE_PATHS 2>&1; }

PREREG="$OR1_PREREG_DIR"
MANIFEST="$PREREG/$GATE_ID.v$GATE_VERSION.manifest.json"
GATEMD="$PREREG/$GATE_ID.v$GATE_VERSION.md"
step "1 identity"
echo "command: $0 $*"
echo "cwd: $PWD  user: $(id -un)  host: $(hostname)"
echo "gate: $GATE_ID v$GATE_VERSION  prereg: $PREREG  engine: $OR1_ENGINE_DIR  native: $OR1_NATIVE_BIN  out: $OR1_OUT_ROOT  dry-run: $DRY"
echo "node: $(node --version) $(sha256sum "$(readlink -f "$(command -v node)")")"
echo "run.sh: $(sha256sum "$0")"
echo "judge: $(sha256sum "$JUDGE")"
for f in "$MANIFEST" "$GATEMD"; do if [ -f "$f" ]; then sha256sum "$f"; else echo "MISSING: $f"; fi; done
echo "environment variable names (values are not printed): $(env | cut -d= -f1 | grep -E '^(SLAM_REFACT_|OR1_|PRECHECK_|NODE_)' | sort | tr '\n' ' ')"
if [ -n "$PRECHECK_SET" ]; then echo "PRECHECK_* variables of the caller (dry run only; removed for the precheck call):"; echo "$PRECHECK_SET" | sed 's/^/  /'; else echo "PRECHECK_* variables: none"; fi
if [ -n "${!DESIGNATION_ENV:-}" ]; then echo "$DESIGNATION_ENV: set, value sha256 $(printf %s "${!DESIGNATION_ENV}" | sha256sum | cut -d' ' -f1)"; else echo "$DESIGNATION_ENV: NOT SET (the run will be HELD)"; fi
echo "load: $(cat /proc/loadavg)"
echo "git: $(git rev-parse HEAD 2>/dev/null || echo unavailable) on $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unavailable)"
DIRTY=$(freeze_dirty)
if [ -n "$DIRTY" ]; then
  echo "git status (freeze paths $FREEZE_PATHS) is NOT empty:"; echo "$DIRTY" | sed 's/^/  /'
  if [ "$DRY" = 0 ]; then echo "run.sh: a real run needs an empty freeze-path git status (commit or restore first; review M-5); nothing was run"; exit 2; fi
else echo "git status (freeze paths): empty"; fi

step "2 lane precheck"
if [ -f "$PRECHECK" ]; then
  # shellcheck disable=SC2046
  logged env $(env | cut -d= -f1 | grep '^PRECHECK_' | sed 's/^/-u /') bash "$PRECHECK" || { echo "run.sh: lane precheck did not pass; nothing was run"; exit 7; }
else echo "run.sh: $PRECHECK not found; nothing was run"; exit 7; fi

# the registered scenarios: name and derived directory, in the manifest's order
SCENARIOS=$(python3 -I - "$MANIFEST" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
byname = {d['name']: d for d in m['pinned']['gate_data']}
for s in m['registered_run']['scenarios']:
    print(s['name'], byname[s['name']]['derived_dir'])
PY
) || { echo "run.sh: cannot read the registered scenarios from $MANIFEST"; exit 2; }
echo "registered scenarios:"; echo "$SCENARIOS" | sed 's/^/  /'

gate_args=(--prereg-dir "$PREREG" --gate-id "$GATE_ID" --gate-version "$GATE_VERSION" --designation-env "$DESIGNATION_ENV")
scenario_args() { echo --bundle "$1" --engine-dir "$OR1_ENGINE_DIR" --native-bin "$OR1_NATIVE_BIN" --arms WCN --reps 3 --kind gate; }

step "3 precheck of every scenario (no marker)"
while read -r name dir; do
  # shellcheck disable=SC2046
  logged node "$OR1" precheck "${gate_args[@]}" $(scenario_args "$dir") || { echo "run.sh: HELD or failed for $name; no marker was created, nothing was run"; exit 3; }
done <<< "$SCENARIOS"
if [ "$DRY" = 1 ]; then echo "dry run: stopping before the first real run (no marker, no output)"; exit 0; fi

step "4 the runs (one process per scenario, once)"
STATUS="$LOGDIR/run-status.tsv"
printf 'scenario\texit_code\tmarker\treport\n' > "$STATUS"
stopped=0
while read -r name dir; do
  DIRTY=$(freeze_dirty)
  if [ -n "$DIRTY" ]; then echo "run.sh: the freeze-path git status changed during the run:"; echo "$DIRTY" | sed 's/^/  /'; echo "run.sh: stopped before $name; it and the scenarios after it were not started"; stopped=2; break; fi
  # shellcheck disable=SC2046
  logged node "$OR1" run "${gate_args[@]}" $(scenario_args "$dir") --out "$OR1_OUT_ROOT/$name" --run-log "$LOGDIR/$name.run.log"
  rc=$?
  marker=no; [ -e "$PREREG/$GATE_ID.v$GATE_VERSION.first-run.$name.marker" ] && marker=yes
  report=no; [ -f "$OR1_OUT_ROOT/$name/report.json" ] && report=yes
  printf '%s\t%s\t%s\t%s\n' "$name" "$rc" "$marker" "$report" >> "$STATUS"
  echo "scenario $name exit code $rc (marker $marker, report $report)"
  if [ "$rc" -ne 0 ]; then
    if [ "$rc" -eq 3 ] && [ "$marker" = no ]; then echo "run.sh: HELD at $name before its marker; the scenarios after it were not started"; stopped=3; else
      echo "run.sh: infrastructure failure at $name (exit $rc, marker $marker, report $report): its run is used (gate document section 8); the scenarios after it were not started and the judge reports them MISSING"; stopped=6; fi
    break
  fi
done <<< "$SCENARIOS"
sha256sum "$STATUS"

step "5 sums and the gate verdict"
for f in "$OR1_OUT_ROOT"/*/SHA256SUMS; do [ -f "$f" ] && sha256sum "$f"; done
# the judge reads <out root>/<scenario>/report.json for every registered scenario; a missing report is MISSING (INVALID), never FAIL
logged node "$JUDGE" --manifest "$MANIFEST" --out-root "$OR1_OUT_ROOT" --run-status "$STATUS"
vrc=$?
echo "gate verdict exit code $vrc (0 PASS, 1 FAIL, 4 INVALID, 5 judge error); run stopped early: $stopped"
echo "next: the coordinator records first_real_run_at in the manifest, then: node $OR1 audit --prereg-dir $PREREG --gate-id $GATE_ID --gate-version $GATE_VERSION"
if [ "$stopped" -ne 0 ] && [ "$vrc" -eq 0 ]; then echo "run.sh: INCONSISTENT: the run stopped early but the judge says PASS"; exit 5; fi
# a FAIL of a scenario that did run decides FAIL (exit 1); otherwise an early stop is reported by its own code (2 dirty tree, 3 HELD, 6 infrastructure; the verdict is INVALID)
if [ "$vrc" -ne 1 ] && [ "$stopped" -ne 0 ]; then exit "$stopped"; fi
exit "$vrc"
