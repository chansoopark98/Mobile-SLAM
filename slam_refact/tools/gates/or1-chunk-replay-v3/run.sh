#!/usr/bin/env bash
# or1-chunk-replay-gate.v3: the registered command (gate document section "first real run command").
#
# NOT RUN by the lane that wrote it. Lane check: `bash -n` and `--dry-run` only (--dry-run stops after the prechecks and creates no marker and no output).
# The coordinator runs it, from the repository root, after the freeze, the independent review and the designation:
#
#   OR1_PREREG_DIR=<preregistration dir> OR1_ENGINE_DIR=<pinned wasm dir> OR1_NATIVE_BIN=<pinned slam_refact_replay> OR1_OUT_ROOT=<NEW dir under build/> \
#   SLAM_REFACT_OR1_V3_DESIGNATION_SHA256=<sha256 of the designation file> \
#     bash slam_refact/tools/gates/or1-chunk-replay-v3/run.sh [--dry-run]
#
# Optional: OR1_GATE_ID (or1-chunk-replay-gate), OR1_GATE_VERSION (3), OR1_DESIGNATION_ENV (the NAME of the variable above), OR1_LOG_DIR (default <OUT_ROOT>.logs),
# OR1_PRECHECK (default slam_refact/docs/design/tools/lane-precheck.sh; its PRECHECK_* thresholds are the caller's, this script overrides none).
#
# What it does, in this order, and what it records (everything below goes through tee into $OR1_LOG_DIR/run.sh.log, which is made read-only and summed when it ends):
#   1 the command line, the date, the sha256 of this script / node / the manifest / the gate file, the NAMES of the steering environment variables and the sha256 of the
#     designation variable's value (never the value of any other)
#   2 lane-precheck.sh (resources; read only)
#   3 `or1.mjs precheck` for every registered bundle: every HELD check, no marker. Any refusal ends the script before a marker exists.
#   4 (not with --dry-run) `or1.mjs run --kind gate` for the bundles in the manifest's order, one process each, the exit code of each recorded. The first non-zero exit
#     stops the script (a marker already claimed stays claimed: an infrastructure failure needs a new version, gate document).
#   5 sha256 of each output tree's SHA256SUMS, then `or1.mjs verdict` (C4 record rule, C9 asymmetry) with its exit code: 0 PASS, 1 FAIL, 4 INSUFFICIENT / UNEVALUATED.
# After it: the coordinator records first_real_run_at in the manifest, then `node slam_refact/web-chunk/tools/or1/or1.mjs audit --prereg-dir ... --gate-id ... --gate-version ...`.
# The run is a desktop node replay of recorded inputs; no browser, no server, no network, nothing under web/.
set -u -o pipefail

GATE_ID="${OR1_GATE_ID:-or1-chunk-replay-gate}"
GATE_VERSION="${OR1_GATE_VERSION:-3}"
DESIGNATION_ENV="${OR1_DESIGNATION_ENV:-SLAM_REFACT_OR1_V3_DESIGNATION_SHA256}"
PRECHECK="${OR1_PRECHECK:-slam_refact/docs/design/tools/lane-precheck.sh}"
OR1=slam_refact/web-chunk/tools/or1/or1.mjs

need() { if [ -z "${!1:-}" ]; then echo "run.sh: $1 is required (see the header)" >&2; exit 2; fi; }
for v in OR1_PREREG_DIR OR1_ENGINE_DIR OR1_NATIVE_BIN OR1_OUT_ROOT; do need "$v"; done
if [ ! -f "$OR1" ]; then echo "run.sh: run it from the repository root ($OR1 not found)" >&2; exit 2; fi
DRY=0
case "${1:-}" in
  --dry-run) DRY=1 ;;
  '') ;;
  *) echo "run.sh: unknown argument $1 (only --dry-run)" >&2; exit 2 ;;
esac
case "$OR1_OUT_ROOT" in build/*) ;; *) echo "run.sh: OR1_OUT_ROOT must be a path under build/" >&2; exit 2 ;; esac
LOGDIR="${OR1_LOG_DIR:-${OR1_OUT_ROOT}.logs}"
if [ "$DRY" = 1 ]; then LOG="$LOGDIR/run.sh.dry-run.$(date +%Y%m%dT%H%M%S).$$.log"; else LOG="$LOGDIR/run.sh.log"; fi

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

PREREG="$OR1_PREREG_DIR"
MANIFEST="$PREREG/$GATE_ID.v$GATE_VERSION.manifest.json"
GATEMD="$PREREG/$GATE_ID.v$GATE_VERSION.md"
step "1 identity"
echo "command: $0 $*"
echo "cwd: $PWD  user: $(id -un)  host: $(hostname)"
echo "gate: $GATE_ID v$GATE_VERSION  prereg: $PREREG  engine: $OR1_ENGINE_DIR  native: $OR1_NATIVE_BIN  out: $OR1_OUT_ROOT  dry-run: $DRY"
echo "node: $(node --version) $(sha256sum "$(readlink -f "$(command -v node)")")"
echo "run.sh: $(sha256sum "$0")"
for f in "$MANIFEST" "$GATEMD"; do if [ -f "$f" ]; then sha256sum "$f"; else echo "MISSING: $f"; fi; done
echo "environment variable names (values are not printed): $(env | cut -d= -f1 | grep -E '^(SLAM_REFACT_|OR1_|PRECHECK_|NODE_)' | sort | tr '\n' ' ')"
if [ -n "${!DESIGNATION_ENV:-}" ]; then echo "$DESIGNATION_ENV: set, value sha256 $(printf %s "${!DESIGNATION_ENV}" | sha256sum | cut -d' ' -f1)"; else echo "$DESIGNATION_ENV: NOT SET (the run will be HELD)"; fi
echo "git: $(git rev-parse HEAD 2>/dev/null || echo unavailable) on $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unavailable)"; git status --porcelain -- slam_refact | head -20 | sed 's/^/git status (slam_refact): /'

step "2 lane precheck"
if [ -f "$PRECHECK" ]; then logged bash "$PRECHECK" || { echo "run.sh: lane precheck did not pass; nothing was run"; exit 1; }; else echo "run.sh: $PRECHECK not found; nothing was run"; exit 1; fi

# the registered bundles: name and derived directory, in the manifest's order
BUNDLES=$(python3 -I - "$MANIFEST" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
for d in m['pinned']['gate_data']:
    print(d['name'], d['derived_dir'])
PY
) || { echo "run.sh: cannot read the registered bundles from $MANIFEST"; exit 2; }
echo "registered bundles:"; echo "$BUNDLES" | sed 's/^/  /'

gate_args=(--prereg-dir "$PREREG" --gate-id "$GATE_ID" --gate-version "$GATE_VERSION" --designation-env "$DESIGNATION_ENV")
bundle_args() { echo --bundle "$1" --engine-dir "$OR1_ENGINE_DIR" --native-bin "$OR1_NATIVE_BIN" --arms WCN --reps 3 --kind gate; }

step "3 precheck of every bundle (no marker)"
while read -r name dir; do
  # shellcheck disable=SC2046
  logged node "$OR1" precheck "${gate_args[@]}" $(bundle_args "$dir") || { echo "run.sh: HELD or failed for $name; no marker was created, nothing was run"; exit 3; }
done <<< "$BUNDLES"
if [ "$DRY" = 1 ]; then echo "dry run: stopping before the first real run (no marker, no output)"; exit 0; fi

step "4 the runs (one process per bundle, once)"
REPORTS=""
while read -r name dir; do
  # shellcheck disable=SC2046
  logged node "$OR1" run "${gate_args[@]}" $(bundle_args "$dir") --out "$OR1_OUT_ROOT/$name" --run-log "$LOGDIR/$name.run.log"
  rc=$?
  echo "bundle $name exit code $rc"
  if [ "$rc" -ne 0 ]; then echo "run.sh: stopped at $name (exit $rc); bundles after it were not started"; exit "$rc"; fi
  REPORTS="${REPORTS:+$REPORTS,}$OR1_OUT_ROOT/$name/report.json"
done <<< "$BUNDLES"

step "5 sums and the gate verdict"
for f in "$OR1_OUT_ROOT"/*/SHA256SUMS; do sha256sum "$f"; done
logged node "$OR1" verdict --reports "$REPORTS" --manifest "$MANIFEST"
vrc=$?
echo "gate verdict exit code $vrc (0 PASS, 1 FAIL, 4 INSUFFICIENT / UNEVALUATED)"
echo "next: the coordinator records first_real_run_at in the manifest, then: node $OR1 audit --prereg-dir $PREREG --gate-id $GATE_ID --gate-version $GATE_VERSION"
exit "$vrc"
