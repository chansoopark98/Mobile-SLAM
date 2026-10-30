#!/usr/bin/env bash
# rm4c-chunk-gate.v5: the registered command (gate document, first real run command): the two kinds of gate data, roi then it, one headless Chromium each, once.
#
# NOT RUN by the lane that wrote it (bash -n and --dry-run only: --dry-run stops after the prechecks, creates no marker, starts no browser). The coordinator runs it
# from the repository root after the freeze, the independent review and the designation:
#
#   RM4C_PREREG_DIR=<preregistration dir> RM4C_ENGINE_DIR=<pinned engine dir> RM4C_OUT_ROOT=<NEW dir under build/> \
#   RM4C_CHUNK_SHA256=<sha256 of xr-slam.js from the freeze record> SLAM_REFACT_RM4C_V5_DESIGNATION_SHA256=<sha256 of the designation file> \
#     bash slam_refact/tools/gates/rm4c-chunk-v5/run.sh [--dry-run]
#
# Optional: RM4C_GATE_ID (rm4c-chunk-gate), RM4C_GATE_VERSION (5), RM4C_DESIGNATION_ENV (the NAME of the variable above), RM4C_LOG_DIR (default <OUT_ROOT>.logs),
# RM4C_MUTATION_EVIDENCE (space separated summary files of the mutation runs, recorded with their sha256 in meta.json p3), RM4C_CHROMIUM_LOCK (a lock file: the
# browser runs are wrapped in `flock`), RM4C_PRECHECK (default slam_refact/docs/design/tools/lane-precheck.sh; this script overrides none of its PRECHECK_* values).
# The sequence length and, when the manifest REGISTERS a seed, the seed are read from the manifest's registered_run (a held coordinator seed is not passed at all: the
# claim returns it); the hold compares them again.
#
# RM4C_CHUNK_SHA256 is the pin INPUT for the current chunk bytes (the freeze record's value, e.g. the sha256 of xr-slam.js): the run is refused (HELD: args) when the
# live xr-slam.js differs. The value is never in the code. The manifest's own pins (every chunk file, device, tests, engine, environment) are checked by the hold.
#
# Order and records (everything goes through tee into $RM4C_LOG_DIR/run.sh.log, which is made read-only and summed when it ends):
#   1 identity: command, sha256 of this script / node / the manifest / the gate file, the NAMES of the steering variables, the sha256 of the designation value
#   2 lane-precheck.sh (headless Chromium count is checked BEFORE the run; the browser of the run starts after it)
#   3 run-chunk-gl.mjs --mode roi --kind gate  (its hold records every check in the run log; HELD ends the script, no marker)
#   4 run-chunk-gl.mjs --mode it  --kind gate  (the sequence length, and the seed unless held)
#   5 sha256 of each output tree's SHA256SUMS; the exit code of each command is "## RC" in the log
#   (--dry-run runs the prechecks of both kinds: the whole hold, no marker)
# After it: the coordinator records first_real_run_at in the manifest and writes the result document (P3: meta.json p3 of each run, the mutation summaries, UNVERIFIED /
# BLOCKED table). The run is a desktop headless Chromium + SwiftShader measurement; it is not a device measurement.
set -u -o pipefail

GATE_ID="${RM4C_GATE_ID:-rm4c-chunk-gate}"
GATE_VERSION="${RM4C_GATE_VERSION:-5}"
DESIGNATION_ENV="${RM4C_DESIGNATION_ENV:-SLAM_REFACT_RM4C_V5_DESIGNATION_SHA256}"
PRECHECK="${RM4C_PRECHECK:-slam_refact/docs/design/tools/lane-precheck.sh}"
GL=slam_refact/web-chunk/tests/run-chunk-gl.mjs

need() { if [ -z "${!1:-}" ]; then echo "run.sh: $1 is required (see the header)" >&2; exit 2; fi; }
for v in RM4C_PREREG_DIR RM4C_ENGINE_DIR RM4C_OUT_ROOT RM4C_CHUNK_SHA256; do need "$v"; done
if [ ! -f "$GL" ]; then echo "run.sh: run it from the repository root ($GL not found)" >&2; exit 2; fi
DRY=0
case "${1:-}" in --dry-run) DRY=1 ;; '') ;; *) echo "run.sh: unknown argument $1 (only --dry-run)" >&2; exit 2 ;; esac
case "$RM4C_OUT_ROOT" in build/*) ;; *) echo "run.sh: RM4C_OUT_ROOT must be a path under build/" >&2; exit 2 ;; esac
LOGDIR="${RM4C_LOG_DIR:-${RM4C_OUT_ROOT}.logs}"
if [ "$DRY" = 1 ]; then LOG="$LOGDIR/run.sh.dry-run.$(date +%Y%m%dT%H%M%S).$$.log"; else LOG="$LOGDIR/run.sh.log"; fi

if [ -z "${RM4C_RUN_INNER:-}" ]; then
  mkdir -p "$LOGDIR" || exit 2
  set -o noclobber
  : > "$LOG" || { echo "run.sh: $LOG exists: a gate log is never reused" >&2; exit 2; }
  set +o noclobber
  RM4C_RUN_INNER=1 bash "$0" "$@" 2>&1 | tee -a "$LOG"
  rc=${PIPESTATUS[0]}
  chmod a-w "$LOG"
  sha256sum "$LOG" > "$LOG.sha256"
  echo "log: $LOG (exit code of the run: $rc, sha256 in $LOG.sha256)"
  exit "$rc"
fi

trap 'echo "## EXIT $? at $(date -Is)"' EXIT
step() { echo "## $(date -Is) STEP $*"; }
logged() { echo "## CMD: $*"; "$@" </dev/null; local rc=$?; echo "## RC $rc: ${1:-} ${2:-} ${3:-}"; return $rc; }
lock_wrap=(); [ -n "${RM4C_CHROMIUM_LOCK:-}" ] && lock_wrap=(flock "$RM4C_CHROMIUM_LOCK")

PREREG="$RM4C_PREREG_DIR"
MANIFEST="$PREREG/$GATE_ID.v$GATE_VERSION.manifest.json"
GATEMD="$PREREG/$GATE_ID.v$GATE_VERSION.md"
step "1 identity"
echo "command: $0 $*"
echo "cwd: $PWD  user: $(id -un)  host: $(hostname)  dry-run: $DRY"
echo "gate: $GATE_ID v$GATE_VERSION  prereg: $PREREG  engine: $RM4C_ENGINE_DIR  out: $RM4C_OUT_ROOT  expected xr-slam.js sha256: $RM4C_CHUNK_SHA256"
echo "node: $(node --version) $(sha256sum "$(readlink -f "$(command -v node)")")"
echo "run.sh: $(sha256sum "$0")"
for f in "$MANIFEST" "$GATEMD"; do if [ -f "$f" ]; then sha256sum "$f"; else echo "MISSING: $f"; fi; done
echo "environment variable names (values are not printed): $(env | cut -d= -f1 | grep -E '^(SLAM_REFACT_|RM4C_|PRECHECK_|BROWSER_|NODE_|PLAYWRIGHT_)' | sort | tr '\n' ' ')"
if [ -n "${!DESIGNATION_ENV:-}" ]; then echo "$DESIGNATION_ENV: set, value sha256 $(printf %s "${!DESIGNATION_ENV}" | sha256sum | cut -d' ' -f1)"; else echo "$DESIGNATION_ENV: NOT SET (the run will be HELD)"; fi
echo "git: $(git rev-parse HEAD 2>/dev/null || echo unavailable) on $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unavailable)"; git status --porcelain -- slam_refact | head -20 | sed 's/^/git status (slam_refact): /'
echo "xr-slam.js live: $(sha256sum slam_refact/web-chunk/xr-slam.js)"

step "2 lane precheck (before the browser of the run starts)"
if [ -f "$PRECHECK" ]; then logged bash "$PRECHECK" || { echo "run.sh: lane precheck did not pass; nothing was run"; exit 1; }; else echo "run.sh: $PRECHECK not found; nothing was run"; exit 1; fi

gate_args=(--prereg-dir "$PREREG" --gate-id "$GATE_ID" --gate-version "$GATE_VERSION" --designation-env "$DESIGNATION_ENV" --engine-dir "$RM4C_ENGINE_DIR"
  --expect-chunk-sha256 "xr-slam.js=$RM4C_CHUNK_SHA256")
for f in ${RM4C_MUTATION_EVIDENCE:-}; do gate_args+=(--mutation-evidence "$f"); done
# the registered sequence: its length and (if registered) its seed come from the manifest
read -r FRAMES REG_SEED < <(python3 -I - "$MANIFEST" <<'PY'
import json, sys
r = json.load(open(sys.argv[1])).get('registered_run', {})
print(r.get('frames', ''), r.get('seed') if r.get('seed') is not None else '-')
PY
) || { echo "run.sh: cannot read registered_run from $MANIFEST; nothing was run"; exit 2; }
if [ -z "$FRAMES" ]; then echo "run.sh: $MANIFEST has no registered_run.frames; nothing was run"; exit 2; fi
seed_args=(); [ "$REG_SEED" != "-" ] && seed_args=(--seed "$REG_SEED")
echo "registered sequence: $FRAMES frames, seed: $([ "$REG_SEED" = "-" ] && echo 'held by the coordinator' || echo "$REG_SEED")"

if [ "$DRY" = 1 ]; then
  step "2b precheck of the hold for both kinds (no marker, no browser, no output)"
  logged node "$GL" --mode roi --kind gate --precheck "${gate_args[@]}" --out "$RM4C_OUT_ROOT/roi" || { echo "run.sh: HELD or failed (roi); nothing was run"; exit 3; }
  logged node "$GL" --mode it --kind gate --precheck --frames "$FRAMES" "${seed_args[@]}" "${gate_args[@]}" --out "$RM4C_OUT_ROOT/it" || { echo "run.sh: HELD or failed (it); nothing was run"; exit 3; }
  echo "dry run: stopping before the first run (no marker, no browser, no output)"; exit 0
fi

step "3 roi gate data"
logged "${lock_wrap[@]}" node "$GL" --mode roi --kind gate "${gate_args[@]}" --out "$RM4C_OUT_ROOT/roi" --run-log "$LOGDIR/roi.run.log"
rc=$?
if [ "$rc" -ne 0 ]; then echo "run.sh: roi exit $rc; the it run was not started"; exit "$rc"; fi

step "4 it gate data (the registered sequence)"
logged "${lock_wrap[@]}" node "$GL" --mode it --kind gate --frames "$FRAMES" "${seed_args[@]}" "${gate_args[@]}" --out "$RM4C_OUT_ROOT/it" --run-log "$LOGDIR/it.run.log"
rc=$?

step "5 sums"
for f in "$RM4C_OUT_ROOT"/*/SHA256SUMS; do [ -f "$f" ] && sha256sum "$f"; done
echo "next: record first_real_run_at in the manifest; the result document needs meta.json (p3, hold.deviceLog, pinsAtRun) of both runs and the mutation summaries"
exit "$rc"
