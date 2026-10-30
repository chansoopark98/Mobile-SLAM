#!/usr/bin/env bash
# us010-samples v4: the registered run sequence AND the replay driver (this file is pinned by `us010-v4.mjs freeze` with its sha256, and every replay run records the
# sha256 of this file: v3 ran its replay loop from an unpinned scratchpad script, post-run review U10-5).
#
# NOT RUN by the lane that wrote it (bash -n and --dry-run only). The coordinator runs it from the repository root after the freeze. Grade: REPORT unless the
# registration names an independent oracle for the world-lock criterion (README.md); this script does not decide the grade.
#
#   US010_EVIDENCE_ROOT=<NEW dir under build/> US010_ENGINE_DIR=<pinned engine dir> US010_PINS=<pin list written by `us010-v4.mjs freeze`> \
#   US010_MANIFEST=<us010-samples.v4.manifest.json> US010_CAPTURE=<label of the capture under the evidence root: capture-<label>/> \
#     bash slam_refact/tools/gates/us010-samples-v4/run.sh [--dry-run]
#
# Optional: US010_REG_CPUS (CPU list for the smoke and control runs; default none, as v3 ran them unpinned), US010_CPUS (series and replay; default 0-15, the P cores),
# US010_ROUNDS (5), US010_CHROMIUM_LOCK (a lock file: every browser run is wrapped in `flock`), US010_PRECHECK (default slam_refact/docs/design/tools/lane-precheck.sh).
#
# Order and what is checked (all output goes through tee into $US010_EVIDENCE_ROOT.logs/run.sh.log; read-only and summed when it ends):
#   0 identity (command, sha256 of this script, node, git); `us010-v4.mjs check-manifest` (mutation.detected true, null-run PASS, no run on record: refused otherwise);
#     `us010-v4.mjs verify` of the pin list (core files, external bytes, nothing added); lane-precheck
#   1 registered smoke runs, three.js and A-Frame, label reg
#   2 negative controls N0 M1..M6 for both samples (--expectations v4: M6 is the v3 observation registered for v4)
#   3 the ABAB timing series (run-us010b-series.mjs, five rounds of S / Sx / N / F)
#   4 the replay series: rounds x {idle, sync, worker} on the pinned capture, this file named with --driver
#   5 the summary (summarize-us010b.mjs, written once) and a last `verify`
# `verify` runs again before steps 2, 3, 4 and at the end: a pinned byte that changed stops the sequence. The exit code of every command is in the log ("## RC").
# With --dry-run only step 0 runs (no browser, nothing under the evidence root except the log).
set -u -o pipefail

need() { if [ -z "${!1:-}" ]; then echo "run.sh: $1 is required (see the header)" >&2; exit 2; fi; }
for v in US010_EVIDENCE_ROOT US010_ENGINE_DIR US010_PINS US010_MANIFEST US010_CAPTURE; do need "$v"; done
if [ ! -f slam_refact/web-chunk/tests/us010-v4.mjs ]; then echo "run.sh: run it from the repository root" >&2; exit 2; fi
DRY=0
case "${1:-}" in --dry-run) DRY=1 ;; '') ;; *) echo "run.sh: unknown argument $1 (only --dry-run)" >&2; exit 2 ;; esac
case "$US010_EVIDENCE_ROOT" in build/*) ;; *) echo "run.sh: US010_EVIDENCE_ROOT must be a path under build/" >&2; exit 2 ;; esac
LOGDIR="${US010_EVIDENCE_ROOT}.logs"
if [ "$DRY" = 1 ]; then LOG="$LOGDIR/run.sh.dry-run.$(date +%Y%m%dT%H%M%S).$$.log"; else LOG="$LOGDIR/run.sh.log"; fi

if [ -z "${US010_RUN_INNER:-}" ]; then
  mkdir -p "$LOGDIR" || exit 2
  set -o noclobber
  : > "$LOG" || { echo "run.sh: $LOG exists: a gate log is never reused" >&2; exit 2; }
  set +o noclobber
  US010_RUN_INNER=1 bash "$0" "$@" 2>&1 | tee -a "$LOG"
  rc=${PIPESTATUS[0]}
  chmod a-w "$LOG"
  sha256sum "$LOG" > "$LOG.sha256"
  echo "log: $LOG (exit code of the run: $rc, sha256 in $LOG.sha256)"
  exit "$rc"
fi

trap 'echo "## EXIT $? at $(date -Is)"' EXIT
step() { echo "## $(date -Is) STEP $*"; }
logged() { echo "## CMD: $*"; "$@" </dev/null; local rc=$?; echo "## RC $rc: ${1:-} ${2:-} ${3:-}"; return $rc; }

EV="$US010_EVIDENCE_ROOT"
ENG="$US010_ENGINE_DIR"
DRIVER=slam_refact/tools/gates/us010-samples-v4/run.sh
V4=slam_refact/web-chunk/tests/us010-v4.mjs
T=slam_refact/web-chunk/tests
ROUNDS="${US010_ROUNDS:-5}"
CPUS="${US010_CPUS:-0-15}"
PRECHECK="${US010_PRECHECK:-slam_refact/docs/design/tools/lane-precheck.sh}"
reg_wrap=(); [ -n "${US010_REG_CPUS:-}" ] && reg_wrap=(taskset -c "$US010_REG_CPUS")
cpu_wrap=(taskset -c "$CPUS")
lock_wrap=(); [ -n "${US010_CHROMIUM_LOCK:-}" ] && lock_wrap=(flock "$US010_CHROMIUM_LOCK")
verify() { logged node "$V4" verify --pins "$US010_PINS" --engine-dir "$ENG" || { echo "run.sh: the pinned core or external bytes changed ($1); stopping before the next run"; exit 3; }; }

step "0 identity and prechecks"
echo "command: $0 $*"
echo "cwd: $PWD  user: $(id -un)  host: $(hostname)  dry-run: $DRY"
echo "evidence root: $EV  engine: $ENG  pins: $US010_PINS  manifest: $US010_MANIFEST  capture: capture-$US010_CAPTURE"
echo "node: $(node --version) $(sha256sum "$(readlink -f "$(command -v node)")")"
echo "driver (this file): $(sha256sum "$0")"
sha256sum "$US010_MANIFEST" "$US010_PINS" 2>&1
echo "environment variable names (values are not printed): $(env | cut -d= -f1 | grep -E '^(SLAM_REFACT_|US010_|PRECHECK_|BROWSER_|NODE_|PLAYWRIGHT_)' | sort | tr '\n' ' ')"
echo "git: $(git rev-parse HEAD 2>/dev/null || echo unavailable) on $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unavailable)"; git status --porcelain -- slam_refact | head -20 | sed 's/^/git status (slam_refact): /'
logged node "$V4" check-manifest --manifest "$US010_MANIFEST" || { echo "run.sh: the manifest does not allow a first run (mutation.detected must be true, null_run PASS); nothing was run"; exit 3; }
verify "start"
if [ -f "$PRECHECK" ]; then logged bash "$PRECHECK" || { echo "run.sh: lane precheck did not pass; nothing was run"; exit 1; }; else echo "run.sh: $PRECHECK not found; nothing was run"; exit 1; fi
if [ "$DRY" = 1 ]; then echo "dry run: stopping before the first run (no browser, no output)"; exit 0; fi
if [ -e "$EV" ]; then echo "run.sh: $EV exists: the evidence root of a registered run is new" >&2; exit 2; fi
mkdir -p "$EV/runs" || exit 2
echo "(the capture must already exist as $EV/capture-$US010_CAPTURE: it is an input pinned through --extra; see README.md)"
if [ ! -d "$EV/capture-$US010_CAPTURE" ]; then echo "run.sh: $EV/capture-$US010_CAPTURE is missing"; exit 2; fi

smoke() {  # $1 sample $2 label [control...]
  local sample="$1" label="$2"; shift 2
  logged "${lock_wrap[@]}" "${reg_wrap[@]}" node "$T/run-sample-smoke.mjs" --sample "$sample" --label "$label" --expectations v4 --out-root "$EV/runs" --owned-root "$EV" --engine-dir "$ENG" --driver "$DRIVER" "$@"
}

step "1 registered smoke runs"
for s in threejs aframe; do smoke "$s" reg; done
verify "after step 1"

step "2 negative controls"
for s in threejs aframe; do for c in N0 M1 M2 M3 M4 M5 M6; do smoke "$s" "reg-$(echo "$c" | tr 'A-Z' 'a-z')" --control "$c"; done; done
verify "after step 2"

step "3 the ABAB timing series (children use the pinned CPU list)"
logged "${lock_wrap[@]}" node "$T/run-us010b-series.mjs" --name t1 --rounds "$ROUNDS" --cpus "$CPUS" --evidence-root "$EV" --series-dir series --engine-dir "$ENG" --expectations v4
verify "after step 3"

step "4 the replay series (driver: this file)"
for i in $(seq 1 "$ROUNDS"); do for m in idle sync worker; do
  logged "${lock_wrap[@]}" "${cpu_wrap[@]}" node "$T/run-exec-model.mjs" --phase replay --capture "$US010_CAPTURE" --mode "$m" --label "rep$i-$m" --evidence-root "$EV" --engine-dir "$ENG" --driver "$DRIVER"
done; done
verify "after step 4"

step "5 summary (written once) and the last verify"
logged node "$T/summarize-us010b.mjs" --series t1 --replay-prefix rep --rounds "$ROUNDS" --evidence-root "$EV" --series-dir series --summary-name exec-model-summary-v4.json
verify "end"
echo "done: every command's exit code is in this log (## RC); the results are in $EV (runs/, series/, replay/, exec-model-summary-v4.json)"
exit 0
