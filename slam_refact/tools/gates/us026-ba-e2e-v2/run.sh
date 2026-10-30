#!/usr/bin/env bash
# us026-ba-e2e-gate.v2: the registered command (gate document section 11, "first real run command").
# FORK of slam_refact/tools/gates/us026-ba-e2e-v1/run.sh (sha256 3f8617d1...81a2). Differences from v1 (gate document section 2.3):
#   R1  GATE_VERSION 1 -> 2, DESIGNATION_ENV SLAM_REFACT_US026_BA_V1_DESIGNATION_SHA256 -> SLAM_REFACT_US026_BA_V2_DESIGNATION_SHA256
#   R2  TOOL is the v2 runner slam_refact/tools/gates/us026-ba-e2e-v2/gate_e2e.mjs (version 2, pinned v2 replay binary, US026B-M1 rule in code)
#   R3  default OUT_ROOT build/slam-refact-full-20261019/us026/gate-runs/us026-ba-e2e-gate-v2 (the v1 output tree gate-runs/us026-ba-e2e-gate is untouched);
#       the scenario output is $OUT_ROOT/us026-ba-e2e-gate/<scenario>, the logs $OUT_ROOT/us026-ba-e2e-gate.logs
#   R4  step 1 also records the python3 / node / chrome identities and the freeze-path git status (R4-12 of the v1 post-run review); a REAL run (not
#       --dry-run) is refused (exit 2, before any marker) when that git status is not empty: the v1 condition US026B-m5 as code
# In the usage text below read v2 for v1 (the script path us026-ba-e2e-v2/run.sh, the variable SLAM_REFACT_US026_BA_V2_DESIGNATION_SHA256).
# Everything else is the v1 text.
#
# Original v1 header follows.
# us026-ba-e2e-gate.v1: the registered command (gate document section "first real run command").
#
# NOT RUN by the lane that wrote it on any registration scenario. Lane check: `bash -n`, `--dry-run` with no designation (stops HELD at step 3) and the
# end-to-end dev mode of gate_e2e.mjs on the dev scenario (not this script). The coordinator runs it, from the repository root, after the
# independent review and the designation:
#
#   US026_PREREG_DIR=<preregistration dir> SLAM_REFACT_US026_BA_V1_DESIGNATION_SHA256=<sha256 of the designation file> \
#     bash slam_refact/tools/gates/us026-ba-e2e-v1/run.sh [--dry-run]
#
# Optional: US026_OUT_ROOT (default build/slam-refact-full-20261019/us026/gate-runs; must not contain <gate id>/<scenario> yet), US026_LOG_DIR
# (default <OUT_ROOT>/<gate id>.logs). The PRECHECK_* thresholds are not taken from the caller: gate_e2e.mjs runs lane-precheck.sh with
# PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0 and the defaults for the rest; step 2 below does the same.
#
# What it does, in this order (everything goes through tee into $US026_LOG_DIR/run.sh.log, made read-only and summed at the end):
#   1 the command line, the date, the sha256 of this script / node / the manifest / the gate file, the NAMES of the steering environment
#     variables and the sha256 of the designation variable's value (never the value of any other)
#   2 lane-precheck.sh (resources; read only; informational here, gate_e2e.mjs repeats it before the marker and inside the Chromium lock)
#   3 `gate_e2e.mjs precheck` for every registered scenario: every HELD check, no marker. Any refusal ends the script before a marker exists.
#   4 (not with --dry-run) `gate_e2e.mjs run --kind gate` for the scenarios in the manifest's order, one process each. A process waits for
#     resources (precheck + 1-min loadavg <= 12, never relaxed) BEFORE its marker; then marker (kind = scenario), output directory, the two
#     original runs (each under flock build/c1-chromium.lock, resources checked again inside the lock), bundles, replays, compare, report,
#     SHA256SUMS, read-only. The first non-zero exit stops the script (exit 5 = resources never ready: no marker; exit 6 = infrastructure
#     failure after the marker: that scenario's run is used, coordinator decision + new version; scenarios after it are not started).
#   5 sha256 of each output tree's SHA256SUMS, then `gate_e2e.mjs verdict` (exit 0 PASS, 1 FAIL, 4 BLOCKED / UNEVALUATED).
# After it: the coordinator records first_real_run_at (gate document §11), then `node slam_refact/tools/harness/us026/gate_e2e.mjs audit ...`.
# Desktop headless Chromium (SwiftShader) and recorded inputs only; ephemeral 127.0.0.1 server inside run-reference.mjs; nothing under web/.
set -u -o pipefail

GATE_KEY=ba
GATE_ID=us026-ba-e2e-gate
GATE_VERSION=2
DESIGNATION_ENV=SLAM_REFACT_US026_BA_V2_DESIGNATION_SHA256
PRECHECK=slam_refact/docs/design/tools/lane-precheck.sh
TOOL=slam_refact/tools/gates/us026-ba-e2e-v2/gate_e2e.mjs

if [ -z "${US026_PREREG_DIR:-}" ]; then echo "run.sh: US026_PREREG_DIR is required (see the header)" >&2; exit 2; fi
if [ ! -f "$TOOL" ]; then echo "run.sh: run it from the repository root ($TOOL not found)" >&2; exit 2; fi
DRY=0
case "${1:-}" in
  --dry-run) DRY=1 ;;
  '') ;;
  *) echo "run.sh: unknown argument $1 (only --dry-run)" >&2; exit 2 ;;
esac
OUT_ROOT="${US026_OUT_ROOT:-build/slam-refact-full-20261019/us026/gate-runs/us026-ba-e2e-gate-v2}"
case "$OUT_ROOT" in build/slam-refact-full-20261019/*) ;; *) echo "run.sh: US026_OUT_ROOT must be under build/slam-refact-full-20261019/ (run-reference.mjs writes only there)" >&2; exit 2 ;; esac
LOGDIR="${US026_LOG_DIR:-$OUT_ROOT/$GATE_ID.logs}"
if [ "$DRY" = 1 ]; then LOG="$LOGDIR/run.sh.dry-run.$(date +%Y%m%dT%H%M%S).$$.log"; else LOG="$LOGDIR/run.sh.log"; fi

if [ -z "${US026_RUN_INNER:-}" ]; then
  mkdir -p "$LOGDIR" || exit 2
  set -o noclobber
  : > "$LOG" || { echo "run.sh: $LOG exists: a gate log is never reused" >&2; exit 2; }
  set +o noclobber
  US026_RUN_INNER=1 bash "$0" "$@" 2>&1 | tee -a "$LOG"
  rc=${PIPESTATUS[0]}
  chmod a-w "$LOG"
  sha256sum "$LOG" > "$LOG.sha256"
  echo "log: $LOG (exit code of the run: $rc, sha256 in $LOG.sha256)"
  exit "$rc"
fi

trap 'echo "## EXIT $? at $(date -Is)"' EXIT
step() { echo "## $(date -Is) STEP $*"; }
logged() { echo "## CMD: $*"; "$@" </dev/null; local rc=$?; echo "## RC $rc: ${1:-} ${2:-} ${3:-}"; return $rc; }

PREREG="$US026_PREREG_DIR"
MANIFEST="$PREREG/$GATE_ID.v$GATE_VERSION.manifest.json"
GATEMD="$PREREG/$GATE_ID.v$GATE_VERSION.md"
step "1 identity"
echo "command: $0 $*"
echo "cwd: $PWD  user: $(id -un)  host: $(hostname)"
echo "gate: $GATE_ID v$GATE_VERSION  prereg: $PREREG  out: $OUT_ROOT  logs: $LOGDIR  dry-run: $DRY"
echo "node: $(node --version) $(sha256sum "$(readlink -f "$(command -v node)")")"
echo "run.sh: $(sha256sum "$0")"
for f in "$MANIFEST" "$GATEMD"; do if [ -f "$f" ]; then sha256sum "$f"; else echo "MISSING: $f"; fi; done
echo "environment variable names (values are not printed): $(env | cut -d= -f1 | grep -E '^(SLAM_REFACT_|US026_|PRECHECK_|NODE_|BROWSER_|PLAYWRIGHT_)' | sort | tr '\n' ' ')"
if [ -n "${!DESIGNATION_ENV:-}" ]; then echo "$DESIGNATION_ENV: set, value sha256 $(printf %s "${!DESIGNATION_ENV}" | sha256sum | cut -d' ' -f1)"; else echo "$DESIGNATION_ENV: NOT SET (the run will be HELD)"; fi
echo "load: $(cat /proc/loadavg)"
echo "python3: $(command -v python3) $(python3 --version 2>&1)"; echo "node: $(readlink -f "$(command -v node)")"; echo "chrome: $(/usr/bin/google-chrome --version 2>&1)"
echo "git: $(git rev-parse HEAD 2>/dev/null || echo unavailable) on $(git rev-parse --abbrev-ref HEAD 2>/dev/null || echo unavailable)"; git status --porcelain -- slam_refact | head -20 | sed 's/^/git status (slam_refact): /'
FREEZE_DIRTY=$(git status --porcelain -- slam_refact/engine slam_refact/web-chunk slam_refact/web-smoke slam_refact/tools wasm src 2>&1)
if [ -n "$FREEZE_DIRTY" ]; then
  echo "git status (freeze paths slam_refact/{engine,web-chunk,web-smoke,tools} wasm src) is NOT empty:"; echo "$FREEZE_DIRTY" | sed 's/^/  /'
  if [ "$DRY" = 0 ]; then echo "run.sh: a real run needs an empty freeze-path git status (v1 condition US026B-m5; commit or restore first); nothing was run"; exit 2; fi
else echo "git status (freeze paths): empty"; fi

step "2 lane precheck (informational; repeated by gate_e2e.mjs before each marker and inside the lock)"
if [ -f "$PRECHECK" ]; then
  env $(env | cut -d= -f1 | grep '^PRECHECK_' | sed 's/^/-u /') PRECHECK_MAX_HEADLESS=0 PRECHECK_MAX_BUILDS=0 bash "$PRECHECK" || echo "run.sh: lane precheck does not pass now; gate_e2e.mjs waits for it before each marker (thresholds unchanged)"
else echo "run.sh: $PRECHECK not found; nothing was run"; exit 1; fi

SCENARIOS=$(python3 -I - "$MANIFEST" <<'PY'
import json, sys
m = json.load(open(sys.argv[1]))
for d in m['registered_run']['scenarios']:
    print(d['name'])
PY
) || { echo "run.sh: cannot read the registered scenarios from $MANIFEST"; exit 2; }
echo "registered scenarios:"; echo "$SCENARIOS" | sed 's/^/  /'
gate_args=(--gate "$GATE_KEY" --prereg-dir "$PREREG" --gate-id "$GATE_ID" --gate-version "$GATE_VERSION" --designation-env "$DESIGNATION_ENV" --out-root "$OUT_ROOT")

step "3 precheck of every scenario (no marker)"
while read -r s; do
  logged node "$TOOL" precheck "${gate_args[@]}" --scenario "$s" || { echo "run.sh: HELD or failed for $s; no marker was created, nothing was run"; exit 3; }
done <<< "$SCENARIOS"
if [ "$DRY" = 1 ]; then echo "dry run: stopping before the first real run (no marker, no output)"; exit 0; fi

step "4 the runs (one process per scenario, once)"
REPORTS=""
while read -r s; do
  logged node "$TOOL" run "${gate_args[@]}" --scenario "$s" --kind gate --run-log "$LOGDIR/$s.run.log"
  rc=$?
  echo "scenario $s exit code $rc"
  if [ "$rc" -ne 0 ]; then echo "run.sh: stopped at $s (exit $rc); scenarios after it were not started"; exit "$rc"; fi
  REPORTS="${REPORTS:+$REPORTS,}$OUT_ROOT/$GATE_ID/$s/report.json"
done <<< "$SCENARIOS"

step "5 sums and the gate verdict"
for f in "$OUT_ROOT/$GATE_ID"/*/SHA256SUMS; do sha256sum "$f"; done
logged node "$TOOL" verdict --gate "$GATE_KEY" --reports "$REPORTS" --manifest "$MANIFEST"
vrc=$?
echo "gate verdict exit code $vrc (0 PASS, 1 FAIL, 4 BLOCKED / UNEVALUATED)"
echo "next: the coordinator records first_real_run_at in the manifest, then: node $TOOL audit --prereg-dir $PREREG --gate-id $GATE_ID --gate-version $GATE_VERSION"
exit "$vrc"
