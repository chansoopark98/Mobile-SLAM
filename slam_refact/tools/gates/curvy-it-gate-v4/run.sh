#!/usr/bin/env bash
# FIRST REAL RUN of curvy-it-gate.v4 (once per data kind). Prepared by lane B3a from the v3 runner
# (build/slam-refact-full-20261019/it3/real-run/run.sh) after the v3 first-run review
# (docs/design/reviews/opus-xhigh-r1-curvy-v3-firstrun.md) and the closure audit (reviews/change-closure-audit-20261020.md B).
#
#   bash run.sh --prereg DIR --held-dir DIR --build-dir DIR --out DIR --designation SHA256 --seed-file FILE [--dry-run]
#
#   --prereg       preregistration directory (curvy-it-gate.v4.{md,manifest.json,review.md,designation file}, markers)
#   --held-dir     directory with the pinned copies, exactly the files of manifest held.pins:
#                  sr_tests_image_target_curvy_roi, sr_tests_image_target_curvy_synth, sr_curvy_held_check,
#                  slam_refact_engine.js, slam_refact_engine.wasm
#   --build-dir    the build directory the held copies were built in (the registered fixtures are rendered under
#                  <build-dir>/tests/image_target/image-target-curvy-registered-{cylinder,cone}, their compiled-in place)
#   --out          evidence directory (new or empty; no file of it is overwritten)
#   --designation  sha256 of the coordinator's designation file (the value of SLAM_REFACT_CURVY_DESIGNATION)
#   --seed-file    the coordinator seed file (the registered data is derived from it; its value is never printed)
#   --dry-run      validate the arguments and the files and print the plan; creates nothing, exports no designation, runs no
#                  gate binary (REPORT mode: nothing is selected, so nothing can be released or claimed)
#
# Order: render the registered fixtures (rendering only, no engine) -> hash/tree-sha precheck of every pinned file and every
# data directory with the node sr_held (side-effect free) -> roi GatesAB -> synth (one process, the filter of the gate md)
# -> WASM node test (REPORT). Differences from v3, each tied to the v3 first-run review:
#   F-4  the checksum list is written LAST, after every other output (the v3 list was made before the timeline's final line)
#   F-6  the WASM test keeps the checker's release log as a test diagnostic (image_target_curvy.test.mjs)
#   F-7  every inherited and every designated SLAM_REFACT_* variable is recorded (names and values; none is a secret), the
#        inherited ones are unset before anything is exported, and all are unset after the run
#   F-10 IT-P3 component maxima are printed by the synth test
#   tee  each step log carries, inside the tee, the binary path, its sha256 immediately before exec, the command and the exit code
# The gate id and the version are this runner's directory (curvy-it-gate-v4); the gate md decides the synth filter below.
# It is an accident guard, not an authentication (protocol 12.6).
set -u

GATE_ID=curvy-it-gate
GATE_VERSION=4
# gate md section 6 of curvy-it-gate.v3 (unchanged tolerances and tests; the lead checks it against the v4 md before the freeze)
SYNTH_FILTER='*GeometryAgainstPhysicalModel*:*TrajectoryAndKMatchRegistration*:*ImageTargetOnly*:*WorldOn*:*RoiLocalSuccessRatio*:CurvyCrop.*'
PINNED_FILES="sr_tests_image_target_curvy_roi sr_tests_image_target_curvy_synth sr_curvy_held_check slam_refact_engine.js slam_refact_engine.wasm"

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$here/../../../.." && pwd)"

usage() {
  echo "usage: bash run.sh --prereg DIR --held-dir DIR --build-dir DIR --out DIR --designation SHA256 --seed-file FILE [--dry-run]" >&2
  echo "(every option is required; there are no defaults; see the header of this file)" >&2
}

PRE="" HELD="" BUILD="" OUT="" DESIGNATION="" SEED_FILE="" DRY=0
while [ $# -gt 0 ]; do
  case "$1" in
    --prereg|--held-dir|--build-dir|--out|--designation|--seed-file)
      [ $# -ge 2 ] || { echo "missing value for $1" >&2; usage; exit 2; } ;;
  esac
  case "$1" in
    --prereg) PRE="${2:-}"; shift 2 ;;
    --held-dir) HELD="${2:-}"; shift 2 ;;
    --build-dir) BUILD="${2:-}"; shift 2 ;;
    --out) OUT="${2:-}"; shift 2 ;;
    --designation) DESIGNATION="${2:-}"; shift 2 ;;
    --seed-file) SEED_FILE="${2:-}"; shift 2 ;;
    --dry-run) DRY=1; shift ;;
    *) echo "unknown argument: $1" >&2; usage; exit 2 ;;
  esac
done
[ -n "$PRE" ] || { echo "missing --prereg (there are no defaults)" >&2; usage; exit 2; }
[ -n "$HELD" ] || { echo "missing --held-dir (there are no defaults)" >&2; usage; exit 2; }
[ -n "$BUILD" ] || { echo "missing --build-dir (there are no defaults)" >&2; usage; exit 2; }
[ -n "$OUT" ] || { echo "missing --out (there are no defaults)" >&2; usage; exit 2; }
[ -n "$DESIGNATION" ] || { echo "missing --designation (there are no defaults)" >&2; usage; exit 2; }
[ -n "$SEED_FILE" ] || { echo "missing --seed-file (there are no defaults)" >&2; usage; exit 2; }
case "$DESIGNATION" in
  *[!0-9a-f]*|"") echo "--designation is not a lowercase sha256" >&2; exit 2 ;;
esac
[ "${#DESIGNATION}" = 64 ] || { echo "--designation is not 64 hex characters" >&2; exit 2; }

MANIFEST="$PRE/$GATE_ID.v$GATE_VERSION.manifest.json"
FIX="$BUILD/tests/image_target"
problems=0
need() { [ -e "$1" ] || { echo "MISSING: $1" >&2; problems=$((problems + 1)); }; }
need "$MANIFEST"; need "$PRE/$GATE_ID.v$GATE_VERSION.md"; need "$SEED_FILE"; need "$FIX"
need "$REPO/slam_refact/tools/image_target/render_synthetic.py"; need "$REPO/slam_refact/tools/prereg/sr_held.mjs"
PINS=""
for f in $PINNED_FILES; do
  need "$HELD/$f"
  PINS="${PINS:+$PINS:}$HELD/$f"
done
for k in roi cylinder cone wasm-cylinder wasm-cone; do
  if [ -e "$PRE/$GATE_ID.v$GATE_VERSION.first-run.$k.marker" ]; then
    echo "ALREADY RUN: $PRE/$GATE_ID.v$GATE_VERSION.first-run.$k.marker exists" >&2
    problems=$((problems + 1))
  fi
done
if [ -e "$OUT" ] && [ -n "$(ls -A "$OUT" 2>/dev/null)" ]; then
  echo "REFUSING: --out $OUT exists and is not empty" >&2
  problems=$((problems + 1))
fi
[ "$problems" = 0 ] || { echo "precondition failures: $problems; nothing was run" >&2; exit 8; }

if [ "$DRY" = 1 ]; then
  echo "DRY RUN (REPORT mode: no designation exported, no gate binary run, nothing created)"
  echo "gate           $GATE_ID.v$GATE_VERSION"
  echo "repo           $REPO"
  echo "prereg         $PRE"
  echo "held dir       $HELD"
  echo "build dir      $BUILD (registered fixtures under $FIX)"
  echo "out            $OUT"
  echo "seed file      $SEED_FILE (sha256 $(sha256sum "$SEED_FILE" | cut -d' ' -f1); the value is not read)"
  echo "designation    $DESIGNATION (would be exported as SLAM_REFACT_CURVY_DESIGNATION)"
  echo "pins           $PINS"
  for f in $PINNED_FILES; do echo "  sha256 $(sha256sum "$HELD/$f" | cut -d' ' -f1)  $f"; done
  echo "synth filter   $SYNTH_FILTER"
  echo "plan           render fixtures -> precheck (node sr_held check, 4 kinds) -> roi -> synth -> wasm -> record -> checksums last"
  exit 0
fi

mkdir -p "$OUT" || exit 9
out() { [ ! -e "$OUT/$1" ] || { echo "REFUSING to overwrite $OUT/$1" >&2; exit 9; }; }
for f in timeline.txt environment.txt render.log precheck.log roi.log synth.log wasm.log SHA256SUMS; do out "$f"; done
T="$OUT/timeline.txt"
stamp() { echo "$*" >> "$T"; }
stamp "start $(date -Is)"

# --- F-7: record, then clean, then export only what the gate designates ---
{
  echo "== inherited SLAM_REFACT_* (names and values, before cleaning)"
  env | grep '^SLAM_REFACT_' | sort
  echo "== end inherited"
} > "$OUT/environment.txt"
for v in $(compgen -e | grep '^SLAM_REFACT_'); do unset "$v"; done
export SLAM_REFACT_CURVY_GATE_ID="$GATE_ID"
export SLAM_REFACT_CURVY_GATE_VERSION="$GATE_VERSION"
export SLAM_REFACT_CURVY_PREREG_DIR="$PRE"
export SLAM_REFACT_CURVY_DESIGNATION="$DESIGNATION"
export SLAM_REFACT_CURVY_PINS="$PINS"
export SLAM_REFACT_CURVY_SEED_FILE="$SEED_FILE"
export SLAM_REFACT_CURVY_DIR="$OUT/roi-work"
export SLAM_REFACT_IT_SYNTH_OUT="$OUT/synth-out"
mkdir -p "$SLAM_REFACT_CURVY_DIR" "$SLAM_REFACT_IT_SYNTH_OUT"
{
  echo "== designated SLAM_REFACT_* for this run (names and values; none is a secret; the seed file's VALUE is not recorded)"
  env | grep '^SLAM_REFACT_' | sort
  echo "== tools"
  uname -a
  python3 --version
  python3 -c 'import numpy; print("numpy", numpy.__version__)'
  node --version
  echo "== sha256 of the pinned files (held dir)"
  (cd "$HELD" && sha256sum $PINNED_FILES)
  echo "== sha256 of the registered files"
  (cd "$PRE" && sha256sum "$GATE_ID.v$GATE_VERSION".* 2>/dev/null)
  echo "== sha256 of the seed file"
  sha256sum "$SEED_FILE"
} >> "$OUT/environment.txt" 2>&1

# One step of the run. Everything below goes through the tee, so the step log is the complete record of that step:
# binary path, sha256 immediately before exec, the command, the exit code, begin and end times.
STEP_RC=0
run_step() {  # run_step <log> <binary> <command...>
  local log="$1" bin="$2"
  shift 2
  {
    echo "[run.sh] begin $(date -Is)"
    echo "[run.sh] binary $bin"
    echo "[run.sh] sha256_before_exec $(sha256sum "$bin" | cut -d' ' -f1)"
    echo "[run.sh] command $*"
    "$@"
    rc=$?
    echo "[run.sh] exit_code $rc"
    echo "[run.sh] end $(date -Is)"
    exit "$rc"
  } 2>&1 | tee "$log"
  STEP_RC=${PIPESTATUS[0]}
}

# --- render the registered fixtures (rendering only; skipped when the directory is already there) ---
stamp "render begin $(date -Is)"
for shape in cylinder cone; do
  dir="$FIX/image-target-curvy-registered-$shape"
  if [ -e "$dir" ]; then
    echo "[run.sh] fixture $dir exists: not rendered by this runner (the precheck compares its tree sha256)" | tee -a "$OUT/render.log"
  else
    run_step "$OUT/render.$shape.log" "$(command -v python3)" \
      python3 -I "$REPO/slam_refact/tools/image_target/render_synthetic.py" --shape "$shape" --variant registered --out "$dir"
    cat "$OUT/render.$shape.log" >> "$OUT/render.log"
    rm -f "$OUT/render.$shape.log"
    [ "$STEP_RC" = 0 ] || { stamp "RENDER FAILED ($shape): stop, no gate was run"; echo "RENDER FAILED: nothing was run" >&2; exit 8; }
  fi
done
stamp "render end $(date -Is)"

# --- precheck: the node sr_held, side-effect free (no marker, no seed); every kind that has a data directory here ---
precheck_failed=0
{
  for k in cylinder cone wasm-cylinder wasm-cone; do
    case "$k" in *cylinder) dd="$FIX/image-target-curvy-registered-cylinder" ;; *) dd="$FIX/image-target-curvy-registered-cone" ;; esac
    echo "== kind $k data $dd"
    pin_args=()
    for f in $PINNED_FILES; do pin_args+=(--pin "$HELD/$f"); done
    node "$REPO/slam_refact/tools/prereg/sr_held.mjs" check --prereg "$PRE" --gate "$GATE_ID" --version "$GATE_VERSION" \
      --kind "$k" --env-var SLAM_REFACT_CURVY_DESIGNATION "${pin_args[@]}" --data "$dd" || precheck_failed=1
  done
  echo "== kind roi: its data directory is made by the roi test itself; the in-process device checks its tree sha256 before the marker"
  echo "== manifest tool_sha256 (when the manifest has one)"
  python3 -I - "$MANIFEST" "$REPO" <<'PYEOF' || precheck_failed=1
import hashlib, json, os, sys
manifest, repo = sys.argv[1], sys.argv[2]
tools = json.load(open(manifest)).get('tool_sha256')
if not isinstance(tools, dict):
    print('manifest has no tool_sha256: not checked')
    sys.exit(0)
bad = 0
for rel, want in tools.items():
    path = os.path.join(repo, rel)
    have = hashlib.sha256(open(path, 'rb').read()).hexdigest() if os.path.isfile(path) else None
    print(('OK  ' if have == want else 'DIFF') + f' tool {rel}: {have} vs {want}')
    bad += have != want
sys.exit(1 if bad else 0)
PYEOF
  echo "no first-run marker yet: $(ls "$PRE" | grep -c "$GATE_ID.v$GATE_VERSION.first-run" || true) found"
  [ "$precheck_failed" = 0 ] && echo "PRECHECK PASS" || echo "PRECHECK FAIL"
} 2>&1 | tee "$OUT/precheck.log"
if ! grep -q '^PRECHECK PASS$' "$OUT/precheck.log"; then
  stamp "PRECHECK FAILED: stop, nothing was run"
  for v in $(compgen -e | grep '^SLAM_REFACT_'); do unset "$v"; done
  echo "PRECHECK FAILED: nothing was run" >&2
  exit 8
fi

# --- the gate steps; the first command of the first step is the first real run ---
FIRST="$(date -Is)"
stamp "first_command $FIRST"
RC_ROI=NA RC_SYNTH=NA RC_WASM=NA
held_refused() { grep -qE 'HELD(:| \[[A-Z_]+\]:)' "$1"; }

stamp "roi begin $(date -Is)"
run_step "$OUT/roi.log" "$HELD/sr_tests_image_target_curvy_roi" \
  "$HELD/sr_tests_image_target_curvy_roi" --gtest_filter=CurvyRoiOracle.GatesAB
RC_ROI=$STEP_RC
stamp "roi end $(date -Is) rc $RC_ROI"
STOP=""
held_refused "$OUT/roi.log" && STOP=roi

if [ -z "$STOP" ]; then
  stamp "synth begin $(date -Is)"
  run_step "$OUT/synth.log" "$HELD/sr_tests_image_target_curvy_synth" \
    "$HELD/sr_tests_image_target_curvy_synth" --gtest_filter="$SYNTH_FILTER"
  RC_SYNTH=$STEP_RC
  stamp "synth end $(date -Is) rc $RC_SYNTH"
  held_refused "$OUT/synth.log" && STOP=synth
fi

if [ -z "$STOP" ]; then
  stamp "wasm begin $(date -Is)"
  run_step "$OUT/wasm.log" "$(command -v node)" \
    env SLAM_REFACT_WASM_DIR="$HELD" SLAM_REFACT_CURVY_HELD_CHECK="$HELD/sr_curvy_held_check" \
        SLAM_REFACT_IT_CYL_DIR="$FIX/image-target-curvy-registered-cylinder" \
        SLAM_REFACT_IT_CONE_DIR="$FIX/image-target-curvy-registered-cone" \
    node --test "$REPO/slam_refact/engine/tests/wasm/image_target_curvy.test.mjs"
  RC_WASM=$STEP_RC
  stamp "wasm end $(date -Is) rc $RC_WASM"
fi
[ -z "$STOP" ] || stamp "HELD refusal in $STOP.log: stopped after $STOP"

# --- after the run: unset everything designated (F-7), record, then the checksums last (F-4) ---
for v in $(compgen -e | grep '^SLAM_REFACT_'); do unset "$v"; done
stamp "env unset $(date -Is) (remaining SLAM_REFACT_*: '$(env | grep -c '^SLAM_REFACT_' || true)')"
stamp "exit codes roi=$RC_ROI synth=$RC_SYNTH wasm=$RC_WASM"

if [ -z "$STOP" ]; then
  python3 -I - "$MANIFEST" "$FIRST" <<'PYEOF'
import sys
p, first = sys.argv[1], sys.argv[2]
s = open(p).read()
if s.count('"first_real_run_at": null') != 1:
    print('first_real_run_at NOT recorded: the manifest does not hold exactly one "first_real_run_at": null; the coordinator records ' + first)
    sys.exit(0)
open(p, 'w').write(s.replace('"first_real_run_at": null', '"first_real_run_at": "%s"' % first))
print('first_real_run_at recorded: ' + first)
PYEOF
  stamp "manifest sha256 after first_real_run_at: $(sha256sum "$MANIFEST" | cut -d' ' -f1)"
  stamp "canonical pin after first_real_run_at: $(node "$REPO/slam_refact/tools/prereg/sr_held.mjs" print-pin "$MANIFEST")"
fi
ls -la "$PRE" | grep "first-run" >> "$T"
stamp "done $(date -Is)"
# F-4: every output exists now; the list is the last thing written to $OUT
(cd "$OUT" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
(cd "$OUT" && sha256sum -c SHA256SUMS > /dev/null) && echo "SHA256SUMS verified ($(wc -l < "$OUT/SHA256SUMS") files)" || echo "SHA256SUMS does not verify" >&2

[ -z "$STOP" ] && [ "$RC_ROI" = 0 ] && [ "$RC_SYNTH" = 0 ] && [ "$RC_WASM" = 0 ] && exit 0
[ -z "$STOP" ] || exit 7
exit 1
