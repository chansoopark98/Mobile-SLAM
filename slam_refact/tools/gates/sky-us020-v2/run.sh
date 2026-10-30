#!/usr/bin/env bash
# Runner of the first real run of the sky gate registration sky-us020.v2 (US-020; shared HELD device sr_held).
#
# This script is the COORDINATOR's command for the one real run. The executor who wrote it did not run it (bash -n only):
# the registration (gate file, manifest, review, designation file, data, binary) does not exist when it is written.
#
#   bash slam_refact/tools/gates/sky-us020-v2/run.sh --check-only --prereg DIR --ref-root DIR --binary FILE --model FILE
#   SR_SKY_REVIEWER=<sha256 of the designation file> \
#   bash slam_refact/tools/gates/sky-us020-v2/run.sh --prereg DIR --ref-root DIR --report-dir NEW_DIR \
#        --binary FILE --model FILE --log NEW_FILE
#   bash slam_refact/tools/gates/sky-us020-v2/run.sh --audit --prereg DIR      # after the coordinator recorded first_real_run_at
#
#   --prereg      the preregistration directory of the lane (manifest, gate file, review, designation file, markers)
#   --ref-root    the gate data root: its immediate subdirectories are the recorded sets (tree sha256 = manifest held.inputs.main)
#   --report-dir  a directory that does not exist yet (the gate writes <set>.json and held-run-record.txt into it)
#   --binary      the frozen sr_tests_sky_gate (its file name is the manifest held.pins key and must be the pinned one)
#   --model       the semantics-model.tflite (pinned in held.pins too)
#   --log         a file that does not exist yet: EVERYTHING below is written into it through tee, including the exit code
#
# Registration: gate id sky-us020, version 2, kind main (constants below). SR_SKY_REVIEWER comes from the environment of the
# caller; this script never sets it. What the script does before the gate, all inside the log:
#   1. records its own sha256, argv, time, host and, for every file the registration names, the resolved path, sha256 and stat
#   2. lane-precheck.sh (slam_refact/docs/design/tools; PRECHECK_* are not redefined here); a block is exit 4, nothing is run
#   3. the node twin of the device (tools/prereg/sr_held.mjs check, no marker) must RELEASE; otherwise exit 3, nothing is run
#   4. the pin printed from the live manifest equals the pin in the designation file (the verify_manifest --designation step)
#   5. runs the gate binary once (SkyHeld.* unit tests, then SkyGate.*); the C++ device decides again and takes the marker
# and after it: the exit code of the binary, the marker (stat, sha256), the report directory listing, the manifest sha256, a
# checksum file of the report directory (made read-only) and the sha256 of the log (<log>.sha256, outside the log).
# Exit codes: 0 gate binary passed; 1-2 usage/input; 3 device check; 4 precheck; other = the gate binary's code.
# Nothing here edits the manifest, the data or any registered file.
set -uo pipefail

readonly GATE_ID=sky-us020
readonly GATE_VERSION=2
readonly KIND=main

here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly HELD_MJS="$here/../../prereg/sr_held.mjs"
readonly PRECHECK="$here/../../../docs/design/tools/lane-precheck.sh"

die() { echo "run.sh: $*" >&2; exit 2; }

MODE=run PREREG="" REF_ROOT="" REPORT_DIR="" BINARY="" MODEL="" LOG=""
while [ $# -gt 0 ]; do
  case "$1" in
    --check-only) MODE=check ;;
    --audit) MODE=audit ;;
    --prereg) PREREG="${2:-}"; shift ;;
    --ref-root) REF_ROOT="${2:-}"; shift ;;
    --report-dir) REPORT_DIR="${2:-}"; shift ;;
    --binary) BINARY="${2:-}"; shift ;;
    --model) MODEL="${2:-}"; shift ;;
    --log) LOG="${2:-}"; shift ;;
    *) die "unknown argument $1" ;;
  esac
  shift
done

[ -n "$PREREG" ] && [ -d "$PREREG" ] || die "--prereg DIR (an existing directory) is required"
PREREG="$(cd "$PREREG" && pwd)"
readonly ID="$GATE_ID.v$GATE_VERSION"
readonly MANIFEST="$PREREG/$ID.manifest.json"
[ -f "$MANIFEST" ] || die "no manifest $MANIFEST: this runner is for the registration $ID"

# value of a top-level or review.* string field of the manifest / designation file (strict enough for names; the device
# itself reads the files with its strict parser)
json_field() { # file path...   e.g. json_field "$MANIFEST" review designation_file
  local file="$1"; shift
  SR_JSON_FILE="$file" SR_JSON_PATH="$*" node -e '
    const o = JSON.parse(require("fs").readFileSync(process.env.SR_JSON_FILE, "utf8"));
    let v = o; for (const k of process.env.SR_JSON_PATH.split(" ")) v = v?.[k];
    process.stdout.write(typeof v === "string" ? v : "");'
}

if [ "$MODE" = audit ]; then
  # after the run and after the coordinator recorded first_real_run_at: created_at < review.at < first_real_run_at, the
  # pinned files and the marker (device Audit). No marker is created.
  SR_HELD_MJS="$HELD_MJS" SR_PREREG="$PREREG" SR_GATE="$GATE_ID" SR_VERSION="$GATE_VERSION" SR_KIND="$KIND" node --input-type=module -e '
    import { pathToFileURL } from "node:url";
    const { audit } = await import(pathToFileURL(process.env.SR_HELD_MJS).href);
    const o = audit({ preregDir: process.env.SR_PREREG, gateId: process.env.SR_GATE, version: Number(process.env.SR_VERSION), kind: process.env.SR_KIND });
    console.log(o.log.join("\n"));
    console.log(o.ok ? "AUDIT OK" : `AUDIT FAILED [${o.code}]: ${o.why}`);
    process.exit(o.ok ? 0 : 3);'
  exit $?
fi

[ -n "$REF_ROOT" ] && [ -d "$REF_ROOT" ] || die "--ref-root DIR (an existing directory) is required"
[ -n "$BINARY" ] && [ -f "$BINARY" ] && [ -x "$BINARY" ] || die "--binary FILE (the frozen gate binary) is required"
[ -n "$MODEL" ] && [ -f "$MODEL" ] || die "--model FILE (semantics-model.tflite) is required"
REF_ROOT="$(cd "$REF_ROOT" && pwd)"
BINARY="$(readlink -f "$BINARY")"
MODEL="$(readlink -f "$MODEL")"
if [ "$MODE" = run ]; then
  [ -n "${SR_SKY_REVIEWER:-}" ] || die "SR_SKY_REVIEWER (the sha256 of the designation file, handed over by the coordinator) is not set"
  [ -n "$REPORT_DIR" ] || die "--report-dir NEW_DIR is required"
  [ -n "$LOG" ] || die "--log NEW_FILE is required"
  [ ! -e "$REPORT_DIR" ] || die "refusing to overwrite $REPORT_DIR"
  [ ! -e "$LOG" ] || die "refusing to overwrite $LOG"
  [ -d "$(dirname "$REPORT_DIR")" ] || die "the parent of $REPORT_DIR does not exist"
fi

describe() { # file: resolved path, sha256, stat
  local f="$1"
  if [ -f "$f" ]; then
    echo "file $(readlink -f "$f")"
    echo "  sha256 $(sha256sum "$f" | cut -d' ' -f1)"
    echo "  stat $(stat -c '%A %U size=%s mtime=%y' "$f")"
  else
    echo "file $f MISSING"
  fi
}

main() {
  local rc
  echo "== sky gate runner: $ID (mode $MODE)"
  echo "time_start $(date -Is)"
  echo "host $(hostname) $(uname -sr)"
  echo "argv $0 $*"
  echo "runner_sha256 $(sha256sum "${BASH_SOURCE[0]}" | cut -d' ' -f1)"
  echo "held_device_sha256 $(sha256sum "$HELD_MJS" | cut -d' ' -f1)"
  echo "== files of the registration (path, sha256, stat)"
  local review designation
  review="$(json_field "$MANIFEST" review file)"
  designation="$(json_field "$MANIFEST" review designation_file)"
  describe "$BINARY"
  describe "$MODEL"
  describe "$MANIFEST"
  describe "$PREREG/$ID.md"
  describe "$PREREG/$review"
  describe "$PREREG/$designation"
  echo "ref_root $REF_ROOT"
  echo "report_dir $REPORT_DIR"
  echo "SR_SKY_REVIEWER set: $([ -n "${SR_SKY_REVIEWER:-}" ] && echo yes || echo no)"

  echo "== 2. lane-precheck"
  bash "$PRECHECK"
  rc=$?
  echo "precheck_exit=$rc"
  if [ "$rc" -ne 0 ]; then echo "exit=4"; return 4; fi

  echo "== 3. sr_held (node twin), check only: no marker"
  node "$HELD_MJS" check --prereg "$PREREG" --gate "$GATE_ID" --version "$GATE_VERSION" --kind "$KIND" \
    --env-var SR_SKY_REVIEWER --pin "$BINARY" --pin "$MODEL" --data "$REF_ROOT"
  rc=$?
  echo "node_check_exit=$rc"
  if [ "$rc" -ne 0 ]; then echo "exit=3"; return 3; fi

  echo "== 4. pin of the live manifest equals the pin in the designation file"
  local live pinned
  live="$(node "$HELD_MJS" print-pin "$MANIFEST")"
  pinned="$(node -e 'const o = JSON.parse(require("fs").readFileSync(process.argv[1], "utf8")); process.stdout.write(o.pinned.manifest_before_review_sha256);' "$PREREG/$designation")"
  echo "manifest_before_review_sha256 live=$live designation=$pinned"
  if [ "$live" != "$pinned" ]; then echo "pin differs"; echo "exit=3"; return 3; fi
  if [ "$MODE" = check ]; then echo "check only: nothing run"; echo "exit=0"; return 0; fi

  echo "== 5. gate binary, once (the C++ device decides again and takes the marker)"
  echo "binary $BINARY"
  echo "binary_sha256_before_exec $(sha256sum "$BINARY" | cut -d' ' -f1)"
  echo "model_sha256_before_exec $(sha256sum "$MODEL" | cut -d' ' -f1)"
  echo "time_exec $(date -Is)"
  env SR_SKY_PREREG_DIR="$PREREG" SR_SKY_GATE_ID="$GATE_ID" SR_SKY_GATE_VERSION="$GATE_VERSION" SR_SKY_GATE_KIND="$KIND" \
      SR_SKY_REF_ROOT="$REF_ROOT" SR_SKY_REPORT_DIR="$REPORT_DIR" \
      "$BINARY" --gtest_filter='SkyHeld.*:SkyGate.*'
  rc=$?
  echo "gate_binary_exit=$rc"
  echo "time_end $(date -Is)"

  echo "== after the run (read-only)"
  local marker="$PREREG/$ID.first-run.$KIND.marker"
  describe "$marker"
  describe "$MANIFEST"
  if [ -d "$REPORT_DIR" ]; then
    ls -l "$REPORT_DIR"
    (cd "$REPORT_DIR" && find . -type f -printf '%P\n' | LC_ALL=C sort | xargs -r -d '\n' sha256sum) > "$REPORT_DIR.SHA256SUMS"
    chmod a-w "$REPORT_DIR.SHA256SUMS"
    echo "== $REPORT_DIR.SHA256SUMS"
    cat "$REPORT_DIR.SHA256SUMS"
    chmod -R a-w "$REPORT_DIR"
  else
    echo "no report directory (the gate was refused before the marker, or it failed early)"
  fi
  echo "exit=$rc"
  return "$rc"
}

if [ "$MODE" = check ]; then
  main
  exit $?
fi
main 2>&1 | tee "$LOG"
rc="${PIPESTATUS[0]}"
sha256sum "$LOG" > "$LOG.sha256"
chmod a-w "$LOG" "$LOG.sha256"
exit "$rc"
