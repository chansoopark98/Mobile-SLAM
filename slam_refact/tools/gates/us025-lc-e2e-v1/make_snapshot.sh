#!/usr/bin/env bash
# us025-lc-e2e-gate.v1 registration snapshot (protocol §5, "medium" strength): copies of every pinned REPO file named in the manifest (pinned.files; not the
# executables node / python / chrome, not the xr*.js bundles' trees) into <prereg dir>/us025-lc-e2e-gate.v1.snapshot/ with a SHA256SUMS, so that a later change of a
# pinned tool can be diffed against the registered bytes. Written exclusively (refuses to overwrite). Run by the coordinator at registration, after make_lc_manifest.mjs.
#   bash make_snapshot.sh <prereg dir> [manifest file name]
set -euo pipefail
cd "$(dirname "$0")/../../../.."
PRE=${1:?usage: make_snapshot.sh <prereg dir> [manifest]}
MF=$PRE/${2:-us025-lc-e2e-gate.v1.manifest.json}
DST=$PRE/us025-lc-e2e-gate.v1.snapshot
[ -f "$MF" ] || { echo "no manifest $MF" >&2; exit 2; }
[ ! -e "$DST" ] || { echo "refusing to overwrite $DST" >&2; exit 2; }
mkdir -p "$DST"
python3 -I - "$MF" "$DST" <<'PY'
import json, pathlib, shutil, sys
m = json.load(open(sys.argv[1]))
dst = pathlib.Path(sys.argv[2])
for rel in m['pinned']['files']:
    src = pathlib.Path(rel)
    if not src.is_file():
        raise SystemExit('missing ' + rel)
    if src.stat().st_size > 64 * 1024 * 1024:
        print('skipped (larger than 64 MiB): ' + rel)
        continue
    out = dst / rel
    out.parent.mkdir(parents=True, exist_ok=True)
    shutil.copyfile(src, out)
PY
(cd "$DST" && find . -type f ! -name SHA256SUMS -print0 | sort -z | xargs -0 sha256sum > SHA256SUMS)
echo "snapshot: $DST ($(wc -l < "$DST/SHA256SUMS") files)"
