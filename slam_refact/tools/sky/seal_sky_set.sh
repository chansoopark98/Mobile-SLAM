#!/usr/bin/env bash
# Seals recorded sky gate sets and prints the values a registration pins. It never touches a set that is already sealed.
#
#   seal_sky_set.sh <set dir>
#       Writes <set>/SHA256SUMS (sha256 of every file of the recorded set, relative paths, sorted) and prints
#       "<name> <sha256 of replay.txt> <sha256 of SHA256SUMS>": the three values a sealed.json entry of the v1 registration
#       pins (US-020, sky-us020.v1.md section 2). Refuses to seal twice.
#
#   seal_sky_set.sh --held-values <gate data root>
#       Read-only. For registrations on the shared HELD device (sr_held, sky-us020.v2 and later): the root holds the sealed
#       sets as immediate subdirectories (each already sealed with the form above). Verifies every set against its
#       SHA256SUMS, then prints the JSON fragment of the manifest `held` object that depends on the data:
#         inputs_tree_sha256       -> held.inputs[<kind>]       (tree sha256 of the whole root, tools/prereg/sr_held.mjs treeSha)
#         sealed_file_sha256       -> held.sealed_file_sha256   (replay.txt and SHA256SUMS of every set: report mode and the
#                                                               replay tool refuse a set that contains one of them)
#       The registration names no set: the sets are the subdirectories of the root and the tree sha256 binds them.
set -euo pipefail
here="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ "${1:-}" = "--held-values" ]; then
  root="${2:-}"
  root="${root%/}"
  [ -n "$root" ] && [ -d "$root" ] || { echo "usage: seal_sky_set.sh --held-values <gate data root>" >&2; exit 2; }
  sets=0
  for d in "$root"/*/; do
    [ -d "$d" ] || continue
    d="${d%/}"
    [ -f "$d/replay.txt" ] || { echo "not a recorded set (no replay.txt): $d" >&2; exit 2; }
    [ -f "$d/SHA256SUMS" ] || { echo "not sealed (no SHA256SUMS): $d" >&2; exit 2; }
    (cd "$d" && sha256sum --check --quiet SHA256SUMS) || { echo "set differs from its SHA256SUMS: $d" >&2; exit 2; }
    sets=$((sets + 1))
  done
  [ "$sets" -gt 0 ] || { echo "no recorded set below $root" >&2; exit 2; }
  exec env SR_HELD_MJS="$here/../prereg/sr_held.mjs" SR_HELD_ROOT="$root" node --input-type=module -e '
    import fs from "node:fs";
    import { pathToFileURL } from "node:url";
    const { treeSha, sha256 } = await import(pathToFileURL(process.env.SR_HELD_MJS).href);
    const root = process.env.SR_HELD_ROOT;
    const t = treeSha(root);
    if (t.sha === undefined) { console.error("tree sha256 failed: " + t.code); process.exit(2); }
    const names = fs.readdirSync(root, { withFileTypes: true }).filter((e) => e.isDirectory()).map((e) => e.name).sort();
    const files = new Set();
    for (const n of names) for (const f of ["replay.txt", "SHA256SUMS"]) files.add(sha256(fs.readFileSync(`${root}/${n}/${f}`)));
    console.log(JSON.stringify({ sets: names, inputs_tree_sha256: t.sha, sealed_file_sha256: [...files].sort() }, null, 1));
  '
fi

dir="${1%/}"
[ -f "$dir/replay.txt" ] || { echo "no replay.txt in $dir" >&2; exit 2; }
[ ! -e "$dir/SHA256SUMS" ] || { echo "already sealed: $dir/SHA256SUMS" >&2; exit 2; }
(cd "$dir" && find . -type f ! -name SHA256SUMS -printf '%P\n' | LC_ALL=C sort | xargs -d '\n' sha256sum > SHA256SUMS)
echo "$(basename "$dir") $(sha256sum "$dir/replay.txt" | cut -d' ' -f1) $(sha256sum "$dir/SHA256SUMS" | cut -d' ' -f1)"
