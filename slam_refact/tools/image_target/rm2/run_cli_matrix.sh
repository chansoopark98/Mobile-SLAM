#!/usr/bin/env bash
# B6 / RM-2: run the PUBLIC image-target CLI (sharp 0.34.5, unmodified source) and our Python CLI
# (--gray-mode pil and libvips) on the 13 public test/*.txt answer scripts plus 3 docs/ images, with the LFS images.
# No network: sharp is already installed under build/slam-refact-full-c1-net/npm/node_modules.
# Writes only under build/slam-refact-full-c1-net/rm2-results/. Never touches assets/references.
set -euo pipefail
REPO=$(cd "$(dirname "$0")/../../../.." && pwd)
NET=$REPO/build/slam-refact-full-c1-net
OUT=${1:-$NET/rm2-results}
REF=$REPO/assets/references/xr_reference/official/xr_reference/apps/image-target-cli
LFS=$NET/lfs-files/apps/image-target-cli
CLI=$REPO/slam_refact/tools/image_target/image_target_cli.py

[ -d "$NET/npm/node_modules/sharp" ] || { echo "sharp not installed"; exit 2; }
mkdir -p "$OUT/orig-cli"
# copy (not symlink) the public source so 'import sharp' resolves; prove it is byte-identical.
rm -rf "$OUT/orig-cli/src"
cp -r "$REF/src" "$OUT/orig-cli/src"
cp "$REF/package.json" "$OUT/orig-cli/package.json"
ln -sfn "$NET/npm/node_modules" "$OUT/orig-cli/node_modules"
( cd "$REF" && find src -type f | sort | xargs sha256sum ) > "$OUT/orig-src.ref.sha256"
( cd "$OUT/orig-cli" && find src -type f | sort | xargs sha256sum ) > "$OUT/orig-src.copy.sha256"
diff -q "$OUT/orig-src.ref.sha256" "$OUT/orig-src.copy.sha256" && echo "orig src copy == reference (byte-identical)"

for impl in orig pil libvips; do
  W=$OUT/runs/$impl
  rm -rf "$W"; mkdir -p "$W/test/exif" "$W/logs"
  for f in "$REF"/test/*.txt; do ln -s "$f" "$W/test/$(basename "$f")"; done
  for f in "$LFS"/test/*.jpg "$LFS"/test/*.png; do ln -s "$f" "$W/test/$(basename "$f")"; done
  for f in "$LFS"/test/exif/*.jpg; do ln -s "$f" "$W/test/exif/$(basename "$f")"; done
  # extra cases not in the public test/*.txt: the three docs/ images (flat, default crop). cone-diagram.jpg
  # is a PNG by content, which exercises the output-extension rule (sharp metadata().format vs PIL).
  for f in "$LFS"/docs/*.jpg; do
    n=$(basename "$f" .jpg)
    ln -s "$f" "$W/test/docs-$n.jpg"
    printf 'test/docs-%s.jpg\nflat\ny\ntmp\ndocs-%s\n' "$n" "$n" > "$W/test/docs-$n.txt"
  done
done

run_case() {  # impl case
  local impl=$1 case=$2 W=$OUT/runs/$1 rc=0
  cd "$W"
  case $impl in
    orig)   OVERWRITE_FILES=true node "$OUT/orig-cli/src/index.js" < "test/$case.txt" \
              > "logs/$case.out" 2> "logs/$case.err" || rc=$? ;;
    pil)    python3 -I "$CLI" --script "test/$case.txt" --overwrite --timestamp 0 \
              > "logs/$case.out" 2> "logs/$case.err" || rc=$? ;;
    libvips) python3 -I "$CLI" --script "test/$case.txt" --overwrite --timestamp 0 --gray-mode libvips \
              > "logs/$case.out" 2> "logs/$case.err" || rc=$? ;;
  esac
  echo "$impl $case exit=$rc"
}
for c in $( (for f in "$REF"/test/*.txt; do basename "$f" .txt; done; for f in "$LFS"/docs/*.jpg; do echo "docs-$(basename "$f" .jpg)"; done) ); do
  for impl in orig pil libvips; do ( run_case $impl $c ); done
done | tee "$OUT/run_cli_matrix.exit.log"
