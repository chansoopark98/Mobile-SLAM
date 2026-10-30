#!/usr/bin/env bash
# B6 / RM-2: reproduce every number in rm2/README.md. No network, no build. Writes only under
# build/slam-refact-full-c1-net/rm2-results/. Re-running deletes and recreates that directory's contents.
set -euo pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../../../.." && pwd)
NET=$REPO/build/slam-refact-full-c1-net
OUT=$NET/rm2-results
LF=$NET/lfs-files
ORIG=$OUT/orig-cli

rm -rf "$OUT"; mkdir -p "$OUT/logs"
exec > >(tee "$OUT/logs/run_all.log") 2>&1

echo "== env"
{ echo "date $(date -Is)"; echo "node $(node --version)"; echo "python $(python3 --version)";
  python3 -I -c "import PIL, numpy; print('PIL', PIL.__version__, 'numpy', numpy.__version__)";
  grep '"version"' "$NET/npm/node_modules/sharp/package.json" | sed 's/^ */sharp /';
  echo "git HEAD $(git -C "$REPO" rev-parse --short HEAD) branch $(git -C "$REPO" rev-parse --abbrev-ref HEAD)"; } | tee "$OUT/env.txt"

echo "== LFS sha256 (re-check, no fetch)"
( cd "$LF" && sha256sum -c ../lfs-files.SHA256SUMS ) | awk '{print $NF}' | sort | uniq -c | tee "$OUT/logs/lfs_sha256_check.txt"

echo "== 1. public CLI vs our CLI on the 13 public answer scripts"
bash "$HERE/run_cli_matrix.sh" "$OUT"

echo "== 2. sharp stages: apps images (CLI contract) and extra LFS images"
APPS=$(cd "$LF" && ls apps/image-target-cli/docs/*.jpg apps/image-target-cli/test/*.jpg apps/image-target-cli/test/*.png apps/image-target-cli/test/exif/*.jpg | sed "s|^|$LF/|")
EXTRA=$(cd "$LF" && ls reality/engine/*/testdata/*.jpg reality/engine/*/testdata/*.png reality/engine/testdata/*.jpg | sed "s|^|$LF/|")
node "$HERE/sharp_gray_stages.mjs" "$ORIG" "$OUT/gray-stages" $APPS > "$OUT/logs/gray_stages_apps.log"
node "$HERE/sharp_gray_stages.mjs" "$ORIG" "$OUT/gray-stages-extra" $EXTRA > "$OUT/logs/gray_stages_extra.log"
node "$HERE/sharp_gray_probe.mjs" "$ORIG" "$OUT/gray-probe" | tee "$OUT/logs/gray_probe.log"

echo "== 3. analysis"
python3 -I "$HERE/fit_gray_lut.py" "$OUT/gray-probe" "$OUT/gray-stages" "$OUT/gray-stages-extra" | tee "$OUT/logs/fit_gray_lut.log"
python3 -I "$HERE/analyze_gray.py" "$OUT/gray-stages" "$OUT/gray_analysis" > "$OUT/logs/analyze_gray_apps.log"
python3 -I "$HERE/analyze_gray.py" "$OUT/gray-stages-extra" "$OUT/gray_analysis_extra" > "$OUT/logs/analyze_gray_extra.log"
python3 -I "$HERE/resample_emulation.py" "$OUT/gray-stages" "$OUT/resample_emulation" > "$OUT/logs/resample_apps.log"
python3 -I "$HERE/resample_emulation.py" "$OUT/gray-stages-extra" "$OUT/resample_emulation_extra" > "$OUT/logs/resample_extra.log"
node "$HERE/lfs_image_inventory.mjs" "$ORIG" "$LF" > "$OUT/lfs_inventory.tsv"

echo "== 4. proposed change (monkeypatch, image_target_cli.py untouched) and comparison"
python3 -I "$HERE/run_lut_patch.py" "$OUT" > "$OUT/logs/run_lut_patch.log" 2>&1
python3 -I "$HERE/compare_runs.py" "$OUT" > "$OUT/logs/compare_runs.stdout.log"

echo "== 5. public unit tests (node) and our ports (python)"
( cd "$ORIG" && node --test src/*.test.js ) > "$OUT/logs/public_node_tests.log" 2>&1 || true
tail -9 "$OUT/logs/public_node_tests.log"
( cd "$HERE/.." && python3 -I -m unittest discover -s tests -p 'test_image_target_cli.py' ) > "$OUT/logs/our_unittest.log" 2>&1 || true
grep -E "^(Ran|OK|FAILED)" "$OUT/logs/our_unittest.log"

echo "== 6. exploratory model probes on the LFS face/hand images"
H=$LF/reality/engine
python3 -I "$HERE/palm_probe.py" "$OUT/palm_probe.json" \
  $H/deepnets/testdata/hand_left_192.jpg $H/deepnets/testdata/hand_left_up_224.jpg \
  $H/deepnets/testdata/hand_right_up_128.jpg $H/deepnets/testdata/desk.jpg $H/deepnets/testdata/livingroom.jpg \
  $H/deepnets/testdata/faces.jpg $H/deepnets/testdata/office.png $H/testdata/real_test_face.jpg \
  $H/deepnets/testdata/ear_left.jpg 2>&1 | grep -v -E "RuntimeWarning|np.exp" > "$OUT/logs/palm_probe.log"
python3 -I "$HERE/blazeface_probe.py" $H/deepnets/testdata/faces.jpg "$OUT/blazeface_probe.json" \
  > "$OUT/logs/blazeface_probe.log" 2>&1
cat "$OUT/logs/blazeface_probe.log"

echo "== 6b. embedded ICC profiles: sharp vs PIL decode (raw dumps are deleted afterwards, the json stays)"
node "$HERE/icc_probe.mjs" "$ORIG" "$LF" "$OUT/icc-raw" > "$OUT/logs/icc_probe_node.log"
python3 -I "$HERE/icc_probe.py" "$OUT/icc-raw" "$OUT/icc_probe.json" > "$OUT/logs/icc_probe.log"
rm -rf "$OUT/icc-raw"
grep -c "'max': 0" "$OUT/logs/icc_probe.log" || true

echo "== 7. sha256 of every result file"
( cd "$OUT" && find . -type f ! -path './logs/run_all.log' ! -path './SHA256SUMS' | sort | xargs sha256sum ) > "$OUT/SHA256SUMS"
wc -l "$OUT/SHA256SUMS"
echo done
