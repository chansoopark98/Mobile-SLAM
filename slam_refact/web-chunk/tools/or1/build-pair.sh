#!/bin/bash
# OR-1: build a MATCHED native + WASM engine pair from one frozen copy of the engine sources, so that a bit comparison between the two
# (chunk WASM vs native exact-input replay) never compares different sources.
#   bash slam_refact/web-chunk/tools/or1/build-pair.sh <out dir under build/>   (jobs 2; native build 1 at a time, project rule)
# Writes <out>/snapshot (frozen copy of slam_refact/engine + the replay harness sources; Ceres / emsdk inputs are symlinks to the
# repo's offline copies), <out>/native/harness/slam_refact_replay, <out>/wasm/wasm/slam_refact_engine.{js,wasm}, and
# <out>/pair.json (sha256 of every frozen source file as one tree sha, of both binaries, and the build commands).
# Does not touch slam_refact/ or web/; refuses to overwrite an existing <out>.
set -euo pipefail
cd "$(dirname "$0")/../../../.."
OUT=${1:?usage: build-pair.sh <out dir under build/>}
case "$OUT" in build/*) ;; *) echo "out must be under build/" >&2; exit 2;; esac
[ ! -e "$OUT" ] || { echo "refusing to overwrite $OUT" >&2; exit 2; }
mkdir -p "$OUT/snapshot/slam_refact/tools/harness" "$OUT/snapshot/build" "$OUT/snapshot/wasm/libs"
SNAP=$OUT/snapshot
date -Is > "$OUT/freeze-at.txt"
tar -C slam_refact -cf - --exclude='.omc' --exclude='__pycache__' --exclude='node_modules' engine | tar -C "$SNAP/slam_refact" -xf -
tar -C slam_refact/tools/harness -cf - --exclude='__pycache__' replay | tar -C "$SNAP/slam_refact/tools/harness" -xf -
ln -s "$PWD/build/_deps" "$SNAP/build/_deps"
ln -s "$PWD/wasm/libs/ceres" "$SNAP/wasm/libs/ceres"
(cd "$SNAP" && find slam_refact -type f -print0 | sort -z | xargs -0 sha256sum) > "$OUT/snapshot-sources.sha256"
TREE_SHA=$(sha256sum "$OUT/snapshot-sources.sha256" | cut -d' ' -f1)
echo "snapshot tree sha $TREE_SHA" | tee "$OUT/build.log"
# native (the replay runner that drives GlueRunner with an exact-input bundle)
cmake -S "$SNAP/slam_refact/engine" -B "$OUT/native" -G Ninja -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  -DSLAM_REFACT_BUILD_TESTS=OFF -DSLAM_REFACT_BUILD_HARNESS=ON >> "$OUT/build.log" 2>&1
cmake --build "$OUT/native" --parallel 2 --target slam_refact_replay >> "$OUT/build.log" 2>&1
# wasm (the engine the chunk loads)
set +u; source ~/emsdk/emsdk_env.sh >/dev/null 2>&1; set -u
cmake -S "$SNAP/slam_refact/engine" -B "$OUT/wasm" -G Ninja -DCMAKE_BUILD_TYPE=Release -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
  -DCMAKE_TOOLCHAIN_FILE="$EMSDK/upstream/emscripten/cmake/Modules/Platform/Emscripten.cmake" \
  -DSLAM_REFACT_BUILD_TESTS=OFF -DSLAM_REFACT_BUILD_HARNESS=OFF >> "$OUT/build.log" 2>&1
cmake --build "$OUT/wasm" --parallel 2 >> "$OUT/build.log" 2>&1
date -Is > "$OUT/built-at.txt"
NB=$OUT/native/harness/slam_refact_replay; WJ=$OUT/wasm/wasm/slam_refact_engine.js; WW=$OUT/wasm/wasm/slam_refact_engine.wasm
python3 - "$OUT" "$TREE_SHA" "$NB" "$WJ" "$WW" <<'PY'
import hashlib, json, sys
out, tree, nb, wj, ww = sys.argv[1:]
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
json.dump({'schema': 'or1-engine-pair-v1', 'snapshot_tree_sha256': tree, 'snapshot_sources': out + '/snapshot-sources.sha256',
           'native_replay': {'file': nb, 'sha256': sha(nb)}, 'wasm_js': {'file': wj, 'sha256': sha(wj)}, 'wasm': {'file': ww, 'sha256': sha(ww)},
           'note': 'native and WASM were built from the same frozen copy; compiler / libm differ (glibc vs musl), so a bit comparison between the two is a tolerance-tier comparison, not an identity'},
          open(out + '/pair.json', 'w'), indent=1)
PY
echo "pair done: $OUT/pair.json"
