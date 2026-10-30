#!/usr/bin/env bash
# US-025 LC-6b: build a DIAGNOSTIC copy of the engine (working tree) under build/slam-refact-pkgh-20261022/us025/lc6b/diag-engine in which every
# LoopClose call prints one line to stderr (outcome, newest/candidate ids, distance, slots/live, matches, correspondences, unlinked, correction matrix).
# The repository engine sources are NOT modified: the copy is patched by exact text replacement (asserted to apply once). The copy is only trusted
# after its replay trace equals the reference binary's trace bit for bit on the same bundle (lc6b_compare.py --binary-check).
#   bash make_diag_engine.sh            (configure + build slam_refact_replay with 2 jobs)
set -euo pipefail
cd "$(dirname "$0")/../../../../.."
ROOT=$PWD
D=build/slam-refact-pkgh-20261022/us025/lc6b/diag-engine
[ ! -e $D ] || { echo "exists: $D" >&2; exit 2; }
mkdir -p $D/snap/slam_refact/tools/harness
cp -r slam_refact/engine $D/snap/slam_refact/engine
cp -r slam_refact/tools/harness/replay $D/snap/slam_refact/tools/harness/replay
python3 -I - "$D/snap/slam_refact/engine/src/map_builder/map_builder.cc" <<'PY'
import sys, pathlib
p = pathlib.Path(sys.argv[1]); s = p.read_text()
old = "    LoopClose(wm, loop_close_.get(), out);\n"
assert s.count(old) == 1
new = """    {
      LoopCloseReport rep;
      static int lc6b_call = 0;
      const bool applied = LoopClose(wm, loop_close_.get(), out, &rep);
      std::fprintf(stderr, "LC6B call=%d outcome=%d applied=%d newest=%u/%u cand=%u/%u dist=%.1f slots=%zu live=%zu matches=%zu corr=%zu unlinked=%zu C=",
                   lc6b_call++, static_cast<int>(rep.outcome), applied ? 1 : 0, static_cast<unsigned>(rep.newest.index()), static_cast<unsigned>(rep.newest.id()), static_cast<unsigned>(rep.candidate.index()), static_cast<unsigned>(rep.candidate.id()),
                   static_cast<double>(rep.candidate_distance), rep.num_slots, rep.num_live, rep.num_matches, rep.num_correspondences, rep.num_unlinked);
      for (int r = 0; r < 4; ++r) for (int c = 0; c < 4; ++c) std::fprintf(stderr, "%.9g%c", static_cast<double>(out->world_correction(r, c)), (r == 3 && c == 3) ? '\\n' : ',');
    }
"""
s = s.replace(old, new)
assert s.count('#include <cstdio>') == 0
s = s.replace('#include "map_builder/loop_close.h"', '#include <cstdio>\n#include "map_builder/loop_close.h"', 1)
p.write_text(s)
PY
cmake -S $D/snap/slam_refact/engine -B $D/build -G Ninja -DCMAKE_BUILD_TYPE=Release -DSLAM_REFACT_LOOP_CLOSE=ON -DSLAM_REFACT_BUILD_TESTS=OFF -DSLAM_REFACT_BUILD_HARNESS=ON \
  -DSLAM_REFACT_WERROR=OFF -DSLAM_REFACT_EVIDENCE_ROOT=$ROOT/build/slam-refact-20261015 \
  -DFETCHCONTENT_SOURCE_DIR_CERES-SOLVER=$ROOT/build/_deps/ceres-solver-src -DFETCHCONTENT_SOURCE_DIR_GOOGLETEST=$ROOT/build/_deps/googletest-src \
  -DSLAM_REFACT_CERES_SOURCE_DIR=$ROOT/build/_deps/ceres-solver-src -DSLAM_REFACT_GTEST_SOURCE_DIR=$ROOT/build/_deps/googletest-src \
  -DSLAM_REFACT_CERES_VERSION=2.2 -DSLAM_REFACT_ALLOW_BLOCKED_MLMODEL=OFF -DSLAM_REFACT_ALLOW_MISSING_VENDOR=OFF \
  > $D/configure.log 2>&1
ninja -C $D/build -j2 slam_refact_replay > $D/build.log 2>&1
echo built $D/build/harness/slam_refact_replay
