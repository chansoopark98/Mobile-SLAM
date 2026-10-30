#!/bin/bash
# Builds rm3b_recenter_replay out of tree: compiles recenter_replay_main.cc with the flags of an existing build's replay_main.cc and links the
# existing build's libraries (no repository file changes, no cmake run). Usage: build_engine.sh <native build dir> <its source snapshot root> <output binary>
#   e.g. build_engine.sh build/slam-refact-final-20261025/lc5/native build/slam-refact-final-20261025/lc5/src build/slam-refact-pkgh-20261022/rm3b-recenter/engine/rm3b_recenter_replay
set -e
N=$(realpath "$1"); S=$(realpath "$2"); OUT=$(realpath -m "$3")
HERE=$(cd "$(dirname "$0")" && pwd)
mkdir -p "$(dirname "$OUT")"
cd "$N"
FLAGS=$(ninja -t commands harness/CMakeFiles/slam_refact_replay_obj.dir/replay_main.cc.o | tail -1 | sed -e 's/ -MD -MT [^ ]* -MF [^ ]*//' -e 's/ -o [^ ]* -c .*$//' -e 's/^\/usr\/bin\/c++ //')
LINK=$(ninja -t commands harness/slam_refact_replay | tail -1 | sed -e 's/^.*-o harness\/slam_refact_replay *//' -e 's/ *&& :$//')
c++ $FLAGS -o "$OUT.o" -c "$HERE/recenter_replay_main.cc"
c++ -O3 -DNDEBUG "$OUT.o" -o "$OUT" $LINK
rm -f "$OUT.o"
echo built "$OUT"
