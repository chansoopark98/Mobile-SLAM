#!/usr/bin/env bash
set -euo pipefail
replay_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
replay_build="${MOBILE_SLAM_NATIVE_BUILD:-$replay_root/build/refactor-native}"
replay_jobs="${MOBILE_SLAM_JOBS:-2}"
[[ "$replay_jobs" =~ ^[1-4]$ ]] || { echo 'MOBILE_SLAM_JOBS must be 1..4' >&2; exit 2; }
replay_configure=(-S "$replay_root" -B "$replay_build" -G Ninja -DCMAKE_BUILD_TYPE=Release
 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DMOBILE_SLAM_BUILD_VIEWER=OFF)
for dependency in ceres-solver googletest; do
 if [[ -f "$replay_root/build/_deps/$dependency-src/CMakeLists.txt" ]]; then
  replay_variable="FETCHCONTENT_SOURCE_DIR_${dependency^^}"
  replay_configure+=("-D$replay_variable=$replay_root/build/_deps/$dependency-src")
 fi
done
if [[ ! -f "$replay_build/build.ninja" ]]; then cmake "${replay_configure[@]}"; fi
# MOBILE_SLAM_SKIP_BUILD only for sequential profiling after a verified build.
if [[ "${MOBILE_SLAM_SKIP_BUILD:-0}" != 1 ]]; then
 cmake --build "$replay_build" --target native_replay --parallel "$replay_jobs"
fi
python3 "$replay_root/scripts/dev/build-provenance.py" --build "$replay_build" --artifact "$replay_build/native_replay" --output "$replay_build/native-replay-provenance.json"
if (( $# )); then exec "$replay_build/native_replay" "$@"; fi
printf '%s\n' "$replay_build/native_replay"
