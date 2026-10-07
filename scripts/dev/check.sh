#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd -- "$root"
jobs="${MOBILE_SLAM_JOBS:-2}"
duration="${MOBILE_SLAM_TIMEOUT:-900}"
python_path="${MOBILE_SLAM_PYTHON:-python3}"
[[ "$jobs" =~ ^[1-4]$ ]] || { echo 'MOBILE_SLAM_JOBS must be 1..4' >&2; exit 2; }
[[ "$duration" =~ ^[1-9][0-9]*$ ]] || { echo 'MOBILE_SLAM_TIMEOUT must be positive seconds' >&2; exit 2; }

check_mode() {
  case "$1" in
    doctor) "$python_path" scripts/dev/doctor.py --strict --output build/dev-tools/doctor.json ;;
    syntax)
      for file in web/server.js web/js/*.js scripts/dev/*.mjs; do
        node --check "$file" || return "$?"
      done
      for file in scripts/dev/*.sh; do bash -n "$file" || return "$?"; done
      "$python_path" -m py_compile scripts/dev/*.py
      ;;
    eval) "$python_path" -m unittest discover -s tests -p test_audit_metrics.py -v ;;
    browser-contracts)
      node --test tests/browser/contracts.test.mjs tests/browser/transport.test.mjs || return "$?"
      "$python_path" -m unittest discover -s tests/browser -p test_export_trajectory.py -v
      ;;
    mobile-contracts) node --test tests/browser/mobile-audit-contracts.test.mjs ;;
    quick)
      check_mode syntax || return "$?"
      check_mode eval
      ;;
    native)
      timeout "$duration" cmake --preset native-dev || return "$?"
      timeout "$duration" cmake --build --preset native-tests --parallel "$jobs" || return "$?"
      timeout "$duration" ctest --preset native-tests --parallel "$jobs"
      ;;
    wasm-build)
      sdk_env="${EMSDK:-$HOME/emsdk}/emsdk_env.sh"
      [[ -f "$sdk_env" ]] || { echo "Missing Emscripten environment: $sdk_env" >&2; return 2; }
      # Emsdk activation is scoped to this subprocess. Outputs stay out of web/.
      (set +u; source "$sdk_env" >/dev/null 2>&1; set -u
        timeout "$duration" emcmake cmake -S wasm -B build/dev-wasm -G Ninja -DCMAKE_EXPORT_COMPILE_COMMANDS=ON &&
        timeout "$duration" cmake --build build/dev-wasm --target vio_engine --parallel "$jobs")
      ;;
    clangd)
      db_dir="${MOBILE_SLAM_COMPILE_DB:-build/dev-native}"
      [[ -f "$db_dir/compile_commands.json" ]] || { echo "Missing compile database: $db_dir" >&2; return 2; }
      timeout 75 "$python_path" scripts/dev/clangd-probe.py --compile-db "$db_dir"
      ;;
    mcp) timeout 120 node scripts/dev/mcp-probe.mjs ;;
    browser) timeout "$duration" node scripts/dev/browser-baseline.mjs ;;
    *) echo 'Usage: scripts/dev/check.sh {doctor|syntax|eval|quick|browser-contracts|mobile-contracts|native|wasm-build|clangd|mcp|browser|all}' >&2; return 2 ;;
  esac
}

if [[ "${1:-all}" == all ]]; then
  status=0
  check_mode doctor || status=1
  check_mode quick || status=1
  check_mode native || status=1
  exit "$status"
fi
check_mode "$1"
