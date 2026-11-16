#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
cd -- "$root"
jobs="${MOBILE_SLAM_JOBS:-2}"
duration="${MOBILE_SLAM_TIMEOUT:-900}"
python_path="${MOBILE_SLAM_PYTHON:-python3}"
[[ "$jobs" =~ ^[1-4]$ ]] || { echo 'MOBILE_SLAM_JOBS must be 1..4' >&2; exit 2; }
# Test processes run at once by refact-native ctest (not compile jobs; those stay at MOBILE_SLAM_JOBS).
test_jobs="${MOBILE_SLAM_TEST_JOBS:-16}"
[[ "$test_jobs" =~ ^([1-9]|[12][0-9]|3[0-2])$ ]] || { echo 'MOBILE_SLAM_TEST_JOBS must be 1..32' >&2; exit 2; }
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
    eval)
      "$python_path" -m unittest discover -s tests -p test_audit_metrics.py -v || return "$?"
      "$python_path" -m unittest discover -s tests -p test_synthetic_vio.py -v || return "$?"
      "$python_path" -m unittest discover -s tests -p test_flip6_lean_tools.py -v
      ;;
    browser-contracts)
      node --test tests/browser/*.test.mjs || return "$?"
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
    refact-native)
      # Dependency and evidence paths come only from the environment; unset values fall back to the engine CMake defaults.
      local rn_dir="${SLAM_REFACT_NATIVE_BUILD_DIR:-build/dev-refact-native}"
      local -a rn_cfg=(-G Ninja -DCMAKE_BUILD_TYPE=Release -DSLAM_REFACT_WERROR=ON -DSLAM_REFACT_LOOP_CLOSE=ON
        -DSLAM_REFACT_BUILD_TESTS=ON -DSLAM_REFACT_BUILD_HARNESS=ON)
      if [[ -n "${SLAM_REFACT_CERES_SOURCE_DIR:-}" ]]; then rn_cfg+=("-DSLAM_REFACT_CERES_SOURCE_DIR=$SLAM_REFACT_CERES_SOURCE_DIR"); fi
      if [[ -n "${SLAM_REFACT_GTEST_SOURCE_DIR:-}" ]]; then rn_cfg+=("-DSLAM_REFACT_GTEST_SOURCE_DIR=$SLAM_REFACT_GTEST_SOURCE_DIR"); fi
      if [[ -n "${SLAM_REFACT_CERES_SOURCE_DIR:-}${SLAM_REFACT_GTEST_SOURCE_DIR:-}" ]]; then rn_cfg+=(-DFETCHCONTENT_FULLY_DISCONNECTED=ON); fi
      if [[ -n "${SLAM_REFACT_EVIDENCE_ROOT:-}" ]]; then rn_cfg+=("-DSLAM_REFACT_EVIDENCE_ROOT=$SLAM_REFACT_EVIDENCE_ROOT"); fi
      timeout "$duration" cmake -S slam_refact/engine -B "$rn_dir" "${rn_cfg[@]}" || return "$?"
      timeout "$duration" cmake --build "$rn_dir" --parallel "$jobs" || return "$?"
      rm -f "$rn_dir/unit.junit.xml"
      # ctest exits nonzero for the known failure; the ledger decides from the JUnit file.
      timeout "$duration" ctest --test-dir "$rn_dir" --no-tests=error -L '^unit$' --parallel "$test_jobs" \
        --output-junit "$PWD/$rn_dir/unit.junit.xml" || true
      "$python_path" scripts/dev/ci-ledger.py --junit "$rn_dir/unit.junit.xml" --mode refact-native \
        --known slam_refact/engine/tests/known-failures.json \
        --tool-version "cmake=$(cmake --version | head -n1)" --tool-version "node=$(node --version)"
      ;;
    refact-web)
      local web_status=0
      mkdir -p build/dev-refact-web
      rm -f build/dev-refact-web/web.junit.xml
      node --test --test-reporter=spec --test-reporter-destination=stdout \
        --test-reporter=junit --test-reporter-destination=build/dev-refact-web/web.junit.xml \
        slam_refact/web-chunk/tests/*.test.mjs slam_refact/web-smoke/*.test.mjs || web_status=$?
      "$python_path" scripts/dev/ci-ledger.py --junit build/dev-refact-web/web.junit.xml --mode refact-web \
        --tool-version "node=$(node --version)" || return "$?"
      return "$web_status"
      ;;
    refact-tools)
      "$python_path" slam_refact/engine/tools/check_vendor.py --data || return "$?"
      "$python_path" slam_refact/engine/tools/check_std_algos.py
      ;;
    rs0-fast|rs0-full)
      # RS-0 parity gate (scripts/dev/rs0-gate.py): parallel replays + replay cache. rs0-full is the merge gate.
      local tier="${1#rs0-}"
      local rs_native="${SLAM_REFACT_RS0_NEW_NATIVE:-${SLAM_REFACT_NATIVE_BUILD_DIR:-build/dev-refact-native}/harness/slam_refact_replay}"
      local -a rs_args=(--tier "$tier" --new-native "$rs_native" --label "check-$1"
        --out "build/rs0-gate/$1-$(date +%Y%m%dT%H%M%S)")
      if [[ -n "${SLAM_REFACT_RS0_ROOT:-}" ]]; then rs_args+=(--root "$SLAM_REFACT_RS0_ROOT"); fi
      if [[ -n "${SLAM_REFACT_RS0_JOBS:-}" ]]; then rs_args+=(--jobs "$SLAM_REFACT_RS0_JOBS"); fi
      if [[ -n "${SLAM_REFACT_RS0_NEW_WASM:-}" ]]; then
        rs_args+=(--new-wasm "$SLAM_REFACT_RS0_NEW_WASM")
      elif [[ "$tier" == full ]]; then
        echo 'rs0-full needs SLAM_REFACT_RS0_NEW_WASM (dir with slam_refact_engine.js/.wasm)' >&2; return 2
      else
        rs_args+=(--sections parity,off,exact)
      fi
      "$python_path" scripts/dev/rs0-gate.py "${rs_args[@]}"
      ;;
    mcp) timeout 120 node scripts/dev/mcp-probe.mjs ;;
    browser) timeout "$duration" node scripts/dev/browser-baseline.mjs ;;
    *) echo 'Usage: scripts/dev/check.sh {doctor|syntax|eval|quick|browser-contracts|mobile-contracts|native|refact-native|refact-web|refact-tools|rs0-fast|rs0-full|wasm-build|clangd|mcp|browser|all}' >&2; return 2 ;;
  esac
}

if [[ "${1:-all}" == all ]]; then
  status=0
  check_mode doctor || status=1
  check_mode quick || status=1
  check_mode native || status=1
  check_mode browser-contracts || status=1
  check_mode refact-tools || status=1
  check_mode refact-web || status=1
  check_mode refact-native || status=1
  exit "$status"
fi
check_mode "$1"
