#!/usr/bin/env bash
set -euo pipefail
ops_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${MOBILE_SLAM_PYTHON:-python3}" "$ops_dir/serve.py" "$@"
