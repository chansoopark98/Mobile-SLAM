#!/usr/bin/env bash
set -euo pipefail
dev_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd -- "$dev_dir/../.."
case "${1:-}" in
  playwright)
    browser_path="${MOBILE_SLAM_BROWSER:-$(command -v google-chrome || true)}"
    if [[ -z "$browser_path" ]]; then
      echo 'Set MOBILE_SLAM_BROWSER to a Chrome executable.' >&2
      exit 2
    fi
    exec node "$dev_dir/node_modules/@playwright/mcp/cli.js" \
      --headless --isolated --executable-path "$browser_path" \
      --output-dir "build/dev-tools/playwright" --caps devtools
    ;;
  context7)
    exec node "$dev_dir/node_modules/@upstash/context7-mcp/dist/index.js" --transport stdio
    ;;
  *) echo 'Usage: scripts/dev/mcp-server.sh {playwright|context7}' >&2; exit 2 ;;
esac
