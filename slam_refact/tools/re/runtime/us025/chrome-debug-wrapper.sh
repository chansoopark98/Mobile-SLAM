#!/usr/bin/env bash
# BROWSER_EXECUTABLE wrapper for hang diagnosis (US-025 LC-G): starts the same /usr/bin/google-chrome plus a loopback-only DevTools port, so that
# cdp_pause.mjs can pause a renderer that is stuck in an endless loop and print the call stack. Playwright keeps using its own --remote-debugging-pipe.
exec /usr/bin/google-chrome "$@" --remote-debugging-port="${LCG_CDP_PORT:-9333}" --remote-debugging-address=127.0.0.1
