---
name: mobile-slam-validation
description: Validate Mobile-SLAM native C++, WASM, browser execution and development tools with reproducible evidence. Use for this repository's regression checks, profiling and accuracy experiments; distinguish smoke execution from real-device tracking and trajectory accuracy.
---

# Mobile-SLAM validation

- Start in the repository root. Read `scripts/dev/README.md` for current commands, `docs/audit/2026-10-06/` for the initial baseline, and `docs/refactor/2026-10-06/` for current implementation evidence. Refactor builds/evidence use `build/refactor-{native,sanitize,wasm,evidence}`; follow the latest manifest.
- Record the source revision, local changes, compiler/build flags and hashes of the served JS/WASM before comparing results. `python3 scripts/dev/doctor.py --output build/dev-tools/doctor.json` records tool versions and hot-file fingerprints; it does not record all source files or establish build provenance.
- Preserve existing binaries and datasets. Use isolated build directories; deploy only the manifest's active artifact set. Current candidate uses a JS loader with embedded WASM (`single_file_embedded_wasm:true`); an old external `.wasm` is not active provenance. Future external paired mode requires matching both files.
- Default to 2 build jobs. Set `MOBILE_SLAM_JOBS` only when additional resource usage is justified.
- Use `/mnt/backup/SLAM` as the canonical read-only dataset root, overridden by `MOBILE_SLAM_DATA_ROOT`. Doctor inspects only bounded shallow presence; do not scan/hash/download the shared datasets by default. Distinguish that root from the separately restored official repo room1 fixture. Existing parity/browser fixture paths are not changed by this environment variable.

## Choose the evidence for the claim

- Native regression: use the current runbook/build manifest; enumerate CTest-discovered suites and keep every outcome, individual GoogleTest counts and failed assertions. Current refactor has nine suites; missing binaries, empty suites and mandatory TODO/skip are failures. An existing failing test is baseline evidence, not a passing gate.
- JS syntax: `bash scripts/dev/check.sh syntax`; plain JavaScript has no configured TypeScript typecheck or ESLint. Do not report syntax checks as either one.
- Browser contracts: `bash scripts/dev/check.sh browser-contracts`; known TODOs remain reproduced defects even if Node exits zero. Keep normal passes, TODO count and exporter analytic passes separate.
- WASM build: `bash scripts/dev/check.sh wasm-build`; link success alone does not prove the browser served that build.
- Browser execution: `npm --prefix scripts/dev run browser:baseline`; review the harness's dataset/frame limit, capabilities, network errors and source hashes. Chromium device emulation does not prove a physical phone's camera/IMU accuracy, WebGPU availability or sustained thermal performance.
- Native C++ language tools: `MOBILE_SLAM_COMPILE_DB=build/dev-native bash scripts/dev/check.sh clangd`; this probes genuine clangd using a compilation database. OMX's JavaScript/TypeScript code-intel and `/usr/bin/sg` are not C++ analysis tools.
- MCP connectivity: `bash scripts/dev/check.sh mcp`; verify initialize, tools/list and actual tool results. A configured server or plugin listing alone is insufficient.

## SLAM performance and accuracy

- Report capture/IMU timestamps, ordering, clock-domain assumptions, processed/dropped frame counts, queue wait and capture-to-pose latency separately from engine processing time.
- Keep p50/p95/p99 latency, initialization outcome, reset/lost-tracking count, memory and thermal/device/browser/build metadata. Use explicit dataset duration and seeds where available.
- For trajectory claims, require ground-truth timestamps/frames and synchronization; declare alignment (`SE(3)` or `Sim(3)`), scale handling, warm-up and lost-track coverage. Evaluate ATE/RPE plus rotation and scale error; a plotted path, visual plausibility or zero process exit is not accuracy evidence.
- Do not call bounded desktop replay production-level. Establish a held-out multi-device protocol and thresholds before the change can make that claim.
- For live mobile pose jitter, record raw IMU/capture/result times and translation/rotation jumps before proposing smoothing. A smooth displayed path can hide estimator error or add latency; validate stationary and controlled-motion behavior separately.
- Use the separate `/audit-mobile.html` observation page and `scripts/dev/serve-mobile-audit.py` command from the runbook. Its `mobile-slam-observation-v1` JSON is a bounded local download, not complete sensor replay or independent GT. Localhost mock/TLS tests do not establish trusted HTTPS or sensor behavior on a physical phone. Check stop/limit/drop metadata before drawing conclusions.
- Use the project Context7 MCP for unfamiliar library APIs, then verify its source links against the version actually used. The keyless documentation service may be rate-limited. Use the isolated Playwright MCP for browser diagnostics; avoid reusing a personal browsing profile.

## Report

- Separate `VERIFIED` (observed execution), `CONFIGURED` (setup exists) and `BLOCKED`/`UNVERIFIED` (failed or absent evidence).
- Include commands, evidence paths, changed files and known limits. Preserve failing baseline assertions until a scoped fix has independent expected values.
