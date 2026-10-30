# us010-samples v4: harness code (not a registration)

This directory and `slam_refact/web-chunk/tests/us010-v4.mjs` are **code only**. There is no gate document, no manifest, no pin list and no result here; the coordinator
registers the gate after the freeze, and the pin list is made then, by `us010-v4.mjs freeze` (written once; an existing list is refused).

## Grade: REPORT unless the oracle is independent (read before registering)

The criteria S1-S14 and the probe that measures them were written by the same author as the chunk samples. The v3 post-run review (`reviews/opus-xhigh-r1-postrun-or4v6-orient-us010v3.md`)
found that S7-S9, `S11.placeErr` and `S12.maxCubeDrift` are self-consistency checks (M5 leaves them PASS), and that P1 of the GATE conditions is structurally impossible for
this author's oracle. v4 can be a **GATE** only if the **world-lock criterion** (the placed point re-projected with the later poses and compared with the image features) has an
**oracle independent of this author** (plan r4b D2, Fable M-8). With an oracle written by the same author, v4 is **REPORT**, whatever the code here does. Nothing in this
directory decides the grade; the registration does, and a wrong "GATE" label is a defect of the registration.

The world-lock oracle is **not implemented here** (no independent author exists in this lane). The held device (`tools/gate-common/held.mjs`: designation, one-run marker) is not
wired into `run-sample-smoke.mjs` either: a REPORT run needs none; a GATE run would call it around the smoke runs the way the OR-1 and rm4c harnesses do.

## What the code does (from the v3 closure audit B and the v3 post-run review)

| Item | Where | Test |
|---|---|---|
| `mutation.detected === true` (and `null_run.result === "PASS"`, no run on record) is required **before** the first run; v3 ran with `null` | `us010-v4.mjs check-manifest`, called by `run.sh` step 0 and by `freeze` | `tests/us010-v4.test.mjs` US10V4-1, -3 |
| the core of the registered run is pinned (chunk, samples, measuring and driving scripts, gate-common helpers, this `run.sh`) together with the external bytes (node, Chromium binary and wrapper, SDK `xr.js`, three.js, A-Frame, the engine files, `--extra` files such as the capture index) | `us010-v4.mjs freeze` | US10V4-2 |
| the pin list is written **once** (exclusive create, read-only); a second write is refused. v3 regenerated `SHA256SUMS` after the registered runs (U10-2) | `freeze` | US10V4-2, -5 |
| a changed, missing or **added** core file and any changed external byte stops the sequence | `us010-v4.mjs verify`, called before every group of runs and at the end | US10V4-4 |
| the replay driver is a pinned repository file (`run.sh`) and every replay run records its sha256 (`--driver`); v3 used an unpinned scratchpad script (U10-5) | `run.sh`, `run-exec-model.mjs --driver`, `run-sample-smoke.mjs --driver`, `run-us010b-series.mjs` (records its own sha256) | US10V4-2 (driver sha in the list) |
| M6 (position scaled by 0.5 on the pose only): v4 registers `S10` and `S10b` as the checks that must FAIL, written from the v3 observation (three.js FAIL S10 S10b S13, A-Frame FAIL S10 S10b) - a **post-result registration of a new version**, labelled so in `CONTROL_EXPECTATIONS.v4.basis`; the v3 expectation (nothing fails) is left as it was | `sample-smoke-criteria.mjs` (`CONTROL_EXPECTATIONS`, `controlOutcome`), `run-sample-smoke.mjs --expectations v3|v4` | US10V4-6 |
| the evidence root, the series directory and the summary name are arguments, the summary is written once (v3 hardcoded `us010-20261020` and overwrote the summary) | `run-exec-model.mjs`, `run-us010b-series.mjs`, `run-pose-nullrun.mjs`, `summarize-us010b.mjs` | defaults keep the v3 behaviour (`sample-smoke-criteria.test.mjs`) |

## Not done, deliberately

- No world-lock criterion, no oracle, no gate document, no manifest, no seed, no pin list, no first run, no device measurement.
- `S10`'s margin (v3: 56-93 % of the tolerance across runs) and the S10b FAIL in 1 of 5 series runs (v3 §7-6) are open observations, not changed here; no tolerance was touched.
- The capture (`capture-<label>/`) is an input made by `run-exec-model.mjs --phase capture` before the freeze and pinned through `freeze --extra <capture>/index.json`.

## Use (the coordinator, after the freeze)

```bash
node slam_refact/web-chunk/tests/us010-v4.mjs freeze --out build/<lane>/us010-v4-pins.json --manifest <manifest> --engine-dir <engine dir> \
     --snapshot-dir build/<lane>/us010-v4-snapshot --extra build/<lane>/evidence/capture-<label>/index.json
bash slam_refact/tools/gates/us010-samples-v4/run.sh --dry-run     # with the environment of its header: step 0 only
bash slam_refact/tools/gates/us010-samples-v4/run.sh               # the registered sequence (one new evidence root; labels are never reused)
```
