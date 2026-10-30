# E5 diag (it-scale-v4 L0 diagnosis; harness only, REPORT tier)

Replays an exact-input E5 bundle with the engine unchanged and records the image-target tracker internals that `images.jsonl` lacks.
No file under `slam_refact/engine/**` is touched: `build-diag.sh` compiles these two sources against the already-built it-scale-v4 engine
libraries (snapshot of git 19fd011) and links with `--wrap` on `c8::ImageTrackingState::track`, `solveImageTargetHomography`, `robustPnP`, `robustP2P`.

| file | role |
|---|---|
| `e5_diag_main.cc` | copy of `../e5_replay_main.cc`; sets `ImageTrackingState::AGGRESSIVELY_FREE_MEMORY=false` (scratch kept, results unchanged), adds locIn/gloIn/scale/target-view position to `images.jsonl`, `ITPROBE_STOP_AFTER=<frame>` ends early |
| `itprobe.cc` | per `track()` call: states, feature/ROI counts, local/global match and inlier counts, solver stages (hom/pnp/p2p: ok, n, inliers, RNG hash), global inlier split pyramid/ROI. `ITPROBE_OUT=<new file>`. Optional sensitivity experiment `ITPROBE_RNG_SKIP=<frame>:<k>` (discards k draws of the shared RNG before the global solver of that frame; the run is then NOT an engine-equivalent replay) |
| `build-diag.sh`, `run-probe.sh`, `run-sens.sh` | build (jobs 1, output `$B/diag/`), one probe replay, one RNG-sensitivity replay |
| `show.py`, `sens.py`, `l0.py` | print probe lines, summarise sensitivity runs, first lost event >= 123 |

Without `ITPROBE_RNG_SKIP` the replay is engine-equivalent: for a-o1 and a-o2 (wt0) the detected/events of all 418 processed frames equal the stock sr_e5_replay output of the v3 run, and the --wrap build's images.jsonl is byte-identical to the non-wrap diag build's.

## Original-SDK probe (approved 2026-10-23; developed and unit-checked, NOT yet run against the SDK)

Records, from the instrumented original `xr-slam.wasm`, the shared `RandomNumbers` mt19937 state per frame and the return values / inlier counts of the
image-target solvers, to find the first frame where original and engine differ (hypotheses H-A RNG coin, H-B inputs). No file under `tools/re/runtime`
or `engine/**` was edited.

| file | role |
|---|---|
| `instrument_it_wat.py` | WAT patcher: global 9 `sr_probe_it` log buffer; func 784 entry (RNG state, tag 3), wrappers for func 992 (nextTargetViewPose; entry state, `found`; tag 2), func 1022 (solveImageTargetHomography; tag 1), func 1025 (robustP2P inside 992; tag 4). robustPnP is inlined in 992 |
| `itprobe_orig.js` | page init script (after memprobe.js): reads the buffer + the RNG state after every processStagedFrame |
| `make_run_original_itprobe.py` -> `run_original_itprobe.mjs` | generated copy of `../run_original.mjs` (+ `--it-probe-wasm`, output allowed under the diag build dir, writes `itprobe.jsonl.gz`); `diff` shows the 39 changed lines |
| `orig_itprobe_driver.py` | stages prep / orig / bundle / engine / analyze (`--dry-run` prints the commands; stage `orig` launches Chromium under `flock build/c1-chromium.lock`) |
| `orig_itprobe_analysis.py`, `mt_tools.py`, `mt_check.cc` | compare (first RNG-state / hom-call / found difference), seed (original state -> libstdc++ text for `ITPROBE_RNG_SET`), summary; libc++ mt19937 emulation and conversion |
| `unit_check_itprobe.mjs`, `unit_check_itprobe_js.mjs`, `test_orig_itprobe_analysis.py` | Node/python unit checks (no browser) |

Go command (after the coordinator's go): `python3 -I orig_itprobe_driver.py --seq <E5 seq dir> --out <new dir under build/slam-refact-full-20261019/it-scale-v4/diag> --k 3 --seed-frames 46,123`.
Recommended first launch: `--k 1 --max-frames 40 --stages prep,orig` as a smoke test (itprobe.jsonl.gz has tag-3 slots and a post state per fresh frame).

### Results of the first run (2026-10-23, `build/slam-refact-full-20261019/it-scale-v4/diag/orig-run1/`, seq-a, 3 new original runs)
`orig_itprobe_analysis.py draws` (draws consumed since the engine re-seed at the start recenter, frame 4) and `pnp_draws.py` (exact draw-count model of robustPnP's
sampling, reproduces the engine's 586 draws for N = 199 at o1 frame 46) are the tools behind the numbers in the report; analysis JSONs are in `orig-run1/analysis/`.

### ROI pixel diagnosis (2026-10-23, run `.../diag/orig-gl2` = original with `--gl-capture` + instrumented wasm, and `orig-gl1`)
`glcap_orig.js` (+ `--gl-capture <n>:<rows>` of `run_original_itprobe.mjs`): wasm-heap pyramid readbacks (`getBufferSubData` >= 4 MiB = the bytes the tracker consumes) and the
GL op trace (viewport, `mvp` uniforms, draws).  Engine side (`sr_e5_atlas`, itprobe.cc): `ITPROBE_ATLAS_DIR` dumps the CPU atlas + ROI list per frame, `ITPROBE_ROI_SS=1` redraws every planar ROI
slot with `ss_roi_model.h` (SwiftShader quad model) and reruns stages 2/3, `ITPROBE_INJECT_DIR` injects the original's atlas bytes (`cap-boot.rgba`, `cap-fNNNN.rgba`).
`glcap_analysis.py compare` = byte diff per region/slot; `roi_model_test.cc` (+ `roi_case_prep.py`, `build-roimodel.sh`) = candidate GPU models against one slot's original bytes.

### Pose -> ROI mvp chain (2026-10-23, run `.../diag/orig-mvp2`, engine = roifix build, `build-diag-roifix.sh`)
`instrument_it_wat.py` also hooks func 670 (glImageTargetWarp, tag 5: targetK, camK, pose, result), func 661 call sites in func 938 (scaleTranslation, tag 7), func 992 after the call (nextPose, tag 6) and the
camMotion before/after of func 1022 (tag 1 payload).  Engine side (`itprobe.cc`): `ITPROBE_WARP_OUT` (glImageTargetWarp inputs/result), `scaleT` and `hom` pre/post matrices in the stage list.
`warp_compare.py` and `chain_compare.py` compare them per frame (float bit patterns, ulp distance).  Result: with `--world-tracker 1` the pose and the warp are bit-identical for frames 0-69.
