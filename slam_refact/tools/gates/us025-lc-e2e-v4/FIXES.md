# us025-lc-e2e-gate v4: execution-machinery fixes (fork of v3)

Source of the defect list: `slam_refact/docs/design/reviews/opus-architecture-review-gate-docs-2026-10-29.md` (section 1 table = findings #1-#15, section 2 = G-16 ...), original text of the 15 findings in the
session scratchpad `fable-gate-findings.txt`. Status of this directory: **dev tool, not registered, not designated, never run on a registration scenario** (no gate, dev, Chromium or replay run was made;
unit tests only). v1/v2/v3 directories and the shared harness files (`make_variant.py`, `us026/make_bundles.sh`, `add_params.py`) are untouched.

## Judgement files (D-V3-LOCK kept)

`judge.mjs`, `compare_full_us025.py`, `test_compare_full_us025.py`, `judge.test.mjs`, `make_lc_manifest.mjs` are byte-identical to v3:

```
for f in judge.mjs compare_full_us025.py test_compare_full_us025.py judge.test.mjs make_lc_manifest.mjs; do cmp slam_refact/tools/gates/us025-lc-e2e-v{3,4}/$f; done     # exit 0, no output
```

`gate_lc.test.mjs` also checks their sha256 against the v3 directory. No decision constant, run count, tolerance or verdict exit changed (the same test compares `decisionConstants()` and `configRecord()` with v3: only
`out_default` and `slot_lock_prefix`, i.e. the v4 names, differ).

## Names that became v4

`GATE_VERSION = 4` (gate id unchanged), tool path `us025-lc-e2e-v4/`, designation variable `SLAM_REFACT_US025_LC_V4_DESIGNATION_SHA256`, default output `.../gate-runs/us025-lc-e2e-gate-v4`, slot locks
`build/us025-lc-v4-slot-<k>.lock`, dev root `.../us025/lc-e2e-v4-dev`, report schema `slam-refact-us025-lc-e2e-report-v4`, mode `v4-two-phase`, run.sh `GATE_VERSION=4`.
`make_lc_static.py` / `make_snapshot.sh` got the v4 names and paths only; the narrative keys of the v3 registration (`exposure`, `classification`, `supersedes`) in `make_lc_static.py` are still the v3 text:
**the v4 registration text must be written by the registration lane** (not part of this task).

## Pins (bundle step)

`MAKE_BUNDLES` = `us025-lc-e2e-v4/make_bundles.sh` (the us026 script with exactly one changed line: `divergence/make_variant_v2.py`, same CLI; a non-monotonic input makes `make_variant_v2.py` exit 21 and write
no bundle, `set -e` passes the 21 on, `makeBundlesFor` records `bundle construction failed (exit 21: ...)` for the run AND sets the scenario-level `infraStop` (coordinator decision M-1): the scenario class is INFRA, the gate cannot PASS, never an engine FAIL). `PINNED_FILES` names `make_bundles.sh`, `make_variant_v2.py` (written by
another worker, exists) and the new `original_run_info.py`; it no longer names `make_variant.py` / `us026/make_bundles.sh`.

## How to run the tests

```
node --test --test-timeout=120000 slam_refact/tools/gates/us025-lc-e2e-v4/*.test.mjs                      # 106 tests: gate_lc.test.mjs (56, v3 suite adapted), gate_lc_v4.test.mjs (30, new), judge.test.mjs (20)
python3 -B -m unittest test_analyze_equivalence test_compare_full_us025 test_v4_fixes                       # from this directory: 73 + 15 + 14 tests
```

## Baseline ("fails before the fix")

The new tests load the implementation from `US025_LC_IMPL_DIR` (node) / `US025_LC_TOOL_DIR` (python), default = this directory. Pointing them at `../us025-lc-e2e-v3` runs the identical tests against the v3 code:

```
US025_LC_IMPL_DIR=../us025-lc-e2e-v3 node --test .../gate_lc_v4.test.mjs        # observed (first 26 tests): 26 of 26 fail (e.g. #1 hangs until the test timeout; #9 uncaught `spawn flock ENOENT`)
US025_LC_TOOL_DIR=../us025-lc-e2e-v3 python3 -B -m unittest test_v4_fixes       # observed: 10 of 14 fail/error (the 4 that pass are companions: they pin behavior that must NOT change, or test the new original_run_info.py, which v3 has not)
```

Where the v3 failure is "export / file does not exist" instead of a behavior difference (#4 `runChild` not exported, G-26 `makeWatchdog`, the bundle tests, G-21 run.sh text) the test still documents the new contract; the behavioral
failures (hang, uncaught exception, wrong exit code, wrong record) are the rest.

## Defect id -> fix -> test names

Node tests are in `gate_lc_v4.test.mjs` (title starts with the id), python tests in `test_v4_fixes.py`.

| id | fix (file) | regression test(s) |
|---|---|---|
| #1 | `originalWorker`: the owner role, `ctx.running.add` and the exclusive prerun write are inside the `try/finally`; `sensorsDone()` always runs, `ctx.running`/`ctx.roots` are cleared (`gate_lc.mjs`) | `#1 the sensors owner throws at the exclusive prerun write (2 slots) ...` |
| #2 | (a) the dev summary exports `replayExits` (`summarizeDev`); (b) `analyze_equivalence.py` rule **R-6**: a non-INFRA lc=1 run without ON verdict and without original success is ambiguous (N = 1), registered rule for success runs (replay exit != 0 = FAIL, G6) unchanged; (c) G-18 fills `originalSuccess` so such a success run is judged by the registered rule | python `test_2_non_success_run_with_failed_on_replay_is_ambiguous_not_a_silent_non_fail`, `test_2_an_old_summary_without_replay_exit_codes_is_also_ambiguous`, `test_2_companion_a_success_run_with_failed_on_replay_is_the_registered_fail_of_g6`; node `G-18 / #2 ... replayExits` |
| #3 | `runPhaseB` works scenario by scenario (bundles serial, replays + comparators in the pool of J inside the scenario), so the first report exists after the first scenario; per-run journal `<scenario>/phase-a.jsonl` (append + fsync per original run, also for runs that did not run); the report writer is created before the first marker and the outer `catch` writes the INFRA report of every claimed scenario without report (`emergencyReports`). **Not done (protocol change, needs v4 registration text + pre-review): rebuilding a report from the journal after a crash** | `#3 phase B works scenario by scenario ...`, `#3 phase A leaves a fsynced journal line per original run ...`, `#3 a failure after phase A (here: a lock that cannot be released) still gives every started scenario its INFRA report ...` |
| #4 | `runChild` closes the log fd once (`closeOnce`) and is exported | `#4 runChild: a failed spawn resolves once ...` |
| #5 | the claim wrapper sets `sc.claimed`/`sc.marker` right after `claim()` returns; subdirectories are made with `{recursive: true}`; `claimScenarioOnce` also sets `claimed` when the wrapper threw after the marker was taken | `#5 a directory that appears after the hold check ...`, `#5 whatever fails AFTER the marker was taken ...` |
| #6 | `abortPhaseA` no longer flags claimed scenarios; `finalizeAbort(ctx)` (after `settleUnfinishedRuns`) flags only claimed scenarios that have a run that did not run. **Judge-adjacent: fewer scenarios end INFRA, more are evaluated (PASS conditions unchanged). Must be named in the v4 registration before any result** | `#6 a claim that fails for the NEXT scenario does not flag a scenario whose planned runs all completed as INFRA ...` |
| #7 | the claim wrapper runs `recheckPins` before EVERY marker; a problem is `HELD: pins: before the marker of <scenario>: ...` and no marker is taken | `#7 the pins are re-checked right before EVERY marker ...` |
| #8 | `runQueue(items, n, worker, stopped)` with the shared `ctx.failed` set by `runPhaseA`; `claimScenarioOnce` re-tests `ctx.failed`/`ctx.aborted` after the readiness wait | `#8 a worker failure stops the claim of a slot that is waiting for readiness ...` |
| #9 | `holdLock` handles the spawn `error` event and resolves `null` (= lock not obtained, exit 5 path); `measure_run.mjs` handles it too | `#9 holdLock: a flock that cannot be spawned gives null ...`, `G-26 / #9 measure_run.mjs uses the staged watchdog ...` |
| #10 | `cmdDev` catches a phase A error per block, aborts, settles, writes `dev-summary.json` with `infraStop: true` and `errors` | `#10 dev: a worker error in a later block does not lose the summary ...` |
| #11 | `supervise` second stage = `killTree`: every descendant of the runner (from `ps`) is killed, then the runner's group; first stage also uses the CPU of the last second from `/proc/<pid>/stat` (`sampleRecentCpu`, injectable), not only the lifetime `pcpu`; `watchdog.killedTree` recorded | `#11 the watchdog's second stage kills the DESCENDANTS of the runner ...` (real processes), `#11 a renderer that wedged late ...` |
| #12 | `apply_rerun` is fail-closed (targets of preregistration 8.1 only, before-values outside PASS/PASS_DECLARED/FAIL, `full` never, INFRA/unknown runs never); rejected entries are listed in `rerun_rejected` and in the notices, nothing is applied for them | python `test_12_a_registered_fail_cannot_be_turned_into_pass_by_the_rerun_file`, `test_12_full_is_never_a_rerun_target_and_a_non_success_run_is_none`, `test_12_an_infra_run_and_an_unknown_run_are_no_targets_and_originalSuccess_is_not_overwritten`, `test_12_companion_the_two_registered_rerun_cases_are_still_accepted` |
| #13 | `t0` / `rec.startedAt` are taken after the wait for the owner's sensors file; the wait is recorded as `rec.sensorsWaitS` | `#13 the run record of a slot that waited for the owner's sensors file starts AFTER that file is complete ...` |
| #14 | the `resource-wait` event also logs `precheck.processes` and `ownHeadless` | `#14 a failed readiness check logs WHICH process blocked ...` |
| #15 | a HELD at claim time (only HELD errors) gives exit 3 and log status HELD | `#15 a HELD raised at claim time keeps its class: exit 3 (not 1)` (and the adapted `R-1 / C-1 in run-all` test in `gate_lc.test.mjs`) |
| G-16 | after the sensors wait a non-owner re-tests `ctx.aborted`, `ctx.failed`, `sc.infraStop`, and re-runs the readiness check when it really waited | `G-16 a slot that waited for the sensors file starts no run when phase A was aborted meanwhile ...` |
| G-18 | `rec.originalRun` falls back to `original_run_info.py` (loads the original run alone with `compare_us025.load_original_lc`; the judge does not read the record) | `G-18 / #2 the original run's success comes from the original run data ...` (node), python `test_g18_success_comes_from_the_original_run_data_alone`, `test_g18_an_unreadable_original_is_recorded_as_an_error_not_raised`, `test_g18_the_cli_writes_a_new_file_and_never_overwrites` |
| G-20 | `unobserved_runs`: the observed time ends GAP_S after the last sample; the `end` line does not extend it | python `test_g20_the_end_line_does_not_make_a_stretch_after_the_last_sample_observed`, `test_g20_companion_a_run_that_ends_within_the_gap_limit_after_the_last_sample_is_observed` |
| G-21 | `run.sh`: exit 5 of `run-all` is remembered (`ANY5`), the reports that exist enter the verdict step, the final exit stays 5; `cmdRunAll` returns `consumed` (scenarios whose marker is taken) | `G-21 run.sh: an exit 5 of run-all ... still runs the verdict step ...`; `#5 whatever fails AFTER the marker ...` asserts `consumed` |
| G-22 | `compareTrees(ref, out, {runs})`; `phase-b --runs` compares only the replayed runs | `G-22 compareTrees with a run filter ...` |
| G-23 | the readiness check before the marker is the check of the claiming run (`sc.firstCheck`): no second wait (up to 4 h) after the marker | `G-23 the readiness check right before the marker is the check of the first run ...` |
| G-25 | `make_lc_static.null_run_result`: an empty / missing `--full-null` directory is FAIL | python `test_g25_an_empty_full_null_directory_is_not_a_passed_measurement`, `test_g25_companion_a_directory_with_passing_files_passes_and_one_failing_file_fails` |
| G-26 | `makeWatchdog` (staged: renderers at the cap, whole tree + group after the grace) used by `measure_run.mjs`; flock spawn error handled | `G-26 the measurement tool's staged watchdog ...`, `G-26 / #9 measure_run.mjs uses the staged watchdog ...` |
| M-1 | exit 21 of `make_bundles.sh`: run `infra` + scenario `infraStop` (`makeBundlesFor`, option `makeBundles` for injection) | `M-1 end to end: make_bundles exit 21 ...` (scenario class INFRA, `aggregate` verdict INFRA, no replay) |
| M-2 | `abortPhaseA(ctx, reason, {lockLost: true})` marks the runs in flight (`rec.inFlightAtAbort`); they end as INFRA (`lock lost during this run`) AND `finalizeAbort` flags their scenario (scenario `infraStop`, class INFRA, gate cannot PASS), even when the in-flight run is the last of its scenario; scenarios whose runs all completed before the abort are evaluated (#6) | `M-2 a lock lost during a run ...` |
| N-4 | `runQueue` carries all slot errors (`allErrors`); `phaseAErrorMessages` lists one entry per error, so `heldOnly` sees every error | `N-4 / #15 runQueue throws the first error and carries the errors of ALL slots ...` |
| N-5 | the G-21 test executes steps 4 and 5 of the real `run.sh` text with a stubbed tool (exit 5 / 0 / 1) | `N-5 / G-21 run.sh executed ...` |
| bundles | `make_bundles.sh` + `make_variant_v2.py` pins (see above) | `bundles: the pinned set names the v4 make_bundles.sh and make_variant_v2.py ...`, `bundles: make_bundles.sh of v4 is the us026 script with ONE change ...` |

## Existing v3 tests whose expectation changed on purpose (all in `gate_lc.test.mjs`, each marked `v4` in a comment)

- `phase A: resources never ready before a marker ...`: one ok check = marker + run 1 (G-23).
- `run-all: the Chromium and slot locks are released before phase B ...`: the recheck stub also sees the claim-time re-check (#7).
- `R-1 / C-1 in run-all`: exit 3 for a HELD at claim time (#15).
- `abortPhaseA / settleUnfinishedRuns`: the scenario flag comes from `finalizeAbort` (#6).
- `R-2 in run-all`: the first re-check call (before the marker) is clean, the drift appears for the check between the phases (#7).
- identity tests: version 4, v4 paths, the byte-identity test compares with the v3 directory.

## Findings of the review not fixed here (not in the task list)

G-17, G-19, G-24 (PARTIAL, record-level / no effect on N = 1), G-27 (efficiency: repeated chrome/playwright sha per report and `checkHold` per scenario). Protocol-level items of the review (journal-based report
reconstruction, splitting `dev_lc.mjs` from `gate_lc.mjs`) need a v4 registration decision and are not done.

## Limits (what was NOT verified)

UNVERIFIED: any real run (gate, dev, Chromium, replay); `make_variant_v2.py` behavior (other worker; only its path, CLI flags and exit code 21 as written in its header are relied on); the real `recheckPins` cost per claim
(seconds, not measured); `sampleRecentCpu` against a real wedged Chromium renderer (tested with a fake sampler and a real `sleep` process); `run.sh` end to end (text tests only, `bash -n` passes).

## Classification changes to name in the v4 registration document BEFORE any result (review `review-us025-lc-e2e-v4-runner-2026-10-29.md` section 4, plus decisions M-1 / M-2)

Direction "easier to PASS" is marked (+); everything else is stricter or neutral. Judging constants, run counts, tolerances, verdict exits: unchanged (judge files cmp-identical).

1. (+, decided M-1) Non-monotonic input bundle (`make_variant_v2.py` exit 21): v3 = replay exit 2 = engine FAIL (US025-M1/M2). v4 = the scenario is INFRA (`infraFailure`), the gate verdict cannot be PASS and is not FAIL. Normal inputs give byte-identical bundles (bundle.json argv aside).
2. (+, decided M-2, final) #6: at an abort only claimed scenarios with a run that did not run, and, when the abort is a LOST LOCK, claimed scenarios with a run in flight (that run is INFRA too), get the scenario-level INFRA flag. Only scenarios whose runs all completed before the abort are evaluated; v3 flagged every claimed scenario.
3. #7: pins, trees, engine pin and freeze paths are re-checked before every marker; a problem = HELD, exit 3, marker not taken. Operating condition: no edit of freeze paths during a run.
4. #15: a HELD at claim time (only HELD errors) = exit 3, log HELD; N-4: all slot errors are listed.
5. G-21: `run.sh` runs the verdict step over the existing reports on exit 5 and ends with 5.
6. (+) G-23: the readiness check of the claiming run is the check done right before the marker (no re-check after the marker); `recheckPins` time lies between that check and the run.
7. #3: INFRA report written on an error after a marker; phase B scenario by scenario; `phase-a.jsonl`.
8. #5: `claimed` from the moment the marker is taken; later failures give an INFRA report.
9. #11 / G-26: watchdog stage 1 also uses the last second of CPU, stage 2 kills the whole descendant tree then the group (classification still by `watchdog.fired`).
10. Names: gate version 4, `SLAM_REFACT_US025_LC_V4_DESIGNATION_SHA256`, output / dev / slot-lock paths, schema `...-v4`, mode `v4-two-phase`.
11. Pin set: `us026/make_bundles.sh` and `make_variant.py` out; `v4/make_bundles.sh`, `make_variant_v2.py`, `original_run_info.py` in.
12. Unchanged: the five judgement files, constants, run counts, tolerances, verdict exits. `make_lc_static.py` narrative keys (`exposure`, `classification`, `supersedes`) are still v3 text: the registration lane replaces them.

Also G-25 (empty `--full-null` = FAIL instead of PASS, stricter, registration-material tool only), #2 / #12 / G-20 / G-22 / #10 (analysis and dev tools, no gate verdict).
