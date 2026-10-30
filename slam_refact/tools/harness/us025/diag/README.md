# us025 diag: loop close positional probe (dev, not a gate)

`lc_positional_probe.py` reads an original (reference runtime) lc=1 run and the engine ON replay of the same bundle, offline, and prints per loop close attempt
the candidate position, slot, descriptor distance, ring state and the lower bound of the covisible exclusion set S2 on both sides, then the first
mismatch. It runs nothing, imports the pinned `../compare_us025.py` only to load the records, and cross-checks its own counts against that
comparator's `judge_attempts` (exit 3 on disagreement). It never changes a gate verdict and its output is not PASS evidence.

```bash
# every lc1 run of a scenario directory (runs/<scenario>/<run>/, replays/<run>-on/, compare/<run>-full.json|-on.json)
python3 -I -B slam_refact/tools/harness/us025/diag/lc_positional_probe.py --scenario-dir <G>/tum-room2-full --out-dir <out>
python3 -I -B slam_refact/tools/harness/us025/diag/lc_positional_probe.py --scenario-dir <E>/flip6-s2 --run b15-s-lc1-03 --out-dir <out>
# explicit directories
python3 -I -B slam_refact/tools/harness/us025/diag/lc_positional_probe.py --original <run dir> --engine <replay dir> --label X --out-dir <out>
# unit tests (synthetic attempts only; no -I, the path form needs the cwd on sys.path)
python3 -m unittest slam_refact/tools/harness/us025/diag/test_lc_positional_probe.py -v
```

Output per run: `<label>.attempts.tsv` (one row per attempt), `<label>.json` (rows, summary, first mismatch, cross-check, recorded trace comparison), and
`summary.tsv` (one line per run). Exit codes: 0 ok, 2 usage, 3 cross-check disagrees with the pinned comparator, 4 invalid input.

Row classes: `MATCH_FIRST` (original picked the first eligible position), `MATCH_SKIPPED` (original skipped eligible positions, engine identical),
`MATCH_NO_PICK`, `DECLARED_U2` (original pick is not a live keyframe), `MISMATCH_SKIPPED/FIRST/OTHER` (picks differ; `witness_pos` names a position that one
side's S2 must contain and the other side's S2 must not), `POS_EQUAL_ID_DIFF` (same position, different handle id: not an S2 difference),
`PRECONDITION_DIFF` (slot layout, newest index or live descriptors differ: no S2 claim is made), `TOO_FEW_SLOTS`, `AFTER_F` (after the original's first success,
undefined state, N2), `ATTEMPT_SET_DIFF`.

Limits: S2 itself is recorded by neither side. `required_s2_*` is a lower bound derived from the selection rule (INFERRED), the distance threshold is the config
default 80.0 (not read from the runs), and positions are positions in the compressed live list (spec 12 U2), not slot indices.
