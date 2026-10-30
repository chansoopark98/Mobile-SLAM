#!/usr/bin/env python3
"""it-scale-gate.v4 manifest writer (registration helper, not part of the run).

  python3 -I make_manifest.py draft  --prereg DIR --engine BIN --snapshot-head SHA --out MANIFEST [--created-at ISO]
  python3 -I make_manifest.py review --manifest M --review-file R --designation D --reviewer TEXT --at ISO --out M2

draft : writes the manifest BEFORE the review fields (review = {}, first_real_run_at = null): gate file sha256 from DIR/it-scale-gate.v4.md,
        the pin set from `run.mjs --print-pins` (so the manifest and the runner cannot disagree), tool sha256, engine snapshot, FREEZE reference.
review: adds the review object (file sha256, designation sha256) to an existing draft; every other field is copied unchanged.  The canonical
        sha256 the designation pins (`node sr_held.mjs print-pin M`) blanks `review`, so this step does not change it.
Refuses to overwrite.  Never reads the coordinator seed.
"""
import argparse, datetime, hashlib, json, os, pathlib, subprocess, sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[3]
GATE, VERSION = 'it-scale-gate', 4
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()


def git(*a):
    return subprocess.run(['git', '-C', str(ROOT), *a], capture_output=True, text=True, check=True).stdout.strip()


def draft(a):
    prereg = pathlib.Path(a.prereg).resolve()
    md = prereg / f'{GATE}.v{VERSION}.md'
    pins = json.loads(subprocess.run(['node', str(HERE / 'run.mjs'), '--engine', a.engine, '--prefix-engine', str(pathlib.Path(a.engine).parent / 'sr_e5_replay_prefix'), '--print-pins'], capture_output=True, text=True, check=True).stdout)
    tool = {}
    for rel in ('slam_refact/tools/harness/e5/scene.py', 'slam_refact/tools/harness/e5/run_original.mjs', 'slam_refact/tools/harness/e5/e5_image_hook.js',
                'slam_refact/tools/harness/e5/e5_pipeline.py', 'slam_refact/tools/harness/e5/e5_extract.py', 'slam_refact/tools/harness/e5/e5_compare.py',
                'slam_refact/tools/harness/e5/prepare_bundle.py', 'slam_refact/tools/harness/e5/e5_kmutation.py', 'slam_refact/tools/harness/e5/e5_replay_main.cc',
                'slam_refact/tools/harness/e5/snapshot-cmake.patch', 'slam_refact/tools/harness/prepare_inputs.py', 'slam_refact/tools/harness/divergence/make_variant.py',
                'slam_refact/tools/gates/it-scale-v4/e5_gate_rule_v4.py', 'slam_refact/tools/gates/it-scale-v4/v4_controls.py', 'slam_refact/tools/gates/it-scale-v4/run.mjs',
                'slam_refact/tools/gates/it-scale-v4/run.sh', 'slam_refact/tools/gates/it-scale-v4/make_manifest.py', 'slam_refact/tools/gates/it-scale-v4/dryrun_analysis.py', 'slam_refact/tools/gates/it-scale-v4/dryrun_engine_rerun.py', 'slam_refact/tools/gates/it-scale-v4/dryrun_controls_study.py', 'slam_refact/tools/gates/it-scale-v4/v4_report_runs.py',
                'slam_refact/tools/gates/it-scale-v4/test_rule.py', 'slam_refact/tools/prereg/sr_held.mjs',
                'slam_refact/tools/re/runtime/page-runtime.js', 'slam_refact/tools/re/runtime/memprobe.js', 'slam_refact/tools/re/runtime/input-recorder.js',
                'slam_refact/tools/re/runtime/capnp-event-queue.mjs'):
        tool[rel] = sha(ROOT / rel)
    head = a.snapshot_head
    trees = {p: git('rev-parse', f'{head}:{p}') for p in ('slam_refact/engine', 'slam_refact/web-chunk', 'slam_refact/web-smoke', 'slam_refact/tools', 'wasm', 'src')}
    frz = sorted((ROOT / 'build/slam-refact-full-c2-20261020').glob('FREEZE.r*.json'), key=lambda f: int(f.name.split('.r')[1].split('.')[0]))[-1]
    fr = json.load(open(frz))
    newest = frz.name
    delta = [l for l in git('log', '--oneline', f'{fr["code_commit_of_refreeze"]}..{head}', '--', 'slam_refact/engine', 'slam_refact/tools', 'slam_refact/web-chunk',
                            'slam_refact/web-smoke', 'wasm', 'src').splitlines()]
    ev = lambda rel: {rel: sha(ROOT / 'build/slam-refact-full-20261019/it-scale-v4' / rel)}
    floors = {'S2 anchor frames': (1.0, 'frames'), 'S3 rho p95': (1e-3, 'relative'), 'S3 direction p95': (0.05, 'deg'), 'S3 relative pose p95': (0.05, 'deg'), 'S4 vs GT rho p95': (1e-2, 'relative')}
    m = {
        'gate_id': GATE, 'version': VERSION,
        'supersedes': {'note': 'v4 does NOT replace v3: it-scale-gate.v3 FAILED (both arms, first run 2026-10-23T01:13:22+09:00) and that result stays on record (protocol 7.4). v4 is a REQUIREMENT CHANGE (PRD amendment, docs/design/prd-amendment-us015-null-variation.md) made after seeing v3.',
                       'file': 'it-scale-gate.v3.md', 'sha256': sha(prereg / 'it-scale-gate.v3.md'),
                       'v3_gate_file': 'build/slam-refact-full-20261019/preregistration/it-scale-gate.v3.md', 'v3_md_sha256': sha(prereg / 'it-scale-gate.v3.md'),
                       'v3_manifest': 'build/slam-refact-full-20261019/preregistration/it-scale-gate.v3.manifest.json', 'v3_manifest_sha256': sha(prereg / 'it-scale-gate.v3.manifest.json'),
                       'v1_gate_file': 'build/slam-refact-full-20261019/preregistration/it-scale-gate.v1.md'},
        'classification': ['REQUIREMENT CHANGE after a registered result (not a tolerance fix, not a registration defect)',
                           'GATE items S2-S4 on the GATE ARM ONLY = world tracker ON (arm 1; the configuration the original always runs) (tolerance = 3 x max of the in-run original-vs-original null of the same sequence, per anchor / per metric); arm 0 (world tracker OFF, shipped default) computed with the same tolerances and REPORTED, never counted; S1 REPORT; positive controls on arm 1 required (else INSENSITIVE = no PASS claim); n_independent = 3 (fixed schedule, see gate md 10)'],
        'file': f'{GATE}.v{VERSION}.md', 'sha256': sha(md), 'created_at': a.created_at or datetime.datetime.now().astimezone().isoformat(timespec='seconds'),
        'registered_by': 'W1-itscale worker (Sonnet 5.5, writing pass; did not run the registered gate, does not approve it, did not read the coordinator seed)',
        'git_head': head, 'tree_sha': None,
        'code_state': 'tools of this gate (slam_refact/tools/gates/it-scale-v4/**) and the amendment/design docs are UNCOMMITTED new files at registration time; sha256 pinned in held.pins and tool_sha256; engine = git archive snapshot of git_head (slam_refact/engine + slam_refact/tools) + sr_e5_replay target, binary sha256 pinned. No write-tree -> grade medium for non-code files',
        'known_before_registration': 'KNOWN (gate md 0): the v3 first-run results (both arms FAIL), the post-hoc original-vs-original values of the v3 registered sequences, the L0 anchor difference (originals 123, engine 127-128), the v4 rule applied post hoc to the v3 data with the v3 engine (seq-a FAILS at L0, seq-b passes), the L0 diagnosis (ROI-draw numerics + engine default world tracker OFF vs the original always running the world-tracker branch), the d7629f6 engine replays of the v3 bundles (IN-SAMPLE: arm 1 identical to the originals in 6/6 pairs, arm 0 FAILS seq-a L0), the arm-role decision made after that knowledge, the dev mutation power under the v4 rule. NOT seen: the v4 registered sequences (coordinator seed not yet created, read only after the marker) and the behaviour of the git_head engine on them.',
        'resolution': {'metric': 'per metric: max(max over the 10 original-vs-original null pairs of the SAME registered sequence (5 original runs), representation floor); measured INSIDE the registered run (protocol 3 deviation, declared in gate md 4)',
                       'value': {k: v[0] for k, v in floors.items()}, 'unit': 'per metric (frames, relative, deg, deg, relative)', 'method': 'e5_gate_rule_v4.py tolerances(): R_m = max(max null, floor); the values here are the representation floors only'},
        'tolerance': {'value': {k: 3.0 * v[0] for k, v in floors.items()}, 'unit': 'per metric; MINIMUM values (= 3 x floor); the applied tolerance is T_m = 3 x R_m >= these',
                      'ratio': 3.0, 'derivation': 'T_m = 3 x max(max_{10 null pairs} v_m, floor_m) per sequence; anchors judged one by one; S4: 3 x max(max over the 5 originals of the GT error p95, 1e-2)'},
        'tiers': [{'id': k, 'resolution': {'value': v[0], 'unit': v[1]}, 'tolerance': {'value': 3.0 * v[0], 'unit': v[1], 'note': 'minimum; applied = 3 x in-run null if larger'}} for k, v in floors.items()],
        'subchecks': [],
        'null_run': {'evidence': 'build/slam-refact-full-20261019/it-scale-v4/dryrun/analysis/v3-on-v4.json (original-vs-original null pairs of the 2 v3 registered sequences, 3 runs each; dev pipe-101/202 via mutation-power.json) -- PRIOR information only',
                     'result': 'PASS', 'result_note': 'PASS = the null-run procedure is implemented and unit-tested and prior null values were measured; the tolerance itself is derived from the in-run null at the first real run (protocol 3 deviation, gate md 4, requirement change)',
                     'evidence_sha256': ev('dryrun/analysis/v3-on-v4.json'), 'detail_file': 'dryrun/analysis/v3-on-v4.json'},
        'mutation': {'description': 'stage 1c controls (focal-1.25, drop-every:2, drop-every:3 at the gate arm; effect sizes vs the tolerances of dev 202/101 and v3 seq-a/seq-b in dryrun/stage1c/controls-study/controls-study.json; frame-drop controls trip S2 and continuous checks on all four, scale control >= 9x rho); stage 1 dev mutants (engine 19fd011, arm 0) judged with the v4 rule using each dev sequence\'s own null (3 originals): drop-every:2/3/5, focal x1.10 DETECTED (seed 202); orient-beta:3, orient-drift:1 NOT detected (world-orientation blind class); in-run positive controls are part of the registered run (INSENSITIVE if a group is undetected); source files unchanged',
                     'evidence': 'build/slam-refact-full-20261019/it-scale-v4/dryrun/stage1c/controls-study/controls-study.json', 'detected': True, 'sub_resolution_detected': False,
                     'blind_class': 'world-orientation prior errors (gate md 9-1)', 'evidence_sha256': ev('dryrun/stage1c/controls-study/controls-study.json')},
        'independent_samples': {'n': 3, 'derivation': 'three registered sequences derived from one coordinator seed; they share the fixed look-away schedule of scene.py (textures, phases, noise differ), so motion structure is not independent; 5 original runs and 2 engine arms per sequence are repeats of the same input',
                                'by_item': {'S2': 3, 'S3': 3, 'S4': 3}},
        'held': {'kinds': ['main'], 'pins': pins, 'seed': True, 'dev_seeds': ['101', '202'], 'inputs': {}},
        'review': {}, 'first_real_run_at': None,
        'runner': {'file': 'slam_refact/tools/gates/it-scale-v4/run.sh', 'sha256': tool['slam_refact/tools/gates/it-scale-v4/run.sh'], 'driver': 'slam_refact/tools/gates/it-scale-v4/run.mjs',
                   'rule': 'slam_refact/tools/gates/it-scale-v4/e5_gate_rule_v4.py'},
        'tool_sha256': tool,
        'engine_snapshot': {'git_head': head, 'build_dir': 'build/slam-refact-full-20261019/it-scale-v4/engine (Release, -Werror, ninja, jobs 2)', 'cmake_patch': 'slam_refact/tools/harness/e5/snapshot-cmake.patch (applied to the snapshot copy only)',
                            'binary_sha256': pins['sr_e5_replay'], 'prefix_engine_binary_sha256': pins['sr_e5_replay_prefix'], 'prefix_engine_note': 'git 19fd011 snapshot build (before US015-CC-1); REPORT-only control, never counted', 'binary_copy': 'build/slam-refact-full-20261019/preregistration/it-scale-gate.v4-held-v1/sr_e5_replay'},
        'freeze': {'reference': str(frz.relative_to(ROOT)), 'latest_freeze_file_at_registration': newest, 'reference_code_commit': fr['code_commit_of_refreeze'], 'reference_head_at_record': fr['head_at_record'],
                   'subtree_sha_at_snapshot_head': trees, 'subtree_sha_in_reference_freeze': fr['subtree_sha'], 'commits_after_reference_freeze_touching_F_paths': delta,
                   'uncommitted_at_registration': 'stage 1b edits of slam_refact/tools/gates/it-scale-v4/** , docs/design/prd-amendment-us015-null-variation.md, docs/design/it-scale-gate-v4-design.md (the stage 1 versions are committed in 4f2413f)',
                   'note': 'a new FREEZE record should follow once the commits after the reference freeze are recorded; engine and tools trees at git_head differ from the reference freeze (see commits_after_reference_freeze_touching_F_paths and subtree_sha_*)'},
        'protocol_exceptions': {'E-IT4': {'rule': 'preregistration-protocol.md section 2 rule 8 (small-sample comparison with the original distribution is REPORT, not GATE)', 'status': 'REQUESTED, approval pending the coordinator; this manifest does not approve it', 'if_not_approved': 'the gate runs as REPORT and its PASS is not passes evidence (designation field protocol_exception_E_IT4)', 'text': 'gate md 10.1'}},
        'pre_declared_rules': 'gate md 1 and 5.2 (GATE arm = world tracker ON; arm 0 REPORT), 5 (per-anchor and per-metric tolerance from the in-run null; 15 GATE pairs + 15 REPORT pairs; S1 REPORT; controls on arm 1), 5.4 (FAIL > INSENSITIVE > PASS per sequence; FAIL > INVALID > INSENSITIVE > PASS per run), 6 (premises P1-P5), 7 (R1-R5: no re-run after FAIL/INSENSITIVE/INVALID, no early stop, no rule change after results)',
        'review_request': 'build/slam-refact-full-20261019/preregistration/it-scale-gate.v4.review-request.md',
        'amendment': 'slam_refact/docs/design/prd-amendment-us015-null-variation.md', 'design': 'slam_refact/docs/design/it-scale-gate-v4-design.md',
        'lane': 'IT/E5 (package H follow-up); protocol 13.1 precondition: post-review + result document of the v3 first run',
        'device_statement': 'safety device against mistakes, not an authentication (protocol 12.6)',
    }
    out = pathlib.Path(a.out)
    with open(out, 'x') as f:
        json.dump(m, f, indent=1)
        f.write('\n')
    print(f'wrote {out} sha256={sha(out)}')


def review(a):
    m = json.load(open(a.manifest))
    if m.get('review'):
        sys.exit('manifest already has a review object')
    m['review'] = {'reviewer': a.reviewer, 'at': a.at, 'file': os.path.basename(a.review_file), 'sha256': sha(a.review_file),
                   'designation_file': os.path.basename(a.designation), 'designation_file_sha256': sha(a.designation)}
    with open(a.out, 'x') as f:
        json.dump(m, f, indent=1)
        f.write('\n')
    print(f'wrote {a.out} sha256={sha(a.out)}')


ap = argparse.ArgumentParser()
sub = ap.add_subparsers(dest='cmd', required=True)
d = sub.add_parser('draft')
d.add_argument('--prereg', required=True); d.add_argument('--engine', required=True); d.add_argument('--snapshot-head', required=True); d.add_argument('--out', required=True)
d.add_argument('--created-at')
r = sub.add_parser('review')
for k in ('manifest', 'review-file', 'designation', 'reviewer', 'at', 'out'):
    r.add_argument('--' + k, required=True)
args = ap.parse_args()
{'draft': draft, 'review': review}[args.cmd](args)
