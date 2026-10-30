#!/usr/bin/env python3
"""it-scale-gate.v5 manifest writer: v5 = the v4 manifest with only the registration identity, pins/tool shas, dev seeds and the path-fix notes changed.

  python3 -I make_manifest.py draft  --prereg DIR --v4-manifest <v4 manifest before review fields> --engine BIN --out MANIFEST [--created-at ISO]
  python3 -I make_manifest.py review --manifest M --review-file R --designation D --reviewer TEXT --at ISO --out M2

draft : copies the v4 manifest, then sets gate_id/version/file/sha256 (DIR/it-scale-gate.v5.md), created_at, supersedes (adds v4 = INVALID, nothing measured),
        held.pins (run.mjs --print-pins of THIS directory), held.dev_seeds (+ the smoke dev seed), tool_sha256 (adds the v5 files), runner, freeze (latest FREEZE),
        protocol_exceptions (E-IT4 scope v4 and v5), v5_changes.  Tolerance, resolution, tiers, null_run, mutation, independent_samples are copied unchanged.
review: adds the review object; everything else unchanged.   Refuses to overwrite.  Never reads the coordinator seed.
"""
import argparse, copy, datetime, hashlib, json, os, pathlib, subprocess, sys

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sha = lambda p: hashlib.sha256(open(p, 'rb').read()).hexdigest()
SMOKE_SEED = '404'


def draft(a):
    prereg = pathlib.Path(a.prereg).resolve()
    v4 = json.load(open(a.v4_manifest))
    m = copy.deepcopy(v4)
    md = prereg / 'it-scale-gate.v5.md'
    pins = json.loads(subprocess.run(['node', str(HERE / 'run.mjs'), '--engine', a.engine, '--prefix-engine', str(pathlib.Path(a.engine).parent / 'sr_e5_replay_prefix'), '--print-pins'],
                                     capture_output=True, text=True, check=True).stdout)
    m.update({'gate_id': 'it-scale-gate', 'version': 5, 'file': 'it-scale-gate.v5.md', 'sha256': sha(md),
              'created_at': a.created_at or datetime.datetime.now().astimezone().isoformat(timespec='seconds'), 'review': {}, 'first_real_run_at': None})
    m['supersedes'] = {'note': 'v5 = v4 + the OUTPUT-PATH fix only (path-only re-registration). it-scale-gate.v4 was INVALID after its marker (run_original.mjs refused the --out path; nothing measured), '
                       'so v5 gains no knowledge from it. it-scale-gate.v3 FAILED (both arms) and that stays on record; v1 is the analytic gate. None of them is replaced.',
                       'file': 'it-scale-gate.v4.md', 'sha256': sha(prereg / 'it-scale-gate.v4.md'), 'v4_manifest': 'it-scale-gate.v4.manifest.json', 'v4_manifest_sha256': sha(prereg / 'it-scale-gate.v4.manifest.json'),
                       'v4_firstrun_results': 'slam_refact/docs/design/it-scale-gate-v4-firstrun-results.md', **{k: v for k, v in v4['supersedes'].items() if k.startswith('v3_') or k.startswith('v1_')}}
    m['classification'] = list(v4['classification']) + ['v5: path-only re-registration of v4 (rules, controls, tolerances, pins unchanged except the runner files); new coordinator seed; adds a marker-free smoke requirement before designation']
    m['held']['pins'] = pins
    m['held']['dev_seeds'] = sorted(set(m['held']['dev_seeds']) | {SMOKE_SEED}, key=int)
    tool = dict(v4['tool_sha256'])
    for rel in ('slam_refact/tools/gates/it-scale-v4/run.mjs', 'slam_refact/tools/gates/it-scale-v4/run.sh', 'slam_refact/tools/gates/it-scale-v4/make_manifest.py'):
        tool.pop(rel, None)
    for rel in ('slam_refact/tools/gates/it-scale-v5/run.mjs', 'slam_refact/tools/gates/it-scale-v5/run.sh', 'slam_refact/tools/gates/it-scale-v5/smoke.sh', 'slam_refact/tools/gates/it-scale-v5/make_manifest.py'):
        tool[rel] = sha(ROOT / rel)
    for rel, v in list(tool.items()):          # the v4 files this gate still uses must be unchanged since the v4 manifest
        if 'it-scale-v5' not in rel and sha(ROOT / rel) != v:
            sys.exit(f'{rel} changed since the v4 manifest')
    m['tool_sha256'] = tool
    m['runner'] = {'file': 'slam_refact/tools/gates/it-scale-v5/run.sh', 'sha256': tool['slam_refact/tools/gates/it-scale-v5/run.sh'], 'driver': 'slam_refact/tools/gates/it-scale-v5/run.mjs',
                   'smoke': 'slam_refact/tools/gates/it-scale-v5/smoke.sh', 'rule': 'slam_refact/tools/gates/it-scale-v4/e5_gate_rule_v4.py (unchanged v4 file)'}
    frz = sorted((ROOT / 'build/slam-refact-full-c2-20261020').glob('FREEZE.r*.json'), key=lambda f: int(f.name.split('.r')[1].split('.')[0]))[-1]
    fr = json.load(open(frz))
    head = subprocess.run(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
    trees = {p: subprocess.run(['git', '-C', str(ROOT), 'rev-parse', f'HEAD:{p}'], capture_output=True, text=True, check=True).stdout.strip()
             for p in ('slam_refact/engine', 'slam_refact/web-chunk', 'slam_refact/web-smoke', 'slam_refact/tools', 'wasm', 'src')}
    m['freeze'] = {'reference': str(frz.relative_to(ROOT)), 'reference_code_commit': fr['code_commit_of_refreeze'], 'reference_head_at_record': fr['head_at_record'], 'head_at_registration': head,
                   'subtree_sha_at_head': trees, 'subtree_sha_in_reference_freeze': fr['subtree_sha'], 'engine_tree_same_as_v4_snapshot_head': trees['slam_refact/engine'] == v4['freeze']['subtree_sha_at_snapshot_head']['slam_refact/engine'],
                   'note': 'the engine binary pins (sr_e5_replay 40e5f81e… from the git d7629f6 snapshot, sr_e5_replay_prefix 45b95a37… from 19fd011) are byte-identical to v4; new files slam_refact/tools/gates/it-scale-v5/** are committed after registration and a FREEZE record follows (coordinator)'}
    m['protocol_exceptions'] = {'E-IT4': {'rule': 'preregistration-protocol.md section 2 rule 8', 'status': 'APPROVED by the orchestrator 2026-10-23 for it-scale-gate v4 and extended to its path-only re-registration v5 (c2-protocol-exceptions.md E-IT4 heading); the record is pinned in the designation file field protocol_exception_E_IT4',
                                'text': 'gate md 10.1'}}
    m['code_state'] = v4['code_state'] + ' | v5: slam_refact/tools/gates/it-scale-v5/** (run.mjs, run.sh, smoke.sh, make_manifest.py) are new files, uncommitted at registration; their sha256 are in held.pins/tool_sha256'
    m['known_before_registration'] = v4['known_before_registration'] + ' | v5: the v4 first run (2026-10-23T21:23:13+09:00) was INVALID after its marker because run_original.mjs refused the output path; it ran no original SDK run and no engine run, so nothing about the engine was observed and v5 gains no knowledge from it. The v5 coordinator seed is new and unread by the authors.'
    m['review_request'] = 'build/slam-refact-full-20261019/preregistration/it-scale-gate.v5.review-request.md'
    m['v5_changes'] = {'diff_only_review': 'compare it-scale-v4 and it-scale-v5 run.mjs/run.sh (diff), it-scale-gate.v4.md and it-scale-gate.v5.md (diff), this manifest and the v4 manifest (diff)',
                       'changes': ['--out of a real run and of the smoke must be a new directory under build/slam-refact-pkgh-20261022 (run.sh and run.mjs refuse anything else before any marker, exit 2); the first-run command in the md uses build/slam-refact-pkgh-20261022/it-scale-v5-run1',
                                   'run.mjs --smoke / smoke.sh: marker-free one-sequence run of the same code path on a declared dev seed (404), required before designation (md section 7 and 11)',
                                   'gate id/version 5: new md, manifest, held dir (it-scale-gate.v5-held-v1: byte-identical binaries), env var SR_ITS5_DESIGNATION, new coordinator seed (the sequence-seed derivation string changes to it-scale-gate.v5)',
                                   'dev seed 404 added to held.dev_seeds'],
                       'unchanged': 'rule, constants, floors, factor, controls, REPORT scripts, premises, stop rules, pins of harness/SDK/Chrome/engine files, tolerances, E-IT4 conditions'}
    with open(a.out, 'x') as f:
        json.dump(m, f, indent=1); f.write('\n')
    print(f'wrote {a.out} sha256={sha(a.out)}')


def review(a):
    m = json.load(open(a.manifest))
    if m.get('review'):
        sys.exit('manifest already has a review object')
    m['review'] = {'reviewer': a.reviewer, 'at': a.at, 'file': os.path.basename(a.review_file), 'sha256': sha(a.review_file),
                   'designation_file': os.path.basename(a.designation), 'designation_file_sha256': sha(a.designation)}
    with open(a.out, 'x') as f:
        json.dump(m, f, indent=1); f.write('\n')
    print(f'wrote {a.out} sha256={sha(a.out)}')


ap = argparse.ArgumentParser()
sub = ap.add_subparsers(dest='cmd', required=True)
d = sub.add_parser('draft')
d.add_argument('--prereg', required=True); d.add_argument('--v4-manifest', required=True); d.add_argument('--engine', required=True); d.add_argument('--out', required=True); d.add_argument('--created-at')
r = sub.add_parser('review')
for k in ('manifest', 'review-file', 'designation', 'reviewer', 'at', 'out'):
    r.add_argument('--' + k, required=True)
args = ap.parse_args()
{'draft': draft, 'review': review}[args.cmd](args)
