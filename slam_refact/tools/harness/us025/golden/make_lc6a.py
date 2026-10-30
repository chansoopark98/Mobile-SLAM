#!/usr/bin/env python3
"""US-025 LC-G item 5 (LC-6a input): package the exact-input replay inputs of original lc=1 runs that contain a loop-close success.

For every run it copies (small files only)
  <out>/<run>/bundle/       exact-input bundle, loop close OFF (what `slam_refact_replay --bundle` reads; made by us026/make_bundles.sh from the run's
                            recorded inputs: frames, sensors, event queues, ids seed)
  <out>/<run>/bundle-on/    the same bundle + `params MapBuilder.loopCloseEnabled=b:1` (engine ON arm)
  <out>/<run>/correspondence.json   frame correspondence (make_correspondence.py): memprobe seq <-> bundle frame, attempts, aborts, restaged frames
  <out>/<run>/replay-off-compare.json   compare_us026.py (draft numbers) of the original lc=1 run vs the engine replay of `bundle/` (loop close OFF)
  <out>/<run>/expected.json  what the original produced at the FIRST loop-close success of the run (the comparison target for the call that is staged for
                            `stageFrame`): the correction matrix C (0x82 B), the newest keyframe pose after the call, ring state, candidate, sizes; for hook-mode runs also
                            P_old, the PnP input/output pose, match/correspondence counts and the exception that ended the call.
Comparable range (VERIFIED by replay-off-compare.json): engine OFF == original lc=1 run bit for bit on frames 0..stageFrame-1; the call staged for `stageFrame`
is where the original's loop close ran; after it the original's state is undefined (it left the call through an exception, success_table.py).
usage: make_lc6a.py --golden <dir> --run <name>=<scenario>:<label> [...]"""
import argparse
import hashlib
import json
import pathlib
import shutil
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--golden', type=pathlib.Path, required=True)
    ap.add_argument('--run', action='append', required=True, help='<name>=<scenario>:<label>')
    a = ap.parse_args()
    g = a.golden
    out = g / 'LC-6a'
    out.mkdir(exist_ok=True)
    index = []
    for spec in a.run:
        name, sl = spec.split('=', 1)
        scenario, label = sl.split(':')
        run = g / 'runs' / scenario / label
        dst = out / name
        if dst.exists():
            continue
        dst.mkdir()
        shutil.copytree(g / 'bundles' / scenario / label, dst / 'bundle')
        shutil.copytree(g / 'bundles' / scenario / (label + '-on'), dst / 'bundle-on')
        shutil.copyfile(g / 'replays' / (label + '-off-compare.json'), dst / 'replay-off-compare.json')
        subprocess.run([sys.executable, '-B', '-I', str(HERE / 'make_correspondence.py'), '--run', str(run), '--bundle', str(dst / 'bundle'), '--out', str(dst / 'correspondence.json')],
                       check=True, stdout=subprocess.DEVNULL)
        # expected.json for the first success
        lg = run / 'lcgold'
        blobs = (lg / 'blobs.bin').read_bytes()
        rd = lambda r: blobs[r['off']:r['off'] + r['len']]
        corr = json.loads((dst / 'correspondence.json').read_text())
        events = [json.loads(l) for l in (lg / 'events.jsonl').read_text().splitlines() if l]
        ev = next(e for e in events if e.get('outLcOk'))
        hooks = [json.loads(l) for l in (lg / 'hooks.jsonl').read_text().splitlines() if l] if (lg / 'hooks.jsonl').exists() else []
        hk = {}
        for h in hooks:
            if h['seq'] == ev['seq']:
                hk.setdefault(h['id'], []).append(h)
        header = json.loads((lg / 'header.json').read_text())
        exp = {'run': str(run), 'seq': ev['seq'], 'stageFrame': next(x['stageFrame'] for x in corr['attempts'] if x['seq'] == ev['seq']), 'newest': ev['newest'],
               'candidateId': ev['ringPost'][2], 'selfCandidate': ev['ringPost'][2] == ev['newest'][1], 'nSlots': ev['nSlots'], 'nLive': ev['nLive'],
               'ringPre': ev['ringPre'], 'ringPost': ev['ringPost'], 'outLcOk': 1,
               'correctionC_hex_0x82': rd(ev['outLcMat'])[:130].hex(), 'newestPoseAfter_hex_0x82': rd(ev['newestExtrinsic'])[:130].hex(),
               'descriptorVector_sha256': ev['desc']['sha256'], 'wpiSizePostCall': ev['post']['newest']['wpiSize'], 'memprobePools': {k: ev['memprobe'][k] for k in ('pts1', 'pts2', 'pts3', 'pts4', 'pts5')},
               'hookMode': header['hookMode'], 'throwsInRun': header.get('throws')}
        if hk:
            exp['poseOld_hex_0x82'] = rd(hk['660-in'][0]['A'])[:130].hex()
            exp['pnpInputPose_hex_0x82'] = rd(hk['1146-in'][0]['pose'])[:130].hex()
            exp['pnpOutputPose_hex_0x82'] = rd(hk['1146-out'][0]['pose'])[:130].hex()
            exp['matches'] = hk['1128-out'][0]['nMatches']
            exp['correspondences'] = hk['1146-in'][0]['nObs']
            exp['wpiSizeBefore655'] = hk['655-in'][0]['newestDetail']['wpiSize']
            exp['unlinkIterationsSeen'] = len(hk.get('unlink-iter', []))
        (dst / 'expected.json').write_text(json.dumps(exp, indent=1) + '\n')
        index.append({'name': name, 'seq': exp['seq'], 'stageFrame': exp['stageFrame'], 'hookMode': exp['hookMode']})
    index = []
    for d in sorted(out.iterdir()):   # index over everything present, not only the runs added by this call
        if (d / 'expected.json').exists():
            e = json.loads((d / 'expected.json').read_text())
            index.append({'name': d.name, 'seq': e['seq'], 'stageFrame': e['stageFrame'], 'hookMode': e['hookMode']})
    (out / 'index.json').write_text(json.dumps(index, indent=1) + '\n')
    print(json.dumps(index))


if __name__ == '__main__':
    main()
