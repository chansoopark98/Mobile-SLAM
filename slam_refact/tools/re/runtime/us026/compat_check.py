#!/usr/bin/env python3
"""US-026 compat evidence for run-reference.mjs --poke / --memprobe-detail (design §2.5, lead decision D2).

Compares a run made with the runner BEFORE the change and a run made AFTER the change, both with the new options
OFF and the same command line (same scenario, --max-frames, --memprobe --record-inputs). The original SDK is not
run-to-run deterministic (docs/reports/parity.md §3.1), so value equality of two runs is reported, not required; the
format checks (file set, header fields, record keys, sourceHashes key set) follow the --memprobe-paths precedent
(build/slam-refact-20261018/re-paths/compat-default-output.json).

  python3 -I compat_check.py --pre <run dir> --post <run dir> --out <new json>
"""
import argparse
import gzip
import hashlib
import json
import pathlib


def jsonl(p):
    op = gzip.open if p.suffix == '.gz' else open
    with op(p, 'rt') as f:
        return [json.loads(line) for line in f if line.strip()]


def sha(p):
    return hashlib.sha256(p.read_bytes()).hexdigest()


def keyset(rows):
    return sorted({tuple(r.keys()) for r in rows})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pre', type=pathlib.Path, required=True)
    ap.add_argument('--post', type=pathlib.Path, required=True)
    ap.add_argument('--out', type=pathlib.Path, required=True)
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    pre, post = a.pre, a.post
    rep = {'pre': str(pre), 'post': str(post), 'checks': {}, 'values': {}}
    c = rep['checks']
    c['fileSetEqual'] = sorted(p.name for p in pre.iterdir()) == sorted(p.name for p in post.iterdir())
    c['files'] = sorted(p.name for p in post.iterdir())
    mpre, mpost = json.loads((pre / 'meta.json').read_text()), json.loads((post / 'meta.json').read_text())
    c['metaTopKeysEqual'] = sorted(mpre) == sorted(mpost)
    c['metaSourceHashKeysEqual'] = sorted(mpre['sourceHashes']) == sorted(mpost['sourceHashes'])
    c['metaSourceHashesDiffering'] = sorted(k for k in mpre['sourceHashes'] if mpre['sourceHashes'][k] != mpost['sourceHashes'].get(k))
    c['metaNoPokeKey'] = 'poke' not in mpost
    c['runnerSha256'] = {'pre': mpre['sourceHashes']['run-reference.mjs'], 'post': mpost['sourceHashes']['run-reference.mjs']}
    c['engineAssetsEqual'] = mpre['engineAssets'] == mpost['engineAssets']
    c['memprobeMetaKeysEqual'] = sorted(mpre.get('memprobe', {})) == sorted(mpost.get('memprobe', {}))
    c['recordedInputsMetaKeysEqual'] = sorted(mpre.get('recordedInputs', {})) == sorted(mpost.get('recordedInputs', {}))
    for name in ('trace.jsonl', 'callbacks.jsonl', 'memprobe.jsonl.gz', 'recorded-inputs.jsonl.gz'):
        ra, rb = jsonl(pre / name), jsonl(post / name)
        ent = {'rowsPre': len(ra), 'rowsPost': len(rb)}
        if name.endswith('.gz'):  # line 1 = header
            ha, hb = ra[0], rb[0]
            ent['headerKeysEqual'] = sorted(ha) == sorted(hb)
            if 'fields' in ha:
                ent['fieldsEqual'] = ha['fields'] == hb['fields']
                ent['nFields'] = len(hb['fields'])
            if 'stageArgNames' in ha:
                ent['stageArgNamesEqual'] = ha['stageArgNames'] == hb['stageArgNames']
            ra, rb = ra[1:], rb[1:]
        ent['recordKeySetsEqual'] = keyset(ra) == keyset(rb)
        ent['recordKeySets'] = len(keyset(rb))
        ent['fileSha256Equal'] = sha(pre / name) == sha(post / name)
        c[name] = ent
    # Values (REPORT: the original is not run-to-run deterministic).
    ta = {r['i']: r for r in jsonl(pre / 'trace.jsonl')}
    tb = {r['i']: r for r in jsonl(post / 'trace.jsonl')}
    both = sorted(i for i in ta if i in tb and ta[i]['fresh'] and tb[i]['fresh'])
    same = [i for i in both if (ta[i]['p'], ta[i]['q_wxyz'], ta[i]['status'], ta[i]['worldPoints']) ==
            (tb[i]['p'], tb[i]['q_wxyz'], tb[i]['status'], tb[i]['worldPoints'])]
    diff = [i for i in both if i not in set(same)]
    rep['values']['trace'] = {'freshBoth': len(both), 'freshPre': sum(1 for r in ta.values() if r['fresh']),
                              'freshPost': sum(1 for r in tb.values() if r['fresh']),
                              'poseStatusWpEqualFrames': len(same), 'firstDifferingFrame': diff[0] if diff else None}
    pa = {r['seq']: r for r in jsonl(pre / 'memprobe.jsonl.gz')[1:]}
    pb = {r['seq']: r for r in jsonl(post / 'memprobe.jsonl.gz')[1:]}
    det = ['mbFrameCount', 'outMapReset', 'outKeyframeCreated', 'kfLive', 'pts1', 'pts2', 'pts3']
    common = sorted(set(pa) & set(pb))
    rep['values']['memprobe'] = {'recordsPre': len(pa), 'recordsPost': len(pb), 'commonSeq': len(common),
                                 'stateFieldsEqualSeq': sum(1 for s in common if all(pa[s].get(k) == pb[s].get(k) for k in det)),
                                 'fieldsCompared': det}
    ok = (c['fileSetEqual'] and c['metaTopKeysEqual'] and c['metaSourceHashKeysEqual'] and c['metaNoPokeKey']
          and c['metaSourceHashesDiffering'] == ['run-reference.mjs'] and c['engineAssetsEqual']
          and c['memprobeMetaKeysEqual'] and c['recordedInputsMetaKeysEqual']
          and all(c[n]['recordKeySetsEqual'] and c[n].get('headerKeysEqual', True) and c[n].get('fieldsEqual', True)
                  and c[n].get('stageArgNamesEqual', True)
                  for n in ('trace.jsonl', 'callbacks.jsonl', 'memprobe.jsonl.gz', 'recorded-inputs.jsonl.gz')))
    rep['formatCompatible'] = ok
    a.out.write_text(json.dumps(rep, indent=1) + '\n')
    print(json.dumps({'formatCompatible': ok, 'trace': rep['values']['trace'], 'memprobe': rep['values']['memprobe']}))


if __name__ == '__main__':
    main()
