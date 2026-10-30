#!/usr/bin/env python3
"""US-025 LC-0 compat evidence: run-reference.mjs / memprobe.js / us026/config-poke.js after the `lc` additions must
write the same file set, headers and record layouts as before when --memprobe-lc and the `lc` poke key are NOT used.

Same method as us026/compat_check.py (format equality; the original SDK is not run-to-run deterministic, so value
equality is reported, not required), generalised: `--allowed-diff` lists the sourceHashes keys that may differ
(default run-reference.mjs,memprobe.js plus us026/config-poke.js when poke.jsonl.gz is present), and poke.jsonl.gz is
compared too (header keys and `fields`/`prePostLayout`, record key sets, write list, pre/post lengths).

  python3 -I compat_check_lc.py --pre <run dir> --post <run dir> --out <new json>
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
    ap.add_argument('--allowed-diff', default='run-reference.mjs,memprobe.js,us026/config-poke.js')
    a = ap.parse_args()
    if a.out.exists():
        raise SystemExit('refusing to overwrite ' + str(a.out))
    pre, post = a.pre, a.post
    allowed = set(a.allowed_diff.split(','))
    rep = {'pre': str(pre), 'post': str(post), 'allowedSourceHashDiff': sorted(allowed), 'checks': {}, 'values': {}}
    c = rep['checks']
    c['fileSetEqual'] = sorted(p.name for p in pre.iterdir()) == sorted(p.name for p in post.iterdir())
    c['files'] = sorted(p.name for p in post.iterdir())
    mpre, mpost = json.loads((pre / 'meta.json').read_text()), json.loads((post / 'meta.json').read_text())
    c['metaTopKeysEqual'] = sorted(mpre) == sorted(mpost)
    c['metaSourceHashKeysEqual'] = sorted(mpre['sourceHashes']) == sorted(mpost['sourceHashes'])
    differing = sorted(k for k in mpre['sourceHashes'] if mpre['sourceHashes'][k] != mpost['sourceHashes'].get(k))
    c['metaSourceHashesDiffering'] = differing
    c['metaSourceHashesDifferingAllowed'] = set(differing) <= allowed
    c['engineAssetsEqual'] = mpre['engineAssets'] == mpost['engineAssets']
    c['memprobeMetaKeysEqual'] = sorted(mpre.get('memprobe', {})) == sorted(mpost.get('memprobe', {}))
    c['pokeMetaKeysEqual'] = sorted(mpre.get('poke', {})) == sorted(mpost.get('poke', {}))
    names = ['trace.jsonl', 'callbacks.jsonl', 'memprobe.jsonl.gz']
    for opt in ('recorded-inputs.jsonl.gz', 'poke.jsonl.gz'):
        if (post / opt).exists():
            names.append(opt)
    for name in names:
        ra, rb = jsonl(pre / name), jsonl(post / name)
        ent = {'rowsPre': len(ra), 'rowsPost': len(rb)}
        if name.endswith('.gz'):  # line 1 = header
            ha, hb = ra[0], rb[0]
            ent['headerKeysEqual'] = sorted(ha) == sorted(hb)
            for k in ('fields', 'prePostLayout', 'stageArgNames', 'keys', 'writes'):
                if k in ha or k in hb:
                    ent[k + 'Equal'] = ha.get(k) == hb.get(k)
            if 'fields' in hb:
                ent['nFields'] = len(hb['fields'])
            ra, rb = ra[1:], rb[1:]
        ent['recordKeySetsEqual'] = keyset(ra) == keyset(rb)
        ent['recordKeySets'] = len(keyset(rb))
        if name == 'poke.jsonl.gz':
            ent['prePostLengthsEqual'] = sorted({(len(r['pre']), len(r['post'])) for r in ra}) == sorted({(len(r['pre']), len(r['post'])) for r in rb})
        ent['fileSha256Equal'] = sha(pre / name) == sha(post / name)
        c[name] = ent
    ta = {r['i']: r for r in jsonl(pre / 'trace.jsonl')}
    tb = {r['i']: r for r in jsonl(post / 'trace.jsonl')}
    both = sorted(i for i in ta if i in tb and ta[i]['fresh'] and tb[i]['fresh'])
    same = [i for i in both if (ta[i]['p'], ta[i]['q_wxyz'], ta[i]['status'], ta[i]['worldPoints']) ==
            (tb[i]['p'], tb[i]['q_wxyz'], tb[i]['status'], tb[i]['worldPoints'])]
    diff = [i for i in both if i not in set(same)]
    rep['values']['trace'] = {'freshBoth': len(both), 'poseStatusWpEqualFrames': len(same), 'firstDifferingFrame': diff[0] if diff else None}
    pa = {r['seq']: r for r in jsonl(pre / 'memprobe.jsonl.gz')[1:]}
    pb = {r['seq']: r for r in jsonl(post / 'memprobe.jsonl.gz')[1:]}
    det = ['mbFrameCount', 'outMapReset', 'outKeyframeCreated', 'kfLive', 'pts1', 'pts2', 'pts3']
    common = sorted(set(pa) & set(pb))
    rep['values']['memprobe'] = {'commonSeq': len(common),
                                 'stateFieldsEqualSeq': sum(1 for s in common if all(pa[s].get(k) == pb[s].get(k) for k in det))}
    fmt = [n for n in names]
    ok = (c['fileSetEqual'] and c['metaTopKeysEqual'] and c['metaSourceHashKeysEqual'] and c['metaSourceHashesDifferingAllowed']
          and c['engineAssetsEqual'] and c['memprobeMetaKeysEqual'] and c['pokeMetaKeysEqual']
          and all(c[n]['recordKeySetsEqual'] and all(v for k, v in c[n].items() if k.endswith('Equal') and k != 'fileSha256Equal') for n in fmt))
    rep['formatCompatible'] = bool(ok)
    a.out.write_text(json.dumps(rep, indent=1) + '\n')
    print(json.dumps({'formatCompatible': ok, 'trace': rep['values']['trace'], 'memprobe': rep['values']['memprobe']}))


if __name__ == '__main__':
    main()
