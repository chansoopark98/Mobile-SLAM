#!/usr/bin/env python3
"""US-026 root-cause A-2 analysis (REPORT tier, read-only): for one a2_repro.sh run, first pose difference (gate
ulp definition, compare_us026.ulps; 3-ulp rule) between the original run trace and the engine BA-on replay trace,
and the first map-hash / pool difference (maphash_compare + maphash_fields logic) against the same run's records.
Usage: a2_analyze.py <rootcause-a2 dir> <label>"""
import gzip, json, sys, os, importlib.util
here = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('cmp', os.path.join(here, 'compare_us026.py'))
cmp = importlib.util.module_from_spec(spec); spec.loader.exec_module(cmp)
root, L = sys.argv[1], sys.argv[2]
run = f'{root}/runs/flip6-s2/a2-{L}'
ot = {r['i']: r for r in cmp.jsonl(f'{run}/trace.jsonl')}
et = {r['i']: r for r in cmp.jsonl(f'{root}/{L}/replay-on/trace.jsonl')}
def withi(p, gz):
    return {r['i']: r for r in cmp.jsonl(p, gz) if r.get('i') is not None}
mp = withi(f'{run}/memprobe.jsonl.gz', True); po = withi(f'{run}/poke.jsonl.gz', True)
mh = withi(f'{root}/{L}/replay-on/maphash.jsonl', False)
res = {'label': L}
fresh = [i for i in sorted(ot) if ot[i].get('fresh') and i in et and et[i].get('fresh')]
viol = []; maxu = 0.0
for i in fresh:
    o, e = ot[i], et[i]
    if o.get('p') is None or e.get('p') is None: continue
    u = max(cmp.ulps(a, b) for a, b in zip(o['p'] + o['q_wxyz'], e['p'] + e['q_wxyz']))
    maxu = max(maxu, u)
    if u > 3: viol.append(i)
res.update(framesCompared=len(fresh), poseViolationFrames=len(viol), poseFirstViolation=viol[0] if viol else None, poseMaxUlps=maxu)
intd = [i for i in fresh if any(ot[i].get(k) != et[i].get(k) for k in ('status', 'reason', 'worldPoints'))]
res['firstIntegerDifference(status/reason/worldPoints)'] = intd[0] if intd else None
cm = [i for i in sorted(mh) if mh[i].get('wm') and i in po]
def first(f):
    d = [i for i in cm if f(i)]
    return d[0] if d else None
res['mapHash'] = {'framesCompared': len(cm), 'engineNoMapFrames': sum(1 for i in mh if not mh[i].get('wm')),
    'firstKfPoseHashDiff': first(lambda i: po[i]['kfPoseHash'] != mh[i]['kfPoseHash']),
    'firstPtsHash1Diff': first(lambda i: po[i]['ptsHash1'] != mh[i]['ptsHash1']),
    'firstKfLiveDiff': first(lambda i: po[i]['kfLive'] != mh[i]['kfLive']),
    'firstPoolDiff': first(lambda i: any(mp[i][f'pts{k}'] != mh[i]['pools'][k-1] if k < 5 else mp[i][f'pts{k}'][2:] != mh[i]['pools'][k-1][2:] for k in range(1, 6)))}
res['kfPoseChangedSum'] = sum((po[i].get('kfPoseChanged') or 0) for i in po)
res['originalMapResets'] = [i for i in mp if mp[i].get('outMapReset')]
res['originalFirstKfPoseChangedFrame'] = next((i for i in sorted(po) if (po[i].get('kfPoseChanged') or 0) > 0), None)
print(json.dumps(res, indent=1))
