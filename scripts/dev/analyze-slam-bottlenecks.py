#!/usr/bin/env python3
"""Recompute timing and Lost associations from preserved numeric phone records."""
import argparse
import bisect
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--recording', action='append', type=Path)
parser.add_argument('--output', type=Path, default=ROOT / 'build/slam-comparison-20261014/input-timing/recomputed.json')
args = parser.parse_args()
paths = args.recording or [ROOT / 'build/dev-android/snapshot-2026-10-12T05-40-50-044Z.json', ROOT / 'build/dev-android/snapshot-2026-10-12T11-59-15-089Z.json']
if args.output.exists():
    raise SystemExit(f'Refusing to overwrite evidence: {args.output}')

def finite(x):
    return isinstance(x, (float, int)) and not isinstance(x, bool) and math.isfinite(x)

def stats(values):
    x = sorted(v for v in values if finite(v))
    if not x:
        return {'n': 0}
    def quantile(q):
        p = (len(x) - 1) * q
        lo, hi = math.floor(p), math.ceil(p)
        return x[lo] + (x[hi] - x[lo]) * (p - lo)
    return {'n': len(x), 'min': x[0], 'mean': sum(x) / len(x), 'p50': quantile(.5), 'p95': quantile(.95), 'p99': quantile(.99), 'max': x[-1]}

def difference(a, b):
    return a - b if finite(a) and finite(b) else None

def pearson(pairs):
    p = [(x, y) for x, y in pairs if finite(x) and finite(y)]
    if len(p) < 3:
        return {'n': len(p), 'r': None}
    ax, ay = (sum(v[i] for v in p) / len(p) for i in (0, 1))
    denom = math.sqrt(sum((x-ax)**2 for x,y in p)*sum((y-ay)**2 for x,y in p))
    return {'n':len(p),'r':sum((x-ax)*(y-ay) for x,y in p)/denom if denom else None}

output = {'schema':'mobile-slam-input-timing-recompute-v1','quantile':'linear interpolation of sorted observations at (n-1)*q',
          'scope':'Historic numeric phone data; no pixels, GT, live capture, product edits, or causal identification',
          'analyzerSha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
          'currentHead':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(), 'sourceHashes':{}, 'records':[]}
for relative in ['web/js/imu.js','web/js/app.js','web/js/vio-wrapper.js','web/js/vio-worker.js','src/vio_engine.cpp','src/backend/estimator.cpp','src/backend/optimizer.cpp','include/backend/factor/integration_base.h','web/vio_engine.js']:
    p=ROOT/relative
    output['sourceHashes'][relative]={'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
for path in paths:
    raw=path.read_bytes();data=json.loads(raw);record=data['recording'];start,stop=record['startedAtMs'],record['stoppedAtMs']
    rows=[r for r in record['results'] if start <= r['inputTimestamp']*1000 < stop]
    imu=record['imu'];its=[r[0] for r in imu]
    enriched=[]
    for row in rows:
        row=dict(row);pipeline=row.get('pipeline') or {};solver=pipeline.get('solver') or {};source=row.get('sourceTiming') or {}
        m={k:row.get(k) for k in ['engineProcessingMs','workerProcessingMs','workerQueueMs','imuWaitMs','imuWaitBudgetMs','roundtripMs','frameCopyMs','imuCount','featureCount']}
        m.update({k:pipeline.get(k) for k in ['imuMs','frontendMs','backendMs','totalMs']})
        m.update({k:solver.get(k) for k in ['ceresMs','marginalizationMs','currentVisualFactors','eligibleFeatures','immatureFeatures','imuFactors','rejectedImuFactors']})
        m['inputAgeAtReceptionMs']=row['arrivalMs']-row['inputTimestamp']*1000
        m['poseAgeAtReceptionMs']=difference(row['arrivalMs'],row.get('poseTimestamp')*1000 if finite(row.get('poseTimestamp')) and row.get('poseValid') else None)
        m['sourceToSubmitMs']=difference(row.get('sentAtMs'),row['inputTimestamp']*1000)
        m['pixelReadMs']=difference(source.get('pixelReadEndMs'),source.get('pixelReadStartMs'))
        m['snapshotToSubmitMs']=difference(row.get('sentAtMs'),source.get('pixelReadEndMs'))
        m['sourceToCameraCallbackMs']=difference(source.get('callbackArrivalMs'),row['inputTimestamp']*1000)
        m['receptionToUiMs']=difference(row.get('uiStateUpdatedAtMs'),row['arrivalMs'])
        m['inputAgeAtUiMs']=difference(row.get('uiStateUpdatedAtMs'),row['inputTimestamp']*1000)
        m['workerNonEngineMs']=difference(m.get('workerProcessingMs'),m.get('engineProcessingMs'))
        m['backendOtherMs']=m['backendMs']-m['ceresMs']-m['marginalizationMs'] if all(finite(m.get(k)) for k in ['backendMs','ceresMs','marginalizationMs']) else None
        m['roundtripResidualMs']=m['roundtripMs']-m['workerQueueMs']-m['imuWaitMs']-m['workerProcessingMs'] if all(finite(m.get(k)) for k in ['roundtripMs','workerQueueMs','imuWaitMs','workerProcessingMs']) else None
        previous=enriched[-1]['inputTimestamp'] if enriched else row['inputTimestamp']-.05
        sampled=imu[bisect.bisect_right(its,previous):bisect.bisect_right(its,row['inputTimestamp'])]
        if sampled:m['gyroIntervalMaxDegS']=max(math.degrees(math.hypot(*r[4:7])) for r in sampled)
        if enriched:m['inputIntervalMs']=(row['inputTimestamp']-previous)*1000
        row['metrics']=m;enriched.append(row)
    groups={'all':enriched,'fresh':[r for r in enriched if r.get('poseFresh') is True], 'lost':[r for r in enriched if r.get('statusCode')==3]}
    result={'path':str(path),'sha256':hashlib.sha256(raw).hexdigest(),'startedAtMs':start,'stoppedAtMs':stop,'durationS':(stop-start)/1000,
            'receivedRows':len(record['results']),'inWindowRows':len(rows),'recordedImuRows':len(imu),'recordingOverflow':record.get('dropped'),
            'calibration':record.get('calibration'),'config':record.get('config'), 'groups':{},'lostCorrelations':{},'lostReasons':dict(Counter(r.get('reason') for r in groups['lost'])),
            'lostCurrentVisualFactors':dict(Counter(str(r['metrics'].get('currentVisualFactors')) for r in groups['lost'])),
            'sourceImuSpacingMs':stats([(b-a)*1000 for a,b in zip(its,its[1:])]),
            'sourceImuHzByRecordingWindow':len(imu)/((stop-start)/1000),
            'sourceImuHzBySpan':(len(its)-1)/(its[-1]-its[0]) if len(its)>1 else None,
            'sourceImuDurationS':its[-1]-its[0] if len(its)>1 else None,
            'sourceCallbackArrivalSnapshotOnlyMs':stats([s['diagnostics']['imu']['lastSample']['arrivalMs']-s['diagnostics']['imu']['lastSample']['timestampS']*1000 for s in data.get('samples',[]) if s.get('diagnostics',{}).get('imu',{}).get('lastSample')]),
            'sourceSnapshots':[s.get('diagnostics',{}).get('imu') for s in data.get('samples',[])],
            'imuDropSnapshots':[r.get('imuDrops') for r in (enriched[:1]+enriched[-1:])],
            'gyroBands':{},'waitGroups':{},'lostRuns':[]}
    metric_names=sorted({k for r in enriched for k in r['metrics']})
    for name,group in groups.items():
        result['groups'][name]={'n':len(group),'metrics':{k:stats([r['metrics'].get(k) for r in group]) for k in metric_names}}
    for key in metric_names:
        result['lostCorrelations'][key]=pearson([(r['metrics'].get(key),int(r.get('statusCode')==3)) for r in enriched])
    for name,predicate in [('gyroBelow30DegS',lambda x:x<30),('gyroAtLeast30DegS',lambda x:x>=30)]:
        group=[r for r in enriched if finite(r['metrics'].get('gyroIntervalMaxDegS')) and predicate(r['metrics']['gyroIntervalMaxDegS'])]
        result['gyroBands'][name]={'n':len(group),'lost':sum(r.get('statusCode')==3 for r in group)}
    for name,predicate in [('noWait',lambda x:x==0),('positiveWait',lambda x:x>0)]:
        group=[r for r in enriched if finite(r['metrics'].get('imuWaitMs')) and predicate(r['metrics']['imuWaitMs'])]
        result['waitGroups'][name]={'n':len(group),'lost':sum(r.get('statusCode')==3 for r in group)}
    active=None
    for index,row in enumerate(enriched):
        if row.get('statusCode')==3:
            if active is None:active={'firstInputS':row['inputTimestamp'],'rows':0}
            active['lastInputS']=row['inputTimestamp'];active['rows']+=1
        elif active:
            active['nextFreshInputS']=row['inputTimestamp'] if row.get('poseFresh') else None
            result['lostRuns'].append(active);active=None
    if active:result['lostRuns'].append(active)
    fresh=groups['fresh'];result['freshInputGapMs']=stats([(b['inputTimestamp']-a['inputTimestamp'])*1000 for a,b in zip(fresh,fresh[1:])])
    result['accounting']={'measuredImuRows':sum(finite(r['metrics'].get('imuMs')) for r in enriched),'totalImuMs':sum(r['metrics']['imuMs'] for r in enriched if finite(r['metrics'].get('imuMs'))) if any(finite(r['metrics'].get('imuMs')) for r in enriched) else None,'totalEngineMs':sum(r['metrics']['engineProcessingMs'] for r in enriched if finite(r['metrics'].get('engineProcessingMs')))}
    output['records'].append(result)
frozen_path=ROOT/'build/research-20261012/baseline/manifest.json'
if frozen_path.exists():
    frozen_raw=frozen_path.read_bytes();frozen=json.loads(frozen_raw);entries={e['path']:e['sha256'] for e in frozen['entries']}
    output['frozenSourceComparison']={'manifest':str(frozen_path),'sha256':hashlib.sha256(frozen_raw).hexdigest(),'created':frozen['created_utc'],'comparisons':{k:{'current':v['sha256'],'frozen':entries.get(k),'matches':v['sha256']==entries.get(k)} for k,v in output['sourceHashes'].items()}}
args.output.parent.mkdir(parents=True,exist_ok=True)
args.output.write_text(json.dumps(output,indent=2)+'\n')
print(json.dumps({'output':str(args.output),'records':len(output['records'])}))
