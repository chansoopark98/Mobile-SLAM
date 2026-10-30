import json,gzip,bisect
B='build/slam-refact-20261015/'
def analyze(scen,cap,S,ts):
    m=json.load(open(B+f'spec/frontend/capture/{scen}/{cap}/meta.json'))
    epoch=m['probe']['epochWallMs']; e_us=round(epoch*1000)
    pub={f[0]:f for f in m['probe']['frames']}
    orient={tuple(s['orientation']):k for k,s in enumerate(S)}
    # setEventQueue sizes preceding each stage
    co=m['callOrder']; sizes={}
    for j,c in enumerate(co):
        if c[1]=='c8EmAsm_stageFrame' and j>0 and co[j-1][1]=='c8EmAsm_setEventQueue': sizes[c[0]]=co[j-1][3][0]
    out={}
    prevk=None
    for sc in m['stageCalls']:
        a=sc['args']; k=orient.get(tuple(a[4:7]))
        if k is None:
            # tolerance match
            for kk,s in enumerate(S):
                if all(abs(x-y)<1e-9 for x,y in zip(s['orientation'],a[4:7])): k=kk;break
        i=sc['inputFrame']
        ft=None if i is None else (pub[i][1]-e_us)/1e6
        ours=None if ft is None else bisect.bisect_right(ts,ft+1e-9)-1
        n_q=(sizes.get(sc['call'],0)-64)/168
        out[i]=dict(k=k,ours=ours,nq=n_q,dk=None if prevk is None or k is None else k-prevk,
                    timeNs_minus_t_ms=None if ft is None else round(a[13]/1e6-epoch-ft*1000,2),
                    frameTime_minus_t_ms=None if ft is None else round(a[12]*1000-epoch-ft*1000,2))
        prevk=k
    return out
for scen,caps in [('flip6-s2',['c1','c2-72','c2-73']),('tum-room1-full',['c1','c2'])]:
    S=[json.loads(l) for l in gzip.open(B+f'reference/runs/{scen}/engine-input-sensors.jsonl.gz','rt')]
    ts=[s['t'] for s in S]
    R={c:analyze(scen,c,S,ts) for c in caps}
    for c in caps:
        r=R[c]; fr=[i for i in r if i is not None]
        nq_ok=sum(1 for i in fr if r[i]['dk'] is not None and abs(r[i]['nq']-r[i]['dk'])<1e-9)
        diff=sum(1 for i in fr if r[i]['k']!=r[i]['ours'])
        lag=[r[i]['timeNs_minus_t_ms'] for i in fr]
        print(scen,c,'frames',len(fr),'orig!=ours',diff,'queueSize==dk',nq_ok,'timeNanos-t ms min/max',min(lag),max(lag))
    # cross-run
    for a_ in caps:
        for b_ in caps:
            if a_<b_:
                common=[i for i in R[a_] if i is not None and i in R[b_]]
                d=sum(1 for i in common if R[a_][i]['k']!=R[b_][i]['k'])
                print('  cross',a_,b_,'common',len(common),'different orientation sample',d)
    c=caps[-1]; print('  sample', [(i,R[c][i]['k'],R[c][i]['ours'],R[c][i]['nq'],R[c][i]['dk'],R[c][i]['timeNs_minus_t_ms']) for i in list(R[c])[:8]])
print('--- restricted windows')
for scen,caps,W in [('flip6-s2',['c1','c2-72','c2-73'],72),('tum-room1-full',['c1','c2'],23)]:
    S=[json.loads(l) for l in gzip.open(B+f'reference/runs/{scen}/engine-input-sensors.jsonl.gz','rt')]
    ts=[s['t'] for s in S]
    for c in caps:
        r=analyze(scen,c,S,ts); h={}
        for i in r:
            if i is None or i>=W: continue
            d=r[i]['k']-r[i]['ours']; h[d]=h.get(d,0)+1
        print(scen,c,'frames<%d'%W,dict(sorted(h.items())))
