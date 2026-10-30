import sys,glob,json,re,collections
sys.path.insert(0,'.')
from show import load
def l0(d,lo=123):
    for l in open(d+'/images.jsonl'):
        r=json.loads(l)
        if r['i']>=lo and any(e['kind']==2 for e in r['events']): return r['i']
    return None
if __name__=='__main__':
    pat=sys.argv[1]
    c=collections.Counter(); rows=[]
    for d in sorted(glob.glob(pat)):
        if not glob.glob(d+'/meta.json'): continue
        x=l0(d); c[x]+=1
        pn=None
        for r in load(d+'.jsonl'):
            if r['i']==123:
                p=[s for s in r['stages'] if s['fn']=='pnp']; pn=(int(p[0]['ok']),p[0]['inl']) if p else None
        rows.append((d.split('-k')[-1],x,pn))
    print(dict(c)); print(rows)
