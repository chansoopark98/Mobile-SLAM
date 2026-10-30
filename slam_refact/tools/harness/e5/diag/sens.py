import sys,glob,re
sys.path.insert(0,'.')
from show import load
def run(pat,frame):
    res=[]
    for f in sorted(glob.glob(pat), key=lambda x:int(re.search(r'-k(\d+)\.jsonl',x).group(1))):
        k=int(re.search(r'-k(\d+)\.jsonl',f).group(1))
        for r in load(f):
            if r['i']==frame:
                pn=[s for s in r['stages'] if s['fn']=='pnp']
                res.append((k,int(pn[0]['ok']) if pn else None,pn[0]['inl'] if pn else None, r['postSt']))
    return res
if __name__=='__main__':
    res=run(sys.argv[1],int(sys.argv[2]))
    ok=[x for x in res if x[1]==1]; fail=[x for x in res if x[1]==0]
    print('n',len(res),'pnp ok',len(ok),'pnp fail',len(fail), 'fail ks',[x[0] for x in fail])
    print('inliers when ok: min %s max %s'%(min([x[2] for x in ok],default=None),max([x[2] for x in ok],default=None)))
