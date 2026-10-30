import json,sys,re
def load(p):
    out=[]
    for l in open(p):
        l=re.sub(r'-?nan','null',l); l=re.sub(r'-?inf','null',l)
        out.append(json.loads(l))
    return out
if __name__=='__main__':
    p,lo,hi=sys.argv[1],int(sys.argv[2]),int(sys.argv[3])
    for r in load(p):
        if lo<=r['i']<=hi:
            print(r['i'],'pre',r['preSt'],r['preLast'],r['preLocal'],'feats',r['feats'],'rois',[(x['src'],x['n']) for x in r['rois']],'locM',r['locMatches'],'locIn',r['locIn'],'gloM',r['gloMatches'],'gloIn',r['gloIn'],'post',r['postSt'],r['postLast'],r['postFirst'],'ang',r['angle'] and round(r['angle'],4))
