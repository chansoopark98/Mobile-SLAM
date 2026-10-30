#!/usr/bin/env python3
"""Locate first frontend diagnostic stage divergence; no acceptance threshold changes."""
import argparse,json,math
from pathlib import Path

def rows(path):
    value=json.loads(Path(path).read_text());return value.get('rows',value.get('frames'))
def differences(left,right,path='',out=None):
    if out is None:out=[]
    if len(out)>=40:return out
    if type(left)!=type(right) and not(isinstance(left,(int,float)) and isinstance(right,(int,float))):out.append({'path':path,'left_type':type(left).__name__,'right_type':type(right).__name__});return out
    if isinstance(left,dict):
        for key in sorted(set(left)|set(right)):
            if key not in left or key not in right:out.append({'path':path+'.'+key,'missing':'left' if key not in left else 'right'})
            else:differences(left[key],right[key],path+'.'+key,out)
    elif isinstance(left,list):
        if len(left)!=len(right):out.append({'path':path,'left_length':len(left),'right_length':len(right)})
        for i,(a,b) in enumerate(zip(left,right)):differences(a,b,f'{path}[{i}]',out)
    elif left!=right:
        value={'path':path,'left':left,'right':right}
        if isinstance(left,(int,float)) and isinstance(right,(int,float)):value['absolute_difference']=abs(left-right)
        out.append(value)
    return out

def main():
    p=argparse.ArgumentParser();p.add_argument('native',type=Path);p.add_argument('wasm',type=Path);p.add_argument('--output',type=Path,required=True);a=p.parse_args();native,wasm=rows(a.native),rows(a.wasm)
    if not isinstance(native,list) or not isinstance(wasm,list):raise ValueError('Expected captured frame rows')
    result={'scope':'explicit development frontend diagnostic; no held-out/performance acceptance','frames':[]}
    for nr,wr in zip(native,wasm):
        nd=nr.get('featureDiagnostics');wd=wr.get('featureDiagnostics')
        if isinstance(nd,str):nd=json.loads(nd)
        if isinstance(wd,str):wd=json.loads(wd)
        if nd is None or wd is None:raise ValueError('Diagnostic capture missing')
        result['frames'].append({'frame':nr['frame'],'differences':differences(nd,wd),'native_feature_count':nr.get('featureCount'),'wasm_feature_count':wr.get('featureCount')})
    a.output.write_text(json.dumps(result,indent=2)+'\n');print(a.output)
if __name__=='__main__':main()
