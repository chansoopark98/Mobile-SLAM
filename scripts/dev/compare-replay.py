#!/usr/bin/env python3
"""Compare native/Worker actual fresh poses; freeze pilot tolerances before validation."""
import argparse,json,math
from pathlib import Path
import numpy as np

def rows(path):
    doc=json.loads(Path(path).read_text());value=doc.get('rows',doc.get('frames'))
    if value is None and 'replay' in doc:value=doc['replay'].get('rows')
    if not isinstance(value,list) or not value:raise ValueError('No replay frame rows')
    indices=[r['frame'] for r in value]
    if indices!=list(range(len(value))):raise ValueError('Incomplete/duplicate replay frame ordering')
    return value

def compare(native,browser,expected):
    a,b=rows(native),rows(browser)
    if len(a)!=expected or len(b)!=expected:raise ValueError('Native/browser input frame counts differ from requested exact replay')
    positions=[];rotations=[];count_a=count_b=0;first_a=first_b=-1;state_differences=[]
    epoch_a=a[0]["engineEpoch"];epoch_b=b[0]["engineEpoch"]
    for x,y in zip(a,b):
        if abs(x['inputTimestamp']-y['inputTimestamp'])>1e-6:raise ValueError('Input timestamps differ')
        for field in ('statusCode','reason','solverIterations','solverTermination'):
            if field not in x or field not in y or x[field]!=y[field]:state_differences.append({'frame':x['frame'],'field':field,'native':x.get(field),'browser':y.get(field)})
        if x['engineEpoch']-epoch_a!=y['engineEpoch']-epoch_b:state_differences.append({'frame':x['frame'],'field':'relative_epoch'})
        if 'imuCount' in x and 'imuCount' in y and x['imuCount']!=y['imuCount']:raise ValueError('IMU packet counts differ')
        valid_a=bool(x.get('poseValid',False)) and x.get('pose') is not None
        valid_b=bool(y.get('poseValid',False)) and bool(y.get('poseFresh',True)) and y.get('pose') is not None
        if valid_a!=valid_b:state_differences.append({'frame':x['frame'],'field':'fresh_valid_pose'})
        if valid_a:count_a+=1;first_a=x['frame'] if first_a<0 else first_a
        if valid_b:count_b+=1;first_b=y['frame'] if first_b<0 else first_b
        if not(valid_a and valid_b):continue
        if abs(x['poseTimestamp']-y['poseTimestamp'])>1e-6:raise ValueError('Fresh pose timestamps differ')
        ta,tb=np.asarray(x['pose']).reshape(4,4),np.asarray(y['pose']).reshape(4,4)
        if not np.isfinite(ta).all() or not np.isfinite(tb).all():raise ValueError('Nonfinite fresh pose')
        positions.append(float(np.linalg.norm(ta[:3,3]-tb[:3,3])))
        cosine=float(np.clip((np.trace(ta[:3,:3].T@tb[:3,:3])-1)/2,-1,1))
        rotations.append(math.degrees(math.acos(cosine)))
    if not positions or min(count_a,count_b)==0:raise ValueError('No valid initialized pose matches')
    def statistics(values):return {'p95':float(np.percentile(values,95)),'max':max(values)}
    return {'expected_frames':expected,'native_poses':count_a,'browser_poses':count_b,'matched_poses':len(positions),
      'native_coverage':count_a/expected,'browser_coverage':count_b/expected,'coverage_difference':abs(count_a-count_b)/expected,
      'native_init_frame':first_a,'browser_init_frame':first_b,'init_frame_difference':abs(first_a-first_b),
      'translation_m':statistics(positions),'rotation_deg':statistics(rotations),'state_differences':state_differences}

def main():
    p=argparse.ArgumentParser();p.add_argument('--native',type=Path);p.add_argument('--browser',type=Path);p.add_argument('--expected-frames',type=int);p.add_argument('--contract',type=Path);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--freeze-pilot',nargs=2,action='append',metavar=('NATIVE','BROWSER'));p.add_argument('--solver-iterations',type=int,default=10);p.add_argument('--solver-time',type=float,default=10)
    args=p.parse_args()
    if args.freeze_pilot:
        if len(args.freeze_pilot)!=3:raise ValueError('Tolerance freezing requires exactly three development pilot pairs')
        samples=[compare(a,b,len(rows(a))) for a,b in args.freeze_pilot]
        if len({s['expected_frames'] for s in samples})!=1:raise ValueError('Pilot replay lengths differ')
        caps={'translation_m':{'p95':1e-3,'max':1e-2},'rotation_deg':{'p95':.01,'max':.1}}
        for sample in samples:
            if sample['expected_frames']!=2821 or sample['state_differences'] or sample['init_frame_difference'] or sample['coverage_difference']:
                raise ValueError('Three full room1 pilots must match init/fresh coverage/state/reason/solver/epoch')
            for axis in caps:
                for stat in caps[axis]:
                    if sample[axis][stat]>caps[axis][stat]:raise ValueError('Cross-platform pilot exceeds independent cap: '+axis+'.'+stat)
        repeats=[]
        for platform in (0,1):
            for run in (1,2):
                repeats.append(compare(args.freeze_pilot[0][platform],args.freeze_pilot[run][platform],2821))
        for repeat in repeats:
            if repeat['state_differences'] or repeat['init_frame_difference'] or repeat['coverage_difference']:raise ValueError('Within-platform repeats change state/coverage/solver')
            for axis in caps:
                for stat in caps[axis]:
                    if repeat[axis][stat]>caps[axis][stat]:raise ValueError('Within-platform repeat exceeds independent cap')
        # Caps precede the experiment; observed cross-platform error never expands acceptance.
        tolerance=caps
        tolerance['init_frame_difference']=0
        tolerance['coverage_difference']=0
        result={'schema':'mobile-slam-parity-contract-v1','status':'frozen_before_room4','development_pilots':[{'native':a,'browser':b} for a,b in args.freeze_pilot],
          'repeat_count':3,'pilot_frames':samples[0]['expected_frames'],'solver_iterations':args.solver_iterations,'solver_time_cap_s':args.solver_time,
          'tolerance_derivation':'independent pre-pilot caps; three paired and within-platform repeats must fit; no cross-error-derived widening','tolerances':tolerance,'samples':samples,'within_platform_repeats':repeats,
          'validation_expected_frames':{'room1':2821,'room4':2228},'alignment':'none_same_metric_frame','acceptance_scope':'same_input_dataset_parity_not_physical_mobile_accuracy'}
    else:
        if not(args.native and args.browser and args.expected_frames and args.contract):raise ValueError('Comparison requires paths, exact expected frames and frozen contract')
        contract=json.loads(args.contract.read_text());tolerance=contract.get('tolerances')
        if contract.get('status')!='frozen_before_room4' or not tolerance:raise ValueError('Parity contract is not frozen')
        result=compare(args.native,args.browser,args.expected_frames);failures=['state_reason_solver_epoch_or_freshness'] if result['state_differences'] else []
        for axis in ('translation_m','rotation_deg'):
            for stat in ('p95','max'):
                if result[axis][stat]>tolerance[axis][stat]:failures.append(axis+'.'+stat)
        for key in ('init_frame_difference','coverage_difference'):
            if result[key]>tolerance[key]:failures.append(key)
        result.update({'status':'fail' if failures else 'pass','failures':failures,'contract':str(args.contract),'acceptance_scope':'same_input_dataset_parity_not_physical_mobile_accuracy'})
    args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2))
    if result.get('status')=='fail':raise SystemExit(1)
if __name__=='__main__':main()
