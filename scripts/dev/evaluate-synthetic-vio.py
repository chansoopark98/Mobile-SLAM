#!/usr/bin/env python3
"""Score synthetic physical-frame poses with fixed source coverage and SE(3), never per epoch."""
import argparse
import hashlib
import importlib.util
import json
import pathlib

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = pathlib.Path(__file__).resolve().parents[2]
_module = importlib.util.spec_from_file_location('synthetic_audit_metrics',ROOT/'scripts/evaluation/audit_metrics.py')
METRICS = importlib.util.module_from_spec(_module)
_module.loader.exec_module(METRICS)


def result_rows(document):
    if isinstance(document,list):return document
    for key in ('frames','rows','results'):
        if isinstance(document.get(key),list):return document[key]
    for key in ('replay','synthetic'):
        if isinstance(document.get(key),dict):return result_rows(document[key])
    raise ValueError('Results must include a frames/rows/results array')



def validate_truth(spec, truth):
    scheduled = spec['frame_count']
    hz = spec['image_hz']
    limits = spec['acceptance']
    if (not isinstance(scheduled, int) or isinstance(scheduled, bool) or scheduled <= 0 or
            not np.isfinite(hz) or hz <= 0 or
            abs(scheduled / hz - spec['duration_s']) > 1e-12):
        raise ValueError('Invalid scheduled source-frame specification')
    frames = truth.get('frames')
    if not isinstance(frames, list) or len(frames) != scheduled:
        raise ValueError('Truth frame count differs from frozen specification')
    times = np.arange(scheduled, dtype=float) / hz
    for index, frame in enumerate(frames):
        if frame.get('index') != index:
            raise ValueError('Truth frame index differs from scheduled source index')
        timestamp = frame.get('timestamp')
        if timestamp is None or not np.isfinite(timestamp) or abs(timestamp - times[index]) > 1e-12:
            raise ValueError('Truth timestamp grid differs from frozen specification')
    warmup = [i for i, t in enumerate(times) if limits['warmup_start_s'] <= t < limits['source_end_s']]
    if not warmup or len(warmup) != limits['scheduled_warmup_frames']:
        raise ValueError('Frozen warmup frame count differs from source time grid')
    return times, warmup


def processed_identity(row, times, tolerance):
    if not isinstance(row, dict):
        raise ValueError('Result row requires source identity and inputTimestamp')
    input_timestamp = row.get('inputTimestamp')
    if (not isinstance(input_timestamp, (int, float, np.number)) or isinstance(input_timestamp, bool) or
            not np.isfinite(input_timestamp) or input_timestamp < 0):
        raise ValueError('Result row requires finite nonnegative inputTimestamp')
    explicit = row.get('sourceFrameIndex') is not None
    index = row.get('sourceFrameIndex', row.get('frame'))
    if index is None:
        insertion = int(np.searchsorted(times, input_timestamp))
        candidates = [i for i in (insertion - 1, insertion) if 0 <= i < len(times)]
        index = min(candidates, key=lambda i: abs(times[i] - input_timestamp)) if candidates else -1
        if index < 0 or abs(times[index] - input_timestamp) > tolerance:
            raise ValueError('Result source identity cannot be associated with scheduled input')
    elif not isinstance(index, (int, np.integer)) or isinstance(index, bool) or not 0 <= index < len(times):
        raise ValueError('Invalid physical source-frame index')
    canonical_input = float(round(times[index] * 1e9)) * 1e-9
    if not explicit and abs(input_timestamp - canonical_input) > 1e-9:
        raise ValueError('Result input timestamp differs from its declared source frame')
    physical_timestamp = row.get('physicalTimestamp', times[index])
    if not np.isfinite(physical_timestamp) or abs(physical_timestamp - times[index]) > 1e-12:
        raise ValueError('Result physical timestamp differs from its declared source frame')
    return int(index), float(input_timestamp), explicit


def load_dataset(dataset):
    dataset = pathlib.Path(dataset)
    manifest_bytes = (dataset / 'manifest.json').read_bytes()
    manifest = json.loads(manifest_bytes)
    if manifest.get('schema') != 'mobile-slam-synthetic-manifest-v1':
        raise ValueError('Synthetic manifest required')
    documents = {}
    hashes = {}
    for name in ('spec.json', 'groundtruth.json'):
        data = (dataset / name).read_bytes()
        actual = hashlib.sha256(data).hexdigest()
        expected = manifest.get('files', {}).get(name, {})
        if expected.get('bytes') != len(data) or expected.get('sha256') != actual:
            raise ValueError(f'{name} hash or size differs from manifest')
        documents[name] = json.loads(data)
        hashes[name.removesuffix('.json') + '_sha256'] = actual
    if manifest.get('spec_sha256') != hashes['spec_sha256']:
        raise ValueError('Manifest spec hash differs from actual specification')
    spec, truth = documents['spec.json'], documents['groundtruth.json']
    if spec.get('schema') != 'mobile-slam-synthetic-spec-v1' or truth.get('schema') != 'mobile-slam-synthetic-groundtruth-v1':
        raise ValueError('Synthetic specification and ground-truth schemas required')
    validate_truth(spec, truth)
    hashes['manifest_sha256'] = hashlib.sha256(manifest_bytes).hexdigest()
    return spec, truth, hashes


def evaluate_rows(spec,truth,rows,pose_frame='camera'):
    if pose_frame not in ('camera','body'):raise ValueError('pose_frame must be camera or body')
    gt=truth['frames'];limits=spec['acceptance'];scheduled=spec['frame_count'];accepted=[];seen=set();invalid=0;unassociated=0
    times,warmup_indices=validate_truth(spec,truth)
    raw_pose_timestamps=[];source_time_mappings=[];processed_indices=[];processed_mappings=[];previous_input=-np.inf
    for row in rows:
        index,input_timestamp,explicit_mapping=processed_identity(row,times,limits['association_tolerance_s'])
        if index in processed_indices:raise ValueError('Duplicate physical source frame in processed results')
        if processed_indices and index<=processed_indices[-1]:raise ValueError('Processed source frames must be strictly ordered')
        if input_timestamp<=previous_input:raise ValueError('Processed input timestamps must be strictly ordered')
        processed_indices.append(index);previous_input=input_timestamp
        processed_mappings.append({'sourceFrameIndex':index,'physicalTimestamp':float(times[index]),
            'inputTimestamp':input_timestamp,'explicitPhysicalMapping':explicit_mapping})
        if not row.get('poseValid',False) or row.get('poseFresh',True) is False or row.get('pose') is None:continue
        pose=np.asarray(row['pose'],dtype=float)
        if pose.size!=16 or not np.isfinite(pose).all():invalid+=1;continue
        pose=pose.reshape(4,4);r=pose[:3,:3]
        if not np.allclose(pose[3],[0,0,0,1],atol=1e-8) or np.linalg.norm(r.T@r-np.eye(3))>1e-6 or abs(np.linalg.det(r)-1)>1e-6:
            invalid+=1;continue
        timestamp=row.get('poseTimestamp')
        if timestamp is None or not np.isfinite(timestamp):invalid+=1;continue
        if abs(timestamp-input_timestamp)>1e-9:
            invalid+=1;continue
        seen.add(index);raw_pose_timestamps.append(float(timestamp))
        source_time_mappings.append({'sourceFrameIndex':index,'physicalTimestamp':float(times[index]),
            'inputTimestamp':float(input_timestamp),'poseTimestamp':float(timestamp),
            'explicitPhysicalMapping':explicit_mapping})
        position=pose[:3,3].copy()
        if pose_frame=='camera':
            r=r@np.asarray(spec['camera']['r_ic']).reshape(3,3).T
            position-=r@np.asarray(spec['camera']['t_ic'])
        accepted.append((index,[times[index],*position,*Rotation.from_matrix(r).as_quat()]))
    warmup_count=sum(i in seen for i in warmup_indices)
    epoch_rows=[row.get('engineEpoch') for row in rows]
    epoch_known=bool(rows) and all(isinstance(e,(int,float)) and np.isfinite(e) for e in epoch_rows)
    resets=sum(b!=a for a,b in zip(epoch_rows,epoch_rows[1:])) if epoch_known else None
    report={'schema':'mobile-slam-synthetic-evaluation-v1','alignment':'SE3_scale_fixed_1_single_global',
        'scope':'Synthetic test-rig acceptance only; no physical phone or field-accuracy claim',
        'scheduled_frames':scheduled,'completed_rows':len(rows),'processed_source_indices':processed_indices,
        'processed_source_time_mappings':processed_mappings,'fresh_valid_associated_poses':len(accepted),
        'invalid_pose_rows':invalid,'unassociated_pose_rows':unassociated,'full_source_coverage':len(accepted)/scheduled,
        'postwarmup_source_frames':len(warmup_indices),'postwarmup_fresh_poses':warmup_count,
        'postwarmup_coverage':warmup_count/len(warmup_indices) if warmup_indices else 0.,
        'first_fresh_physical_time_s':times[accepted[0][0]].item() if accepted else None,
        'observed_epoch_transitions':resets,'reported_pose_timestamps':raw_pose_timestamps,
        'physical_source_indices':[i for i,_ in accepted],'source_time_mappings':source_time_mappings,'metrics':None,'failures':[]}
    if limits['expected_degenerate']:
        report['passed']=processed_indices==list(range(scheduled)) and len(accepted)==0 and invalid==0
        if not report['passed']:report['failures'].append('degenerate scenario needs full processing and no false metric poses')
        return report
    if len(accepted)>=3:
        estimate=np.asarray([values for _,values in accepted]);ground=np.asarray([[frame['timestamp'],*frame['body']['position'],*frame['body']['quaternion_xyzw']] for frame in gt])
        try:report['metrics']=METRICS.evaluate(estimate,ground,scheduled,limits['association_tolerance_s'],limits['rpe_interval_s'],limits['rpe_tolerance_s'])
        except ValueError as error:report['failures'].append(str(error))
    else:report['failures'].append('fewer than three valid physical pose associations')
    if invalid:report['failures'].append('invalid pose rows')
    if unassociated:report['failures'].append('unassociated pose rows')
    if report['postwarmup_coverage']<limits['postwarmup_coverage']:report['failures'].append('fixed source coverage below threshold')
    if report['first_fresh_physical_time_s'] is None or report['first_fresh_physical_time_s']>limits['first_pose_latest_s']:report['failures'].append('initialization deadline missed')
    metrics=report['metrics']
    if metrics:
        if metrics['ate_translation_m']['rmse']>limits['ate_rmse_m']:report['failures'].append('metric translation ATE above threshold')
        if metrics['ape_rotation_deg'] is None or metrics['ape_rotation_deg']['rmse']>limits['ape_rmse_deg']:report['failures'].append('rotation APE unavailable or above threshold')
        if not spec['scenario'].startswith('mobile'):
            if resets!=0:report['failures'].append('clean sequence reset or epoch evidence missing')
            rpe=metrics['rpe']
            if rpe is None or rpe['translation_m']['rmse']>limits['clean_rpe_m'] or rpe['rotation_deg']['rmse']>limits['clean_rpe_deg']:report['failures'].append('clean RPE unavailable or above threshold')
    report['passed']=not report['failures']
    return report


def load_trajectory(path):
    poses=METRICS.load_poses(path)
    rows=[]
    for values in poses:
        matrix=np.eye(4);matrix[:3,:3]=Rotation.from_quat(values[4:]).as_matrix();matrix[:3,3]=values[1:4]
        rows.append({'poseValid':True,'poseFresh':True,'inputTimestamp':float(values[0]),'poseTimestamp':float(values[0]),'pose':matrix.ravel().tolist()})
    return rows


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--dataset',required=True,type=pathlib.Path)
    group=parser.add_mutually_exclusive_group(required=True);group.add_argument('--results',type=pathlib.Path);group.add_argument('--trajectory',type=pathlib.Path)
    parser.add_argument('--estimate-frame',choices=('body','camera'),default='camera');parser.add_argument('--output',required=True,type=pathlib.Path)
    args=parser.parse_args(argv);dataset=args.dataset.resolve();output=args.output.resolve()
    if dataset==output or dataset in output.parents:raise ValueError('Evaluation output must be outside frozen dataset')
    spec,truth,provenance=load_dataset(dataset)
    source=args.results or args.trajectory;rows=result_rows(json.loads(source.read_text())) if args.results else load_trajectory(source)
    report=evaluate_rows(spec,truth,rows,args.estimate_frame)
    report['inputs']={'dataset':str(dataset),**provenance,
        'results':str(source.resolve()),'results_sha256':hashlib.sha256(source.read_bytes()).hexdigest(),'estimate_frame':args.estimate_frame}
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as f:json.dump(report,f,indent=2,allow_nan=False);f.write('\n')
    print(json.dumps({k:report[k] for k in ['passed','fresh_valid_associated_poses','postwarmup_coverage','observed_epoch_transitions','failures']},indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
