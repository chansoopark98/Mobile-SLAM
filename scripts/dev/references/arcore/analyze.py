#!/usr/bin/env python3
"""Analyze actual SDK rows; no synthetic solver timing or accuracy claim."""
import argparse,collections,hashlib,json,math,pathlib,statistics,tarfile

def pct(values,q):
    if not values:return None
    values=sorted(values);k=(len(values)-1)*q;i=int(k);return values[i]+(values[min(i+1,len(values)-1)]-values[i])*(k-i)
def stats(values):return {'count':len(values),'p50':pct(values,.5),'p95':pct(values,.95),'p99':pct(values,.99),'min':min(values)if values else None,'max':max(values)if values else None}
def read(run):
    summary=json.loads((run/'summary.json').read_text());mode=summary['mode'];rows=[json.loads(line)for line in(run/(mode+'-poses.jsonl')).read_text().splitlines()];frames=[r for r in rows if r.get('event')=='frame'];imu=[json.loads(line)for line in(run/(mode+'-imu.jsonl')).read_text().splitlines()]
    times=[int(r['timestampNs'])for r in frames];tracking=[r for r in frames if r['trackingState']=='TRACKING'];trans=[];quat=[];orth=[]
    for r in frames:
        m=r['matrixColumnMajorT_W_C_OpenGL'];p=r['translationM'];q=r['quaternionXyzw'];trans.append(max(abs(m[12+i]-p[i])for i in range(3)));quat.append(abs(sum(x*x for x in q)-1));orth.append(max(abs(sum(m[k*4+i]*m[k*4+j]for k in range(3))-(1 if i==j else 0))for i in range(3)for j in range(3)))
    image_verification={'tarAvailable':False,'verifiedPayloads':0,'failures':[]}
    tar=run/(mode+'-images.tar')
    if tar.exists():
        image_verification['tarAvailable']=True
        with tarfile.open(tar)as archive:
            for r in frames:
                if 'imagePayloadSha256'not in r:continue
                member=archive.extractfile('reference-'+mode+'-images/'+f"{r['index']:06d}.pgm")
                data=member.read();payload=data.split(b'\n',3)[3]
                if hashlib.sha256(payload).hexdigest()!=r['imagePayloadSha256']:image_verification['failures'].append(r['index'])
                image_verification['verifiedPayloads']+=1
    by_sensor=collections.defaultdict(list)
    for r in imu:by_sensor[r['sensorType']].append(int(r['timestampNs']))
    output={'run':str(run),'summary':summary,'frames':len(frames),'trackingFrames':len(tracking),'trackingStates':dict(collections.Counter(r['trackingState']for r in frames)),'trackingFailureReasons':dict(collections.Counter(r['trackingFailureReason']for r in frames)),'firstTrackingFrame':tracking[0]['index']if tracking else None,'timestampStrictlyIncreasing':all(b>a for a,b in zip(times,times[1:])),'sourceDurationS':(times[-1]-times[0])/1e9 if len(times)>1 else None,'uniqueTimestamps':len(set(times)),'sessionUpdateBlockingApiMs':stats([int(r['sessionUpdateDurationNs'])/1e6 for r in frames]),'cameraIntervalsMs':stats([(b-a)/1e6 for a,b in zip(times,times[1:])]),'pointCloudCounts':stats([r['pointCloudPoints']for r in frames]),'imageTimestampMinusFrameNs':stats([int(r['imageTimestampNs'])-int(r['timestampNs'])for r in frames if'imageTimestampNs'in r]),'cpuImageDimensions':list({tuple(r['cpuImageDimensions'])for r in frames}),'publicPoseContract':{'translationMatrixMaxAbsDiffM':max(trans)if trans else None,'quaternionNormSquaredMaxAbsError':max(quat)if quat else None,'rotationOrthogonalityMaxAbsError':max(orth)if orth else None},'images':image_verification,'externalImu':{str(k):{'count':len(v),'strictlyIncreasing':all(b>a for a,b in zip(v,v[1:])),'intervalMs':stats([(b-a)/1e6 for a,b in zip(v,v[1:])])}for k,v in by_sensor.items()},'privateSolverTiming':'UNAVAILABLE','independentGroundTruth':'UNAVAILABLE','externalImuProvenIdenticalToARCoreInternal':'UNVERIFIED'}
    return output,frames

def compare(a,b):
    ma={int(r['timestampNs']):r for r in a};mb={int(r['timestampNs']):r for r in b};common=sorted(ma.keys()&mb.keys());image_common=[t for t in common if'imagePayloadSha256'in ma[t]and'imagePayloadSha256'in mb[t]];tracked=[t for t in common if ma[t]['trackingState']==mb[t]['trackingState']=='TRACKING'];dist=[];angles=[]
    # Worlds can differ between sessions; raw differences are agreement diagnostics, not ATE.
    for t in tracked:
        x=ma[t];y=mb[t];dist.append(math.sqrt(sum((a-b)**2 for a,b in zip(x['translationM'],y['translationM']))));qx=x['quaternionXyzw'];qy=y['quaternionXyzw'];norm=math.sqrt(sum(v*v for v in qx)*sum(v*v for v in qy));angles.append(2*math.acos(min(1,abs(sum(a*b for a,b in zip(qx,qy))/norm)))*180/math.pi)
    return {'sameMp4Input':'caller must verify recording hash','framesA':len(a),'framesB':len(b),'exactTimestampIntersection':len(common),'imageHashComparable':len(image_common),'imageHashEqual':sum(ma[t]['imagePayloadSha256']==mb[t]['imagePayloadSha256']for t in image_common),'trackingIntersection':len(tracked),'statusMismatchesAtCommonTimestamps':sum(ma[t]['trackingState']!=mb[t]['trackingState']for t in common),'rawWorldTranslationDifferenceM':stats(dist),'rawWorldRotationDifferenceDeg':stats(angles),'accuracyClaim':False,'alignment':'none; world gauges may differ','arcoreInternalImuBytesComparable':'UNAVAILABLE'}
def main():
    p=argparse.ArgumentParser();p.add_argument('runs',type=pathlib.Path,nargs='+');p.add_argument('--output',required=True,type=pathlib.Path);a=p.parse_args();data=[read(p)for p in a.runs];out={'experiments':[x[0]for x in data]}
    if len(data)==2:out['pairedPlaybackAgreement']=compare(data[0][1],data[1][1])
    a.output.write_text(json.dumps(out,indent=2)+'\n');print(json.dumps({'runs':len(data),'frames':[x[0]['frames']for x in data],'trackingFrames':[x[0]['trackingFrames']for x in data]}))
if __name__=='__main__':main()
