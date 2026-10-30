#!/usr/bin/env python3
"""SHA256 exact native OpenCV decoded bytes and serialized IMU packets."""
import argparse,hashlib,json,struct
from pathlib import Path

def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda:source.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--raw',type=Path,required=True);p.add_argument('--dataset',type=Path,required=True);p.add_argument('--config',type=Path,required=True);p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    result={'schema':'mobile-slam-input-oracle-v1','gray_decoder':'native linked OpenCV IMREAD_GRAYSCALE uint8','imu_encoding':'IEEE754 Float64 little endian timestamp,ax,ay,az,gx,gy,gz','source_sha256':{'config':sha(args.config)},'frames':[]}
    for name in ('cam0','imu0','mocap0'):result['source_sha256'][name]=sha(args.dataset/'mav0'/name/'data.csv')
    with args.raw.open('rb') as source:
        def exact(count):
            value=source.read(count)
            if len(value)!=count:raise ValueError('Truncated native input oracle')
            return value
        frame=0
        while header:=source.read(4):
            if len(header)!=4:raise ValueError('Truncated image size')
            size=struct.unpack('<I',header)[0]
            if not 0<size<=16*1024**2:raise ValueError('Invalid image size')
            gray=exact(size);count=struct.unpack('<I',exact(4))[0]
            if count>100000:raise ValueError('Invalid IMU count')
            packet=exact(count*56)
            result['frames'].append({'frame':frame,'imageSha256':hashlib.sha256(gray).hexdigest(),'imageBytes':size,
              'imuSha256':hashlib.sha256(packet).hexdigest(),'imuCount':count,'imuFirstTimestamp':struct.unpack('<d',packet[:8])[0] if count else None,'imuLastTimestamp':struct.unpack('<d',packet[-56:-48])[0] if count else None})
            frame+=1
    result['frame_count']=len(result['frames']);args.output.write_text(json.dumps(result,indent=2)+'\n');print(args.output)
if __name__=='__main__':main()
