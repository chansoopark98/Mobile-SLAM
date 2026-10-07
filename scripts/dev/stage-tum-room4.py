#!/usr/bin/env python3
"""Extract verified NAS TUM rooms into owned local staging; never write the NAS."""
import argparse,csv,hashlib,json,os,shutil,struct,tarfile,tempfile
from pathlib import Path,PurePosixPath
ROOT=Path(__file__).resolve().parents[2]
def sha(path):
    value=hashlib.sha256()
    with path.open('rb') as source:
        for data in iter(lambda:source.read(1024*1024),b''):value.update(data)
    return value.hexdigest()
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--sequence',choices=('room1','room2','room3','room4'),default='room4');parser.add_argument('--inventory',type=Path,default=ROOT/'docs/audit/2026-10-06/datasets-inventory.json');parser.add_argument('--destination',type=Path);args=parser.parse_args()
    dataset_name=f'dataset-{args.sequence}_512_16'
    inventory=json.loads(args.inventory.read_text());entry=next(x for x in inventory['tum_vi'] if Path(x['archive']).name==f'{dataset_name}.tar')
    if entry['archive_root']!=dataset_name:raise ValueError('Inventory archive root differs from selected sequence')
    archive=Path(entry['archive']);destination=(args.destination or ROOT/'build/refactor-data/tum'/dataset_name).resolve()
    if not destination.is_relative_to(ROOT/'build'):raise ValueError('Destination must stay in owned ignored build directory')
    if destination.exists():raise FileExistsError('Staging already exists; preserve it and use its manifest')
    before=archive.stat();digest=sha(archive)
    destination.parent.mkdir(parents=True,exist_ok=True);temporary=Path(tempfile.mkdtemp(prefix=f'.{args.sequence}-stage-',dir=destination.parent))
    try:
        total=0;members=0;omitted_symlinks=[]
        with tarfile.open(archive,'r:') as source:
            for member in source:
                path=PurePosixPath(member.name)
                if path.is_absolute() or '..' in path.parts or not path.parts or path.parts[0]!=entry['archive_root']:raise ValueError('Unsafe archive member')
                if member.issym():
                    omitted_symlinks.append({'member':member.name,'target':member.linkname,'reason':'unused convenience alias; symlinks are never extracted'})
                    continue
                if not(member.isdir() or member.isfile()):raise ValueError('Unsupported archive member type')
                total+=member.size;members+=1
                if total>5*1024**3 or members>30000:raise ValueError('Archive exceeds staging budget')
                target=temporary.joinpath(*path.parts)
                if member.isdir():target.mkdir(parents=True,exist_ok=True);continue
                target.parent.mkdir(parents=True,exist_ok=True)
                if target.exists():raise ValueError('Duplicate archive member')
                stream=source.extractfile(member)
                with target.open('xb') as output:shutil.copyfileobj(stream,output)
        dataset=temporary/entry['archive_root'];verified={}
        for key,item in entry['csv'].items():
            path=temporary/item['member'];value=sha(path)
            if value!=item['sha256']:raise ValueError(f'{key} inventory hash mismatch')
            with path.open() as source:rows=[row for row in csv.reader(source) if row and not row[0].startswith('#')]
            if len(rows)!=item['rows'] or any(int(rows[i][0])<=int(rows[i-1][0]) for i in range(1,len(rows))):raise ValueError(f'{key} count/order mismatch')
            verified[key]={'sha256':value,'records':len(rows)}
            if key in ('cam0','cam1'):
                for row in rows:
                    image=dataset/'mav0'/key/'data'/row[1]
                    if Path(row[1]).name!=row[1]:raise ValueError('Unsafe CSV image name')
                    with image.open('rb') as stream:header=stream.read(24)
                    if header[:8]!=b'\x89PNG\r\n\x1a\n' or struct.unpack('>II',header[16:24])!=(512,512):raise ValueError('Invalid image header/resolution')
        for item in entry['yaml']:
            path=temporary/item['member']
            if sha(path)!=item['sha256']:raise ValueError('Calibration inventory mismatch')
        after=archive.stat()
        if (before.st_size,before.st_mtime_ns)!=(after.st_size,after.st_mtime_ns):raise ValueError('NAS archive changed while staging')
        manifest={'schema':'mobile-slam-staged-dataset-v1','source_read_only':True,'source':str(archive),'archive_sha256':digest,'archive_bytes':before.st_size,'members':members,'omitted_symlinks':omitted_symlinks,'uncompressed_bytes':total,'verified_csv':verified,'calibration_verified_against_inventory':True,'destination':str(destination),'split':'locked_final_confirmation' if args.sequence=='room2' else 'selection_validation_not_locked_final'}
        (dataset/'staging-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n');os.replace(dataset,destination);print(json.dumps(manifest,indent=2))
    finally:shutil.rmtree(temporary)
if __name__=='__main__':main()
