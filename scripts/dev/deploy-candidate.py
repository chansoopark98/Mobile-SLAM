#!/usr/bin/env python3
"""Atomically promote the verified single-file embedded-WASM candidate."""
import argparse,hashlib,json,os,shutil,tempfile,time
from pathlib import Path
ROOT=Path(__file__).resolve().parents[2]
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser();p.add_argument('--candidate',type=Path,required=True);p.add_argument('--manifest',type=Path,required=True);p.add_argument('--authorization',type=Path,default=ROOT/'build/refactor-orchestration/deploy-authorized.json');args=p.parse_args()
    candidate=args.candidate.resolve();manifest=json.loads(args.manifest.read_text());authorization=json.loads(args.authorization.read_text())
    digest=sha(candidate)
    if authorization.get('candidate_sha256')!=digest or not authorization.get('authorized'):raise ValueError('Parent deployment gate does not authorize this candidate hash')
    if manifest['artifact']['sha256']!=digest or not manifest.get('single_file_embedded_wasm'):raise ValueError('Expected verified embedded-WASM single-file artifact')
    destination=ROOT/'web/vio_engine.js';backup=ROOT/'build/refactor-deploy'/str(time.time_ns());backup.mkdir(parents=True)
    if destination.exists():shutil.copy2(destination,backup/'vio_engine.js')
    sidecar=ROOT/'web/vio_engine.manifest.json'
    if sidecar.exists():shutil.copy2(sidecar,backup/sidecar.name)
    fd,name=tempfile.mkstemp(prefix='.vio-candidate-',dir=destination.parent)
    try:
        with os.fdopen(fd,'wb') as output:output.write(candidate.read_bytes());output.flush();os.fsync(output.fileno())
        if sha(Path(name))!=digest:raise ValueError('Candidate copy checksum mismatch')
        os.replace(name,destination)
    finally:
        if Path(name).exists():Path(name).unlink()
    manifest['deployment']={'backup_directory':str(backup),'served_js_sha256':digest,'parent_authorization':str(args.authorization),'single_file_atomic_promotion':True}
    temporary=sidecar.with_suffix('.json.tmp');temporary.write_text(json.dumps(manifest,indent=2)+'\n');os.replace(temporary,sidecar)
    (backup/'deployment.json').write_text(json.dumps(manifest['deployment'],indent=2)+'\n');print(json.dumps(manifest['deployment'],indent=2))
if __name__=='__main__':main()
