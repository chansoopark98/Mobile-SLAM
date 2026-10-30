#!/usr/bin/env python3
"""Fetch pinned official ARCore emulator prerequisites into ignored references."""
import argparse,hashlib,json,os,pathlib,urllib.request,zipfile
ROOT=pathlib.Path(__file__).resolve().parents[4]
REFERENCE=ROOT/'assets/references/ARCore'
EVIDENCE=ROOT/'build/research-20261012/commercial-experiments'
COMMAND_TOOLS={'url':'https://dl.google.com/android/repository/commandlinetools-linux-15859902_latest.zip','sha256':'4e4c464f145a7512b57d088ac6c278c03c9eea610886b35a5e0804e74eedf583','maxBytes':250*1024*1024}
ARCORE_EMULATOR={'url':'https://github.com/google-ar/arcore-android-sdk/releases/download/1.56.0/Google_Play_Services_for_AR_1.56.0_x86_for_emulator.apk','sha256':'7cbeb1bba836ab69551ac7c7ae64ed6e94f0b2818fc921b83d5edd57f51f4066','maxBytes':120*1024*1024}
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def download(spec,path):
    path.parent.mkdir(parents=True,exist_ok=True)
    if path.exists():
        if spec.get('sha256')and sha(path)!=spec['sha256']:raise ValueError('Existing hash mismatch '+str(path))
    else:
        tmp=path.with_suffix(path.suffix+'.partial')
        with urllib.request.urlopen(spec['url'],timeout=60)as src,tmp.open('xb')as out:
            count=0
            while chunk:=src.read(1024*1024):
                count+=len(chunk)
                if count>spec['maxBytes']:raise ValueError('Bounded download exceeded')
                out.write(chunk)
        if spec.get('sha256')and sha(tmp)!=spec['sha256']:raise ValueError('Publisher hash mismatch')
        tmp.rename(path)
    return {'url':spec['url'],'path':str(path),'bytes':path.stat().st_size,'sha256':sha(path),'publisherChecksumVerified':bool(spec.get('sha256'))}
def main():
    p=argparse.ArgumentParser();p.add_argument('--tools',action='store_true');p.add_argument('--arcore-apk',action='store_true');a=p.parse_args();EVIDENCE.mkdir(parents=True,exist_ok=True);records=[]
    if a.tools:
        path=REFERENCE/'toolchain/downloads/commandlinetools-linux-15859902_latest.zip';records.append(download(COMMAND_TOOLS,path));out=REFERENCE/'toolchain/sdk/cmdline-tools/latest';out.mkdir(parents=True,exist_ok=True)
        with zipfile.ZipFile(path)as z:
            for info in z.infolist():
                parts=pathlib.PurePosixPath(info.filename).parts
                if not parts or parts[0]!='cmdline-tools'or'..'in parts:raise ValueError('Unsafe SDK zip entry')
                target=out.joinpath(*parts[1:])
                if info.is_dir():target.mkdir(parents=True,exist_ok=True);continue
                target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(z.read(info));os.chmod(target,((info.external_attr>>16)&0o777) or 0o644)
    if a.arcore_apk:records.append(download(ARCORE_EMULATOR,REFERENCE/'toolchain/downloads/Google_Play_Services_for_AR_1.56.0_x86_for_emulator.apk'))
    name='download-'+('-'.join(k for k,b in [('tools',a.tools),('apk',a.arcore_apk)]if b))+'.json';(EVIDENCE/name).write_text(json.dumps(records,indent=2)+'\n');print(json.dumps(records,indent=2))
if __name__=='__main__':main()
