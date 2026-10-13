#!/usr/bin/env python3
"""Own one Docker Android environment; never address the host ADB server."""
import argparse,json,pathlib,subprocess,time
ROOT=pathlib.Path(__file__).resolve().parents[4]
REFERENCE=ROOT/'assets/references/ARCore'
EVIDENCE=ROOT/'build/research-20261012/commercial-experiments'
CONTAINER='mobile-slam-arcore-20261012'
IMAGE='mobile-slam-arcore-env:20261012'
def command(name,argv,input=None):
    started=time.monotonic_ns()
    with(EVIDENCE/(name+'.log')).open('x')as out:r=subprocess.run(argv,input=input,text=True,stdout=out,stderr=subprocess.STDOUT)
    record={'argv':argv,'exit':r.returncode,'durationMonotonicMs':(time.monotonic_ns()-started)/1e6,'log':str(EVIDENCE/(name+'.log'))}
    (EVIDENCE/(name+'-command.json')).write_text(json.dumps(record,indent=2)+'\n');print(name,r.returncode,flush=True)
    return r.returncode

def main():
    p=argparse.ArgumentParser();p.add_argument('phase',choices=['start','sdk']);a=p.parse_args();EVIDENCE.mkdir(parents=True,exist_ok=True)
    if a.phase=='start':
        result=subprocess.run(['docker','container','inspect',CONTAINER],capture_output=True,text=True)
        if result.returncode==0:
            existing=json.loads(result.stdout)[0]
            if existing.get('Config',{}).get('Labels',{}).get('mobile-slam-reference')!='arcore-20261012':raise RuntimeError('Container is not owned by this reference task')
            raise RuntimeError('Owned container already exists; inspect rather than reset')
        status=command('docker-runtime-start',['docker','run','-d','--name',CONTAINER,'--label','mobile-slam-reference=arcore-20261012','--device','/dev/kvm','--shm-size=2g','--cpus=2','--memory=8g','-v',str(REFERENCE)+':/work',IMAGE])
        if status:raise SystemExit(status)
        status=command('sdkmanager-version',['docker','exec',CONTAINER,'sdkmanager','--sdk_root=/work/toolchain/sdk','--version'])
        if status:raise SystemExit(status)
    else:
        if command('sdk-licenses',['docker','exec','-i',CONTAINER,'sdkmanager','--sdk_root=/work/toolchain/sdk','--licenses'],input='y\n'*1000):raise SystemExit(1)
        command('sdk-platform37-attempt',['docker','exec','-i',CONTAINER,'sdkmanager','--sdk_root=/work/toolchain/sdk','platforms;android-37'],input='y\n'*1000)
        status=command('sdk-packages-install',['docker','exec','-i',CONTAINER,'sdkmanager','--sdk_root=/work/toolchain/sdk','platform-tools','emulator','platforms;android-35','platforms;android-36','build-tools;35.0.0','build-tools;34.0.0','system-images;android-35;google_apis;x86_64'],input='y\n'*1000)
        if status:raise SystemExit(status)
        status=command('sdk-package-list',['docker','exec',CONTAINER,'sdkmanager','--sdk_root=/work/toolchain/sdk','--list_installed'])
        if status:raise SystemExit(status)
if __name__=='__main__':main()
