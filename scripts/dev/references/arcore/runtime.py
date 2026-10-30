#!/usr/bin/env python3
"""Boot and observe the owned ARCore virtual device through its private ADB."""
import argparse,hashlib,json,pathlib,subprocess,time
from environment import ROOT,REFERENCE,EVIDENCE,CONTAINER
SERIAL='emulator-5556'
PACKAGE='com.google.ar.core.examples.java.helloar'
AVD='serdic_arcore_api30'
AVD_HOME='/work/toolchain/android-user/avd'
class Runner:
    def __init__(self,name):self.out=EVIDENCE/name;self.out.mkdir(exist_ok=False);self.records=[];(self.out/'runner-source.sha256').write_text(hashlib.sha256(pathlib.Path(__file__).read_bytes()).hexdigest()+'\n')
    def host(self,name,argv,input=None,check=True):
        start=time.monotonic_ns()
        with(self.out/(name+'.log')).open('x')as log:result=subprocess.run(argv,input=input,text=True,stdout=log,stderr=subprocess.STDOUT)
        record={'name':name,'argv':argv,'exit':result.returncode,'monotonicDurationMs':(time.monotonic_ns()-start)/1e6};self.records.append(record);(self.out/'commands.json').write_text(json.dumps(self.records,indent=2)+'\n');print(name,result.returncode,flush=True)
        if check and result.returncode:raise RuntimeError(name+' failed; see '+str(self.out/(name+'.log')))
        return result.returncode
    def run(self,name,args,**kwargs):return self.host(name,['docker','exec',*( ['-i'] if kwargs.get('input') is not None else []),'-e','ANDROID_AVD_HOME='+AVD_HOME,CONTAINER,*args],**kwargs)
    def adb(self,name,args,**kwargs):return self.run(name,['adb','-s',SERIAL,*args],**kwargs)

def boot(a):
    r=Runner(a.attempt)
    r.host('resume-owned-container',['docker','unpause',CONTAINER],check=False)
    result=subprocess.run(['docker','exec',CONTAINER,'test','-d',AVD_HOME+'/'+AVD+'.avd'],capture_output=True)
    if result.returncode:r.run('avd-create',['avdmanager','create','avd','--name',AVD,'--package','system-images;android-30;google_apis;x86_64','--device','pixel_3a'],input='no\n')
    code="""from pathlib import Path
p=Path('/work/toolchain/android-user/avd/serdic_arcore_api30.avd/config.ini');s=dict(line.split('=',1)for line in p.read_text().splitlines()if '='in line);s.update({'hw.camera.back':'virtualscene','hw.camera.front':'none','hw.gpu.enabled':'yes','hw.gpu.mode':'swiftshader','hw.cpu.ncore':'2','hw.ramSize':'4096','hw.lcd.width':'720','hw.lcd.height':'1280','hw.lcd.density':'320','disk.dataPartition.size':'4G','showDeviceFrame':'no'});p.write_text(''.join(k+'='+v+'\\n'for k,v in s.items()));print(p)
"""
    r.run('avd-config',['python3','-c',code]);
    grpc_code="import grpc_tools,pathlib,subprocess; d=pathlib.Path('/work/toolchain/grpc-generated'); d.mkdir(exist_ok=True); inc=pathlib.Path(grpc_tools.__file__).parent/'_proto'; subprocess.run(['/work/toolchain/grpc-python/bin/python','-m','grpc_tools.protoc','-I/work/toolchain/sdk/emulator/lib','-I'+str(inc),'--python_out='+str(d),'--grpc_python_out='+str(d),'/work/toolchain/sdk/emulator/lib/emulator_controller.proto'],check=True)"
    r.run('grpc-generate',['/work/toolchain/grpc-python/bin/python','-c',grpc_code]);r.run('accel-check',['emulator','-accel-check']);r.run('artifact-dir',['mkdir','-p','/work/artifacts'])
    argv=['docker','exec','-d','-e','ANDROID_AVD_HOME='+AVD_HOME,CONTAINER,'bash','-c','exec emulator -avd serdic_arcore_api30 -port 5556 -grpc 8554 -no-window -no-audio -gpu swiftshader -accel on -cores 2 -memory 4096 -no-snapshot -no-boot-anim -camera-back virtualscene > /work/artifacts/emulator.log 2>&1']
    result=subprocess.run(argv,capture_output=True,text=True);r.records.append({'name':'emulator-start','argv':argv,'exit':result.returncode});assert result.returncode==0,result.stderr
    r.run('adb-start-owned',['adb','start-server']);deadline=time.monotonic()+240
    while time.monotonic()<deadline:
        result=subprocess.run(['docker','exec',CONTAINER,'adb','-s',SERIAL,'shell','getprop','sys.boot_completed'],capture_output=True,text=True,timeout=8)
        if result.returncode==0 and result.stdout.strip()=='1':break
        print('waiting for owned virtual device boot',round(deadline-time.monotonic()),flush=True);time.sleep(5)
    else:raise RuntimeError('Owned emulator boot timeout; preserve emulator.log')
    r.adb('device-properties',['shell','getprop']);r.adb('virtual-console-help',['emu','help']);r.adb('virtual-sensors-help',['emu','sensor','status']);r.adb('disable-auto-rotation',['shell','settings','put','system','accelerometer_rotation','0']);r.adb('install-arcore',['install','-r','/work/toolchain/downloads/Google_Play_Services_for_AR_1.56.0_x86_for_emulator.apk']);r.adb('arcore-package',['shell','dumpsys','package','com.google.ar.core']);r.adb('install-observed-sample',['install','-r','-g','/work/source/samples/hello_ar_java/app/build/outputs/apk/debug/app-debug.apk']);r.adb('sample-package',['shell','dumpsys','package',PACKAGE]);(r.out/'commands.json').write_text(json.dumps(r.records,indent=2)+'\n');print('owned emulator and public SDK sample ready',flush=True)

def experiment(a):
    if a.coldboot:
        restart=Runner(a.attempt+'-restart')
        restart.host('resume-owned-container',['docker','unpause',CONTAINER],check=False)
        restart.adb('stop-owned-emulator',['emu','kill'],check=False)
        time.sleep(3)
        import types
        boot(types.SimpleNamespace(attempt=a.attempt+'-coldboot'))
    r=Runner(a.attempt);r.host('resume-owned-container',['docker','unpause',CONTAINER],check=False);r.adb('remove-stale-summary',['shell','run-as',PACKAGE,'rm','-f','files/reference-'+a.mode+'-summary.json']);r.adb('sample-stop',['shell','am','force-stop',PACKAGE]);r.adb('clear-owned-logcat',['logcat','-c']);r.adb('sample-start',['shell','am','start','-n',PACKAGE+'/.HelloArActivity','--es','compare_mode',a.mode,'--ei','compare_frames',str(a.frames),'--ez','compare_save_images',str(a.images).lower()]);
    if a.motion:
        r.run('physical-model-motion',['/work/toolchain/grpc-python/bin/python','/work/observer-tools/motion.py'])
    deadline=time.monotonic()+a.timeout
    summary_path='files/reference-'+a.mode+'-summary.json';summary=None
    while time.monotonic()<deadline:
        result=subprocess.run(['docker','exec',CONTAINER,'adb','-s',SERIAL,'shell','run-as',PACKAGE,'cat',summary_path],capture_output=True,text=True,timeout=8)
        if result.returncode==0:
            try:summary=json.loads(result.stdout);break
            except json.JSONDecodeError:pass
        print('observing SDK',a.mode,'remaining',round(deadline-time.monotonic()),flush=True);time.sleep(5)
    r.adb('app-logcat',['logcat','-d','-s','ARCORE_COMPARE:V','HelloArActivity:V','AndroidRuntime:E','*:S']);r.adb('sample-final-stop',['shell','am','force-stop',PACKAGE])
    if summary is None:raise RuntimeError('SDK experiment did not complete; logcat preserved')
    (r.out/'summary.json').write_text(json.dumps(summary,indent=2)+'\n')
    for kind in ['poses','imu']:
        dest=r.out/(a.mode+'-'+kind+'.jsonl');argv=['docker','exec',CONTAINER,'adb','-s',SERIAL,'exec-out','run-as',PACKAGE,'cat','files/reference-'+a.mode+'-'+kind+'.jsonl']
        with dest.open('xb')as f:result=subprocess.run(argv,stdout=f,stderr=subprocess.PIPE)
        r.records.append({'name':'export-'+kind,'argv':argv,'exit':result.returncode,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'bytes':dest.stat().st_size});assert result.returncode==0,result.stderr
    if a.images and summary.get('savedImages',0):
        dest=r.out/(a.mode+'-images.tar');argv=['docker','exec',CONTAINER,'adb','-s',SERIAL,'exec-out','run-as',PACKAGE,'tar','-C','files','-cf','-','reference-'+a.mode+'-images']
        with dest.open('xb')as f:result=subprocess.run(argv,stdout=f,stderr=subprocess.PIPE)
        r.records.append({'name':'export-images','argv':argv,'exit':result.returncode,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'bytes':dest.stat().st_size});assert result.returncode==0,result.stderr
    if a.mode=='record':
        dest=r.out/'reference-recording.mp4';argv=['docker','exec',CONTAINER,'adb','-s',SERIAL,'exec-out','run-as',PACKAGE,'cat','files/reference-recording.mp4']
        with dest.open('xb')as f:result=subprocess.run(argv,stdout=f,stderr=subprocess.PIPE)
        r.records.append({'name':'export-recording','argv':argv,'exit':result.returncode,'sha256':hashlib.sha256(dest.read_bytes()).hexdigest(),'bytes':dest.stat().st_size});assert result.returncode==0,result.stderr
    (r.out/'commands.json').write_text(json.dumps(r.records,indent=2)+'\n');print(json.dumps(summary),flush=True)
    if summary.get('setupError')is not None or summary['frames']==0:raise SystemExit(1)

def main():
    p=argparse.ArgumentParser();s=p.add_subparsers(dest='phase',required=True);b=s.add_parser('boot');b.add_argument('--attempt',required=True);e=s.add_parser('experiment');e.add_argument('--attempt',required=True);e.add_argument('--mode',choices=['live','record','playback1','playback2'],required=True);e.add_argument('--frames',type=int,default=300);e.add_argument('--images',action='store_true');e.add_argument('--motion',action='store_true');e.add_argument('--coldboot',action='store_true');e.add_argument('--timeout',type=int,default=120);a=p.parse_args()
    owner=subprocess.run(['docker','inspect','--format','{{ index .Config.Labels \"mobile-slam-reference\" }}',CONTAINER],capture_output=True,text=True)
    if owner.returncode or owner.stdout.strip()!='arcore-20261012':raise RuntimeError('Container ownership label mismatch')
    try:
        boot(a)if a.phase=='boot'else experiment(a)
    finally:
        pause=subprocess.run(['docker','pause',CONTAINER],capture_output=True,text=True)
        print('pause-owned-container',pause.returncode,flush=True)
if __name__=='__main__':main()
