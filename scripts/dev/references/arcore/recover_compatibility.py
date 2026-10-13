#!/usr/bin/env python3
"""Preserve failed API35 evidence; prepare the official API30 virtual image."""
import json,subprocess
from environment import ROOT,REFERENCE,EVIDENCE,CONTAINER,command
try:
    if command('compat-unpause',['docker','unpause',CONTAINER]):raise SystemExit(1)
    command('compat-stop-owned-emulator',['docker','exec',CONTAINER,'adb','-s','emulator-5556','emu','kill'])
    if command('sdk-api30-install',['docker','exec','-i',CONTAINER,'sdkmanager','--sdk_root=/work/toolchain/sdk','system-images;android-30;google_apis;x86_64'],input='y\n'*1000):raise SystemExit(1)
    if command('grpc-venv',['docker','exec',CONTAINER,'python3','-m','venv','/work/toolchain/grpc-python']):raise SystemExit(1)
    if command('grpc-install',['docker','exec',CONTAINER,'/work/toolchain/grpc-python/bin/pip','install','grpcio==1.78.0','grpcio-tools==1.78.0','protobuf==6.33.5']):raise SystemExit(1)
    command('grpc-installed',['docker','exec',CONTAINER,'/work/toolchain/grpc-python/bin/pip','freeze'])
    if command('helloar-build-v3',['docker','exec','-e','JAVA_TOOL_OPTIONS=-XX:ActiveProcessorCount=2 -Xmx2048m',CONTAINER,'bash','-c','cd /work/source/samples/hello_ar_java && ./gradlew --no-daemon --max-workers=2 assembleDebug']):raise SystemExit(1)
finally:
    pause=subprocess.run(['docker','pause',CONTAINER],capture_output=True,text=True);print('paused-owned-container',pause.returncode,flush=True)
