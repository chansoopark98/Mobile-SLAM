#!/usr/bin/env python3
"""Drive only the owned emulator's public physical model; log actual readbacks."""
import argparse,json,math,pathlib,sys,time
HERE=pathlib.Path('/work/toolchain/grpc-generated');sys.path.insert(0,str(HERE))
import grpc
import emulator_controller_pb2 as pb
import emulator_controller_pb2_grpc as rpc

def values(stub,target):
    result=stub.getPhysicalModel(pb.PhysicalModelValue(target=target),timeout=5)
    return {'status':result.status,'values':list(result.value.data)}
def main():
    p=argparse.ArgumentParser();p.add_argument('--steps',type=int,default=24);p.add_argument('--interval',type=float,default=.5);a=p.parse_args()
    with grpc.insecure_channel('127.0.0.1:8554') as channel:
        grpc.channel_ready_future(channel).result(timeout=10)
        stub=rpc.EmulatorControllerStub(channel)
        initial={str(t):values(stub,t) for t in [0,1]}
        print(json.dumps({'event':'initial','physicalModels':initial,'monotonicNs':str(time.monotonic_ns())}),flush=True)
        if any(v['status']!=0 or len(v['values'])!=3 for v in initial.values()):raise RuntimeError('Physical-model readback unavailable')
        position=initial['0']['values'];rotation=initial['1']['values'];start=time.monotonic()
        for i in range(a.steps+1):
            target=start+i*a.interval
            if target>time.monotonic():time.sleep(target-time.monotonic())
            phase=2*math.pi*i/a.steps
            pos=[position[0]+.15*math.sin(phase),position[1],position[2]+.10*(math.cos(phase)-1)]
            rot=[rotation[0],rotation[1]+15*math.sin(phase),rotation[2]]
            for t,v in [(0,pos),(1,rot)]:stub.setPhysicalModel(pb.PhysicalModelValue(target=t,value=pb.ParameterValue(data=v),interpolation=pb.PhysicalModelValue.SMOOTH),timeout=5)
            print(json.dumps({'event':'command','index':i,'monotonicNs':str(time.monotonic_ns()),'positionRequestM':pos,'rotationRequestDeg':rot,'actualReadback':{str(t):values(stub,t) for t in [0,1]}}),flush=True)
        time.sleep(1)
        print(json.dumps({'event':'final','physicalModels':{str(t):values(stub,t) for t in [0,1]},'monotonicNs':str(time.monotonic_ns())}),flush=True)
if __name__=='__main__':main()
