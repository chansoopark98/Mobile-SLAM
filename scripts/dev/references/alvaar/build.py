#!/usr/bin/env python3
"""Build pinned AlvaAR and bundled required dependencies, serialized with other reference builds."""
import argparse
import datetime
import difflib
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]
SDK = Path(os.environ.get('EMSDK', str(Path.home() / 'emsdk'))) / 'upstream/emscripten'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prepared', required=True, type=Path)
    args = parser.parse_args()
    run = args.prepared.resolve()
    manifest = json.loads((run/'source-manifest.json').read_text())
    source = Path(manifest['source'])
    attempt = run/('build-'+datetime.datetime.now(datetime.timezone.utc).strftime('%Y%m%dT%H%M%SZ'))
    attempt.mkdir(exist_ok=False)
    lock = (ROOT/'build/reference-env-build.lock').open('a')
    print('Waiting for exclusive reference build lock',flush=True)
    fcntl.flock(lock,fcntl.LOCK_EX)
    print('Reference build lock acquired; compiler jobs=2',flush=True)
    env = dict(os.environ)
    env.update({'EM_CACHE':str(run/'emscripten-cache'),'EM_PORTS':str(run/'emscripten-ports'),
        'EMCC_CORES':'2','BINARYEN_CORES':'2','CMAKE_BUILD_PARALLEL_LEVEL':'2','MAKEFLAGS':'-j2'})
    if not (run/'emscripten-cache').exists():
        shutil.copytree(SDK/'cache',run/'emscripten-cache',symlinks=True)
    deps, build = run/'deps', run/'build'
    deps.mkdir(exist_ok=True);build.mkdir(exist_ok=True)
    link = source/'src/libs/build'
    if not link.exists():link.symlink_to(deps,target_is_directory=True)
    elif link.resolve()!=deps:raise ValueError('Unexpected dependency prefix in isolated source')
    changes = []
    for relative, replacements in {
        'src/libs/eigen/CMakeLists.txt': [
            ('add_subdirectory(scripts EXCLUDE_FROM_ALL)','if(BUILD_TESTING)\nadd_subdirectory(scripts EXCLUDE_FROM_ALL)\nendif()')],
        'src/libs/opengv/CMakeLists.txt': [
            ('    add_definitions (-march=native)', '    if(NOT EMSCRIPTEN)\n    add_definitions (-march=native)'),
            (' # for now I even removed std=gnu++0x\n', ' # for now I even removed std=gnu++0x\n    endif()\n')],
        'src/slam/CMakeLists.txt': [
            ('include_directories(${LIBS_BUILD_FOLDER}/opencv/)','include_directories(${LIBS_BUILD_FOLDER}/opencv/)\ninclude_directories(${LIBS_BUILD_FOLDER}/opencv/include/opencv4)\ninclude_directories(${LIBS_BUILD_FOLDER}/ceres-solver/include/ceres/internal/miniglog)'),
            ('--extern-post-js ../../system.js','--extern-post-js ${CMAKE_CURRENT_SOURCE_DIR}/../system.js')],
    }.items():
        target = source/relative
        old = target.read_text();new = old
        for before, after in replacements:
            if after in new:continue
            if before not in new:raise ValueError(f'Build portability patch no longer matches {relative}')
            new = new.replace(before,after)
        if new!=old:
            target.write_text(new)
            changes.extend(difflib.unified_diff(old.splitlines(True),new.splitlines(True),fromfile=relative,tofile=relative))
    (attempt/'build-portability.patch').write_text(''.join(changes))
    events = []
    def execute(name, command):
        print('START '+name,flush=True)
        event={'name':name,'command':command,'startedAt':datetime.datetime.now(datetime.timezone.utc).isoformat()}
        events.append(event)
        with (attempt/(name+'.log')).open('x') as log:
            event['exitCode']=subprocess.run(command,env=env,stdout=log,stderr=subprocess.STDOUT).returncode
        event['finishedAt']=datetime.datetime.now(datetime.timezone.utc).isoformat()
        (attempt/'commands.json').write_text(json.dumps(events,indent=2)+'\n')
        print('FINISH '+name+' exit='+str(event['exitCode']),flush=True)
        if event['exitCode']:raise subprocess.CalledProcessError(event['exitCode'],command)
        fcntl.flock(lock,fcntl.LOCK_UN)
        time.sleep(.05) # Let queued reference SDK probes run at component boundaries.
        fcntl.flock(lock,fcntl.LOCK_EX)
    common=['-G','Ninja','-DCMAKE_BUILD_TYPE=Release','-DCMAKE_CXX_STANDARD=17',
        '-DCMAKE_EXPORT_COMPILE_COMMANDS=ON','-DCMAKE_TOOLCHAIN_FILE='+str(SDK/'cmake/Modules/Platform/Emscripten.cmake'),
        '-DCMAKE_C_FLAGS=-O3 -msimd128','-DCMAKE_CXX_FLAGS=-O3 -msimd128','-DBUILD_SHARED_LIBS=OFF']
    libraries=[
        ('eigen','eigen',['-DBUILD_TESTING=OFF','-DEIGEN_BUILD_DOC=OFF']),
        ('opencv','opencv',['-DBUILD_LIST=core,imgproc,calib3d,features2d,flann,video,objdetect',
            '-DBUILD_TESTS=OFF','-DBUILD_PERF_TESTS=OFF','-DBUILD_EXAMPLES=OFF','-DBUILD_opencv_js=OFF',
            '-DBUILD_opencv_python2=OFF','-DBUILD_opencv_python3=OFF','-DBUILD_opencv_java=OFF','-DBUILD_opencv_apps=OFF',
            '-DWITH_IPP=OFF','-DWITH_OPENCL=OFF','-DWITH_TBB=OFF','-DWITH_PTHREADS_PF=OFF','-DWITH_ITT=OFF',
            '-DWITH_JPEG=OFF','-DWITH_PNG=OFF','-DWITH_TIFF=OFF','-DWITH_WEBP=OFF','-DWITH_OPENEXR=OFF']),
        ('Sophus','Sophus',['-DBUILD_TESTS=OFF','-DEIGEN3_INCLUDE_DIR='+str(deps/'eigen/include/eigen3')]),
        ('ceres-solver','ceres-solver',['-DBUILD_TESTING=OFF','-DBUILD_EXAMPLES=OFF','-DMINIGLOG=ON','-DGFLAGS=OFF',
            '-DSUITESPARSE=OFF','-DCXSPARSE=OFF','-DLAPACK=OFF','-DEIGENSPARSE=ON','-DCERES_THREADING_MODEL=NO_THREADS',
            '-DEigen3_DIR='+str(deps/'eigen/share/eigen3/cmake')]),
        ('opengv','opengv',['-DBUILD_TESTS=OFF','-DBUILD_PYTHON=OFF','-DEIGEN_INCLUDE_DIR='+str(deps/'eigen/include/eigen3')]),
    ]
    try:
        execute('toolchain',['bash','-c','emcc --version; cmake --version; node --version'])
        for name, directory, flags in libraries:
            execute(name+'-configure',['cmake','-S',str(source/'src/libs'/directory),'-B',str(build/name),*common,
                '-DCMAKE_INSTALL_PREFIX='+str(deps/name),*flags])
            execute(name+'-build',['cmake','--build',str(build/name),'--parallel','2'])
            execute(name+'-install',['cmake','--install',str(build/name)])
        required=[deps/'opencv/lib'/('libopencv_'+name+'.a')for name in ['core','imgproc','calib3d','features2d','flann','video','objdetect']]
        required += [deps/'ceres-solver/lib/libceres.a',deps/'opengv/lib/libopengv.a']
        if not all(path.is_file() for path in required):raise ValueError('Required reference static libraries missing')
        execute('alvaar-configure',['cmake','-S',str(source/'src/slam'),'-B',str(build/'slam'),*common,'-DDIST_DIR='+str(run/'dist')])
        execute('alvaar-build',['cmake','--build',str(build/'slam'),'--parallel','2'])
        artifact = build/'slam/alva_ar.js'
        if not artifact.is_file():raise ValueError('New source-build loader missing')
        (run/'dist').mkdir(exist_ok=True)
        shutil.copy2(artifact,run/'dist/alva_ar.js')
        fingerprints=[]
        for path in [*required,artifact,*[build/name/'CMakeCache.txt'for name,_,_ in libraries],build/'slam/CMakeCache.txt']:
            fingerprints.append({'path':str(path),'bytes':path.stat().st_size,'sha256':hashlib.file_digest(path.open('rb'),'sha256').hexdigest()})
        result={'completed':True,'sourceManifest':str(run/'source-manifest.json'),'sourceManifestSha256':hashlib.sha256((run/'source-manifest.json').read_bytes()).hexdigest(),
            'algorithmEdits':False,'buildPortabilityPatch':str(attempt/'build-portability.patch'),
            'allPortabilityPatches':[{'path':str(path),'sha256':hashlib.sha256(path.read_bytes()).hexdigest()}
                for path in sorted(run.glob('build-*/build-portability.patch'))],
            'artifact':str(run/'dist/alva_ar.js'),'artifactSha256':hashlib.sha256(artifact.read_bytes()).hexdigest(),
            'jobs':2,'privateEmscriptenCache':str(run/'emscripten-cache'),'fingerprints':fingerprints}
        result['upstreamBuildMode']='SIMD; matching upstream BUILD_TYPE=SIMD feature flags'
        result['boundedBuildEnvironment']={name:env[name]for name in ['EM_CACHE','EM_PORTS','EMCC_CORES','BINARYEN_CORES','CMAKE_BUILD_PARALLEL_LEVEL','MAKEFLAGS']}
        (attempt/'result.json').write_text(json.dumps(result,indent=2)+'\n');(run/'latest-build.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result,indent=2),flush=True)
    except Exception as error:
        (attempt/'result.json').write_text(json.dumps({'completed':False,'error':str(error),'commands':events},indent=2)+'\n')
        print('BUILD FAILED; preserved at '+str(attempt),flush=True)
        raise


if __name__=='__main__':main()
