#!/usr/bin/env python3
"""Record source, dirty diff, actual compile flags, dependency and artifact hashes."""
import argparse, hashlib, json, subprocess, re
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()
def opencv_version(header):
    text=header.read_text();return '.'.join(re.search(r"#define CV_VERSION_"+key+r"\s+(\d+)",text).group(1) for key in ('MAJOR','MINOR','REVISION'))

def record(build,artifact):
    files=[ROOT/'CMakeLists.txt',ROOT/'wasm/CMakeLists.txt',ROOT/'wasm/vio_bindings.cpp',ROOT/'tests/audit_dataset_replay.cpp']
    for directory in ('src','include','cmake'):
        files.extend(p for p in (ROOT/directory).rglob('*') if p.is_file())
    sources={str(p.relative_to(ROOT)):sha(p) for p in sorted(set(files))}
    commands=json.loads((build/'compile_commands.json').read_text())
    core=[entry for entry in commands if str(ROOT/'src')+'/' in entry['file']]
    dependencies={str(p.relative_to(ROOT)):sha(p) for folder in ('wasm/libs/ceres/lib','wasm/libs/opencv/lib') for p in (ROOT/folder).glob('*.a')}
    diff=subprocess.check_output(['git','diff','--binary'],cwd=ROOT)
    return {'schema':'mobile-slam-build-v1','head':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
      'dirty_diff_sha256':hashlib.sha256(diff).hexdigest(),'source_files':sources,
      'source_manifest_sha256':hashlib.sha256(json.dumps(sources,sort_keys=True).encode()).hexdigest(),
      'build_directory':str(build),'core_compile_commands':core,'cmake_cache_sha256':sha(build/'CMakeCache.txt'),
      'dependency_versions':{'native_opencv_header':opencv_version(Path('/usr/include/opencv4/opencv2/core/version.hpp')),'wasm_opencv_header':opencv_version(ROOT/'wasm/libs/opencv/include/opencv4/opencv2/core/version.hpp'),'ceres':'2.2.0','gtest':'1.14.0'},'wasm_dependencies':dependencies,'artifact':{'path':str(artifact),'sha256':sha(artifact),'bytes':artifact.stat().st_size},
      'single_file_embedded_wasm':artifact.suffix=='.js' and ('data:application/octet-stream;base64,' in artifact.read_text(errors='replace') or 'function findWasmBinary(){return binaryDecode(' in artifact.read_text(errors='replace'))}
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--build',type=Path,required=True);parser.add_argument('--artifact',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();data=record(args.build.resolve(),args.artifact.resolve());args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(data,indent=2)+'\n');print(args.output)
if __name__=='__main__':main()
