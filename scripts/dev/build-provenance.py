#!/usr/bin/env python3
"""Record source, dirty diff, actual compile flags, dependency and artifact hashes."""
import argparse, hashlib, json, subprocess, re
from pathlib import Path

def sha(path):
    digest=hashlib.sha256()
    with path.open('rb') as source:
        for chunk in iter(lambda: source.read(1024*1024),b''):digest.update(chunk)
    return digest.hexdigest()
def opencv_version(header):
    if not header.is_file():return None
    text=header.read_text();return '.'.join(re.search(r"#define CV_VERSION_"+key+r"\s+(\d+)",text).group(1) for key in ('MAJOR','MINOR','REVISION'))

def cmake_cache(build):
    return {match[1]:match[2] for line in (build/'CMakeCache.txt').read_text().splitlines()
            if (match:=re.match(r'^([^/#][^:]*):[^=]*=(.*)$',line))}

def resolve_source_root(cache,source_root=None):
    root=Path(source_root or cache['CMAKE_HOME_DIRECTORY']).resolve()
    if source_root is None and root.name=='wasm' and (root.parent/'src').is_dir():root=root.parent
    if not (root/'CMakeLists.txt').is_file():raise FileNotFoundError('Build source tree unavailable: '+str(root))
    return root

def record(build,artifact,source_root=None,native_ceres_archives=None):
    build,artifact=Path(build).resolve(),Path(artifact).resolve();cache=cmake_cache(build)
    root=resolve_source_root(cache,source_root)
    files=[root/'CMakeLists.txt',root/'wasm/CMakeLists.txt',root/'wasm/vio_bindings.cpp',root/'tests/audit_dataset_replay.cpp']
    for directory in ('src','include','cmake'):
        files.extend(p for p in (root/directory).rglob('*') if p.is_file())
    sources={str(p.relative_to(root)):sha(p) for p in sorted(set(files)) if p.is_file()}
    commands=json.loads((build/'compile_commands.json').read_text())
    def compiled_source(entry):
        path=Path(entry['file']);return (path if path.is_absolute() else Path(entry['directory'])/path).resolve()
    core=[entry for entry in commands if compiled_source(entry).is_relative_to(root/'src')]
    dependencies={str(p.relative_to(root)):sha(p) for folder in ('wasm/libs/ceres/lib','wasm/libs/opencv/lib') for p in (root/folder).glob('*.a')}
    head=diff_hash=None
    if (root/'.git').exists():
        head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=root,text=True).strip()
        diff_hash=hashlib.sha256(subprocess.check_output(['git','diff','--binary'],cwd=root)).hexdigest()
    ceres_source=Path(cache['Ceres_SOURCE_DIR']) if cache.get('Ceres_SOURCE_DIR') else None
    ceres_version=None
    if ceres_source and (header:=ceres_source/'include/ceres/version.h').is_file():
        text=header.read_text();ceres_version='.'.join(re.search(r'#define CERES_VERSION_'+key+r'\s+(\d+)',text).group(1) for key in ('MAJOR','MINOR','REVISION'))
    gtest_version=None
    if cache.get('gtest_SOURCE_DIR'):
        cmake=Path(cache['gtest_SOURCE_DIR']).parent/'CMakeLists.txt'
        if cmake.is_file() and (match:=re.search(r'set\(GOOGLETEST_VERSION\s+([\d.]+)\)',cmake.read_text())):gtest_version=match[1]
    ceres_archives={Path(p).resolve() for p in native_ceres_archives} if native_ceres_archives is not None else set(build.rglob('libceres.a'))
    if native_ceres_archives is None and cache.get('Ceres_BINARY_DIR'):ceres_archives.update(Path(cache['Ceres_BINARY_DIR']).rglob('libceres.a'))
    return {'schema':'mobile-slam-build-v1','head':head,
      'source_root':str(root),'source_root_resolution':'explicit' if source_root is not None else 'cmake_cache',
      'cmake_source_directory':cache.get('CMAKE_HOME_DIRECTORY'),'git_context_available':head is not None,
      'dirty_diff_sha256':diff_hash,'source_files':sources,
      'source_manifest_sha256':hashlib.sha256(json.dumps(sources,sort_keys=True).encode()).hexdigest(),
      'build_directory':str(build),'core_compile_commands':core,'cmake_cache_sha256':sha(build/'CMakeCache.txt'),
      'dependency_versions':{'native_opencv_header':opencv_version(Path('/usr/include/opencv4/opencv2/core/version.hpp')),'wasm_opencv_header':opencv_version(root/'wasm/libs/opencv/include/opencv4/opencv2/core/version.hpp'),'ceres':ceres_version,'gtest':gtest_version},
      'nativeCeresArchives':[{'path':str(p.resolve()),'sha256':sha(p),'bytes':p.stat().st_size} for p in sorted(ceres_archives)],
      'native_ceres_archive_resolution':'explicit' if native_ceres_archives is not None else 'cmake_cache_or_build',
      'dependency_context':{'wasm_source_root':str(root),'native_ceres_source':str(ceres_source) if ceres_source else None},
      'wasm_dependencies':dependencies,'artifact':{'path':str(artifact),'sha256':sha(artifact),'bytes':artifact.stat().st_size},
      'single_file_embedded_wasm':artifact.suffix=='.js' and ('data:application/octet-stream;base64,' in artifact.read_text(errors='replace') or 'function findWasmBinary(){return binaryDecode(' in artifact.read_text(errors='replace'))}
def main():
    parser=argparse.ArgumentParser();parser.add_argument('--build',type=Path,required=True);parser.add_argument('--artifact',type=Path,required=True);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--source-root',type=Path)
    parser.add_argument('--native-ceres-archive',type=Path,action='append',help='Actual imported archive when CMake builds no local Ceres target')
    args=parser.parse_args();data=record(args.build.resolve(),args.artifact.resolve(),args.source_root,args.native_ceres_archive);args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(data,indent=2)+'\n');print(args.output)
if __name__=='__main__':main()
