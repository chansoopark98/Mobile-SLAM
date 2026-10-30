"""Exercise dependency acquisition/build contracts with command mocks, without network or compilation."""
import json
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
SCRIPTS = ROOT / 'wasm'
PINS = {'opencv': 'dad26339a975b49cfb6c7dbe4bd5276c9dcb36e2',
        'ceres': '85331393dc0dff09f6fb9903ab0c4bfa3e134b01'}
VERSIONS = {'opencv': '4.5.5', 'ceres': '2.2.0'}

MOCK = r'''#!/usr/bin/env python3
import json, os, pathlib, sys
p = pathlib.Path(os.environ['COMMAND_LOG'])
with p.open('a') as f: f.write(json.dumps([pathlib.Path(sys.argv[0]).name] + sys.argv[1:]) + '\n')
args = sys.argv[1:]
kind = os.environ['DEPENDENCY_KIND']
if pathlib.Path(sys.argv[0]).name == 'git':
    if args[0] == 'clone':
        dest = pathlib.Path(args[-1]); dest.mkdir(parents=True)
        (dest/'CMakeLists.txt').write_text('# source fixture\n')
        (dest/'.git').mkdir()
        header = dest / ('modules/core/include/opencv2/core/version.hpp' if kind == 'opencv' else 'include/ceres/version.h')
        header.parent.mkdir(parents=True, exist_ok=True)
        macros = ['CV_VERSION_MAJOR', 'CV_VERSION_MINOR', 'CV_VERSION_REVISION'] if kind == 'opencv' else ['CERES_VERSION_MAJOR', 'CERES_VERSION_MINOR', 'CERES_VERSION_REVISION']
        header.write_text(''.join('#define %s %s\n' % pair for pair in zip(macros, os.environ['DEPENDENCY_VERSION'].split('.'))))
    elif args[2:4] == ['diff', '--quiet']:
        sys.exit(int(os.environ.get('DEPENDENCY_DIRTY', '0')))
    else: print(os.environ['DEPENDENCY_HEAD'])
if pathlib.Path(sys.argv[0]).name == 'emcmake':
    prefix = next(a.split('=',1)[1] for a in args if a.startswith('-DCMAKE_INSTALL_PREFIX='))
    pathlib.Path(prefix, 'include').mkdir(parents=True, exist_ok=True)
'''


class DependencyBuildTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.root = Path(self.tmp.name)
        (self.root/'wasm').mkdir()
        for kind in PINS:
            shutil.copyfile(SCRIPTS/f'build_{kind}.sh', self.root/'wasm'/f'build_{kind}.sh')
        sdk = self.root/'sdk/upstream/emscripten/cmake/Modules/Platform'
        sdk.mkdir(parents=True)
        (sdk/'Emscripten.cmake').write_text('# mock SDK\n')
        self.bin = self.root/'bin'; self.bin.mkdir()
        for name in ('git', 'emcmake', 'emmake'):
            p = self.bin/name; p.write_text(MOCK); p.chmod(0o755)
        self.log = self.root/'calls.jsonl'

    def tearDown(self):
        self.tmp.cleanup()

    def source(self, kind, *, cache=False, version=None, complete=True):
        name = 'opencv-4.5.5-src' if kind == 'opencv' else 'ceres-solver-src'
        p = self.root/('build/_deps/'+name if cache else 'provided-'+kind)
        p.mkdir(parents=True, exist_ok=True)
        (p/'sentinel').write_text('preserve owner data')
        if complete:
            (p/'CMakeLists.txt').write_text('# fixture\n')
            header = p/('modules/core/include/opencv2/core/version.hpp' if kind == 'opencv' else 'include/ceres/version.h')
            header.parent.mkdir(parents=True, exist_ok=True)
            macros = ['CV_VERSION_MAJOR','CV_VERSION_MINOR','CV_VERSION_REVISION'] if kind == 'opencv' else ['CERES_VERSION_MAJOR','CERES_VERSION_MINOR','CERES_VERSION_REVISION']
            header.write_text(''.join('#define %s %s\n' % pair for pair in zip(macros,(version or VERSIONS[kind]).split('.'))))
            if cache: (p/'.git').mkdir(exist_ok=True)
        return p

    def run_build(self, kind, *, source=None, jobs=None, head=None, dirty=False):
        self.log.unlink(missing_ok=True)
        env = dict(os.environ, PATH=str(self.bin)+os.pathsep+os.environ['PATH'],
                   EMSDK=str(self.root/'sdk'), COMMAND_LOG=str(self.log),
                   DEPENDENCY_KIND=kind, DEPENDENCY_VERSION=VERSIONS[kind],
                   DEPENDENCY_HEAD=head or PINS[kind], DEPENDENCY_DIRTY='1' if dirty else '0')
        for key in ('OPENCV_SOURCE_DIR','CERES_SOURCE_DIR','MOBILE_SLAM_JOBS'): env.pop(key,None)
        if source is not None: env[kind.upper()+'_SOURCE_DIR']=str(source)
        if jobs is not None: env['MOBILE_SLAM_JOBS']=jobs
        r = subprocess.run(['bash',str(self.root/'wasm'/f'build_{kind}.sh')],env=env,capture_output=True,text=True,cwd=self.root)
        calls = [json.loads(s) for s in self.log.read_text().splitlines()] if self.log.exists() else []
        return r,calls

    def test_missing_default_cache_acquires_official_tag_and_builds_with_two_jobs(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                r,calls=self.run_build(kind)
                self.assertEqual(r.returncode,0,r.stderr)
                clone=next(c for c in calls if c[:2]==['git','clone'])
                self.assertIn(VERSIONS[kind],clone)
                self.assertIn('https://github.com/'+('opencv/opencv.git' if kind=='opencv' else 'ceres-solver/ceres-solver.git'),clone)
                self.assertIn(['emmake','make','-j2'],calls)
                self.assertIn(['emmake','make','install'],calls)

    def test_explicit_complete_source_is_used_without_git_or_source_mutation(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                p=self.source(kind); before={str(x.relative_to(p)):x.read_bytes() for x in p.rglob('*') if x.is_file()}
                r,calls=self.run_build(kind,source=p)
                self.assertEqual(r.returncode,0,r.stderr)
                self.assertFalse(any(c[0]=='git' for c in calls))
                self.assertEqual(next(c[2] for c in calls if c[0]=='emcmake'),str(p))
                self.assertEqual(before,{str(x.relative_to(p)):x.read_bytes() for x in p.rglob('*') if x.is_file()})

    def test_partial_default_cache_is_preserved_and_not_fetched_over(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                p=self.source(kind,cache=True,complete=False)
                r,calls=self.run_build(kind)
                self.assertNotEqual(r.returncode,0)
                self.assertIn('Incomplete',r.stderr)
                self.assertEqual((p/'sentinel').read_text(),'preserve owner data')
                self.assertEqual(calls,[])

    def test_partial_explicit_source_fails_without_download(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                p=self.source(kind,complete=False);r,calls=self.run_build(kind,source=p)
                self.assertNotEqual(r.returncode,0)
                self.assertIn(kind.upper()+'_SOURCE_DIR',r.stderr)
                self.assertEqual(calls,[])

    def test_wrong_source_version_is_rejected_before_build(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                p=self.source(kind,version='9.9.9');r,calls=self.run_build(kind,source=p)
                self.assertNotEqual(r.returncode,0)
                self.assertIn('Expected',r.stderr)
                self.assertEqual(calls,[])

    def test_wrong_default_commit_is_rejected_without_fetching(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                self.source(kind,cache=True);r,calls=self.run_build(kind,head='0'*40)
                self.assertNotEqual(r.returncode,0)
                self.assertIn('pinned commit',r.stderr)
                self.assertFalse(any(c[:2]==['git','clone'] or c[0] in ('emmake','emcmake') for c in calls))

    def test_existing_pinned_cache_is_reused(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                p=self.source(kind,cache=True);r,calls=self.run_build(kind)
                self.assertEqual(r.returncode,0,r.stderr)
                self.assertFalse(any(c[:2]==['git','clone'] for c in calls))
                self.assertEqual((p/'sentinel').read_text(),'preserve owner data')

    def test_parallelism_must_be_one_to_four(self):
        for kind in PINS:
            for jobs in ('0','5','-1','2 2','all'):
                with self.subTest(kind=kind,jobs=jobs):
                    r,calls=self.run_build(kind,jobs=jobs)
                    self.assertNotEqual(r.returncode,0)
                    self.assertIn('MOBILE_SLAM_JOBS',r.stderr)
                    self.assertEqual(calls,[])

    def test_relative_explicit_source_is_resolved_before_changing_build_directory(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                p=self.source(kind);r,calls=self.run_build(kind,source=p.relative_to(self.root))
                self.assertEqual(r.returncode,0,r.stderr)
                self.assertEqual(next(c[2] for c in calls if c[0]=='emcmake'),str(p))

    def test_modified_default_cache_is_rejected_without_overwrite(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                p=self.source(kind,cache=True);r,calls=self.run_build(kind,dirty=True)
                self.assertNotEqual(r.returncode,0)
                self.assertIn('pinned commit',r.stderr)
                self.assertEqual((p/'sentinel').read_text(),'preserve owner data')
                self.assertFalse(any(c[0] in ('emmake','emcmake') for c in calls))

    def test_explicit_four_jobs_are_passed_to_build(self):
        for kind in PINS:
            with self.subTest(kind=kind):
                r,calls=self.run_build(kind,source=self.source(kind),jobs='4')
                self.assertEqual(r.returncode,0,r.stderr)
                self.assertIn(['emmake','make','-j4'],calls)


if __name__=='__main__': unittest.main()
