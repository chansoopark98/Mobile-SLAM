#!/usr/bin/env python3
"""Verify source provenance, local vendor hashes and golden fixtures.

Always: every file under xr_source, xr_shim, libcxx_compat and musl_libm must match its
MANIFEST.tsv. --data also checks every tracked golden fixture and its recorded size.

--upstream checks original proof-base hashes before applying patches and the deterministic
public_source_bytes naming/comment pass. upstream_sha256 therefore describes original inputs;
vendored_sha256 describes the local public tree. The historical worktree policy remains the
default: edited reference files are hashed as they are, without stripping. --upstream-base commit
reads pinned git objects and applies only the historical copyright-line policy before that hash.
Neither mode accepts arbitrary source changes. Source repository and original author attribution
are centralized in the root README's external-code section.

The generated analytic residual is regenerated from the proof base. Local device-table and
residual generator inputs are also vendored as source-input files; they are not compiled TUs.
Shim probe comment changes reconstruct and verify their pinned historical input hashes. --write-manifests
is an explicit maintenance operation: review its diff, including upstream source identities.

Exit status: 0 verified; 1 failed check; 2 usage/environment error.
"""
import argparse
import hashlib
import io
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile

ENGINE = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
REPO = os.path.abspath(os.path.join(ENGINE, '..', '..'))
TP = os.path.join(ENGINE, 'third_party')

UPSTREAM_8W = os.path.join(REPO, 'assets/references/xr_reference/official/xr_reference')
sys.path.insert(0, os.path.join(REPO, 'scripts', 'dev'))
from reference_metadata import load_reference_metadata
REFERENCE_METADATA = load_reference_metadata(REPO)
REPO_8W = REFERENCE_METADATA['repository_url']
SOURCE_DISCLOSURE = '../../../../README.md#external-code'
COMMIT_8W = '4edfa484936a2cc691366eeec5c6e58bb6d4c8bd'
PATCH_ORDER = [
    'gcc-frame-point-comparator.patch',
    'cxx17-constexpr-uninit.patch',
    'cxx17-set-contains.patch',
    'cxx17-erase-if.patch',
    'libcxx-compat.patch',
    'gcc-isnan-global.patch',  # US-011: image-targets-rigid-assembly.cc unqualified isnan
    'face-ear-closure.patch',  # US-018/019: include lines only (map.h, intrinsics-clip.h shim)
    # US-015 (contract FULL-12): the closed world tracker member and call site of the released binary's
    # ImageTrackingState (xr-slam.wasm func938/994); nullptr hook (the default) = public behavior.
    'us015-image-world-tracker.patch',
]
# Patches that the docs describe as "include lines only" (PROVENANCE.md): every added or removed line
# of their hunks must be an #include.
INCLUDE_ONLY_PATCHES = ('gcc-isnan-global.patch', 'face-ear-closure.patch')
# Project policy (commit 5b8f6f5): the vendored copies do not carry the reference runtime / Niantic copyright
# line ("//" in C++ files, "#" in the generator script). It is on line 1 of 170 files and on line 2
# of 2 files of the closure: 172 of 192 upstream files, which the output below counts as 171 of the
# 191 vendored files (the generator script is the 172nd and is not a vendored file); the line of
# third_party/murmurhash/constexpr_murmur3.h ("Tamás Szelei") is not an reference runtime line and stays.
COPYRIGHT_LINE_8W = re.compile(r'^(//|#) Copyright \(c\) 20\d\d\b.*(?:' +
    '|'.join(re.escape(owner) for owner in REFERENCE_METADATA['copyright_owners']) + ')')
GENERATED_8W = {
    'reality/engine/geometry/analytical-residual/pose-estimation.cc':
        'reality/engine/geometry/analytical-residual/generate-pose-estimation.py',
}
SOURCE_INPUTS = {
    'c8/camera/device-infos.h', 'c8/camera/device-infos.cc', 'c8/geometry/intrinsics.cc',
    'reality/engine/geometry/analytical-residual/generate-pose-estimation.py',
    'reality/engine/geometry/analytical-residual/rotation.py',
    'reality/engine/geometry/analytical-residual/memoizer.py',
}
# Vendored but excluded from the host build (third_party/CMakeLists.txt, review NI-1).
WASM_ONLY_8W = {'c8/time/now.cc': 'tu-wasm-only'}
# The 52 translation units of build_probe.sh (C8_SRCS + FEAT_SRCS + GEO_SRCS) are role "tu".
TU_8W = set('''
c8/hmatrix.cc c8/hpoint.cc c8/hvector.cc c8/quaternion.cc c8/c8-log.cc c8/exceptions.cc
c8/string/format.cc c8/task-queue.cc c8/thread-pool.cc c8/time/now.cc
c8/geometry/egomotion.cc c8/geometry/homography.cc c8/geometry/parameterized-geometry.cc
c8/geometry/vectors.cc c8/geometry/device-pose.cc c8/geometry/two-d.cc c8/geometry/line.cc
c8/geometry/interpolation.cc c8/pixels/gr8-pyramid.cc c8/pixels/pixel-transforms.cc c8/pixels/image-roi.cc
reality/engine/binning/linear-bin.cc
reality/engine/features/gr8gl.cc reality/engine/features/image-point.cc
reality/engine/features/image-descriptor.cc reality/engine/features/frame-point.cc
reality/engine/features/keypoint-queue.cc reality/engine/features/feature-scales.cc
reality/engine/features/feature-manager.cc reality/engine/features/detection-config.cc
reality/engine/features/descriptor-lsh-index.cc reality/engine/features/descriptor-bit-counter.cc
reality/engine/features/global-matcher.cc reality/engine/features/local-matcher.cc
reality/engine/features/point-descriptor.cc reality/engine/features/point-matches.cc
reality/engine/features/gr8cpu-interface.cc reality/engine/features/gr8cpu-nop.cc
reality/engine/tracking/ray-point-filter.cc reality/engine/tracking/rolling-lin-reg.cc
reality/engine/geometry/bundle.cc reality/engine/geometry/bundle-residual.cc
reality/engine/geometry/bundle-residual-analytic.cc reality/engine/geometry/observed-point.cc
reality/engine/geometry/epipolar.cc reality/engine/geometry/pose-pnp.cc
reality/engine/geometry/ap3p.cc reality/engine/geometry/p2p.cc
reality/engine/geometry/reprojection.cc reality/engine/geometry/homography-decomp.cc
reality/engine/geometry/point-regression.cc
reality/engine/geometry/analytical-residual/pose-estimation.cc
'''.split())
# Image target core (US-011): translation units of sr_vendor_imagedetection
# (src/image_target/CMakeLists.txt); the loader and request executor are replaced, not vendored.
TU_8W |= set('''
reality/engine/imagedetection/tracked-image.cc reality/engine/imagedetection/target-point.cc
reality/engine/imagedetection/detection-image-matcher.cc
reality/engine/imagedetection/detection-image-local-matcher.cc
reality/engine/imagedetection/detection-image.cc reality/engine/imagedetection/image-tracking-state.cc
reality/engine/imagedetection/detection-image-tracker.cc reality/engine/imagedetection/location.cc
reality/engine/imagedetection/local-points-filter.cc
reality/engine/imagedetection/image-targets-rigid-assembly.cc
reality/engine/geometry/image-warp.cc
'''.split())
# Face and ear (US-018/US-019): translation units of sr_vendor_face (src/face/CMakeLists.txt); the GL
# ROI renderers, face-messages (capnp) and the JS glue facecontroller.cc are replaced, not vendored.
TU_8W |= set('''
c8/geometry/box2.cc c8/geometry/face-types.cc c8/geometry/face-augmentation.cc c8/geometry/mesh.cc
c8/geometry/mesh-types.cc reality/engine/deepnets/detection-anchor-nms.cc
reality/engine/faces/face-geometry.cc reality/engine/faces/tracked-face-state.cc
reality/engine/faces/face-detector-global.cc reality/engine/faces/face-detector-local.cc
reality/engine/faces/face-tracker.cc reality/engine/ears/ear-types.cc
reality/engine/ears/ear-detector-local.cc c8/color-maps.cc c8/pixels/packed.cc
'''.split())
# Lighting (US-005, pkg D 2026-10-22): lighting-estimator.cc is a translation unit of sr_vendor_c8
# (third_party/CMakeLists.txt); the request executor (capnp) is replaced by src/engine/lighting.cc.
TU_8W.add('reality/engine/lighting/lighting-estimator.cc')
# Sky (US-020): translation units of sr_vendor_sky (src/sky/CMakeLists.txt); the GL cubemap renderer
# and its shaders, the capnp messages and the JS glue (layers-controller, semantics-worker) are
# replaced by src/sky, not vendored.
TU_8W |= set('''
reality/engine/semantics/semantics-classifier.cc reality/engine/deepnets/tflite-multiclassifier.cc
'''.split())

SHIM_SRC = {
    'c8/stats/scope-timer.h': ('slam_refact/tools/re/public_build/shim/c8/stats/scope-timer.h', '-', 'header'),
    'c8/geometry/intrinsics.h': ('slam_refact/tools/re/public_build/shim/c8/geometry/intrinsics.h', '-', 'header'),
    'parameter-data.cc': ('slam_refact/tools/re/public_build/shim_src/parameter-data-shim.cc',
                          'adapted:throw->C8_THROW', 'tu'),
    # US-011: plain-struct CurvyGeometry of the schema (reality.capnp:393-406); no lane-B source,
    # the upstream column pins the schema file of the reference checkout.
    'reality/engine/api/reality.capnp.h': ('assets/references/xr_reference/official/xr_reference/reality/engine/api/reality.capnp',
                                           'new:plain-struct CurvyGeometry (schema :393-406)', 'header'),
    # US-018/US-019 (face, ear): new shims; the upstream column pins the replaced original.
    'reality/engine/api/face.capnp.h': ('assets/references/xr_reference/official/xr_reference/reality/engine/api/face.capnp',
                                        'new:AttachmentName enum (:97-134) + DebugOptions::Reader (mesh.capnp:21-25)',
                                        'header'),
    'reality/engine/deepnets/tflite-interpreter.h': (
        'assets/references/xr_reference/official/xr_reference/reality/engine/deepnets/tflite-interpreter.h',
        'new:same call surface on slam_refact::ml (no TFLite/XNNPACK)', 'header'),
    'tflite-interpreter.cc': ('assets/references/xr_reference/official/xr_reference/reality/engine/deepnets/tflite-interpreter.cc',
                              'new:slam_refact::ml backend', 'tu'),
    'c8/io/capnp-messages.h': ('assets/references/xr_reference/official/xr_reference/c8/io/capnp-messages.h',
                               'new:default ConstRootMessage<T>::reader() only', 'header'),
    'c8/pixels/render/object8.h': ('assets/references/xr_reference/official/xr_reference/c8/pixels/render/object8.h',
                                   'new:subset CAM_PROJECTION_{NEAR,FAR}_CLIP (:21-22)', 'header'),
    'c8/geometry/intrinsics-clip.h': ('assets/references/xr_reference/official/xr_reference/c8/geometry/intrinsics.cc',
                                      'new:subset class Intrinsics (:129-193,:245-263)', 'header'),
    # US-020 (sky): the original header declares rotateSquareTensorFP16 / rotateModel (third_party/half);
    # tflite-multiclassifier.cc only includes it, so the shim is empty.
    'reality/engine/deepnets/tflite-model-operations.h': (
        'assets/references/xr_reference/official/xr_reference/reality/engine/deepnets/tflite-model-operations.h',
        'new:empty (declarations unused by tflite-multiclassifier.cc)', 'header'),
}

EMSDK = os.environ.get('EMSDK', os.path.expanduser('~/emsdk'))
LIBCXX_ROOT = os.path.join(EMSDK, 'upstream/emscripten')
LIBCXX_REPO = 'emsdk 5.0.0 libc++ (llvm-project 21.1.8, emscripten-libs-21)'
LIBCXX_COMMIT = 'emscripten a7c5deabd7c88ba1c38ebe988112256775f944c6'
LLVM14_ROOT = os.environ.get('LLVM14', '/usr/lib/llvm-14')
LLVM14_REPO = 'libc++ 14.0.0 (Ubuntu libc++-14-dev 1:14.0.0-1ubuntu1.1)'
LIBCXX_MANUAL = {
    'include/libcxx_compat/__algorithm/sort.h':
        'transform+manual:extern-template block -> algorithm.cpp __sort; public sort -> llvm14::sort',
    'include/libcxx_compat/__algorithm/shuffle.h':
        'transform+manual:drop debug randomizer/random_shuffle; distribution int32_t (D:§5.2)',
    'include/libcxx_compat/__random/uniform_int_distribution.h': 'transform+manual:drop iostream operators',
    'include/libcxx_compat/__random/uniform_real_distribution.h': 'transform+manual:drop iostream operators',
    'include/libcxx_compat/__random/normal_distribution.h': 'transform+manual:drop iostream operators',
}
LIBCXX_NEW = {
    'include/libcxx_compat/__config.h': ('system/lib/libcxx/include (composite, see file header)', 'new'),
    'include/libcxx_compat/algorithm.h': ('-', 'new'),
    'include/libcxx_compat/random.h': ('-', 'new'),
    'include/libcxx_compat/queue.h': ('system/lib/libcxx/include/queue', 'excerpt-manual'),
    'LICENSE.TXT': ('system/lib/libcxx/LICENSE.TXT', 'license'),
    # libc++ 14 sort of the original binary (precision lane 2026-10-16): generated from the libc++-14-dev
    # headers by the script named in the file header; the upstream hash pins __algorithm/sort.h.
    'include/libcxx_compat/__algorithm/sort_llvm14.h':
        (os.path.join(LLVM14_ROOT, 'include/c++/v1/__algorithm/sort.h'), 'excerpt-llvm14'),
}

# musl libm subset (host builds only, third_party/musl_libm/PROVENANCE.md): emsdk 5.0.0 musl 1.2.5.
MUSL_ROOT = os.path.join(LIBCXX_ROOT, 'system/lib/libc/musl')
MUSL_REPO = 'emsdk 5.0.0 musl 1.2.5 (system/lib/libc/musl)'
MUSL_SRC = {  # vendored path -> (upstream path relative to MUSL_ROOT, patches, role)
    'COPYRIGHT': ('COPYRIGHT', 'verbatim', 'license'),
    'include/libm.h': ('src/internal/libm.h', 'manual:subset (see file header)', 'excerpt-manual'),
    'include/features.h': ('src/include/features.h', 'manual:subset (see file header)', 'excerpt-manual'),
}
for _f in ('sinf.c', 'cosf.c', 'sincosf.c', '__sindf.c', '__cosdf.c', '__rem_pio2f.c', '__rem_pio2_large.c',
           'atan2f.c', 'atanf.c', 'log10f.c',
           'acosf.c', 'logf.c', 'logf_data.c', 'logf_data.h', 'expf.c', 'exp2f_data.c', 'exp2f_data.h',
           'sin.c', 'cos.c', 'sincos.c', '__sin.c', '__cos.c', '__rem_pio2.c',
           'pow.c', 'pow_data.c', 'pow_data.h', 'exp_data.c', 'exp_data.h',
           '__math_xflowf.c', '__math_uflowf.c', '__math_oflowf.c', '__math_divzerof.c', '__math_invalidf.c',
           '__math_xflow.c', '__math_uflow.c', '__math_oflow.c', '__math_divzero.c', '__math_invalid.c'):
  MUSL_SRC['src/' + _f] = ('src/math/' + _f, 'verbatim', 'header' if _f.endswith('.h') else 'tu')

HEADER = ['path', 'upstream_repo', 'upstream_commit', 'upstream_path', 'upstream_sha256', 'vendored_sha256',
          'patches', 'role']
META_FILES = {'MANIFEST.tsv', 'PROVENANCE.md'}


def sha256(path):
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    for chunk in iter(lambda: f.read(1 << 20), b''):
      h.update(chunk)
  return h.hexdigest()


def tree_files(root):
  out = []
  for d, _, files in os.walk(root):
    for f in files:
      rel = os.path.relpath(os.path.join(d, f), root)
      if rel not in META_FILES:
        out.append(rel)
  return sorted(out)


def read_tsv(path):
  rows = []
  header = None
  with open(path) as f:
    for line in f:
      line = line.rstrip('\n')
      if not line or line.startswith('#'):
        continue
      cols = line.split('\t')
      if header is None:
        header = cols
        continue
      rows.append(dict(zip(header, cols)))
  return header, rows


def write_tsv(path, comment, rows):
  with open(path, 'w') as f:
    for c in comment:
      f.write('# ' + c + '\n')
    f.write('\t'.join(HEADER) + '\n')
    for r in rows:
      f.write('\t'.join(r[h] for h in HEADER) + '\n')


def git_last_commit(rel):
  if os.path.join(REPO, rel).startswith(UPSTREAM_8W + os.sep):
    return COMMIT_8W  # gitignored reference checkout (verified by the xr_reference upstream check)
  try:
    return subprocess.check_output(['git', '-C', REPO, 'log', '-1', '--format=%H', '--', rel],
                                   text=True).strip() or 'uncommitted'
  except (OSError, subprocess.CalledProcessError):
    return 'unknown'


def patched_files(patch_path):
  files = []
  with open(patch_path) as f:
    for line in f:
      if line.startswith('+++ b/'):
        files.append(line[6:].split('\t')[0].strip())
  return files


def strip_policy_line(data):
  """Removes the reference runtime / Niantic copyright line (first three lines). Returns (bytes, removed)."""
  lines = data.split(b'\n')
  for i, line in enumerate(lines[:3]):
    if COPYRIGHT_LINE_8W.match(line.decode('latin-1')):
      del lines[i]
      return b'\n'.join(lines), True
  return data, False


def public_source_bytes(data, rel, public_path=None):
  """Apply the documented public naming/comment pass after verifying original source bytes.

  This is deliberately separate from upstream_sha256: source mutations must still fail even
  when they happen to produce the same public spelling. Algorithm and whitespace edits outside
  these exact substitutions are not accepted.
  """
  try:
    text = data.decode('utf-8')
  except UnicodeDecodeError:
    return data  # binary test data is copied byte for byte
  compact = REPO_8W.rstrip('/').rsplit('/', 1)[-1].removesuffix('.git')
  owner = REFERENCE_METADATA['copyright_owners'][0]
  owner_pattern = r'[\s_-]*'.join(re.escape(part) for part in owner.split())
  names = re.compile(re.escape(compact) + '|' + owner_pattern, re.I)
  text = re.sub(r'\s*\([^()\n]*@' + re.escape(compact) + r'\.com\)', '', text, flags=re.I)
  held = {}
  def hold(match):
    url = match.group()
    if not names.search(url):
      return url
    key = '__SOURCE_URL_%d__' % len(held)
    held[key] = url
    return key
  text = re.sub(r'''https?://[^\s)>\]"'`]+''', hold, text)
  text = text.replace(compact + '_shim', 'xr_shim').replace(compact.upper() + '_SHIM', 'XR_SHIM')
  text = text.replace('third_party/' + compact, 'third_party/xr_source')
  text = text.replace('SR_' + compact.upper(), 'SR_XR_SOURCE')
  text = re.sub(re.escape(compact), 'xr_reference', text, flags=re.I)
  text = re.sub(owner_pattern, 'reference runtime', text, flags=re.I)
  target = public_path or os.path.join(TP, 'xr_source', rel)
  disclosure = os.path.relpath(os.path.join(REPO, 'README.md'), os.path.dirname(target)).replace(os.sep, '/')
  for key, url in held.items():
    text = text.replace(key, url if rel.endswith('.py') else disclosure + '#external-code')
  return text.encode('utf-8')


def upstream_8w_commit_available():
  r = subprocess.run(['git', '-C', UPSTREAM_8W, 'cat-file', '-e', COMMIT_8W + '^{commit}'],
                     capture_output=True)
  return r.returncode == 0


def upstream_8w_bytes(rel):
  """(bytes of `rel` as vendored upstream, copyright line removed) from the commit's git objects,
  or (None, False) if the commit has no such file. Not the reference worktree."""
  r = subprocess.run(['git', '-C', UPSTREAM_8W, 'show', '%s:%s' % (COMMIT_8W, rel)],
                     capture_output=True)
  if r.returncode != 0:
    return None, False
  return strip_policy_line(r.stdout)


def upstream_8w_worktree_bytes(rel):
  """(bytes of `rel` in the reference clone's CURRENT working tree, False), or (None, False) if the
  file is missing. No transformation: the clone is the base as it is (R4)."""
  path = os.path.join(UPSTREAM_8W, rel)
  if not os.path.isfile(path):
    return None, False
  with open(path, 'rb') as f:
    return f.read(), False


def sha256_bytes(data):
  return hashlib.sha256(data).hexdigest()


# ---- manifest writing ---------------------------------------------------------------------------
def write_manifests():
  # xr_reference
  root = os.path.join(TP, 'xr_source')
  touched = {}
  for p in PATCH_ORDER:
    for f in patched_files(os.path.join(root, 'patches', p)):
      touched.setdefault(f, []).append(p[:-len('.patch')])
  rows = []
  for rel in tree_files(root):
    v = sha256(os.path.join(root, rel))
    if rel.startswith('patches/'):
      rows.append(dict(path=rel, upstream_repo='slam_refact', upstream_commit='-', upstream_path='-',
                       upstream_sha256='-', vendored_sha256=v, patches='-', role='patch'))
      continue
    if rel in GENERATED_8W:
      up = GENERATED_8W[rel]
      rows.append(dict(path=rel, upstream_repo=SOURCE_DISCLOSURE, upstream_commit=COMMIT_8W, upstream_path=up,
                       upstream_sha256=sha256_bytes(upstream_8w_bytes(up)[0]), vendored_sha256=v,
                       patches=','.join(touched.get(rel, [])) or '-', role='generated'))
      continue
    rows.append(dict(path=rel, upstream_repo=SOURCE_DISCLOSURE, upstream_commit=COMMIT_8W, upstream_path=rel,
                     upstream_sha256=sha256_bytes(upstream_8w_bytes(rel)[0]), vendored_sha256=v,
                     patches=','.join(touched.get(rel, [])) or '-',
                     role='source-input' if rel in SOURCE_INPUTS else
                          WASM_ONLY_8W.get(rel, 'tu' if rel in TU_8W else 'header')))
  write_tsv(os.path.join(root, 'MANIFEST.tsv'),
            ['Public reference runtime closure vendored by WP0a (architecture.md §2.2). Written by',
             'tools/check_vendor.py --write-manifests; checked by tools/check_vendor.py.',
             'Proof base (R4, user decision 2026-10-20): the CURRENT WORKING TREE of the reference clone',
             '(check_vendor.py --upstream --upstream-base worktree). upstream_sha256 = sha256 of that clone',
             'file as it is; vendored_sha256 = that file + patches/ (PATCH_ORDER) + public naming pass.',
             'Original repository identity and author attribution are in the root README external-code section.',
             'upstream_commit is the clone HEAD the files were taken from, not the proof base.',
             'Local bytes also apply public_source_bytes: names and author emails are centralized in README.',
             'role tu-wasm-only: c8/time/now.cc is excluded on native: unreferenced in closure;',
             'emscripten-only under JAVASCRIPT (emscripten.h, emscripten_get_now).'], rows)
  # xr_shim
  root = os.path.join(TP, 'xr_shim')
  rows = []
  for rel in tree_files(root):
    up, patches, role = SHIM_SRC[rel]
    repo = SOURCE_DISCLOSURE if up.startswith('assets/references/xr_reference/') else 'Mobile-SLAM (RE lane B)'
    rows.append(dict(path=rel, upstream_repo=repo, upstream_commit=git_last_commit(up),
                     upstream_path=up, upstream_sha256=sha256(os.path.join(REPO, up)),
                     vendored_sha256=sha256(os.path.join(root, rel)), patches=patches, role=role))
  write_tsv(os.path.join(root, 'MANIFEST.tsv'),
            ['Shims for the vendored closure (architecture.md §2.2 item 4). Precede third_party/xr_source',
             'on the include path. Written by tools/check_vendor.py --write-manifests.'], rows)
  # libcxx_compat
  root = os.path.join(TP, 'libcxx_compat')
  rows = []
  for rel in tree_files(root):
    v = sha256(os.path.join(root, rel))
    if rel in LIBCXX_NEW:
      up, role = LIBCXX_NEW[rel]
      upsha = sha256(os.path.join(LIBCXX_ROOT, up)) if os.path.isfile(os.path.join(LIBCXX_ROOT, up)) else '-'
      llvm14 = role == 'excerpt-llvm14'
      rows.append(dict(path=rel, upstream_repo=LLVM14_REPO if llvm14 else LIBCXX_REPO,
                       upstream_commit='-' if llvm14 else LIBCXX_COMMIT, upstream_path=up,
                       upstream_sha256=upsha, vendored_sha256=v,
                       patches='verbatim' if role == 'license' else 'manual', role=role))
      continue
    sub = rel[len('include/libcxx_compat/'):]
    up = 'system/lib/libcxx/include/' + sub
    rows.append(dict(path=rel, upstream_repo=LIBCXX_REPO, upstream_commit=LIBCXX_COMMIT, upstream_path=up,
                     upstream_sha256=sha256(os.path.join(LIBCXX_ROOT, up)), vendored_sha256=v,
                     patches=LIBCXX_MANUAL.get(rel, 'transform'), role='excerpt'))
  write_tsv(os.path.join(root, 'MANIFEST.tsv'),
            ['libc++ excerpt of emsdk 5.0.0 in namespace slam_refact::libcxx (architecture.md §5.3).',
             'upstream_path is relative to $EMSDK/upstream/emscripten. Written by',
             'tools/check_vendor.py --write-manifests.'], rows)
  # musl_libm
  root = os.path.join(TP, 'musl_libm')
  rows = []
  for rel in tree_files(root):
    up, patches, role = MUSL_SRC[rel]
    rows.append(dict(path=rel, upstream_repo=MUSL_REPO, upstream_commit=LIBCXX_COMMIT, upstream_path=up,
                     upstream_sha256=sha256(os.path.join(MUSL_ROOT, up)),
                     vendored_sha256=sha256(os.path.join(root, rel)), patches=patches, role=role))
  write_tsv(os.path.join(root, 'MANIFEST.tsv'),
            ['musl 1.2.5 libm subset of emsdk 5.0.0 (host builds only, PROVENANCE.md).',
             'upstream_path is relative to $EMSDK/upstream/emscripten/system/lib/libc/musl. Written by',
             'tools/check_vendor.py --write-manifests.'], rows)
  print('wrote 4 manifests')


# ---- checks -------------------------------------------------------------------------------------
class Result:
  def __init__(self):
    self.failures = []
    self.notes = []

  def fail(self, msg):
    self.failures.append(msg)

  def note(self, msg):
    self.notes.append(msg)


def check_tree_hashes(res, name):
  root = os.path.join(TP, name)
  mpath = os.path.join(root, 'MANIFEST.tsv')
  if not os.path.isfile(mpath):
    res.fail('%s: MANIFEST.tsv missing' % name)
    return []
  header, rows = read_tsv(mpath)
  if header != HEADER:
    res.fail('%s: MANIFEST.tsv header %s != %s' % (name, header, HEADER))
    return rows
  listed = {r['path'] for r in rows}
  actual = set(tree_files(root))
  for rel in sorted(actual - listed):
    res.fail('%s: file not in MANIFEST: %s' % (name, rel))
  for rel in sorted(listed - actual):
    res.fail('%s: MANIFEST entry has no file: %s' % (name, rel))
  ok = 0
  for r in rows:
    p = os.path.join(root, r['path'])
    if os.path.isfile(p):
      if sha256(p) != r['vendored_sha256']:
        res.fail('%s: hash mismatch: %s' % (name, r['path']))
      else:
        ok += 1
  res.note('(a) %s: %d/%d files match MANIFEST' % (name, ok, len(rows)))
  return rows


def check_data(res, upstream):
  data = os.path.join(ENGINE, 'tests/data')
  header, rows = read_tsv(os.path.join(data, 'MANIFEST.tsv'))
  listed = {r['path'] for r in rows}
  actual = set(tree_files(data)) - {'MANIFEST.tsv'}
  for rel in sorted(actual - listed):
    res.fail('data: file not in MANIFEST: %s' % rel)
  ok = 0
  src_ok = src_missing = 0
  for r in rows:
    p = os.path.join(data, r['path'])
    if not os.path.isfile(p):
      res.fail('data: missing %s' % r['path'])
      continue
    if sha256(p) != r['sha256'] or os.path.getsize(p) != int(r['size']):
      res.fail('data: hash/size mismatch %s' % r['path'])
    else:
      ok += 1
    if upstream and r['kind'] in ('copy', 'excerpt'):
      s = os.path.join(REPO, r['src_path'])
      if not os.path.isfile(s):
        src_missing += 1
      elif sha256(s) != r['src_sha256']:
        res.fail('data: source changed since copy: %s' % r['src_path'])
      else:
        src_ok += 1
        if r['kind'] == 'copy':
          with open(s, 'rb') as source:
            public = public_source_bytes(source.read(), r['path'], p)
          if sha256_bytes(public) != r['sha256']:
            res.fail('data: source plus public naming pass differs from copy: %s' % r['path'])
  res.note('(a) tests/data: %d/%d files match MANIFEST (%d bytes)' %
           (ok, len(rows), sum(int(r['size']) for r in rows)))
  # A file present on disk but ignored by git passes the hash check locally and
  # is then missing on a fresh clone (e.g. *.out under the root .gitignore).
  ls = subprocess.run(['git', 'ls-files', '--', data], cwd=REPO, capture_output=True, text=True)
  if ls.returncode == 0:
    tracked = {os.path.relpath(os.path.join(REPO, f), data) for f in ls.stdout.split()}
    for r in rows:
      if r['path'] not in tracked:
        res.fail('data: listed in MANIFEST but not tracked by git: %s' % r['path'])
  if upstream:
    res.note('(b) tests/data sources: %d unchanged, %d not present' % (src_ok, src_missing))
    if src_missing:
      res.fail('data: %d source files not present (evidence tier BLOCKED)' % src_missing)


def apply_patches(tree, patch_dir):
  for p in PATCH_ORDER:
    r = subprocess.run(['patch', '-s', '-p1', '-d', tree, '-i', os.path.join(patch_dir, p)],
                       capture_output=True, text=True)
    if r.returncode != 0:
      return '%s failed: %s%s' % (p, r.stdout, r.stderr)
  return None


HUNK_HEADER = re.compile(r'^@@ -\d+(?:,(\d+))? \+\d+(?:,(\d+))? @@')
# A changed line of an include-only patch is, as a whole, one #include with at most one comment: a
# `//` comment or ONE closed block comment. The block comment is matched without a greedy `.*`
# (RM-1c, review RM-1b/RM-5 MINOR-3): `/\*.*\*/` also matched `#include "a.h" /* x */ int y; /* z */`
# because it swallowed the code between two comments.
_BLOCK_COMMENT = r'/\*(?:[^*]|\*(?!/))*\*/'
# `//[^\r\n]*`, not `//.*`: `.` also matches a carriage return (RM-1d, review RM-1c MINOR-2).
INCLUDE_LINE = re.compile(
    r'^[+-]\s*#\s*include\s*(<[^>]+>|"[^"]+")\s*(//[^\r\n]*|' + _BLOCK_COMMENT + r')?\s*$')


def is_include_only_line(line):
  """True when a changed diff line (with its leading + or -) is just one #include.

  A changed line that ends in a backslash is refused whatever it holds: the backslash joins the
  next, unchanged line to it, so a `//` comment ending in a backslash hides that code line from
  this check (RM-1c, review RM-1b/RM-5 MINOR-3).

  A changed line also must not contain a carriage return (other than the one of a CRLF line end),
  a form feed or a vertical tab (RM-1d, review RM-1c MINOR-2): `patch` applies a lone CR as it is
  and a C compiler treats it as a line end, so `#include "a.h" // c<CR>int y = 7;` is an include
  line for this check and code for the compiler. FF and VT are not line ends for the check but
  have no place in an include line."""
  body = line[:-1] if line.endswith('\r') else line
  if '\r' in body or '\f' in body or '\v' in body:
    return False
  if line.rstrip().endswith('\\'):
    return False
  return INCLUDE_LINE.match(line) is not None


def patch_changed_lines(path):
  """Yields every added or removed line of the hunks of a unified diff (RM-1b, review MINOR-6).

  Hunks are read by their @@ -a,b +c,d @@ counts, so a changed line that itself starts with '++'
  or '--' (`+++x;` adds the line `++x;`) is not mistaken for a file header."""
  # newline='': keep a lone CR inside its line. The universal-newline default turns it into a line
  # break, which would make the code after it look like an unchanged context line (RM-1d, review
  # RM-1c MINOR-2).
  with open(path, encoding='utf-8', errors='replace', newline='') as f:
    lines = f.read().split('\n')
  i = 0
  while i < len(lines):
    m = HUNK_HEADER.match(lines[i])
    i += 1
    if not m:
      continue  # diff --git / index / --- / +++ headers and anything else between hunks
    old_left = int(m.group(1)) if m.group(1) is not None else 1
    new_left = int(m.group(2)) if m.group(2) is not None else 1
    while (old_left > 0 or new_left > 0) and i < len(lines):
      line = lines[i]
      i += 1
      if line.startswith('+'):
        new_left -= 1
        yield line
      elif line.startswith('-'):
        old_left -= 1
        yield line
      elif line.startswith('\\'):  # "\ No newline at end of file"
        continue
      else:  # context line (a blank context line may have lost its leading space)
        old_left -= 1
        new_left -= 1


def check_patch_hunks(res):
  """(a) The patches documented as "include lines only" change nothing but #include lines."""
  root = os.path.join(TP, 'xr_source', 'patches')
  checked = 0
  for name in INCLUDE_ONLY_PATCHES:
    path = os.path.join(root, name)
    if not os.path.isfile(path):
      res.fail('xr_reference patches: %s missing' % name)
      continue
    changed = 0
    for line in patch_changed_lines(path):
      if not line[1:].strip():  # a blank line next to the include is not code
        continue
      changed += 1
      if not is_include_only_line(line):
        res.fail('xr_reference patches: %s is documented as include-only but changes: %s' % (name, line))
    if changed == 0:
      res.fail('xr_reference patches: %s has no changed lines' % name)
    checked += 1
  res.note('(a) xr_reference patches: %d include-only patches change only #include lines' % checked)


def check_xr_reference_upstream(res, rows, base_mode='commit'):
  """base_mode 'commit': git objects of COMMIT_8W minus the copyright line (the original check).
  base_mode 'worktree': the reference clone's current working tree as it is (R4)."""
  root = os.path.join(TP, 'xr_source')
  worktree = base_mode == 'worktree'
  base_of = upstream_8w_worktree_bytes if worktree else upstream_8w_bytes
  label = 'clone working tree' if worktree else 'commit'
  if not os.path.isdir(UPSTREAM_8W):
    res.fail('(b) xr_reference: upstream tree %s not present (BLOCKED)' % UPSTREAM_8W)
    return
  if not worktree and not upstream_8w_commit_available():
    res.fail('(b) xr_reference: commit %s is not in the reference checkout %s; the commit-based check '
             'cannot run and does not fall back to the (edited) worktree (BLOCKED)' %
             (COMMIT_8W, UPSTREAM_8W))
    return
  stripped = unchanged = 0
  with tempfile.TemporaryDirectory() as tmp:
    for r in rows:
      if r['role'] in ('patch',):
        continue
      base, removed = base_of(r['upstream_path'])
      if base is None:
        res.fail('(b) xr_reference: %s is not in the %s%s' % (
            r['upstream_path'], label, '' if worktree else ' ' + COMMIT_8W[:8]))
        continue
      if sha256_bytes(base) != r['upstream_sha256']:
        res.fail('(b) xr_reference: upstream_sha256 is not the %s file%s: %s' %
                 (label, ' minus its copyright line' if removed else '', r['upstream_path']))
        continue
      if r['role'] == 'generated':
        continue
      if removed:
        stripped += 1
      else:
        unchanged += 1
      dst = os.path.join(tmp, r['path'])
      os.makedirs(os.path.dirname(dst), exist_ok=True)
      with open(dst, 'wb') as f:
        f.write(base)
    err = apply_patches(tmp, os.path.join(root, 'patches'))
    if err:
      res.fail('(b) xr_reference: ' + err)
      return
    same = 0
    for r in rows:
      if r['role'] in ('patch', 'generated'):
        continue
      a = os.path.join(tmp, r['path'])
      if not os.path.isfile(a):
        continue
      with open(a, 'rb') as source:
        public = public_source_bytes(source.read(), r['path'])
      if sha256_bytes(public) != r['vendored_sha256']:
        res.fail('(b) xr_reference: %s file + patches != vendored for %s' % (label, r['path']))
      else:
        same += 1
    if worktree:
      res.note('(b) xr_reference: base = current working tree of the reference clone %s (no git, no '
               'pre-hash transformation); %d upstream files hashed as they are' % (UPSTREAM_8W, unchanged))
    else:
      res.note('(b) xr_reference: base = git objects of commit %s (reference worktree not used); %d files = '
               'commit file minus its reference runtime/Niantic copyright line, %d files = commit file unchanged' %
               (COMMIT_8W[:8], stripped, unchanged))
    res.note('(b) xr_reference: %s file + %d patches + public naming/comment pass reproduces %d vendored files byte for byte' %
             (label, len(PATCH_ORDER), same))
    # Informational (never a failure): how far the two bases are from each other.
    if worktree and not upstream_8w_commit_available():
      res.note('(b) xr_reference: commit %s not in the clone; commit-vs-working-tree comparison skipped' %
               COMMIT_8W[:8])
    else:
      differ = 0
      for r in rows:
        if r['role'] in ('patch', 'generated'):
          continue
        other, _ = upstream_8w_bytes(r['upstream_path']) if worktree else \
            upstream_8w_worktree_bytes(r['upstream_path'])
        mine, _ = base_of(r['upstream_path'])
        if other is None or mine is None or other != mine:
          differ += 1
      res.note('(b) xr_reference: %d of %d vendored upstream files differ between the clone working tree and '
               'the commit file minus its copyright line' % (differ, stripped + unchanged))
    for r in rows:
      if r['role'] != 'generated':
        continue
      # Run the generator from the base's own files (it imports rotation.py and memoizer.py).
      gen_dir = os.path.dirname(r['upstream_path'])
      src = os.path.join(tmp, 'gen_src')
      os.makedirs(src, exist_ok=True)
      if worktree:
        shutil.copytree(os.path.join(UPSTREAM_8W, gen_dir), os.path.join(src, gen_dir),
                        ignore=shutil.ignore_patterns('__pycache__'))
      else:
        arc = subprocess.run(['git', '-C', UPSTREAM_8W, 'archive', COMMIT_8W, gen_dir],
                             capture_output=True)
        if arc.returncode != 0:
          res.fail('(b) xr_reference: git archive of %s failed: %s' % (gen_dir, arc.stderr.decode()[-200:]))
          continue
        with tarfile.open(fileobj=io.BytesIO(arc.stdout)) as tf:
          try:
            tf.extractall(src, filter='data')  # python >= 3.10.12
          except TypeError:
            tf.extractall(src)
      out = os.path.join(tmp, 'gen', r['path'])
      os.makedirs(os.path.dirname(out), exist_ok=True)
      g = subprocess.run([sys.executable, os.path.join(src, r['upstream_path']), out],
                         capture_output=True, text=True)
      if g.returncode != 0:
        res.fail('(b) xr_reference: generator failed for %s (sympy needed): %s' % (r['path'], g.stderr.strip()[-300:]))
      elif sha256_bytes(public_source_bytes(open(out, 'rb').read(), r['path'])) != r['vendored_sha256']:
        res.fail('(b) xr_reference: regenerated %s differs from vendored' % r['path'])
      else:
        res.note('(b) xr_reference: regenerated %s matches (generator from the %s)' % (r['path'], label))


def check_shim_upstream(res, rows):
  for r in rows:
    up = os.path.join(REPO, r['upstream_path'])
    if not os.path.isfile(up):
      res.fail('(b) shim: missing %s' % r['upstream_path'])
      continue
    with open(up, 'rb') as source:
      original = source.read()
    if sha256_bytes(original) != r['upstream_sha256']:
      # These local probe headers changed only public source names. Reconstruct and hash the
      # pinned input, so a fresh public clone does not need unpublished historical git objects.
      compact = REPO_8W.rstrip('/').rsplit('/', 1)[-1].removesuffix('.git').encode()
      owner = REFERENCE_METADATA['copyright_owners'][0].encode()
      historical = original.replace(b'xr_reference', compact).replace(b'reference runtime', owner)
      if (sha256_bytes(historical) != r['upstream_sha256'] or
          public_source_bytes(historical, r['path'], up) != original):
        res.fail('(b) shim: lane-B source changed: %s' % r['upstream_path'])
        continue
      original = historical
    if r['patches'] == '-' and sha256_bytes(public_source_bytes(original, r['path'])) != r['vendored_sha256']:
      res.fail('(b) shim: verbatim copy plus public naming pass differs: %s' % r['path'])
  res.note('(b) xr_shim: %d lane-B sources checked' % len(rows))


def check_libcxx_upstream(res, rows):
  if not os.path.isdir(LIBCXX_ROOT):
    res.fail('(b) libcxx_compat: %s not present (BLOCKED)' % LIBCXX_ROOT)
    return
  with tempfile.TemporaryDirectory() as tmp:
    r = subprocess.run([sys.executable, os.path.join(ENGINE, 'tools/libcxx_compat_transform.py'), tmp],
                       capture_output=True, text=True)
    if r.returncode != 0:
      res.fail('(b) libcxx_compat: transform failed: %s' % r.stderr)
      return
    exact = manual = 0
    for row in rows:
      up = os.path.join(LIBCXX_ROOT, row['upstream_path'])
      if row['upstream_sha256'] != '-':
        if not os.path.isfile(up) or sha256(up) != row['upstream_sha256']:
          res.fail('(b) libcxx_compat: upstream hash mismatch %s' % row['upstream_path'])
          continue
      if row['role'] != 'excerpt':
        continue
      stage1 = os.path.join(tmp, row['path'][len('include/libcxx_compat/'):])
      if row['patches'] == 'transform':
        if sha256(stage1) != row['vendored_sha256']:
          res.fail('(b) libcxx_compat: %s differs from transform output' % row['path'])
        else:
          exact += 1
      else:
        d = subprocess.run(['diff', stage1, os.path.join(TP, 'libcxx_compat', row['path'])],
                           capture_output=True, text=True).stdout
        changed = sum(1 for line in d.splitlines() if line[:2] in ('< ', '> '))
        res.note('(b) libcxx_compat: %s manual diff %d lines (%s)' % (row['path'], changed, row['patches']))
        manual += 1
    res.note('(b) libcxx_compat: %d excerpts equal the mechanical transform, %d carry manual edits' %
             (exact, manual))


def check_musl_upstream(res, rows):
  if not os.path.isdir(MUSL_ROOT):
    res.fail('(b) musl_libm: %s not present (BLOCKED)' % MUSL_ROOT)
    return
  verbatim = 0
  for row in rows:
    up = os.path.join(MUSL_ROOT, row['upstream_path'])
    if not os.path.isfile(up) or sha256(up) != row['upstream_sha256']:
      res.fail('(b) musl_libm: upstream hash mismatch %s' % row['upstream_path'])
      continue
    if row['patches'] == 'verbatim':
      if row['upstream_sha256'] != row['vendored_sha256']:
        res.fail('(b) musl_libm: %s is marked verbatim but differs from upstream' % row['path'])
      else:
        verbatim += 1
  res.note('(b) musl_libm: %d files verbatim, %d manual' % (verbatim, len(rows) - verbatim))


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument('--upstream', action='store_true', help='also run the (b) evidence-tier checks')
  ap.add_argument('--upstream-base', choices=('commit', 'worktree'), default='worktree',
                  help='xr_reference proof base of --upstream. worktree (default; R4, user decision 2026-10-20): the clone '
                       'current working tree as it is, no git. commit: git objects of the clone commit minus '
                       'the copyright line (previous behavior, kept as an option).')
  ap.add_argument('--data', action='store_true', help='also check tests/data against its MANIFEST')
  ap.add_argument('--write-manifests', action='store_true')
  a = ap.parse_args()
  if a.write_manifests:
    write_manifests()
    return 0
  res = Result()
  rows_8w = check_tree_hashes(res, 'xr_source')
  rows_shim = check_tree_hashes(res, 'xr_shim')
  rows_lcx = check_tree_hashes(res, 'libcxx_compat')
  rows_musl = check_tree_hashes(res, 'musl_libm')
  check_patch_hunks(res)
  if a.upstream:
    check_xr_reference_upstream(res, rows_8w, a.upstream_base)
    check_shim_upstream(res, rows_shim)
    check_libcxx_upstream(res, rows_lcx)
    check_musl_upstream(res, rows_musl)
  if a.data:
    check_data(res, a.upstream)
  for n in res.notes:
    print(n)
  for f in res.failures:
    print('FAIL ' + f)
  print('check_vendor: %s' % ('PASS' if not res.failures else 'FAIL (%d)' % len(res.failures)))
  return 0 if not res.failures else 1


if __name__ == '__main__':
  sys.exit(main())
