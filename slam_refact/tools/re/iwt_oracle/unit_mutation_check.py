#!/usr/bin/env python3
"""Mutation run of sr_tests_image_world_tracker (US-015 review F-3): the 20 mutants + 3 nulls of the
independent review (opus-xhigh-r1-us015-impl.md section 5.2) and X1-X4 of the it-lane closure review (N-4; this file is that reviewer script with the build
directory as an argument). Each mutant is a copy of one source with one planted bug, compiled with the exact
ninja compile command of the original and linked in front of the static libraries; the shared sources and the
build tree are not written. A mutant is detected when the test binary exits non-zero.

  python3 -I unit_mutation_check.py BUILD_DIR OUT_DIR
Stdlib only."""
import json, os, pathlib, shlex, subprocess, sys

REPO = pathlib.Path(__file__).resolve().parents[4]
BUILD = pathlib.Path(sys.argv[1]).resolve()
OUT = pathlib.Path(sys.argv[2]).resolve()
CMDS = subprocess.run(['ninja', '-C', str(BUILD), '-t', 'commands', 'sr_tests_image_world_tracker'],
                      check=True, capture_output=True, text=True).stdout.splitlines()
ENG = REPO / 'slam_refact/engine'
IWT = ENG / 'src/image_target/image_world_tracker.cc'
ITS = ENG / 'third_party/xr_source/reality/engine/imagedetection/image-tracking-state.cc'
VT = ENG / 'src/vio_tracker/vio_tracker.cc'

MUTANTS = [
  ('N0-null-iwt', IWT, 'constexpr size_t kMinPairs = 6;', 'constexpr size_t kMinPairs = 6;'),
  ('N1-null-its', ITS, 'if (trackedImage_.lastSeen > WORLD_RESET_UNTRACKED_FRAMES) {', 'if (trackedImage_.lastSeen > WORLD_RESET_UNTRACKED_FRAMES) {'),
  ('N2-null-vt', VT, 'out->source = static_cast<int32_t>(p->source);', 'out->source = static_cast<int32_t>(p->source);'),
  ('U1-min-pairs-5', IWT, 'constexpr size_t kMinPairs = 6;', 'constexpr size_t kMinPairs = 5;'),
  ('U2-any-source', IWT, 'mp.source != kSourceTriangulated', 'mp.source < 0'),
  ('U3-ignore-visible', IWT, 'for (size_t idx : st->visible) {', 'for (size_t idx = 0; idx < st->searchPtsInCamera.size(); ++idx) {'),
  ('U4-no-tam-inv', IWT, 'tam.inv() * st->worldPts', 'tam * st->worldPts'),
  ('U5-guess-not-stored', IWT, '    st->scale = scale_guess;\n', ''),
  ('U6-unusable-overwrites', IWT, 'if (c8::poseEstimationScale3d(st->targetPts, model, &st->scale)) {', 'c8::poseEstimationScale3d(st->targetPts, model, &st->scale); {'),
  ('U7-offset-sign-x', IWT, '(mp.position.x() - xf.offset[0]) * xf.scale', '(mp.position.x() + xf.offset[0]) * xf.scale'),
  ('U8-fallback-P-not-inv', IWT, 'c8::scaleTranslation(scale_guess, target_view_pose.inv())', 'c8::scaleTranslation(scale_guess, target_view_pose)'),
  ('U9-solve-starts-at-guess', IWT, 'const c8::Vector<c8::HPoint3> model = tam.inv() * st->worldPts;', 'st->scale = scale_guess; const c8::Vector<c8::HPoint3> model = tam.inv() * st->worldPts;'),
  ('U10-offset-sign-y', IWT, '(mp.position.y() - xf.offset[1]) * xf.scale', '(mp.position.y() + xf.offset[1]) * xf.scale'),
  ('V1-no-reset', ITS, 'if (trackedImage_.lastSeen > WORLD_RESET_UNTRACKED_FRAMES) {', 'if (false) {'),
  ('V2-reset-every-frame', ITS, 'if (trackedImage_.lastSeen > WORLD_RESET_UNTRACKED_FRAMES) {', 'if (true) {'),
  ('V3-no-scaleguess-copy', ITS, '      scaleGuess_ = worldTracker_.scale;\n', ''),
  ('V4-lost-public-path', ITS, 'if (!localMode_ && worldTracker != nullptr) {', 'if (false) {'),
  ('V5-hook-first-frame-only', ITS, '    if (worldTracker != nullptr) {', '    if (worldTracker != nullptr && !worldInit_) {'),
  ('V6-reset-after-any-loss', ITS, 'if (trackedImage_.lastSeen > WORLD_RESET_UNTRACKED_FRAMES) {', 'if (trackedImage_.lastSeen > 0) {'),
  ('W1-ignore-disable-vio', VT, 'diagnostics::ImageWorldTrackerEnabled() && !s.disable_vio', 'diagnostics::ImageWorldTrackerEnabled()'),
  ('W2-no-lookup', VT, 's.image_world_tracker.set_lookup(&lookup);', 's.image_world_tracker.set_lookup(nullptr);'),
  ('W3-adapter-source-always-tri', VT, 'out->source = static_cast<int32_t>(p->source);', 'out->source = 1;'),
  ('W4-hook-always-on', VT, 'const bool world_tracker = diagnostics::ImageWorldTrackerEnabled() && !s.disable_vio;', 'const bool world_tracker = !s.disable_vio;'),
  # it-lane closure review N-4 (extra_mut.py of that reviewer): reset threshold and scale kept at reset.
  ('X1-reset-at-100', ITS, 'if (trackedImage_.lastSeen > WORLD_RESET_UNTRACKED_FRAMES) {', 'if (trackedImage_.lastSeen >= WORLD_RESET_UNTRACKED_FRAMES) {'),
  ('X2-reset-at-91', ITS, 'if (trackedImage_.lastSeen > WORLD_RESET_UNTRACKED_FRAMES) {', 'if (trackedImage_.lastSeen > WORLD_RESET_UNTRACKED_FRAMES - 10) {'),
  ('X3-reset-clears-scale', ITS, '        worldTracker_.reset();\n', '        worldTracker_.reset();\n        worldTracker_.scale = 1.0f;\n'),
  ('X4-lost-scale-not-inverted', ITS, 'scaleTranslation(1.0f / scaleGuess_, egomotion(worldPose, tamExtrinsic))', 'scaleTranslation(scaleGuess_, egomotion(worldPose, tamExtrinsic))'),
]

def compile_cmd_for(src):
  c = [l for l in CMDS if (' -c ' + str(src)) in l]
  assert len(c) == 1, (src, len(c))
  return c[0]

def link_cmd():
  c = [l for l in CMDS if '-o tests/image_target/sr_tests_image_world_tracker ' in l]
  assert len(c) == 1
  return c[0]

def absolutize(argv):
  out = []
  for a in argv:
    if not a.startswith('-') and not os.path.isabs(a) and (BUILD / a).exists():
      out.append(str(BUILD / a))
    else:
      out.append(a)
  return out

def main():
  OUT.mkdir(parents=True, exist_ok=True)
  lc = link_cmd().strip()
  lc = lc[lc.index('/usr/bin/c++'):]
  lc = lc.split(' && ')[0]
  results = {}
  for name, src, old, new in MUTANTS:
    text = src.read_text()
    n = text.count(old)
    if n != 1:
      results[name] = {'error': f'pattern count {n}'}
      continue
    d = OUT / name
    d.mkdir(exist_ok=True)
    msrc = d / src.name
    msrc.write_text(text.replace(old, new))
    argv = shlex.split(compile_cmd_for(src))
    argv[argv.index('-o') + 1] = str(d / 'mutant.o')
    argv[argv.index('-c') + 1] = str(msrc)
    for flag in ('-MT', '-MF'):
      while flag in argv:
        i = argv.index(flag); del argv[i:i + 2]
    argv = [a for a in argv if a != '-MD']
    argv = absolutize(argv)
    r = subprocess.run(argv, cwd=str(OUT), capture_output=True, text=True)
    if r.returncode != 0:
      results[name] = {'error': 'compile', 'stderr': r.stderr[-2000:]}
      continue
    largv = absolutize(shlex.split(lc))
    lo = largv.index('-o')
    largv[lo + 1] = str(d / 'test_mutant')
    largv.insert(lo, str(d / 'mutant.o'))
    r = subprocess.run(largv, cwd=str(OUT), capture_output=True, text=True)
    if r.returncode != 0:
      results[name] = {'error': 'link', 'stderr': r.stderr[-3000:]}
      continue
    r = subprocess.run([str(d / 'test_mutant'), '--gtest_brief=1'], cwd=str(d), capture_output=True, text=True, timeout=600)
    failed = sorted({l.split('] ')[1].split(' ')[0] for l in r.stdout.splitlines() if l.startswith('[  FAILED  ]') and '] ' in l and '.' in l.split('] ')[1]})
    results[name] = {'exit': r.returncode, 'detected': r.returncode != 0, 'failed_tests': failed}
    (d / 'gtest.out').write_text(r.stdout[-20000:])
    print(name, results[name], flush=True)
  (OUT / 'results.json').write_text(json.dumps(results, indent=1))
  print(json.dumps({k: (v.get('detected'), v.get('error')) for k, v in results.items()}, indent=1))


if __name__ == '__main__':
  main()
