#!/usr/bin/env python3
"""Gate it-world-gate runner (US-015): engine image_target::TrackImageWorld against the original
xr-slam.wasm func994 on random cases (preregistration/it-world-gate.v<N>.md; v1 and v2 use this runner).

  dev        python3 -I it_world_gate.py dev --out DIR [--seed 777] [--n 60]
  selftest   python3 -I it_world_gate.py selftest --out DIR --dev DEVDIR --oracle-repeat FILE --mutants MUTDIR
             (null-run, perturbation sensitivity and code mutants of the metric on a dev run; section 4-5)
  real run   python3 -I it_world_gate.py registered --out DIR --designation FILE --gate it-world-gate.v<N>
                    [--replay BIN] [--wasm WASM]
             (ONE run only; refuses to start (HELD) unless every condition of the gate md section 6 holds. The version is
             designated by --gate, never by this file; --replay / --wasm name the pinned binaries of that registration, the
             designation file pins their sha256. dev mode keeps the v1 defaults of the two paths.)

Both modes: make_cases.py --variant random -> run_f994.mjs (original) and sr_iwt_replay (engine) ->
the registered metric below -> DIR/result.json. The registered mode takes the seed from the coordinator
seed file named in the designation, checks the pins, and creates the one-run marker with O_EXCL before any
case is generated. It never edits the manifest.

Metric (gate md section 3): per case, integer fields (pair count, visible indices, ids, element counts)
must be equal (G-O1); every float output (scale, out HMatrix fwd + inv, worldPose, rays, searchPtsInCamera,
worldPts, targetPts) e = |engine - original| / max(1, |original|), non-finite values must be non-finite on
both sides; G-O2: max e over all cases <= 1e-5.
Stdlib only.
"""
import hashlib
import json
import math
import os
import pathlib
import struct
import subprocess
import sys

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parents[3]
GATE_ID = 'it-world-gate'
PREREG = REPO / 'build/slam-refact-full-20261019/preregistration'
WASM = REPO / 'build/slam-refact-full-20261019/us015-re/oracle/xr-slam-f994.wasm'
REPLAY = REPO / 'build/slam-refact-full-us015re/tests/image_target/sr_iwt_replay'
TOL = 1e-5
N_REGISTERED = 200
DEV_SEEDS = (777, 4242)  # seeds the executor ran before the review


def sha(p):
  return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def f32(x):
  if x is None:
    return math.nan
  return struct.unpack('<f', struct.pack('<f', x))[0]


def held(msg):
  print('HELD: ' + msg)
  sys.exit(3)


def gate_version(name):
  """N of the designated gate '<GATE_ID>.v<N>'; anything else is refused before any file is read."""
  prefix = GATE_ID + '.v'
  digits = name[len(prefix):] if name.startswith(prefix) else ''
  if not digits.isdigit() or digits.startswith('0') or len(digits) > 6:
    held('the gate is not designated as %s<N> (--gate %r)' % (prefix, name))
  return int(digits)


def check_registered(designation_path, version):
  """Gate md section 6: every condition before any case exists."""
  checks = []
  manifest_path = PREREG / ('%s.v%d.manifest.json' % (GATE_ID, version))
  gate_md = PREREG / ('%s.v%d.md' % (GATE_ID, version))
  m = json.loads(manifest_path.read_text())
  d_bytes = pathlib.Path(designation_path).read_bytes()
  d = json.loads(d_bytes)
  review = m.get('review') or {}
  if review.get('designation_file_sha256') != hashlib.sha256(d_bytes).hexdigest():
    held('designation sha256 differs from manifest review.designation_file_sha256')
  checks.append('designation sha256 ok')
  if not review.get('at'):
    held('manifest review.at not recorded')
  if (d.get('gate') not in (GATE_ID, '%s.v%d' % (GATE_ID, version)) or d.get('version', version) != version or
      d.get('first_real_run_permitted') is not True):
    held('designation does not permit the first real run of %s.v%d' % (GATE_ID, version))
  if m.get('sha256') != sha(gate_md) or (d.get('pinned') or {}).get('gate_md_sha256') != sha(gate_md):
    held('gate md sha256 differs from the pins')
  pins = d.get('pinned') or {}
  for key, path in (('replay_sha256', REPLAY), ('wasm_sha256', WASM),
                    ('make_cases_sha256', HERE / 'make_cases.py'), ('run_f994_sha256', HERE / 'run_f994.mjs'),
                    ('compare_sha256', HERE / 'compare.py'),
                    ('runner_sha256', pathlib.Path(__file__).resolve())):
    if pins.get(key) != sha(path):
      held('%s differs from the designation pin (%s)' % (key, path))
    checks.append(key + ' ok')
  seed_file = PREREG / ('%s.v%d.coordinator-seed.txt' % (GATE_ID, version))
  if not seed_file.exists():
    held('coordinator seed file missing: %s' % seed_file)
  seed = int(seed_file.read_text().split()[0])
  if seed in DEV_SEEDS:
    held('the coordinator seed equals an executor dev seed')
  marker = PREREG / ('%s.v%d.first-run.marker' % (GATE_ID, version))
  try:
    fd = os.open(marker, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o444)
  except FileExistsError:
    held('one-run marker exists: %s' % marker)
  os.write(fd, ('first real run started; seed file sha256 %s\n' % sha(seed_file)).encode())
  os.close(fd)
  checks.append('marker created')
  return seed, checks


def evaluate(cases_json, oracle_json, engine_txt):
  sys.path.insert(0, str(HERE))
  from compare import parse_engine  # noqa: E402
  oracle = json.loads(pathlib.Path(oracle_json).read_text())
  engine = parse_engine(engine_txt)
  per_case, g1_fail, worst = {}, [], (0.0, None)
  for r in oracle['results']:
    name = r['name']
    e = engine.get(name)
    if r['error'] or e is None:
      g1_fail.append(name)
      per_case[name] = {'error': r['error'] or 'engine missing'}
      continue
    s = r['state']
    ints_ok = (s['visible'] == e['visible'] and s['ids'] == e['ids'] and
               all(len(s[a]) == len(e[b]) for a, b in (('rays', 'rays'), ('search_pts_in_camera', 'search'),
                                                         ('world_pts', 'world'), ('target_pts', 'target'))))
    pairs = [(s['scale'], e['scale'])]
    for a, b in ((r['out'], e['out']), (s['world_pose'], e['state_pose'])):
      pairs += list(zip(a['m'] + a['inv'], b['m'] + b['inv']))
    if ints_ok:
      for a, b in (('rays', 'rays'), ('search_pts_in_camera', 'search'), ('world_pts', 'world'),
                   ('target_pts', 'target')):
        for pa, pb in zip(s[a], e[b]):
          pairs += list(zip(pa, pb))
    emax, class_mismatch = 0.0, 0
    for o, g in pairs:
      o, g = f32(o), f32(g)
      if not (math.isfinite(o) and math.isfinite(g)):  # JSON null (NaN or +-inf) on the original side
        class_mismatch += int(math.isfinite(o) or math.isfinite(g))
        continue
      emax = max(emax, abs(g - o) / max(1.0, abs(o)))
    if not ints_ok or class_mismatch:
      g1_fail.append(name)
    if emax > worst[0]:
      worst = (emax, name)
    per_case[name] = {'integers_equal': ints_ok, 'nonfinite_class_mismatch': class_mismatch, 'max_rel': emax,
                      'n_values': len(pairs), 'pairs': len(s['world_pts'])}
  g1 = not g1_fail
  g2 = worst[0] <= TOL and g1
  return {'G-O1': {'pass': g1, 'failing': g1_fail}, 'G-O2': {'pass': g2, 'max_rel': worst[0], 'worst_case': worst[1],
                                                            'tolerance': TOL}, 'cases': per_case}


def oracle_to_engine_txt(oracle_json, path):
  """Writes an original run in the engine text format (null-run: original against original)."""
  lines = []
  def fl(v):
    return ' '.join('%.9g' % f32(x) for x in v)
  for r in json.loads(pathlib.Path(oracle_json).read_text())['results']:
    s = r['state']
    lines.append('case ' + r['name'])
    lines.append('scale %.9g' % f32(s['scale']))
    lines.append('out ' + fl(r['out']['m'] + r['out']['inv']))
    lines.append('state_pose ' + fl(s['world_pose']['m'] + s['world_pose']['inv']))
    lines.append('rays %d %s' % (len(s['rays']), fl([x for p in s['rays'] for x in p])))
    lines.append('search %d %s' % (len(s['search_pts_in_camera']), fl([x for p in s['search_pts_in_camera'] for x in p])))
    lines.append('visible %d %s' % (len(s['visible']), ' '.join(str(v) for v in s['visible'])))
    lines.append('world %d %s' % (len(s['world_pts']), fl([x for p in s['world_pts'] for x in p])))
    lines.append('ids %d %s' % (len(s['ids']), ' '.join('%d %d' % tuple(i) for i in s['ids'])))
    lines.append('target %d %s' % (len(s['target_pts']), fl([x for p in s['target_pts'] for x in p])))
  pathlib.Path(path).write_text('\n'.join(lines) + '\n')


def selftest(out, dev, oracle_repeat, mutants):
  out.mkdir(parents=True, exist_ok=False)
  oracle = dev / 'oracle.json'
  rep = {}
  oracle_to_engine_txt(oracle_repeat, out / 'null-run.txt')
  r = evaluate(dev / 'cases.json', oracle, out / 'null-run.txt')
  rep['null_run_original_vs_original'] = {'G-O1': r['G-O1']['pass'], 'G-O2': r['G-O2']['pass'], 'max_rel': r['G-O2']['max_rel']}
  base = (dev / 'engine.txt').read_text().split('\n')
  first_scale = next(i for i, l in enumerate(base) if l.startswith('scale '))
  v = float(base[first_scale].split()[1])
  def variant(name, lines):
    path = out / (name + '.txt')
    path.write_text('\n'.join(lines))
    r = evaluate(dev / 'cases.json', oracle, path)
    rep[name] = {'G-O1': r['G-O1']['pass'], 'G-O2': r['G-O2']['pass'], 'max_rel': r['G-O2']['max_rel']}
  ulp = struct.unpack('<f', struct.pack('<I', struct.unpack('<I', struct.pack('<f', v))[0] + 1))[0]
  for name, nv in (('perturb-1ulp-scale', ulp), ('perturb-rel2e-5-scale', v * (1 + 2e-5)),
                   ('perturb-rel5e-6-scale', v * (1 + 5e-6))):
    lines = list(base)
    lines[first_scale] = 'scale %.9g' % nv
    variant(name, lines)
  vis = next(i for i, l in enumerate(base) if l.startswith('visible ') and len(l.split()) > 3)
  tok = base[vis].split()
  tok[2], tok[3] = tok[3], tok[2]
  lines = list(base)
  lines[vis] = ' '.join(tok)
  variant('swap-two-visible-indices', lines)
  for mdir in sorted(pathlib.Path(mutants).iterdir()):
    if (mdir / 'engine.txt').exists():
      r = evaluate(dev / 'cases.json', oracle, mdir / 'engine.txt')
      rep['code-mutant ' + mdir.name] = {'G-O1': r['G-O1']['pass'], 'G-O2': r['G-O2']['pass'], 'max_rel': r['G-O2']['max_rel']}
  (out / 'selftest.json').write_text(json.dumps(rep, indent=1) + '\n')
  print(json.dumps(rep, indent=1))


def main():
  mode = sys.argv[1]
  out = pathlib.Path(sys.argv[sys.argv.index('--out') + 1])
  if mode == 'selftest':
    arg = lambda k: pathlib.Path(sys.argv[sys.argv.index(k) + 1])
    selftest(out, arg('--dev'), arg('--oracle-repeat'), arg('--mutants'))
    return
  checks = []
  version = 1
  global REPLAY, WASM
  if '--replay' in sys.argv:
    REPLAY = pathlib.Path(sys.argv[sys.argv.index('--replay') + 1])
  if '--wasm' in sys.argv:
    WASM = pathlib.Path(sys.argv[sys.argv.index('--wasm') + 1])
  if mode == 'registered':
    version = gate_version(sys.argv[sys.argv.index('--gate') + 1] if '--gate' in sys.argv else '')
    seed, checks = check_registered(sys.argv[sys.argv.index('--designation') + 1], version)
    n = N_REGISTERED
  elif mode == 'dev':
    seed = int(sys.argv[sys.argv.index('--seed') + 1]) if '--seed' in sys.argv else 777
    n = int(sys.argv[sys.argv.index('--n') + 1]) if '--n' in sys.argv else 60
  else:
    sys.exit('mode: dev | registered')
  out.mkdir(parents=True, exist_ok=False)
  cases_json, cases_txt = out / 'cases.json', out / 'cases.txt'
  subprocess.run([sys.executable, '-I', str(HERE / 'make_cases.py'), str(cases_json), '--variant', 'random',
                  '--seed', str(seed), '--n', str(n), '--flat', str(cases_txt)], check=True)
  with open(out / 'oracle.json', 'w') as f:
    subprocess.run(['node', str(HERE / 'run_f994.mjs'), str(WASM), str(cases_json)], check=True, stdout=f)
  with open(out / 'engine.txt', 'w') as f, open(out / 'engine.stderr', 'w') as e:  # v2 (review R-8): keep stderr
    subprocess.run([str(REPLAY), str(cases_txt)], check=True, stdout=f, stderr=e)
  res = evaluate(cases_json, out / 'oracle.json', out / 'engine.txt')
  res.update({'gate': GATE_ID, 'version': version, 'mode': mode, 'n_cases': n, 'held_checks': checks,
              'pins': {'wasm': sha(WASM), 'replay': sha(REPLAY), 'make_cases': sha(HERE / 'make_cases.py'),
                       'run_f994': sha(HERE / 'run_f994.mjs'), 'compare': sha(HERE / 'compare.py'), 'runner': sha(pathlib.Path(__file__).resolve())}})
  if mode == 'dev':
    res['seed'] = seed
  (out / 'result.json').write_text(json.dumps(res, indent=1) + '\n')
  print(json.dumps({k: res[k] for k in ('G-O1', 'G-O2')}, default=str)[:600])
  sys.exit(0 if res['G-O1']['pass'] and res['G-O2']['pass'] else 1)


if __name__ == '__main__':
  main()
