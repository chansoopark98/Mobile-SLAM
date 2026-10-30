#!/usr/bin/env python3
"""Driver for the ORIGINAL-SDK discriminating probe (it-scale-v4 L0 diagnosis).  NOTHING runs on import; --dry-run prints every command.
**Launches headless Chromium (stage orig) -- run only after the coordinator's go (the browser lane is shared; the stage takes
flock build/c1-chromium.lock like e5_pipeline.py).**

  orig_itprobe_driver.py --seq <E5 sequence dir> --out <NEW dir under build/slam-refact-full-20261019/it-scale-v4/diag> [--k 3] [--max-frames 0]
                         [--stages prep,orig,bundle,engine,analyze] [--arms 0] [--seed-frames 123] [--dry-run]

stages:
  prep     build the instrumented wasm (wasm2wat -> instrument_it_wat.py -> wat2wasm), run the Node unit checks (must pass), record sha256s
  orig     K original-SDK runs of the sequence with run_original_itprobe.mjs (--memprobe --record-inputs --memprobe-detail 12 as e5_pipeline.py, plus
           --it-probe-wasm); output <out>/runs/<e5-seq>/oK/{trace.jsonl,image-targets.json,memprobe.jsonl.gz,itprobe.jsonl.gz,...}
  bundle   exact-input engine bundle per original (prepare_bundle.py + make_variant.py, same flags as e5_pipeline.py)
  engine   engine replay per original and arm with sr_e5_diag (ITPROBE_OUT, ITPROBE_RNG_OUT), jobs <= 2
  analyze  orig_itprobe_analysis.py compare per original (first RNG-state / hom-call / found difference); seeded replays: for each --seed-frames F the engine
           is started with the ORIGINAL's RNG state of frame F (ITPROBE_RNG_SET) and stops at F+6; the report lists orig vs seeded engine decision at F
"""
import argparse, json, os, pathlib, subprocess, sys, hashlib

HERE = pathlib.Path(__file__).resolve().parent
E5 = HERE.parent
ROOT = HERE.parents[4]
DIAG_BUILD = ROOT / 'build/slam-refact-full-20261019/it-scale-v4/diag'
WABT = pathlib.Path.home() / '.local/opt/rea-tools/bin'
ORIG_WASM = ROOT / 'build/slam-refact-20261015/inputs/xr-slam.wasm'
ORIG_SHA = '780731fe294a725acf1b7717ed5ffb07ad9f0929e408a1d74f47450c291ec422'
RUN_PY = [sys.executable, '-I']


def sh(cmd, a, env=None, log=None, check=True):
    print('$ ' + ' '.join(map(str, cmd)))
    if a.dry_run:
        return 0
    r = subprocess.run(cmd, env=env, capture_output=True, text=True)
    if log:
        pathlib.Path(log).write_text('$ ' + ' '.join(map(str, cmd)) + '\n' + r.stdout + '\n--- stderr\n' + r.stderr + f'\nrc={r.returncode}\n')
    if check and r.returncode != 0:
        sys.exit(f'FAILED rc={r.returncode}: {cmd[:4]} (see log)\n{r.stderr[-800:]}')
    return r.returncode


def sha(p):
    return hashlib.sha256(pathlib.Path(p).read_bytes()).hexdigest()


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--seq', required=True); ap.add_argument('--out', required=True)
    ap.add_argument('--k', type=int, default=3); ap.add_argument('--max-frames', default='0')
    ap.add_argument('--stages', default='prep,orig,bundle,engine,analyze'); ap.add_argument('--arms', default='0')
    ap.add_argument('--orig-args', default='', help='extra arguments for run_original_itprobe.mjs (e.g. "--gl-capture 70:1024")'); ap.add_argument('--no-probe-wasm', action='store_true', help='run the unmodified original (no --it-probe-wasm)')
    ap.add_argument('--seed-frames', default=''); ap.add_argument('--dry-run', action='store_true'); ap.add_argument('--no-lock', action='store_true')
    a = ap.parse_args()
    stages = a.stages.split(',')
    seq, out = pathlib.Path(a.seq).resolve(), pathlib.Path(a.out).resolve()
    if DIAG_BUILD not in out.parents and out != DIAG_BUILD:
        sys.exit('--out must be under ' + str(DIAG_BUILD))
    name = 'e5-' + seq.name
    wd = out / 'wasm'
    probe_wasm = wd / 'xr-it.wasm'
    runs, bundles, eng, logs, ana = out / 'runs', out / 'bundles', out / 'eng', out / 'logs', out / 'analysis'
    if not a.dry_run:
        if 'prep' in stages and out.exists():
            sys.exit('refusing existing ' + str(out))
        for d in (out, wd, runs, bundles, eng, logs, ana):
            d.mkdir(parents=True, exist_ok=True)
    if sha(ORIG_WASM) != ORIG_SHA:
        sys.exit('original wasm sha mismatch')
    scen = out / 'scenarios.json'
    if not a.dry_run:
        scen.write_text(json.dumps({name: {'kind': 'tum', 'dataset': str(seq.relative_to(ROOT)), 'ua': 'reduced', 'scale': 'responsive', 'note': 'E5 synthetic room (package H)'}}))
    env = dict(os.environ, E5_SCENARIOS=str(scen))
    lock = [] if a.no_lock else ['flock', '-w', '28800', str(ROOT / 'build/c1-chromium.lock')]
    if 'prep' in stages:
        sh([str(WABT / 'wasm2wat'), str(ORIG_WASM), '-o', str(wd / 'xr.wat')], a)
        sh(RUN_PY + [str(HERE / 'instrument_it_wat.py'), str(wd / 'xr.wat'), str(wd / 'xr-it.wat')], a)
        sh(RUN_PY + [str(HERE / 'instrument_it_wat.py'), str(wd / 'xr.wat'), str(wd / 'xr-it-test.wat'), '--test-exports'], a)
        sh([str(WABT / 'wat2wasm'), str(wd / 'xr-it.wat'), '-o', str(probe_wasm)], a)
        sh([str(WABT / 'wat2wasm'), str(wd / 'xr-it-test.wat'), '-o', str(wd / 'xr-it-test.wasm')], a)
        sh(['node', str(HERE / 'unit_check_itprobe.mjs'), str(wd / 'xr-it-test.wasm')], a)
        sh(['node', str(HERE / 'unit_check_itprobe_js.mjs'), str(wd / 'xr-it-test.wasm')], a)
        sh(RUN_PY + [str(HERE / 'test_orig_itprobe_analysis.py')], a)
        if not a.dry_run:
            (out / 'prep.json').write_text(json.dumps({'original_wasm_sha256': ORIG_SHA, 'probe_wasm_sha256': sha(probe_wasm),
                                                       'instrumenter_sha256': sha(HERE / 'instrument_it_wat.py'), 'reader_sha256': sha(HERE / 'itprobe_orig.js')}, indent=1))
    if 'orig' in stages:
        for k in range(1, a.k + 1):
            cmd = lock + ['timeout', '1800', 'node', str(HERE / 'run_original_itprobe.mjs'), '--scenario', name, '--label', f'o{k}', '--out-root', str(runs),
                          '--image-targets', str(seq / 'target'), '--memprobe', '--record-inputs', '--memprobe-detail', '12'] + ([] if a.no_probe_wasm else ['--it-probe-wasm', str(probe_wasm.relative_to(ROOT))]) + a.orig_args.split()
            if a.max_frames != '0':
                cmd += ['--max-frames', a.max_frames]
            sh(cmd, a, env, logs / f'orig-o{k}.log')
    if 'bundle' in stages:
        sensors = runs / name / ('engine-input-sensors.jsonl.gz' if a.max_frames == '0' else f'engine-input-sensors-first{a.max_frames}.jsonl.gz')
        for k in range(1, a.k + 1):
            sh(RUN_PY + [str(E5 / 'prepare_bundle.py'), '--scenarios-file', str(scen), '--runs-dir', str(runs), '--bundles-dir', str(bundles), '--scenario', name, '--label', f'base-o{k}',
                         '--sensors', str(sensors), '--drop-mask-from', str(runs / name / f'o{k}' / 'trace.jsonl')] + (['--max-frames', a.max_frames] if a.max_frames != '0' else []), a, None, logs / f'bundle-base-o{k}.log')
            sh(RUN_PY + [str(ROOT / 'slam_refact/tools/harness/divergence/make_variant.py'), '--base', str(bundles / name / f'base-o{k}'), '--out', str(bundles / name / f'ex-o{k}'),
                         '--capture', str(runs / name / f'o{k}'), '--capture-binding', '--capture-wallclock', '--queue-times', '--capture-id-seed'], a, None, logs / f'bundle-ex-o{k}.log')
    engine = DIAG_BUILD / 'sr_e5_diag'

    def replay(label, k, arm, extra_env=None, stop=None):
        e = dict(os.environ, ITPROBE_OUT=str(eng / f'{label}.itprobe.jsonl'), ITPROBE_RNG_OUT=str(eng / f'{label}.rng'))
        if stop:
            e['ITPROBE_STOP_AFTER'] = str(stop)
        e.update(extra_env or {})
        sh(['nice', '-n', '10', str(engine), '--bundle', str(bundles / name / f'ex-o{k}'), '--out', str(eng / label), '--targets', str(seq / 'target'), '--world-tracker', str(arm)], a, e, logs / f'eng-{label}.log')
    if 'engine' in stages:
        for k in range(1, a.k + 1):
            for arm in [int(v) for v in a.arms.split(',') if v]:
                replay(f'o{k}-wt{arm}', k, arm)           # sequential: jobs <= 2 rule (one replay at a time is enough here)
    if 'analyze' in stages:
        for k in range(1, a.k + 1):
            orig = runs / name / f'o{k}' / 'itprobe.jsonl.gz'
            for arm in [int(v) for v in a.arms.split(',') if v]:
                sh(RUN_PY + [str(HERE / 'orig_itprobe_analysis.py'), 'compare', '--orig', str(orig), '--engine-probe', str(eng / f'o{k}-wt{arm}.itprobe.jsonl'),
                             '--engine-rng', str(eng / f'o{k}-wt{arm}.rng'), '--frames', '1-150', '--json', str(ana / f'o{k}-wt{arm}.compare.json')], a, None, logs / f'ana-o{k}-wt{arm}.log')
            for f in [int(v) for v in a.seed_frames.split(',') if v]:
                st = ana / f'o{k}-state-f{f}.txt'
                sh(RUN_PY + [str(HERE / 'orig_itprobe_analysis.py'), 'seed', '--orig', str(orig), '--frame', str(f), '--out', str(st)], a, None, logs / f'seed-o{k}-f{f}.log')
                replay(f'o{k}-seeded-f{f}', k, 0, {'ITPROBE_RNG_SET': f'{f}:{st}'}, stop=f + 6)


if __name__ == '__main__':
    main()
