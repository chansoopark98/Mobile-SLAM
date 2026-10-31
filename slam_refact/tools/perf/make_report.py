#!/usr/bin/env python3
"""Builds build/xr-perf-20261031/REPORT.json from raw/ (aggregate.py output + profile summaries + provenance + opportunities).
usage: make_report.py <perf dir>"""
import glob, hashlib, json, os, subprocess, sys
P = sys.argv[1]
R = os.path.join(P, 'raw')
subprocess.run([sys.executable, os.path.join(os.path.dirname(__file__), 'aggregate.py'), R, os.path.join(R, 'aggregate.json')], check=True)
rep = json.load(open(os.path.join(R, 'aggregate.json')))
sha = lambda f: hashlib.sha256(open(f, 'rb').read()).hexdigest()
repo = os.path.abspath(os.path.join(P, '..', '..'))
rep['label'] = 'DESKTOP numbers (i9-13900K, shared machine, other agents running); NOT phone numbers. Dataset/desktop replay results are not phone real-time performance (CLAUDE.md section 3).'
rep['provenance'] = {
    'source_head': open(os.path.join(R, 'source-head.txt')).read().strip(), 'engine_tree_diff_vs_head': open(os.path.join(R, 'source-diff.txt')).read().strip() or 'none (slam_refact/engine clean)',
    'native_binary_sha256': open(os.path.join(R, 'binary.sha256')).read().split()[0], 'native_build': 'cmake Release (-O3 -DNDEBUG, -march=x86-64, no fast-math, ffp-contract=off), Ninja -j4, build/xr-perf-20261031/native',
    'native_binary_equals_earlier_build': 'sha256 d44d66a1... identical to build/xr-drift-trace-20261030/native/harness/slam_refact_replay (VERIFIED by sha256)',
    'wasm_engine': {l.split()[1].split('/')[-1]: l.split()[0] for l in open(os.path.join(R, 'wasm-deployed.sha256'))},
    'wasm_note': 'DEPLOYED web/slam-refact-xr engine (no name section). A fresh WASM build was not made; deployed == HEAD source is UNVERIFIED.',
    'fixtures': 'build/xr-drift-trace-20261030/bundles/flip6-arcore-{s2,s3,l1}/k425 (ARCore-derived Flip6 recordings, 480x640, K=425.46; drop_mode ideal -> 0 dropped, pyramid model swiftshader-approx)',
    'node': subprocess.run(['node', '--version'], capture_output=True, text=True).stdout.strip(),
    'pinning': 'native + node-WASM runs: taskset -c 12 (one P-core, SMT sibling idle at start); browser: taskset -c 0-15. Runs were serial (no two measured runs overlapped). Two 1 kHz sampling-profile runs (cpu 6, first-pass) OVERLAPPED the WASM s2-r1/s3-r1 runs (cpu 12); the native and browser series and the 10 kHz profiles ran alone (serial).',
    'loadavg_at_start': {'wasm': [l.split('loadavg=')[1] for l in open(os.path.join(R, 'wasm-series.log')) if 'loadavg=' in l],
                         'native': [l.split('loadavg=')[1] for l in open(os.path.join(R, 'native-series.log')) if 'loadavg=' in l]},
    'warmup': 'first 3 processed frames of every run excluded (page faults / JIT); percentiles nearest-rank per run, then median over 3 repeats',
}
# parity baseline for any bit-identical optimization: sha256 of the pose/state columns of each native trace
base = {}
for d in sorted(glob.glob(os.path.join(R, 'native-*-r1'))):
    rows = [json.loads(l) for l in open(os.path.join(d, 'trace.jsonl'))]
    key = [[r['i'], r['fresh'], r['p'], r['q_wxyz'], r.get('worldPoints'), r.get('srKeyframes'), r.get('srPointsBySource')] for r in rows]
    base[os.path.basename(d)[7:-3]] = hashlib.sha256(json.dumps(key).encode()).hexdigest()
rep['parity_baseline_native_pose_state_sha256'] = {'columns': 'i,fresh,p,q_wxyz,worldPoints,srKeyframes,srPointsBySource (r1==r2==r3 bit-identical, cmp of p/q rows)', **base}
# profiles
prof = {}
for n in ('s2', 's3', 'l1'):
    d = json.load(open(os.path.join(R, 'profile', f'hf-{n}-profile.json')))
    k = next(x for x in d if x.startswith('without'))
    prof[n] = {'total_samples_10kHz_wall': d['all']['samples'], 'pyramid_build_inclusive_pct_of_all': next((r['pct'] for r in d['all']['inclusive'] if 'CpuGr8Pyramid::Build' in r['fn']), None),
               'non_pyramid_samples': d[k]['samples'], 'non_pyramid_pct_of_all': round(100 * d[k]['samples'] / d['all']['samples'], 2), 'non_pyramid_ms_per_frame_incl_harness': round(d[k]['samples'] / 10000 / 850 * 1000, 2),
               'non_pyramid_self_top25': d[k]['self'][:25], 'non_pyramid_inclusive_top40': d[k]['inclusive'][:40]}
rep['native_profile'] = {
    'method': 'perf(1) is not permitted (perf_event_paranoid=4, no sudo) -> LD_PRELOAD SIGPROF/hrtimer sampler (slam_refact/tools/perf/sigprof_sampler.c) on the UNMODIFIED production binary; 10 kHz CLOCK_MONOTONIC timer, glibc backtrace (24 frames), symbols by nm; '
              'samples inside the sampler dump are dropped. Sampling run wall 139.9 s vs 136.8 s unsampled (+2%). Inclusive = sample stack contains the function. Single-threaded, pinned to cpu 6. The non_pyramid set = samples whose stack has no CpuGr8Pyramid::Build and still includes harness work (trace write, PGM read).',
    'headline': 'CPU shader-port pyramid (CpuGr8Pyramid::Build and callees) is ~98.5% of native CPU time. It does not exist in the browser path (GPU pyramid + readback), so native replay frontend ms is NOT the product frontend cost.',
    'fixtures': prof}
rep['phone_evidence_existing'] = {
    'source': 'build/xr-camera-res-20261030/probe-results.jsonl (Galaxy Z Flip6 Chrome, deployed XR page, 15.2 s short stationary probe, accuracy not measured, pre/post-init not split, n~420 frames per resolution)',
    'camera_1440x1080_req': {'rafFps': 27.3, 'dropRatio': 0, 'stageMs': {'p50': 2.87, 'p95': 4.65, 'p99': 6.14}, 'readMs': {'p50': 6.8, 'p95': 9.26, 'p99': 11.3}, 'engineMs': {'p50': 8.77, 'p95': 11.13, 'p99': 12.6}},
    'camera_1920x1440_req': {'rafFps': 27.8, 'dropRatio': 0, 'stageMs': {'p50': 3.01, 'p95': 4.52, 'p99': 5.95}, 'readMs': {'p50': 6.79, 'p95': 9.23, 'p99': 10.52}, 'engineMs': {'p50': 8.19, 'p95': 10.77, 'p99': 13.13}},
    'ratio_phone_over_desktop_engineMs_p50': 'about 3.6-3.8x (8.2-8.8 vs 2.3 ms browser desktop); readMs about 17x (6.8 vs 0.4) - readback wait, not compute'}
rep['reference_8thwall_note'] = {
    'what': 'ORIGINAL 8th Wall SDK bytes (xr.js + xr-slam.js) run in headless Chromium + SwiftShader on this desktop (slam_refact/docs/re/03-runtime-reference.md section 7). xrCpuStageMs = XrController onProcessCpu wall time (processStagedFrame + decode), xrGpuStageMs = onProcessGpu (stageFrame). latencyMs = feed->callback.',
    'phone': 'NO 8th Wall timing on a phone exists in the repo (UNVERIFIED). "Similar speed to 8th Wall" cannot be judged on the phone from existing evidence; it needs the original SDK run on the Flip6 under the same page (user-gated measurement).',
    'load': 'reference runs started with loadavg 14.7 (flip6-s2 pmr1) - not load controlled'}
rep['observations'] = [
    'Ceres prints "Residual and Jacobian evaluation failed" (a 1-param-block(6) x 2-residual block with inf Jacobian, residual 1e-10, 3.4e-4) once per flip6-arcore-s3 run in BOTH native and node-WASM (3/3 runs each); not seen in s2/l1. Not investigated here (CLAUDE.md: no numeric patches); poses stay bit-identical across repeats.',
    'Per-run MAP_RESET count is 1 (frame 13 = map creation) in all 9 native and 9 WASM runs; keyframes at end: s2 27, s3 15, l1 16. All fixtures reach the first keyframe/world point at frame 13, so the pre-init sample is only 9 frames per run after warm-up (p95/p99 of pre-init are NOT meaningful).',
    'node-WASM engineWallMs - sum(stageMs) = marshal/copy overhead of processFramePyramid: p50 0.42 ms, p95 0.49 ms (desktop).',
]
rep['opportunities'] = json.load(open(os.path.join(P, 'opportunities.json')))
json.dump(rep, open(os.path.join(P, 'REPORT.json'), 'w'), indent=1)
print('wrote', os.path.join(P, 'REPORT.json'))
