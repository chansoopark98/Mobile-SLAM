// node --test slam_refact/web-chunk/tests/run-engine-worker-ab-options.test.mjs
// P4 options of run-engine-worker-ab.mjs without a browser: command line parsing, Chromium launch arguments per --gpu, the --fps resample, the
// idle rule (process classification, verdict, wait loop with a fake clock) and the renderer verdict.
import assert from 'node:assert/strict';
import test from 'node:test';
import { IDLE_RULE, busyProcessName, findBusyProcesses, idleVerdict, launchArgsFor, parseCli, rendererVerdict, resampleFrames, waitForIdle } from './run-engine-worker-ab.mjs';

const SMOKE = ['--smoke', '--label', 'x-1'];
const MEASURE = ['--preregistered', 'prereg.md', '--label', 'x-2'];

test('AB-O1: defaults keep the previous behaviour (swiftshader, recorded timestamps, idle check only for a measurement)', () => {
  const s = parseCli(SMOKE);
  assert.deepEqual([s.gpu, s.fps, s.idleCheck, s.m, s.n, s.replayFrames], ['swiftshader', null, false, 25, 80, 80]);
  const r = parseCli(MEASURE);
  assert.deepEqual([r.gpu, r.fps, r.idleCheck, r.m, r.n, r.replayFrames], ['swiftshader', null, true, 120, 270, 300]);
});

test('AB-O2: --gpu, --fps and --idle-check / --no-idle-check are parsed', () => {
  const a = parseCli([...SMOKE, '--gpu', 'hardware', '--fps', '8']);
  assert.deepEqual([a.gpu, a.fps, a.idleCheck], ['hardware', 8, false]);
  assert.equal(parseCli([...SMOKE, '--idle-check']).idleCheck, true);
  assert.equal(parseCli([...MEASURE, '--no-idle-check']).idleCheck, false);
  assert.equal(parseCli([...SMOKE, '--fps', '12.5']).fps, 12.5);
});

test('AB-O3: bad option values are refused', () => {
  for (const bad of [['--gpu', 'metal'], ['--gpu', ''], ['--fps', '0'], ['--fps', '-8'], ['--fps', 'abc'], ['--fps', '8fps'], ['--fps', '121'], ['--fps', '']]) {
    assert.throws(() => parseCli([...SMOKE, ...bad]), /--gpu|--fps/, JSON.stringify(bad));
  }
  assert.throws(() => parseCli(['--smoke']), /--label/);
  assert.throws(() => parseCli(['--label', 'x-3']), /--preregistered/);
  assert.throws(() => parseCli([...MEASURE, '--m', '119']), /m >= 120/);
  assert.throws(() => parseCli([...MEASURE, '--replay-frames', '299']), /300 frames/);
});

test('AB-O4: launch args: swiftshader is the previous list in the previous order; hardware has no SwiftShader and disables the software rasterizer', () => {
  assert.deepEqual(launchArgsFor('swiftshader'), ['--enable-unsafe-swiftshader', '--autoplay-policy=no-user-gesture-required', '--use-fake-ui-for-media-stream', '--use-fake-device-for-media-stream']);
  const hw = launchArgsFor('hardware');
  assert.ok(!hw.some((a) => /swiftshader/i.test(a)));
  for (const a of ['--use-gl=angle', '--use-angle=gl-egl', '--disable-software-rasterizer', '--enable-gpu', '--ignore-gpu-blocklist', '--use-fake-device-for-media-stream']) assert.ok(hw.includes(a), a);
  assert.throws(() => launchArgsFor('cpu'));
});

test('AB-O5: renderer verdict: hardware needs an NVIDIA renderer, software or missing strings are R1-INFRA, swiftshader only records', () => {
  assert.equal(rendererVerdict('hardware', 'ANGLE (NVIDIA Corporation, NVIDIA GeForce RTX 5090/PCIe/SSE2, OpenGL ES 3.2)').valid, true);
  assert.equal(rendererVerdict('hardware', 'ANGLE (NVIDIA, Vulkan 1.4.312 (NVIDIA NVIDIA GeForce RTX 5090 (0x00002B85)), NVIDIA)').valid, true);
  for (const bad of ['ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device (Subzero) (0x0000C0DE)), SwiftShader driver)', 'ANGLE (Intel, Mesa Intel(R) UHD Graphics)',
    'llvmpipe (LLVM 15)', 'ANGLE (NVIDIA, Vulkan 1.3 (SwiftShader), NVIDIA)', '', null, undefined]) {
    const v = rendererVerdict('hardware', bad);
    assert.equal(v.valid, false, String(bad));
    assert.ok(v.reason);
  }
  assert.equal(rendererVerdict('swiftshader', 'ANGLE (Google, Vulkan 1.3.0 (SwiftShader Device)').valid, true);
});

test('AB-O6: resample onto a regular grid: 125 ms slots at 8 fps, each takes the nearest recorded frame, offsets and repeats are reported', () => {
  // recorded at ~16 fps with jitter, plus a 0.9 s stall in the middle
  const times = [];
  let t = 0.013;
  for (let i = 0; i < 80; i++) { times.push(+t.toFixed(6)); t += (i === 40 ? 0.9 : 0.0625) + (i % 3 === 0 ? 0.004 : -0.002); }
  const { slots, report } = resampleFrames(times, 8, 40);
  assert.equal(slots.length, 40);
  assert.equal(slots[0].sourceIndex, 0);
  assert.equal(slots[0].t, times[0]);
  slots.slice(1).forEach((s, i) => { assert.ok(Math.abs((s.t - slots[i].t) * 1000 - 125) < 1e-3, 'interval ' + i); assert.ok(s.sourceIndex >= slots[i].sourceIndex, 'sources never go back'); });
  for (const s of slots) {
    const nearest = times.reduce((best, x, k) => (Math.abs(x - s.t) < Math.abs(times[best] - s.t) ? k : best), 0);
    assert.ok(Math.abs(times[s.sourceIndex] - s.t) <= Math.abs(times[nearest] - s.t) + 1e-12, 'nearest ' + s.slot);
    assert.equal(s.offsetMs, +((times[s.sourceIndex] - s.t) * 1000).toFixed(3));
  }
  assert.equal(report.intervalMs, 125);
  assert.equal(report.fps, 8);
  assert.ok(report.absOffsetMs.max >= report.absOffsetMs.median);
  assert.ok(report.repeatedSourceFrames >= 4, 'the stall repeats source frames');
  assert.equal(report.repeatedSourceFrames, 40 - new Set(slots.map((s) => s.sourceIndex)).size);
});

test('AB-O7: resample refuses a recording that is too short and bad arguments', () => {
  const times = Array.from({ length: 20 }, (_, i) => i * 0.0625);
  assert.throws(() => resampleFrames(times, 8, 40), /shorter/);
  assert.throws(() => resampleFrames(times, 0, 4));
  assert.throws(() => resampleFrames(times, 8, 0));
  assert.equal(resampleFrames(times, 8, 5).slots.at(-1).t, 0.5);
});

test('AB-O8: busy process classification: build tools, node --test, the replay tools; not look-alikes or shells that merely mention them', () => {
  const yes = { 'cmake --build build/dev-native --parallel 2': 'cmake', '/usr/bin/make -j4': 'make', 'ninja -C build': 'ninja', '/usr/bin/gmake': 'make', 'ctest --preset native-tests': 'ctest',
    'node --test slam_refact/web-chunk/tests/a.test.mjs': 'node --test', '/usr/bin/node --experimental-vm-modules --test x.mjs': 'node --test',
    'build/dev-native/drift_probe --scenario s2': 'drift_probe', 'python3 scripts/dev/drift_probe.py --x': 'drift_probe', 'build/x/slam_refact_replay in.json': 'slam_refact_replay',
    'bash scripts/slam_refact_replay.sh': 'slam_refact_replay' };
  for (const [args, name] of Object.entries(yes)) assert.equal(busyProcessName(args), name, args);
  const no = ['node run-engine-worker-ab.mjs --smoke --label a', 'node --check x.mjs', 'node tests/run.mjs --test', 'cmakelists-viewer', 'vim Makefile', 'grep -r ninja docs', 'python3 report.py drift_probe',
    "/bin/bash -c eval 'cmake --build build; ctest; node --test x' < /dev/null", '/bin/bash -c cmake', 'google-chrome --headless', 'ps -eo pid=,args=', ''];
  for (const args of no) assert.equal(busyProcessName(args), null, args);
});

test('AB-O9: findBusyProcesses parses ps output, skips this process and unreadable lines', () => {
  const ps = ['    1 /sbin/init', ' 4021 cmake --build build', ' 4022 /usr/bin/make -C build', ' 4023 node --test a.test.mjs', ' 9999 node --test self.test.mjs', 'garbage', ' 5000 bash -c ctest'].join('\n');
  const found = findBusyProcesses(ps, [9999]);
  assert.deepEqual(found.map((f) => [f.pid, f.name]), [[4021, 'cmake'], [4022, 'make'], [4023, 'node --test']]);
});

test('AB-O10: idle verdict: load1 <= 1.0 and no busy process', () => {
  assert.equal(idleVerdict({ load1: 1.0, busy: [] }).ok, true);
  assert.equal(idleVerdict({ load1: 0.2, busy: [] }).ok, true);
  assert.equal(idleVerdict({ load1: 1.01, busy: [] }).ok, false);
  assert.equal(idleVerdict({ load1: NaN, busy: [] }).ok, false);
  const v = idleVerdict({ load1: 0.1, busy: [{ pid: 7, name: 'ctest' }] });
  assert.equal(v.ok, false);
  assert.match(v.reasons[0], /ctest/);
  assert.deepEqual([IDLE_RULE.loadMax, IDLE_RULE.pollMs, IDLE_RULE.maxWaitMs], [1.0, 30000, 600000]);
});

// a fake clock: sleep advances it
const fake = (loadAt, busyAt = () => []) => {
  let clock = 5_000;
  const sleeps = [];
  return { sleeps, now: () => clock, sleep: async (ms) => { sleeps.push(ms); clock += ms; }, readLoad: () => loadAt(clock - 5_000), listBusy: async () => busyAt(clock - 5_000) };
};

test('AB-O11: idle wait: idle at once = one check, no wait', async () => {
  const f = fake(() => 0.3);
  const r = await waitForIdle(f);
  assert.deepEqual([r.ok, r.attempts, r.waitedMs, r.loadAtPass, f.sleeps.length], [true, 1, 0, 0.3, 0]);
  assert.deepEqual(r.found, []);
});

test('AB-O12: idle wait: re-checks every 30 s and passes once the load has dropped and the process has gone', async () => {
  const f = fake((ms) => (ms < 70_000 ? 2.4 : 0.8), (ms) => (ms < 100_000 ? [{ pid: 12, name: 'ctest', args: 'ctest' }] : []));
  const r = await waitForIdle(f);
  assert.equal(r.ok, true);
  assert.deepEqual(f.sleeps, [30000, 30000, 30000, 30000]);
  assert.equal(r.waitedMs, 120_000);
  assert.equal(r.attempts, 5);
  assert.deepEqual(r.found.map((b) => b.name), ['ctest']);
  assert.equal(r.checks[0].ok, false);
  assert.match(r.checks[0].reasons.join(), /load1 2.40/);
});

test('AB-O13: idle wait: gives up after 10 minutes (21 checks), the run is invalid', async () => {
  const f = fake(() => 3.0, () => [{ pid: 5, name: 'node --test', args: 'node --test' }, { pid: 6, name: 'make', args: 'make' }]);
  const r = await waitForIdle(f);
  assert.equal(r.ok, false);
  assert.equal(r.attempts, 21);
  assert.equal(r.waitedMs, 600_000);
  assert.equal(f.sleeps.length, 20);
  assert.equal(r.loadAtPass, null);
  assert.deepEqual(r.found.map((b) => b.pid).sort(), [5, 6]);
});
