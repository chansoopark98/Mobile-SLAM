// OR-1 mutants: defects injected into COPIES (build/ only; the repo chunk and the bundles are never edited). A gate that does not FAIL on
// them is not evidence. Three families:
//   chunk   a modified copy of the chunk files, played as arm C (what a wiring bug in xr-slam.js / sensors.js would do)
//   native  a modified copy of the bundle that only arm N reads (what the native side would see if its input differed)
//   rows    a one-ulp / one-bit change of a recorded output row (the resolution of the comparators themselves, unit-tested)
import fs from 'node:fs';
import path from 'node:path';

const CHUNK_FILES_RE = /\.js$/;

// [id, file, from, to, what it models]; `from` must occur exactly once in the file 
export const CHUNK_MUTANTS = [
  ['X0-noop-comment', 'xr-slam.js', 'let atlas = s.pyramid.read();', 'let atlas = s.pyramid.read();  // or1 no-op mutant', 'equivalent (must NOT be detected)'],
  ['X1-orientation-alpha-beta-swapped', 'xr-slam.js', 'Number(f.alpha), Number(f.beta), Number(f.gamma));', 'Number(f.beta), Number(f.alpha), Number(f.gamma));', 'Euler angle order bug'],
  ['X2-first-motion-event-lost', 'sensors.js', 'const staged = eventQueue.slice();', 'const staged = eventQueue.slice(eventQueue.length > 1 ? 1 : 0);', 'IMU event dropped at staging'],
  ['X2b-all-motion-events-lost', 'sensors.js', 'const staged = eventQueue.slice();', 'const staged = [];', 'no IMU reaches the engine'],
  ['X11-engine-time-is-stage-time', 'xr-slam.js', 'f.frameTime ? f.frameTime * 1e9 : f.timeNanos,', 'f.timeNanos,', 'CHK-1 reverted: the stage time instead of the func 647 time (the state before 2026-10-20)'],
  ['X3-frame-time-plus-1ns', 'xr-slam.js', 'f.frameTime ? f.frameTime * 1e9 : f.timeNanos,', '(f.frameTime ? f.frameTime * 1e9 : f.timeNanos) + 1,', 'one nanosecond of timing error in the engine time'],
  ['X4-video-time-plus-1ns', 'xr-slam.js', 'f.videoTime * 1e9, atlas', '(f.videoTime * 1e9) + 1, atlas', 'output-only time argument off by 1 ns'],
  ['X5-atlas-one-bit-flipped', 'xr-slam.js', 'let atlas = s.pyramid.read();', 'let atlas = s.pyramid.read(); atlas[(100 * 1024 + 100) * 4] ^= 1;', 'one bit of one pyramid pixel'],
  ['X6-answer-rotation-x-y-swapped', 'xr-slam.js', 'rotation: { x, y, z, w }, position:', 'rotation: { x: y, y: x, z, w }, position:', 'answer wiring only (the engine input and output are right)'],
  ['X7-interval-one-ulp', 'sensors.js', 'interval: e.interval,', 'interval: e.interval * (1 + 2.220446049250313e-16),', 'sub-resolution input change (a double ulp of the interval)'],
  ['X8-one-frame-not-staged', 'xr-slam.js', 'if (repeatFrame || cameraProcessingDisabled_) return;\n    maybeDispatchConfigChange(framework);',
    'if (repeatFrame || cameraProcessingDisabled_) return;\n    if (diag.stats.stagedFrames % 50 === 49 && !diag.or1Skipped) { diag.or1Skipped = true; return; }\n    maybeDispatchConfigChange(framework);',
    'one frame lost on the way to the engine (the 50th)'],
  ['X9-gyro-alpha-beta-swapped', 'sensors.js', 'rotationRate: e.rotationRate,', 'rotationRate: e.rotationRate && { alpha: e.rotationRate.beta, beta: e.rotationRate.alpha, gamma: e.rotationRate.gamma },', 'rotationRate fields mixed up'],
  ['X10-acceleration-fields-swapped', 'sensors.js', 'acceleration: e.acceleration,\n    accelerationIncludingGravity: e.accelerationIncludingGravity,',
    'acceleration: e.accelerationIncludingGravity,\n    accelerationIncludingGravity: e.acceleration,', 'accelerometer / linear acceleration mixed up'],
];

function copyAndPatch(id, srcDir, outDir, [, file, from, to]) {
  const dir = path.join(outDir, id, 'chunk');
  fs.mkdirSync(dir, { recursive: true });
  for (const f of fs.readdirSync(srcDir).filter((x) => CHUNK_FILES_RE.test(x))) fs.copyFileSync(path.join(srcDir, f), path.join(dir, f));
  const text = fs.readFileSync(path.join(dir, file), 'utf8');
  const n = text.split(from).length - 1;
  if (n !== 1) throw new Error(`${id}: pattern occurs ${n} times in ${file}`);
  fs.writeFileSync(path.join(dir, file), text.replace(from, () => to));
  return dir;
}
export function makeChunkMutant(id, srcDir, outDir) {
  const m = CHUNK_MUTANTS.find((x) => x[0] === id);
  if (!m) throw new Error(`unknown chunk mutant ${id}`);
  return copyAndPatch(id, srcDir, outDir, m);
}
// Mutants of the bundle that only the native arm reads. `at` = a sensor row / frame in the middle of the run.
export const NATIVE_MUTANTS = [
  ['N1-one-accelerometer-sample-plus-1e-3', 'one accelerometer component of one sensor row changed by 1e-3 m/s^2'],
  ['N2-one-sensor-row-missing', 'one sensor row removed'],
  ['N3-one-frame-time-plus-1us', 'the engine time of one input frame (column 9 when recorded, else column 3) moved by 1000 ns'],
];
export function makeNativeMutant(id, bundleDir, outDir) {
  const dir = path.join(outDir, id, 'bundle');
  fs.mkdirSync(dir, { recursive: true });
  for (const f of ['engine-config.tsv', 'sensors.tsv', 'frames.tsv', 'or1-derived.json']) fs.copyFileSync(path.join(bundleDir, f), path.join(dir, f));
  const lines = (f) => fs.readFileSync(path.join(dir, f), 'utf8').split('\n').filter((l) => l).map((l) => l.split('\t'));
  const write = (f, rows) => fs.writeFileSync(path.join(dir, f), rows.map((r) => `${r.join('\t')}\n`).join(''));
  const sensors = lines('sensors.tsv'), frames = lines('frames.tsv');
  const at = Math.floor(sensors.length / 2);
  // a processed frame in the middle whose successor is more than 2000 ns later (drop-masked rows can sit 1 ns after their predecessor)
  let fat = Math.floor(frames.length / 2);
  while (fat < frames.length - 1 && !(frames[fat][5] === '1' && Number(frames[fat + 1][2]) - Number(frames[fat][2]) > 2000)) fat++;
  if (id === 'N1-one-accelerometer-sample-plus-1e-3') { sensors[at][3] = String(Number(sensors[at][3]) + 1e-3); write('sensors.tsv', sensors); }
  else if (id === 'N2-one-sensor-row-missing') { sensors.splice(at, 1); write('sensors.tsv', sensors); }
  else if (id === 'N3-one-frame-time-plus-1us') { const col = frames[fat].length > 8 && Number(frames[fat][8]) !== 0 ? 8 : 2;  // the engine time is column 9 (func 647) when it is recorded
    frames[fat][col] = String(Number(frames[fat][col]) + 1000); write('frames.tsv', frames); }
  else throw new Error(`unknown native mutant ${id}`);
  return dir;
}
