// Headless smoke of the slam_refact xr-slam chunk under the deployed xr.js (spec docs/spec/07-xr-slam-chunk.md C7-2,
// C8-1, C8-2 (b)). Runs slam_refact/tools/re/runtime/run-chunk-ab.mjs with the refact chunk only (one headless
// Chromium, ephemeral 127.0.0.1 server, external requests blocked) and checks its meta.json.
//   node slam_refact/web-chunk/tests/run-chunk-smoke.mjs --label <new label> [--scenario flip6-s2] [--max-frames 300]
//     [--gpu-check 10,30] [--min-gpu-checks 15] [--engine-dir <dir>]   (default: the engine pinned for RM-4d)
import { spawnSync } from 'node:child_process';
import fs from 'node:fs';
import path from 'node:path';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values } = parseArgs({ options: {
  label: { type: 'string' }, scenario: { type: 'string', default: 'flip6-s2' }, 'max-frames': { type: 'string', default: '300' },
  'gpu-check': { type: 'string', default: '10,30' }, 'min-gpu-checks': { type: 'string', default: '15' },
  'engine-dir': { type: 'string', default: 'build/slam-refact-full-20261019/chunk/rm4d-20261020/engine-pinned' },
} });
if (!values.label) throw new Error('--label required (new label per attempt)');
const runner = path.join(repo, 'slam_refact/tools/re/runtime/run-chunk-ab.mjs');
const r = spawnSync(process.execPath, [runner, '--scenario', values.scenario, '--label', values.label, '--chunks', 'refact',
  '--max-frames', values['max-frames'], '--gpu-check', values['gpu-check'], '--engine-dir', values['engine-dir']], { cwd: repo, stdio: 'inherit' });
const meta = JSON.parse(fs.readFileSync(path.join(repo, 'build/slam-refact-full-20261019/chunk/runs', values.scenario, values.label, 'refact/meta.json'), 'utf8'));
const s = meta.summary ?? {}, d = meta.chunkDiag ?? {};
const lag = d.responseLag ?? {};
const checks = {
  runnerExit0: r.status === 0,
  status: meta.status === 'COMPLETED',
  chunkRequested: s.chunkRequested === true,
  engineMatchesManifest: d.engine?.matchesManifest === true,
  engineNotReplaced: d.engine?.glueReplaced === false,
  onStart: !!s.onStart,
  nonNullReality: (s.nonNullReality ?? 0) > 0,
  pageErrors0: s.pageErrors === 0,
  probeErrors0: s.errors === 0,
  external0: s.blockedExternal === 0,
  gpuCheckFrames: (d.gpuCheck?.frames ?? 0) >= Number(values['min-gpu-checks']),
  gpuCheckAllIdentical: d.gpuCheck?.frames > 0 && d.gpuCheck.swiftshaderApproxIdentical === d.gpuCheck.frames,
  responseLagAll1: Object.keys(lag).length === 1 && lag['1'] > 0,
};
console.log(JSON.stringify({ label: values.label, checks, gpuCheck: d.gpuCheck, responseLag: lag }, null, 1));
process.exitCode = Object.values(checks).every(Boolean) ? 0 : 1;
