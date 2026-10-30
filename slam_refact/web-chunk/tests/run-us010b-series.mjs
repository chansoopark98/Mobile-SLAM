// US-010b timing series (preregistration us010-samples.v3.md section 3.1): conditions S (chunk, engine on), Sx (S without the probe's per-update hitTest),
// N (engine call removed), F (fake chunk), interleaved (ABAB: odd rounds S F N Sx, even rounds reversed), 5 rounds, every run pinned to the P cores
// (`taskset -c 0-15`) with loadavg and per-CPU utilisation recorded. One Chromium at a time. Output:
// build/slam-refact-full-20261019/chunk/us010-20261020/series-v3/<name>/series.json and the runs under runs/threejs/<name>-r<round>-<cond>.
//   node slam_refact/web-chunk/tests/run-us010b-series.mjs --name t1 [--rounds 5] [--cpus 0-15]
import { spawn } from 'node:child_process';
import fs from 'node:fs/promises';
import { createHash } from 'node:crypto';
import os from 'node:os';
import path from 'node:path';
import { parseArgs } from 'node:util';

const repo = path.resolve(import.meta.dirname, '../../..');
const { values } = parseArgs({ options: { name: { type: 'string' }, rounds: { type: 'string', default: '5' }, cpus: { type: 'string', default: '0-15' },
  // v4: the evidence root (default: the v3 one), the directory of the series inside it, the engine, and the driver (this script is recorded by sha256)
  'evidence-root': { type: 'string', default: 'build/slam-refact-full-20261019/chunk/us010-20261020' }, 'series-dir': { type: 'string', default: 'series-v3' },
  'engine-dir': { type: 'string' }, expectations: { type: 'string', default: 'v3' } } });
if (!/^[a-z0-9]+$/.test(values.name || '')) throw new Error('--name [a-z0-9]+');
const evidence = path.resolve(repo, values['evidence-root']);
if (!evidence.startsWith(path.join(repo, 'build') + path.sep)) throw new Error('--evidence-root must be under build/');
const out = path.join(evidence, values['series-dir'], values.name);
// the children write their runs under <evidence>/runs and must stay under <evidence> (run-sample-smoke --owned-root); v3 used its defaults
const childArgs = ['--out-root', path.relative(repo, path.join(evidence, 'runs')), '--owned-root', path.relative(repo, evidence), '--expectations', values.expectations,
  ...(values['engine-dir'] ? ['--engine-dir', values['engine-dir']] : [])];
await fs.mkdir(path.dirname(out), { recursive: true });
await fs.mkdir(out, { recursive: false });
const cpuSet = (() => { const [a, b] = values.cpus.split('-').map(Number); return new Set(Array.from({ length: b - a + 1 }, (_, i) => a + i)); })();

const readStat = async () => {
  const rows = {};
  for (const line of (await fs.readFile('/proc/stat', 'utf8')).split('\n')) {
    const m = /^cpu(\d+) (.*)$/.exec(line);
    if (!m) continue;
    const v = m[2].split(' ').map(Number);
    rows[Number(m[1])] = { total: v.reduce((a, b) => a + b, 0), idle: v[3] + (v[4] || 0) };
  }
  return rows;
};
const busy = (a, b, pick) => {
  let t = 0, i = 0;
  for (const k of Object.keys(b).map(Number)) if (pick(k)) { t += b[k].total - a[k].total; i += b[k].idle - a[k].idle; }
  return t ? +(1 - i / t).toFixed(4) : null;
};
const ABL = 'slam_refact/web-chunk/tests/run-sample-ablation.mjs', SMK = 'slam_refact/web-chunk/tests/run-sample-smoke.mjs';
const extra = values['evidence-root'] === 'build/slam-refact-full-20261019/chunk/us010-20261020' && !values['engine-dir'] && values.expectations === 'v3' ? [] : childArgs;
const cond = {
  S: (l) => [SMK, '--sample', 'threejs', '--label', l, '--timing', '--no-tap', ...extra],
  Sx: (l) => [SMK, '--sample', 'threejs', '--label', l, '--timing', '--no-tap', '--probe-extra', 'hits=0', ...extra],
  N: (l) => [ABL, '--ablation', 'noengine', '--sample', 'threejs', '--label', l, '--timing', '--no-tap', ...extra],
  F: (l) => [SMK, '--sample', 'threejs', '--label', l, '--timing', '--no-tap', '--control', 'N0', ...extra],
};
const runs = [];
for (let r = 1; r <= Number(values.rounds); r++) {
  const order = r % 2 ? ['S', 'F', 'N', 'Sx'] : ['Sx', 'N', 'F', 'S'];
  for (const c of order) {
    const label = `${values.name}-r${r}-${c.toLowerCase()}`;
    const s0 = await readStat(), l0 = os.loadavg();
    const t0 = new Date().toISOString();
    const code = await new Promise((resolve) => {
      const p = spawn('taskset', ['-c', values.cpus, process.execPath, ...cond[c](label)], { cwd: repo, stdio: ['ignore', 'pipe', 'pipe'] });
      p.stdout.resume(); p.stderr.resume();
      p.on('close', resolve);
    });
    const s1 = await readStat(), l1 = os.loadavg();
    runs.push({ round: r, cond: c, label, exit: code, startedAt: t0, loadavg1Start: l0[0], loadavg1End: l1[0], pCoreBusy: busy(s0, s1, (k) => cpuSet.has(k)),
      eCoreBusy: busy(s0, s1, (k) => !cpuSet.has(k)), allBusy: busy(s0, s1, () => true) });
    console.log(JSON.stringify(runs.at(-1)));
  }
}
const selfSha = createHash('sha256').update(await fs.readFile(import.meta.filename)).digest('hex');
await fs.writeFile(path.join(out, 'series.json'), JSON.stringify({ schema: 'us010b-series-v1', name: values.name, cpus: values.cpus, nodeArgvHead: 'taskset -c', driver: { file: path.relative(repo, import.meta.filename), sha256: selfSha },
  evidenceRoot: path.relative(repo, evidence), runs }, null, 1) + '\n');
