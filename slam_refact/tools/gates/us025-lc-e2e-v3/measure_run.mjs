// us025-lc-e2e-gate.v3 measurement tool (task item 2, design 1): runs ONE original run (run_lcgold.mjs, same options and --poke as the gate) inside
// build/c1-chromium.lock and samples, once per second, what that run costs: process-tree CPU (utime+stime), runnable threads (the load-average contribution), RSS, thread count,
// per-core busy time, the machine load; and it records whether the Chrome profile directory, the temporary directory and the listening port are private to the run.
// DEV tool: writes only below build/slam-refact-pkgh-20261022/us025/lc-e2e-v3-dev/. Refuses to start when the 1-min load is above 8 (the gate's rule) or the lock is not free.
//   node measure_run.mjs --scenario flip6-n1 --label m01 --lc 1 --out <dir below lc-e2e-v3-dev> [--cpus 8-19] [--wait-s 600]
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import { spawn, spawnSync } from 'node:child_process';
import { parseArgs } from 'node:util';
import { RUN_OPTIONS_LC0, RUN_OPTIONS_LC1, RUNNER, LOAD_MAX, LOCK, RUN_WALL_MAX_S, RENDERER_CPU_MIN, findHungRenderers, parsePs, root } from './gate_lc.mjs';

const { values } = parseArgs({ options: { scenario: { type: 'string' }, label: { type: 'string' }, lc: { type: 'string', default: '1' }, out: { type: 'string' }, cpus: { type: 'string' }, 'wait-s': { type: 'string', default: '600' } } });
const DEV_ROOT = path.join(root, 'build/slam-refact-pkgh-20261022/us025/lc-e2e-v3-dev');
const out = path.resolve(root, values.out ?? '');
if (!(out + path.sep).startsWith(DEV_ROOT + path.sep)) throw new Error(`--out must be below ${DEV_ROOT}`);
if (!values.scenario || !/^[a-z0-9-]+$/.test(values.label ?? '')) throw new Error('--scenario and --label required');
const lc = Number(values.lc);
fs.mkdirSync(out, { recursive: true });
const TICKS = 100;   // CLK_TCK on Linux

const readStat = () => {
  const per = [];
  for (const line of fs.readFileSync('/proc/stat', 'utf8').split('\n')) {
    const m = /^cpu(\d+) (.*)$/.exec(line);
    if (!m) continue;
    const v = m[2].split(/\s+/).map(Number);
    per[Number(m[1])] = { busy: v[0] + v[1] + v[2] + v[5] + v[6] + (v[7] ?? 0), total: v.slice(0, 8).reduce((a, b) => a + b, 0) };
  }
  return per;
};
const psTree = (rootPid) => {
  const rows = spawnSync('ps', ['-eo', 'pid=,ppid=,args='], { encoding: 'utf8' }).stdout.split('\n').map((l) => l.trim().match(/^(\d+)\s+(\d+)\s+(.*)$/)).filter(Boolean).map((m) => ({ pid: +m[1], ppid: +m[2], args: m[3] }));
  const kids = new Map();
  for (const r of rows) { if (!kids.has(r.ppid)) kids.set(r.ppid, []); kids.get(r.ppid).push(r); }
  const tree = [];
  const walk = (pid) => { for (const k of kids.get(pid) ?? []) { tree.push(k); walk(k.pid); } };
  walk(rootPid);
  return tree;
};
const procCpu = (pid) => {
  try {
    const s = fs.readFileSync(`/proc/${pid}/stat`, 'utf8');
    const f = s.slice(s.lastIndexOf(')') + 2).split(' ');
    return { ticks: Number(f[11]) + Number(f[12]), threads: Number(f[17]), rssKb: Number(f[21]) * 4 };
  } catch { return null; }
};
const runnable = (pid) => {
  let n = 0;
  try {
    for (const t of fs.readdirSync(`/proc/${pid}/task`)) {
      try { const s = fs.readFileSync(`/proc/${pid}/task/${t}/stat`, 'utf8'); const st = s.slice(s.lastIndexOf(')') + 2, s.lastIndexOf(')') + 3); if (st === 'R' || st === 'D') n++; } catch { /* thread gone */ }
    }
  } catch { /* process gone */ }
  return n;
};
const tmpList = () => { try { return fs.readdirSync(os.tmpdir()).filter((n) => /playwright|chrome|\.org\.chromium|\.com\.google/.test(n)).sort(); } catch { return []; } };

// lock: hold build/c1-chromium.lock for the duration with a flock child whose stdin we close at the end
const lockProc = spawn('flock', ['-w', values['wait-s'], path.join(root, LOCK), 'bash', '-c', 'echo locked; cat >/dev/null'], { stdio: ['pipe', 'pipe', 'inherit'] });
await new Promise((resolve, reject) => {
  lockProc.stdout.once('data', resolve);
  lockProc.once('exit', (c) => reject(new Error(`could not take ${LOCK} (flock exit ${c})`)));
});
const release = () => { try { lockProc.stdin.end(); } catch { /* closed */ } };
try {
  const load0 = os.loadavg();
  if (load0[0] > LOAD_MAX) throw new Error(`load ${load0[0]} > ${LOAD_MAX}: not measuring`);
  const tmpBefore = tmpList();
  const opts = lc === 1 ? RUN_OPTIONS_LC1 : RUN_OPTIONS_LC0;
  const runnerArgs = [path.join(root, RUNNER), '--scenario', values.scenario, '--label', values.label, '--out-root', path.relative(root, path.join(out, 'runs')), ...opts, '--poke', JSON.stringify({ lc })];
  const cmd = values.cpus ? 'taskset' : process.execPath;
  const args = values.cpus ? ['-c', values.cpus, process.execPath, ...runnerArgs] : runnerArgs;
  const t0 = Date.now();
  const child = spawn(cmd, args, { cwd: root, stdio: ['ignore', fs.openSync(path.join(out, `${values.label}.runner.log`), 'wx'), 2], detached: true });
  let exited = null;
  child.on('close', (code, signal) => { exited = { code, signal }; });
  const samples = [];
  const seenUserDataDirs = new Set();
  const seenPorts = new Set();
  const seenChromeArgs = new Set();
  let prevStat = readStat();
  let prevTicks = 0;
  const lastTicks = new Map();
  let killedHung = [];
  while (!exited) {
    await new Promise((r) => setTimeout(r, 1000));
    if (!killedHung.length && (Date.now() - t0) / 1000 > RUN_WALL_MAX_S) {   // the gate's watchdog rule (renderer at >= 90 percent CPU inside THIS run's tree), measured runs must not hold the lock for ever
      killedHung = findHungRenderers(parsePs(spawnSync('ps', ['-eo', 'pid=,ppid=,etimes=,pcpu=,args='], { encoding: 'utf8' }).stdout), child.pid, RENDERER_CPU_MIN);
      for (const pid of killedHung) { try { process.kill(pid, 'SIGKILL'); } catch { /* gone */ } }
    }
    const tree = [{ pid: child.pid, args: 'runner' }, ...psTree(child.pid)];
    let ticks = 0; let thr = 0; let rss = 0; let run = 0;
    for (const p of tree) {
      const c = procCpu(p.pid);
      if (!c) { if (lastTicks.has(p.pid)) ticks += lastTicks.get(p.pid); continue; }
      lastTicks.set(p.pid, c.ticks);
      ticks += c.ticks; thr += c.threads; rss += c.rssKb; run += runnable(p.pid);
      const ud = /--user-data-dir=(\S+)/.exec(p.args ?? '');
      if (ud) seenUserDataDirs.add(ud[1]);
      if (/\/chrome\b|google-chrome/.test(p.args ?? '') && !/--type=/.test(p.args ?? '')) seenChromeArgs.add((p.args ?? '').slice(0, 400));
      for (const m of (p.args ?? '').matchAll(/--remote-debugging-(?:port|pipe)(=\S+)?/g)) seenChromeArgs.add(m[0]);
    }
    // finished processes keep their last ticks (children that exited between samples): use the max total seen
    const allTicks = [...lastTicks.values()].reduce((a, b) => a + b, 0);
    const st = readStat();
    const dBusy = st.map((c, i) => (c.busy - prevStat[i].busy) / Math.max(1, c.total - prevStat[i].total));
    prevStat = st;
    const ss = spawnSync('ss', ['-ltnpH'], { encoding: 'utf8' }).stdout.split('\n');
    for (const l of ss) { if (tree.some((p) => l.includes(`pid=${p.pid},`))) seenPorts.add(l.trim().split(/\s+/)[3]); }
    samples.push({ t: (Date.now() - t0) / 1000, load1: os.loadavg()[0], treeCoresNow: (allTicks - prevTicks) / TICKS, treeThreads: thr, treeRssMb: rss / 1024, treeRunnable: run,
      busyCoresTotal: dBusy.reduce((a, b) => a + b, 0), perCoreBusy: dBusy.map((x) => +x.toFixed(2)), procs: tree.length });
    prevTicks = allTicks;
  }
  const wall = (Date.now() - t0) / 1000;
  const treeCpuS = [...lastTicks.values()].reduce((a, b) => a + b, 0) / TICKS;
  const q = (arr, p) => { const s = [...arr].sort((a, b) => a - b); return s.length ? s[Math.min(s.length - 1, Math.floor(p * s.length))] : null; };
  const cores = samples.map((s) => s.treeCoresNow);
  const rn = samples.map((s) => s.treeRunnable);
  const usedCpus = (() => { const used = new Set(); for (const s of samples) s.perCoreBusy.forEach((b, i) => { if (b > 0.05) used.add(i); }); return [...used]; })();
  const meta = (() => { try { return JSON.parse(fs.readFileSync(path.join(out, 'runs', values.scenario, values.label, 'meta.json'), 'utf8')); } catch { return null; } })();
  const tmpAfter = tmpList();
  const summary = { scenario: values.scenario, label: values.label, lc, cpus: values.cpus ?? null, killedHungRenderers: killedHung, exited, status: meta?.status ?? null, wallSeconds: wall, treeCpuSeconds: treeCpuS, meanCores: treeCpuS / wall,
    treeCoresPerSecond: { p50: q(cores, 0.5), p95: q(cores, 0.95), max: Math.max(...cores) }, runnableThreads: { mean: rn.reduce((a, b) => a + b, 0) / rn.length, p50: q(rn, 0.5), p95: q(rn, 0.95), max: Math.max(...rn) },
    treeThreadsMax: Math.max(...samples.map((s) => s.treeThreads)), treeRssMbMax: Math.max(...samples.map((s) => s.treeRssMb)), procsMax: Math.max(...samples.map((s) => s.procs)),
    load1: { before: load0[0], max: Math.max(...samples.map((s) => s.load1)), after: os.loadavg()[0] }, busyCoresTotal: { mean: samples.reduce((a, s) => a + s.busyCoresTotal, 0) / samples.length, max: Math.max(...samples.map((s) => s.busyCoresTotal)) },
    cpusEverBusy: usedCpus.length, cpus: usedCpus, userDataDirs: [...seenUserDataDirs], listeningPorts: [...seenPorts], chromeArgs: [...seenChromeArgs], tmpDirsBefore: tmpBefore, tmpDirsAfter: tmpAfter,
    runnerReport: (() => { try { const r = JSON.parse(fs.readFileSync(path.join(out, 'runs', values.scenario, values.label, 'meta.json'), 'utf8')); return { origin: r.origin ?? null, browserLaunch: r.browserLaunch ?? null, wallSeconds: r.wallSeconds ?? null, loadAvgAtStart: r.loadAvgAtStart ?? null }; } catch { return null; } })() };
  fs.writeFileSync(path.join(out, `${values.label}.summary.json`), `${JSON.stringify(summary, null, 1)}\n`, { flag: 'wx' });
  fs.writeFileSync(path.join(out, `${values.label}.samples.json`), `${JSON.stringify(samples)}\n`, { flag: 'wx' });
  console.log(JSON.stringify(summary, null, 1));
} finally { release(); }
