// OR-1 command line (full-parity plan r2 OR-1; spec 07 section 12). Node only: no browser, no GPU, no network, no server.
//   node slam_refact/web-chunk/tools/or1/or1.mjs derive   --src <bundle dir> --out <new dir under build/> [--frames N]
//   node slam_refact/web-chunk/tools/or1/or1.mjs run      --bundle <derived dir> --out <new dir under build/> --engine-dir <dir> --native-bin <path>
//                                                         [--chunk-dir <dir>] [--arms WCN] [--reps R] [--native-bundle <dir>] [--kind dev|gate]
//                                                         GATE ARGUMENTS (all four, every run): --prereg-dir <dir> --gate-id <id> --gate-version <N> --designation-env <NAME>
//                                                         --run-log <new file under build/, outside --out>   (required for --kind gate)
//   node slam_refact/web-chunk/tools/or1/or1.mjs precheck (the arguments of a gate run) -> every HELD check, no marker, nothing written, nothing run
//   node slam_refact/web-chunk/tools/or1/or1.mjs verdict  --reports <report.json>,<report.json>,... --manifest <manifest.json>
//                                                         -> the gate over its bundles (C4 record rule, C9 asymmetry); exit 0 PASS, 1 FAIL, 4 INSUFFICIENT / UNEVALUATED
//   node slam_refact/web-chunk/tools/or1/or1.mjs audit    --prereg-dir <dir> --gate-id <id> --gate-version <N>   (after the coordinator recorded first_real_run_at)
//   node slam_refact/web-chunk/tools/or1/or1.mjs pin      --manifest <manifest.json> -> the values a designation file pins (gate_md_sha256, manifest_before_review_sha256)
//   node slam_refact/web-chunk/tools/or1/or1.mjs selftest --out <new dir under build/>
// The gate (id, version, preregistration directory) and the NAME of the environment variable that holds the designation sha256 are arguments; nothing about
// a version is a constant of this tool (v2 pinned them, review V-1 / V-8). `run --kind gate` is held (hold.mjs, the shared sr_held device) until the
// coordinator's designation exists; without it the run exits with "HELD: ..." (exit 3) before anything is created. A run that is not a gate run refuses
// the registered gate data.
import fs from 'node:fs';
import path from 'node:path';
import { parseArgs } from 'node:util';
import { fileURLToPath } from 'node:url';
import { readBundle, derive, inputFiles } from './bundle.mjs';
import { runAll, evaluate, summarize, aggregateGate } from './run.mjs';
import { writeRunOutputs } from './outputs.mjs';
import { sha256File, checkHold, claimFirstRun, assertNotGateData, fileSetSha256, designationPins, registeredFiles } from './hold.mjs';
import { SR_HELD_FILE, gateArgs, gateMdPath, manifestPath, sr } from '../gate-common/held.mjs';
import { RunLog, envRecord } from '../gate-common/runlog.mjs';
import { copyInto, makeReadOnly, readOnlyFiles, writeSums } from '../gate-common/freeze.mjs';

const here = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(here, '../../../..');
const underBuild = (p) => {
  const abs = path.resolve(root, p);
  if (!abs.startsWith(path.join(root, 'build') + path.sep)) throw new Error(`${p}: output must be under build/`);
  return abs;
};
const rel = (p) => path.relative(root, path.resolve(root, p));

export function runMeta({ ctx, values, derivedMeta, atlasSetSha256, gate = null, hold = null, now = new Date() }) {
  const files = (dir, list) => Object.fromEntries(list.map((f) => [`${dir}/${f}`, sha256File(path.join(root, dir, f))]));
  const chunkDir = rel(ctx.chunkDir);
  const gateCommonDir = path.join(here, '../gate-common');
  const md = gate && fs.existsSync(gateMdPath(gate)) ? gateMdPath(gate) : null;
  let gateMeta = null;
  if (gate) {
    gateMeta = { id: gate.gateId, version: gate.version, preregDir: rel(gate.preregDir), designationEnvName: gate.designationEnv,
      gateMd: md ? { file: rel(md), sha256: sha256File(md) } : null, srHeld: { file: rel(SR_HELD_FILE), sha256: sha256File(SR_HELD_FILE) } };
    if (hold) {
      const reg = registeredFiles(gate, hold.manifest);
      gateMeta.manifest = { file: rel(reg.manifest), sha256: sha256File(reg.manifest), beforeReviewSha256: designationPins(reg.manifest).manifest_before_review_sha256 };
      gateMeta.designation = { file: rel(reg.designation), sha256: sha256File(reg.designation) };
      gateMeta.review = { file: rel(reg.review), sha256: sha256File(reg.review) };
      gateMeta.dataReuse = hold.manifest.data_reuse ?? null;  // the registered data was already observed by a previous version (C12 item 6)
      gateMeta.datum = hold.datum.name;
      gateMeta.record = hold.datum.record ?? null;
    }
  }
  return {
    schema: 'or1-run-meta-v2', at: now.toISOString(), kind: values.kind ?? 'dev', command: process.argv.slice(1).join(' '), gate: gateMeta,
    arms: ctx.arms.join(''), reps: ctx.reps, node: process.version,
    bundle: { dir: rel(ctx.bundleDir), nativeBundleDir: ctx.nativeBundleDir ? rel(ctx.nativeBundleDir) : null, derived: derivedMeta, atlasSetSha256 },
    engine: { dir: rel(ctx.engineDir), ...files(rel(ctx.engineDir), ['slam_refact_engine.js', 'slam_refact_engine.wasm', 'manifest.json']) },
    nativeBin: { file: ctx.nativeBin, sha256: sha256File(path.join(root, ctx.nativeBin)) },
    chunk: files(chunkDir, fs.readdirSync(ctx.chunkDir).filter((f) => f.endsWith('.js')).sort()),
    harness: files('slam_refact/web-chunk/tools/or1', fs.readdirSync(here).filter((f) => /\.(mjs|sh)$/.test(f)).sort()),
    gateCommon: files('slam_refact/web-chunk/tools/gate-common', fs.readdirSync(gateCommonDir).filter((f) => f.endsWith('.mjs')).sort()),
    spec07Sha256: sha256File(path.join(root, 'slam_refact/docs/spec/07-xr-slam-chunk.md')),
    // v2 recorded the sha of a hardcoded v1 gate file here (review V-8); the sha of the file the run was registered with is in `gate.gateMd`
    gateMdSha256: md ? sha256File(md) : null,
  };
}

// everything a run or a precheck needs from the arguments
function prepare(values) {
  const gate = gateArgs(values, root);
  const bundleDir = path.resolve(root, values.bundle);
  const derivedMeta = JSON.parse(fs.readFileSync(path.join(bundleDir, 'or1-derived.json'), 'utf8'));
  const bundle = readBundle(bundleDir);
  const atlasSetSha256 = fileSetSha256(root, inputFiles(bundle, root).map((f) => path.relative(root, f)));
  const kind = values.kind ?? 'dev';
  const engineDir = path.resolve(root, values['engine-dir']);
  const ctx = { bundleDir, nativeBundleDir: values['native-bundle'] ? path.resolve(root, values['native-bundle']) : null, engineDir, nativeBin: values['native-bin'],
    chunkDir: path.resolve(root, values['chunk-dir'] ?? 'slam_refact/web-chunk'), repoRoot: root, reps: Number(values.reps ?? 1), arms: [...(values.arms ?? 'WCN')] };
  const holdValues = { kind, chunkDir: values['chunk-dir'], engineDir: values['engine-dir'], nativeBin: values['native-bin'], nativeBundle: values['native-bundle'],
    arms: values.arms ?? 'WCN', reps: ctx.reps, atlasSetSha256, bundle: values.bundle };
  return { gate, derivedMeta, atlasSetSha256, kind, ctx, holdValues };
}

async function cmdRun(values) {
  const { gate, derivedMeta, atlasSetSha256, kind, ctx, holdValues } = prepare(values);
  const outDir = underBuild(values.out);
  ctx.outDir = outDir;
  let log = null;
  if (values['run-log'] || kind === 'gate') {
    if (!values['run-log']) throw new Error('--run-log <new file> is required for --kind gate (outside --out)');
    log = new RunLog(underBuild(values['run-log']), { out: outDir });
    log.write('start', { argv: process.argv.slice(2), node: process.version, kind, gate: { id: gate.gateId, version: gate.version, preregDir: rel(gate.preregDir), designationEnvName: gate.designationEnv },
      env: envRecord(process.env, [gate.designationEnv]) });
  }
  let hold = null;
  try {
    if (kind === 'gate') {
      try { hold = checkHold({ root, gate, values: holdValues, derivedMeta }); } catch (error) { log.write('held', { message: error.message }); throw error; }
      log.write('hold-checks', { lines: hold.log });
      if (fs.existsSync(outDir)) throw new Error(`refusing to overwrite ${rel(outDir)} (checked before the marker so that a wrong --out does not use the first run)`);
      const marker = claimFirstRun({ hold });
      log.write('first-run-claimed', { marker: rel(marker), datum: hold.datum.name, designationSha256: hold.designationSha, lines: hold.log.slice(-1) });
    } else {
      assertNotGateData({ root, gate, derivedMeta, bundleDir: values.bundle });
    }
    const { runs } = await runAll(ctx);
    const g = evaluate({ runs, ctx });
    const meta = runMeta({ ctx, values: { kind }, derivedMeta, atlasSetSha256, gate, hold });
    const report = writeRunOutputs({ outDir, runs, g, meta });
    let frozen = null;
    if (kind === 'gate') {
      const reg = registeredFiles(gate, hold.manifest);
      const copies = copyInto(outDir, 'registered', [reg.gateMd, reg.manifest, reg.designation, reg.review, hold.outcome.marker]);
      const sums = writeSums(outDir);
      const entries = makeReadOnly(outDir);
      frozen = { copies, sums, readOnlyEntries: entries, originalsReadOnly: readOnlyFiles([reg.gateMd, reg.designation, reg.review]) };
    }
    console.log(summarize(g));
    console.log(`report: ${rel(path.join(outDir, 'report.json'))}`);
    if (log) log.write('finished', { report: rel(path.join(outDir, 'report.json')), verdict: g.verdict, C4: g.C4, frozen });
    log?.close('OK');
    return { g, report };
  } catch (error) {
    if (log && !log.closed) { log.write('error', { message: String(error?.message ?? error) }); log.close(String(error?.message ?? error).startsWith('HELD:') ? 'HELD' : 'ERROR'); }
    throw error;
  }
}

// every HELD check of a gate run, nothing created (the dry-run of the registered command)
function cmdPrecheck(values) {
  const { gate, derivedMeta, holdValues } = prepare({ ...values, kind: 'gate' });
  const hold = checkHold({ root, gate, values: holdValues, derivedMeta });
  console.log(`${hold.log.join('\n')}\nRELEASED (precheck: no marker created, no arm run) datum ${hold.datum.name}`);
}

function cmdVerdict(values) {
  const manifest = JSON.parse(fs.readFileSync(path.resolve(root, values.manifest), 'utf8'));
  const records = Object.fromEntries((manifest.pinned?.gate_data ?? []).map((d) => [d.name, d.record]));
  const bundles = values.reports.split(',').filter(Boolean).map((f) => {
    const r = JSON.parse(fs.readFileSync(path.resolve(root, f), 'utf8'));
    const name = r.meta?.gate?.datum;
    if (!name || !(name in records)) throw new Error(`${f}: not a report of a registered datum of this manifest (meta.gate.datum ${name})`);
    return { name, record: records[name], g: r.gates };
  });
  const missing = Object.keys(records).filter((n) => !bundles.some((b) => b.name === n));
  const agg = aggregateGate(bundles);
  if (missing.length) { agg.verdict = 'UNEVALUATED'; agg.unevaluated.push(...missing); }
  console.log(JSON.stringify(agg, null, 1));
  process.exitCode = agg.verdict === 'PASS' ? 0 : agg.verdict === 'FAIL' ? 1 : 4;
}

function cmdAudit(values) {
  const gate = gateArgs({ ...values, 'designation-env': 'UNUSED_BY_AUDIT' }, root);
  const manifest = JSON.parse(fs.readFileSync(manifestPath(gate), 'utf8'));
  let bad = 0;
  for (const kind of manifest.held?.kinds ?? []) {
    const o = sr.audit({ preregDir: gate.preregDir, gateId: gate.gateId, version: gate.version, kind });
    console.log(`${kind}: ${o.ok ? 'AUDIT OK' : `AUDIT FAILED [${o.code}] ${o.why}`}`);
    if (!o.ok) bad++;
  }
  process.exitCode = bad ? 3 : 0;
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === fileURLToPath(import.meta.url);
if (isMain) {
  const [cmd, ...rest] = process.argv.slice(2);
  const { values } = parseArgs({ args: rest, options: { src: { type: 'string' }, out: { type: 'string' }, frames: { type: 'string' }, bundle: { type: 'string' },
    'engine-dir': { type: 'string' }, 'native-bin': { type: 'string' }, 'chunk-dir': { type: 'string' }, arms: { type: 'string' }, reps: { type: 'string' },
    'native-bundle': { type: 'string' }, kind: { type: 'string' }, 'prereg-dir': { type: 'string' }, 'gate-id': { type: 'string' }, 'gate-version': { type: 'string' },
    'designation-env': { type: 'string' }, 'run-log': { type: 'string' }, manifest: { type: 'string' }, reports: { type: 'string' } } });
  try {
    if (cmd === 'derive') {
      const m = derive(path.resolve(root, values.src), underBuild(values.out), { frames: Number(values.frames ?? 0), sourceLabel: rel(values.src) });
      console.log(JSON.stringify({ out: values.out, frames: m.frames, sensorRows: m.sensorRows, droppedConfigKeys: m.droppedConfigKeys, framesWithoutAtlasDisabled: m.framesWithoutAtlasDisabled }));
    } else if (cmd === 'run') {
      await cmdRun(values);
    } else if (cmd === 'precheck') {
      cmdPrecheck(values);
    } else if (cmd === 'verdict') {
      cmdVerdict(values);
    } else if (cmd === 'audit') {
      cmdAudit(values);
    } else if (cmd === 'pin') {  // for the coordinator's designation file: the values the device compares
      console.log(JSON.stringify(designationPins(path.resolve(root, values.manifest)), null, 1));
    } else if (cmd === 'selftest') {
      const { selftest } = await import('./selftest.mjs');
      await selftest({ root, out: underBuild(values.out) });
    } else {
      console.error('usage: or1.mjs derive|run|precheck|verdict|audit|pin|selftest ... (see the header of this file)');
      process.exitCode = 2;
    }
  } catch (error) {
    console.error(String(error && error.message || error));
    process.exitCode = String(error && error.message).startsWith('HELD:') ? 3 : 1;
  }
}
