#!/usr/bin/env node
// us026-mse-unit-gate.v2, step 1 of the registered first run (called by run.sh in this directory): the golden step.
//
// Runs the ORIGINAL xr-slam.wasm in isolation on the registered inputs and writes the goldens the C++ judge
// (sr_tests_us026_method_precision_held, step 2) compares against. Nothing here compares anything and nothing prints a
// result value: every generator runs with --quiet 1 (only the output sha256 is printed).
//
//   node golden.mjs --prereg DIR --gate <gate_id>.v<N> --env-var NAME --out NEW_GOLDEN_DIR
//        --mmg mse_method_golden.mjs --kf6 mse-probe-kf6.mjs --wasm xr-slam.wasm --pin FILE... [--check]
//
// Order (protocol 12.5 / 12.6; shared device sr_held, kind "golden"):
//   1. sr_held heldGate: env value = sha256 of the registered designation file, manifest / review / designation pins,
//      held.pins = this node executable + every --pin (exactly the registered set), the input tree held.inputs.golden
//      (the registered 944-input dumps), the coordinator seed policy (held.seed, held.dev_seeds, designation pin)
//      -> then the O_EXCL one-run marker <gate>.first-run.golden.marker (never removed). --check: the same checks
//      without the marker (nothing is generated).
//   2. only after the marker: the output directory, then one mse_method_golden.mjs run per registered methods golden
//      (dump, core, method runs, estimate_after_have from the manifest "registration" block) and one mse-probe-kf6.mjs
//      --class registration run on the coordinator seed FILE (the seed value is never read or printed here; the
//      generator records only the seed file's sha256), then GOLDEN-SHA256SUMS; files 0444, directory 0555.
// The executed generator files must be among the presented pins (same real path), so what runs is what is pinned.
//
// Exit codes: 0 goldens written; 2 usage; 3 HELD (refused before the marker: not a run); 4 failure AFTER the marker
// (the golden step's one run is used up: INVALID, coordinator decision and a new version, gate file section 6).
// This file is an accident guard, not an authentication (protocol 12.6).
import fs from 'node:fs';
import path from 'node:path';
import { spawnSync } from 'node:child_process';
import { checkRelease, heldGate, sha256 } from '../../prereg/sr_held.mjs';

const argv = process.argv.slice(2);
const args = { pin: [] };
for (let i = 0; i < argv.length; i++) {
  const a = argv[i];
  if (a === '--check') args.check = true;
  else if (a === '--pin') args.pin.push(argv[++i]);
  else if (['--prereg', '--gate', '--env-var', '--out', '--mmg', '--kf6', '--wasm'].includes(a)) args[a.slice(2)] = argv[++i];
  else { console.error(`unknown argument ${a}`); process.exit(2); }
}
const usage = () => {
  console.error('usage: golden.mjs --prereg DIR --gate ID.vN --env-var NAME --out NEWDIR --mmg F --kf6 F --wasm F --pin F... [--check]');
  process.exit(2);
};
for (const k of ['prereg', 'gate', 'env-var', 'mmg', 'kf6', 'wasm']) if (!args[k]) usage();
if (!args.check && !args.out) usage();
const m = /^(.+)\.v([1-9][0-9]{0,5})$/.exec(args.gate);
if (!m) usage();
const [gateId, version] = [m[1], Number(m[2])];

const real = (p) => {
  try {
    return fs.realpathSync(p);
  } catch {
    console.error(`HELD [PIN_UNREADABLE]: ${p} does not exist`);
    process.exit(3);
  }
};
const pins = [process.execPath, ...args.pin];
const pinned = new Set(pins.map(real));
for (const k of ['mmg', 'kf6', 'wasm']) {
  if (!pinned.has(real(args[k]))) {
    console.error(`HELD [ARGS]: --${k} ${args[k]} is not among the presented pins`);
    process.exit(3);
  }
}

let reg;
try {
  reg = JSON.parse(fs.readFileSync(`${args.prereg}/${gateId}.v${version}.manifest.json`, 'utf8')).registration;
} catch (e) {
  console.error(`HELD [MANIFEST_UNREADABLE]: ${e.message}`);
  process.exit(3);
}
if (!reg || typeof reg.inputs_dir !== 'string' || !Array.isArray(reg.goldens) || !Array.isArray(reg.methods_runs)) {
  console.error('HELD [REGISTRATION]: the manifest has no usable registration block');
  process.exit(3);
}
const inputsDir = path.join(args.prereg, reg.inputs_dir);
const rq = {
  preregDir: args.prereg, gateId, version, kind: 'golden', env: process.env[args['env-var']] ?? '',
  pins, dataDir: inputsDir, wantSeed: true,
};
if (args.check) {
  const o = checkRelease(rq);
  console.log(o.log.join('\n'));
  console.log(o.ok ? 'RELEASED (check only: no marker created, nothing generated)' : `HELD [${o.code}]: ${o.why}`);
  process.exit(o.ok ? 0 : 3);
}
const held = heldGate(rq);
console.log(held.log.join('\n'));  // never contains the seed
if (!held.ok) {
  console.error(`HELD [${held.code}]: ${held.why}`);
  process.exit(3);
}
// ---- after the marker: a failure from here on uses up the golden step's one run (exit 4)
const after = (why) => {
  console.error(`INVALID (after the golden marker): ${why}`);
  process.exit(4);
};
try {
  if (fs.existsSync(args.out)) after(`${args.out} exists (a new directory is required)`);
  fs.mkdirSync(args.out, { recursive: false });
  const seedFile = `${args.prereg}/${gateId}.v${version}.coordinator-seed.txt`;
  const t0 = Date.now();
  for (const g of reg.goldens) {
    if (typeof g.file !== 'string' || g.file.includes('/')) after(`bad golden name ${g.file}`);
    const out = path.join(args.out, g.file);
    let cmd;
    if (g.kind === 'methods') {
      cmd = [args.mmg, '--dump', path.join(inputsDir, g.dump), '--core', String(g.core), '--methods', reg.methods_runs.join(','),
        '--output', out, '--quiet', '1', '--input', args.wasm];
      if (g.estimate_after_have === 1) cmd.push('--estimate-after-have', '1');
    } else if (g.kind === 'kf6') {
      cmd = [args.kf6, '--class', 'registration', '--seed-file', seedFile, '--output', out, '--quiet', '1', '--input', args.wasm];
    } else {
      after(`unknown golden kind ${g.kind}`);
    }
    const r = spawnSync(process.execPath, cmd, { encoding: 'utf8', maxBuffer: 1 << 26 });
    if (r.status !== 0) after(`${g.file}: generator exit ${r.status} ${r.signal ?? ''}\n${r.stderr}`);
    console.log(`[US026-GOLDEN] ${g.file} ${r.stdout.trim()}`);
  }
  const names = fs.readdirSync(args.out).sort();
  const sums = names.map((n) => `${sha256(fs.readFileSync(path.join(args.out, n)))}  ${n}\n`).join('');
  fs.writeFileSync(path.join(args.out, 'GOLDEN-SHA256SUMS'), sums, { flag: 'wx' });
  for (const n of fs.readdirSync(args.out)) fs.chmodSync(path.join(args.out, n), 0o444);
  fs.chmodSync(args.out, 0o555);
  console.log(`[US026-GOLDEN] ${names.length} goldens + GOLDEN-SHA256SUMS in ${args.out} (${((Date.now() - t0) / 1000).toFixed(1)} s)`);
} catch (e) {
  after(String(e && e.stack ? e.stack : e));
}
