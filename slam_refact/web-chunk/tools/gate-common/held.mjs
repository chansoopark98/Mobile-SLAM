// Thin adapter between the web-chunk gates (OR-1 v3, rm4c v5, US-010 v4) and the shared first-real-run device
// slam_refact/tools/prereg/sr_held.mjs (docs/design/sr-held-design.md; protocol 12.5). It adds nothing to the decision: it turns a refusal into the
// "HELD: <CODE>: <why>" error the gates print, and it lists the pins a gate presents. A gate keeps its own extra checks (arguments, data it alone
// knows) around releaseCheck() / claim().
//
// WHAT THIS IS NOT: an accident-prevention device, not an authentication (protocol 12.6; sr-held-design.md section 7).
import fs from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import * as sr from '../../../tools/prereg/sr_held.mjs';

export { sr };
// The file the import above loaded. A gate pins THIS path (held.pins key `sr_held.mjs`), so the device that decides is the device that was hashed.
export const SR_HELD_FILE = fileURLToPath(new URL('../../../tools/prereg/sr_held.mjs', import.meta.url));

export const heldError = (outcome) => new Error(`HELD: ${outcome.code}: ${String(outcome.why).replace(/^HELD: /, '')}`);

// A gate is named by four values the caller supplies (nothing here is a default): gate id, version, the preregistration directory and the NAME of the
// environment variable that holds the designation file's sha256.
export function gateArgs(values, root) {
  const missing = ['prereg-dir', 'gate-id', 'gate-version', 'designation-env'].filter((k) => !values[k]);
  if (missing.length) throw new Error(`gate arguments missing: ${missing.map((k) => `--${k}`).join(', ')}`);
  const version = Number(values['gate-version']);
  if (!Number.isInteger(version) || version < 1 || String(version) !== String(values['gate-version']).trim()) throw new Error(`--gate-version ${values['gate-version']}: a positive integer is required`);
  if (!/^[A-Z][A-Z0-9_]{2,}$/.test(values['designation-env'])) throw new Error(`--designation-env ${values['designation-env']}: an environment variable NAME is required (the value is read from the environment)`);
  return { gateId: values['gate-id'], version, preregDir: path.resolve(root, values['prereg-dir']), designationEnv: values['designation-env'] };
}

export const manifestPath = (gate) => path.join(gate.preregDir, `${gate.gateId}.v${gate.version}.manifest.json`);
export const gateMdPath = (gate) => path.join(gate.preregDir, `${gate.gateId}.v${gate.version}.md`);

// {basename: sha256} of the presented files, the flat form held.pins registers. Two files with one base name cannot be told apart by the device.
// `files` is [[path, sha256], ...].
export function flatPins(files) {
  const out = {};
  for (const [file, sha] of files) {
    const base = path.basename(file);
    if (base in out) throw new Error(`two pinned files share the base name ${base}: sr_held cannot register both`);
    out[base] = sha;
  }
  return out;
}

export function request(gate, { kind, env, pins, dataDir = '', wantSeed = false, now = null }) {
  return { preregDir: gate.preregDir, gateId: gate.gateId, version: gate.version, kind, env: env ?? '', pins, dataDir, wantSeed, ...(now ? { now: new Date(now).toISOString() } : {}) };
}

// CheckRelease: every condition of the device, no marker, no seed. Throws HELD.
export function releaseCheck(rq) {
  const o = sr.checkRelease(rq);
  if (!o.ok) throw heldError(o);
  return o;
}

// HeldGate: the checks again, then the one-run marker (O_EXCL, mode 0444, never removed). After this the run has started.
export function claim(rq) {
  const o = sr.heldGate(rq);
  if (!o.ok) throw heldError(o);
  return o;
}

// Report mode must not measure registered data: decided by the content of the directory (sr_held refuseSealed). A missing manifest means nothing is
// registered yet, so nothing is refused; an unreadable or invalid one refuses (fail closed, as the device does).
export function refuseSealedData(gate, dataDir) {
  if (!fs.existsSync(manifestPath(gate))) return { ok: true, registered: false };
  const o = sr.refuseSealed({ preregDir: gate.preregDir, gateId: gate.gateId, version: gate.version, dataDir });
  if (!o.ok) throw new Error(`refused: ${o.code}: ${o.why}`);
  return { ok: true, registered: true };
}
