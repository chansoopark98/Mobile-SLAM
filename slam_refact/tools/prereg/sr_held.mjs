// sr_held: the shared HELD device of the preregistered gates (docs/design/preregistration-protocol.md section 12.5,
// design and usage: docs/design/sr-held-design.md). Node side; the C++ side is
// slam_refact/engine/tests/support/sr_held.h. Both read the same manifest, designation file, one-run marker and seed file
// and make the same decision for the same input (conformance: sr_held_cases.json, read by sr_held_conformance.test.mjs and
// by the C++ sr_held_conformance).
//
// A first real run is refused (HELD) by CODE unless every condition holds; it can happen once per (gate, version, kind).
// The conditions are listed at the top of sr_held.h and in the design document; this file implements the same decision
// procedure in the same order, so that the same defect gives the same refusal code. It is an accident guard, not an
// authentication (protocol 12.6).
//
// CLI (no seed is ever printed):
//   node sr_held.mjs print-pin <manifest.json>        canonical sha256 for the designation's pinned.manifest_before_review_sha256
//   node sr_held.mjs check --prereg DIR --gate ID --version N [--kind K] [--env-var NAME] [--pin FILE]... [--data DIR] [--seed]
//                                                       CheckRelease only: no marker is created, no run starts
import fs from 'node:fs';
import { createHash } from 'node:crypto';
import { fileURLToPath } from 'node:url';

// ---------------------------------------------------------------------------------------------------------------------
// Files and hashes

export const sha256 = (bytes) => createHash('sha256').update(bytes).digest('hex');

function readFile(file) {
  try {
    if (!fs.statSync(file).isFile()) return null;
    return fs.readFileSync(file);
  } catch {
    return null;
  }
}

const sha256File = (file) => {
  const b = readFile(file);
  return b === null ? null : sha256(b);
};

const isHex64 = (s) => typeof s === 'string' && /^[0-9a-f]{64}$/.test(s);
export const isBareName = (s) => typeof s === 'string' && s.length > 0 && Buffer.byteLength(s) <= 255 && s !== '.' && s !== '..' && !/[/\\\u0000-\u001f]/.test(s);
export const isId = (s) => typeof s === 'string' && /^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$/.test(s);
export const isDecimal = (s) => typeof s === 'string' && /^(0|[1-9][0-9]{0,19})$/.test(s) && BigInt(s) <= 18446744073709551615n;

// sha256 over the sorted lines "<relative path>\t<file sha256>\n" of every regular file below `dir`. Symlinks and special
// files are refused (INPUT_NONREGULAR), as is an empty or unreadable tree (INPUT_UNREADABLE).
function listTree(root, rel, files) {
  let names;
  try {
    names = fs.readdirSync(rel === '' ? root : `${root}/${rel}`);
  } catch {
    return 'INPUT_UNREADABLE';
  }
  for (const name of names) {
    const r = rel === '' ? name : `${rel}/${name}`;
    let st;
    try {
      st = fs.lstatSync(`${root}/${r}`);
    } catch {
      return 'INPUT_UNREADABLE';
    }
    if (st.isDirectory()) {
      const code = listTree(root, r, files);
      if (code !== null) return code;
    } else if (st.isFile()) {
      files.push(r);
    } else {
      return 'INPUT_NONREGULAR';
    }
  }
  return null;
}

const byBytes = (a, b) => Buffer.compare(Buffer.from(a), Buffer.from(b));

// returns { sha } or { code }
export function treeSha(dir) {
  try {
    if (!fs.statSync(dir).isDirectory()) return { code: 'INPUT_UNREADABLE' };
  } catch {
    return { code: 'INPUT_UNREADABLE' };
  }
  const files = [];
  const code = listTree(dir, '', files);
  if (code !== null) return { code };
  if (files.length === 0) return { code: 'INPUT_UNREADABLE' };
  files.sort(byBytes);
  let lines = '';
  for (const f of files) {
    const h = sha256File(`${dir}/${f}`);
    if (h === null) return { code: 'INPUT_UNREADABLE' };
    lines += `${f}\t${h}\n`;
  }
  return { sha: sha256(lines) };
}

export const selfExe = () => process.execPath;

// ---------------------------------------------------------------------------------------------------------------------
// Strict JSON (RFC 8259 value grammar; a duplicate key in one object, a BOM, invalid UTF-8, a lone surrogate escape, NaN
// and Infinity are errors). Values are read by path through get(); numbers keep their raw token.

export class Json {
  constructor(type) {
    this.type = type; // 'null' | 'bool' | 'number' | 'string' | 'array' | 'object'
    this.boolean = false;
    this.text = '';
    this.items = [];
    this.keys = [];
    this.values = [];
  }
  get(key) {
    if (this.type !== 'object') return null;
    const i = this.keys.indexOf(key);
    return i < 0 ? null : this.values[i];
  }
}

class JsonError extends Error {}

class Parser {
  constructor(s) {
    this.s = s;
    this.i = 0;
  }
  err(why) {
    throw new JsonError(`${why} at ${this.i}`);
  }
  ws() {
    while (this.i < this.s.length && ' \t\n\r'.includes(this.s[this.i])) this.i++;
  }
  literal(word) {
    if (!this.s.startsWith(word, this.i)) this.err('bad literal');
    this.i += word.length;
  }
  hex4() {
    const h = this.s.slice(this.i, this.i + 4);
    if (!/^[0-9a-fA-F]{4}$/.test(h)) this.err('bad \\u escape');
    this.i += 4;
    return parseInt(h, 16);
  }
  string() {
    this.i++;
    let out = '';
    for (;;) {
      if (this.i >= this.s.length) this.err('unterminated string');
      const c = this.s[this.i];
      const code = c.charCodeAt(0);
      if (c === '"') {
        this.i++;
        return out;
      }
      if (code < 0x20) this.err('control character in string');
      if (c !== '\\') {
        out += c;
        this.i++;
        continue;
      }
      this.i++;
      if (this.i >= this.s.length) this.err('unterminated escape');
      const e = this.s[this.i++];
      const simple = { '"': '"', '\\': '\\', '/': '/', b: '\b', f: '\f', n: '\n', r: '\r', t: '\t' };
      if (e in simple) {
        out += simple[e];
      } else if (e === 'u') {
        let cp = this.hex4();
        if (cp >= 0xd800 && cp <= 0xdbff) {
          if (!this.s.startsWith('\\u', this.i)) this.err('lone high surrogate');
          this.i += 2;
          const lo = this.hex4();
          if (lo < 0xdc00 || lo > 0xdfff) this.err('lone high surrogate');
          cp = 0x10000 + ((cp - 0xd800) << 10) + (lo - 0xdc00);
        } else if (cp >= 0xdc00 && cp <= 0xdfff) {
          this.err('lone low surrogate');
        }
        out += String.fromCodePoint(cp);
      } else {
        this.err('bad escape');
      }
    }
  }
  number() {
    const m = /^-?(?:0|[1-9][0-9]*)(?:\.[0-9]+)?(?:[eE][+-]?[0-9]+)?/.exec(this.s.slice(this.i));
    if (m === null) this.err('bad number');
    this.i += m[0].length;
    const v = new Json('number');
    v.text = m[0];
    return v;
  }
  value(depth) {
    if (depth > 64) this.err('nesting too deep');
    if (this.i >= this.s.length) this.err('unexpected end');
    const c = this.s[this.i];
    if (c === '{') {
      const o = new Json('object');
      this.i++;
      this.ws();
      if (this.s[this.i] === '}') {
        this.i++;
        return o;
      }
      for (;;) {
        this.ws();
        if (this.s[this.i] !== '"') this.err('object key expected');
        const key = this.string();
        if (o.keys.includes(key)) this.err(`duplicate key "${key}"`);
        this.ws();
        if (this.s[this.i] !== ':') this.err("':' expected");
        this.i++;
        this.ws();
        const v = this.value(depth + 1);
        o.keys.push(key);
        o.values.push(v);
        this.ws();
        if (this.s[this.i] === ',') {
          this.i++;
          continue;
        }
        if (this.s[this.i] === '}') {
          this.i++;
          return o;
        }
        this.err("',' or '}' expected");
      }
    }
    if (c === '[') {
      const a = new Json('array');
      this.i++;
      this.ws();
      if (this.s[this.i] === ']') {
        this.i++;
        return a;
      }
      for (;;) {
        this.ws();
        a.items.push(this.value(depth + 1));
        this.ws();
        if (this.s[this.i] === ',') {
          this.i++;
          continue;
        }
        if (this.s[this.i] === ']') {
          this.i++;
          return a;
        }
        this.err("',' or ']' expected");
      }
    }
    if (c === '"') {
      const v = new Json('string');
      v.text = this.string();
      return v;
    }
    if (c === 't') {
      this.literal('true');
      const v = new Json('bool');
      v.boolean = true;
      return v;
    }
    if (c === 'f') {
      this.literal('false');
      return new Json('bool');
    }
    if (c === 'n') {
      this.literal('null');
      return new Json('null');
    }
    if (c === '-' || (c >= '0' && c <= '9')) return this.number();
    return this.err('unexpected character');
  }
}

// returns { json } or { err }
export function parseJson(bytes) {
  let text;
  try {
    text = new TextDecoder('utf-8', { fatal: true, ignoreBOM: true }).decode(bytes);
  } catch {
    return { err: 'invalid UTF-8' };
  }
  if (text.charCodeAt(0) === 0xfeff) return { err: 'byte order mark' };
  try {
    const p = new Parser(text);
    p.ws();
    const json = p.value(0);
    p.ws();
    if (p.i !== text.length) return { err: 'trailing characters' };
    return { json };
  } catch (e) {
    if (e instanceof JsonError) return { err: e.message };
    throw e;
  }
}

// ---------------------------------------------------------------------------------------------------------------------
// Canonical form of the manifest: json.dumps(sort_keys=True, separators=(",", ":"), ensure_ascii=False) of Python, over
// the manifest without the keys the coordinator and the runner fill in afterwards (what verify_manifest.py --print-pin
// prints; the designation pins it as manifest_before_review_sha256).

const PIN_BLANKED = new Set(['review', 'first_real_run_at', 'held_run']);

function canonString(s) {
  let out = '"';
  for (const ch of s) {
    const c = ch.codePointAt(0);
    if (ch === '"') out += '\\"';
    else if (ch === '\\') out += '\\\\';
    else if (ch === '\n') out += '\\n';
    else if (ch === '\r') out += '\\r';
    else if (ch === '\t') out += '\\t';
    else if (ch === '\b') out += '\\b';
    else if (ch === '\f') out += '\\f';
    else if (c < 0x20) out += `\\u00${c.toString(16).padStart(2, '0')}`;
    else out += ch;
  }
  return `${out}"`;
}

// Python: an integer token prints as the integer ("-0" is 0); a token with '.', 'e' or 'E' is a float and prints as
// repr(float): the shortest round-trip digits, exponent form when the decimal exponent is < -4 or >= 16. null = no form.
function canonNumber(tok) {
  if (!/[.eE]/.test(tok)) return tok === '-0' ? '0' : tok;
  const d = Number(tok);
  if (!Number.isFinite(d)) return null;
  if (d === 0) return Object.is(d, -0) ? '-0.0' : '0.0';
  const sci = d.toExponential(); // shortest round-trip digits: [-]d[.ddd]e[+-]x
  const epos = sci.indexOf('e');
  let mant = sci.slice(0, epos);
  const e10 = parseInt(sci.slice(epos + 1), 10);
  let sign = '';
  if (mant[0] === '-') {
    sign = '-';
    mant = mant.slice(1);
  }
  const digits = mant.replace('.', '');
  const decpt = e10 + 1;
  const n = digits.length;
  let r;
  if (decpt <= -4 || decpt > 16) {
    const ae = Math.abs(e10);
    r = digits.slice(0, 1) + (n > 1 ? `.${digits.slice(1)}` : '') + 'e' + (e10 < 0 ? '-' : '+') + (ae < 10 ? '0' : '') + String(ae);
  } else if (decpt <= 0) {
    r = `0.${'0'.repeat(-decpt)}${digits}`;
  } else if (decpt >= n) {
    r = `${digits}${'0'.repeat(decpt - n)}.0`;
  } else {
    r = `${digits.slice(0, decpt)}.${digits.slice(decpt)}`;
  }
  return sign + r;
}

// returns the canonical text, or null when a number has no canonical form
function canonValue(v, skip) {
  switch (v.type) {
    case 'null': return 'null';
    case 'bool': return v.boolean ? 'true' : 'false';
    case 'number': return canonNumber(v.text);
    case 'string': return canonString(v.text);
    case 'array': {
      const parts = [];
      for (const it of v.items) {
        const c = canonValue(it, null);
        if (c === null) return null;
        parts.push(c);
      }
      return `[${parts.join(',')}]`;
    }
    default: {
      const order = [];
      for (let i = 0; i < v.keys.length; i++) if (skip === null || !skip.has(v.keys[i])) order.push(i);
      order.sort((a, b) => byBytes(v.keys[a], v.keys[b]));
      const parts = [];
      for (const i of order) {
        const c = canonValue(v.values[i], null);
        if (c === null) return null;
        parts.push(`${canonString(v.keys[i])}:${c}`);
      }
      return `{${parts.join(',')}}`;
    }
  }
}

export const canonManifestText = (manifest) => canonValue(manifest, PIN_BLANKED);
export function canonManifestSha(manifest) {
  const t = canonManifestText(manifest);
  return t === null ? null : sha256(Buffer.from(t, 'utf8'));
}

// ---------------------------------------------------------------------------------------------------------------------
// Time: "YYYY-MM-DDTHH:MM:SS[.f{1,9}](Z|+HH:MM|-HH:MM)", the time zone is required (`date -Is` form).

export function parseInstant(t) {
  const m = /^(\d{4})-(\d{2})-(\d{2})T(\d{2}):(\d{2}):(\d{2})(?:\.(\d{1,9}))?(Z|[+-]\d{2}:\d{2})$/.exec(typeof t === 'string' ? t : '');
  if (m === null) return null;
  const [y, mo, d, h, mi, s] = m.slice(1, 7).map(Number);
  const ns = m[7] === undefined ? 0 : Number(m[7].padEnd(9, '0'));
  let offset = 0;
  if (m[8] !== 'Z') {
    const oh = Number(m[8].slice(1, 3));
    const om = Number(m[8].slice(4, 6));
    if (oh > 23 || om > 59) return null;
    offset = (oh * 3600 + om * 60) * (m[8][0] === '-' ? -1 : 1);
  }
  const leap = (y % 4 === 0 && y % 100 !== 0) || y % 400 === 0;
  const days = [31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31];
  if (y < 1 || mo < 1 || mo > 12 || d < 1 || d > days[mo - 1] + (mo === 2 && leap ? 1 : 0)) return null;
  if (h > 23 || mi > 59 || s > 59) return null;
  const yy = y - (mo <= 2 ? 1 : 0);
  const era = Math.floor(yy / 400);
  const yoe = yy - era * 400;
  const doy = Math.floor((153 * (mo + (mo > 2 ? -3 : 9)) + 2) / 5) + d - 1;
  const doe = yoe * 365 + Math.floor(yoe / 4) - Math.floor(yoe / 100) + doy;
  return { sec: (era * 146097 + doe - 719468) * 86400 + h * 3600 + mi * 60 + s - offset, ns };
}

const before = (a, b) => (a.sec !== b.sec ? a.sec < b.sec : a.ns < b.ns);
const systemNow = () => {
  const ms = Date.now();
  return { sec: Math.floor(ms / 1000), ns: (ms % 1000) * 1e6 };
};

// ---------------------------------------------------------------------------------------------------------------------
// The device. Request: { preregDir, gateId, version, kind = 'main', env, pins = [], dataDir = '', wantSeed = false, now = '' }
// Outcome: { ok, code, why, log, designationSha256, seed, marker } (seed and marker only from heldGate()).

const manifestName = (rq) => `${rq.gateId}.v${rq.version}.manifest.json`;
const gateFileName = (rq) => `${rq.gateId}.v${rq.version}.md`;
const seedFileName = (rq) => `${rq.gateId}.v${rq.version}.coordinator-seed.txt`;
export const markerName = (rq) => `${rq.gateId}.v${rq.version}.first-run.${rq.kind ?? 'main'}.marker`;

const newOutcome = () => ({ ok: false, code: '', why: '', log: [], designationSha256: '', seed: '', marker: '' });
function fail(o, code, why) {
  o.ok = false;
  o.code = code;
  o.why = why;
  return false;
}
const log = (o, line) => o.log.push(`[SR-HELD] ${line}`);
const member = (obj, key, type) => {
  const v = obj === null ? null : obj.get(key);
  return v !== null && v.type === type ? v : null;
};
const basename = (p) => p.slice(p.lastIndexOf('/') + 1);

function checkManifestShape(m, o) {
  const review = member(m, 'review', 'object');
  const held = member(m, 'held', 'object');
  if (member(m, 'file', 'string') === null) return fail(o, 'MANIFEST_INVALID', 'manifest file is not a string');
  if (member(m, 'sha256', 'string') === null) return fail(o, 'MANIFEST_INVALID', 'manifest sha256 is not a string');
  if (member(m, 'created_at', 'string') === null) return fail(o, 'MANIFEST_INVALID', 'manifest created_at is not a string');
  if (review === null) return fail(o, 'MANIFEST_INVALID', 'manifest review is not an object');
  if (held === null) return fail(o, 'MANIFEST_INVALID', 'manifest held is not an object');
  const at = review.get('at');
  if (at === null || (at.type !== 'null' && at.type !== 'string')) return fail(o, 'MANIFEST_INVALID', 'manifest review.at is neither null nor a string');
  const run = m.get('first_real_run_at');
  if (run === null || (run.type !== 'null' && run.type !== 'string')) return fail(o, 'MANIFEST_INVALID', 'manifest first_real_run_at is neither null nor a string');
  for (const key of ['file', 'sha256', 'designation_file', 'designation_file_sha256', 'reviewer']) {
    const v = member(review, key, 'string');
    if (v === null || v.text === '') return fail(o, 'MANIFEST_INVALID', `manifest review.${key} is not a non-empty string`);
  }
  const kinds = member(held, 'kinds', 'array');
  if (kinds === null || kinds.items.length === 0) return fail(o, 'MANIFEST_INVALID', 'manifest held.kinds is not a non-empty array');
  const seen = new Set();
  for (const k of kinds.items) {
    if (k.type !== 'string' || !isId(k.text) || seen.has(k.text)) return fail(o, 'MANIFEST_INVALID', 'manifest held.kinds has a bad or repeated kind');
    seen.add(k.text);
  }
  const pins = member(held, 'pins', 'object');
  if (pins === null) return fail(o, 'MANIFEST_INVALID', 'manifest held.pins is not an object');
  for (let i = 0; i < pins.keys.length; i++) {
    if (!isBareName(pins.keys[i]) || pins.values[i].type !== 'string' || !isHex64(pins.values[i].text)) return fail(o, 'MANIFEST_INVALID', 'manifest held.pins has a bad entry');
  }
  const inputs = member(held, 'inputs', 'object');
  if (inputs === null) return fail(o, 'MANIFEST_INVALID', 'manifest held.inputs is not an object');
  for (let i = 0; i < inputs.keys.length; i++) {
    if (!seen.has(inputs.keys[i]) || inputs.values[i].type !== 'string' || !isHex64(inputs.values[i].text)) return fail(o, 'MANIFEST_INVALID', 'manifest held.inputs has a bad entry (a kind that is not in held.kinds, or a bad sha256)');
  }
  if (member(held, 'seed', 'bool') === null) return fail(o, 'MANIFEST_INVALID', 'manifest held.seed is not a boolean');
  const dev = held.get('dev_seeds');
  if (dev !== null) {
    if (dev.type !== 'array') return fail(o, 'MANIFEST_INVALID', 'manifest held.dev_seeds is not an array');
    for (const s of dev.items) if (s.type !== 'string' || !isDecimal(s.text)) return fail(o, 'MANIFEST_INVALID', 'manifest held.dev_seeds has a bad entry');
  }
  const sealed = held.get('sealed_file_sha256');
  if (sealed !== null) {
    if (sealed.type !== 'array') return fail(o, 'MANIFEST_INVALID', 'manifest held.sealed_file_sha256 is not an array');
    for (const s of sealed.items) if (s.type !== 'string' || !isHex64(s.text)) return fail(o, 'MANIFEST_INVALID', 'manifest held.sealed_file_sha256 has a bad entry');
  }
  return true;
}

// The coordinator seed: validated before the marker is claimed (a bad seed must not burn the run), returned only by
// heldGate(). The value never goes to the log or to a reason; only the sha256 of the seed file does.
function loadSeed(rq, held, pinned, o, box) {
  const bytes = readFile(`${rq.preregDir}/${seedFileName(rq)}`);
  if (bytes === null) return fail(o, 'SEED_MISSING', `the coordinator seed file ${seedFileName(rq)} is missing`);
  const token = bytes.toString('latin1').split(/[ \t\n\v\f\r]+/).filter((t) => t !== '')[0] ?? '';
  if (!isDecimal(token)) return fail(o, 'SEED_INVALID', 'the first token of the coordinator seed file is not an unsigned decimal integer');
  const dev = held.get('dev_seeds');
  if (dev !== null) for (const s of dev.items) if (s.text === token) return fail(o, 'SEED_FORBIDDEN', 'the coordinator seed equals a seed the executor ran before the review (held.dev_seeds)');
  const fileSha = sha256(bytes);
  log(o, `seed_file ${seedFileName(rq)} sha256=${fileSha} (the value is not logged)`);
  const pin = pinned.get('seed_file_sha256');
  if (pin !== null && (pin.type !== 'string' || pin.text !== fileSha)) return fail(o, 'SEED_PIN', 'the coordinator seed file differs from designation pinned.seed_file_sha256');
  box.seed = token;
  return true;
}

function evaluate(rq, mode, o, box) {
  const kind = rq.kind ?? 'main';
  const version = rq.version;
  // 1 args
  if (!isId(rq.gateId)) return fail(o, 'ARGS', 'gate_id is not [A-Za-z0-9][A-Za-z0-9_-]{0,63}');
  if (!Number.isInteger(version) || version < 1 || version > 999999) return fail(o, 'ARGS', 'version is not in 1..999999');
  if (!isId(kind)) return fail(o, 'ARGS', 'kind is not [A-Za-z0-9][A-Za-z0-9_-]{0,63}');
  if (typeof rq.preregDir !== 'string' || rq.preregDir === '') return fail(o, 'ARGS', 'prereg_dir is empty');
  let now = systemNow();
  if (rq.now) {
    now = parseInstant(rq.now);
    if (now === null) return fail(o, 'ARGS', 'now is not an ISO instant with a time zone');
  }
  const env = rq.env ?? '';
  const req = { ...rq, kind };
  // 2 env
  if (mode === 'release' && env === '') return fail(o, 'ENV_MISSING', `HELD: the environment value is not set (the first real run of ${rq.gateId} needs the coordinator-designated review)`);
  // 3 manifest
  const manifestBytes = readFile(`${rq.preregDir}/${manifestName(rq)}`);
  if (manifestBytes === null) return fail(o, 'MANIFEST_UNREADABLE', `HELD: manifest ${manifestName(rq)} is unreadable`);
  const parsed = parseJson(manifestBytes);
  if (parsed.err !== undefined) return fail(o, 'MANIFEST_INVALID', `manifest is not strict JSON: ${parsed.err}`);
  const m = parsed.json;
  if (m.type !== 'object') return fail(o, 'MANIFEST_INVALID', 'manifest is not a JSON object');
  // 4 identity and shape
  const id = member(m, 'gate_id', 'string');
  const ver = member(m, 'version', 'number');
  if (id === null || ver === null) return fail(o, 'MANIFEST_INVALID', 'manifest gate_id / version missing or mistyped');
  if (id.text !== rq.gateId) return fail(o, 'MANIFEST_IDENTITY', `manifest names gate_id ${id.text}, not ${rq.gateId}`);
  if (ver.text !== String(version)) return fail(o, 'MANIFEST_IDENTITY', `manifest names version ${ver.text}, not ${version}`);
  if (!checkManifestShape(m, o)) return false;
  const review = m.get('review');
  const held = m.get('held');
  // 5 names
  const gateName = m.get('file').text;
  const reviewName = review.get('file').text;
  const designationName = review.get('designation_file').text;
  if (!isBareName(gateName) || gateName !== gateFileName(rq)) return fail(o, 'NAME_UNSAFE', `manifest file must be ${gateFileName(rq)}`);
  if (!isBareName(reviewName)) return fail(o, 'NAME_UNSAFE', 'manifest review.file is not a bare file name');
  if (!isBareName(designationName)) return fail(o, 'NAME_UNSAFE', 'manifest review.designation_file is not a bare file name');
  // 6 time
  const created = parseInstant(m.get('created_at').text);
  if (created === null) return fail(o, 'MANIFEST_INVALID', 'manifest created_at is not an ISO instant with a time zone');
  if (review.get('at').type === 'null') return fail(o, 'REVIEW_MISSING', 'HELD: manifest review.at is null: the review is not recorded');
  const reviewAt = parseInstant(review.get('at').text);
  if (reviewAt === null) return fail(o, 'MANIFEST_INVALID', 'manifest review.at is not an ISO instant with a time zone');
  const run = m.get('first_real_run_at');
  if (mode === 'release' && run.type !== 'null') return fail(o, 'ALREADY_RUN', 'manifest first_real_run_at is already set: this gate was run before (one run per registration)');
  if (mode === 'audit' && run.type === 'null') return fail(o, 'AUDIT_NOT_RUN', 'manifest first_real_run_at is null: there is no run to audit');
  const runAt = mode === 'audit' ? parseInstant(run.text) : null;
  if (mode === 'audit' && runAt === null) return fail(o, 'MANIFEST_INVALID', 'manifest first_real_run_at is not an ISO instant with a time zone');
  if (!before(created, reviewAt)) return fail(o, 'ORDER', 'created_at < review.at violated');
  if (mode === 'release' && !before(reviewAt, now)) return fail(o, 'ORDER', 'review.at < now violated (the review is dated in the future)');
  if (mode === 'audit' && !before(reviewAt, runAt)) return fail(o, 'ORDER', 'review.at < first_real_run_at violated (the run is not after the review)');
  // 7 gate file, review file, designation file
  const gateSha = sha256File(`${rq.preregDir}/${gateName}`);
  if (gateSha === null) return fail(o, 'GATE_FILE_MISSING', `HELD: gate file ${gateName} is unreadable`);
  log(o, `gate_file ${gateName} sha256 actual=${gateSha} registered=${m.get('sha256').text}`);
  if (gateSha !== m.get('sha256').text) return fail(o, 'GATE_SHA_MISMATCH', 'gate file sha256 differs from the manifest');
  const reviewSha = sha256File(`${rq.preregDir}/${reviewName}`);
  if (reviewSha === null) return fail(o, 'REVIEW_FILE_MISSING', `HELD: review file ${reviewName} is unreadable`);
  log(o, `review_file ${reviewName} sha256 actual=${reviewSha} registered=${review.get('sha256').text}`);
  if (reviewSha !== review.get('sha256').text) return fail(o, 'REVIEW_FILE_SHA', 'review file sha256 differs from the manifest');
  const designationBytes = readFile(`${rq.preregDir}/${designationName}`);
  if (designationBytes === null) return fail(o, 'DESIGNATION_MISSING', `HELD: designation file ${designationName} is unreadable`);
  o.designationSha256 = sha256(designationBytes);
  log(o, `designation_file ${designationName} sha256 actual=${o.designationSha256} manifest=${review.get('designation_file_sha256').text}`);
  if (o.designationSha256 !== review.get('designation_file_sha256').text) return fail(o, 'DESIGNATION_SHA', 'designation file sha256 differs from the manifest');
  if (mode === 'release' && env !== o.designationSha256) return fail(o, 'ENV_MISMATCH', 'HELD: the environment value is not the sha256 of the registered designation file');
  // 8 designation
  const dparsed = parseJson(designationBytes);
  if (dparsed.err !== undefined) return fail(o, 'DESIGNATION_INVALID', `designation file is not strict JSON: ${dparsed.err}`);
  const d = dparsed.json;
  if (d.type !== 'object') return fail(o, 'DESIGNATION_INVALID', 'designation file is not a JSON object');
  const pinned = member(d, 'pinned', 'object');
  if (pinned === null) return fail(o, 'DESIGNATION_INVALID', 'designation pinned is not an object');
  const dgate = member(d, 'gate', 'string');
  if (dgate === null || dgate.text !== `${rq.gateId}.v${version}`) return fail(o, 'DESIGNATION_GATE', `designation file does not name ${rq.gateId}.v${version}`);
  const permitted = d.get('first_real_run_permitted');
  if (permitted === null || permitted.type !== 'bool' || !permitted.boolean) return fail(o, 'DESIGNATION_NOT_PERMITTED', 'designation file: first_real_run_permitted is not true');
  const pinGate = member(pinned, 'gate_md_sha256', 'string');
  const pinReview = member(pinned, 'review_file_sha256', 'string');
  const pinManifest = member(pinned, 'manifest_before_review_sha256', 'string');
  if (pinGate === null || pinReview === null || pinManifest === null) return fail(o, 'DESIGNATION_INVALID', 'designation pinned lacks gate_md_sha256 / review_file_sha256 / manifest_before_review_sha256');
  const canonSha = canonManifestSha(m);
  if (canonSha === null) return fail(o, 'MANIFEST_INVALID', 'manifest has a number that has no canonical form');
  log(o, `manifest_before_review_sha256 actual=${canonSha} designation=${pinManifest.text}`);
  if (pinGate.text !== gateSha) return fail(o, 'DESIGNATION_PIN', 'designation pinned.gate_md_sha256 differs from the gate file');
  if (pinReview.text !== reviewSha) return fail(o, 'DESIGNATION_PIN', 'designation pinned.review_file_sha256 differs from the review file');
  if (pinManifest.text !== canonSha) return fail(o, 'DESIGNATION_PIN', 'manifest differs from the one the designation pinned (edited beyond review / first_real_run_at / held_run)');
  if (mode === 'audit') {
    if (!lstatExists(`${rq.preregDir}/${markerName(req)}`)) return fail(o, 'MARKER_MISSING', `no one-run marker ${markerName(req)} for a manifest that records a run`);
    o.ok = true;
    return true;
  }
  // 9 kind, pins, inputs
  if (!held.get('kinds').items.some((k) => k.text === kind)) return fail(o, 'KIND_UNREGISTERED', `kind ${kind} is not in manifest held.kinds`);
  const pins = held.get('pins');
  const presented = new Set();
  for (const path of rq.pins ?? []) {
    const base = basename(path);
    if (presented.has(base)) return fail(o, 'PIN_DUPLICATE', `two presented pin files have the base name ${base}`);
    presented.add(base);
    const want = pins.get(base);
    if (want === null) return fail(o, 'PIN_UNREGISTERED', `manifest held.pins has no entry for ${base}`);
    const have = sha256File(path);
    if (have === null) return fail(o, 'PIN_UNREADABLE', `pinned file ${base} is unreadable`);
    log(o, `pin ${base} sha256 actual=${have} registered=${want.text}`);
    if (have !== want.text) return fail(o, 'PIN_MISMATCH', `held file ${base} sha256 differs from the registered one`);
  }
  for (const key of pins.keys) {
    if (!presented.has(key)) return fail(o, 'PIN_NOT_PRESENTED', `manifest held.pins registers ${key} but the request does not present it`);
  }
  const dataDir = rq.dataDir ?? '';
  const registered = held.get('inputs').get(kind);
  if (registered !== null && dataDir === '') return fail(o, 'INPUT_NOT_SUPPLIED', `manifest registers input data for kind ${kind} but the request supplies no data_dir`);
  if (registered === null && dataDir !== '') return fail(o, 'INPUT_UNREGISTERED', `the request supplies a data_dir but the manifest registers no input data for kind ${kind}`);
  if (registered !== null) {
    const have = treeSha(dataDir);
    if (have.code !== undefined) return fail(o, have.code, `input data ${dataDir} cannot be hashed (${have.code})`);
    log(o, `inputs ${kind} tree sha256 actual=${have.sha} registered=${registered.text}`);
    if (have.sha !== registered.text) return fail(o, 'INPUT_MISMATCH', `input data ${dataDir} differs from the registered data (tree sha256)`);
  }
  // 10 seed
  const wantSeed = rq.wantSeed === true;
  if (held.get('seed').boolean !== wantSeed) return fail(o, 'SEED_POLICY', `the manifest registers held.seed=${held.get('seed').boolean} but the request says otherwise`);
  if (wantSeed && !loadSeed(rq, held, pinned, o, box)) return false;
  o.ok = true;
  return true;
}

function lstatExists(p) {
  try {
    fs.lstatSync(p);
    return true;
  } catch {
    return false;
  }
}

function run(rq, mode) {
  const o = newOutcome();
  const box = { seed: '' };
  evaluate(rq, mode, o, box);
  return { o, box };
}

// Every check, no side effect (no marker, no seed handed out).
export const checkRelease = (rq) => run(rq, 'release').o;

// Post-run audit of a manifest that records first_real_run_at: created_at < review.at < first_real_run_at, the pinned
// files, and the marker of this kind. No env value, no pins, no data.
export const audit = (rq) => run(rq, 'audit').o;

// The complete gate: checkRelease, then the one-run marker (O_EXCL, never removed). Only here the seed is handed out.
// A refusal before the marker is not a run; a call that created the marker is a run, whatever happens afterwards.
// Keep the returned outcome: a second call for the same (gate, version, kind) is refused (MARKER_EXISTS).
export function heldGate(rq) {
  const { o, box } = run(rq, 'release');
  if (!o.ok) return o;
  const req = { ...rq, kind: rq.kind ?? 'main' };
  const path = `${rq.preregDir}/${markerName(req)}`;
  let fd;
  try {
    fd = fs.openSync(path, 'wx', 0o444);
  } catch (e) {
    fail(o, e.code === 'EEXIST' ? 'MARKER_EXISTS' : 'MARKER_FAILED', `HELD: the first-run marker ${path} exists (or cannot be created): this kind was run before`);
    return o;
  }
  const body = `{"gate":"${rq.gateId}.v${rq.version}","kind":"${req.kind}","designation_sha256":"${o.designationSha256}","pid":${process.pid}}\n`;
  fs.writeSync(fd, body);
  fs.fsyncSync(fd);
  fs.closeSync(fd);
  log(o, `first-run marker created: ${path}`);
  o.seed = box.seed;
  o.marker = path;
  return o;
}

// Report mode must never measure registered gate data. Decided by what the directory CONTAINS, not by its name or path:
// its tree sha256 equals a registered held.inputs value, or a file in it has a sha256 listed in held.sealed_file_sha256.
// A manifest that cannot be read or parsed gives "sealed" (fail closed). ok === true means "not registered data".
export function refuseSealed(rq) {
  const o = newOutcome();
  const refuse = (code, why) => {
    fail(o, code, why);
    return o;
  };
  const dataDir = rq.dataDir ?? '';
  if (!isId(rq.gateId) || !Number.isInteger(rq.version) || rq.version < 1 || rq.version > 999999 || dataDir === '') return refuse('ARGS', 'gate_id / version / data_dir invalid');
  const bytes = readFile(`${rq.preregDir}/${manifestName(rq)}`);
  const parsed = bytes === null ? { err: 'unreadable' } : parseJson(bytes);
  if (parsed.err !== undefined || parsed.json.type !== 'object') return refuse('SEALED_DATA', 'the manifest cannot be read, so the data cannot be told from registered data (fail closed)');
  const held = member(parsed.json, 'held', 'object');
  const inputs = member(held, 'inputs', 'object');
  if (inputs === null) return refuse('SEALED_DATA', 'manifest held.inputs is missing (fail closed)');
  const tree = treeSha(dataDir);
  if (tree.sha === undefined) return refuse('SEALED_DATA', `${dataDir} cannot be hashed (${tree.code}), so it cannot be told from registered data (fail closed)`);
  for (const v of inputs.values) if (v.type === 'string' && v.text === tree.sha) return refuse('SEALED_DATA', `the files of ${dataDir} have the tree sha256 of registered gate data`);
  const arr = held.get('sealed_file_sha256');
  const sealedFiles = new Set(arr === null ? [] : arr.items.map((s) => s.text));
  const files = [];
  if (sealedFiles.size > 0 && listTree(dataDir, '', files) === null) {
    for (const f of files) {
      const h = sha256File(`${dataDir}/${f}`);
      if (h !== null && sealedFiles.has(h)) return refuse('SEALED_DATA', `${dataDir}/${f} has the sha256 of a sealed file`);
    }
  }
  o.ok = true;
  return o;
}

// ---------------------------------------------------------------------------------------------------------------------
// CLI

function cli(argv) {
  const [cmd, ...rest] = argv;
  if (cmd === 'print-pin' && rest.length === 1) {
    const bytes = readFile(rest[0]);
    const parsed = bytes === null ? { err: 'unreadable' } : parseJson(bytes);
    const sha = parsed.err === undefined ? canonManifestSha(parsed.json) : null;
    if (sha === null) {
      process.stderr.write(`cannot compute the canonical sha256 of ${rest[0]} (${parsed.err ?? 'number without canonical form'})\n`);
      return 1;
    }
    process.stdout.write(`${sha}\n`);
    return 0;
  }
  if (cmd === 'check') {
    const rq = { pins: [], kind: 'main', wantSeed: false };
    let envVar = '';
    for (let i = 0; i < rest.length; i++) {
      const a = rest[i];
      if (a === '--prereg') rq.preregDir = rest[++i];
      else if (a === '--gate') rq.gateId = rest[++i];
      else if (a === '--version') rq.version = Number(rest[++i]);
      else if (a === '--kind') rq.kind = rest[++i];
      else if (a === '--env-var') envVar = rest[++i];
      else if (a === '--pin') rq.pins.push(rest[++i]);
      else if (a === '--data') rq.dataDir = rest[++i];
      else if (a === '--seed') rq.wantSeed = true;
      else {
        process.stderr.write(`unknown argument ${a}\n`);
        return 2;
      }
    }
    rq.env = envVar === '' ? '' : (process.env[envVar] ?? '');
    const o = checkRelease(rq);
    process.stdout.write(`${o.log.join('\n')}${o.log.length ? '\n' : ''}${o.ok ? 'RELEASED (check only: no marker created)' : `HELD [${o.code}]: ${o.why}`}\n`);
    return o.ok ? 0 : 3;
  }
  process.stderr.write('usage: sr_held.mjs print-pin <manifest.json> | check --prereg DIR --gate ID --version N [--kind K] [--env-var NAME] [--pin FILE]... [--data DIR] [--seed]\n');
  return 2;
}

if (process.argv[1] && fs.existsSync(process.argv[1]) && fs.realpathSync(process.argv[1]) === fs.realpathSync(fileURLToPath(import.meta.url))) {
  process.exitCode = cli(process.argv.slice(2));
}
