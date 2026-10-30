// OR-1 canonical forms: every number is compared by the bits of its IEEE double (a hex string), never by a rounded decimal, so
// "equal" means bit-identical and a one-ulp difference is visible. Typed arrays are compared by length and sha256.
import { createHash } from 'node:crypto';

export const sha256 = (bytes) => createHash('sha256').update(bytes).digest('hex');

const f64 = new Float64Array(1);
const u64 = new BigUint64Array(f64.buffer);
export function f64hex(x) {
  if (typeof x !== 'number') throw new TypeError(`f64hex: ${typeof x}`);
  f64[0] = x;
  return u64[0].toString(16).padStart(16, '0');
}
export const hexToF64 = (h) => { u64[0] = BigInt(`0x${h}`); return f64[0]; };

// Deep canonical form of a JSON-like value: numbers -> "f:<16 hex>", typed arrays -> "u8:<len>:<sha256>" (or the element-wise form for
// float arrays), objects with sorted keys. The result is a plain JSON-serialisable value, so two canonical forms are compared with
// JSON.stringify.
export function canon(v) {
  if (v === null || v === undefined) return null;
  if (typeof v === 'number') return `f:${f64hex(v)}`;
  if (typeof v === 'boolean' || typeof v === 'string') return v;
  if (typeof v === 'bigint') return `i:${v.toString(16)}`;
  if (ArrayBuffer.isView(v)) {
    if (v instanceof Uint8Array || v instanceof Uint8ClampedArray) return `u8:${v.length}:${sha256(v)}`;
    return Array.from(v, (x) => canon(Number(x)));
  }
  if (Array.isArray(v)) return v.map(canon);
  if (typeof v === 'object') return Object.fromEntries(Object.keys(v).sort().map((k) => [k, canon(v[k])]));
  throw new TypeError(`canon: unsupported ${typeof v}`);
}
export const canonText = (v) => JSON.stringify(canon(v));

// Paths (dotted) at which two canonical values differ; the empty array = identical.
export function diffPaths(a, b, prefix = '', out = []) {
  if (JSON.stringify(a) === JSON.stringify(b)) return out;
  const ao = a && typeof a === 'object', bo = b && typeof b === 'object';
  if (ao && bo && Array.isArray(a) === Array.isArray(b)) {
    const keys = Array.isArray(a) ? [...Array(Math.max(a.length, b.length)).keys()] : [...new Set([...Object.keys(a), ...Object.keys(b)])].sort();
    for (const k of keys) diffPaths(a?.[k], b?.[k], prefix ? `${prefix}.${k}` : String(k), out);
    return out;
  }
  out.push(prefix || '(root)');
  return out;
}

// ULP distance of two doubles (same sign assumed non-negative distance; 0 = identical; Infinity for NaN / sign-crossing mismatch).
export function ulpDistance(x, y) {
  if (Number.isNaN(x) || Number.isNaN(y)) return Number.isNaN(x) && Number.isNaN(y) ? 0 : Infinity;
  f64[0] = x; const a = BigInt.asIntN(64, u64[0]);
  f64[0] = y; const b = BigInt.asIntN(64, u64[0]);
  const ord = (n) => (n < 0n ? -(n & 0x7fffffffffffffffn) : n);  // sign-magnitude -> ordered integer
  const d = ord(a) - ord(b);
  return Number(d < 0n ? -d : d);
}
