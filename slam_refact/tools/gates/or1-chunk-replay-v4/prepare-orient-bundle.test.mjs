// node --test slam_refact/tools/gates/or1-chunk-replay-v4/prepare-orient-bundle.test.mjs
// The orient recording -> OR-1 source bundle converter: values and times on a synthetic recording written here, every refusal, and (when present) the real
// recordings flip6-s2 v2-b-o0 / o90 / o180 / o270. No engine; the tests write only under build/.
import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';
import test from 'node:test';
import zlib from 'node:zlib';
import { pathToFileURL } from 'node:url';

const repo = path.resolve(import.meta.dirname, '../../../..');
const { prepare } = await import(pathToFileURL(path.join(import.meta.dirname, 'prepare-orient-bundle.mjs')).href);
const { derive, readBundle } = await import(pathToFileURL(path.join(repo, 'slam_refact/web-chunk/tools/or1/bundle.mjs')).href);
const scratch = path.join(repo, 'build/slam-refact-g4-20261025/unit-tmp', `prep-${process.pid}`);
fs.mkdirSync(scratch, { recursive: true });
process.on('exit', () => fs.rmSync(scratch, { recursive: true, force: true }));
let n = 0;
const fresh = (tag) => path.join(scratch, `${tag}-${++n}`);
const sha = (b) => createHash('sha256').update(b).digest('hex');
const f32 = new Float32Array(1), u32 = new Uint32Array(f32.buffer);
const hex = (x) => { f32[0] = x; return u32[0].toString(16).padStart(8, '0'); };

// A recording of 6 stages: bootstrap (queue 0), input frames 3, 4, 5, 6, 7 (queues 1..5; queue 2 has no packet, queue 3 has two). Captures of input frames 4, 5, 7
// (6 has none). Packet values are float32-exact so that the float32 bits of the event queue and the doubles of packets.tsv agree.
const QUEUES = [[0], [1], [], [2, 3], [4], [5]];
function recording(over = {}) {
  const dir = fresh('rec');
  fs.mkdirSync(path.join(dir, 'tsv'), { recursive: true });
  const w = over.w ?? 640, h = over.h ?? 480, rot = over.rot ?? 90;
  const packets = Array.from({ length: 6 }, (_, i) => ({ i, interval: 18.985, ax: 0.5 / (i + 1), ay: 8.75, az: 4.5, a: 4.25 - i * 0.25, b: -0.5, g: 3.25 }));
  const ts = (p) => 100_000_000 + p * 10_000_000;
  const queues = over.emptyBootstrap ? [[], ...QUEUES.slice(1)] : QUEUES;
  const ev = [];
  queues.forEach((q, m) => q.forEach((pi) => {
    const p = packets[pi];
    const row = (kind, x, y, z) => [m, ev.filter((e) => e[0] === m).length, kind, ts(pi) + (over.shiftQueue === m ? -50_000_000 : 0), 18985000, hex(x), hex(y), hex(z)];
    ev.push(row(4, 0, 0, 0), row(1, p.ax, p.ay, p.az), row(2, p.a, p.b, p.g));
  }));
  const wrote = over.dropPacket === undefined ? packets : packets.filter((p) => p.i !== over.dropPacket);
  const inputs = [-1, 3, 4, 5, 6, 7];
  const stages = inputs.map((input, k) => [4 + 3 * k, k, input, 0.4 + 0.1 * k, 0.45 + 0.1 * k, (over.sameTime && k === 3 ? 160 : 150 + 10 * k) * 1e6 + 0.5, 359.5 + k / 10, 63.5 + k / 10, 1.5 + k / 10]);
  const write = (f, text) => fs.writeFileSync(path.join(dir, f), text);
  write('tsv/device.tsv', `scenario\tsyn\nlabel\tsyn-o${rot}\nua\treduced\norientation\t${rot}\nmanufacturer\t\nmodel\tK\nengine_init\t${over.engineInit ?? `${w} ${h} ${rot}`}\nserved_width\t${w}\nserved_height\t${h}\n`);
  write('tsv/stages.tsv', `# header\n${stages.map((s) => [...s, over.stageRot ?? rot, w, h].join('\t')).join('\n')}\n`);
  write('tsv/raw-events.tsv', `# header\n${ev.map((e) => e.join('\t')).join('\n')}\n`);
  write('tsv/packets.tsv', `# header\n${wrote.map((p) => [p.i, p.interval, p.ax, p.ay, p.az, p.a, p.b, p.g].join('\t')).join('\n')}\n`);
  const caps = [[0, 4], [1, 5], [2, 7]];
  const raws = caps.map(([c]) => Buffer.alloc(1024 * 2 * 4, c + 1));
  write('tsv/captures.tsv', `# header\n${caps.map(([c, input], k) => [c, input, over.capRot ?? rot, w, h, over.badSha && k === 1 ? '0'.repeat(64) : sha(raws[k]), `served-${input}.pgm`].join('\t')).join('\n')}\n`);
  caps.forEach(([c], k) => fs.writeFileSync(path.join(dir, `cap-${String(c).padStart(3, '0')}.rgba.gz`), zlib.gzipSync(raws[k])));
  write('meta.json', '{}\n');
  return dir;
}
const tsv = (f) => fs.readFileSync(f, 'utf8').split('\n').filter(Boolean).map((l) => l.split('\t'));
const rel = (p) => path.relative(repo, p);

test('PREP-1: sensors.tsv has one row per packet in stage order, the bootstrap queue as bootstrap rows, and each row carries the orientation of the stage that consumed its queue', () => {
  const rec = recording(), out = fresh('out');
  const prep = prepare({ rec, out, repoRoot: repo });
  const s = tsv(path.join(out, 'sensors.tsv'));
  assert.deepEqual(s.map((r) => r[0]), ['bootstrap', 'run', 'run', 'run', 'run', 'run']);
  assert.equal(s.length, 6, 'packets 0..5, none lost, none invented');
  assert.deepEqual(s.map((r) => Number(r[1])), [100, 110, 120, 130, 140, 150].map((x) => x * 1e6));
  // aig xyz from the accelerometer event, lin xyz from the linear-acceleration event (zeros are values, not '-'), rate abg from the gyro event, interval from packets.tsv
  assert.deepEqual(s[1].slice(2, 5), ['0.25', '8.75', '4.5']);
  assert.deepEqual(s[1].slice(5, 8), ['0', '0', '0']);
  assert.deepEqual(s[1].slice(8, 11), ['4', '-0.5', '3.25']);
  assert.equal(s[1][11], '18.985');
  // orientation: the stage that consumed the queue (stage 0 bootstrap, queue 1 -> stage 1, queue 3 -> stage 3 for packets 2 and 3)
  assert.deepEqual(s[0].slice(12), ['359.5', '63.5', '1.5']);
  assert.deepEqual(s[1].slice(12), ['359.6', '63.6', '1.6']);
  assert.deepEqual(s[2].slice(12), s[3].slice(12));
  assert.deepEqual(s[2].slice(12), ['359.8', '63.8', '1.8']);
  assert.deepEqual(s[4].slice(12), ['359.9', '63.9', '1.9']);
  assert.deepEqual(prep.counts, { stages: 6, bootstrapRows: 1, runRows: 5, frames: 4, keptCaptures: 3, atlasRows: 2, uncapturedFramesInRange: [6], firstInputFrame: 4, lastInputFrame: 7 });
  assert.deepEqual(prep.anomalies, { unmatchedPackets: 0, stagesWithoutPackets: 1, packetsLaterThanTheirStageTime: 0, queueOverlapWithEarlierQueue: 0 });
});

test('PREP-2: frames.tsv has the bootstrap stage (atlas of the first kept capture) and the stages from the first to the last kept capture; times are the truncated recorded ones; column 9 is the frame time', () => {
  const rec = recording(), out = fresh('out');
  prepare({ rec, out, repoRoot: repo });
  const f = tsv(path.join(out, 'frames.tsv'));
  assert.deepEqual(f.map((r) => [r[0], r[1]]), [['bootstrap', '-1'], ['frame', '4'], ['frame', '5'], ['frame', '6'], ['frame', '7']], 'input frame 3 (before the first capture) is not a frame row');
  assert.deepEqual(f.map((r) => Number(r[2])), [150, 170, 180, 190, 200].map((x) => x * 1e6));
  assert.equal(Number(f[1][3]), Math.trunc(0.6 * 1e9)); assert.equal(Number(f[1][8]), Math.trunc(0.65 * 1e9));
  assert.deepEqual(f.map((r) => r[7].split('/').at(-1)), ['cap-000.rgba.gz', 'cap-000.rgba.gz', 'cap-001.rgba.gz', '-', 'cap-002.rgba.gz']);
  assert.ok(f.every((r) => r.length === 9));
  // the config: the engineInit of the recording
  const c = Object.fromEntries(tsv(path.join(out, 'engine-config.tsv')).map(([k, ...v]) => [k, v.join('\t')]));
  assert.deepEqual([c.capture_width, c.capture_height, c.rotation_deg, c.model, c.pyramid_model, c.origin], ['640', '480', '90', 'K', 'swiftshader-approx', '0 2 0']);
  // the arms' reader accepts it, derive turns the frame without an atlas into process = 0, the expected counts follow
  const derived = fresh('derived');
  const d = derive(out, derived, { sourceLabel: rel(out) });
  assert.deepEqual(d.framesWithoutAtlasDisabled, [6]);
  const b = readBundle(derived);
  assert.equal(b.frames.slice(1).filter((x) => x.process).length, 3, 'inputs 4, 5, 7 are processed; 6 has no atlas');
  assert.equal(b.frames.slice(1).length, 4);
});

test('PREP-3: deterministic (two runs, same bytes) and the provenance names the sha256 of every file read and written', () => {
  const rec = recording(), a = fresh('a'), b = fresh('b');
  const pa = prepare({ rec, out: a, repoRoot: repo }), pb = prepare({ rec, out: b, repoRoot: repo });
  for (const f of ['engine-config.tsv', 'sensors.tsv', 'frames.tsv']) assert.equal(sha(fs.readFileSync(path.join(a, f))), sha(fs.readFileSync(path.join(b, f))), f);
  assert.deepEqual(pa.outputSha256, Object.fromEntries(['engine-config.tsv', 'sensors.tsv', 'frames.tsv'].map((f) => [f, sha(fs.readFileSync(path.join(a, f)))])));
  assert.deepEqual(Object.keys(pa.readFiles), ['tsv/device.tsv', 'tsv/stages.tsv', 'tsv/raw-events.tsv', 'tsv/packets.tsv', 'tsv/captures.tsv', 'meta.json']);
  assert.equal(pa.readFiles['tsv/stages.tsv'], sha(fs.readFileSync(path.join(rec, 'tsv/stages.tsv'))));
  assert.equal(JSON.stringify({ ...pa, outputSha256: 0 }).replace(/build\/[^"]*/g, ''), JSON.stringify({ ...pb, outputSha256: 0 }).replace(/build\/[^"]*/g, ''));
});

test('PREP-4: refusals: an unmatched packet, a wrong capture sha256, a capture or stage with another rotation / size than engineInit, an existing output, no bootstrap packet, frame times not increasing, run rows out of order', () => {
  const refuse = (over, re) => assert.throws(() => prepare({ rec: recording(over), out: fresh('out'), repoRoot: repo }), re, JSON.stringify(over));
  refuse({ dropPacket: 2 }, /not in packets\.tsv/);
  refuse({ badSha: true }, /capture 1: sha256 or size mismatch/);
  refuse({ capRot: 180 }, /capture 0: 640x480 rotation 180 differs from engine_init/);
  refuse({ stageRot: 180 }, /stage call 4: 640x480 rotation 180 differs from engine_init/);
  refuse({ engineInit: '480 640 90' }, /served size differs/);
  refuse({ emptyBootstrap: true }, /bootstrap stage has no packets/);
  refuse({ sameTime: true }, /frame times not strictly increasing/);
  refuse({ shiftQueue: 3 }, /run rows out of time order/);
  const rec = recording(), out = fresh('out');
  prepare({ rec, out, repoRoot: repo });
  assert.throws(() => prepare({ rec, out, repoRoot: repo }), /refusing to overwrite/);
});

// ---- the real recordings (skipped when absent) ----
const REC = 'build/slam-refact-full-20261019/orient/original/flip6-s2';
const real = [['v2-b-o0', 480, 640, 0], ['v2-b-o90', 640, 480, 90], ['v2-b-o180', 480, 640, 180], ['v2-b-o270', 640, 480, -90]];
for (const [name, w, h, rot] of real) {
  test(`PREP-5: the real recording ${name}: ${w}x${h} rotation ${rot}, 60 kept captures, every packet matched, the derived bundle has 61 calls and 60 frames`,
    { skip: !fs.existsSync(path.join(repo, REC, name, 'tsv/stages.tsv')) && 'recording not present' }, () => {
      const out = fresh(name);
      const prep = prepare({ rec: path.join(repo, REC, name), out, repoRoot: repo });
      assert.deepEqual(prep.engineInit, { w, h, rotation: rot });
      assert.equal(prep.anomalies.unmatchedPackets, 0);
      assert.equal(prep.anomalies.packetsLaterThanTheirStageTime, 0);
      assert.equal(prep.anomalies.queueOverlapWithEarlierQueue, 0);
      assert.equal(prep.counts.keptCaptures, 60); assert.equal(prep.counts.frames, 60); assert.equal(prep.counts.atlasRows, 660);
      const derived = fresh(`${name}-derived`);
      derive(out, derived, { sourceLabel: rel(out) });
      const b = readBundle(derived);
      assert.equal(b.frames.length, 61); assert.equal(b.frames.slice(1).filter((f) => f.process).length, 60);
      assert.equal(b.sensors.filter((s) => s.bootstrap).length, prep.counts.bootstrapRows);
    });
}
