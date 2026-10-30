// or1-chunk-replay-gate.v4: the input of the rotated / landscape scenarios. A run-orient.mjs recording of the ORIGINAL SDK (flip6-s2, orientation 0 / 90 / 180 /
// 270; extract_orient.py has written its tsv/ files) becomes a SOURCE bundle in the format of tools/harness/prepare_inputs.py (engine-config.tsv, sensors.tsv,
// frames.tsv), which `or1.mjs derive` turns into the derived bundle the arms play. Format conversion only: no value is corrected, filtered or invented.
//
//   node prepare-orient-bundle.mjs --rec <recording dir> --out <NEW source bundle dir under build/> [--label <text>]
//
// What is read (the same files the engine's own orient replay reads, engine/tests/engine/orient_replay.h, whose rules this ports):
//   tsv/device.tsv    engine_init "<captureW> <captureH> <rotation>" (the engineInit the original made), manufacturer, model, served_width/height
//   tsv/stages.tsv    one row per original stageFrame call: the event queue (msg) set before it, the input frame (-1 = bootstrap), videoTime, frameTime, timeNanos,
//                     the stage's deviceorientation alpha/beta/gamma, the capture size and rotation
//   tsv/raw-events.tsv  the capnp event queue of every msg: 3 events per DeviceMotion packet (linear acceleration kind 4, accelerometer kind 1, gyro kind 2), float32 bits
//   tsv/packets.tsv   the DeviceMotion packets as dispatched (doubles; the interval in ms the event queue does not carry in ms)
//   tsv/captures.tsv + cap-NNN.rgba.gz   the original's own GPU pyramid readbacks (rawSha256 checked against the decompressed bytes)
// What is written:
//   sensors.tsv   bootstrap rows = the packets of the bootstrap stage's queue; run rows = the packets of every later queue in stage order, each with the
//                 deviceorientation of the stage that consumed its queue (the engine keeps the latest, so the value the engine has at a stage is the stage's)
//   frames.tsv    the bootstrap stage (its atlas: the first kept capture, a documented simplification as in prepare_inputs.py D-4) and every stage from the first kept
//                 capture to the last one; a stage without a kept capture has pyramid '-' (derive then sets process = 0, the same in every arm).
//                 time = timeNanos (the stage time), video time = videoTimeS * 1e9, column 9 = frameTimeS * 1e9 (what func 647 picks), all truncated like the original
//   engine-config.tsv  capture size and rotation of engine_init, the device of device.tsv, swiftshader-approx pyramid, origin (0, 2, 0) as the original's page session
//   orient-prep.json   provenance: sha256 of every file read, counts, and the anomaly counters (a non-zero count is a documented limitation, see the registration)
// Refuses (exit 1) when a packet cannot be matched to the packet stream (the engine replay skips such a packet; this tool does not), when a capture's sha256 or size
// is wrong, when two frame times are not strictly increasing, or when the run rows are not in time order.
import fs from 'node:fs';
import path from 'node:path';
import zlib from 'node:zlib';
import { createHash } from 'node:crypto';
import { parseArgs } from 'node:util';

const root = path.resolve(import.meta.dirname, '../../../..');
const sha = (b) => createHash('sha256').update(b).digest('hex');
const tsv = (file) => fs.readFileSync(file, 'utf8').split('\n').filter((l) => l && !l.startsWith('#')).map((l) => l.split('\t'));
const f32 = new Float32Array(1), u32 = new Uint32Array(f32.buffer);
const bitsToF = (hex) => { u32[0] = parseInt(hex, 16); return f32[0]; };
const fToBits = (x) => { f32[0] = x; return u32[0]; };

export function prepare({ rec, out, label = null, repoRoot = root }) {
  const rel = (p) => path.relative(repoRoot, path.resolve(p));
  if (fs.existsSync(out)) throw new Error(`refusing to overwrite ${out}`);
  const read = (name) => path.join(rec, name);
  const files = ['tsv/device.tsv', 'tsv/stages.tsv', 'tsv/raw-events.tsv', 'tsv/packets.tsv', 'tsv/captures.tsv', 'meta.json'];
  const device = Object.fromEntries(tsv(read('tsv/device.tsv')).filter((r) => r.length >= 2).map((r) => [r[0], r[1]]));
  const [w, h, rot] = device.engine_init.split(' ').map(Number);
  if (![w, h, rot].every(Number.isFinite)) throw new Error(`engine_init ${device.engine_init}`);
  if (Number(device.served_width) !== w || Number(device.served_height) !== h) throw new Error('served size differs from the engineInit size');
  const stages = tsv(read('tsv/stages.tsv')).map((r) => ({ call: Number(r[0]), msg: Number(r[1]), input: Number(r[2]), videoS: Number(r[3]), frameS: Number(r[4]), timeNanos: Number(r[5]),
    alpha: r[6], beta: r[7], gamma: r[8], rotation: Number(r[9]), width: Number(r[10]), height: Number(r[11]) }));
  if (stages.length < 3 || stages[0].input !== -1) throw new Error('stages.tsv: the first stage must be the bootstrap stage (input frame -1)');
  for (const s of stages) if (s.width !== w || s.height !== h || s.rotation !== rot) throw new Error(`stage call ${s.call}: ${s.width}x${s.height} rotation ${s.rotation} differs from engine_init ${w}x${h} ${rot}`);
  const msgs = [];
  for (const r of tsv(read('tsv/raw-events.tsv'))) (msgs[Number(r[0])] ??= []).push({ kind: Number(r[2]), ts: Number(r[3]), interval: Number(r[4]), x: r[5], y: r[6], z: r[7] });
  const packets = tsv(read('tsv/packets.tsv')).map((r) => ({ interval: Number(r[1]), ax: Number(r[2]), ay: Number(r[3]), az: Number(r[4]), a: Number(r[5]), b: Number(r[6]), g: Number(r[7]) }));

  // ---- the packets of every queue, matched to the packet stream by float32 bits (orient_replay.h) ----
  let cursor = 0, unmatched = 0, queuesWithoutPackets = 0, packetsAfterStageTime = 0, queueOverlap = 0, lastTs = null;
  const rowsOfStage = stages.map((s, si) => {
    const ev = msgs[s.msg] ?? [];
    if (ev.length % 3 !== 0) throw new Error(`msg ${s.msg}: ${ev.length} events is not a multiple of 3`);
    const rows = [];
    for (let i = 0; i < ev.length; i += 3) {
      const [lin, acc, gyr] = [ev[i], ev[i + 1], ev[i + 2]];
      if (lin.kind !== 4 || acc.kind !== 1 || gyr.kind !== 2) throw new Error(`msg ${s.msg} event ${i}: kinds ${lin.kind}/${acc.kind}/${gyr.kind}, expected 4/1/2`);
      let best = -1;
      for (let c = cursor; c < Math.min(packets.length, cursor + 3); c++) {
        const p = packets[c];
        if (fToBits(p.ax) === parseInt(acc.x, 16) && fToBits(p.ay) === parseInt(acc.y, 16) && fToBits(p.az) === parseInt(acc.z, 16)
          && fToBits(p.a) === parseInt(gyr.x, 16) && fToBits(p.b) === parseInt(gyr.y, 16) && fToBits(p.g) === parseInt(gyr.z, 16)) { best = c; }
      }
      if (best < 0) { unmatched++; continue; }
      cursor = best;
      if (acc.ts > s.timeNanos) packetsAfterStageTime++;
      rows.push({ ts: acc.ts, lin: [lin.x, lin.y, lin.z].map(bitsToF), aig: [acc.x, acc.y, acc.z].map(bitsToF), rate: [gyr.x, gyr.y, gyr.z].map(bitsToF), interval: packets[best].interval });
    }
    if (rows.length === 0) queuesWithoutPackets++;
    if (rows.length) {
      if (lastTs !== null && rows[0].ts < lastTs) queueOverlap++;  // a queue starting before the end of an earlier one: the run rows would be out of order
      lastTs = rows.at(-1).ts;
    }
    return rows;
  });
  if (unmatched) throw new Error(`${unmatched} packet(s) of the event queues are not in packets.tsv (orient_replay.h would skip them; this tool refuses)`);

  // ---- captures ----
  const caps = tsv(read('tsv/captures.tsv')).map((r) => ({ capture: Number(r[0]), input: Number(r[1]), rotation: Number(r[2]), w: Number(r[3]), h: Number(r[4]), rawSha: r[5] }));
  const capFile = (c) => path.join(rec, `cap-${String(c.capture).padStart(3, '0')}.rgba.gz`);
  let atlasRows = null;
  for (const c of caps) {
    if (c.w !== w || c.h !== h || c.rotation !== rot) throw new Error(`capture ${c.capture}: ${c.w}x${c.h} rotation ${c.rotation} differs from engine_init`);
    const raw = zlib.gunzipSync(fs.readFileSync(capFile(c)));
    if (sha(raw) !== c.rawSha || raw.length === 0 || raw.length % 4096 !== 0) throw new Error(`capture ${c.capture}: sha256 or size mismatch`);
    if (atlasRows !== null && raw.length / 4096 !== atlasRows) throw new Error(`capture ${c.capture}: ${raw.length / 4096} rows, others ${atlasRows}`);
    atlasRows = raw.length / 4096;
  }
  const capOf = new Map(caps.map((c) => [c.input, c]));
  if (capOf.size !== caps.length) throw new Error('two captures of one input frame');
  const keptInputs = caps.map((c) => c.input).sort((a, b) => a - b);
  if (!keptInputs.length) throw new Error('no kept capture');
  const [firstIn, lastIn] = [keptInputs[0], keptInputs.at(-1)];

  // ---- sensors.tsv ----
  const orientOf = (s) => [s.alpha, s.beta, s.gamma];
  const sensorLines = [];
  const pushRows = (phase, rows, s) => { for (const r of rows) sensorLines.push([phase, r.ts, ...r.aig, ...r.lin, ...r.rate, r.interval, ...orientOf(s)].join('\t')); };
  pushRows('bootstrap', rowsOfStage[0], stages[0]);
  const runStart = sensorLines.length;
  for (let si = 1; si < stages.length; si++) pushRows('run', rowsOfStage[si], stages[si]);
  const runTimes = sensorLines.slice(runStart).map((l) => Number(l.split('\t')[1]));
  for (let i = 1; i < runTimes.length; i++) if (runTimes[i] < runTimes[i - 1]) throw new Error(`run rows out of time order at run row ${i}`);
  if (sensorLines.length === runStart) throw new Error('no run rows');
  if (rowsOfStage[0].length === 0) throw new Error('the bootstrap stage has no packets: the bundle needs a bootstrap sensor row');

  // ---- frames.tsv ----
  const ns = (x) => Math.trunc(x);
  const firstCap = capOf.get(firstIn);
  const atlasRel = (c) => rel(capFile(c));
  const frameLine = (kind, index, s, pyramid) => [kind, index, ns(s.timeNanos), ns(s.videoS * 1e9), (s.timeNanos / 1e9).toString(), 1, rel(path.join(rec, `served-${String(Math.max(index, 0)).padStart(5, '0')}.pgm`)),
    pyramid, ns(s.frameS * 1e9)].join('\t');
  const frameLines = [frameLine('bootstrap', -1, stages[0], atlasRel(firstCap))];
  const kept = stages.filter((s) => s.input >= firstIn && s.input <= lastIn);
  let prev = ns(stages[0].timeNanos);
  const uncaptured = [];
  for (const s of kept) {
    const c = capOf.get(s.input);
    if (!c) uncaptured.push(s.input);
    if (ns(s.timeNanos) <= prev) throw new Error(`frame times not strictly increasing at input frame ${s.input}`);
    prev = ns(s.timeNanos);
    frameLines.push(frameLine('frame', s.input, s, c ? atlasRel(c) : '-'));
  }
  const haveStage = new Set(kept.map((s) => s.input));
  const missingStages = keptInputs.filter((i) => !haveStage.has(i));
  if (missingStages.length) throw new Error(`captures of input frames without a stage row: ${missingStages.join(',')}`);

  // ---- engine-config.tsv ----
  const cfg = [['scenario', device.scenario], ['label', label ?? `${device.label}`], ['capture_width', w], ['capture_height', h], ['rotation_deg', rot],
    ['manufacturer', device.manufacturer ?? ''], ['model', device.model ?? ''], ['estimate_scale', 0], ['disable_vio', 0], ['pyramid_model', 'swiftshader-approx'],
    ['origin', '0 2 0'], ['facing_wxyz', '1 0 0 0'], ['left_handed_axes', 0], ['mirrored_display', 0], ['t0_s', '1.0'], ['world_point_sample_every', 50],
    ['drop_mode', `orient-capture-stage-set:${rel(rec)}`], ['pyramid_injection', rel(rec)]];

  fs.mkdirSync(out, { recursive: true });
  const write = (name, text) => fs.writeFileSync(path.join(out, name), text, { flag: 'wx' });
  write('engine-config.tsv', cfg.map(([k, v]) => `${k}\t${v}\n`).join(''));
  write('sensors.tsv', `${sensorLines.join('\n')}\n`);
  write('frames.tsv', `${frameLines.join('\n')}\n`);
  const prep = { schema: 'or1-orient-prep-v1', recording: rel(rec), label: label ?? device.label, orientation: Number(device.orientation), engineInit: { w, h, rotation: rot },
    device: { manufacturer: device.manufacturer ?? '', model: device.model ?? '', ua: device.ua },
    readFiles: Object.fromEntries(files.map((f) => [f, sha(fs.readFileSync(read(f)))])),
    outputSha256: Object.fromEntries(['engine-config.tsv', 'sensors.tsv', 'frames.tsv'].map((n) => [n, sha(fs.readFileSync(path.join(out, n)))])),
    counts: { stages: stages.length, bootstrapRows: rowsOfStage[0].length, runRows: sensorLines.length - rowsOfStage[0].length, frames: frameLines.length - 1, keptCaptures: caps.length,
      atlasRows, uncapturedFramesInRange: uncaptured, firstInputFrame: firstIn, lastInputFrame: lastIn },
    anomalies: { unmatchedPackets: unmatched, stagesWithoutPackets: queuesWithoutPackets, packetsLaterThanTheirStageTime: packetsAfterStageTime, queueOverlapWithEarlierQueue: queueOverlap },
    simplifications: ['the bootstrap stage uses the atlas of the first kept capture (no capture of the bootstrap stage was kept; prepare_inputs.py D-4)',
      'stages before the first kept capture and after the last one are not in frames.tsv; their packets stay in sensors.tsv (queued for the next processed frame, as for any process = 0 frame)',
      'a stage whose queue had no packet pushes no deviceorientation of its own (the bundle format carries the orientation on packet rows)'] };
  fs.writeFileSync(path.join(out, 'orient-prep.json'), `${JSON.stringify(prep, null, 1)}\n`, { flag: 'wx' });
  return prep;
}

const isMain = process.argv[1] && path.resolve(process.argv[1]) === import.meta.filename;
if (isMain) {
  const { values } = parseArgs({ options: { rec: { type: 'string' }, out: { type: 'string' }, label: { type: 'string' } } });
  try {
    const out = path.resolve(root, values.out ?? '');
    if (!values.out || !out.startsWith(path.join(root, 'build') + path.sep)) throw new Error('--out must be a new directory under build/');
    const prep = prepare({ rec: path.resolve(root, values.rec ?? ''), out, label: values.label ?? null });
    console.log(JSON.stringify({ out: values.out, orientation: prep.orientation, engineInit: prep.engineInit, counts: prep.counts, anomalies: prep.anomalies }));
  } catch (error) {
    console.error(String(error?.message ?? error));
    process.exitCode = 1;
  }
}
