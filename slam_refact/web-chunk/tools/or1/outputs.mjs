// What an OR-1 run leaves on disk (OR-1 v2 post-run review F-1, F-8; v3 list C12 items 1, 4):
//   rows-<arm>-<rep>.json     the canonical output rows of every repetition of every arm (as v2)
//   inputs-<W|C>-<rep>.json   the per-call engine input records of the two WASM arms: the raw record and its canonical form (IN-CW compares the canonical
//                             forms), so IN-CW can be recomputed after the run, which v2 could not
//   answers-C-<rep>.json      what the chunk answered the app for every call (ANS compares these with the rows)
//   report.json               meta, the gates (incl. C4 / C6 / C7 / C8), and the sha256 of every file above
// `rowsSha256` is the sha256 of JSON.stringify(rows) in memory (the v2 definition); it is NOT the sha256 of the file (the file has a trailing newline):
// `files` holds the sha256 of every written file, which is what SHA256SUMS lists.
import fs from 'node:fs';
import path from 'node:path';
import { sha256 } from './canon.mjs';
import { comparableFrame } from './players.mjs';

const fileSha = (file) => sha256(fs.readFileSync(file));
const write = (dir, name, value) => {
  const file = path.join(dir, name);
  fs.writeFileSync(file, `${JSON.stringify(value)}\n`, { flag: 'wx' });
  return fileSha(file);
};

export function writeRunOutputs({ outDir, runs, g, meta }) {
  const rowsSha256 = Object.fromEntries(Object.entries(runs).map(([a, rs]) => [a, rs.map((r) => sha256(JSON.stringify(r.rows)))]));
  const files = { rows: {}, inputs: {}, answers: {} };
  for (const [arm, rs] of Object.entries(runs)) {
    rs.forEach((r, i) => {
      files.rows[`rows-${arm}-${i}.json`] = write(outDir, `rows-${arm}-${i}.json`, r.rows);
      if (r.frames) {
        files.inputs[`inputs-${arm}-${i}.json`] = write(outDir, `inputs-${arm}-${i}.json`, { schema: 'or1-inputs-v1', arm, rep: i, glueConfig: r.glueConfig ?? null,
          calls: r.frames.map((f, call) => ({ call, raw: f, canonical: comparableFrame(f) })) });
      }
      if (r.answers) files.answers[`answers-${arm}-${i}.json`] = write(outDir, `answers-${arm}-${i}.json`, { schema: 'or1-answers-v1', arm, rep: i, answers: r.answers });
    });
  }
  const report = { meta, gates: g, rowsSha256, rowsSha256Definition: 'sha256 of JSON.stringify(rows) in memory; the file sha256 is in `files`', files };
  fs.writeFileSync(path.join(outDir, 'report.json'), `${JSON.stringify(report, null, 1)}\n`, { flag: 'wx' });
  return report;
}
