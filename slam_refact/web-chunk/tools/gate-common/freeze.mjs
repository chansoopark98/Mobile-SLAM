// Evidence hygiene of a finished gate run (OR-1 v2 post-run review F-10; rm4c / hand F-6 class): SHA256SUMS over every file of the output tree, then the
// whole tree read-only (files 0444, directories 0555). The SUMS file lists every file below `dir` except itself; it is written exclusively.
import { createHash } from 'node:crypto';
import fs from 'node:fs';
import path from 'node:path';

export const sha256File = (file) => createHash('sha256').update(fs.readFileSync(file)).digest('hex');

export function listFiles(dir, rel = '') {
  const out = [];
  for (const name of fs.readdirSync(path.join(dir, rel)).sort()) {
    const r = rel ? `${rel}/${name}` : name;
    const st = fs.lstatSync(path.join(dir, r));
    if (st.isDirectory()) out.push(...listFiles(dir, r));
    else if (st.isFile()) out.push(r);
    else throw new Error(`${path.join(dir, r)}: neither a regular file nor a directory`);
  }
  return out;
}

export function writeSums(dir, name = 'SHA256SUMS') {
  const files = listFiles(dir).filter((f) => f !== name);
  const text = files.map((f) => `${sha256File(path.join(dir, f))}  ${f}\n`).join('');
  fs.writeFileSync(path.join(dir, name), text, { flag: 'wx' });
  return { name, files: files.length, sha256: createHash('sha256').update(text).digest('hex') };
}

// chmod a-w for every file and directory below (and including) `dir`; returns the number of entries changed
export function makeReadOnly(dir) {
  let n = 0;
  const walk = (d) => {
    for (const name of fs.readdirSync(d)) {
      const p = path.join(d, name);
      if (fs.lstatSync(p).isDirectory()) walk(p);
      else { fs.chmodSync(p, 0o444); n++; }
    }
    fs.chmodSync(d, 0o555);
    n++;
  };
  walk(dir);
  return n;
}

// for tests and cleanup: undo makeReadOnly
export function makeWritable(dir) {
  if (!fs.existsSync(dir)) return;
  const walk = (d) => {
    fs.chmodSync(d, 0o755);
    for (const name of fs.readdirSync(d)) {
      const p = path.join(d, name);
      if (fs.lstatSync(p).isDirectory()) walk(p); else fs.chmodSync(p, 0o644);
    }
  };
  walk(dir);
}

// copies `files` (absolute) into dir/<subdir>/<basename>; returns [{name, source, sha256}] (the registered-file copies of a gate run)
export function copyInto(dir, subdir, files) {
  fs.mkdirSync(path.join(dir, subdir), { recursive: true });
  return files.map((f) => {
    const dest = path.join(dir, subdir, path.basename(f));
    fs.copyFileSync(f, dest, fs.constants.COPYFILE_EXCL);
    return { name: path.basename(f), source: f, sha256: sha256File(dest) };
  });
}

// chmod a-w for single files (the registered originals: gate file, designation, review); a failure is reported, not thrown
export function readOnlyFiles(files) {
  return files.map((f) => {
    try {
      fs.chmodSync(f, fs.statSync(f).mode & 0o555);
      return { file: f, ok: true };
    } catch (error) {
      return { file: f, ok: false, error: String(error.message) };
    }
  });
}
