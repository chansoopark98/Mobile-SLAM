#!/usr/bin/env python3
"""Generates diag/run_original_itprobe.mjs from ../run_original.mjs by exact text hunks (each hunk is asserted to apply exactly once, so the
generated runner differs from run_original.mjs only in the listed places; `diff ../run_original.mjs run_original_itprobe.mjs` shows them).
New behavior: --it-probe-wasm <instrumented xr-slam.wasm> (needs --memprobe): the page compiles that module instead of the original bytes through
memprobe.js's existing probeWasmUrl option (served at /__itprobe/xr-probe.wasm by the harness route), itprobe_orig.js reads the log buffer after each
processStagedFrame and the runner writes itprobe.jsonl.gz next to memprobe.jsonl.gz. Outputs may also go under build/slam-refact-full-20261019/it-scale-v4/diag.
usage: make_run_original_itprobe.py   (writes run_original_itprobe.mjs next to this file)"""
import pathlib
HERE = pathlib.Path(__file__).resolve().parent
src = (HERE.parent / 'run_original.mjs').read_text()


def sub(old, new, count=1):
    global src
    assert src.count(old) == count, (src.count(old), old[:90])
    src = src.replace(old, new)


sub("from '../../../../scripts/dev/node_modules/playwright/index.mjs'", "from '../../../../../scripts/dev/node_modules/playwright/index.mjs'")
sub("from '../../../../scripts/dev/replay-android-input.mjs'", "from '../../../../../scripts/dev/replay-android-input.mjs'")
sub("from '../../../../scripts/dev/references/xr_reference/sensor_adapter.mjs'", "from '../../../../../scripts/dev/references/xr_reference/sensor_adapter.mjs'")
sub("from '../../re/runtime/capnp-event-queue.mjs'", "from '../../../re/runtime/capnp-event-queue.mjs'")
sub("path.dirname(fileURLToPath(import.meta.url)), '../../re/runtime');", "path.dirname(fileURLToPath(import.meta.url)), '../../../re/runtime');")
sub("const e5dir = path.dirname(fileURLToPath(import.meta.url));",
    "const diagdir = path.dirname(fileURLToPath(import.meta.url));  // diag copy: itprobe_orig.js lives here, the e5 files one level up\nconst e5dir = path.resolve(diagdir, '..');")
sub("  'image-targets': {type: 'string'},\n}});", "  'image-targets': {type: 'string'},\n  'it-probe-wasm': {type: 'string'},\n}});")
sub("if (values.wrap && values.memprobe) throw",
    "if (values['it-probe-wasm'] && !values.memprobe) throw new Error('--it-probe-wasm requires --memprobe (it reuses memprobe.js probeWasmUrl)');\nif (values.wrap && values.memprobe) throw")
sub("const ownedFull = path.join(root, 'build/slam-refact-pkgh-20261022');  // E5 copy: package H build directory only",
    "const ownedFull = path.join(root, 'build/slam-refact-pkgh-20261022');  // E5 copy: package H build directory only\n"
    "const ownedDiag = path.join(root, 'build/slam-refact-full-20261019/it-scale-v4/diag');  // diag copy: also the it-scale-v4 diag directory")
sub("if (![ownedFull].some(d =>", "if (![ownedFull, ownedDiag].some(d =>")
sub("if (imageTargetsDir && outRoot !== ownedFull && !outRoot.startsWith(ownedFull + path.sep)) throw new Error('--image-targets output must stay under ' + ownedFull);",
    "if (imageTargetsDir && ![ownedFull, ownedDiag].some(d => outRoot === d || outRoot.startsWith(d + path.sep))) throw new Error('--image-targets output must stay under ' + ownedFull + ' or ' + ownedDiag);\n"
    "const itProbeWasm = values['it-probe-wasm'] ? await fs.readFile(path.resolve(root, values['it-probe-wasm'])) : null;\n"
    "if (itProbeWasm && createHash('sha256').update(itProbeWasm).digest('hex') === XR_SLAM_WASM_SHA256) throw new Error('--it-probe-wasm is the unmodified original');")
sub("    if (values['memprobe-detail'] !== undefined) probeOptions.detailUntilSeq = Number(values['memprobe-detail']);\n    await page.addInitScript(`${probeSource}\\nwindow.__memProbeInit(${JSON.stringify(probeOptions)});`);\n",
    "    if (values['memprobe-detail'] !== undefined) probeOptions.detailUntilSeq = Number(values['memprobe-detail']);\n"
    "    if (itProbeWasm) probeOptions.probeWasmUrl = '/__itprobe/xr-probe.wasm';\n"
    "    await page.addInitScript(`${probeSource}\\nwindow.__memProbeInit(${JSON.stringify(probeOptions)});`);\n"
    "    if (itProbeWasm) {  // after memprobe.js (and input-recorder.js): outermost wrapper of c8EmAsm_processStagedFrame\n"
    "      const itSource = await fs.readFile(path.join(diagdir, 'itprobe_orig.js'), 'utf8');\n"
    "      await page.addInitScript(`${itSource}\\nwindow.__itProbeInit();`);\n"
    "      report.itProbe = {wasmSha256: sha(itProbeWasm), wasmBytes: itProbeWasm.length, readerSha256: sha(Buffer.from(itSource))};\n"
    "    }\n")
sub("  await page.route('**/*', route => {\n    const url = route.request().url();\n",
    "  await page.route('**/*', route => {\n    const url = route.request().url();\n"
    "    if (itProbeWasm && url === origin + '/__itprobe/xr-probe.wasm') return route.fulfill({status: 200, contentType: 'application/wasm', body: itProbeWasm});\n")
sub("  const memProbe = values.memprobe ? await page.evaluate(() => window.__memProbe) : null;\n",
    "  const memProbe = values.memprobe ? await page.evaluate(() => window.__memProbe) : null;\n"
    "  const itProbe = itProbeWasm ? await page.evaluate(() => window.__srItProbe) : null;\n")
sub("  if (inputRec) report.recordedInputs = await writeRecordedInputs(inputRec, observed, frames);\n",
    "  if (itProbe) {\n"
    "    const lines = [JSON.stringify({schema: 'slam-refact-xr_reference-itprobe-v1', scenario: values.scenario, label: values.label, wrapped: itProbe.wrapped, error: itProbe.error,\n"
    "      records: itProbe.records.length, probeWasmSha256: report.itProbe.wasmSha256,\n"
    "      layout: 'see diag/instrument_it_wat.py; state = base64(mt19937 x[624] u32 LE + libc++ index u32); i = input frame of the first fresh callback whose last processStagedFrame seq is this record (null for bootstrap/repeat)'})];\n"
    "    for (const rec of itProbe.records) lines.push(JSON.stringify({i: seqToFrame.get(rec.seq) ?? null, ...rec}));\n"
    "    await fs.writeFile(path.join(out, 'itprobe.jsonl.gz'), zlib.gzipSync(lines.join('\\n') + '\\n', {level: 9, mtime: 0}));\n"
    "    report.itProbeResult = {file: 'itprobe.jsonl.gz', records: itProbe.records.length, wrapped: itProbe.wrapped, error: itProbe.error, associated: itProbe.records.filter(r => seqToFrame.has(r.seq)).length};\n"
    "  }\n"
    "  if (inputRec) report.recordedInputs = await writeRecordedInputs(inputRec, observed, frames);\n")
sub("'capnp-event-queue.mjs', 'api-oracle.js']) report.sourceHashes[name]", "'capnp-event-queue.mjs', 'api-oracle.js']) report.sourceHashes[name]")  # unchanged anchor sanity

# --gl-capture <maxCaptures>:<rows>  (pixel-level ROI diagnosis): glcap_orig.js after the other probes; writes glcap/cap-NNN.rgba.gz + glcap.json
sub("  'it-probe-wasm': {type: 'string'},\n}});", "  'it-probe-wasm': {type: 'string'},\n  'gl-capture': {type: 'string'},\n}});")
sub("  if (apiOracle) {\n    const oracleSource",
    "  if (values['gl-capture']) {\n"
    "    const [mc, rw] = values['gl-capture'].split(':').map(Number);\n"
    "    const glSource = await fs.readFile(path.join(diagdir, 'glcap_orig.js'), 'utf8');\n"
    "    await page.addInitScript(`${glSource}\nwindow.__glCapInit(${JSON.stringify({maxCaptures: mc, rows: rw})});`);\n"
    "    report.glCapture = {maxCaptures: mc, rows: rw, readerSha256: sha(Buffer.from(glSource))};\n"
    "  }\n"
    "  if (apiOracle) {\n    const oracleSource")
sub("  const itProbe = itProbeWasm ? await page.evaluate(() => window.__srItProbe) : null;\n",
    "  const itProbe = itProbeWasm ? await page.evaluate(() => window.__srItProbe) : null;\n"
    "  if (values['gl-capture']) {\n"
    "    await fs.mkdir(path.join(out, 'glcap'), {recursive: true});\n"
    "    const n = await page.evaluate(() => window.__glCap.captures.length);\n"
    "    for (let k = 0; k < n; k++) {\n"
    "      const b64 = await page.evaluate(k => { const u = window.__glCap.captures[k]; let s = ''; for (let i = 0; i < u.length; i += 0x8000) s += String.fromCharCode.apply(null, u.subarray(i, i + 0x8000)); return btoa(s); }, k);\n"
    "      await fs.writeFile(path.join(out, 'glcap', `cap-${String(k).padStart(3, '0')}.rgba.gz`), zlib.gzipSync(Buffer.from(b64, 'base64'), {level: 6, mtime: 0}));\n"
    "    }\n"
    "    const meta = await page.evaluate(() => ({events: window.__glCap.events, ops: window.__glCap.ops, stageCalls: window.__glCap.stageCalls, processCalls: window.__glCap.processCalls, wrapped: window.__glCap.wrapped, error: window.__glCap.error}));\n"
    "    await fs.writeFile(path.join(out, 'glcap', 'glcap.json'), JSON.stringify(meta));\n"
    "    report.glCaptureResult = {captures: n, events: meta.events.length, ops: meta.ops.length, wrapped: meta.wrapped, error: meta.error};\n"
    "  }\n")
header = ("// GENERATED by make_run_original_itprobe.py from ../run_original.mjs (do not edit; see that script for the exact hunks). Adds --it-probe-wasm.\n")
(HERE / 'run_original_itprobe.mjs').write_text(header + src)
print('wrote run_original_itprobe.mjs', len(src))
