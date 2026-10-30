#!/usr/bin/env python3
"""RM-4c mutation run (gate rm4c-chunk-gate.v1.md section 3). Copies of slam_refact/web-chunk go to --out (under build/),
one single-site edit per copy, then the node contract tests (and the Python tool tests) run against the copy via
SLAM_REFACT_CHUNK_DIR / SLAM_REFACT_REPORT_PY. A mutant is DETECTED when at least one test fails.

  python3 -I slam_refact/web-chunk/tests/mutants/run-mutants.py --out build/.../rm4c-20261020/mutants [--only R1,S1]
"""
import argparse
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[4]
SRC = ROOT / 'slam_refact/web-chunk'
TEST = SRC / 'tests/chunk-contract.test.mjs'
PYTEST = SRC / 'tests'
REPORT = ROOT / 'slam_refact/tools/re/runtime/chunk_ab_report.py'
X, P, S, IT = 'xr-slam.js', 'gr8-compute-pyramid.js', 'sensors.js', 'image-targets.js'

M = {
    # ---- A: RM-4a3 (reviewer mutants R1-R4 + the fixes) ----
    'R1-recenter-sends-recenter-point': [(X, "s.glue.updateCameraProjectionMatrix({ origin: { ...configured_.origin }, facing: { ...configured_.facing }, updateRecenterPoint: false });\n        s.liveStale = false;",
        "s.glue.updateCameraProjectionMatrix({ origin: { ...recenterOrigin_ }, facing: { ...recenterFacing_ }, updateRecenterPoint: false });\n        s.liveStale = false;")],
    'R2-resend-configured-not-js': [(X, "s.glue.updateCameraProjectionMatrix({ origin: { ...origin_ }, facing: { ...facing_ }, updateRecenterPoint: false });\n    s.liveStale = false;",
        "s.glue.updateCameraProjectionMatrix({ origin: { ...configured_.origin }, facing: { ...configured_.facing }, updateRecenterPoint: false });\n    s.liveStale = false;")],
    'R3-yrule-uses-js-origin': [(X, "py > 1e-3 ? py : configured_.origin.y", "py > 1e-3 ? py : origin_.y")],
    'R4-orientation-uses-configured': [(X, "coords: kind === 'videoSize' ? configured_ : { origin: origin_, facing: facing_ }, framesUntilRecenter", "coords: configured_, framesUntilRecenter")],
    'R5-first-attach-uses-js-origin': [(X, "rotation, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: startupFramesLeft_ });", "rotation, computeCtx: computeCtx_, coords: { origin: origin_, facing: facing_ }, framesUntilRecenter: startupFramesLeft_ });")],
    'R6-recenter-without-session-keeps-pose': [(X, "    if (!session_) lastExtrinsic_ = null;\n", "")],
    'R7-gap-after-inactive-repeats-old-normal': [(X, "      if (!s.lastDesc && heldInactive_) return inactiveAnswer(framework, frameStartResult);\n", "")],
    'R8a-rebuild-while-context-lost': [(X, "    if (computeContextLost(computeCtx || computeCtx_)) {\n      deferredRebuild_ = { kind, videoWidth, videoHeight, orientation, computeCtx };\n      pushEntry(diag.sessionLog, { event: 'deferred', kind, reason: 'context lost' });\n      return;\n    }\n", "")],
    'R8b-lost-without-session-answers-limited': [(X, "return computeContextLost(deferredRebuild_ && deferredRebuild_.computeCtx || computeCtx_) ? null : inactiveAnswer(framework, frameStartResult);", "return inactiveAnswer(framework, frameStartResult);")],
    'R9-glue-replaced-only-at-load': [(X, "    if (createGlue) engine_.loaded.glueReplaced = true;\n", "")],
    'R10-provenance-does-not-check-bytes': [('tools/record-provenance.mjs', "const matchesManifest = actual.js.sha256 === actual.js.manifestSha256 && actual.wasm.sha256 === actual.wasm.manifestSha256;", "const matchesManifest = true;")],
    # ---- B1 sensors ----
    'S1-swap-acceleration-and-gravity': [(S, "    acceleration: e.acceleration,\n    accelerationIncludingGravity: e.accelerationIncludingGravity,", "    acceleration: e.accelerationIncludingGravity,\n    accelerationIncludingGravity: e.acceleration,")],
    'S2-swap-alpha-beta': [(S, "    rotationRate: e.rotationRate,", "    rotationRate: e.rotationRate && { alpha: e.rotationRate.beta, beta: e.rotationRate.alpha, gamma: e.rotationRate.gamma },")],
    'S3-skip-event-without-gravity-or-rate': [(S, "  const ns = e.interval * 1000 * 1000;", "  if (!e.accelerationIncludingGravity || !e.rotationRate) return false;\n  const ns = e.interval * 1000 * 1000;")],
    'S4-no-interval-guard': [(S, "  if (!Number.isNaN(ns) && !(Math.abs(ns) / TWO_POW_32 <= TWO_POW_32)) return false;\n", "")],
    # ---- B2 startup recenter ----
    'F1-never-pass-count': [(X, "    if (framesUntilRecenter !== null) glueConfig.framesUntilRecenter = framesUntilRecenter;\n", "")],
    'F2-always-5': [(X, "glueConfig.framesUntilRecenter = framesUntilRecenter;", "glueConfig.framesUntilRecenter = 5;")],
    'F3-always-0': [(X, "coords: kind === 'videoSize' ? configured_ : { origin: origin_, facing: facing_ }, framesUntilRecenter: startupFramesLeft_ });", "coords: kind === 'videoSize' ? configured_ : { origin: origin_, facing: facing_ }, framesUntilRecenter: 0 });")],
    'F4a-detach-keeps-count': [(X, "    startupFramesLeft_ = null;  // engineCleanup: the next engineInit starts with a fresh count\n", "")],
    # ---- B3 image targets ----
    'I1-also-dispatch-engine-image-events': [(X, "    if (!pose.repeated && pose.imageEvents && pose.imageEvents.length) diag.stats.engineImageEventsIgnored += pose.imageEvents.length;",
        "    if (!pose.repeated && pose.imageEvents && pose.imageEvents.length) { diag.stats.engineImageEventsIgnored += pose.imageEvents.length; pose.imageEvents.forEach((e) => framework_ && framework_.dispatchEvent(e.name, e.detail)); }")],
    'I2-no-js-dispatch': [(X, "    dispatchImageEvents(detectedImageMap_, result.detectedImages, (name, detail) => framework.dispatchEvent(name, detail));", "")],
    'I3-unknown-names-pass': [(IT, "const stored = targetMetadata_.find((it) => it.name === msg.name);", "const stored = targetMetadata_.find((it) => it.name === msg.name) || { name: msg.name, metadata: null, properties: {} };")],
    'I4-geometry-swapped': [(IT, "scaledWidth: properties.isRotated ? 1.0 : (properties.width / properties.height),", "scaledWidth: properties.isRotated ? (properties.width / properties.height) : 1.0,")],
    'I5-no-reload-into-new-glue': [(IT, "      replay_ = loaded_.size ? { glue, framework, pending: [...first, ...names.filter((n) => !detecting.includes(n))] } : null;\n      if (replay_) pumpReplay(glue, attachBudgetMs);", "      replay_ = null;")],
    'I6-no-unload': [(IT, "    if (glue && names.length) glue.unloadImageTargets(names);\n", "")],
    'I7-no-limit': [(IT, "if (Array.isArray(data) && !cameraProcessingDisabled && data.length > MAX_ACTIVE_IMAGE_TARGETS) {", "if (false) {")],
    'I8-inactive-keeps-detections': [(X, "trackingStatus: 'LIMITED', trackingReason: 'UNSPECIFIED', worldPoints: [], detectedImages: [],", "trackingStatus: 'LIMITED', trackingReason: 'UNSPECIFIED', worldPoints: [],")],
    'I9-limit-checked-after-other-options': [(X, "    if (args.imageTargetData !== undefined) imageTargets.precheck(args.imageTargetData, { cameraProcessingDisabled: cameraProcessingDisabled_ });\n", "")],
    'I10-failed-load-keeps-metadata': [(IT, "    targetMetadata_ = targetMetadata_.filter((it) => it.name !== target.name);\n    scanningIfDone(framework);", "    scanningIfDone(framework);")],
    'I11-failed-load-no-scanning': [(IT, "    targetMetadata_ = targetMetadata_.filter((it) => it.name !== target.name);\n    scanningIfDone(framework);", "    targetMetadata_ = targetMetadata_.filter((it) => it.name !== target.name);")],
    'I12-stale-batch-still-loads': [
        (IT, "    if (batch !== batchId_) { removeId(id); return; }\n    while (imagesToLoad_[0] !== id) {  // wait until id is first in the queue\n      await nextFrame();\n      if (batch !== batchId_) { removeId(id); return; }\n    }",
         "    while (imagesToLoad_[0] !== id) {  // wait until id is first in the queue\n      await nextFrame();\n    }"),
        (IT, "    if (batch !== batchId_) { imagesToLoad_.shift(); return; }\n    const { properties } = target;", "    const { properties } = target;")],
    # ---- B4 ROI draw ----
    'O1-no-warp-draw': [(P, "      gl.uniformMatrix4fv(s1.loc('mvp'), false, roi.warp);\n      gl.uniform1i(s1.loc('clear'), 0);\n      gl.uniform1i(s1.loc('isCurvy'), 0);\n      quad();\n      this.stats.roiDraws++;", "      this.stats.roiDraws++;")],
    'O2-identity-instead-of-warp': [(P, "gl.uniformMatrix4fv(s1.loc('mvp'), false, roi.warp);", "gl.uniformMatrix4fv(s1.loc('mvp'), false, identity);")],
    'O3-wrong-slot': [(P, "      const l = slots[roi.slot];\n", "      const l = slots[(roi.slot + 1) % slots.length];\n")],
    'O4-rows-not-extended': [(P, "    const rows = rois.length ? this.roiRows : this.rows;", "    const rows = this.rows;")],
    'O5-no-clear-before-warp': [(P, "      gl.viewport(l.c, l.r, l.w, l.h);\n      gl.uniformMatrix4fv(s1.loc('mvp'), false, identity);\n      gl.uniform1i(s1.loc('clear'), 1);\n      quad();\n      if (roi.source === 'CURVY_IMAGE_TARGET')", "      gl.viewport(l.c, l.r, l.w, l.h);\n      if (roi.source === 'CURVY_IMAGE_TARGET')")],
    'O6-no-iscurvy': [(P, "      gl.uniform1i(s1.loc('isCurvy'), 0);\n", "")],
    'O7-no-padding': [(X, "    if (rows < s.layout.roiAtlasRows && s.glue.getRequestedRois().length) {", "    if (false) {")],
    'O8-stage-without-rois': [(X, "    s.pyramid.stage(frame.computeTexture, frame.rois);", "    s.pyramid.stage(frame.computeTexture);")],
    # O9 (RM-4c: "ask the engine without targets") became the fix in RM-4d; its inverse is T1, the code the RM-4c review found defective
    'T1-stage-asks-engine-only-with-targets': [(X, "    frame.rois = s.glue.getRequestedRois();", "    frame.rois = imageTargets.hasLoaded() ? s.glue.getRequestedRois() : [];")],
    'T3-rm4c-defect-both-gated-on-loaded-targets': [(X, "    frame.rois = s.glue.getRequestedRois();", "    frame.rois = imageTargets.hasLoaded() ? s.glue.getRequestedRois() : [];"),
        (X, "    if (rows < s.layout.roiAtlasRows && s.glue.getRequestedRois().length) {", "    if (rows < s.layout.roiAtlasRows && imageTargets.hasLoaded() && s.glue.getRequestedRois().length) {")],
    'T2-padding-needs-loaded-targets': [(X, "    if (rows < s.layout.roiAtlasRows && s.glue.getRequestedRois().length) {", "    if (rows < s.layout.roiAtlasRows && imageTargets.hasLoaded() && s.glue.getRequestedRois().length) {")],
    # ---- RM-4d: replay of the loaded targets into a rebuilt glue, deferred rebuild, review mutants K1 K3 K8 X1 ----
    'N1-replay-rethrows-engine-refusal': [(IT, "        rejectLoaded(entry, error, q.framework);", "        throw error;")],
    'N2-replay-never-pumped': [(X, "    imageTargets.pump();  // a pending replay of the loaded targets into the rebuilt glue goes on, one time budget per frame\n", "")],
    'N3-replay-ignores-budget': [(IT, "      if (performance.now() - t0 >= budget) break;\n", "")],
    'N4-refused-target-stays-loaded': [(IT, "    loaded_.delete(entry.name);\n    currentTargetUrls_ = currentTargetUrls_.filter((u) => u !== entry.imagePath);", "    currentTargetUrls_ = currentTargetUrls_.filter((u) => u !== entry.imagePath);")],
    'N5-deferred-rebuild-never-resumed': [(X, "if (!session_ && deferredRebuild_ && !computeContextLost(deferredRebuild_.computeCtx || computeCtx_)) {", "if (false) {")],
    'N6-replay-inputs-unfiltered': [(IT, ".filter((t) => !(replay_ && replay_.pending.includes(t.name)))", ".filter(() => true)")],
    'N7-infinity-budget-per-target': [(IT, "    if (budget === Infinity) {", "    if (false) {")],
    'N8-detach-keeps-deferred-rebuild': [(X, "    heldInactive_ = false;\n    deferredRebuild_ = null;\n    lastExtrinsic_ = null;", "    heldInactive_ = false;\n    lastExtrinsic_ = null;")],
    'N9-refused-target-not-removed-from-urls': [(IT, "    currentTargetUrls_ = currentTargetUrls_.filter((u) => u !== entry.imagePath);\n    loadFailed({ name: entry.name }", "    loadFailed({ name: entry.name }")],
    'K1-destroy-keeps-heldInactive': [(X, "if (s.lastDesc) { heldDesc_ = s.lastDesc; heldInactive_ = false; }", "if (s.lastDesc) { heldDesc_ = s.lastDesc; }")],
    'K3-interval-boundary': [(S, "!(Math.abs(ns) / TWO_POW_32 <= TWO_POW_32)", "!(Math.abs(ns) / TWO_POW_32 < TWO_POW_32)")],
    'K8-padding-drops-pyramid': [(X, "      padded.set(atlas);\n", "")],
    'X1-attach-always-fresh-count': [(X, "rotation, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: startupFramesLeft_ });", "rotation, computeCtx: computeCtx_, coords: configured_, framesUntilRecenter: null });")],
    # one float32 ulp of a coordinate the engine is given (the bit-equality / determinism tier of the gate; reviewer mutants X2, X3, X4)
    'X2-recenter-C-one-ulp': [(X, "s.glue.updateCameraProjectionMatrix({ origin: { ...configured_.origin }, facing: { ...configured_.facing }, updateRecenterPoint: false });\n        s.liveStale = false;",
        "s.glue.updateCameraProjectionMatrix({ origin: { ...configured_.origin, x: configured_.origin.x + 2.4e-7 }, facing: { ...configured_.facing }, updateRecenterPoint: false });\n        s.liveStale = false;")],
    'X3-resend-P-one-ulp': [(X, "s.glue.updateCameraProjectionMatrix({ origin: { ...origin_ }, facing: { ...facing_ }, updateRecenterPoint: false });\n    s.liveStale = false;",
        "s.glue.updateCameraProjectionMatrix({ origin: { ...origin_, y: origin_.y + 2.4e-7 }, facing: { ...facing_ }, updateRecenterPoint: false });\n    s.liveStale = false;")],
    'X4-no-clear-of-unused-slots': [(P, "      if (done.has(i)) return;", "      if (true) return;")],
    'P1-provenance-build-scope-blind': [('tools/record-provenance.mjs', ".filter(([f, h]) => now.has(f) && now.get(f) !== h).map(([f]) => f);", ".filter(() => false).map(([f]) => f);")],
    'P2-provenance-names-any-provenance': [('tools/record-provenance.mjs', "manifestNamesThisProvenance: sha(provenanceBytes) === provenanceRef.sha256", "manifestNamesThisProvenance: true")],
    # ---- sub-resolution mutant of the exact (bit-equality) node tests: the origin reset one float32 ulp off ----
    'U1-reset-origin-one-ulp-off': [(X, "    const [px, py, pz] = e.position;\n    const [w, x, y, z] = e.rotationWxyz;", "    const [px0, py, pz] = e.position;\n    const px = px0 + 1.2e-7;\n    const [w, x, y, z] = e.rotationWxyz;")],
    # ---- RM-4e: the v3 review's mutants MV1, MV2, MV6 (survived in RM-4d) and the replay-order change of review A-5 ----
    'MV1-deferred-kind-lost': [(X, "      buildSession(d.kind, d);", "      buildSession('orientation', d);")],
    'MV2-reinit-keeps-stale-deferred': [(X, "    destroySession();\n    deferredRebuild_ = null;\n    const unsupported", "    destroySession();\n    const unsupported")],
    'MV6-stage-draws-first-roi-only': [(X, "    frame.rois = s.glue.getRequestedRois();", "    frame.rois = s.glue.getRequestedRois().slice(0, 1);")],
    'V5a-attach-ignores-detected-targets': [(X, "imageTargets.attach(glue, framework_, Object.keys(detectedImageMap_));", "imageTargets.attach(glue, framework_);")],
    'V5b-detected-targets-replayed-last': [(IT, "pending: [...first, ...names.filter((n) => !detecting.includes(n))]", "pending: [...names.filter((n) => !detecting.includes(n)), ...first]")],
    # GL-only (RM-4e): one stage, one ROI draw skipped - the fake-GL node tests never reach stage 57; the GL-E6 run does (gate v4 section 3).
    'P6-one-stage-one-roi-draw-skipped': [(P, "      quad();\n      this.stats.roiDraws++;", "      if (!(this.stats.staged === 57 && roi.slot === 3)) quad();\n      this.stats.roiDraws++;")],
    # ---- Python report check (RM4C-A5b) ----
    'R11-report-fail-open': [('PYREPORT', "    return engine.get('matchesManifest') is True and engine.get('glueReplaced') is False", "    return engine.get('matchesManifest') is True and not engine.get('glueReplaced')")],
}


GL_ONLY = {'P6-one-stage-one-roi-draw-skipped'}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--only')
    a = ap.parse_args()
    out = pathlib.Path(a.out).resolve()
    if not str(out).startswith(str(ROOT / 'build') + os.sep):
        sys.exit('--out must be under build/')
    out.mkdir(parents=True, exist_ok=True)
    only = set(a.only.split(',')) if a.only else None
    results = {}
    for name, edits in M.items():
        if only and name.split('-')[0] not in only and name not in only:
            continue
        d = out / name
        if d.exists():
            shutil.rmtree(d)
        shutil.copytree(SRC, d, ignore=shutil.ignore_patterns('__pycache__', 'mutants'))
        report_copy = None
        for f, old, new in edits:
            if f == 'PYREPORT':
                report_copy = d / 'chunk_ab_report.py'
                shutil.copy(REPORT, report_copy)
                path = report_copy
            else:
                path = d / f
            text = path.read_text()
            if text.count(old) != 1:
                sys.exit(f'{name}: pattern found {text.count(old)} times in {f}')
            path.write_text(text.replace(old, new))
        env = dict(os.environ, SLAM_REFACT_CHUNK_DIR=str(d))
        if name in GL_ONLY:  # the copy is the product (served by run-chunk-gl.mjs --chunk-dir); the node tests are not expected to fail on it
            results[name] = {'detected': None, 'gl_only': True, 'failed': []}
            print('GL-ONLY   ', name, flush=True)
            continue
        if report_copy:
            r = subprocess.run([sys.executable, '-I', '-m', 'unittest', 'discover', '-s', str(PYTEST), '-p', 'test_chunk_tools.py'], cwd=ROOT,
                               env=dict(env, SLAM_REFACT_REPORT_PY=str(report_copy)), capture_output=True, text=True)
            (d / 'test.log').write_text(r.stdout + r.stderr)
            failed = re.findall(r'^(?:FAIL|ERROR): (\S+)', r.stderr, re.M)
        else:
            r = subprocess.run(['node', '--test', str(TEST)], cwd=ROOT, env=env, capture_output=True, text=True)
            (d / 'test.log').write_text(r.stdout + r.stderr)
            failed = re.findall(r'^not ok \d+ - (.*)$', r.stdout, re.M)
        results[name] = {'detected': r.returncode != 0, 'failed': [x[:60] for x in failed]}
        print(('DETECTED  ' if r.returncode else 'UNDETECTED'), name, '|', len(failed), [x[:14] for x in failed], flush=True)
    (out / 'results.json').write_text(json.dumps(results, indent=1) + '\n')
    undetected = [k for k, v in results.items() if v['detected'] is False]
    print(json.dumps({'mutants': len(results), 'undetected': undetected}))
    sys.exit(1 if undetected else 0)


if __name__ == '__main__':
    main()
