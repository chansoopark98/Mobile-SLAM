#!/usr/bin/env python3
"""E5: replay bundle (sensors.tsv / frames.tsv / engine-config.tsv) for the engine from one original-SDK run of an E5 sequence.

Thin wrapper around tools/harness/prepare_inputs.py (unchanged): it points the module's RUNTIME/RUNS/BUNDLES at the E5 files and
passes the remaining arguments through. The inputs are exactly the bytes the original run consumed (its engine-input-sensors file)
and the same frame files; the drop mask is the original run's own fresh/not-fresh frame set (trace.jsonl), so engine and
original process the same frames.

  python3 -I prepare_bundle.py --scenarios-file <scenarios.json> --runs-dir <runs root> --bundles-dir <new root> \
      --scenario e5-x --label b1 --sensors <runs>/e5-x/engine-input-sensors.jsonl.gz --drop-mask-from <runs>/e5-x/o1/trace.jsonl
"""
import importlib.util, pathlib, shutil, sys, tempfile

here = pathlib.Path(__file__).resolve().parent
args = sys.argv[1:]


def take(flag):
    if flag not in args:
        sys.exit('missing ' + flag)
    i = args.index(flag)
    v = args[i + 1]
    del args[i:i + 2]
    return pathlib.Path(v).resolve()


scen, runs, bundles = take('--scenarios-file'), take('--runs-dir'), take('--bundles-dir')
spec = importlib.util.spec_from_file_location('prepare_inputs', here.parent / 'prepare_inputs.py')
pi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(pi)
with tempfile.TemporaryDirectory() as tmp:
    shutil.copy(scen, pathlib.Path(tmp) / 'scenarios.json')
    pi.RUNTIME, pi.RUNS, pi.BUNDLES = pathlib.Path(tmp), runs, bundles
    sys.argv = ['prepare_inputs.py'] + args + ['--out-root', str(bundles)]
    pi.main()
