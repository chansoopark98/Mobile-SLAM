#!/usr/bin/env python3
"""US-025 LC-6b: bit-level equality of two engine replay traces (trace.jsonl), ignoring only the timing field(s) listed in TIMING.
Used to show that the diagnostic engine copy (make_diag_engine.sh) behaves like the reference binary on the same bundle.
usage: lc6b_tracecmp.py <trace A> <trace B>   -> prints JSON {rows, differingFields, firstDifferingRow}; exit 0 when identical"""
import json, sys
TIMING = {'srStageMs'}
a = [json.loads(l) for l in open(sys.argv[1])]
b = [json.loads(l) for l in open(sys.argv[2])]
diff, first = {}, None
for k, (x, y) in enumerate(zip(a, b)):
    for f in set(x) | set(y):
        if f not in TIMING and x.get(f) != y.get(f):
            diff[f] = diff.get(f, 0) + 1
            first = k if first is None else first
res = {'rows': [len(a), len(b)], 'ignoredTimingFields': sorted(TIMING), 'differingFields': diff, 'firstDifferingRow': first}
print(json.dumps(res))
sys.exit(0 if len(a) == len(b) and not diff else 1)
