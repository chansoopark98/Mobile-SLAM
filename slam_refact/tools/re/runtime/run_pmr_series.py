#!/usr/bin/env python3
"""Low-load memory-probe reference series (thresholds-registration.md Addendum 6, items 1-2), applied mechanically.

For slot in --slots (default pmr3, pmr4), for scenario in --scenarios: up to 4 attempts labelled <slot>a, <slot>b, ...
(never overwritten; an existing attempt dir is re-read, not re-run). Before EACH attempt: wait until the 1-minute load
average < 12 and no other Playwright browser runs (poll every 15 s; a note at > 20 min; give up after 60 min -> the slot
is BLOCKED). Each attempt = run-reference.mjs --memprobe --record-inputs (one browser). Valid iff meta.status ==
COMPLETED and summary.freshCallbacks >= 0.99 * min(r1, r2 freshCallbacks). The slot ends at its first valid attempt; the
valid attempt is linked as <slot> (symlink) for the existing evaluators. Log lines go to --log; validity to --validity.

  python3 -I slam_refact/tools/re/runtime/run_pmr_series.py --log build/.../runs.log --validity build/.../validity.json
"""
import argparse
import datetime
import json
import os
import pathlib
import subprocess
import time

ROOT = pathlib.Path(__file__).resolve().parents[4]
RUNS = ROOT / 'build/slam-refact-20261015/reference/runs'
SCENARIOS = ['tum-room1-full', 'tum-room1-lowtex', 'flip6-s2', 'flip6-n1', 'flip6-s3-lowtex']
LOAD_MAX, POLL_S, NOTE_S, GIVEUP_S, MAX_ATTEMPTS = 12.0, 15, 20 * 60, 60 * 60, 4


def now():
    return datetime.datetime.now().astimezone().isoformat(timespec='seconds')


def load():
    return [float(x) for x in pathlib.Path('/proc/loadavg').read_text().split()[:3]]


def other_browsers():
    # Playwright-launched Chrome browser processes (renderers/GPU children carry --type=).
    out = subprocess.run(['pgrep', '-af', 'remote-debugging-pipe'], capture_output=True, text=True).stdout
    return [l for l in out.splitlines() if '--type=' not in l and 'pgrep' not in l and 'chrome' in l]


def log(path, msg):
    with open(path, 'a') as f:
        f.write(msg + '\n')
    print(msg, flush=True)


def wait_low_load(logp, tag):
    t0 = time.time()
    noted = False
    while True:
        la, br = load(), other_browsers()
        if la[0] < LOAD_MAX and not br:
            return {'waitedS': round(time.time() - t0), 'loadAtStart': la}
        waited = time.time() - t0
        if waited > NOTE_S and not noted:
            log(logp, f'note  {tag} {now()} load >= {LOAD_MAX} or other browser for > 20 min (load={la} browsers={len(br)}); lead not reachable from this worker, keep waiting')
            noted = True
        if waited > GIVEUP_S:
            return {'waitedS': round(waited), 'blocked': True, 'loadAtGiveUp': la}
        time.sleep(POLL_S)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--slots', nargs='+', default=['pmr3', 'pmr4'])
    ap.add_argument('--scenarios', nargs='+', default=SCENARIOS)
    ap.add_argument('--log', type=pathlib.Path, required=True)
    ap.add_argument('--validity', type=pathlib.Path, required=True)
    a = ap.parse_args()
    validity = json.loads(a.validity.read_text()) if a.validity.exists() else {
        'rule': 'valid iff status COMPLETED and freshCallbacks >= 0.99 * min(r1, r2) freshCallbacks (Addendum 6 item 2)',
        'loadRule': 'start only when 1-min loadavg < 12 and no other Playwright browser; poll 15 s', 'attempts': [], 'slots': {}}
    for slot in a.slots:
        for s in a.scenarios:
            key = f'{s}/{slot}'
            if validity['slots'].get(key, {}).get('state') in ('VALID', 'INVALID-EXHAUSTED', 'BLOCKED'):
                continue
            r = [json.loads((RUNS / s / l / 'meta.json').read_text())['summary']['freshCallbacks'] for l in ('r1', 'r2')]
            need = 0.99 * min(r)
            state, chosen = 'INVALID-EXHAUSTED', None
            for n in range(MAX_ATTEMPTS):
                label = slot + 'abcd'[n]
                d = RUNS / s / label
                if not d.exists():
                    w = wait_low_load(a.log, key)
                    if w.get('blocked'):
                        log(a.log, f'BLOCKED {key} {now()} waited {w["waitedS"]} s load={w["loadAtGiveUp"]}')
                        validity['attempts'].append({'scenario': s, 'slot': slot, 'label': label, 'blocked': True, **w})
                        state = 'BLOCKED'
                        break
                    log(a.log, f'start {s}/{label} {now()} load={" ".join(map(str, load()))} waited={w["waitedS"]}s')
                    with open(RUNS / f'{s}-{label}.log', 'w') as lf:
                        rc = subprocess.run(['node', 'slam_refact/tools/re/runtime/run-reference.mjs', '--scenario', s, '--label', label,
                                             '--memprobe', '--record-inputs'], cwd=ROOT, stdout=lf, stderr=subprocess.STDOUT).returncode
                    log(a.log, f'end   {s}/{label} {now()} exit={rc} load={" ".join(map(str, load()))}')
                meta = json.loads((d / 'meta.json').read_text())
                fresh = (meta.get('summary') or {}).get('freshCallbacks')
                ok = meta.get('status') == 'COMPLETED' and fresh is not None and fresh >= need
                validity['attempts'].append({'scenario': s, 'slot': slot, 'label': label, 'status': meta.get('status'), 'fresh': fresh,
                                             'needFresh': round(need, 2), 'r1r2Fresh': r, 'loadAvgAtStart': meta.get('loadAvgAtStart'),
                                             'loadAvgAtEnd': meta.get('loadAvgAtEnd'), 'startedAt': meta.get('startedAt'),
                                             'finishedAt': meta.get('finishedAt'), 'recordedInputs': meta.get('recordedInputs'),
                                             'memprobeRecords': (meta.get('memprobe') or {}).get('records'), 'valid': ok})
                log(a.log, f'valid {s}/{label} {ok} fresh={fresh} need>={need:.2f} status={meta.get("status")}')
                a.validity.write_text(json.dumps(validity, indent=1) + '\n')
                if ok:
                    state, chosen = 'VALID', label
                    link = RUNS / s / slot
                    if not link.exists():
                        os.symlink(label, link)
                    break
            validity['slots'][key] = {'state': state, 'chosen': chosen}
            a.validity.write_text(json.dumps(validity, indent=1) + '\n')
    refs = {}
    for s in a.scenarios:
        valid = [x['label'] for x in validity['attempts'] if x['scenario'] == s and x.get('valid')]
        refs[s] = {'reference': valid[:2], 'state': 'OK' if len(valid) >= 2 else 'BLOCKED (fewer than two valid runs)'}
    validity['reference'] = refs
    a.validity.write_text(json.dumps(validity, indent=1) + '\n')
    log(a.log, 'ALLDONE ' + now())


if __name__ == '__main__':
    main()
