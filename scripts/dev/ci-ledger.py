#!/usr/bin/env python3
"""Compare a JUnit result with the known-failure ledger and append one line to an append-only ledger.

Exit 0 only when the failing set equals the known set (every message fingerprint matches), at least one test
ran, and the JUnit file exists. A known failure that now passes is reported as a mismatch so the record gets updated.
Skipped tests are counted and never treated as passes.
"""
import argparse
import json
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


def read_junit(path):
    """Return (passed, failed{name: text}, skipped[name]) from ctest or node junit output."""
    root = ET.parse(path).getroot()
    passed, failed, skipped = [], {}, []
    for case in root.iter('testcase'):
        name = case.get('name', '')
        if case.find('skipped') is not None or case.get('status') in ('notrun', 'skipped'):
            skipped.append(name)
        elif case.find('failure') is not None or case.find('error') is not None or case.get('status') == 'fail':
            text = []
            for tag in ('failure', 'error', 'system-out', 'system-err'):
                for node in case.findall(tag):
                    text.append(node.get('message') or '')
                    text.append(node.text or '')
            failed[name] = '\n'.join(text)
        else:
            passed.append(name)
    return passed, failed, skipped


def load_known(path):
    if not path:
        return {}
    data = json.load(open(path))
    return {entry['name']: entry['fingerprint'] for entry in data['failures']}


def evaluate(passed, failed, skipped, known):
    problems = []
    if len(passed) + len(failed) + len(skipped) == 0:
        problems.append('no tests ran')
    for name in sorted(set(failed) - set(known)):
        problems.append(f'new failure: {name}')
    for name in sorted(set(known) - set(failed)):
        problems.append(f'known failure no longer fails (update the record): {name}')
    for name in sorted(set(failed) & set(known)):
        if known[name] not in failed[name]:
            problems.append(f'fingerprint mismatch: {name}')
    return problems


def git(*args):
    try:
        return subprocess.run(['git', *args], capture_output=True, text=True, check=True).stdout.strip()
    except Exception:
        return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--junit', required=True)
    parser.add_argument('--mode', required=True)
    parser.add_argument('--known', help='known-failures.json; omitted means no failure is expected')
    parser.add_argument('--ledger', default='build/ci-ledger/ledger.jsonl')
    parser.add_argument('--tool-version', action='append', default=[], help='name=version, repeatable')
    args = parser.parse_args()

    if not os.path.isfile(args.junit):
        print(f'ci-ledger: JUnit file missing: {args.junit}', file=sys.stderr)
        return 1
    try:
        passed, failed, skipped = read_junit(args.junit)
    except ET.ParseError as error:
        print(f'ci-ledger: unreadable JUnit: {error}', file=sys.stderr)
        return 1
    known = load_known(args.known)
    problems = evaluate(passed, failed, skipped, known)
    entry = {
        'time': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'head': git('rev-parse', 'HEAD'),
        'dirty': bool(git('status', '--porcelain')), 'mode': args.mode,
        'passed': len(passed), 'failed': len(failed), 'skipped': len(skipped),
        'failedNames': sorted(failed), 'skippedNames': sorted(skipped), 'problems': problems,
        'tools': dict(item.split('=', 1) for item in args.tool_version if '=' in item),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.ledger)), exist_ok=True)
    with open(args.ledger, 'a') as handle:
        handle.write(json.dumps(entry, sort_keys=True) + '\n')
    print(f"ci-ledger: mode={args.mode} passed={len(passed)} failed={len(failed)} skipped={len(skipped)} "
          f"problems={len(problems)}")
    for problem in problems:
        print(f'  - {problem}', file=sys.stderr)
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
