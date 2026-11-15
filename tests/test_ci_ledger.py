import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
TOOL = os.path.join(ROOT, 'scripts/dev/ci-ledger.py')


def junit(cases):
    rows = []
    for name, state, text in cases:
        if state == 'pass':
            rows.append(f'<testcase name="{name}" status="run"/>')
        elif state == 'skip':
            rows.append(f'<testcase name="{name}" status="notrun"><skipped/></testcase>')
        else:
            rows.append(f'<testcase name="{name}" status="fail"><failure message="Failed"/><system-out>{text}</system-out></testcase>')
    return '<?xml version="1.0"?><testsuite>' + ''.join(rows) + '</testsuite>'


class CiLedgerTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.dir = self.temp.name
        self.known = os.path.join(self.dir, 'known.json')
        with open(self.known, 'w') as handle:
            json.dump({'failures': [{'name': 'Suite.Known', 'fingerprint': 'expected-needle'}]}, handle)
        self.ledger = os.path.join(self.dir, 'out', 'ledger.jsonl')

    def tearDown(self):
        self.temp.cleanup()

    def run_tool(self, cases, known=True, write=True):
        path = os.path.join(self.dir, 'unit.junit.xml')
        if write:
            with open(path, 'w') as handle:
                handle.write(junit(cases))
        args = [sys.executable, TOOL, '--junit', path, '--mode', 'test', '--ledger', self.ledger]
        if known:
            args += ['--known', self.known]
        return subprocess.run(args, capture_output=True, text=True, cwd=ROOT)

    def test_same_failure_set_passes(self):
        r = self.run_tool([('A', 'pass', ''), ('Suite.Known', 'fail', 'x expected-needle y')])
        self.assertEqual(r.returncode, 0, r.stderr)

    def test_new_failure_fails(self):
        r = self.run_tool([('Suite.Known', 'fail', 'expected-needle'), ('B', 'fail', 'boom')])
        self.assertNotEqual(r.returncode, 0)
        self.assertIn('new failure: B', r.stderr)

    def test_known_failure_now_passing_is_mismatch(self):
        r = self.run_tool([('A', 'pass', ''), ('Suite.Known', 'pass', '')])
        self.assertNotEqual(r.returncode, 0)
        self.assertIn('no longer fails', r.stderr)

    def test_fingerprint_mismatch_fails(self):
        r = self.run_tool([('Suite.Known', 'fail', 'some other message')])
        self.assertNotEqual(r.returncode, 0)
        self.assertIn('fingerprint mismatch', r.stderr)

    def test_ledger_is_append_only(self):
        cases = [('A', 'pass', ''), ('Suite.Known', 'fail', 'expected-needle')]
        self.run_tool(cases)
        with open(self.ledger, 'rb') as handle:
            first = handle.readline()
        self.run_tool(cases)
        with open(self.ledger, 'rb') as handle:
            lines = handle.readlines()
        self.assertEqual(len(lines), 2)
        self.assertEqual(lines[0], first)
        entry = json.loads(lines[1])
        for key in ('head', 'dirty', 'mode', 'passed', 'failed', 'skipped', 'failedNames', 'tools'):
            self.assertIn(key, entry)

    def test_missing_or_empty_junit_fails(self):
        self.assertNotEqual(self.run_tool([], write=False).returncode, 0)
        self.assertNotEqual(self.run_tool([], known=False).returncode, 0)

    def test_skips_are_counted_not_passed(self):
        r = self.run_tool([('A', 'pass', ''), ('S', 'skip', '')], known=False)
        self.assertEqual(r.returncode, 0, r.stderr)
        with open(self.ledger) as handle:
            entry = json.loads(handle.readlines()[-1])
        self.assertEqual((entry['passed'], entry['skipped']), (1, 1))


if __name__ == '__main__':
    unittest.main()
