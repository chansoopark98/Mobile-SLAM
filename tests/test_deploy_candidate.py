import subprocess, sys, tempfile, unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / 'scripts/dev/deploy-candidate.py'
LEGACY = ROOT / 'build/refactor-orchestration/deploy-authorized.json'


def run(*args):
    return subprocess.run([sys.executable, str(SCRIPT), *args], capture_output=True, text=True)


class DeployCandidateAuthorizationTest(unittest.TestCase):
    def setUp(self):
        self.deploy_dir = ROOT / 'build/refactor-deploy'
        self.before = sorted(p.name for p in self.deploy_dir.iterdir()) if self.deploy_dir.exists() else None

    def assert_no_side_effects(self):
        after = sorted(p.name for p in self.deploy_dir.iterdir()) if self.deploy_dir.exists() else None
        self.assertEqual(self.before, after)

    def test_missing_authorization_is_argparse_error(self):
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'c.js'; c.write_text('x')
            m = Path(tmp) / 'm.json'; m.write_text('{}')
            r = run('--candidate', str(c), '--manifest', str(m))
        self.assertEqual(r.returncode, 2)
        self.assertIn('--authorization', r.stderr)
        self.assert_no_side_effects()

    def test_legacy_default_path_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            c = Path(tmp) / 'c.js'; c.write_text('x')
            m = Path(tmp) / 'm.json'; m.write_text('{}')
            r = run('--candidate', str(c), '--manifest', str(m), '--authorization', str(LEGACY))
        self.assertEqual(r.returncode, 2)
        self.assertIn('legacy', r.stderr)
        self.assert_no_side_effects()


if __name__ == '__main__':
    unittest.main()
