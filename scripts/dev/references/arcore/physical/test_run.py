"""Host completion gate tests; every device subprocess is replaced at the boundary."""
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import Mock, patch

import run


class CompletionGateTests(unittest.TestCase):
    def execute(self, adb_exit, status="finished"):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            build = root / "build"
            build.mkdir()
            (build / "start-token.txt").write_text("test-token")
            out = root / "result"
            summary = {"runId": "fixture", "mode": "live", "frames": 5,
                       "setupError": None, "cleanupError": None,
                       "saveImages": False, "saveImu": False, "saveArcoreRecording": False}
            lifecycle = {"sessionClosed": True, "sessionCleanupError": None}

            def start(argv, stdout, stderr):
                stdout.write(("INSTRUMENTATION_RESULT: status=" + status + "\nINSTRUMENTATION_CODE: -1\n").encode())
                return Mock(returncode=adb_exit, wait=Mock(return_value=adb_exit))

            def command(argv, **kwargs):
                content = b""
                if argv[-1].endswith("/summary.json"):
                    content = json.dumps(summary).encode()
                elif argv[-1].endswith("/lifecycle.json"):
                    content = json.dumps(lifecycle).encode()
                return subprocess.CompletedProcess(argv, 0, content, b"")

            args = ["run.py", "--serial", "mock-device", "--build", str(build), "--output", str(out),
                    "--run-id", "fixture", "--mode", "live", "--duration", "1", "--save-images", "false",
                    "--save-imu", "false", "--save-recording", "false", "--authorization-note", "unit fixture only"]
            with patch("sys.argv", args), patch.object(run.subprocess, "Popen", side_effect=start), \
                    patch.object(run.subprocess, "run", side_effect=command), patch("builtins.print"):
                code = 0
                try:
                    run.main()
                except SystemExit as error:
                    code = error.code
            return code, json.loads((out / "execution.json").read_text())

    def test_success_requires_current_completed_instrumentation(self):
        code, result = self.execute(0)
        self.assertEqual(code, 0)
        self.assertTrue(result["sdkExecutionComplete"])

    def test_nonzero_adb_cannot_pass_with_success_text_and_files(self):
        code, result = self.execute(7)
        self.assertEqual(code, 1)
        self.assertFalse(result["sdkExecutionComplete"])

    def test_rejected_attempt_cannot_reuse_old_successful_summary(self):
        code, result = self.execute(0, "rejected")
        self.assertEqual(code, 1)
        self.assertFalse(result["sdkExecutionComplete"])


if __name__ == "__main__":
    unittest.main()
