import importlib.util
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location("mobile_audit_server", Path(__file__).resolve().parents[2] / "scripts/dev/serve-mobile-audit.py")
server = importlib.util.module_from_spec(spec)
spec.loader.exec_module(server)


class PublicStaticTests(unittest.TestCase):
    def test_public_javascript_is_allowed(self):
        with tempfile.TemporaryDirectory() as directory:
            web = Path(directory)
            (web / "app.js").write_text("public")
            self.assertEqual(server.resolve_public_file("/app.js?v=11", web), web / "app.js")

    def test_dot_directories_and_encoded_variants_are_denied(self):
        with tempfile.TemporaryDirectory() as directory:
            web = Path(directory)
            (web / ".certs").mkdir()
            (web / ".certs/key.pem").write_text("test sentinel only")
            for route in ["/.certs/key.pem", "/%2ecerts/key.pem", "/%2Ecerts%2fkey.pem", "/.certs/key.pem?public.js", "/key.pem", "/private.key", "/certificate.crt", "/%00.js"]:
                self.assertIsNone(server.resolve_public_file(route, web), route)

    def test_traversal_is_denied(self):
        with tempfile.TemporaryDirectory() as directory:
            web = Path(directory) / "web"
            web.mkdir()
            for route in ["/../secret.js", "/%2e%2e/secret.js", "/js/../app.js"]:
                self.assertIsNone(server.resolve_public_file(route, web), route)

    def test_symlink_to_secret_or_outside_root_is_denied(self):
        with tempfile.TemporaryDirectory() as directory:
            web = Path(directory) / "web"
            web.mkdir()
            (web / ".certs").mkdir()
            sentinel = web / ".certs/key.pem"
            sentinel.write_text("test sentinel only")
            (web / "public.js").symlink_to(sentinel)
            outside = Path(directory) / "outside.js"
            outside.write_text("private fixture")
            (web / "outside.js").symlink_to(outside)
            self.assertIsNone(server.resolve_public_file("/public.js", web))
            self.assertIsNone(server.resolve_public_file("/outside.js", web))


if __name__ == "__main__":
    unittest.main()
