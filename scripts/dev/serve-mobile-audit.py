#!/usr/bin/env python3
"""Read-only development server; existing logs, product artifacts and certs untouched."""
import argparse
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import mimetypes
import os
from pathlib import Path
import ssl
import subprocess
from urllib.parse import unquote, urlsplit

ROOT = Path(__file__).resolve().parents[2]
WEB = (ROOT / "web").resolve()
PUBLIC_EXTENSIONS = {".html", ".js", ".mjs", ".css", ".wasm", ".json", ".png", ".jpg", ".jpeg", ".webp", ".svg", ".ico"}


def resolve_public_file(request_path, web=WEB):
    route = unquote(urlsplit(request_path).path)
    if "\0" in route or any(part.startswith(".") for part in route.split("/") if part):
        return None
    requested = web / (route.lstrip("/") or "audit-mobile.html")
    if requested.suffix.lower() not in PUBLIC_EXTENSIONS:
        return None
    path = requested.resolve()
    if not path.is_relative_to(web) or path.suffix.lower() not in PUBLIC_EXTENSIONS:
        return None
    if any(part.startswith(".") for part in path.relative_to(web).parts):
        return None
    return path


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_):
        pass

    def send(self, code, body=b"", mime="text/plain"):
        self.send_response(code)
        self.send_header("Content-Type", mime)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Cross-Origin-Opener-Policy", "same-origin")
        self.send_header("Cross-Origin-Embedder-Policy", "credentialless")
        self.send_header("Permissions-Policy", "camera=(self), accelerometer=(self), gyroscope=(self)")
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(body)

    def do_GET(self):
        route = unquote(urlsplit(self.path).path)
        if route == "/__audit-health__":
            self.send(200, json.dumps({"service": "mobile-slam-read-only-audit", "writes": "disabled"}).encode(), "application/json")
            return
        path = resolve_public_file(self.path)
        if path is None:
            self.send(403)
            return
        if not path.is_file():
            self.send(404)
            return
        mime = "application/javascript" if path.suffix in (".js", ".mjs") else mimetypes.guess_type(path)[0] or "application/octet-stream"
        self.send(200, path.read_bytes(), mime)

    do_HEAD = do_GET

    def reject_write(self):
        self.close_connection = True
        self.send(405, b"Read-only audit server: no uploads or log writes")

    do_POST = reject_write
    do_PUT = reject_write
    do_PATCH = reject_write
    do_DELETE = reject_write


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1", help="Use 0.0.0.0 explicitly for phone access")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--http", action="store_true", help="Localhost mock tests only; LAN phone sensors require HTTPS")
    parser.add_argument("--cert", type=Path)
    parser.add_argument("--key", type=Path)
    parser.add_argument("--generate-dev-cert", action="store_true", help="Create/reuse isolated self-signed cert under build/dev-mobile-audit/certs")
    args = parser.parse_args()
    if not 0 <= args.port <= 65535:
        parser.error("Invalid port")
    if args.http and (args.cert or args.key or args.generate_dev_cert):
        parser.error("--http cannot be combined with TLS options")
    if args.generate_dev_cert:
        if args.cert or args.key:
            parser.error("Choose either --generate-dev-cert or explicit --cert/--key")
        directory = ROOT / "build/dev-mobile-audit/certs"
        directory.mkdir(parents=True, exist_ok=True)
        args.cert, args.key = directory / "cert.pem", directory / "key.pem"
        if args.cert.exists() != args.key.exists():
            parser.error("Incomplete dev cert pair; supply another explicit pair without overwrite")
        if not args.cert.exists():
            subprocess.run(["openssl", "req", "-x509", "-newkey", "rsa:2048", "-nodes", "-days", "7", "-subj", "/CN=localhost", "-addext", "subjectAltName=DNS:localhost,IP:127.0.0.1", "-keyout", str(args.key), "-out", str(args.cert)], check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            os.chmod(args.key, 0o600)
    if not args.http and (not args.cert or not args.key):
        parser.error("HTTPS requires --cert/--key or --generate-dev-cert")
    server = ThreadingHTTPServer((args.host, args.port), Handler)
    if not args.http:
        context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
        context.minimum_version = ssl.TLSVersion.TLSv1_2
        context.load_cert_chain(args.cert, args.key)
        server.socket = context.wrap_socket(server.socket, server_side=True)
    scheme = "http" if args.http else "https"
    print(json.dumps({"service": "mobile-slam-read-only-audit", "url": f"{scheme}://{args.host}:{server.server_port}/audit-mobile.html", "writes": "disabled", "certificate": "explicit existing pair" if not args.generate_dev_cert else "isolated self-signed localhost pair; not trusted phone TLS"}), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
