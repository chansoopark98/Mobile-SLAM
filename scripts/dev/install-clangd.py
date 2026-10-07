#!/usr/bin/env python3
"""Install the pinned official clangd release into this checkout only."""
import hashlib
import os
from pathlib import Path
import urllib.request
import zipfile

VERSION = "23.1.0"
SHA256 = "e53b1a96196095faedb7642cf64964f7fb9ad4a0c1f00dd2c172a3d9dcbafdfd"
URL = f"https://github.com/clangd/clangd/releases/download/{VERSION}/clangd-linux-{VERSION}.zip"
TOOLS = Path(__file__).resolve().parent / "tools"


def main():
    TOOLS.mkdir(parents=True, exist_ok=True)
    archive = TOOLS / f"clangd-{VERSION}.zip"
    if not archive.exists():
        with urllib.request.urlopen(URL, timeout=30) as response, archive.open("wb") as out:
            while chunk := response.read(1024 * 1024):
                out.write(chunk)
    with archive.open("rb") as source:
        digest = hashlib.file_digest(source, "sha256").hexdigest()
    if digest != SHA256:
        raise SystemExit(f"SHA256 mismatch: {archive}; remove this incomplete download before retrying")
    with zipfile.ZipFile(archive) as source:
        for entry in source.infolist():
            destination = (TOOLS / entry.filename).resolve()
            if not destination.is_relative_to(TOOLS.resolve()):
                raise SystemExit("Unsafe archive entry")
        source.extractall(TOOLS)
    for executable in (TOOLS / f"clangd_{VERSION}" / "bin").iterdir():
        executable.chmod(executable.stat().st_mode | 0o111)
    print(TOOLS / f"clangd_{VERSION}" / "bin" / "clangd")
    print(f"sha256:{digest}")


if __name__ == "__main__":
    if os.uname().machine != "x86_64" or os.uname().sysname != "Linux":
        raise SystemExit("This pin is for Linux x86_64; use the matching official clangd release for this host")
    main()
