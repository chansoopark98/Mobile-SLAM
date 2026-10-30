#!/usr/bin/env python3
"""Pin official public standalone bytes; preserve existing xr_reference trees."""
import hashlib
import json
import pathlib
import stat
import sys
import urllib.request
import zipfile

ROOT = pathlib.Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts/dev'))
from reference_metadata import load_reference_metadata
SOURCE = load_reference_metadata(ROOT)
ASSETS = ROOT / "assets/references/xr_reference/analysis-runtime"
EVIDENCE = ROOT / "build/research-20261012/xr_reference-binary"
COMMIT = "e1421ae742a9c5d654871367c64c1de3ad044caf"
EXPECTED_SHA = "a2fc73ba450c1a96ee2a20db442f7fb328a3c8b76565e7e9d15b68cd75c86a93"
URL = SOURCE["archive_url_template"].format(commit=COMMIT)
ASSETS.mkdir(parents=True, exist_ok=True)
EVIDENCE.mkdir(parents=True, exist_ok=True)
archive = ASSETS / "xr-standalone.zip"
data = archive.read_bytes() if archive.exists() else urllib.request.urlopen(URL, timeout=45).read(32 * 1024 * 1024 + 1)
actual = hashlib.sha256(data).hexdigest()
if actual != EXPECTED_SHA or len(data) != 13998848:
    raise SystemExit("Official LFS object checksum/length mismatch; not extracted")
if not archive.exists():
    archive.write_bytes(data)
files = []
with zipfile.ZipFile(archive) as package:
    if sum(x.file_size for x in package.infolist()) > 256 * 1024 * 1024:
        raise SystemExit("Standalone extraction exceeds 256 MiB bound")
    for item in package.infolist():
        name = pathlib.PurePosixPath(item.filename)
        if name.is_absolute() or ".." in name.parts or stat.S_ISLNK(item.external_attr >> 16):
            raise SystemExit("Unsafe ZIP member")
        if item.is_dir():
            continue
        content = package.read(item)
        target = ASSETS / "standalone" / item.filename
        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists() and target.read_bytes() != content:
            raise SystemExit("Existing standalone asset differs; preserving it: " + str(target))
        if not target.exists():
            target.write_bytes(content)
        files.append({"path": str(target.relative_to(ROOT)), "bytes": len(content), "sha256": hashlib.sha256(content).hexdigest()})
manifest = {"schema": "xr_reference-binary-acquisition-v1", "url": URL, "commit": COMMIT,
            "bytes": len(data), "sha256": actual, "status": "ACTUAL_LFS_OBJECT_VERIFIED", "files": files,
            "authorization": "User explicitly requested public binary analysis and local execution on 2026-10-12",
            "original_assets_preserved": ["assets/references/xr_reference/engine", "assets/references/xr_reference/web"]}
(EVIDENCE / "acquisition.json").write_text(json.dumps(manifest, indent=2) + "\n")
print(json.dumps({"status": manifest["status"], "bytes": len(data), "sha256": actual, "files": len(files)}, indent=2))
