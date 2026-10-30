#!/usr/bin/env python3
"""Recover the official TUM-VI room1 fixture without replacing existing data."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import tarfile
from urllib.request import Request, urlopen

ROOT = Path(__file__).resolve().parents[2]
NAME = "dataset-room1_512_16"
URL = f"https://cdn3.vision.in.tum.de/tumvi/exported/euroc/512_16/{NAME}.tar"
SIZE = 1707110400


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, default=ROOT / "build/dev-tools/dataset-provenance.json")
    args = parser.parse_args()
    base = ROOT / "assets/datasets/tum"
    target, stage = base / NAME, base / ".room1-audit-extraction"
    if target.exists() or (stage.exists() and any(stage.iterdir())):
        raise SystemExit("Existing dataset or nonempty staging directory: refusing overwrite")
    base.mkdir(parents=True, exist_ok=True)
    partial, complete = base / f"{NAME}.tar.partial", base / f"{NAME}.tar"
    archive = complete if complete.exists() else partial
    offset = archive.stat().st_size if archive.exists() else 0
    if complete.exists() and offset != SIZE:
        raise SystemExit("Existing completed archive has an unexpected size; preserved without modification")
    if offset > SIZE:
        raise SystemExit("Archive exceeds pinned official size")
    if offset < SIZE:
        headers = {"Range": f"bytes={offset}-"} if offset else {}
        with urlopen(Request(URL, headers=headers), timeout=30) as response:
            if offset and (response.status != 206 or not response.headers.get("Content-Range", "").startswith(f"bytes {offset}-")):
                raise SystemExit("Server did not honor resume range; existing partial preserved")
            with archive.open("ab" if offset else "wb") as stream:
                while chunk := response.read(4 * 1024 * 1024):
                    stream.write(chunk)
    if archive.stat().st_size != SIZE:
        raise SystemExit("Incomplete archive: retained for resume")
    published = urlopen(URL + ".md5", timeout=30).read().decode().strip()
    md5, sha = hashlib.md5(), hashlib.sha256()
    with archive.open("rb") as stream:
        while chunk := stream.read(8 * 1024 * 1024):
            md5.update(chunk)
            sha.update(chunk)
    if md5.hexdigest() != published.split()[0]:
        raise SystemExit("Published MD5 mismatch: archive preserved, extraction rejected")
    stage.mkdir(exist_ok=True)
    with tarfile.open(archive) as source:
        members = source.getmembers()
        for entry in members:
            destination = (stage / entry.name).resolve()
            if not destination.is_relative_to(stage.resolve()):
                raise SystemExit(f"Unsafe archive path: {entry.name}")
            if entry.issym():
                link = (destination.parent / entry.linkname).resolve()
                if Path(entry.linkname).is_absolute() or not link.is_relative_to(stage.resolve()):
                    raise SystemExit(f"Unsafe archive link: {entry.name}")
            elif not (entry.isfile() or entry.isdir()):
                raise SystemExit(f"Unsupported archive member: {entry.name}")
        source.extractall(stage, filter="data")
    extracted = stage / NAME
    record = {"source": URL, "published_md5_url": URL + ".md5", "published_md5": published,
              "md5": md5.hexdigest(), "sha256": sha.hexdigest(), "bytes": SIZE,
              "license": "CC BY 4.0", "authors": "David Schubert, Thore Goll, Nikolaus Demmel, Vladyslav Usenko, Joerg Stueckler, Daniel Cremers",
              "license_source": "https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset",
              "extracted_utc": datetime.now(timezone.utc).isoformat(), "members": len(members),
              "dataset_directory": str(target)}
    for sensor in ("cam0", "imu0", "mocap0"):
        csv = extracted / "mav0" / sensor / "data.csv"
        record[sensor + "_csv_sha256"] = hashlib.sha256(csv.read_bytes()).hexdigest()
        record[sensor + "_records"] = sum(1 for line in csv.read_text().splitlines() if line and not line.startswith("#"))
    extracted.rename(target)
    stage.rmdir()
    if archive != complete:
        archive.rename(complete)
    args.evidence.parent.mkdir(parents=True, exist_ok=True)
    args.evidence.write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
