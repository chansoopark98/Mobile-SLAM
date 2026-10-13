#!/usr/bin/env python3
"""Archive pinned AlvaAR plus its preserved preexisting toolchain patch into a new directory."""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import tarfile

PIN = '7796af500ee92001ac2a9888363ff64d7a3bee75'
HEADER = 'src/libs/opencv/modules/core/include/opencv2/core/hal/intrin_wasm.hpp'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    repo, output = args.repo.resolve(), args.output.resolve()
    assert subprocess.check_output(['git','-C',str(repo),'rev-parse','HEAD'],text=True).strip() == PIN
    output.mkdir(parents=True, exist_ok=False)
    status = subprocess.check_output(['git','-C',str(repo),'status','--short'])
    patch = subprocess.check_output(['git','-C',str(repo),'diff','--binary'])
    changed = subprocess.check_output(['git','-C',str(repo),'diff','--name-only'],text=True).splitlines()
    if changed != [HEADER]:
        raise ValueError(f'Unexpected tracked reference changes: {changed}; original preserved, no reset performed')
    (output/'original-status.txt').write_bytes(status)
    (output/'preexisting.patch').write_bytes(patch)
    archive = output/'upstream-source.tar'
    with archive.open('xb') as target:
        subprocess.run(['git','-C',str(repo),'archive','--format=tar',PIN],stdout=target,check=True)
    source = output/'source'
    source.mkdir()
    with tarfile.open(archive) as packed:
        packed.extractall(source,filter='data')
    shutil.copy2(repo/HEADER,source/HEADER)
    manifest = {'repo':'https://github.com/alanross/AlvaAR','commit':PIN,'source':str(source),
        'originalCheckout':str(repo),'originalStatus':status.decode(),'algorithmEdits':False,
        'preexistingPatchSha256':hashlib.sha256(patch).hexdigest(),
        'stockArchiveSha256':hashlib.file_digest(archive.open('rb'),'sha256').hexdigest(),
        'headerSha256':hashlib.sha256((source/HEADER).read_bytes()).hexdigest(),
        'license':'GPL-3.0; bundled dependencies carry their own notices',
        'prebuilt':{'path':str(source/'dist/alva_ar.js'),'sha256':hashlib.sha256((source/'dist/alva_ar.js').read_bytes()).hexdigest(),
            'meaning':'Repository prebuilt; not newly compiled and not proof of exact source/toolchain provenance'}}
    (output/'source-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
    print(json.dumps(manifest,indent=2))


if __name__ == '__main__':
    main()
