"""Lighting rows of an original-SDK api-oracle run for fixtures whose frame list is not a record.json (RM-3a).

  python3 -I slam_refact/tools/harness/oracle/extract_lighting_oracle.py \
      --run <run dir with api-oracle.json.gz> --frames-json <fixture input.json> \
      --image-dir <dir with the frame PGMs> --out <new dir>

Writes <out>/lighting.tsv in the format of extract_api_oracle.py (L fresh frame exposure temperature prev_frame
pgm_path; prev_frame = input frame of the previous fresh callback, the first callback's pipeline frame is a bootstrap
copy of frame 0) and <out>/provenance.json. Every frame used is checked against the pixel sha256 of the frame list, so a
changed fixture fails here instead of producing a silently different comparison. Values keep the sign of zero.
New output dirs only.
"""
import argparse, gzip, hashlib, json, pathlib, sys


def num(v):
    return -0.0 if v == '-0' else float(v)


def sha(path):
    return hashlib.sha256(pathlib.Path(path).read_bytes()).hexdigest()


def pgm_pixels_sha(path, w, h):
    # The frame list hashes the P5 pixel payload (the last w*h bytes), not the file with its header.
    b = pathlib.Path(path).read_bytes()
    assert b.startswith(b'P5'), path
    return hashlib.sha256(b[-w * h:]).hexdigest()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', required=True)
    ap.add_argument('--frames-json', required=True)
    ap.add_argument('--image-dir', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    run = pathlib.Path(a.run)
    d = json.load(gzip.open(run / 'api-oracle.json.gz'))
    fj = json.loads(pathlib.Path(a.frames_json).read_text())
    frames = fj['frames']
    w, h = fj['config']['width'], fj['config']['height']
    image_dir = pathlib.Path(a.image_dir)
    prov = {'argv': sys.argv, 'sources': {str(run / 'api-oracle.json.gz'): sha(run / 'api-oracle.json.gz'),
                                          a.frames_json: sha(a.frames_json)}}
    lines = []
    prev = 0  # bootstrap frames repeat frame 0
    checked = {}
    for row in d['lighting']:
        if row['frameIndex'] is None:
            continue
        for idx in (prev, row['frameIndex']):
            if idx not in checked:
                fr = frames[idx]
                assert fr['index'] == idx, (idx, fr['index'])
                assert pgm_pixels_sha(image_dir / fr['file'], w, h) == fr['sha256'], f'frame {idx} pixel sha256 differs from the frame list'
                checked[idx] = True
        lg = row['lighting']
        lines.append('\t'.join(['L', str(row['fresh']), str(row['frameIndex']), repr(num(lg['exposure'])),
                                repr(num(lg['temperature'])), str(prev), str(image_dir / frames[prev]['file'])]))
        prev = row['frameIndex']
    (out / 'lighting.tsv').write_text('\n'.join(lines) + '\n')
    prov['frames_sha256_checked'] = len(checked)
    (out / 'provenance.json').write_text(json.dumps(prov, indent=1) + '\n')
    print('lighting rows', len(lines), 'frames checked', len(checked))


if __name__ == '__main__':
    main()
