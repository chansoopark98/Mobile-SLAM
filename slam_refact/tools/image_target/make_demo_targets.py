#!/usr/bin/env python3
"""Phone demo image targets (package C): procedural images -> PUBLIC reference runtime image-target CLI -> loader inputs + print sheets.

  make_demo_targets.py --work DIR --install web-chunk/samples/phone/targets

Steps (all desktop tooling; nothing here is product runtime):
  1. make_demo_images.py writes demo-flat.png and demo-cylinder.png into DIR/src.
  2. run_public_cli.mjs runs the unmodified public CLI (answers piped like the public test/*.txt) into DIR/out:
       demo-flat      flat, default crop
       demo-cylinder  cylinder, 220 mm circumference, 160 mm wide label (80 mm tall)
  3. <install>/ gets, per target, the CLI's <name>.json (byte for byte; the phone page's targets.js replaces imagePath by the URL of
     the image) and <name>_luminance.png, manifest.json (schema phone-targets-v1 of web-chunk/samples/phone/targets.js: {name, label, json,
     image}), MANIFEST.sha256 and print/<name>-sheet.{png,pdf} (A4, exact physical size at 300 dpi, scale bar).
Needs numpy, Pillow, node, and the sharp install of sharp_path.mjs.
"""
import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys

from PIL import Image, ImageDraw

HERE = os.path.dirname(os.path.abspath(__file__))
DPI = 300
MM = DPI / 25.4

# name -> (CLI answers after the image path, printed size in mm of the ORIGINAL image)
TARGETS = {
    'demo-flat': (['flat', 'y'], (150.0, 200.0)),
    'demo-cylinder': (['cylinder', 'mm', '220', '160', 'y'], (160.0, 80.0)),
}


def sha(path):
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        h.update(f.read())
    return h.hexdigest()


def run(cmd):
    r = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    if r.returncode != 0:
        sys.exit('failed: %s\n%s' % (' '.join(cmd), r.stdout))


def sheet(src_png, size_mm, title, note, out_base):
    pw_mm, ph_mm = (210, 297) if size_mm[1] > size_mm[0] else (297, 210)  # A4 portrait for a tall card, landscape for a wide label
    w, h = int(round(pw_mm * MM)), int(round(ph_mm * MM))
    page = Image.new('RGB', (w, h), 'white')
    pw, ph = int(round(size_mm[0] * MM)), int(round(size_mm[1] * MM))
    img = Image.open(src_png).convert('RGB').resize((pw, ph), Image.LANCZOS)
    x0, y0 = int(round(20 * MM)), int(round(20 * MM))
    assert y0 + ph + int(round(30 * MM)) <= h and x0 + pw + int(round(10 * MM)) <= w, 'image does not fit the sheet'
    page.paste(img, (x0, y0))
    d = ImageDraw.Draw(page)
    for cx, cy in ((x0, y0), (x0 + pw, y0), (x0, y0 + ph), (x0 + pw, y0 + ph)):  # crop marks
        d.line([(cx - 40, cy), (cx - 10, cy)], fill='black', width=2)
        d.line([(cx + 10, cy), (cx + 40, cy)], fill='black', width=2)
        d.line([(cx, cy - 40), (cx, cy - 10)], fill='black', width=2)
        d.line([(cx, cy + 10), (cx, cy + 40)], fill='black', width=2)
    bx, by = int(round(20 * MM)), h - int(round(14 * MM))  # 100 mm scale bar: measure it after printing
    d.line([(bx, by), (bx + int(round(100 * MM)), by)], fill='black', width=4)
    for k in range(0, 101, 10):
        d.line([(bx + int(round(k * MM)), by - 12), (bx + int(round(k * MM)), by + 12)], fill='black', width=3)
    d.text((bx, by + 18), '%s - print at 100%% (no fit-to-page). This bar must measure 100 mm. %s' % (title, note), fill='black')
    page.save(out_base + '.png', dpi=(DPI, DPI))
    page.save(out_base + '.pdf', resolution=DPI)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--work', required=True)
    ap.add_argument('--install', required=True)
    args = ap.parse_args()
    src, out = os.path.join(args.work, 'src'), os.path.join(args.work, 'out')
    for d in (src, out, args.install, os.path.join(args.install, 'print')):
        os.makedirs(d, exist_ok=True)
    run([sys.executable, os.path.join(HERE, 'make_demo_images.py'), '--out', src])
    for name, (answers, _) in TARGETS.items():
        script = os.path.join(args.work, name + '-answers.txt')
        with open(script, 'w') as f:
            f.write('\n'.join([os.path.abspath(os.path.join(src, name + '.png'))] + answers + [os.path.abspath(out), name]) + '\n')
        run(['node', os.path.join(HERE, 'run_public_cli.mjs'), '--script', script, '--overwrite'])
    notes = {'demo-flat': 'Flat card 150 x 200 mm.',
             'demo-cylinder': 'Wrap round a 70 mm diameter (220 mm circumference) cylinder; 60 mm stays blank.'}
    labels = {'demo-flat': 'Flat card 150x200 mm', 'demo-cylinder': 'Cylinder label 160x80 mm on a 70 mm can'}
    arr, lines = [], []
    for name, (_, size) in TARGETS.items():
        shutil.copyfile(os.path.join(out, name + '.json'), os.path.join(args.install, name + '.json'))
        shutil.copyfile(os.path.join(out, name + '_luminance.png'), os.path.join(args.install, name + '_luminance.png'))
        sheet(os.path.join(src, name + '.png'), size, name, notes[name], os.path.join(args.install, 'print', name + '-sheet'))
        arr.append({'name': name, 'label': labels[name], 'json': name + '.json', 'image': name + '_luminance.png'})
    with open(os.path.join(args.install, 'manifest.json'), 'w') as f:
        json.dump({'schema': 'phone-targets-v1', 'note': 'Written by tools/image_target/make_demo_targets.py.', 'targets': arr}, f, indent=2)
        f.write('\n')
    for root, _, files in sorted(os.walk(args.install)):
        for fn in sorted(files):
            if fn != 'MANIFEST.sha256':
                p = os.path.join(root, fn)
                lines.append('%s  %s' % (sha(p), os.path.relpath(p, args.install)))
    with open(os.path.join(args.install, 'MANIFEST.sha256'), 'w') as f:
        f.write('\n'.join(sorted(lines, key=lambda s: s.split('  ')[1])) + '\n')
    return 0


if __name__ == '__main__':
    sys.exit(main())
