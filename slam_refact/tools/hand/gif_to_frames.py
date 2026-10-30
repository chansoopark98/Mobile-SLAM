#!/usr/bin/env python3
"""Extract frames of an animated GIF to a frame directory for sr_hand_run (US-021; docs/spec/10-hand.md §5).

  gif_to_frames.py <gif> <new dir> [--step N] [--max M]

Writes <dir>/frame-%04d.rgb (raw RGB8, row 0 = top) and <dir>/meta.txt ("<w> <h> <count>").
The GIFs of mediapipe-fork/docs/images/mobile are recordings of the original MediaPipe demo app WITH
its overlay graphics burnt in (palm box, hand rect, landmarks, the MediaPipe logo): they are real
camera content of real hands, but the input is NOT clean. PIL only. Run with python3 -I.
"""
import argparse
import os

from PIL import Image


def main():
  ap = argparse.ArgumentParser()
  ap.add_argument('gif')
  ap.add_argument('out')
  ap.add_argument('--step', type=int, default=1)
  ap.add_argument('--max', type=int, default=1 << 30)
  a = ap.parse_args()
  os.makedirs(a.out, exist_ok=False)
  im = Image.open(a.gif)
  n = 0
  w = h = 0
  for i in range(0, getattr(im, 'n_frames', 1), a.step):
    if n >= a.max:
      break
    im.seek(i)
    f = im.convert('RGB')
    w, h = f.size
    open(os.path.join(a.out, 'frame-%04d.rgb' % n), 'wb').write(f.tobytes())
    n += 1
  open(os.path.join(a.out, 'meta.txt'), 'w').write('%d %d %d\n' % (w, h, n))
  print(a.gif, w, h, n)


if __name__ == '__main__':
  main()
