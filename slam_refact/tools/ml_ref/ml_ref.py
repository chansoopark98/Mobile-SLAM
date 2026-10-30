#!/usr/bin/env python3
"""numpy reference runner for the slam_refact ML engine (US-016/US-017).

Commands
  inventory --out <json>         graph summary of the seven models (ops, tensors, weights)
  graph --model <name> --out <txt>
                                 canonical graph text (compared line by line by the C++ test)
  run-all --out <dir> [--models a,b]
                                 per model: inputs-<k>.srt (float32), ref-<k>.srt (float64 outputs),
                                 graph.txt; plus <dir>/manifest.json

Models are read from assets/references (read-only; never copied). Run with python3 -I.
Docs: slam_refact/docs/design/ml-runtime.md.
"""
import argparse
import hashlib
import json
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

from ref_ops import RefInterpreter  # noqa: E402
from tensor_bundle import write_bundle  # noqa: E402
from tflite_reader import read_model  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..'))
RES = 'assets/references/xr_reference/engine/resources'
MP = 'assets/references/xr_reference/official/mediapipe-fork/mediapipe/models'

# name, relative path, input value range, LCG seed
MODELS = [
    ('face-model', RES + '/face-model.tflite', (-1.0, 1.0), 101),
    ('face-mesh-model', RES + '/face-mesh-model.tflite', (-1.0, 1.0), 202),
    ('face-ear-model', RES + '/face-ear-model.tflite', (0.0, 1.0), 303),
    ('semantics-model', RES + '/semantics-model.tflite', (0.0, 1.0), 404),
    ('palm_detection', MP + '/palm_detection.tflite', (-1.0, 1.0), 505),
    ('hand_landmark', MP + '/hand_landmark.tflite', (-1.0, 1.0), 606),
    ('hand_landmark_3d', MP + '/hand_landmark_3d.tflite', (-1.0, 1.0), 707),
]


def model_path(rel):
  return os.path.join(REPO, rel)


def sha256_file(path):
  h = hashlib.sha256()
  with open(path, 'rb') as f:
    h.update(f.read())
  return h.hexdigest()


# ---- canonical graph text ---------------------------------------------------------------------
def _f32hex(v):
  return '0x%08x' % np.array([v], dtype=np.float32).view(np.uint32)[0]


def canonical_options(op):
  o = op.options
  n = op.name
  if n == 'CONV_2D':
    return 'pad=%s sw=%d sh=%d dw=%d dh=%d act=%s' % (o['padding'], o['stride_w'], o['stride_h'],
                                                      o['dilation_w'], o['dilation_h'], o['activation'])
  if n == 'DEPTHWISE_CONV_2D':
    return 'pad=%s sw=%d sh=%d dm=%d dw=%d dh=%d act=%s' % (
        o['padding'], o['stride_w'], o['stride_h'], o['depth_multiplier'], o['dilation_w'],
        o['dilation_h'], o['activation'])
  if n == 'MAX_POOL_2D':
    return 'pad=%s sw=%d sh=%d fw=%d fh=%d act=%s' % (o['padding'], o['stride_w'], o['stride_h'],
                                                      o['filter_w'], o['filter_h'], o['activation'])
  if n == 'CONCATENATION':
    return 'axis=%d act=%s' % (o['axis'], o['activation'])
  if n in ('ADD', 'SUB', 'MUL') and o:
    return 'act=%s' % o['activation']
  if n == 'RESIZE_BILINEAR':
    return 'ac=%d hp=%d' % (o['align_corners'], o['half_pixel_centers'])
  if n == 'RESHAPE' and o:
    ns = o['new_shape']
    return 'new_shape=none' if ns is None else 'new_shape=[%s]' % ','.join(str(v) for v in ns)
  if n in ('MEAN', 'REDUCE_MAX') and o:
    return 'keep=%d' % o['keep_dims']
  if n == 'STRIDED_SLICE':
    return 'bm=%d em=%d elm=%d nam=%d sam=%d' % (o['begin_mask'], o['end_mask'], o['ellipsis_mask'],
                                                o['new_axis_mask'], o['shrink_axis_mask'])
  if n == 'ARG_MAX' and o:
    return 'out=%s' % o['output_type']
  if n == 'LEAKY_RELU':
    return 'alpha=%s' % _f32hex(o['alpha'])
  if n == 'CAST' and o:
    return 'in=%s out=%s' % (o['in_type'], o['out_type'])
  return ''


def graph_text(m):
  lines = ['model version=%d subgraphs=%d name=%s' % (m.version, m.num_subgraphs, m.subgraph_name)]
  for t in m.tensors:
    nbytes = 0 if t.data is None else t.data.nbytes
    lines.append('tensor %d %s %s [%s] bytes=%d quant=%d var=%d sparse=%d' % (
        t.index, t.name, t.dtype, ','.join(str(d) for d in t.shape), nbytes, len(t.scale),
        int(t.is_variable), int(t.has_sparsity)))
  for op in m.ops:
    lines.append('op %d %s v%d in=[%s] out=[%s] opts={%s} custom=%s' % (
        op.index, op.name, op.version, ','.join(str(i) for i in op.inputs),
        ','.join(str(i) for i in op.outputs), canonical_options(op), op.custom_options.hex()))
  lines.append('inputs [%s]' % ','.join(str(i) for i in m.inputs))
  lines.append('outputs [%s]' % ','.join(str(i) for i in m.outputs))
  return '\n'.join(lines) + '\n'


def summary(name, rel, m):
  ops = {}
  for op in m.ops:
    ops[op.name] = ops.get(op.name, 0) + 1
  wbytes = {}
  quant = 0
  for t in m.tensors:
    if t.data is not None:
      wbytes[t.dtype] = wbytes.get(t.dtype, 0) + t.data.nbytes
    if t.scale:
      quant += 1
  return dict(
      name=name, path=rel, sha256=sha256_file(model_path(rel)), size=os.path.getsize(model_path(rel)),
      description=m.description, subgraph=m.subgraph_name, subgraphs=m.num_subgraphs,
      tensors=len(m.tensors), ops=len(m.ops), op_counts=dict(sorted(ops.items())),
      inputs=[dict(name=m.tensors[i].name, shape=m.tensors[i].shape, dtype=m.tensors[i].dtype)
              for i in m.inputs],
      outputs=[dict(name=m.tensors[i].name, shape=m.tensors[i].shape, dtype=m.tensors[i].dtype)
               for i in m.outputs],
      weight_bytes=wbytes, quantized_tensors=quant)


# ---- inputs -------------------------------------------------------------------------------------
def pattern_input(shape, lo, hi):
  n, h, w, c = shape
  ni, hi_, wi, ci = np.meshgrid(np.arange(n), np.arange(h), np.arange(w), np.arange(c), indexing='ij')
  u = wi / max(w - 1, 1)
  v = hi_ / max(h - 1, 1)
  s = np.sin(2 * math.pi * (1.7 * u + 0.9 * v) + 1.1 * ci + 0.6 * ni) * \
      np.cos(2 * math.pi * (0.6 * u - 1.3 * v) + 0.3 * ci)
  x01 = 0.5 + 0.45 * s
  return (lo + (hi - lo) * x01).astype(np.float32)


def noise_input(shape, lo, hi, seed):
  count = int(np.prod(shape))
  out = np.empty(count, dtype=np.float64)
  state = seed & 0xffffffff
  for i in range(count):
    state = (1664525 * state + 1013904223) & 0xffffffff
    out[i] = state / 4294967296.0
  return (lo + (hi - lo) * out).reshape(shape).astype(np.float32)


# ---- commands ----------------------------------------------------------------------------------
def select_models(arg):
  if not arg:
    return MODELS
  want = arg.split(',')
  sel = [m for m in MODELS if m[0] in want]
  if len(sel) != len(want):
    raise SystemExit('unknown model in %s' % arg)
  return sel


def require(path):
  if not os.path.exists(path):
    raise SystemExit('model file missing: %s (assets/references is required; it is not copied '
                     'into the repo)' % path)


def cmd_inventory(args):
  out = []
  for name, rel, _, _ in select_models(args.models):
    require(model_path(rel))
    out.append(summary(name, rel, read_model(model_path(rel))))
  text = json.dumps(out, indent=1)
  if args.out:
    with open(args.out, 'w') as f:
      f.write(text + '\n')
  else:
    print(text)


def cmd_graph(args):
  (name, rel, _, _), = select_models(args.model)
  require(model_path(rel))
  with open(args.out, 'w') as f:
    f.write(graph_text(read_model(model_path(rel))))


def cmd_run_all(args):
  os.makedirs(args.out, exist_ok=True)
  manifest = dict(tool='slam_refact/tools/ml_ref/ml_ref.py run-all', numpy=np.__version__,
                  precision='float64', models=[])
  for name, rel, rng, seed in select_models(args.models):
    path = model_path(rel)
    require(path)
    m = read_model(path)
    d = os.path.join(args.out, name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, 'graph.txt'), 'w') as f:
      f.write(graph_text(m))
    interp = RefInterpreter(m)
    in_t = m.tensors[m.inputs[0]]
    if len(m.inputs) != 1:
      raise SystemExit('%s: expected one input' % name)
    inputs = [('pattern', pattern_input(in_t.shape, *rng)),
              ('noise', noise_input(in_t.shape, rng[0], rng[1], seed))]
    entry = dict(name=name, path=rel, sha256=sha256_file(path), inputs=[])
    for k, (kind, x) in enumerate(inputs):
      t0 = time.time()
      vals = interp.run([x])
      dt = time.time() - t0
      write_bundle(os.path.join(d, 'inputs-%d.srt' % k), [(in_t.name, x)])
      outs = []
      for oi in m.outputs:
        t = m.tensors[oi]
        v = vals[oi]
        outs.append((t.name, v.astype(np.float64) if v.dtype.kind == 'f' else v))
      write_bundle(os.path.join(d, 'ref-%d.srt' % k), outs)
      entry['inputs'].append(dict(kind=kind, file='inputs-%d.srt' % k, ref='ref-%d.srt' % k,
                                  seconds=round(dt, 3)))
      print('%s input %d (%s): %.2f s' % (name, k, kind, dt), flush=True)
    manifest['models'].append(entry)
  with open(os.path.join(args.out, 'manifest.json'), 'w') as f:
    json.dump(manifest, f, indent=1)
    f.write('\n')


def main():
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  sub = ap.add_subparsers(dest='cmd', required=True)
  p = sub.add_parser('inventory')
  p.add_argument('--out')
  p.add_argument('--models')
  p.set_defaults(fn=cmd_inventory)
  p = sub.add_parser('graph')
  p.add_argument('--model', required=True)
  p.add_argument('--out', required=True)
  p.set_defaults(fn=cmd_graph)
  p = sub.add_parser('run-all')
  p.add_argument('--out', required=True)
  p.add_argument('--models')
  p.set_defaults(fn=cmd_run_all)
  args = ap.parse_args()
  args.fn(args)


if __name__ == '__main__':
  main()
