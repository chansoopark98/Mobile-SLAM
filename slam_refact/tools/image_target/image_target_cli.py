#!/usr/bin/env python3
"""Image target generator: Python port of the public reference runtime image-target CLI (US-012).

Port of PUB:apps/image-target-cli/src/{cli,interactive,crop,apply,unconify,curved-geometry}.js
(PUB = assets/references/xr_reference/official/xr_reference). The public CLI needs Node + sharp, which is not
installed locally (installing it needs the network). This port uses PIL and numpy only.

Outputs per target (same names and JSON structure as the public CLI):
  <name>.json, <name>_original.<ext>, <name>_cropped.<ext>, <name>_thumbnail.<ext> (height 350),
  <name>_luminance.<ext> (gray, height 640), <name>_geometry.<ext> (conical only).
Engine addition (not referenced from the JSON, so the JSON stays diffable with the public CLI):
  <name>_luminance.pgm - the decoded luminance bytes for the C++ loader, which has no JPEG/PNG
  decoder (slam_refact/engine/src/image_target/target_files.h). Disable with --no-pgm.

Answer sources, in the order of the public prompts (image path, type, geometry, folder, name):
  --script FILE   answers one per line, like `cat test/<case>.txt | node src/index.js`
  flags           --image/--type/... build the same answer list non-interactively
  (neither)       interactive prompts on stdin, like the public CLI

Known differences from sharp (pixels, not structure): resize kernel and rounding, grayscale and JPEG
encoder (quality 80 like sharp's default). Geometry, crop arithmetic and the JSON are computed with
the public formulas. Run with `python3 -I`.

Grayscale (--gray-mode, docs/design/image-target.md section 2.4): `pil` (default) is PIL's
ITU-R 601-2 luma on the gamma-encoded values, applied after the resize. `libvips` follows what
sharp's `.grayscale()` does as far as it is known (sRGB -> linear light, Rec.709 Y, sRGB encode, and
before the resize); it is UNVERIFIED because sharp is not installed here. The two differ by up to
70 levels on saturated colours (mean 13, p95 38 on uniform random RGB). For a parity comparison
with another engine feed both the same <name>_luminance.pgm instead of regenerating it.
"""
import argparse
import decimal
import math
import os
import re
import sys
import time

import numpy as np
from PIL import Image, ImageOps

# PUB:apps/image-target-cli/src/constants.json
CONSTANTS = {
    'minimumWidth': 480,
    'thumbnailHeight': 350,
    'luminanceHeight': 640,
    'minimumHeight': 640,
}


class CliError(Exception):
  pass


# ---- JavaScript number semantics ------------------------------------------------------------------
def js_round(x):
  """Math.round: round half toward +infinity."""
  return int(math.floor(x + 0.5))


_JS_DECIMAL = re.compile(r'^[+-]?(Infinity|\d+\.?\d*(?:[eE][+-]?\d+)?|\.\d+(?:[eE][+-]?\d+)?)$')


def js_number(text):
  """Number(string) of ECMAScript (StringToNumber); NaN as float('nan')."""
  s = text.strip()
  if s == '':
    return 0
  for prefix, base in (('0x', 16), ('0o', 8), ('0b', 2)):
    if s[:2].lower() == prefix:
      try:
        return int(s[2:], base) if s[2:] and '_' not in s else float('nan')
      except ValueError:
        return float('nan')
  if not _JS_DECIMAL.match(s):
    return float('nan')
  if s.lstrip('+-') == 'Infinity':
    return float('-inf') if s.startswith('-') else float('inf')
  v = float(s)
  if v == 0 and math.copysign(1.0, v) < 0:
    return v  # Number('-0') is -0: an int cannot carry the sign (RM-1b, review MINOR-5)
  return int(v) if v.is_integer() and abs(v) < 2 ** 53 else v


def js_parse_int(text):
  """parseInt(string, 10): leading integer or None (NaN)."""
  m = re.match(r'^\s*([+-]?\d+)', text)
  return int(m.group(1)) if m else None


def js_number_to_string(x):
  """Number::toString (ECMAScript 7.1.12.1) for JSON.stringify."""
  if isinstance(x, bool):
    return 'true' if x else 'false'
  if isinstance(x, int):
    return str(x)
  if math.isnan(x) or math.isinf(x):
    return 'null'
  if x == 0:
    return '0'
  sign = '-' if x < 0 else ''
  d = decimal.Decimal(repr(abs(x))).normalize().as_tuple()
  digits = ''.join(str(v) for v in d.digits)
  k = len(digits)
  n = d.exponent + k
  if k <= n <= 21:
    return sign + digits + '0' * (n - k)
  if 0 < n <= 21:
    return sign + digits[:n] + '.' + digits[n:]
  if -6 < n <= 0:
    return sign + '0.' + '0' * (-n) + digits
  e = n - 1
  exp = ('+' if e > 0 else '-') + str(abs(e))
  if k == 1:
    return sign + digits + 'e' + exp
  return sign + digits[0] + '.' + digits[1:] + 'e' + exp


def _js_quote(s):
  out = ['"']
  for ch in s:
    c = ord(ch)
    if ch == '"':
      out.append('\\"')
    elif ch == '\\':
      out.append('\\\\')
    elif ch == '\b':
      out.append('\\b')
    elif ch == '\f':
      out.append('\\f')
    elif ch == '\n':
      out.append('\\n')
    elif ch == '\r':
      out.append('\\r')
    elif ch == '\t':
      out.append('\\t')
    elif c < 0x20 or 0xD800 <= c <= 0xDFFF:
      out.append('\\u%04x' % c)
    else:
      out.append(ch)
  out.append('"')
  return ''.join(out)


def js_stringify(value, indent=2, _level=0):
  """JSON.stringify(value, null, indent) for dict/list/str/int/float/bool/None."""
  pad = ' ' * (indent * (_level + 1))
  end = ' ' * (indent * _level)
  if value is None:
    return 'null'
  if isinstance(value, (bool, int, float)):
    return js_number_to_string(value)
  if isinstance(value, str):
    return _js_quote(value)
  if isinstance(value, dict):
    if not value:
      return '{}'
    items = [pad + _js_quote(str(k)) + ': ' + js_stringify(v, indent, _level + 1)
             for k, v in value.items()]
    return '{\n' + ',\n'.join(items) + '\n' + end + '}'
  if isinstance(value, (list, tuple)):
    if not value:
      return '[]'
    items = [pad + js_stringify(v, indent, _level + 1) for v in value]
    return '[\n' + ',\n'.join(items) + '\n' + end + ']'
  raise TypeError('cannot stringify %r' % (value,))


def js_div(a, b):
  """a / b with ECMAScript semantics: Python raises ZeroDivisionError, JavaScript gives +-Infinity
  (sign of the zero divisor counts) or NaN for 0 / 0."""
  try:
    return a / b
  except ZeroDivisionError:
    if a != a or a == 0:
      return float('nan')
    return math.copysign(math.inf, a) * math.copysign(1.0, b)


def js_neg(x):
  """Unary minus with ECMAScript semantics: -0 for a zero (an int zero would stay 0 in Python)."""
  return -float(x) if x == 0 else -x


def js_log2(x):
  """Math.log2: log2(0) = -Infinity, log2(negative or NaN) = NaN, log2(Infinity) = Infinity."""
  if x != x or x < 0:
    return float('nan')
  if x == 0:
    return float('-inf')
  if x == math.inf:
    return math.inf
  return math.log2(x)


def js_asin(x):
  """Math.asin: NaN outside [-1, 1] (Python raises ValueError)."""
  if x != x or x < -1 or x > 1:
    return float('nan')
  return math.asin(x)


def js_ceil_int(x):
  """Math.ceil result that must be an integer for sharp (height): NaN / Infinity are an error."""
  if x != x or x in (math.inf, -math.inf):
    raise CliError('Invalid cone geometry: the flattened image height is not a finite number')
  return int(math.ceil(x))


# ---- crop.js -------------------------------------------------------------------------------------
def get_default_crop(metadata, is_rotated):
  width, height = ((metadata['height'], metadata['width']) if is_rotated
                   else (metadata['width'], metadata['height']))
  if width / 3 > height / 4:
    cropped_width = js_round((height * 3) / 4)
    return {
        'left': js_round((width - cropped_width) / 2),
        'top': 0,
        'width': cropped_width,
        'height': height,
        'isRotated': is_rotated,
        'originalWidth': width,
        'originalHeight': height,
    }
  cropped_height = js_round((width * 4) / 3)
  return {
      'left': 0,
      'top': js_round((height - cropped_height) / 2),
      'width': width,
      'height': cropped_height,
      'isRotated': is_rotated,
      'originalWidth': width,
      'originalHeight': height,
  }


def validate_crop(crop, image_metadata):
  issues = []
  top, left, width, height = crop['top'], crop['left'], crop['width'], crop['height']
  if top < 0:
    issues.append('Top offset cannot be negative')
  if left < 0:
    issues.append('Left offset cannot be negative')
  if width < CONSTANTS['minimumWidth']:
    issues.append('Width must be at least %d' % CONSTANTS['minimumWidth'])
  if height < CONSTANTS['minimumHeight']:
    issues.append('Height must be at least %d' % CONSTANTS['minimumHeight'])
  if top + height > image_metadata['height']:
    issues.append('Bottom edge of crop exceeds image height (%s > %s)' %
                  (js_number_to_string(top + height), js_number_to_string(image_metadata['height'])))
  if left + width > image_metadata['width']:
    issues.append('Right edge of crop exceeds image width (%s > %s)' %
                  (js_number_to_string(left + width), js_number_to_string(image_metadata['width'])))
  return issues


# ---- curved-geometry.js --------------------------------------------------------------------------
def get_circumference_ratio(top_radius, bottom_radius):
  if top_radius < 0:
    return js_div(js_neg(bottom_radius), top_radius)
  return js_div(top_radius, bottom_radius)


def get_coniness_for_radii(top_radius, bottom_radius):
  base = js_log2(abs(js_div(top_radius, bottom_radius)))
  return -base if top_radius < 0 else base


def get_target_circumference_bottom(target_top, cylinder_top, cylinder_bottom):
  return js_div(target_top * cylinder_bottom, cylinder_top)


# ---- unconify.js ---------------------------------------------------------------------------------
def compute_pixel_points_from_radius(top_radius, bottom_radius, width):
  is_fez = top_radius < 0
  abs_top = abs(top_radius)
  x0 = js_div((width / 2) * bottom_radius, abs_top)
  theta = js_asin(js_div(width / 2, abs_top)) * 2
  top_y = 2 * abs_top * math.sin(theta / 4) ** 2
  y0 = js_div(x0, math.tan(theta / 2))
  return {
      'tl': (0.0, top_y),
      'bl': (width / 2 - x0, abs_top - y0),
      'tr': (width - 1.0, top_y),
      'apex': (width / 2, float(abs_top)),
      'theta': theta,
      'isFez': is_fez,
  }


def _point_dist(a, b):
  dx = a[0] - b[0]
  dy = a[1] - b[1]
  return math.sqrt(dx * dx + dy * dy)


def get_unconified_data(points, output_width):
  tl, bl, tr, apex = points['tl'], points['bl'], points['tr'], points['apex']
  top_radius = _point_dist(apex, tl)
  bottom_radius = _point_dist(apex, bl)
  image_width = _point_dist(tl, tr)
  theta = 2 * js_asin(js_div(image_width / 2, top_radius))
  ratio = js_div(top_radius * theta, top_radius - bottom_radius)
  output_height = js_ceil_int(js_div(output_width, ratio))
  return {'topRadius': top_radius, 'bottomRadius': bottom_radius, 'theta': theta,
          'outputHeight': output_height}


def _rgb_linear_interp(flat, width, xs, ys):
  """rgbLinearInterp2d on a flat RGBA Uint8ClampedArray; indices past the end read undefined."""
  x_floor = np.floor(xs)
  y_floor = np.floor(ys)
  x_scale = 1 - (xs - x_floor)
  y_scale = 1 - (ys - y_floor)
  row_bytes = width * 4
  xf = x_floor.astype(np.int64)
  yf = y_floor.astype(np.int64)
  n = flat.shape[0]

  def color(x, y):
    base = y * row_bytes + x * 4
    out = np.full((base.shape[0], 3), np.nan)
    for c in range(3):
      idx = base + c
      ok = idx < n
      out[ok, c] = flat[idx[ok]]
    return out

  xs_ = x_scale[:, None]
  ys_ = y_scale[:, None]
  top = color(xf, yf) * xs_ + (1 - xs_) * color(xf + 1, yf)
  bottom = color(xf, yf + 1) * xs_ + (1 - xs_) * color(xf + 1, yf + 1)
  return top * ys_ + (1 - ys_) * bottom


def _to_uint8_clamped(values):
  """ToUint8Clamp: NaN -> 0, clamp, round half to even."""
  v = np.where(np.isnan(values), 0.0, values)
  return np.rint(np.clip(v, 0, 255)).astype(np.uint8)


def unconify(rgba, points, output_width):
  """rgba: HxWx4 uint8 array. Returns the flattened geometry image (outputHeight x W x 4)."""
  height, width = rgba.shape[0], rgba.shape[1]
  tl, apex, is_fez = points['tl'], points['apex'], points['isFez']
  data = get_unconified_data(points, output_width)
  top_radius, bottom_radius = data['topRadius'], data['bottomRadius']
  theta, output_height = data['theta'], data['outputHeight']
  alpha = math.asin((apex[0] - tl[0]) / top_radius)

  i = np.arange(output_height, dtype=np.float64)[:, None]
  j = np.arange(output_width, dtype=np.float64)[None, :]
  beta = (theta * j) / (output_width - 1)
  frac = (output_height - 1 - i) / output_height
  r_sampling = top_radius * frac + (1 - frac) * bottom_radius  # lerp0(top, bottom, frac)
  psi = math.pi / 2 + alpha - beta
  sx = apex[0] + r_sampling * np.cos(psi)
  if is_fez:
    sy = height - apex[1] + r_sampling * np.sin(psi)
    sx = sx[::-1, :]
    sy = sy[::-1, :]
  else:
    sy = apex[1] - r_sampling * np.sin(psi)
  sx = np.broadcast_to(sx, (output_height, output_width)).reshape(-1)
  sy = np.broadcast_to(sy, (output_height, output_width)).reshape(-1)

  out = np.zeros((output_height * output_width, 4), dtype=np.uint8)
  inside = ~((sx < 0) | (sy < 0) | (sx >= width) | (sy >= height))
  flat = rgba.reshape(-1)
  rgb = _rgb_linear_interp(flat, width, sx[inside], sy[inside])
  out[inside, 0:3] = _to_uint8_clamped(rgb)
  out[inside, 3] = 255
  return out.reshape(output_height, output_width, 4)


# ---- answers (cli.js) ----------------------------------------------------------------------------
class Answers:
  """CliInterface of cli.js over a line source; invalid answers re-prompt like the public CLI."""

  def __init__(self, lines=None, stream=None, echo=None):
    self._lines = list(lines) if lines is not None else None
    self._stream = stream
    self._echo = echo

  def prompt(self, question):
    if self._echo is not None:
      print(question, file=self._echo)
    if self._lines is not None:
      if not self._lines:
        raise CliError('No more answers for prompt: %s' % question.strip())
      return self._lines.pop(0)
    line = self._stream.readline()
    if line == '':
      raise CliError('End of input at prompt: %s' % question.strip())
    return line.rstrip('\n').rstrip('\r')

  def choose(self, question, options, first_is_default=False):
    option_str = '\n'.join('%d) %s %s' % (i + 1, o, '(default)' if i == 0 and first_is_default else '')
                           for i, o in enumerate(options))
    while True:
      normalized = self.prompt('%s\n%s\n' % (question, option_str)).strip()
      if normalized == '' and first_is_default:
        return options[0]
      parsed = js_parse_int(normalized)
      if parsed is not None and 0 <= parsed - 1 < len(options):
        return options[parsed - 1]
      if normalized in options:
        return normalized
      self._say('Invalid choice. Please try again.')

  def confirm(self, question, default_value=None):
    default_str = '[Y/n]' if default_value else '[y/N]'
    while True:
      normalized = self.prompt('%s %s: ' % (question, default_str)).strip().lower()
      if normalized in ('y', 'yes'):
        return True
      if normalized in ('n', 'no'):
        return False
      if normalized == '' and default_value is not None:
        return default_value
      self._say('Invalid input. Please enter y/n.')

  def prompt_integer(self, question):
    while True:
      v = js_number(self.prompt(question))
      if isinstance(v, int):
        return v
      if isinstance(v, float) and v == 0:  # -0 is an integer in JavaScript (Number.isInteger)
        return 0
      self._say('Invalid input. Please enter an integer.')

  def prompt_float(self, question):
    while True:
      v = js_number(self.prompt(question))
      if not (isinstance(v, float) and math.isnan(v)):
        return v
      self._say('Invalid input. Please enter a number.')

  def _say(self, text):
    if self._echo is not None:
      print(text, file=self._echo)


# ---- interactive.js ------------------------------------------------------------------------------
def select_planar_geometry(rl, image_metadata):
  source_is_landscape = image_metadata['width'] >= image_metadata['height']
  if rl.confirm('Use default crop?', True):
    return get_default_crop(image_metadata, source_is_landscape)
  options = ['landscape', 'portrait'] if source_is_landscape else ['portrait', 'landscape']
  is_rotated = rl.choose('Select the image orientation of the trackable region:', options,
                         True) == 'landscape'
  visual_top = rl.prompt_integer('Enter the top offset of the crop')
  visual_left = rl.prompt_integer('Enter the left offset of the crop')
  visual_width = rl.prompt_integer('Enter the width of the crop')
  if is_rotated:
    # The crop is stored in the coordinates of the rotated image (origin at the bottom left).
    height = visual_width
    width = js_round((height * 3) / 4)
    return {
        'top': visual_left,
        'left': image_metadata['height'] - visual_top - width,
        'width': width,
        'height': height,
        'isRotated': is_rotated,
        'originalWidth': image_metadata['height'],
        'originalHeight': image_metadata['width'],
    }
  height = js_round((visual_width * 4) / 3)
  return {
      'top': visual_top,
      'left': visual_left,
      'width': visual_width,
      'height': height,
      'isRotated': is_rotated,
      'originalWidth': image_metadata['width'],
      'originalHeight': image_metadata['height'],
  }


def _num(v):
  """JS numbers are doubles: integral results print without a fraction (js_number_to_string)."""
  return int(v) if isinstance(v, float) and v.is_integer() and abs(v) < 2 ** 53 else v


def select_cylindrical_geometry(rl, image_metadata):
  unit = rl.choose('Select the unit to measure physical lengths', ['mm', 'in'], True)
  cylinder_circumference = rl.prompt_float('Enter the circumference of the cylinder in %s: ' % unit)
  target_circumference = rl.prompt_float('Enter the width of the image in %s: ' % unit)
  if target_circumference > cylinder_circumference:
    raise CliError('Image width cannot be greater than cylinder circumference')
  arc_angle = (target_circumference / cylinder_circumference) * 360
  cylinder_side_length = (image_metadata['height'] / image_metadata['width']) * target_circumference
  base = select_planar_geometry(rl, image_metadata)
  out = dict(base)
  out.update({
      'targetCircumferenceTop': _num(target_circumference),
      'cylinderCircumferenceTop': _num(cylinder_circumference),
      'cylinderCircumferenceBottom': _num(cylinder_circumference),
      'cylinderSideLength': _num(cylinder_side_length),
      'arcAngle': _num(arc_angle),
      'coniness': 0,
      'inputMode': 'ADVANCED',
      'unit': unit,
  })
  return out


def select_conical_geometry(rl, image_metadata):
  wider_side = rl.choose('Which side of the cone is wider?', ['top', 'bottom'], True)
  is_fez = wider_side == 'bottom'
  outer_radius = rl.prompt_float('Enter the outer radius of the image')
  if outer_radius <= 0:
    raise CliError('Outer radius must be positive')
  inner_radius = rl.prompt_float('Enter the inner radius of the image')
  if inner_radius < 0:
    raise CliError('Inner radius cannot be negative')
  arc_angle = rl.prompt_float('Enter the arc angle of the cone in degrees (e.g. 360 for a full cone)')
  if arc_angle <= 0 or arc_angle > 360:
    raise CliError('Arc angle must be between 0 and 360 degrees')
  top_radius = -outer_radius if is_fez else outer_radius
  bottom_radius = inner_radius

  points = compute_pixel_points_from_radius(top_radius, bottom_radius, image_metadata['width'])
  output_height = get_unconified_data(points, image_metadata['width'])['outputHeight']
  geometry_metadata = {'width': image_metadata['width'], 'height': output_height}
  crop = select_planar_geometry(rl, geometry_metadata)

  cylinder_circumference_top = 100
  target_circumference_top = (cylinder_circumference_top * arc_angle) / 360
  ratio = get_circumference_ratio(top_radius, bottom_radius)
  cylinder_circumference_bottom = js_div(cylinder_circumference_top, ratio)
  coniness = get_coniness_for_radii(top_radius, bottom_radius)
  wider_target_circumference = max(
      target_circumference_top,
      get_target_circumference_bottom(target_circumference_top, cylinder_circumference_top,
                                      cylinder_circumference_bottom))
  cylinder_side_length = wider_target_circumference * (
      geometry_metadata['height'] / geometry_metadata['width'])
  out = dict(crop)
  out.update({
      'cylinderCircumferenceTop': cylinder_circumference_top,
      'cylinderCircumferenceBottom': _num(cylinder_circumference_bottom),
      'targetCircumferenceTop': _num(target_circumference_top),
      'cylinderSideLength': _num(cylinder_side_length),
      'coniness': _num(coniness),
      'arcAngle': _num(arc_angle),
      'inputMode': 'ADVANCED',
      'unit': 'mm',
      'topRadius': _num(top_radius),
      'bottomRadius': _num(bottom_radius),
  })
  return out


def select_geometry(rl, image_metadata):
  kind = rl.choose('Select the image type:', ['flat', 'cylinder', 'cone'], True)
  if kind == 'flat':
    return {'type': 'PLANAR', 'properties': select_planar_geometry(rl, image_metadata)}
  if kind == 'cylinder':
    return {'type': 'CYLINDER', 'properties': select_cylindrical_geometry(rl, image_metadata)}
  return {'type': 'CONICAL', 'properties': select_conical_geometry(rl, image_metadata)}


def normalize_path(raw):
  if not raw or not isinstance(raw, str):
    raise CliError('Invalid path input')
  p = raw.strip()
  p = re.sub(r'^([\'"])(.*)\1$', r'\2', p, flags=re.S)
  if os.name != 'nt':
    p = p.replace('\\ ', ' ')
  if os.name != 'nt' and p.startswith('~'):
    p = os.path.join(os.path.expanduser('~'), p[1:].lstrip('/'))
  return os.path.abspath(p)


# ---- apply.js ------------------------------------------------------------------------------------
_SAVE_FORMAT = {'jpg': 'JPEG', 'png': 'PNG', 'webp': 'WEBP', 'gif': 'GIF', 'tiff': 'TIFF'}


def format_extension(pil_format):
  """File extension apply.js derives from sharp's metadata().format (jpeg -> jpg). libvips reads
  a JPEG that carries a Multi-Picture block (MPO: Ultra HDR, iPhone HDR, some cameras) as plain
  'jpeg' using its first frame, but PIL reports 'MPO'."""
  fmt = (pil_format or 'PNG').lower()
  return 'jpg' if fmt in ('jpeg', 'mpo') else fmt


def open_oriented(path):
  """sharp(path).autoOrient(): EXIF orientation applied. Returns (image, extension)."""
  with Image.open(path) as img:
    ext = format_extension(img.format)
    img.load()
    oriented = ImageOps.exif_transpose(img)  # a copy, detached from the file
    return (oriented if oriented is not img else img.copy()), ext


def _rotate_cw90(img):
  return img.transpose(Image.Transpose.ROTATE_270)  # sharp.rotate(90) is clockwise


_SIXTEEN_BIT_MODES = ('I;16', 'I;16L', 'I;16B', 'I;16N', 'I')


def _to_gray8(img):
  """convert('L') that keeps the range of 16-bit images. PIL clips I;16 / I values at 255 (a 16-bit
  gray PNG became all 255); libvips scales 16 -> 8 bit by dropping the low byte, as done here."""
  if img.mode in _SIXTEEN_BIT_MODES:
    a = np.asarray(img)
    if img.mode == 'I' and (a.size == 0 or a.max() <= 255):
      return Image.fromarray(a.astype(np.uint8), 'L')  # an 8-bit range stored in 32 bits
    return Image.fromarray((np.clip(a, 0, 65535).astype(np.uint32) >> 8).astype(np.uint8), 'L')
  return img.convert('L')


def _to_gray_libvips(img):
  """sharp(...).grayscale() as libvips colourspace(B_W) does it, as far as it is known: sRGB values
  -> linear light, Y = 0.2126 R + 0.7152 G + 0.0722 B, sRGB encode, round to 8 bit. UNVERIFIED
  against libvips (not installed; the last level may differ by 1 from its lookup table). Alpha is
  dropped. Gray inputs (R = G = B) come out unchanged."""
  if img.mode in _SIXTEEN_BIT_MODES or img.mode == 'L':
    return _to_gray8(img)
  c = np.asarray(img.convert('RGB'), dtype=np.float64) / 255.0
  lin = np.where(c <= 0.04045, c / 12.92, ((c + 0.055) / 1.055) ** 2.4)
  y = lin @ np.array([0.2126, 0.7152, 0.0722])
  v = np.where(y <= 0.0031308, 12.92 * y, 1.055 * np.power(y, 1 / 2.4) - 0.055)
  return Image.fromarray(np.rint(np.clip(v, 0.0, 1.0) * 255.0).astype(np.uint8), 'L')


def _luminance(cropped, gray_mode):
  if gray_mode == 'libvips':
    return _resize_height(_to_gray_libvips(cropped), CONSTANTS['luminanceHeight'])
  return _to_gray8(_resize_height(cropped, CONSTANTS['luminanceHeight']))


def _resize_height(img, height):
  w, h = img.size
  return img.resize((max(1, js_round(w * height / h)), height), Image.Resampling.LANCZOS)


def _save(img, path, ext):
  fmt = _SAVE_FORMAT.get(ext, ext.upper())
  if fmt == 'JPEG':
    if img.mode in _SIXTEEN_BIT_MODES:
      img = _to_gray8(img)
    if img.mode not in ('L', 'RGB'):
      img = img.convert('RGB')
    img.save(path, format='JPEG', quality=80)
  else:
    img.save(path, format=fmt)


def _raw_rgba_like_apply_js(img):
  """apply.js copies sharp raw() bytes (source channel count) into a w*h*4 ImageData buffer."""
  bands = {'L': 1, 'LA': 2, 'RGB': 3, 'RGBA': 4}
  if img.mode not in bands:
    # libvips loads palette images as RGB, or RGBA when the palette has transparency.
    has_alpha = 'A' in img.getbands() or 'transparency' in img.info
    img = img.convert('RGBA' if has_alpha else 'RGB')
  raw = np.frombuffer(img.tobytes(), dtype=np.uint8)
  buf = np.zeros(img.size[0] * img.size[1] * 4, dtype=np.uint8)
  buf[:raw.shape[0]] = raw[:buf.shape[0]]
  return buf.reshape(img.size[1], img.size[0], 4)


def write_pgm(path, gray):
  h, w = gray.shape
  with open(path, 'wb') as f:
    f.write(b'P5\n%d %d\n255\n' % (w, h))
    f.write(np.ascontiguousarray(gray, dtype=np.uint8).tobytes())


def apply_crop(raw_image, ext, crop, folder, name, overwrite_files, extra_data=None,
               timestamp_ms=None, write_luminance_pgm=True, gray_mode='pil'):
  """applyCrop of apply.js. raw_image is the auto-oriented PIL image. Returns the JSON path."""
  base_metadata = {'width': raw_image.size[0], 'height': raw_image.size[1]}
  geometry_image = None
  props = crop['properties']
  if crop['type'] == 'CONICAL':
    if raw_image.mode in _SIXTEEN_BIT_MODES:
      # sharp raw() hands back 2 bytes per sample, which apply.js then reads as 1 byte (garbage).
      raise CliError('16-bit images are not supported for cone targets')
    original_data = _raw_rgba_like_apply_js(raw_image)
    points = compute_pixel_points_from_radius(props['topRadius'], props['bottomRadius'],
                                              base_metadata['width'])
    geometry_data = unconify(original_data, points, base_metadata['width'])
    original_image = raw_image
    geometry_image = Image.fromarray(geometry_data, 'RGBA')
    if props.get('isRotated'):
      geometry_image = _rotate_cw90(geometry_image)
    gh, gw = geometry_data.shape[0], geometry_data.shape[1]
    metadata = ({'width': gh, 'height': gw} if props.get('isRotated')
                else {'width': gw, 'height': gh})
  elif props.get('isRotated'):
    original_image = _rotate_cw90(raw_image)
    metadata = {'width': base_metadata['height'], 'height': base_metadata['width']}
  else:
    original_image = raw_image
    metadata = base_metadata

  issues = validate_crop(props, metadata)
  if issues:
    raise CliError('Invalid crop geometry:\n' + '\n'.join(issues))

  resources = {
      'originalImage': '%s_original.%s' % (name, ext),
      'croppedImage': '%s_cropped.%s' % (name, ext),
      'thumbnailImage': '%s_thumbnail.%s' % (name, ext),
      'luminanceImage': '%s_luminance.%s' % (name, ext),
  }
  if geometry_image is not None:
    resources['geometryImage'] = '%s_geometry.%s' % (name, ext)

  now = int(time.time() * 1000) if timestamp_ms is None else int(timestamp_ms)
  data = dict(crop)
  data.update({
      'imagePath': 'image-targets/%s' % resources['luminanceImage'],
      'metadata': None,
      'name': name,
      'resources': resources,
      'created': now,
      'updated': now,
  })
  if extra_data:
    data.update(extra_data)
  data_path = os.path.join(folder, '%s.json' % name)
  pgm_path = os.path.join(folder, '%s_luminance.pgm' % name)

  source = geometry_image if geometry_image is not None else original_image
  left, top = props['left'], props['top']
  cropped = source.crop((left, top, left + props['width'], top + props['height']))
  thumbnail = _resize_height(cropped, CONSTANTS['thumbnailHeight'])
  luminance = _luminance(cropped, gray_mode)

  planned = [os.path.join(folder, f) for f in resources.values()] + [data_path]
  if write_luminance_pgm:
    planned.append(pgm_path)
  if not overwrite_files:
    for p in planned:
      if os.path.exists(p):
        raise CliError('File already exists, overwrite is disabled: %s' % p)
  os.makedirs(folder, exist_ok=True)

  _save(original_image, os.path.join(folder, resources['originalImage']), ext)
  _save(thumbnail, os.path.join(folder, resources['thumbnailImage']), ext)
  lum_path = os.path.join(folder, resources['luminanceImage'])
  _save(luminance, lum_path, ext)
  _save(cropped, os.path.join(folder, resources['croppedImage']), ext)
  if geometry_image is not None:
    _save(geometry_image, os.path.join(folder, resources['geometryImage']), ext)
  with open(data_path, 'w', encoding='utf-8') as f:
    f.write(js_stringify(data, 2) + '\n')
  if write_luminance_pgm:
    # The bytes a decoder hands the engine: decode the luminance file just written.
    with Image.open(lum_path) as decoded:
      write_pgm(pgm_path, np.asarray(decoded.convert('L')))
  return data_path


def select_processor_options(rl, overwrite_files, timestamp_ms=None, write_luminance_pgm=True,
                             gray_mode='pil'):
  """selectProcessorOptions of interactive.js."""
  image_path = normalize_path(rl.prompt('Enter the path to the image file: ').strip())
  image, ext = open_oriented(image_path)
  metadata = {'width': image.size[0], 'height': image.size[1]}
  geometry = select_geometry(rl, metadata)
  folder = normalize_path(rl.prompt('Enter the output folder: '))
  name = rl.prompt('Enter a name for the image target: ')
  data_path = apply_crop(image, ext, geometry, folder, name, overwrite_files,
                         timestamp_ms=timestamp_ms, write_luminance_pgm=write_luminance_pgm,
                         gray_mode=gray_mode)
  print('Image target data saved to: %s' % data_path)
  return data_path


def answers_from_flags(a):
  """The prompt answers the public CLI would read for the given flags."""
  lines = [a.image, {'flat': 'flat', 'cylinder': 'cylinder', 'cone': 'cone'}[a.type]]
  if a.type == 'cylinder':
    lines += [a.unit, repr(a.cylinder_circumference), repr(a.target_width)]
  elif a.type == 'cone':
    lines += [a.wider_side, repr(a.outer_radius), repr(a.inner_radius), repr(a.arc_angle)]
  if a.orientation is None:
    lines.append('y')
  else:
    if a.top is None or a.left is None or a.width is None:
      raise CliError('--orientation needs --top, --left and --width')
    lines += ['n', a.orientation, str(a.top), str(a.left), str(a.width)]
  lines += [a.out, a.name]
  return lines


def main(argv=None):
  ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
  src = ap.add_argument_group('answer sources')
  src.add_argument('--script', help='answers, one per line (public test/*.txt format)')
  src.add_argument('--image', help='input image (non-interactive mode)')
  ap.add_argument('--type', choices=['flat', 'cylinder', 'cone'], default='flat')
  ap.add_argument('--out', help='output folder')
  ap.add_argument('--name', help='image target name')
  ap.add_argument('--orientation', choices=['portrait', 'landscape'],
                  help='manual crop orientation (omit for the default crop)')
  ap.add_argument('--top', type=int)
  ap.add_argument('--left', type=int)
  ap.add_argument('--width', type=int, help='visual crop width')
  ap.add_argument('--unit', choices=['mm', 'in'], default='mm')
  ap.add_argument('--cylinder-circumference', type=float)
  ap.add_argument('--target-width', type=float, help='printed image width (cylinder)')
  ap.add_argument('--wider-side', choices=['top', 'bottom'], default='top')
  ap.add_argument('--outer-radius', type=float)
  ap.add_argument('--inner-radius', type=float)
  ap.add_argument('--arc-angle', type=float)
  ap.add_argument('--overwrite', action='store_true',
                  help='overwrite existing files (or OVERWRITE_FILES=true like the public CLI)')
  ap.add_argument('--timestamp', type=int, help='created/updated in ms (default: now)')
  ap.add_argument('--no-pgm', action='store_true', help='do not write <name>_luminance.pgm')
  ap.add_argument('--gray-mode', choices=['pil', 'libvips'], default='pil',
                  help='luminance conversion: pil (default; 601 luma on gamma values, after the '
                       'resize) or libvips (linear-light Rec.709 before the resize like sharp; '
                       'UNVERIFIED, see docs/design/image-target.md 2.4)')
  a = ap.parse_args(argv)
  overwrite = a.overwrite or os.environ.get('OVERWRITE_FILES') == 'true'
  try:
    if a.script:
      with open(a.script, encoding='utf-8') as f:
        rl = Answers(lines=f.read().splitlines())
    elif a.image:
      if not a.out or not a.name:
        raise CliError('--image needs --out and --name')
      if a.type == 'cylinder' and (a.cylinder_circumference is None or a.target_width is None):
        raise CliError('--type cylinder needs --cylinder-circumference and --target-width')
      if a.type == 'cone' and None in (a.outer_radius, a.inner_radius, a.arc_angle):
        raise CliError('--type cone needs --outer-radius, --inner-radius and --arc-angle')
      rl = Answers(lines=answers_from_flags(a))
    else:
      rl = Answers(stream=sys.stdin, echo=sys.stdout)
    select_processor_options(rl, overwrite, timestamp_ms=a.timestamp,
                             write_luminance_pgm=not a.no_pgm, gray_mode=a.gray_mode)
  except (CliError, OSError) as e:
    print('Error: %s' % e, file=sys.stderr)
    return 1
  return 0


if __name__ == '__main__':
  sys.exit(main())
