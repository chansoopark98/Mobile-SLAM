"""Unit tests of the Python image target CLI (US-012).

Ports PUB:apps/image-target-cli/src/crop.test.js and interactive.test.js (same inputs and
expected values), then checks the JS number/JSON semantics, unconify, the answer parser and the
end-to-end file output on synthetic images (the public test images are git-lfs pointers).

Run (repo root): python3 -I -m unittest discover -s slam_refact/tools/image_target/tests -v
"""
import io
import json
import math
import os
import pathlib
import sys
import tempfile
import unittest

import numpy as np
from PIL import Image

TOOL = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOL))

import diff_target_json  # noqa: E402
import image_target_cli as cli  # noqa: E402


class MockRl:
  """mockRl of interactive.test.js: every prompt returns the next value as given."""

  def __init__(self, *values):
    self._values = list(values)

  def _next(self, *args, **kwargs):
    if not self._values:
      raise RuntimeError('No more mock values available')
    return self._values.pop(0)

  choose = confirm = prompt_integer = prompt_float = prompt = _next


class GetDefaultCropTest(unittest.TestCase):
  """crop.test.js."""

  def test_portrait_crop_for_portrait_image(self):
    crop = cli.get_default_crop({'width': 995, 'height': 1326}, False)
    self.assertEqual(crop['isRotated'], False)
    self.assertEqual(crop['originalWidth'], 995)
    self.assertEqual(crop['originalHeight'], 1326)
    self.assertEqual(crop['width'], cli.js_round((1326 * 3) / 4))
    self.assertEqual(crop['height'], 1326)
    self.assertGreaterEqual(crop['left'], 0)
    self.assertEqual(crop['top'], 0)
    self.assertLessEqual(crop['left'] + crop['width'], 995)

  def test_landscape_crop_for_portrait_image_rotated(self):
    crop = cli.get_default_crop({'width': 995, 'height': 1326}, True)
    self.assertEqual(crop['isRotated'], True)
    self.assertEqual(crop['originalWidth'], 1326)
    self.assertEqual(crop['originalHeight'], 995)
    self.assertEqual(crop['height'], 995)
    self.assertEqual(crop['width'], cli.js_round((995 * 3) / 4))
    self.assertGreaterEqual(crop['left'], 0)
    self.assertEqual(crop['top'], 0)
    self.assertLessEqual(crop['left'] + crop['width'], 1326)

  def test_landscape_crop_for_landscape_image(self):
    crop = cli.get_default_crop({'width': 1600, 'height': 900}, True)
    self.assertEqual(crop['isRotated'], True)
    self.assertEqual(crop['originalWidth'], 900)
    self.assertEqual(crop['originalHeight'], 1600)
    self.assertLessEqual(crop['left'] + crop['width'], 900)
    self.assertLessEqual(crop['top'] + crop['height'], 1600)

  def test_portrait_crop_for_square_image(self):
    crop = cli.get_default_crop({'width': 1000, 'height': 1000}, False)
    self.assertEqual(crop['originalWidth'], 1000)
    self.assertEqual(crop['originalHeight'], 1000)
    self.assertEqual(crop['width'], cli.js_round((1000 * 3) / 4))
    self.assertEqual(crop['height'], 1000)


class SelectPlanarGeometryTest(unittest.TestCase):
  """interactive.test.js selectPlanarGeometry."""

  def test_landscape_default_crop(self):
    crop = cli.select_planar_geometry(MockRl(True), {'width': 1200, 'height': 600})
    self.assertEqual(crop['isRotated'], True)
    self.assertEqual(crop['originalWidth'], 600)
    self.assertEqual(crop['originalHeight'], 1200)
    self.assertEqual(crop['width'], 600)
    self.assertEqual(crop['height'], 800)
    self.assertEqual(crop['left'], 0)
    self.assertEqual(crop['top'], 200)

  def test_portrait_default_crop(self):
    crop = cli.select_planar_geometry(MockRl(True), {'width': 600, 'height': 800})
    self.assertEqual(crop['isRotated'], False)
    self.assertEqual((crop['width'], crop['height'], crop['left'], crop['top']), (600, 800, 0, 0))

  def test_portrait_manual_crop(self):
    crop = cli.select_planar_geometry(MockRl(False, 'portrait', 10, 20, 600),
                                      {'width': 600, 'height': 800})
    self.assertEqual(crop, {'top': 10, 'left': 20, 'width': 600, 'height': 800, 'isRotated': False,
                            'originalWidth': 600, 'originalHeight': 800})

  def test_landscape_manual_crop(self):
    crop = cli.select_planar_geometry(MockRl(False, 'landscape', 0, 0, 400),
                                      {'width': 600, 'height': 800})
    self.assertEqual(crop['width'], 300)
    self.assertEqual(crop['height'], 400)
    self.assertEqual(crop['isRotated'], True)


class SelectCylindricalGeometryTest(unittest.TestCase):
  """interactive.test.js selectCylindricalGeometry (deepEqual, key order checked too)."""

  def check(self, crop, expected):
    self.assertEqual(crop, expected)

  def test_landscape_default_crop(self):
    crop = cli.select_cylindrical_geometry(MockRl('mm', 1000, 500, True),
                                           {'width': 1200, 'height': 600})
    self.check(crop, {
        'isRotated': True, 'originalWidth': 600, 'originalHeight': 1200, 'width': 600,
        'height': 800, 'left': 0, 'top': 200, 'targetCircumferenceTop': 500,
        'cylinderSideLength': (600 / 1200) * 500, 'cylinderCircumferenceTop': 1000,
        'cylinderCircumferenceBottom': 1000, 'arcAngle': 180, 'coniness': 0,
        'inputMode': 'ADVANCED', 'unit': 'mm'})

  def test_portrait_default_crop(self):
    crop = cli.select_cylindrical_geometry(MockRl('mm', 3000, 1000, True),
                                           {'width': 600, 'height': 1200})
    self.check(crop, {
        'isRotated': False, 'originalWidth': 600, 'originalHeight': 1200, 'width': 600,
        'height': 800, 'left': 0, 'top': 200, 'targetCircumferenceTop': 1000,
        'cylinderSideLength': (1200 / 600) * 1000, 'cylinderCircumferenceTop': 3000,
        'cylinderCircumferenceBottom': 3000, 'arcAngle': 120, 'coniness': 0,
        'inputMode': 'ADVANCED', 'unit': 'mm'})

  def test_love_jpg_default_crop(self):
    crop = cli.select_cylindrical_geometry(MockRl('mm', 30, 10, True),
                                           {'width': 1500, 'height': 1128})
    self.check(crop, {
        'top': 0, 'left': 2, 'width': 1125, 'height': 1500, 'isRotated': True,
        'originalWidth': 1128, 'originalHeight': 1500, 'cylinderCircumferenceTop': 30,
        'cylinderCircumferenceBottom': 30, 'targetCircumferenceTop': 10,
        'cylinderSideLength': 7.52, 'inputMode': 'ADVANCED', 'coniness': 0, 'arcAngle': 120,
        'unit': 'mm'})

  def test_tower_jpg_manual_crop(self):
    crop = cli.select_cylindrical_geometry(
        MockRl('mm', 100, 25, False, 'portrait', 82, 0, 1000, 1333),
        {'width': 1000, 'height': 1500})
    self.check(crop, {
        'top': 82, 'left': 0, 'width': 1000, 'height': 1333, 'isRotated': False,
        'originalWidth': 1000, 'originalHeight': 1500, 'cylinderCircumferenceTop': 100,
        'cylinderCircumferenceBottom': 100, 'targetCircumferenceTop': 25,
        'cylinderSideLength': 37.5, 'inputMode': 'ADVANCED', 'coniness': 0, 'arcAngle': 90,
        'unit': 'mm'})

  def test_target_wider_than_cylinder_is_an_error(self):
    with self.assertRaises(cli.CliError):
      cli.select_cylindrical_geometry(MockRl('mm', 10, 20), {'width': 600, 'height': 800})


class SelectConicalGeometryTest(unittest.TestCase):
  """interactive.test.js selectConicalGeometry."""

  BASE = {'cylinderCircumferenceTop': 100, 'targetCircumferenceTop': 50, 'inputMode': 'ADVANCED',
          'arcAngle': 180, 'unit': 'mm', 'bottomRadius': 600}

  def expected(self, crop, wide_top):
    e = dict(crop)
    e.update(self.BASE)
    e.update({'cylinderCircumferenceBottom': 40 if wide_top else 250,
              'cylinderSideLength': 20.575 if wide_top else 51.4375,
              'coniness': 1.3219280948873624 if wide_top else -1.3219280948873624,
              'topRadius': 1500 if wide_top else -1500})
    return e

  def test_wide_top_default_crop(self):
    crop = cli.select_conical_geometry(MockRl('top', 1500, 600, 180, True),
                                       {'width': 2000, 'height': 1250})
    self.assertEqual(crop, self.expected({
        'top': 452, 'left': 0, 'width': 823, 'height': 1097, 'isRotated': True,
        'originalWidth': 823, 'originalHeight': 2000}, True))

  def test_wide_top_portrait_crop(self):
    crop = cli.select_conical_geometry(
        MockRl('top', 1500, 600, 180, False, 'portrait', 0, 692, 617, 823),
        {'width': 2000, 'height': 1250})
    self.assertEqual(crop, self.expected({
        'top': 0, 'left': 692, 'width': 617, 'height': 823, 'isRotated': False,
        'originalWidth': 2000, 'originalHeight': 823}, True))

  def test_wide_bottom_default_crop(self):
    crop = cli.select_conical_geometry(MockRl('bottom', 1500, 600, 180, True),
                                       {'width': 2000, 'height': 1250})
    self.assertEqual(crop, self.expected({
        'top': 452, 'left': 0, 'width': 823, 'height': 1097, 'isRotated': True,
        'originalWidth': 823, 'originalHeight': 2000}, False))

  def test_wide_bottom_portrait_crop(self):
    crop = cli.select_conical_geometry(
        MockRl('bottom', 1500, 600, 180, False, 'portrait', 0, 692, 617, 823),
        {'width': 2000, 'height': 1250})
    self.assertEqual(crop, self.expected({
        'top': 0, 'left': 692, 'width': 617, 'height': 823, 'isRotated': False,
        'originalWidth': 2000, 'originalHeight': 823}, False))

  def test_invalid_cone_inputs(self):
    meta = {'width': 2000, 'height': 1250}
    for values in (('top', 0), ('top', 1500, -1), ('top', 1500, 600, 0), ('top', 1500, 600, 361)):
      with self.assertRaises(cli.CliError):
        cli.select_conical_geometry(MockRl(*values), meta)


class NormalizePathTest(unittest.TestCase):
  """interactive.test.js normalizePath."""

  def test_cases(self):
    self.assertEqual(cli.normalize_path('/foo/bar'), '/foo/bar')
    self.assertEqual(cli.normalize_path('"/foo/bar"'), '/foo/bar')
    self.assertEqual(cli.normalize_path("'/foo/bar'"), '/foo/bar')
    self.assertEqual(cli.normalize_path('/foo/bar\\ baz'), '/foo/bar baz')
    self.assertEqual(cli.normalize_path('~/docs'), os.path.join(os.path.expanduser('~'), 'docs'))
    with self.assertRaisesRegex(cli.CliError, 'Invalid path input'):
      cli.normalize_path('')
    with self.assertRaisesRegex(cli.CliError, 'Invalid path input'):
      cli.normalize_path(None)


class JsSemanticsTest(unittest.TestCase):

  def test_round_half_up(self):
    self.assertEqual(cli.js_round(2.5), 3)
    self.assertEqual(cli.js_round(-2.5), -2)
    self.assertEqual(cli.js_round(822.75), 823)

  def test_number_parsing(self):
    self.assertEqual(cli.js_number(''), 0)
    self.assertEqual(cli.js_number(' 12 '), 12)
    self.assertEqual(cli.js_number('1e3'), 1000)
    self.assertEqual(cli.js_number('0x10'), 16)
    self.assertEqual(cli.js_number('2.5'), 2.5)
    self.assertTrue(math.isnan(cli.js_number('1_000')))
    self.assertTrue(math.isnan(cli.js_number('abc')))
    self.assertTrue(math.isnan(cli.js_number('nan')))
    self.assertEqual(cli.js_number('-Infinity'), float('-inf'))
    self.assertEqual(cli.js_parse_int('2abc'), 2)
    self.assertIsNone(cli.js_parse_int('x2'))

  def test_number_to_string(self):
    cases = {250.0: '250', 7.52: '7.52', 1.3219280948873624: '1.3219280948873624',
             1e21: '1e+21', 1e-7: '1e-7', 0.00001: '0.00001', -0.5: '-0.5', 123456789012.0:
             '123456789012', 1776988431956: '1776988431956', 20.575: '20.575'}
    for value, text in cases.items():
      self.assertEqual(cli.js_number_to_string(value), text, value)

  def test_stringify_matches_json_stringify_layout(self):
    data = {'type': 'PLANAR', 'properties': {'top': 1, 'isRotated': False, 'w': 2.0},
            'metadata': None, 'empty': {}, 'list': [], 'name': 'a"\n\u00e9'}
    text = cli.js_stringify(data, 2)
    self.assertEqual(text, '{\n  "type": "PLANAR",\n  "properties": {\n    "top": 1,\n'
                     '    "isRotated": false,\n    "w": 2\n  },\n  "metadata": null,\n'
                     '  "empty": {},\n  "list": [],\n  "name": "a\\"\\n\u00e9"\n}')
    self.assertEqual(json.loads(text)['properties']['w'], 2)


class AnswersTest(unittest.TestCase):
  """cli.js prompt semantics: defaults, numeric choices, re-prompt on invalid input."""

  def test_choose_confirm_and_numbers(self):
    rl = cli.Answers(lines=['', '2', 'bad', 'cone', 'maybe', 'Y', '', '1.5', '7', 'x', '0.25'])
    self.assertEqual(rl.choose('q', ['flat', 'cylinder'], True), 'flat')
    self.assertEqual(rl.choose('q', ['flat', 'cylinder'], True), 'cylinder')
    self.assertEqual(rl.choose('q', ['flat', 'cone'], True), 'cone')  # 'bad' re-prompts
    self.assertTrue(rl.confirm('q', True))  # 'maybe' re-prompts
    self.assertTrue(rl.confirm('q', True))  # '' -> default
    self.assertEqual(rl.prompt_integer('q'), 7)  # '1.5' is not an integer
    self.assertEqual(rl.prompt_float('q'), 0.25)  # 'x' is NaN
    with self.assertRaises(cli.CliError):
      rl.prompt('q')

  def test_flags_build_public_answer_order(self):
    class A:
      image = 'in.png'
      type = 'cone'
      wider_side = 'bottom'
      outer_radius = 1500.0
      inner_radius = 600.0
      arc_angle = 180.0
      orientation = 'portrait'
      top, left, width = 0, 692, 617
      out = 'o'
      name = 'n'
    self.assertEqual(cli.answers_from_flags(A), ['in.png', 'cone', 'bottom', '1500.0', '600.0',
                                                 '180.0', 'n', 'portrait', '0', '692', '617',
                                                 'o', 'n'])


class UnconifyTest(unittest.TestCase):

  def test_pixel_points_and_output_height(self):
    pts = cli.compute_pixel_points_from_radius(1500, 600, 2000)
    self.assertAlmostEqual(pts['apex'][0], 1000)
    self.assertAlmostEqual(pts['tr'][0], 1999)
    self.assertFalse(pts['isFez'])
    self.assertTrue(cli.compute_pixel_points_from_radius(-1500, 600, 2000)['isFez'])
    # The conical CLI tests imply an 823 px tall geometry image for a 2000 px wide input.
    self.assertEqual(cli.get_unconified_data(pts, 2000)['outputHeight'], 823)

  def test_unconify_flattens_a_ring_texture(self):
    # A ring image whose gray level encodes the radius from the apex: every output row of the
    # flattened image samples one radius, so rows are (nearly) constant and monotonic.
    width, height = 400, 300
    top_r, bottom_r = 300.0, 120.0
    pts = cli.compute_pixel_points_from_radius(top_r, bottom_r, width)
    yy, xx = np.mgrid[0:height, 0:width].astype(np.float64)
    r = np.hypot(xx - pts['apex'][0], yy - pts['apex'][1])
    gray = np.clip((r - bottom_r) / (top_r - bottom_r) * 200 + 20, 0, 255).astype(np.uint8)
    rgba = np.dstack([gray, gray, gray, np.full_like(gray, 255)])
    out = cli.unconify(rgba, pts, width)
    self.assertEqual(out.shape[1], width)
    inside = out[:, :, 3] == 255
    self.assertGreater(inside.mean(), 0.9)
    row_means = [out[i, inside[i], 0].mean() for i in range(out.shape[0]) if inside[i].any()]
    self.assertGreater(row_means[0], row_means[-1])  # top row = outer radius = brighter
    for i in range(out.shape[0]):
      vals = out[i, inside[i], 0].astype(int)
      if vals.size > 10:
        self.assertLessEqual(vals.max() - vals.min(), 3, i)


def synthetic_rgb(width, height, seed=1):
  rng = np.random.default_rng(seed)
  img = np.zeros((height, width, 3), dtype=np.uint8)
  img[:] = (np.linspace(40, 200, width)[None, :, None]).astype(np.uint8)
  for _ in range(300):
    x, y = rng.integers(0, width), rng.integers(0, height)
    w, h = rng.integers(5, 60, size=2)
    img[y:y + h, x:x + w] = rng.integers(0, 256, size=3)
  return img


class ApplyCropTest(unittest.TestCase):
  """End-to-end output of select_processor_options on synthetic images."""

  def run_cli(self, folder, image, answers_after_image, name, ext='png', **kwargs):
    path = os.path.join(folder, 'input.' + ext)
    image.save(path)
    rl = cli.Answers(lines=[path] + answers_after_image + [os.path.join(folder, 'out'), name])
    data_path = cli.select_processor_options(rl, overwrite_files=False, timestamp_ms=1234,
                                             **kwargs)
    with open(data_path, encoding='utf-8') as f:
      text = f.read()
    return data_path, json.loads(text), text

  def test_flat_default_crop_png(self):
    with tempfile.TemporaryDirectory() as d:
      img = Image.fromarray(synthetic_rgb(900, 1500))
      data_path, data, text = self.run_cli(d, img, ['flat', 'y'], 'flat-target')
      self.assertTrue(text.endswith('}\n'))
      self.assertEqual(list(data.keys()), ['type', 'properties', 'imagePath', 'metadata', 'name',
                                           'resources', 'created', 'updated'])
      self.assertEqual(data['type'], 'PLANAR')
      self.assertEqual(data['properties'], {'left': 0, 'top': 150, 'width': 900, 'height': 1200,
                                            'isRotated': False, 'originalWidth': 900,
                                            'originalHeight': 1500})
      self.assertEqual(data['imagePath'], 'image-targets/flat-target_luminance.png')
      self.assertEqual(data['resources'], {
          'originalImage': 'flat-target_original.png', 'croppedImage': 'flat-target_cropped.png',
          'thumbnailImage': 'flat-target_thumbnail.png',
          'luminanceImage': 'flat-target_luminance.png'})
      self.assertEqual((data['created'], data['updated']), (1234, 1234))
      out = os.path.dirname(data_path)
      with Image.open(os.path.join(out, 'flat-target_luminance.png')) as lum:
        self.assertEqual(lum.size, (480, 640))
        self.assertEqual(lum.mode, 'L')
        lum_px = np.asarray(lum)
      with Image.open(os.path.join(out, 'flat-target_thumbnail.png')) as th:
        self.assertEqual(th.size, (263, 350))  # README: thumbnail 263x350
      with Image.open(os.path.join(out, 'flat-target_cropped.png')) as cr:
        self.assertEqual(cr.size, (900, 1200))
      with open(os.path.join(out, 'flat-target_luminance.pgm'), 'rb') as f:
        pgm = f.read()
      header = b'P5\n480 640\n255\n'
      self.assertTrue(pgm.startswith(header))
      self.assertEqual(pgm[len(header):], lum_px.tobytes())
      # Overwrite protection like the public CLI.
      with self.assertRaises(cli.CliError):
        self.run_cli(d, img, ['flat', 'y'], 'flat-target')

  def test_landscape_jpeg_is_rotated(self):
    with tempfile.TemporaryDirectory() as d:
      img = Image.fromarray(synthetic_rgb(1600, 1000))
      data_path, data, _ = self.run_cli(d, img, ['flat', 'y'], 'land', ext='jpg')
      props = data['properties']
      self.assertTrue(props['isRotated'])
      self.assertEqual((props['originalWidth'], props['originalHeight']), (1000, 1600))
      out = os.path.dirname(data_path)
      with Image.open(os.path.join(out, 'land_original.jpg')) as orig:
        self.assertEqual(orig.size, (1000, 1600))  # rotated 90 degrees clockwise
      with Image.open(os.path.join(out, 'land_luminance.jpg')) as lum:
        self.assertEqual(lum.size[1], 640)
      self.assertEqual(data['resources']['luminanceImage'], 'land_luminance.jpg')

  def test_exif_orientation_is_applied(self):
    with tempfile.TemporaryDirectory() as d:
      img = Image.fromarray(synthetic_rgb(1200, 900))
      exif = Image.Exif()
      exif[0x0112] = 6  # rotate 90 CW to display
      path = os.path.join(d, 'exif.jpg')
      img.save(path, exif=exif.tobytes())
      rl = cli.Answers(lines=[path, 'flat', 'y', os.path.join(d, 'out'), 'exif'])
      data_path = cli.select_processor_options(rl, False, timestamp_ms=1)
      with open(data_path, encoding='utf-8') as f:
        props = json.load(f)['properties']
      # Displayed 900x1200 portrait: default portrait crop, no rotation.
      self.assertFalse(props['isRotated'])
      self.assertEqual((props['originalWidth'], props['originalHeight']), (900, 1200))

  def test_cylinder_json(self):
    with tempfile.TemporaryDirectory() as d:
      img = Image.fromarray(synthetic_rgb(1500, 1128))
      _, data, _ = self.run_cli(d, img, ['cylinder', 'mm', '30', '10', 'y'], 'cyl')
      self.assertEqual(data['type'], 'CYLINDER')
      self.assertEqual(data['properties'], {
          'top': 0, 'left': 2, 'width': 1125, 'height': 1500, 'isRotated': True,
          'originalWidth': 1128, 'originalHeight': 1500, 'targetCircumferenceTop': 10,
          'cylinderCircumferenceTop': 30, 'cylinderCircumferenceBottom': 30,
          'cylinderSideLength': 7.52, 'arcAngle': 120, 'coniness': 0, 'inputMode': 'ADVANCED',
          'unit': 'mm'})

  def test_cone_writes_geometry_image(self):
    with tempfile.TemporaryDirectory() as d:
      rgba = np.dstack([synthetic_rgb(2000, 1250), np.full((1250, 2000), 255, np.uint8)])
      img = Image.fromarray(rgba, 'RGBA')
      data_path, data, _ = self.run_cli(d, img, ['cone', 'top', '1500', '600', '180', 'y'], 'cone')
      self.assertEqual(data['type'], 'CONICAL')
      self.assertEqual(data['resources']['geometryImage'], 'cone_geometry.png')
      self.assertEqual(data['properties']['originalWidth'], 823)
      out = os.path.dirname(data_path)
      with Image.open(os.path.join(out, 'cone_geometry.png')) as geo:
        self.assertEqual(geo.size, (823, 2000))  # 2000x823 flattened, rotated (isRotated)
      with Image.open(os.path.join(out, 'cone_original.png')) as orig:
        self.assertEqual(orig.size, (2000, 1250))  # conical original is not rotated

  def test_too_small_crop_is_rejected(self):
    with tempfile.TemporaryDirectory() as d:
      img = Image.fromarray(synthetic_rgb(400, 500))
      with self.assertRaisesRegex(cli.CliError, 'Width must be at least 480'):
        self.run_cli(d, img, ['flat', 'y'], 'small')

  def test_main_with_flags_and_script(self):
    with tempfile.TemporaryDirectory() as d:
      src = os.path.join(d, 'in.png')
      Image.fromarray(synthetic_rgb(960, 1280)).save(src)
      out = os.path.join(d, 'out')
      rc = cli.main(['--image', src, '--type', 'flat', '--out', out, '--name', 'f',
                     '--timestamp', '5'])
      self.assertEqual(rc, 0)
      script = os.path.join(d, 'case.txt')
      with open(script, 'w') as f:
        f.write('\n'.join([src, 'flat', 'y', out, 's']) + '\n')
      self.assertEqual(cli.main(['--script', script, '--timestamp', '5', '--no-pgm']), 0)
      self.assertFalse(os.path.exists(os.path.join(out, 's_luminance.pgm')))
      with open(os.path.join(out, 'f.json')) as fa, open(os.path.join(out, 's.json')) as fb:
        a, b = json.load(fa), json.load(fb)
      diffs = diff_target_json.compare(a, b)
      self.assertEqual(diffs, ['/imagePath: '
                               "'image-targets/f_luminance.png' != 'image-targets/s_luminance.png'",
                               "/name: 'f' != 's'",
                               "/resources/originalImage: 'f_original.png' != 's_original.png'",
                               "/resources/croppedImage: 'f_cropped.png' != 's_cropped.png'",
                               "/resources/thumbnailImage: 'f_thumbnail.png' != 's_thumbnail.png'",
                               "/resources/luminanceImage: 'f_luminance.png' != 's_luminance.png'"])
      err = io.StringIO()
      old = sys.stderr
      sys.stderr = err
      try:
        self.assertEqual(cli.main(['--image', src, '--out', out]), 1)
      finally:
        sys.stderr = old
      self.assertIn('--name', err.getvalue())


class DiffTargetJsonTest(unittest.TestCase):

  def test_ignores_time_and_reports_differences(self):
    a = {'type': 'PLANAR', 'properties': {'top': 1, 'left': 2.0}, 'created': 1, 'updated': 2}
    b = {'type': 'PLANAR', 'properties': {'top': 1, 'left': 2}, 'created': 3, 'updated': 4}
    self.assertEqual(diff_target_json.compare(a, b), [])
    self.assertEqual(len(diff_target_json.compare(a, b, strict_time=True)), 2)
    c = {'type': 'PLANAR', 'properties': {'left': 2, 'top': 1.5}}
    self.assertEqual(diff_target_json.compare(a, c),
                     ["/properties: keys ['top', 'left'] != ['left', 'top']",
                      '/properties/top: 1 != 1.5'])
    self.assertEqual(diff_target_json.compare(a, c, abs_tol=0.5)[1:], [])


class RmTwoRegressionTest(unittest.TestCase):
  """RM-2 (review opus-xhigh-r1-it-core MAJOR-1/2, MINOR-1/2/9)."""

  def run_cli(self, folder, path, answers_after_image, name, **kwargs):
    rl = cli.Answers(lines=[path] + answers_after_image + [os.path.join(folder, 'out'), name])
    data_path = cli.select_processor_options(rl, overwrite_files=False, timestamp_ms=1234, **kwargs)
    with open(data_path, encoding='utf-8') as f:
      text = f.read()
    return data_path, json.loads(text), text

  # -- MAJOR-1: MPO ------------------------------------------------------------------------------
  def test_format_extension_follows_sharp(self):
    self.assertEqual(cli.format_extension('JPEG'), 'jpg')
    self.assertEqual(cli.format_extension('MPO'), 'jpg')
    self.assertEqual(cli.format_extension('PNG'), 'png')
    self.assertEqual(cli.format_extension('WEBP'), 'webp')
    self.assertEqual(cli.format_extension(None), 'png')

  def test_mpo_jpeg_is_named_jpg(self):
    with tempfile.TemporaryDirectory() as d:
      first = Image.fromarray(synthetic_rgb(960, 1280))
      second = Image.fromarray(synthetic_rgb(960, 1280, seed=2))
      path = os.path.join(d, 'photo.jpg')
      first.save(path, format='MPO', save_all=True, append_images=[second])
      with Image.open(path) as probe:
        self.assertEqual(probe.format, 'MPO')  # the input really is a Multi-Picture file
      data_path, data, _ = self.run_cli(d, path, ['flat', 'y'], 'mpo')
      self.assertEqual(data['imagePath'], 'image-targets/mpo_luminance.jpg')
      self.assertEqual(data['resources'], {
          'originalImage': 'mpo_original.jpg', 'croppedImage': 'mpo_cropped.jpg',
          'thumbnailImage': 'mpo_thumbnail.jpg', 'luminanceImage': 'mpo_luminance.jpg'})
      out = os.path.dirname(data_path)
      self.assertEqual(sorted(f for f in os.listdir(out) if f.endswith('.mpo')), [])
      for f in data['resources'].values():
        with Image.open(os.path.join(out, f)) as img:
          self.assertEqual(img.format, 'JPEG')  # a single-frame JPEG, not another MPO

  # -- MAJOR-2: grayscale --------------------------------------------------------------------------
  def test_libvips_gray_matches_the_known_formula(self):
    def gray(c):
      return int(np.asarray(cli._to_gray_libvips(Image.new('RGB', (2, 2), c)))[0, 0])

    def pil(c):
      return int(np.asarray(cli._to_gray8(Image.new('RGB', (2, 2), c)))[0, 0])

    # Numbers quoted in docs/design/image-target.md 2.4.
    self.assertEqual([gray(c) for c in [(255, 0, 0), (0, 255, 0), (0, 0, 255)]], [127, 220, 76])
    self.assertEqual([pil(c) for c in [(255, 0, 0), (0, 255, 0), (0, 0, 255)]], [76, 150, 29])
    for level in (0, 1, 17, 100, 254, 255):  # gray stays gray in both conversions
      self.assertEqual(gray((level,) * 3), level)
      self.assertEqual(pil((level,) * 3), level)
    # Brightness order flips: PIL puts red below gray 100, the linear-light conversion above it.
    self.assertLess(pil((255, 0, 0)), pil((100, 100, 100)))
    self.assertGreater(gray((255, 0, 0)), gray((100, 100, 100)))

  def test_gray_mode_changes_only_the_luminance(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, 'in.png')
      Image.fromarray(synthetic_rgb(960, 1280)).save(path)
      outs = {}
      for mode in ('pil', 'libvips'):
        folder = os.path.join(d, mode)
        os.makedirs(folder)
        data_path, data, _ = self.run_cli(folder, path, ['flat', 'y'], 't', gray_mode=mode)
        out = os.path.dirname(data_path)
        with Image.open(os.path.join(out, 't_luminance.png')) as lum:
          arr = np.asarray(lum)
        with open(os.path.join(out, 't_luminance.pgm'), 'rb') as f:
          pgm = f.read()
        self.assertTrue(pgm.endswith(arr.tobytes()), mode)  # the PGM is the decoded file
        outs[mode] = (data, arr)
      self.assertEqual(outs['pil'][0], outs['libvips'][0])  # JSON identical
      diff = np.abs(outs['pil'][1].astype(int) - outs['libvips'][1].astype(int))
      self.assertGreater(diff.mean(), 1.0)  # coloured test image: the conversions really differ
      self.assertEqual(outs['pil'][1].shape, (640, 480))
      self.assertEqual(outs['libvips'][1].shape, (640, 480))

  # -- MINOR-1: 16-bit ----------------------------------------------------------------------------
  def test_16_bit_gray_png_keeps_its_range(self):
    with tempfile.TemporaryDirectory() as d:
      ramp = np.tile(np.linspace(0, 65535, 960).astype(np.uint16), (1280, 1))
      path = os.path.join(d, 'g16.png')
      Image.fromarray(ramp).save(path)
      with Image.open(path) as probe:
        self.assertEqual(probe.mode, 'I;16')
      for mode in ('pil', 'libvips'):
        folder = os.path.join(d, mode)
        os.makedirs(folder)
        data_path, _, _ = self.run_cli(folder, path, ['flat', 'y'], 'g', gray_mode=mode)
        out = os.path.dirname(data_path)
        with Image.open(os.path.join(out, 'g_luminance.png')) as lum:
          arr = np.asarray(lum)
        self.assertEqual(arr.dtype, np.uint8, mode)
        self.assertGreater(len(np.unique(arr)), 100, mode)  # a ramp, not a constant 255
        self.assertLess(int(arr[:, 0].mean()), 5, mode)
        self.assertGreater(int(arr[:, -1].mean()), 250, mode)
        with open(os.path.join(out, 'g_luminance.pgm'), 'rb') as f:
          self.assertTrue(f.read().endswith(arr.tobytes()))

  def test_16_bit_to_jpeg_and_gray8_helper(self):
    a = np.array([[0, 255, 256, 4096, 65535]], dtype=np.uint16)
    self.assertEqual(np.asarray(cli._to_gray8(Image.fromarray(a))).tolist(), [[0, 0, 1, 16, 255]])
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, 'x.jpg')
      cli._save(Image.fromarray(np.tile(np.linspace(0, 65535, 64).astype(np.uint16), (64, 1))),
                path, 'jpg')
      with Image.open(path) as img:
        arr = np.asarray(img.convert('L'))
      self.assertLess(int(arr[:, 0].mean()), 10)
      self.assertGreater(int(arr[:, -1].mean()), 245)

  def test_16_bit_cone_is_an_explicit_error(self):
    with tempfile.TemporaryDirectory() as d:
      path = os.path.join(d, 'c16.png')
      Image.fromarray(np.tile(np.linspace(0, 65535, 2000).astype(np.uint16), (1250, 1))).save(path)
      with self.assertRaisesRegex(cli.CliError, '16-bit images are not supported for cone'):
        self.run_cli(d, path, ['cone', 'top', '1500', '600', '180', 'y'], 'c')

  # -- MINOR-2: cone inner radius 0 -----------------------------------------------------------------
  def test_js_arithmetic_helpers(self):
    inf = float('inf')
    self.assertEqual(cli.js_div(1, 0), inf)
    self.assertEqual(cli.js_div(-1, 0), -inf)
    self.assertEqual(cli.js_div(1, -0.0), -inf)
    self.assertTrue(math.isnan(cli.js_div(0, 0)))
    self.assertEqual(cli.js_div(6, 3), 2)
    self.assertEqual(cli.js_log2(inf), inf)
    self.assertEqual(cli.js_log2(0), -inf)
    self.assertTrue(math.isnan(cli.js_log2(-1)))
    self.assertEqual(cli.js_log2(8), 3)
    self.assertTrue(math.isnan(cli.js_asin(1.5)))
    self.assertAlmostEqual(cli.js_asin(1), math.pi / 2)
    with self.assertRaises(cli.CliError):
      cli.js_ceil_int(float('nan'))

  def test_negative_zero_survives_number_parsing_and_negation(self):
    # RM-1b, review MINOR-5. JavaScript: Number('-0') is -0 and -x of a zero is -0.
    self.assertTrue(math.copysign(1.0, cli.js_number('-0')) < 0)
    self.assertEqual(math.copysign(1.0, cli.js_number('0')), 1.0)
    self.assertTrue(math.copysign(1.0, cli.js_neg(0)) < 0)
    self.assertTrue(math.copysign(1.0, cli.js_neg(-0.0)) > 0)
    self.assertEqual(cli.js_neg(5), -5)
    self.assertEqual(cli.js_div(100, cli.js_div(cli.js_neg(0), -1500)), float('inf'))  # 100 / +0
    rl = cli.Answers(lines=['-0'])
    self.assertEqual(rl.prompt_integer('q'), 0)

  def test_fez_cone_with_zero_inner_radius_matches_the_public_javascript(self):
    # curved-geometry.js: getCircumferenceRatio(-1500, 0) = -0 / -1500 = +0, so the bottom
    # circumference is 100 / +0 = Infinity and the wider circumference (the side length factor)
    # is Infinity. Before the fix the ratio was -0.0, the bottom circumference -Infinity, and
    # cylinderSideLength came out as 34.3 instead of null (JSON.stringify(Infinity)).
    with tempfile.TemporaryDirectory() as d:
      rgba = np.dstack([synthetic_rgb(2000, 1250), np.full((1250, 2000), 255, np.uint8)])
      path = os.path.join(d, 'fez.png')
      Image.fromarray(rgba, 'RGBA').save(path)
      data_path, data, text = self.run_cli(d, path, ['cone', 'bottom', '1500', '0', '180', 'y'], 'fz')
      props = data['properties']
      self.assertIsNone(props['cylinderCircumferenceBottom'])  # 100 / +0 = Infinity
      self.assertIsNone(props['cylinderSideLength'])
      self.assertIsNone(props['coniness'])  # -log2(Infinity)
      self.assertEqual((props['topRadius'], props['bottomRadius']), (-1500, 0))
      self.assertIn('"cylinderSideLength": null', text)

  def test_cone_with_zero_inner_radius_writes_json_like_the_public_cli(self):
    with tempfile.TemporaryDirectory() as d:
      rgba = np.dstack([synthetic_rgb(2000, 1250), np.full((1250, 2000), 255, np.uint8)])
      path = os.path.join(d, 'cone.png')
      Image.fromarray(rgba, 'RGBA').save(path)
      data_path, data, text = self.run_cli(d, path, ['cone', 'top', '1500', '0', '180', 'y'], 'c0')
      props = data['properties']
      self.assertIsNone(props['coniness'])  # Infinity -> null in JSON.stringify
      self.assertEqual(props['cylinderCircumferenceBottom'], 0)  # 100 / Infinity
      self.assertEqual((props['topRadius'], props['bottomRadius']), (1500, 0))
      self.assertIn('"coniness": null', text)
      self.assertEqual(data['type'], 'CONICAL')
      self.assertTrue(os.path.exists(os.path.join(os.path.dirname(data_path), 'c0_geometry.png')))

  def test_degenerate_cones_are_clean_errors(self):
    with tempfile.TemporaryDirectory() as d:
      rgba = np.dstack([synthetic_rgb(2000, 1250), np.full((1250, 2000), 255, np.uint8)])
      path = os.path.join(d, 'cone.png')
      Image.fromarray(rgba, 'RGBA').save(path)
      # Equal radii: the flattened image would be 0 px tall (division by zero gives Infinity).
      with self.assertRaisesRegex(cli.CliError, 'Invalid crop geometry'):
        self.run_cli(d, path, ['cone', 'top', '1500', '1500', '180', 'y'], 'eq')
      # Outer radius below half the image width: asin(>1) is NaN, there is no height.
      with self.assertRaisesRegex(cli.CliError, 'not a finite number'):
        self.run_cli(d, path, ['cone', 'top', '500', '100', '180', 'y'], 'nan')

  # -- MINOR-9: diff tool text mode -----------------------------------------------------------------
  def test_diff_text_mode_sees_number_notation(self):
    a = '{\n  "x": 1,\n  "y": 1e-7,\n  "created": 5,\n  "updated": 5\n}\n'
    b = '{\n  "x": 1.0,\n  "y": 1e-07,\n  "created": 6,\n  "updated": 9\n}\n'
    self.assertEqual(diff_target_json.compare(json.loads(a), json.loads(b)), [])  # default: equal
    self.assertEqual(diff_target_json.compare_text(a, b),
                     ['text line 2: "x": 1, != "x": 1.0,', 'text line 3: "y": 1e-7, != "y": 1e-07,'])
    self.assertEqual(diff_target_json.compare_text(a, a.replace('5', '7')), [])  # times blanked
    self.assertEqual(len(diff_target_json.compare_text(a, b, strict_time=True)), 4)
    with tempfile.TemporaryDirectory() as d:
      pa, pb = os.path.join(d, 'a.json'), os.path.join(d, 'b.json')
      with open(pa, 'w') as f:
        f.write(a)
      with open(pb, 'w') as f:
        f.write(b)
      self.assertEqual(diff_target_json.main([pa, pb]), 0)
      self.assertEqual(diff_target_json.main([pa, pb, '--text']), 1)
      self.assertEqual(diff_target_json.main([pa, pa, '--text']), 0)


if __name__ == '__main__':
  unittest.main()
