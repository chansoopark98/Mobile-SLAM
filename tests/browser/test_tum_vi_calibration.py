"""Independent TUM-VI cam0 extrinsic oracle; no engine, GT or YAML dependency."""
import ast
import hashlib
from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[2]
# Published room1/room4 dso/camchain.yaml, SHA256 3a0c0285...:
# Kalibr T_cam_imu maps IMU coordinates to camera coordinates (T_C_I).
PRIMARY_T_C_I = [
    [-0.9995250378696743, 0.029615343885863205, -0.008522328211654736, 0.04727988224914392],
    [0.0075019185074052044, -0.03439736061393144, -0.9993800792498829, -0.047443232143367084],
    [-0.02989013031643309, -0.998969345370175, 0.03415885127385616, -0.0681999605066297],
    [0.0, 0.0, 0.0, 1.0],
]
# Independent numeric golden from the published primary transform, not product config.
GOLDEN_T_I_C_TRANSLATION = [
    0.045574835649698026, -0.07116180183799704, -0.04468125411714437,
]
PRIMARY_SHA = "3a0c0285f4d0a3a1678053b2e5912e3e250e78ac09bd009f6d7d8bba669d5703"


def multiply(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(4)) for j in range(4)] for i in range(4)]


def rigid_inverse(t):
    rotation = [[t[j][i] for j in range(3)] for i in range(3)]
    translation = [-sum(rotation[i][j] * t[j][3] for j in range(3)) for i in range(3)]
    return [rotation[i] + [translation[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def configured_transform(rotation, translation):
    if len(rotation) != 9 or len(translation) != 3:
        raise ValueError("Expected row-major rotation9 / translation3")
    return [rotation[i * 3:i * 3 + 3] + [translation[i]] for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]


def config_array(text, name):
    match = re.search(rf"{name}:\s*!!opencv-matrix\s+.*?data:\s*(\[[^\]]+\])", text, re.S)
    if not match:
        raise ValueError(f"Missing config {name}")
    return ast.literal_eval(match[1])


def browser_array(text, name):
    config = re.search(r"const TUM_VI_CONFIG = \{(.*?)\n\};", text, re.S)
    match = re.search(rf"\b{name}:\s*(\[[^\]]+\])", config[1] if config else "", re.S)
    if not match:
        raise ValueError(f"Missing actual TUM_VI_CONFIG {name}")
    return ast.literal_eval(match[1])


class PrimaryCam0Calibration(unittest.TestCase):
    def assert_identity(self, matrix):
        for i in range(4):
            for j in range(4):
                self.assertAlmostEqual(matrix[i][j], float(i == j), delta=2e-14)

    def assert_matches_primary(self, rotation, translation):
        inverse = rigid_inverse(PRIMARY_T_C_I)
        for actual, expected in zip(translation, GOLDEN_T_I_C_TRANSLATION):
            self.assertAlmostEqual(actual, expected, delta=1e-12)
        for i in range(3):
            for j in range(3):
                self.assertAlmostEqual(rotation[i * 3 + j], inverse[i][j], delta=1e-12)
        applied = configured_transform(rotation, translation)
        self.assert_identity(multiply(PRIMARY_T_C_I, applied))
        self.assert_identity(multiply(applied, PRIMARY_T_C_I))

    def test_primary_inverse_has_independent_translation_and_both_identity_products(self):
        inverse = rigid_inverse(PRIMARY_T_C_I)
        for row, expected in zip(inverse[:3], GOLDEN_T_I_C_TRANSLATION):
            self.assertAlmostEqual(row[3], expected, delta=2e-14)
        self.assert_identity(multiply(PRIMARY_T_C_I, inverse))
        self.assert_identity(multiply(inverse, PRIMARY_T_C_I))

    def test_room1_and_room4_metadata_identify_the_published_cam0(self):
        paths = [ROOT / "assets/datasets/tum/dataset-room1_512_16/dso/camchain.yaml",
                 ROOT / "build/refactor-data/tum/dataset-room4_512_16/dso/camchain.yaml"]
        contents = [p.read_bytes() for p in paths]
        self.assertEqual(contents[0], contents[1])
        for raw in contents:
            self.assertEqual(hashlib.sha256(raw).hexdigest(), PRIMARY_SHA)
            text = raw.decode()
            cam0 = text.split("cam1:", 1)[0]
            rows = re.search(r"T_cam_imu:\n((?:  - \[[^\n]+\]\n){4})", cam0)
            actual = [ast.literal_eval(line.split("- ", 1)[1]) for line in rows[1].splitlines()]
            self.assertEqual(actual, PRIMARY_T_C_I)
            self.assertIn("rostopic: /cam0/image_raw", cam0)
            self.assertIn("rostopic: /cam1/image_raw", text.split("cam1:", 1)[1])

    def test_native_yaml_matches_primary_cam0_inverse(self):
        text = (ROOT / "config/tum_vi_room1.yaml").read_text()
        self.assert_matches_primary(config_array(text, "extrinsicRotation"),
                                    config_array(text, "extrinsicTranslation"))

    def test_actual_browser_constant_matches_primary_cam0_inverse(self):
        text = (ROOT / "web/js/test-tumvi-app.js").read_text()
        self.assert_matches_primary(browser_array(text, "r_ic"), browser_array(text, "t_ic"))


if __name__ == "__main__":
    unittest.main()
