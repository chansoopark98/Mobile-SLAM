import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np


spec = importlib.util.spec_from_file_location("export_trajectory", Path(__file__).with_name("export-trajectory.py"))
exporter = importlib.util.module_from_spec(spec)
spec.loader.exec_module(exporter)


class ExportTrajectoryTests(unittest.TestCase):
    def run_export(self, rows):
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / "report.json"
            output = Path(directory) / "camera.tum"
            report.write_text(json.dumps({"wasmVariant": "active", "pages": [{"pathname": "/test-tumvi.html", "replayFrames": rows}]}))
            metadata = exporter.export(report, output)
            data = [line for line in output.read_text().splitlines() if not line.startswith("#")]
            return metadata, data

    def test_known_z_rotation_and_translation(self):
        matrix = [0, -1, 0, 1, 1, 0, 0, 2, 0, 0, 1, 3, 0, 0, 0, 1]
        metadata, data = self.run_export([{"timestamp": 5, "pose": matrix}, {"timestamp": 6, "pose": None}])
        np.testing.assert_allclose([float(value) for value in data[0].split()], [5, 1, 2, 3, 0, 0, np.sqrt(0.5), np.sqrt(0.5)], atol=1e-10)
        self.assertEqual(metadata["estimated_poses"], 1)
        self.assertEqual(metadata["processed_frames"], 2)

    def test_rejects_nonrotation_matrix(self):
        with self.assertRaisesRegex(ValueError, "Invalid rotation"):
            self.run_export([{"timestamp": 1, "pose": [2, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]}])

    def test_rejects_duplicate_timestamps(self):
        identity = [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            self.run_export([{"timestamp": 1, "pose": identity}, {"timestamp": 1, "pose": identity}])


if __name__ == "__main__":
    unittest.main()
