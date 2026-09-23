import json
from pathlib import Path
import tempfile
import unittest

import numpy as np

from sentinel.native_terrain import analyze, build, can_build


@unittest.skipUnless(can_build(), "requires a working C++ compiler")
class NativeTerrainTests(unittest.TestCase):
    def test_unknown_and_steep_cells_are_not_traversable(self):
        with tempfile.TemporaryDirectory() as folder:
            executable = build(Path(folder) / "atlas_terrain_cost")
            elevation = np.array([[0.0, 0.0, np.nan], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]])
            result = analyze(elevation, executable, resolution_m=0.5, max_step_m=0.25, max_slope_deg=20)
        self.assertEqual(result["schema"], "atlas-native-terrain-cost-v1")
        self.assertFalse(result["traversable_mask"][0, 2])
        self.assertFalse(result["traversable_mask"][1, 1])
        self.assertGreater(result["metrics"]["blocked_by_step"], 0)

    def test_result_is_json_serializable_after_mask_conversion(self):
        with tempfile.TemporaryDirectory() as folder:
            executable = build(Path(folder) / "atlas_terrain_cost")
            result = analyze(np.zeros((3, 3)), executable)
        result["traversable_mask"] = result["traversable_mask"].astype(int).tolist()
        json.dumps(result)
