import tempfile
import unittest
from pathlib import Path

import numpy as np

from sentinel.experiments import recent_experiments
from sentinel.pointcloud import clean_points, process_cloud, save_result, terrain_grid


class PointCloudTests(unittest.TestCase):
    def setUp(self):
        x, y = np.meshgrid(np.linspace(0, 2, 21), np.linspace(0, 2, 21))
        self.points = np.column_stack((x.ravel(), y.ravel(), .2 * x.ravel() + .1 * y.ravel()))

    def test_open3d_cleaning_removes_far_outlier(self):
        raw = np.vstack((self.points, [[30., 30., 30.]]))
        cleaned = clean_points(raw, voxel_size_m=.04, neighbors=12, std_ratio=1.0)
        self.assertLess(len(cleaned), len(raw))
        self.assertLess(np.max(cleaned[:, 0]), 3.)

    def test_grid_retains_observed_gaps_and_highest_surface(self):
        points = np.array([[0., 0., 1.], [0.1, 0.1, 2.], [2., 0., 3.]])
        origin, elevation, observed, samples = terrain_grid(points, 1.)
        self.assertEqual(origin, (0., 0.))
        self.assertEqual(elevation[0, 0], 2.)
        self.assertEqual(samples[0, 0], 2)
        self.assertFalse(observed[0, 1])
        self.assertTrue(np.isnan(elevation[0, 1]))

    def test_cli_pipeline_writes_portable_evidence(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "sample.xyz"
            np.savetxt(source, self.points)
            result = process_cloud(source, resolution_m=.25, voxel_size_m=.04)
            report = save_result(result, Path(folder) / "out", len(self.points))
            self.assertGreater(report["observed_cells"], 0)
            saved = np.load(Path(folder) / "out" / "terrain_grid.npz")
            self.assertIn("elevation", saved.files)
            self.assertTrue((Path(folder) / "out" / "report.json").is_file())

    def test_experiment_ledger_stores_input_hash_and_metrics(self):
        with tempfile.TemporaryDirectory() as folder:
            source = Path(folder) / "sample.xyz"
            np.savetxt(source, self.points)
            result = process_cloud(source, resolution_m=.25, voxel_size_m=.04)
            report = save_result(result, Path(folder) / "out", len(self.points))
            from sentinel.experiments import record_experiment
            record_experiment(
                Path(folder) / "experiments.sqlite",
                kind="open3d_pointcloud",
                input_path=source,
                configuration={"resolution_m": .25},
                metrics=report,
                artifacts={"grid": "terrain_grid.npz"},
            )
            history = recent_experiments(Path(folder) / "experiments.sqlite")
            self.assertEqual(len(history), 1)
            self.assertEqual(history[0]["input_name"], "sample.xyz")
            self.assertEqual(history[0]["metrics"]["input_points"], len(self.points))
