import unittest
from pathlib import Path
from tempfile import TemporaryDirectory
import numpy as np
from sentinel.swarm import FieldWorld, run_swarm, save_swarm, N, ALTITUDES


class SwarmTests(unittest.TestCase):
    def test_truth_is_not_disclosed_by_unobserved_cells(self):
        result=run_swarm(seed=17,rounds=2)
        last=result["events"][-1]["map"]
        unknown=np.array(last["sources"])==0
        estimated=np.array(last["height"])
        self.assertTrue(unknown.any())
        self.assertTrue(np.all(estimated[unknown]==-1))
        self.assertEqual(last["known"],int((~unknown).sum()))
        self.assertGreater(len(set(e["drone"] for e in result["events"])),2)

    def test_range_returns_and_structure_changes_are_measured(self):
        world=FieldWorld(17)
        self.assertLess(float(world.surface.max()),min(ALTITUDES))
        result=run_swarm(seed=17,rounds=8)
        report=result["report"]
        self.assertGreater(report["coverage_pct"],80)
        self.assertGreater(report["anomaly_cells"],0)
        self.assertGreater(report["damage_precision"],.5)
        volume=np.array(result["voxel"]["labels"],dtype=np.int8)
        self.assertEqual(volume.size,32*N*N)
        self.assertEqual(int((volume==-1).sum()),report["unknown_voxels"])
        self.assertGreater(report["free_voxels"],report["occupied_voxels"])
        self.assertLessEqual(abs(result["events"][-1]["map"]["known"]
                                 -report["coverage_pct"]*N*N/100),5)

    def test_unified_workspace_is_generated_with_embedded_data_and_code(self):
        with TemporaryDirectory() as temp:
            path = save_swarm(run_swarm(seed=17, rounds=2), Path(temp))
            self.assertEqual(path.name, "field.html")
            html = path.read_text()
            self.assertIn("ATLAS / SURVEY + ROUTE", html)
            self.assertIn("simulateSurvey", html)
            self.assertIn("groundCostmap", html)
            self.assertNotIn("__FIELD_DATA__", html)


if __name__=="__main__":
    unittest.main()
