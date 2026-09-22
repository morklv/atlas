import json
import tempfile
import unittest
from pathlib import Path
import numpy as np
from sentinel.geometry import Camera, Pose, enu_to_ned, yaw_enu_to_ned_degrees, optical_depth_to_range
from sentinel.world import World, Box
from sentinel.sensor import DepthSensor, DepthFrame, SensorCondition
from sentinel.mapping import EvidenceMap
from sentinel.planning import ViewpointPlanner, candidates
from sentinel.mission import MissionConfig, KinematicDrone, run_mission, load_result


class GeometryTests(unittest.TestCase):
    def test_camera_basis_and_rays(self):
        camera = Camera(width=3, height=3)
        for yaw in (0., .7, np.pi):
            pose = Pose(1, 2, 7, yaw)
            basis = np.array(camera.basis(pose))
            np.testing.assert_allclose(basis @ basis.T, np.eye(3), atol=1e-10)
            rays = camera.rays(pose)
            np.testing.assert_allclose(np.linalg.norm(rays, axis=-1), 1)
            np.testing.assert_allclose(rays[1, 1], basis[0])

    def test_enu_ned(self):
        np.testing.assert_array_equal(enu_to_ned([1, 2, 3]), [2, 1, -3])
        self.assertEqual(yaw_enu_to_ned_degrees(0), 90)
        self.assertAlmostEqual(yaw_enu_to_ned_degrees(np.pi/2), 0)

    def test_optical_depth_is_not_ray_range(self):
        c = Camera(width=3, height=3)
        ranges = optical_depth_to_range(np.full((3, 3), 5.), c)
        self.assertEqual(ranges[1, 1], 5.)
        self.assertGreater(ranges[0, 0], 5.)

    def test_ground_reconstruction_at_rotated_pose(self):
        camera = Camera()
        world = World(24)
        pose = Pose(12, 12, 7, .9)
        frame = DepthSensor(world, camera, SensorCondition(0, 0, 0)).capture(pose)
        points = frame.points()
        self.assertGreater(len(points), 100)
        np.testing.assert_allclose(points[:, 2], 0, atol=1e-9)


class SensorTests(unittest.TestCase):
    def test_occlusion_returns_nearest_surface(self):
        world = World(24, [Box(5, 4, 2, 2, 4)])
        origin = np.array([1., 5., 2.])
        ray = np.array([[1., 0., 0.]])
        self.assertEqual(world.raycast(origin, ray)[0], 4.)

    def test_parallel_ray_outside_box_misses(self):
        world = World(24, [Box(5, 4, 2, 2, 4)])
        self.assertTrue(np.isnan(world.raycast(np.array([1., 9., 2.]), np.array([[1., 0., 0.]]))[0]))

    def test_missing_depth_does_not_clear_map(self):
        belief = EvidenceMap()
        c = Camera()
        frame = DepthFrame(np.full((c.height, c.width), np.nan), Pose(3, 3, 7), c, 0)
        self.assertEqual(belief.update(frame), 0)
        self.assertTrue(np.all(belief.labels == -1))

    def test_occluded_ground_stays_unknown(self):
        world = World(24, [Box(7, 4, 1, 8, 4)])
        camera = Camera(pitch=40)
        frame = DepthSensor(world, camera, SensorCondition(0, 0, 0)).capture(Pose(3, 8, 5, 0))
        belief = EvidenceMap()
        belief.update(frame)
        self.assertFalse(belief.observed[16, 18])  # x=9.25 behind wall


class PlannerTests(unittest.TestCase):
    def test_choose_has_only_belief_inputs(self):
        import inspect
        sig = str(inspect.signature(ViewpointPlanner.choose))
        self.assertNotIn("world", sig)
        self.assertNotIn("truth", sig)
        belief = EvidenceMap()
        home = Pose(3, 3, 7)
        a = ViewpointPlanner(candidates()).choose(belief, Camera(), home, home, 100, 2, 3, 8)
        b = ViewpointPlanner(candidates()).choose(belief, Camera(), home, home, 100, 2, 3, 8)
        self.assertEqual(a, b)

    def test_return_budget_rejects_unreachable_view(self):
        home = Pose(3, 3, 7)
        planner = ViewpointPlanner([Pose(21, 21, 7)])
        self.assertIsNone(planner.choose(EvidenceMap(), Camera(), home, home, 10, 2, 3, 8))

    def test_known_wall_reduces_predicted_visibility(self):
        belief = EvidenceMap()
        pose = Pose(3, 8, 5, 0)
        camera = Camera(pitch=40)
        clear = belief.predicted_visibility(pose, camera)
        belief.hits[8:25, 14:16] = 1
        belief.heights[8:25, 14:16] = 4.
        blocked = belief.predicted_visibility(pose, camera)
        self.assertLess(blocked.sum(), clear.sum())

    def test_preplanned_order_ignores_actual_map(self):
        a, b = EvidenceMap(), EvidenceMap()
        b.hits[5:20, 5:20] = 1
        b.heights[5:20, 5:20] = 4
        home = Pose(3, 3, 7)
        planners = [ViewpointPlanner(candidates(), "preplanned") for _ in range(2)]
        for planner, belief in zip(planners, [a, b]):
            planner.visited.add(0)
            planner.choose(belief, Camera(), home, home, 100, 2, 3, 8)
        self.assertEqual(planners[0].preplanned_order, planners[1].preplanned_order)


class MissionTests(unittest.TestCase):
    def test_collision_checked_between_endpoints(self):
        drone = KinematicDrone(World(24, [Box(5, 4, 2, 2, 4)]), Pose(1, 5, 2), 2)
        with self.assertRaises(RuntimeError):
            drone.move(Pose(10, 5, 2))

    def test_budget_and_reproducibility(self):
        world = World.generated(2)
        config = MissionConfig(max_observations=3, time_budget=35)
        a = run_mission(world, config=config)
        b = run_mission(world, config=config)
        self.assertEqual(a.metadata, b.metadata)
        self.assertLessEqual(a.metadata["metrics"]["elapsed_seconds"], 35)
        self.assertTrue(a.metadata["metrics"]["returned_home"])

    def test_same_initial_observation_for_baselines(self):
        world = World.generated(3)
        config = MissionConfig(max_observations=1)
        a = run_mission(world, "active", config=config)
        b = run_mission(world, "fixed", config=config)
        np.testing.assert_array_equal(a.depths[0], b.depths[0])

    def test_save_and_load(self):
        result = run_mission(World.generated(1), config=MissionConfig(max_observations=1))
        with tempfile.TemporaryDirectory() as directory:
            result.save(directory)
            read = load_result(directory)
            self.assertEqual(result.metadata, read.metadata)
            np.testing.assert_array_equal(result.snapshots[0], read.snapshots[0])

    def test_world_export(self):
        from xml.etree import ElementTree
        world = World.generated(8)
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)
            world.save(p/"scene.json")
            world.export_sdf(p/"scene.sdf")
            self.assertEqual(world.boxes, World.load(p/"scene.json").boxes)
            self.assertEqual(ElementTree.parse(p/"scene.sdf").getroot().tag, "sdf")

    def test_invalid_budget(self):
        with self.assertRaises(ValueError):
            MissionConfig(time_budget=1).validate()

    def test_total_sensor_failure_returns_without_inventing_map(self):
        from unittest.mock import patch
        from sentinel.sensor import CONDITIONS
        with patch.dict(CONDITIONS, {"broken": SensorCondition(0, 0, 1)}):
            result = run_mission(World.generated(9), condition="broken")
        self.assertEqual(result.metadata["stop_reason"], "sensor_failure_return")
        self.assertEqual(result.metadata["metrics"]["coverage"], 0)
        self.assertTrue(result.metadata["metrics"]["returned_home"])

    def test_flight_envelope_violation_rejected(self):
        with self.assertRaises(ValueError):
            run_mission(World(24, [Box(10, 10, 1, 1, 8)]))


if __name__ == "__main__":
    unittest.main()
