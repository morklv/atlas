"""A complete mission loop using a lightweight kinematic drone backend.

This backend models sensor occlusion and geometric collisions, not motors,
inertia, wind, battery discharge, flight dynamics, or an actual autopilot.
"""
from dataclasses import dataclass, asdict
import json
from pathlib import Path
import numpy as np
from .geometry import Pose, Camera
from .sensor import DepthSensor, CONDITIONS
from .mapping import EvidenceMap
from .planning import ViewpointPlanner, candidates


@dataclass(frozen=True)
class MissionConfig:
    resolution: float = .5
    altitude: float = 7.0
    speed: float = 2.0
    observation_seconds: float = 3.0
    reserve_seconds: float = 8.0
    time_budget: float = 160.0
    max_observations: int = 10
    target_coverage: float = .97

    def validate(self):
        if self.resolution <= 0 or self.altitude <= 0 or self.speed <= 0:
            raise ValueError("Resolution, altitude and speed must be positive")
        if self.max_observations < 1 or self.observation_seconds <= 0:
            raise ValueError("At least one positive-duration observation is required")
        if self.reserve_seconds < 0 or self.time_budget <= self.reserve_seconds + self.observation_seconds:
            raise ValueError("Time budget must allow an observation and the return reserve")
        if not 0 < self.target_coverage <= 1:
            raise ValueError("Target coverage must be in (0, 1]")


class KinematicDrone:
    def __init__(self, world, pose, speed):
        self._world = world
        self.pose = pose
        self.speed = speed
        self.distance = 0.0
        self.elapsed = 0.0
        self.path = [pose.xyz.tolist()]

    def move(self, target):
        length = self.pose.distance(target)
        # Check intermediate positions: endpoints alone can miss a collision.
        for t in np.linspace(0, 1, max(2, int(length/.2) + 1)):
            point = self.pose.xyz * (1-t) + target.xyz * t
            if self._world.collision(point):
                raise RuntimeError("Geometric collision: requested movement is unsafe")
        self.distance += length
        self.elapsed += length / self.speed
        self.pose = target
        self.path.append(target.xyz.tolist())


@dataclass
class MissionResult:
    metadata: dict
    snapshots: list
    depths: list
    heights: list
    truth: np.ndarray

    def save(self, folder):
        folder = Path(folder)
        folder.mkdir(parents=True, exist_ok=True)
        (folder / "mission.json").write_text(json.dumps(self.metadata, indent=2, allow_nan=False))
        np.savez_compressed(folder / "observations.npz",
                            labels=np.stack(self.snapshots), depths=np.stack(self.depths),
                            heights=np.stack(self.heights), truth=self.truth)


def run_mission(world, policy="active", condition="nominal", config=None, camera=None):
    config = config or MissionConfig()
    camera = camera or Camera()
    config.validate()
    # A declared operating envelope, not a scene-derived flight plan.
    # Generated obstacles are <=4m and inspection altitude is 7m by default.
    if any(b.height + .5 >= config.altitude for b in world.boxes):
        raise ValueError("Scene violates the above-obstacle flight envelope")
    if abs(round(world.size/config.resolution) * config.resolution - world.size) > 1e-6:
        raise ValueError("World size must be a multiple of map resolution")
    home = Pose(3., 3., config.altitude, 0.)
    drone = KinematicDrone(world, home, config.speed)
    sensor = DepthSensor(world, camera, CONDITIONS[condition], world.seed + 10000)
    belief = EvidenceMap(world.size, config.resolution)
    planner = ViewpointPlanner(candidates(world.size, config.altitude), policy)
    truth = world.occupancy(config.resolution)  # Only passed to evaluator below.
    snapshots, depths, heights, events = [], [], [], []
    failures = 0
    reason = "observation_limit"
    choice = None
    for step in range(config.max_observations):
        if step:
            choice = planner.choose(belief, camera, drone.pose, home,
                                    config.time_budget-drone.elapsed, config.speed,
                                    config.observation_seconds, config.reserve_seconds)
            if choice is None:
                reason = "no_feasible_useful_viewpoint" if policy == "active" else "sweep_or_budget_complete"
                break
            drone.move(choice.pose)
            planner.visited.add(choice.index)
        else:
            planner.visited.add(0)  # home equals the first candidate
        frame = sensor.capture(drone.pose, drone.elapsed)
        drone.elapsed += config.observation_seconds
        valid_fraction = float(np.isfinite(frame.ranges).mean())
        gained = belief.update(frame)
        if valid_fraction == 0:
            failures += 1
        else:
            failures = 0
        # Retry an empty observation once, only if a safe return still fits.
        retried = False
        if valid_fraction == 0 and (drone.elapsed + config.observation_seconds
                                    + drone.pose.distance(home)/config.speed
                                    + config.reserve_seconds <= config.time_budget):
            frame = sensor.capture(drone.pose, drone.elapsed)
            drone.elapsed += config.observation_seconds
            gained += belief.update(frame)
            valid_fraction = float(np.isfinite(frame.ranges).mean())
            retried = True
            failures = 0 if valid_fraction else failures + 1
        events.append({"step": step, "pose": asdict(drone.pose),
                       "elapsed_seconds": drone.elapsed, "distance_m": drone.distance,
                       "predicted_new_cells": choice.predicted_new_cells if choice else None,
                       "new_cells": gained, "valid_depth_fraction": valid_fraction,
                       "retried": retried, **belief.evaluate(truth)})
        snapshots.append(belief.labels.copy())
        depths.append(frame.ranges.copy())
        heights.append(belief.heights.copy())
        if failures >= 2:
            reason = "sensor_failure_return"
            break
        if belief.observed.mean() >= config.target_coverage:
            reason = "coverage_target"
            break
    drone.move(home)
    if drone.elapsed > config.time_budget + 1e-6:
        raise RuntimeError("Mission exceeded its time budget including return")
    metrics = belief.evaluate(truth)
    metrics.update({"flight_distance_m": drone.distance, "elapsed_seconds": drone.elapsed,
                    "observations": len(events), "returned_home": drone.pose.distance(home) < .01})
    metadata = {"schema_version": 1, "backend": "kinematic_depth_sim",
                "seed": world.seed, "policy": policy, "condition": condition,
                "world_size": world.size, "config": asdict(config), "camera": asdict(camera),
                "stop_reason": reason, "metrics": metrics, "events": events,
                "path_including_return": drone.path,
                "assumptions": ["Known exact pose; no SLAM", "Flat ground and solid boxes",
                                "Constant-speed translation; no flight dynamics",
                                "Instant yaw changes", "Range camera; no RGB semantic model",
                                "Time budget, not a battery model",
                                "Ground truth used only by simulator and evaluator"]}
    return MissionResult(metadata, snapshots, depths, heights, truth)


def load_result(folder):
    folder = Path(folder)
    metadata = json.loads((folder / "mission.json").read_text())
    with np.load(folder / "observations.npz", allow_pickle=False) as data:
        return MissionResult(metadata, list(data["labels"]), list(data["depths"]),
                             list(data["heights"]), data["truth"])
