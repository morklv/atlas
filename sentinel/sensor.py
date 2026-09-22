from dataclasses import dataclass
import numpy as np
from .geometry import Camera, Pose


@dataclass(frozen=True)
class SensorCondition:
    noise: float = .01
    pixel_dropout: float = .01
    frame_dropout: float = 0.0


CONDITIONS = {
    "nominal": SensorCondition(),
    "noisy": SensorCondition(.08, .15, 0.0),
    "intermittent": SensorCondition(.03, .05, .2),
}


@dataclass
class DepthFrame:
    ranges: np.ndarray
    pose: Pose
    camera: Camera
    timestamp: float

    def points(self):
        rays = self.camera.rays(self.pose)
        valid = (np.isfinite(self.ranges) & (self.ranges >= self.camera.near)
                 & (self.ranges <= self.camera.far))
        return self.pose.xyz + rays[valid] * self.ranges[valid, None]


class DepthSensor:
    def __init__(self, world, camera, condition, seed=0):
        self._world = world  # Hidden world is accessible only to simulation/evaluation.
        self.camera = camera
        self.condition = condition
        self.rng = np.random.default_rng(seed)

    def capture(self, pose, timestamp=0.0):
        c = self.condition
        if self.rng.random() < c.frame_dropout:
            ranges = np.full((self.camera.height, self.camera.width), np.nan)
        else:
            ranges = self._world.raycast(pose.xyz, self.camera.rays(pose),
                                          self.camera.near, self.camera.far)
            ranges += self.rng.normal(0, c.noise, ranges.shape)
            ranges[self.rng.random(ranges.shape) < c.pixel_dropout] = np.nan
            ranges[(ranges < self.camera.near) | (ranges > self.camera.far)] = np.nan
        return DepthFrame(ranges, pose, self.camera, timestamp)
