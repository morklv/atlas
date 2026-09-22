"""Metric ENU coordinates: x east, y north, z up; yaw counterclockwise from east.

Camera rays use Euclidean range, NOT optical-axis depth. Keep this distinction
when integrating a real sensor or Gazebo: z-depth must be converted first.
"""
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float = 0.0

    @property
    def xyz(self):
        return np.array([self.x, self.y, self.z], dtype=float)

    def distance(self, other):
        return float(np.linalg.norm(self.xyz - other.xyz))


@dataclass(frozen=True)
class Camera:
    width: int = 64
    height: int = 48
    hfov: float = 74.0
    vfov: float = 58.0
    pitch: float = 60.0  # degrees below horizontal; fixed mount
    near: float = 0.15
    far: float = 35.0

    def basis(self, pose):
        p = np.deg2rad(self.pitch)
        c, s = np.cos(pose.yaw), np.sin(pose.yaw)
        forward = np.array([c * np.cos(p), s * np.cos(p), -np.sin(p)])
        right = np.array([s, -c, 0.0])
        up = np.cross(right, forward)
        return forward, right, up

    def rays(self, pose):
        # Pixel centers, top row first. Unit rays expressed in world coordinates.
        u = (2 * (np.arange(self.width) + .5) / self.width - 1)
        v = (1 - 2 * (np.arange(self.height) + .5) / self.height)
        u, v = np.meshgrid(u, v)
        f, r, up = self.basis(pose)
        rays = (f + u[..., None] * np.tan(np.deg2rad(self.hfov / 2)) * r
                + v[..., None] * np.tan(np.deg2rad(self.vfov / 2)) * up)
        return rays / np.linalg.norm(rays, axis=-1, keepdims=True)

    def in_view(self, pose, points):
        delta = points - pose.xyz
        f, r, up = self.basis(pose)
        z = delta @ f
        return ((z > self.near)
                & (np.linalg.norm(delta, axis=1) < self.far)
                & (np.abs(delta @ r) <= z * np.tan(np.deg2rad(self.hfov / 2)))
                & (np.abs(delta @ up) <= z * np.tan(np.deg2rad(self.vfov / 2))))


def enu_to_ned(xyz):
    """PX4 uses north/east/down. No arbitrary origin shift is performed."""
    x, y, z = xyz
    return np.array([y, x, -z], dtype=float)


def yaw_enu_to_ned_degrees(yaw):
    return float((90 - np.rad2deg(yaw)) % 360)


def optical_depth_to_range(depth, camera):
    """Convert pinhole z-depth into ray range for the mapper's sensor contract."""
    u = (2 * (np.arange(camera.width) + .5) / camera.width - 1)
    v = (1 - 2 * (np.arange(camera.height) + .5) / camera.height)
    u, v = np.meshgrid(u, v)
    return depth * np.sqrt(1 + (u * np.tan(np.deg2rad(camera.hfov / 2)))**2
                           + (v * np.tan(np.deg2rad(camera.vfov / 2)))**2)
