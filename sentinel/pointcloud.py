"""Mac-native Open3D processing for measured terrain point clouds.

The pipeline accepts an existing PLY, XYZ, or CSV cloud.  It cleans duplicate
and statistical outlier points, creates a highest-surface terrain grid, and
keeps every unmeasured cell unknown.  It never fabricates a surface between
gaps in the input cloud.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class TerrainCloudResult:
    """Cleaned points and a route-ready measured terrain representation."""

    points: np.ndarray
    origin_xy: tuple[float, float]
    resolution_m: float
    elevation: np.ndarray
    observed: np.ndarray
    samples: np.ndarray

    def report(self, input_points: int) -> dict:
        return {
            "schema": "atlas-open3d-terrain-v1",
            "input_points": int(input_points),
            "clean_points": int(len(self.points)),
            "grid_shape": list(self.elevation.shape),
            "resolution_m": self.resolution_m,
            "origin_xy_m": list(self.origin_xy),
            "observed_cells": int(self.observed.sum()),
            "unknown_cells": int((~self.observed).sum()),
        }


def _open3d():
    try:
        import open3d as o3d
    except ImportError as exc:  # pragma: no cover - dependency error path
        raise RuntimeError("Open3D is required. Install it with: python -m pip install -e '.[pointcloud]'") from exc
    return o3d


def load_points(path: str | Path) -> np.ndarray:
    """Read ASCII XYZ/CSV or PLY points as an N by 3 float array."""
    path = Path(path)
    if not path.is_file():
        raise FileNotFoundError(path)
    if path.suffix.lower() == ".ply":
        cloud = _open3d().io.read_point_cloud(str(path))
        points = np.asarray(cloud.points, dtype=float)
    elif path.suffix.lower() in {".xyz", ".csv", ".txt"}:
        delimiter = "," if path.suffix.lower() == ".csv" else None
        try:
            points = np.loadtxt(path, delimiter=delimiter, usecols=(0, 1, 2), dtype=float)
        except ValueError:
            # A common CSV export includes one header row.
            points = np.loadtxt(path, delimiter=delimiter, usecols=(0, 1, 2), dtype=float, skiprows=1)
    else:
        raise ValueError("Use an ASCII .ply, .xyz, .csv, or .txt point cloud.")
    points = np.atleast_2d(points)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("Point cloud must contain at least three XYZ points.")
    if not np.isfinite(points).all():
        raise ValueError("Point cloud contains non-finite coordinates.")
    return points


def clean_points(points: np.ndarray, voxel_size_m: float = 0.15, neighbors: int = 16, std_ratio: float = 2.0) -> np.ndarray:
    """Voxel-downsample and remove statistical outliers with Open3D."""
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("points must be an N by 3 array with at least three rows")
    if not np.isfinite(voxel_size_m) or voxel_size_m <= 0:
        raise ValueError("voxel_size_m must be positive")
    if neighbors < 2 or not np.isfinite(std_ratio) or std_ratio <= 0:
        raise ValueError("neighbors and std_ratio must be positive")
    o3d = _open3d()
    cloud = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(points))
    cloud = cloud.voxel_down_sample(voxel_size_m)
    if len(cloud.points) >= neighbors + 1:
        cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=neighbors, std_ratio=std_ratio)
    cleaned = np.asarray(cloud.points, dtype=float)
    if len(cleaned) < 3:
        raise ValueError("Cleaning removed too many points; use a smaller voxel size or inspect the input cloud.")
    return cleaned


def terrain_grid(points: np.ndarray, resolution_m: float = 0.5) -> tuple[tuple[float, float], np.ndarray, np.ndarray, np.ndarray]:
    """Rasterize the highest measured surface in each cell.

    The max height avoids planning through an observed elevated obstacle.  Empty
    cells remain NaN and unobserved instead of being interpolated.
    """
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or len(points) < 3:
        raise ValueError("points must be an N by 3 array with at least three rows")
    if not np.isfinite(resolution_m) or resolution_m <= 0:
        raise ValueError("resolution_m must be positive")
    min_x, min_y = np.min(points[:, :2], axis=0)
    ix = np.floor((points[:, 0] - min_x) / resolution_m + 1e-9).astype(int)
    iy = np.floor((points[:, 1] - min_y) / resolution_m + 1e-9).astype(int)
    shape = (int(iy.max()) + 1, int(ix.max()) + 1)
    # Accumulate with negative infinity first; NaN would poison np.maximum.at.
    elevation = np.full(shape, -np.inf, dtype=float)
    samples = np.zeros(shape, dtype=np.uint32)
    np.maximum.at(elevation, (iy, ix), points[:, 2])
    np.add.at(samples, (iy, ix), 1)
    observed = samples > 0
    elevation[~observed] = np.nan
    return (float(min_x), float(min_y)), elevation, observed, samples


def process_cloud(
    path: str | Path,
    resolution_m: float = 0.5,
    voxel_size_m: float = 0.15,
    neighbors: int = 16,
    std_ratio: float = 2.0,
) -> TerrainCloudResult:
    """Load, clean, and rasterize a measured cloud into ATLAS terrain evidence."""
    raw = load_points(path)
    cleaned = clean_points(raw, voxel_size_m, neighbors, std_ratio)
    origin, elevation, observed, samples = terrain_grid(cleaned, resolution_m)
    return TerrainCloudResult(cleaned, origin, resolution_m, elevation, observed, samples)


def save_result(result: TerrainCloudResult, output_dir: str | Path, input_points: int) -> dict:
    """Write portable NumPy terrain evidence and a concise JSON report."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        output_dir / "terrain_grid.npz",
        points=result.points,
        origin_xy=np.asarray(result.origin_xy),
        resolution_m=np.asarray(result.resolution_m),
        elevation=result.elevation,
        observed=result.observed,
        samples=result.samples,
    )
    report = result.report(input_points)
    (output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    return report
