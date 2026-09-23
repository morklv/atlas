"""Build and use ATLAS's dependency-free native terrain-cost implementation."""
from __future__ import annotations

import json
from pathlib import Path
import shutil
import subprocess
import tempfile
from typing import Any

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / "native" / "atlas_terrain_cost.cpp"


def compiler() -> str | None:
    """Return the first available C++ compiler."""
    return shutil.which("clang++") or shutil.which("c++") or shutil.which("g++")


def build(output: Path) -> Path:
    """Compile the portable C++ terrain-cost executable."""
    tool = compiler()
    if tool is None:
        raise RuntimeError("No C++ compiler found. Install Xcode Command Line Tools on macOS.")
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    completed = subprocess.run(
        [tool, "-O3", "-std=c++17", str(SOURCE), "-o", str(output)],
        text=True,
        capture_output=True,
    )
    if completed.returncode:
        detail = completed.stderr.strip().splitlines()[-1] if completed.stderr.strip() else "unknown compiler error"
        raise RuntimeError(
            "ATLAS could not build the native terrain tool. "
            "On macOS, repair or install Xcode Command Line Tools with `xcode-select --install`. "
            f"Compiler said: {detail}"
        )
    return output


def can_build() -> bool:
    """Check buildability without leaving an executable in the project."""
    if compiler() is None:
        return False
    with tempfile.TemporaryDirectory() as folder:
        try:
            build(Path(folder) / "atlas_terrain_cost")
        except RuntimeError:
            return False
    return True


def analyze(
    elevation: np.ndarray,
    executable: Path,
    resolution_m: float = 0.5,
    max_step_m: float = 0.25,
    max_slope_deg: float = 20.0,
) -> dict[str, Any]:
    """Run native terrain feasibility code for an elevation grid.

    NaN values remain unknown and are never described as traversable.
    """
    grid = np.asarray(elevation, dtype=float)
    if grid.ndim != 2 or min(grid.shape) == 0:
        raise ValueError("elevation must be a non-empty two-dimensional grid")
    text = "\n".join(" ".join("nan" if not np.isfinite(v) else f"{v:.12g}" for v in row) for row in grid)
    completed = subprocess.run(
        [str(executable), "--resolution", str(resolution_m), "--max-step", str(max_step_m), "--max-slope", str(max_slope_deg)],
        input=text,
        text=True,
        capture_output=True,
        check=True,
    )
    result = json.loads(completed.stdout)
    result["traversable_mask"] = np.array(result["traversable_mask"], dtype=bool).reshape(grid.shape)
    return result
