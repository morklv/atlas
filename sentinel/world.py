"""The simulation's hidden world. Mapping and planning never receive World."""
from dataclasses import dataclass, asdict
import json
from pathlib import Path
import numpy as np


@dataclass(frozen=True)
class Box:
    x: float
    y: float
    width: float
    depth: float
    height: float

    @property
    def low(self):
        return np.array([self.x, self.y, 0.0])

    @property
    def high(self):
        return np.array([self.x + self.width, self.y + self.depth, self.height])


class World:
    def __init__(self, size=24.0, boxes=(), seed=0):
        self.size = float(size)
        self.boxes = list(boxes)
        self.seed = seed

    @classmethod
    def generated(cls, seed=7, size=24.0):
        rng = np.random.default_rng(seed)
        # Grid-aligned footprints allow unambiguous reference cell labels.
        boxes = []
        for _ in range(12):
            x, y = rng.integers(4, int(2 * size) - 10, size=2) / 2
            w, d = rng.integers(2, 8, size=2) / 2
            h = float(rng.uniform(.8, 4.0))
            boxes.append(Box(float(x), float(y), float(w), float(d), h))
        return cls(size, boxes, seed)

    def raycast(self, origin, rays, near=.15, far=35.):
        """Nearest intersection with finite ground and axis-aligned solid boxes.

        Vectorized slab intersection, with explicit handling of parallel rays.
        NaN means no return; it does NOT mean free space.
        """
        shape = rays.shape[:-1]
        directions = rays.reshape(-1, 3)
        origin = np.asarray(origin, dtype=float)
        best = np.full(len(directions), np.inf)
        down = directions[:, 2] < -1e-9
        ground_t = np.full(len(directions), np.inf)
        ground_t[down] = -origin[2] / directions[down, 2]
        with np.errstate(invalid="ignore"):
            ground_xy = origin[:2] + ground_t[:, None] * directions[:, :2]
        valid = down & np.all((ground_xy >= 0) & (ground_xy < self.size), axis=1)
        best[valid] = ground_t[valid]
        for box in self.boxes:
            entry = np.full(len(directions), -np.inf)
            leave = np.full(len(directions), np.inf)
            possible = np.ones(len(directions), dtype=bool)
            for axis in range(3):
                parallel = np.abs(directions[:, axis]) < 1e-10
                possible &= ~(parallel & ((origin[axis] < box.low[axis])
                                           | (origin[axis] > box.high[axis])))
                a = np.full(len(directions), -np.inf)
                b = np.full(len(directions), np.inf)
                np.divide(box.low[axis] - origin[axis], directions[:, axis],
                          out=a, where=~parallel)
                np.divide(box.high[axis] - origin[axis], directions[:, axis],
                          out=b, where=~parallel)
                entry = np.maximum(entry, np.minimum(a, b))
                leave = np.minimum(leave, np.maximum(a, b))
            hit = np.where(entry >= near, entry, leave)
            valid = possible & (leave >= np.maximum(entry, near)) & (hit >= near)
            best = np.where(valid, np.minimum(best, hit), best)
        best[(best < near) | (best > far)] = np.nan
        return best.reshape(shape)

    def occupancy(self, resolution):
        """Evaluation-only footprint labels."""
        n = int(round(self.size / resolution))
        x, y = np.meshgrid((np.arange(n) + .5) * resolution,
                           (np.arange(n) + .5) * resolution)
        result = np.zeros((n, n), dtype=bool)
        for b in self.boxes:
            result |= (x >= b.x) & (x < b.x + b.width) & (y >= b.y) & (y < b.y + b.depth)
        return result

    def collision(self, point, radius=.25):
        point = np.asarray(point)
        return any(np.all(point >= b.low - radius) and np.all(point <= b.high + radius)
                   for b in self.boxes)

    def save(self, path):
        Path(path).write_text(json.dumps({"size": self.size, "seed": self.seed,
                                         "boxes": [asdict(b) for b in self.boxes]}, indent=2))

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text())
        return cls(data["size"], [Box(**b) for b in data["boxes"]], data.get("seed", 0))
