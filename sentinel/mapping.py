"""Endpoint evidence map for a flat-ground scene of solid boxes.

Observed elevated surfaces imply occupied footprint cells. Ground returns imply
free cells. Missing returns imply nothing. This is not SLAM or general 3D mapping.
"""
import numpy as np


class EvidenceMap:
    def __init__(self, size=24., resolution=.5, obstacle_threshold=.3, boundary_margin=.025):
        self.size, self.resolution = size, resolution
        self.threshold = obstacle_threshold
        self.boundary_margin = min(boundary_margin, resolution * .2)
        n = int(round(size / resolution))
        self.hits = np.zeros((n, n), dtype=np.int32)
        self.free_hits = np.zeros((n, n), dtype=np.int32)
        self.heights = np.zeros((n, n), dtype=float)

    @property
    def observed(self):
        return (self.hits + self.free_hits) > 0

    @property
    def occupied(self):
        # Require >50% elevated endpoint votes, with a conservative tie break.
        return (self.hits > 0) & (self.hits >= self.free_hits)

    @property
    def labels(self):
        return np.where(~self.observed, -1, self.occupied.astype(int))

    def centers(self):
        n = self.hits.shape[0]
        x, y = np.meshgrid((np.arange(n) + .5) * self.resolution,
                           (np.arange(n) + .5) * self.resolution)
        return np.column_stack([x.ravel(), y.ravel(), np.zeros(n*n)])

    def update(self, frame):
        points = frame.points()
        # Height and extent bounds reject invalid/noisy points, not unseen space.
        valid = (np.all(points[:, :2] >= 0, axis=1)
                 & np.all(points[:, :2] < self.size, axis=1)
                 & (points[:, 2] > -.3))
        points = points[valid]
        # A tiny depth error at a cell boundary can put an elevated endpoint in
        # the adjacent free cell. Leave these ambiguous endpoints unclassified.
        # This is a geometric heuristic, not a calibrated uncertainty model.
        within_cell = np.mod(points[:, :2], self.resolution)
        interior = np.all((within_cell > self.boundary_margin)
                          & (within_cell < self.resolution-self.boundary_margin), axis=1)
        points = points[interior]
        if len(points) == 0:
            return 0
        cells = np.floor(points[:, :2] / self.resolution).astype(int)
        x, y = cells[:, 0], cells[:, 1]
        elevated = points[:, 2] > self.threshold
        before = int(self.observed.sum())
        # At most one vote per class per cell per frame avoids pixel density bias.
        for mask, array in ((elevated, self.hits), (~elevated, self.free_hits)):
            unique = np.unique(cells[mask], axis=0)
            if len(unique):
                array[unique[:, 1], unique[:, 0]] += 1
        np.maximum.at(self.heights, (y, x), np.maximum(points[:, 2], 0))
        return int(self.observed.sum()) - before

    def predicted_visibility(self, pose, camera):
        """Optimistic visibility of cell centers using ONLY currently known heights.

        Unknown obstacles may invalidate this prediction. That is precisely why
        actual measured gain is logged separately from predicted gain.
        """
        points = self.centers()
        visible = camera.in_view(pose, points)
        ids = np.flatnonzero(visible)
        if len(ids) == 0:
            return visible.reshape(self.hits.shape)
        end = points[ids]
        # Sample rays through the known height map. Ignore unknown cells.
        t = np.linspace(.05, .96, 24)
        samples = pose.xyz + (end[:, None, :] - pose.xyz) * t[None, :, None]
        cells = np.floor(samples[..., :2] / self.resolution).astype(int)
        valid = np.all((cells >= 0) & (cells < self.hits.shape[0]), axis=-1)
        cells = np.clip(cells, 0, self.hits.shape[0] - 1)
        x, y = cells[..., 0], cells[..., 1]
        blocked = valid & self.occupied[y, x] & (self.heights[y, x] >= samples[..., 2])
        visible[ids[np.any(blocked, axis=1)]] = False
        return visible.reshape(self.hits.shape)

    def evaluate(self, truth):
        """Called by evaluator AFTER planning; truth never enters update/visibility."""
        observed, predicted = self.observed, self.occupied
        tp = int((predicted & truth & observed).sum())
        fp = int((predicted & ~truth & observed).sum())
        fn_observed = int((~predicted & truth & observed).sum())
        return {
            "coverage": float(observed.mean()),
            "accuracy_observed": float((predicted[observed] == truth[observed]).mean()) if observed.any() else None,
            "obstacle_recall_all": tp / int(truth.sum()) if truth.any() else None,
            "obstacle_precision": tp / (tp + fp) if tp + fp else None,
            "false_free_cells": fn_observed,
            "unknown_obstacle_cells": int((truth & ~observed).sum()),
            "observed_cells": int(observed.sum()),
            "total_cells": int(observed.size),
        }
