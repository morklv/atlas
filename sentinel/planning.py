"""Map-only viewpoint planning. This module intentionally has no World import."""
from dataclasses import dataclass
import numpy as np
from .geometry import Pose
from .mapping import EvidenceMap


@dataclass(frozen=True)
class Choice:
    index: int
    pose: Pose
    predicted_new_cells: int
    score: float


def candidates(size=24.0, altitude=7.0):
    result = []
    for y in np.linspace(3, size - 3, 4):
        for x in np.linspace(3, size - 3, 4):
            for yaw in (0., np.pi/2, np.pi, -np.pi/2):
                result.append(Pose(float(x), float(y), altitude, float(yaw)))
    return result


class ViewpointPlanner:
    def __init__(self, viewpoints, policy="active", distance_weight=.08):
        if policy not in ("active", "fixed", "preplanned"):
            raise ValueError("Policy must be active, fixed or preplanned")
        self.viewpoints = viewpoints
        self.policy = policy
        self.distance_weight = distance_weight
        self.visited = set()
        self.preplanned_order = None
        # A deterministic lawnmower order, independent of the hidden scene.
        rows = sorted({p.y for p in viewpoints})
        self.fixed_order = []
        for row, y in enumerate(rows):
            xs = sorted({p.x for p in viewpoints if p.y == y}, reverse=bool(row % 2))
            heading = 0. if row % 2 == 0 else np.pi
            for j, x in enumerate(xs):
                # At the far edge, face back into the site.
                yaw = heading if j < len(xs)-1 else (np.pi if heading == 0 else 0.)
                matching = [i for i, p in enumerate(viewpoints) if p.x == x and p.y == y]
                self.fixed_order.append(min(matching, key=lambda i: abs(
                    np.arctan2(np.sin(viewpoints[i].yaw-yaw), np.cos(viewpoints[i].yaw-yaw)))))

    def choose(self, belief, camera, current, home, remaining_seconds,
               speed, observation_seconds, reserve_seconds):
        if self.policy == "preplanned" and self.preplanned_order is None:
            self.preplanned_order = self._plan_open_ground(belief.size, belief.resolution, camera, home)
        order = (range(len(self.viewpoints)) if self.policy == "active"
                 else self.preplanned_order if self.policy == "preplanned" else self.fixed_order)
        best = None
        for i in order:
            if i in self.visited:
                continue
            p = self.viewpoints[i]
            outbound = current.distance(p)
            needed = (outbound + p.distance(home)) / speed + observation_seconds + reserve_seconds
            if needed > remaining_seconds:
                continue
            visible = belief.predicted_visibility(p, camera)
            gain = int((visible & ~belief.observed).sum())
            score = gain / (1 + self.distance_weight * outbound)
            choice = Choice(i, p, gain, score)
            if self.policy != "active":
                return choice
            if best is None or score > best.score:
                best = choice
        if best is not None and best.predicted_new_cells == 0:
            return None
        return best

    def _plan_open_ground(self, size, resolution, camera, home):
        """Stronger fixed baseline: greedy coverage on an assumed empty site.

        The order uses camera geometry and site bounds, but no observed map or
        hidden scene. Replanning after seeing a wall is intentionally disabled.
        """
        assumed = EvidenceMap(size, resolution)
        assumed.free_hits[assumed.predicted_visibility(home, camera)] = 1
        planner = ViewpointPlanner(self.viewpoints, "active", self.distance_weight)
        planner.visited = set(self.visited)
        current, order = home, []
        for _ in range(len(self.viewpoints)):
            choice = planner.choose(assumed, camera, current, home, 1e6, 2, 3, 0)
            if choice is None:
                break
            order.append(choice.index)
            planner.visited.add(choice.index)
            assumed.free_hits[assumed.predicted_visibility(choice.pose, camera)] = 1
            current = choice.pose
        return order
