"""Route-focused survey tasks for the simulated ATLAS swarm.

The planner consumes only a vehicle route and the shared evidence map.  It does
not use hidden world geometry: every task represents a route cell that remains
unknown to the swarm.  Drone assignment is greedy but deterministic, which
makes the policy easy to benchmark before a ROS 2 implementation exists.
"""

from dataclasses import dataclass
from math import hypot
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class InspectionTask:
    """One contiguous uncertain segment of a planned ground route."""

    cells: tuple[tuple[int, int], ...]
    center: tuple[float, float]
    priority: float


@dataclass(frozen=True)
class DroneAssignment:
    drone_id: int
    task: InspectionTask
    travel_m: float


def uncertain_route_tasks(
    observed: np.ndarray,
    route_cells: Iterable[tuple[int, int]],
    resolution_m: float,
) -> list[InspectionTask]:
    """Group consecutive unobserved route cells into high-value survey tasks.

    Route order matters: separated unknown stretches stay separate even if they
    happen to be adjacent elsewhere in the grid.  Priority is the route length
    that cannot yet be supported by evidence, in metres.
    """
    if observed.ndim != 2 or observed.dtype != np.bool_:
        raise ValueError("observed must be a two-dimensional boolean array")
    if not np.isfinite(resolution_m) or resolution_m <= 0:
        raise ValueError("resolution_m must be positive")
    rows, cols = observed.shape
    cells = list(route_cells)
    for x, y in cells:
        if not isinstance(x, int) or not isinstance(y, int) or not (0 <= x < cols and 0 <= y < rows):
            raise ValueError("route cell is outside the evidence map")

    runs: list[list[tuple[int, int]]] = []
    current: list[tuple[int, int]] = []
    for cell in cells:
        x, y = cell
        if observed[y, x]:
            if current:
                runs.append(current)
                current = []
        else:
            current.append(cell)
    if current:
        runs.append(current)

    tasks = []
    for run in runs:
        center = (sum(x for x, _ in run) / len(run) + .5, sum(y for _, y in run) / len(run) + .5)
        tasks.append(InspectionTask(tuple(run), center, len(run) * resolution_m))
    return tasks


def assign_inspections(
    tasks: Iterable[InspectionTask], drone_positions: Iterable[tuple[float, float]], resolution_m: float
) -> list[DroneAssignment]:
    """Assign highest-value route inspections to the closest available drones."""
    if not np.isfinite(resolution_m) or resolution_m <= 0:
        raise ValueError("resolution_m must be positive")
    positions = list(drone_positions)
    if not positions:
        return []
    remaining = list(range(len(positions)))
    assignments = []
    for task in sorted(tasks, key=lambda item: (-item.priority, item.center)):
        if not remaining:
            break
        drone_id = min(remaining, key=lambda i: (hypot(positions[i][0] - task.center[0], positions[i][1] - task.center[1]), i))
        travel_m = hypot(positions[drone_id][0] - task.center[0], positions[drone_id][1] - task.center[1]) * resolution_m
        assignments.append(DroneAssignment(drone_id, task, travel_m))
        remaining.remove(drone_id)
    return assignments
