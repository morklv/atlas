# How the pieces fit

The unit of decision is an **inspection viewpoint**, consisting of x, y, z and
yaw. A mission uses one camera with a fixed downward pitch. All coordinates use
meters, radians and ENU axes: east, north and up.

```text
Hidden World -> DepthSensor -> DepthFrame -> EvidenceMap -> ViewpointPlanner
     |                                                        |
     v                                                        v
Evaluator / renderer <- recorded mission <- KinematicDrone <- chosen pose
```

The planner never receives `World`, its boxes or the truth grid. The world is
read by the simulated sensor, geometric movement checks, evaluator and replay
renderer. The map and planner receive only a depth frame, camera calibration,
known pose, site bounds and previously measured map evidence.

## Camera model

A pixel defines a ray through a pinhole camera. The ray is rotated into world
coordinates using yaw and the fixed camera pitch. The sensor intersects that
ray with the nearest box or ground surface.

`world_point = camera_position + unit_world_ray * measured_range`

The sensor emits **ray range**, which differs from optical-axis depth away from
the center pixel. `optical_depth_to_range` provides the conversion for a future
Gazebo adapter. The live probe saves raw optical depth; it does not silently
feed it into the range mapper.

The current camera is colocated with the drone pose. A physical or PX4-mounted
camera will need an explicit mount translation and rotation, including vehicle
roll/pitch and a time-matched pose. None of that is inferred from the latest
unsynchronized telemetry.

## Evidence map

Each cell has counts of elevated and ground endpoints. One frame contributes at
most one vote of each class per cell. This prevents a close or dense region of
an image from overwhelming several other observations just because it has more pixels.

- A valid endpoint over 0.3m above ground votes occupied.
- A ground endpoint votes free.
- A missing range provides no evidence.
- No votes means unknown.
- Ties with occupied votes are treated conservatively as occupied.

Points within 2.5cm of a grid boundary are omitted because tiny depth errors can
otherwise put wall returns into a neighboring ground cell. This has costs: it
discards useful evidence and suits this simplified scene distribution. Record
that tradeoff when testing other resolutions or irregular geometry.

We do not ray-clear the entire XY path to an elevated hit. The airborne ray
passes over ground cells; that does not establish that their ground surface is
free. A ground return is needed to label a footprint cell free.

## Choosing a view

The candidate set is a 4 by 4 lattice, with four yaw directions at each position.
For each candidate, project map-cell centers into the camera. Ray sample against
**known** occupied heights to estimate occlusion. Unknown obstacles are absent
from this predicted visibility calculation, making it optimistic.

`score = predicted_new_cells / (1 + 0.08 * travel_distance_m)`

This is a greedy heuristic. It is not optimal mission planning, a probability
distribution or Shannon entropy. Predicted cells and actually gained cells are
both logged so that optimism can be measured.

Reject a candidate if its movement, observation, direct return and reserve do
not fit the remaining time. The mission flies at a constant altitude above all
allowed obstacles. This is why direct return is valid in this simulation.

## Comparisons

The sweep baseline follows a fixed spatial order. The preplanned baseline runs
the same greedy coverage idea on assumed empty ground before observations.
The active policy updates its choice using sensor-derived obstacles and unknown
areas. Preplanned order is unit-tested to be independent of actual map evidence.

All use the same camera, candidate positions, speed, maximum observations,
random seed and time budget. They may finish at different times or observation
counts. Report those differences; equal maximum budget is not equal work used.

Depth noise is sampled from a seeded generator. Retries advance the generator,
so traces are reproducible but the exact noise values need not remain pixelwise
paired once execution diverges.

## Metrics

Coverage is observed footprint cells divided by all site cells. Observed-cell
accuracy excludes unknown cells. Obstacle recall includes unknown obstacle
cells in its denominator. A map can have high observed accuracy and still miss
many unseen obstacles; that is why both are reported.

Simulator elapsed time includes movement and capture/retry time, not Python
planning/runtime overhead. Planning time must be measured separately before
claiming real-time performance on hardware.
