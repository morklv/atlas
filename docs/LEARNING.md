# Your guided robotics course

The code runs already, but understanding it is the project you and I will work
through. Spend a session on each lesson. Predict results before running changes,
then use observations to explain differences. Keep answers in your own words.

## Lesson 1 — What moves, what senses, what decides?

Run `bash run.sh demo`. Open the flight replay and map report. Identify:

- The simulator's hidden world, which supplies sensor readings.
- The camera image and the robot's incomplete map.
- The planner's choice of a new position.
- A new observation changing the map.

Exercise: point to a dark area in an early observation. Does dark mean empty,
blocked or unobserved? Explain why a missing depth return cannot clear that area.

Checkpoint: explain the complete loop without naming a software library.

## Lesson 2 — Position and direction

Read `Pose`, `Camera.basis` and `enu_to_ned` in `geometry.py`.
Position is where the sensor is; orientation is where it points. Yaw is measured
counterclockwise from east here. PX4's NED convention instead uses north, east,
down and yaw clockwise from north.

Exercise: convert ENU position [3, 8, 7] into NED. Which axis should change sign?
Predict the center camera ray when yaw rotates by 90 degrees.

Checkpoint: explain why swapping x/y accidentally can make a valid command fly
in the wrong direction. The unit tests check both conventions.

## Lesson 3 — A depth pixel becomes a point

Read `Camera.rays` and `DepthFrame.points`. A unit ray is a direction of length
one. Multiplying it by range gives a displacement from the camera.

Exercise: a camera at [0, 0, 7] observes a point along ray [0, 0, -1] at range 7.
Where is that point? Now explain why a corner pixel's ray range differs from its
optical-axis depth even if both are measured in meters.

Run the ground-reconstruction and optical-depth tests. Check why a flat scene
reconstructs points at height zero even after the drone rotates.

Checkpoint: draw the ray and explain the equation from memory.

## Lesson 4 — Occlusion

Read `World.raycast`. A surface behind a wall is not measured because the first
intersection wins. This is geometric visibility, not an AI detection model.

Exercise: inspect `test_occluded_ground_stays_unknown`. Move the wall or the
camera in a copy of that test and predict which location will become visible.

Checkpoint: explain why flying somewhere else can reveal new information even
when the sensor itself has not improved.

## Lesson 5 — Mapping and honest uncertainty

Read `EvidenceMap.update`. The map stores evidence at observed endpoints. It
does not know the world boxes. The mapper uses a flat ground assumption and a
height threshold to distinguish ground from elevated surfaces.

Exercise: compare nominal and noisy runs on the same development seed:

```bash
bash run.sh demo --seed 7 --condition nominal --out output/lesson5-clean
bash run.sh demo --seed 7 --condition noisy --out output/lesson5-noisy
```

Inspect false-free cells, precision and coverage in each `mission.json`.
Explain why 100% accuracy on observed cells does not mean 100% of obstacles were found.

Checkpoint: explain the cost of discarding boundary measurements and why the
current vote rule is a heuristic rather than calibrated uncertainty.

## Lesson 6 — Viewpoint planning

Read `predicted_visibility` and `ViewpointPlanner.choose`. The planner imagines
what a new view might show using the partial map. Unknown obstacles can block
that view unexpectedly.

Exercise: inspect `predicted_new_cells` and `new_cells` in the event log. Find a
step where they differ. Explain at least two causes: unseen occlusion, camera
sampling density, sensor dropout, noisy mapping or the visibility approximation.

Checkpoint: explain the distance penalty and propose a change. Record your
hypothesis before asking me to implement it.

## Lesson 7 — A complete mission

Read `run_mission`. The mission starts in hover, receives a frame, updates its
map, chooses a pose, moves and repeats. It retries a completely missing frame
once if time permits. Persistent failure triggers a return.

Exercise:

```bash
bash run.sh demo --seed 7 --time-budget 30 --out output/lesson7-short
```

Why does it stop earlier? Check whether the return trip is included in elapsed
time. Explain why this implementation cannot fly below unknown tall structures.

Checkpoint: distinguish a time budget from battery state, and a kinematic mover
from a flight controller.

## Lesson 8 — Experiments that can prove you wrong

Compare all three policies on the same seeds. Read the actual CSV, including
scenes where the active policy loses. The preplanned baseline is particularly
important because a poorly chosen sweep can make any adaptive system look good.

Exercise: select two policies with similar coverage. Compare their flight
distance and sensor failures. Is one better on every metric? What would you
choose if missing an obstacle mattered more than collecting more map cells?

Checkpoint: explain why a scene used to adjust the algorithm is no longer an
unseen test, and select fresh seeds for the next final evaluation.

## Lesson 9 — Explain the code, then change it

Pick one small feature to implement together:

- A configurable travel-distance penalty.
- Extra weight for a declared inspection region.
- A pause/resume mission state with a test.
- Sensor pose noise, with explicit limits on interpretation.

You explain the expected behavior, I help write or review code, and you interpret
the test. Increasing feature count is not the goal of this exercise.

## Lesson 10 — The portfolio explanation

Prepare answers to these questions using this repository's actual behavior:

1. What problem does your system solve?
2. What exactly comes from the sensor, and what is assumed known?
3. How is the next viewpoint chosen?
4. Which part did you change or investigate yourself?
5. What is the strongest baseline, and where does your method fail?
6. What would need to change to run on a physical drone?
7. Where did AI help, and how did you check its output?

An honest first description is: "I am developing and evaluating a simulated
active-inspection system, using AI-assisted implementation while learning its
geometry, mapping and planning components." Update it as your understanding and
independent contributions grow.
