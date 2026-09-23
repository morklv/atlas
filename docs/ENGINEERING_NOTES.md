# Engineering notebook — initial implementation

## Environment finding

The working directory contained résumé artifacts but no existing ATLAS repo.
The new project is isolated in `sentinel/`. The Mac reported Apple silicon and
a missing Command Line Tools installation. A bundled Python with NumPy and
Pillow could run the reference implementation without installing system software.

## Why a separate reference simulator exists

It lets us test camera projection, occlusion, mapping and viewpoint selection
immediately. It has an explicit sensor interface and supplies depth, not detections.
This is a local reference model with simplified motion assumptions. Physical-platform integration is recorded rather than silently treated as done.

## First observed mapping issue

Noise near grid-aligned wall boundaries put elevated endpoints into neighboring
free cells. The map reached high coverage but obstacle precision suffered.
The current 2.5cm cell-boundary rejection is a geometric heuristic to reduce that
artifact. It discards evidence and needs retesting with other geometry and noise.
Do not infer that excellent nominal simulation accuracy proves general perception.

## First planner test issue

The fixed sweep constructor assumed every candidate location had an exact
requested yaw. A single-candidate budget test exposed this. The planner now picks
the closest available yaw at that location.

## Baseline correction

The first comparison used only a short lawnmower sweep. The adaptive policy
covered much more area, but the sweep was truncated before reaching every row.
A stronger preplanned empty-ground coverage policy was added. Active and
preplanned coverage are much closer; flight distance is also worth inspecting.

All results remain in the report, including cases where active coverage is lower.
Noise/dropout do not stand in for real lighting, dust, rolling shutter or pose drift.

## Add your own entry after each session

Date / question / prediction / change / result / explanation / next experiment.
Be specific. "Why did an extra view fail to add cells?" is a useful question;
"Make the project better" is too broad to test.
