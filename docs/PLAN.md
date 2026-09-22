# Fourteen-day build and learning plan

The initial codebase already implements a runnable geometric simulation. The
remaining two-week effort is integration, understanding, controlled changes and
evidence. It is not a promise that all the original flight-stack integration is done.

| Days | Engineering work | Your learning checkpoint |
|---|---|---|
| 1–2 | Run the current demo/tests; install and validate PX4/Gazebo tools if available | Explain sensor -> map -> planner -> movement |
| 3–4 | Verify camera pose, range conventions and scene geometry; examine mapper errors | Reconstruct a point from a depth pixel |
| 5–6 | Work through mapping; introduce one deliberate perturbation and explain it | Distinguish unknown from free and measured from assumed |
| 7–8 | Study/adapt viewpoint scoring; test against preplanned coverage | Explain predicted versus actual gain |
| 9–10 | Strengthen mission failure handling; validate live simulator adapter if tools work | Explain return feasibility and sensor failure behavior |
| 11–12 | Freeze a version and run new unseen seeds; retain every result | Explain comparisons, limitations and counterexamples |
| 13–14 | Record your own narration, write results and prepare interview walkthrough | Demonstrate and modify a core component confidently |

## Already implemented in this checkout

- [x] Single drone with geometric depth observations and occlusion.
- [x] Camera and coordinate math, map updates, three viewpoint policies.
- [x] Sensor noise/dropout, retries, mission completion and return-time accounting.
- [x] Saved observations, event logs, reproducible scenarios and evaluation reports.
- [x] Animated isometric replay and an observed-map/depth report.
- [x] Automated geometry, mapping, planner, mission and failure checks.
- [x] Learning guide and Gazebo world export.

## Still pending

- [ ] Xcode developer tools/Homebrew/Gazebo/PX4 installed and smoke-tested.
- [ ] Live camera mount, intrinsics and timestamped pose validated.
- [ ] Live PX4 movement interface connected to the mission runner.
- [ ] An end-to-end mission actually run under PX4/Gazebo.
- [ ] Mark's explanations, modifications, fresh experiments and narrated demo.

The current backend is a useful executable reference for those tasks. It does
not substitute for claiming the flight integration has been tested.
