# Swarm field survey: measured results

The experiment runs three kinematic aerial observers over a 48 × 48 m synthetic heightfield with tree canopies, terrain relief, and two roofs with structural changes. Each observer receives noisy ray-marched range images. The team fuses measured endpoints into one shared height surface. A pre-incident roof height map supports geometric change flags. The reference surface and damage labels are used only by the simulator and evaluator.

Run `bash run.sh swarm-benchmark` to reproduce the paired comparison. Seeds 17–26 use the same structure layout but vary tree placement and range noise. Eight survey rounds are allowed in each run. Results in `output/swarm-benchmark.json`:

| Allocation | Observed surface | Mean surface height error on observed cells | Mean total flight distance | Roof-change precision | Recall among observed changed roof cells |
|---|---:|---:|---:|---:|---:|
| Coordinated reservations | 95.80% | 0.192 m | 178.84 m | 0.741 | 0.983 |
| Independent choices | 95.26% | 0.192 m | 204.18 m | 0.708 | 0.987 |

In these ten paired synthetic cases, reservations saved about 25 m of combined travel and improved surface coverage by 0.54 percentage points. They did **not** reduce the count of cells measured by multiple drones; that overlap metric was 4,987 versus 4,726. The reservation rule reduces projected duplicate *unknown-cell choices* at assignment time, which is not the same as reducing actual observation overlap after measurements.

The single replay seed 17 finished at 95.6% coverage, 0.19 m mean height error, and 0.692 roof-change precision. The change detector has substantial false positives and should be treated as a review queue, not a validated damage diagnosis.

The project also exports a ray-carved 0.5 m 3D grid with unknown, free, and occupied voxels. The displayed terrain is the simpler 2.5D height surface; the 3D grid has no semantics and cannot fill surfaces hidden behind obstacles. No live LiDAR, radar, camera imagery, ML model, SLAM, autopilot, communication latency, weather, or battery/return model is implemented. The field does not generalize to real deployments from this experiment alone.
