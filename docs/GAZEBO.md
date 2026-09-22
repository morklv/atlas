# PX4 / Gazebo integration status

**Pending. No live simulator telemetry or camera data has been received in this
checkout yet.** The initial environment had no working Apple developer tools,
Homebrew, CMake or Gazebo. Core code runs through the available Python runtime.

## What is available already

- `bash run.sh export-world --seed 7` writes matching scene JSON and Gazebo SDF.
- `sentinel.integrations.px4_probe` receives position telemetry without arming.
- `sentinel.integrations.gazebo_depth` saves one float32 optical-depth image.
- Coordinate and range-conversion functions have unit tests.

These probes import optional tools only when run. They are not included in the
core's passed integration tests. Their purpose is to establish data access before
connecting a flight mission.

## Environment prerequisite

Apple's Command Line Tools require the installer to be completed on the Mac.
In a normal Terminal session, the initial command is:

```bash
xcode-select --install
```

After completing Apple's installer, verify `xcode-select -p` and `clang --version`.
Do not assume invoking the installer means it completed.

Follow the official PX4 macOS setup for the chosen release:
https://docs.px4.io/main/en/dev_setup/dev_env_mac

Gazebo Harmonic's macOS support is best effort:
https://gazebosim.org/docs/harmonic/install/

The official development setup can build the x500 Gazebo model. Freeze a PX4
commit and its dependencies after successful setup; moving `main` is not a
reproducible environment specification. The project currently has no verified
PX4 commit to claim.

## Smoke-test sequence

1. Launch a stock single-drone PX4/Gazebo scene before the custom world.
2. Verify flight telemetry with the probe in a separate environment containing
   MAVSDK-Python's 3.x API (the v4 package has an API migration).
3. Use `gz topic -l` to find the actual depth topic. Do not guess a model/topic name.
4. Inspect the topic type and capture one frame with the Gazebo probe.
5. Validate focal geometry, metric units, near/far clipping and camera extrinsics
   on a simple plane at a known distance.
6. Synchronize pose with the frame timestamp and convert optical depth to range.
7. Add a simulator-only movement adapter with arrival checks and timeouts.
8. Only then connect the mission runner and replay the same benchmark scene.

Example probes, after their optional dependencies and simulator are ready:

```bash
python -m sentinel.integrations.px4_probe --address udpin://127.0.0.1:14540
python -m sentinel.integrations.gazebo_depth --topic YOUR_VERIFIED_DEPTH_TOPIC
```

The depth probe assumes the local float32 byte format and preserves the raw
optical-depth image in NPZ. It cannot manufacture the missing camera pose.

## Adapter acceptance criteria

- A known plane reconstructs at the right height at several drone orientations.
- ENU/NED conversion matches independent telemetry/scene measurements.
- A requested waypoint is acknowledged and reached within a tolerance.
- Stale camera data is rejected; missing telemetry does not advance the mission.
- Timeouts and return behavior are verified in simulation.
- Sensor observations feed the mapper without importing world obstacle geometry.

Until these checks pass, the published demo must remain labeled as the kinematic
sensor simulator, not PX4/Gazebo flight validation.
