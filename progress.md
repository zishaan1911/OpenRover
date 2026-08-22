# OpenRover — Progress Log

This file exists so that work can resume correctly even if the chat
history is lost. Whenever we finish a step, this file gets updated.
If you're picking this back up in a new chat, paste this whole file
back to Claude first.

---

## How we're working

- Style: beginner-friendly. Descriptive names, minimal abbreviations,
  go slow, one concept at a time.
- Learning method: Claude writes the surrounding structure (headers,
  tests, CMake), but leaves some function *bodies* as pseudocode
  comments for you to implement yourself. Tests are already written,
  so you can check your own work by running them.
- Environment: you are building/running locally with C++20, CMake,
  ROS 2 Jazzy, and Gazebo Harmonic already (or soon to be) installed.
  Claude cannot run Gazebo/ROS 2 in its own sandbox, so simulator-
  dependent steps are written as instructions for you to run locally,
  not executed by Claude directly. Pure C++ / math / logic code that
  doesn't need Gazebo or ROS 2 *can* be compiled and tested by Claude
  directly, as a sanity check, before handing it to you.
- Repo layout follows the structure defined in the project README
  (see "Repository Structure" section).

---

## Current Phase

**Phase 1 — Simulation** (see README "Development Roadmap")

Overall Phase 1 deliverables (from README):
- [ ] Gazebo world
- [ ] Rover model
- [ ] Differential-drive controller
- [ ] Basic command interface
- [ ] Simulation startup scripts
- [ ] Basic logging
- [ ] Simulation reset functionality

We are starting with the **math underneath the differential-drive
controller**, since it doesn't require Gazebo and is needed by nearly
everything downstream (odometry, control, planning).

---

## Step-by-Step Status

### Step 1: Core differential-drive kinematics (IN PROGRESS)

Goal: implement the math that converts left/right wheel speeds into
the rover's body velocity (v, omega), and integrates pose (x, y,
theta) forward over time.

Files created so far:

| File | Status | Notes |
|---|---|---|
| `CMakeLists.txt` (root) | Done | Sets C++20, enables testing, wires up subdirectories |
| `core/CMakeLists.txt` | Done | Adds `core/common/math` |
| `core/common/math/CMakeLists.txt` | Done | Builds `openrover_math` static library |
| `core/common/math/pose2d.hpp` | Done | `Pose2D` struct + `normalizeAngleRadians()` declaration |
| `core/common/math/pose2d.cpp` | Done | `normalizeAngleRadians()` fully implemented (utility, not a learning exercise) |
| `core/common/math/differential_drive_kinematics.hpp` | Done | Declares `WheelVelocities`, `BodyVelocity`, `DifferentialDriveConfiguration`, and two functions with pseudocode for you to implement |
| `core/common/math/differential_drive_kinematics.cpp` | **Stub — YOU implement this** | Two `TODO` functions: `computeBodyVelocityFromWheelVelocities()` and `integratePoseForward()` |
| `tests/CMakeLists.txt` | Done | Fetches GoogleTest via CMake FetchContent |
| `tests/unit/CMakeLists.txt` | Done | Builds `openrover_unit_tests` executable |
| `tests/unit/test_differential_drive_kinematics.cpp` | Done | 4 tests already written, will fail until you implement the two functions above |

**Your action item right now:**
Open `core/common/math/differential_drive_kinematics.cpp` and implement
the two functions, following the pseudocode comments in the matching
`.hpp` file. Then build and run the tests locally:

```bash
cmake -S . -B build
cmake --build build
ctest --test-dir build --output-on-failure
```

All 4 tests in `test_differential_drive_kinematics.cpp` should pass
once both functions are implemented correctly.

**Come back and tell Claude when tests pass (or if you get stuck)** —
next step will be a `RoverModel` class that wraps this math with
state (current pose + current wheel velocities) and a `setVelocity /
tick(deltaTime)` interface, matching the `IMotorController` idea from
the README's hardware abstraction section.

---

## Decisions Made So Far

- Starting with pure math (no Gazebo/ROS dependency) so it's testable
  in isolation and easy to unit test with GoogleTest.
- Using CMake's `FetchContent` for GoogleTest instead of requiring a
  system install — keeps setup simple.
- Using Euler integration (simplest method) for the first pass at
  `integratePoseForward()`. README explicitly flags that "exact
  integration and numerical stability should be handled in the
  implementation" — we will revisit this with a more accurate
  arc-based integration method as a deliberate follow-up learning
  step, not right away.
- Namespace convention: `openrover::math` for math/common code. We'll
  likely follow `openrover::<module>` for other modules
  (e.g. `openrover::localization`, `openrover::planning`) later.

---

## Not Started Yet (for later reference, from README roadmap)

- Gazebo world + rover model (URDF/SDF)
- RoverModel class wrapping kinematics + state
- Command interface (FORWARD / BACKWARD / ROTATE_LEFT / ROTATE_RIGHT / STOP, then velocity-based API)
- Sensor simulation (LiDAR, IMU, encoders, GPS, camera)
- Localization (odometry, EKF)
- SLAM, planning, local avoidance, vision, missions, dashboard — see README for full roadmap

---

## Open Questions / Things to Revisit

- None yet.
