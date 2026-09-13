# OpenRover — Architecture

This document describes the system as it actually exists, and separately,
the system as the [README](./README.md) plans for it to eventually become.
Anything under **"Built"** has been implemented, compiled/run, and confirmed
working. Anything under **"Planned"** is design intent only — no code exists
for it yet.

For the chronological story of *how* we got here (including debugging
detours and dead ends), see `progress.md`. This document is the opposite:
a snapshot of *what is true right now*, without the history.

---

## 1. System Overview

OpenRover is a simulated autonomous rover project built on:

- **C++20** for all robotics logic (math, control, and — eventually —
  perception/planning algorithms)
- **ROS 2 Jazzy** for inter-process communication (topics, services, nodes)
- **Gazebo Harmonic** (`gz-sim`) for physics simulation
- **CMake** as the build system for pure C++ code, and **colcon/ament_cmake**
  for anything that needs to run as a ROS 2 node

The project deliberately keeps **pure algorithmic code** (math, kinematics,
state machines) separate from **ROS 2/Gazebo-facing code** (nodes, topics,
simulation assets). This is the single most important architectural decision
in the project so far — see §5.

---

## 2. Repository Structure (as it exists today)

```
OpenRover/
├── CMakeLists.txt              Root CMake project — builds core/ and tests/
├── README.md                   Full project vision and roadmap (all phases)
├── architecture.md             This file
├── progress.md                 Chronological build log (local only, not in git)
│
├── core/                       Pure C++20, NO ROS 2 or Gazebo dependency.
│   │                           Buildable/testable with plain cmake+ctest.
│   ├── CMakeLists.txt
│   └── common/
│       ├── math/                          [BUILT]
│       │   ├── pose2d.{hpp,cpp}
│       │   └── differential_drive_kinematics.{hpp,cpp}
│       └── rover_model/                   [BUILT]
│           └── rover_model.{hpp,cpp}
│
├── tests/                      GoogleTest unit tests for core/
│   └── unit/
│       ├── test_differential_drive_kinematics.cpp    [BUILT] 4 tests
│       └── test_rover_model.cpp                      [BUILT] 7 tests
│
├── simulation/                 ROS 2 package `openrover_simulation`.
│   │                           ament_cmake. Symlinked into the colcon
│   │                           workspace as ~/ros2_ws/src/openrover_simulation.
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── worlds/                            [BUILT]
│   │   ├── empty_world.sdf     Ground + light + core plugins, no robot
│   │   └── rover_world.sdf     Same, plus <include> of the rover model
│   ├── models/rover/                      [BUILT]
│   │   ├── model.config
│   │   └── model.sdf           Chassis + 2 driven wheels + caster + DiffDrive plugin
│   ├── config/
│   │   └── ros_gz_bridge.yaml             [BUILT] Topic bridge config
│   └── launch/
│       ├── rover_sim.launch.py            [BUILT] World + bridge
│       └── full_system.launch.py          [BUILT] World + bridge + control node + reset-pose service bridge
│
├── control/                    ROS 2 package `openrover_control`.
│   │                           ament_cmake. Symlinked into the colcon
│   │                           workspace as ~/ros2_ws/src/openrover_control.
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── include/
│   │   └── command_interface_node.hpp     [BUILT]
│   └── src/
│       ├── command_interface_node.cpp     [BUILT]
│       └── main.cpp                       [BUILT]
│
└── scripts/
    └── start_simulation.sh                [BUILT] One-command full-stack startup
```

Two entirely separate build systems coexist on purpose:

| | Built with | Tested with | Depends on ROS 2/Gazebo? |
|---|---|---|---|
| `core/`, `tests/` | plain `cmake` + `cmake --build` | `ctest` (GoogleTest) | No |
| `simulation/`, `control/` | `colcon build` (wraps ament_cmake) | manual `ros2 topic`/`service` calls | Yes |

`core/` and `tests/` are **not** part of the colcon workspace. `simulation/`
and `control/` are each individually symlinked into `~/ros2_ws/src/` — not
the whole repo (colcon treats any folder containing a `CMakeLists.txt` as
one opaque package and won't look inside it for nested packages, which is
exactly what the repo root's own `CMakeLists.txt` would otherwise cause).

---

## 3. Built Components — Detail

### 3.1 `core/common/math` — Differential-drive kinematics

Pure math, no state. Two free functions:

- `computeBodyVelocityFromWheelVelocities(WheelVelocities, DifferentialDriveConfiguration) -> BodyVelocity`
  — converts left/right wheel speeds into linear + angular body velocity.
- `integratePoseForward(Pose2D, BodyVelocity, deltaTimeSeconds) -> Pose2D`
  — advances a pose forward in time using Euler integration (see §6 for the
  known limitation and planned improvement).

Plus `Pose2D` (x, y, theta) and `normalizeAngleRadians()` as supporting types.

**Depended on by:** `core/common/rover_model`.

### 3.2 `core/common/rover_model` — `RoverModel` class

Wraps the math above with actual state: current pose, current commanded
velocity. Public interface:

- `setVelocity(linear, angular)` / `setWheelVelocities(WheelVelocities)`
- `stop()`
- `tick(deltaTimeSeconds)` — advances `currentPose_` using
  `integratePoseForward`
- `getCurrentPose()`, `getCurrentCommandedVelocity()`
- `resetToPose(Pose2D)`

This class is **simulator-independent** — it doesn't know Gazebo or ROS 2
exist. It's a pure C++ model of rover motion, currently used only in unit
tests. It is **not yet wired into any ROS 2 node** (see §4, Planned).

### 3.3 `simulation/` — Gazebo world, rover model, ROS 2 bridge

- **`rover_world.sdf`**: ground plane, directional light, core `gz-sim`
  plugins (Physics, UserCommands, SceneBroadcaster, Sensors), and an
  `<include>` of the rover model (resolved via `GZ_SIM_RESOURCE_PATH`,
  set by the launch file).
- **`models/rover/model.sdf`**: chassis (box) + two driven wheels
  (cylinders, revolute joints) + one caster wheel (sphere, ball joint).
  Carries Gazebo's `DiffDrive` system plugin (subscribes to Gazebo
  Transport topic `/cmd_vel`, publishes `/model/rover/odometry` and
  `/model/rover/tf`) and `JointStatePublisher` (publishes
  `/world/openrover_world/model/rover/joint_state`).
- **`config/ros_gz_bridge.yaml`**: maps 5 Gazebo Transport topics to real
  ROS 2 topics via `ros_gz_bridge`'s `parameter_bridge`:

  | ROS 2 topic | Gazebo topic | Direction |
  |---|---|---|
  | `/cmd_vel` | `/cmd_vel` | ROS → Gazebo |
  | `/odom` | `/model/rover/odometry` | Gazebo → ROS |
  | `/tf` | `/model/rover/tf` | Gazebo → ROS |
  | `/joint_states` | `/world/openrover_world/model/rover/joint_state` | Gazebo → ROS |
  | `/clock` | `/clock` | Gazebo → ROS |

- A **second**, separately-configured bridge (command-line-argument style,
  not YAML — services use a different bridging mechanism than topics)
  exposes Gazebo's `/world/openrover_world/set_pose` service as a ROS 2
  service (`ros_gz_interfaces/srv/SetEntityPose`), used for reset (§3.4).
- **`launch/full_system.launch.py`**: the single entry point — starts
  Gazebo + world + both bridges + the command interface node together.
  Wrapped by `scripts/start_simulation.sh` for one-command startup.

### 3.4 `control/` — `CommandInterfaceNode`

A single `rclcpp::Node`. Two ROS 2 interfaces:

- **Subscribes** to `/rover_command` (`std_msgs/String`): accepts
  `FORWARD`, `BACKWARD`, `ROTATE_LEFT`, `ROTATE_RIGHT`, `STOP`. Each maps to
  a fixed linear/angular velocity, published via `setVelocity()` →
  `geometry_msgs/Twist` on `/cmd_vel`.
- **Serves** `/reset_rover` (`std_srvs/srv/Trigger`): internally acts as a
  *client* of Gazebo's bridged `set_pose` service, teleporting the rover
  back to `(0, 0, 0.1)` with identity orientation, then calls
  `setVelocity(0, 0)`. Uses `async_send_request` with a callback — **never**
  blocks waiting for the result inside the service callback (would deadlock
  a single-threaded executor).

This node does **not** currently use `core/common/rover_model::RoverModel`
at all — it publishes `geometry_msgs/Twist` directly. `RoverModel` and this
node are two independent, not-yet-connected implementations of "turn a
velocity command into motion" — one simulated in pure C++ (unit-tested,
no Gazebo), one real-world-facing (drives the actual Gazebo simulation).
Connecting them is a natural, currently-unplanned next step (§4).

---

## 4. Planned Components (design intent only — not implemented)

Everything below is copied/summarized from the README's roadmap. **None of
this exists yet.** It's listed here so future architectural decisions (e.g.
where a new node should live, what it should depend on) can be made
consistently with the rest of the system, not because any of it has been
started.

### Phase 2 — Sensor Simulation (next up)
- LiDAR, IMU, camera, wheel encoders, GPS — as Gazebo sensor plugins on the
  rover model, bridged to ROS 2 topics the same way `/odom` is now.
- Likely new package: `sensors/` (ament_cmake, following the `simulation/`
  pattern), or sensor SDF added directly into `models/rover/model.sdf`.

### Phase 3+ (from README, not yet detailed here)
- Localization (odometry fusion, EKF)
- SLAM
- Path planning, local obstacle avoidance
- Computer vision
- Mission execution
- Web/desktop dashboard

### Near-term, smaller planned items
- **Connect `RoverModel` (§3.2) to a real ROS 2 node.** Currently
  `CommandInterfaceNode` publishes `Twist` directly; `RoverModel` sits
  unused outside tests. A natural next step is a node that owns a
  `RoverModel`, calls `tick()` on a timer, and publishes/subscribes
  accordingly — giving the pure C++ layer an actual role in the running
  system, not just tests.
- **More accurate pose integration.** `integratePoseForward()` currently
  uses simple Euler integration, which is a known, documented
  approximation (see README quote in §6). An arc-based/exact integration
  method is planned as a deliberate follow-up, not urgent.

---

## 5. Key Architectural Decisions

**Pure math/state, kept separate from ROS 2/Gazebo.**
`core/` has zero ROS 2 or Gazebo dependency, by design. It's built with
plain CMake and tested with GoogleTest, independent of whether ROS 2 or
Gazebo are even installed. This is why Step 1/2 of the project could be
fully compiled and tested inside Claude's own sandbox (no ROS 2 available
there), while everything in `simulation/`/`control/` could only ever be
verified on the actual development machine.

**Two build systems, on purpose, not by accident.**
`core`/`tests` use plain CMake; `simulation`/`control` use colcon/
ament_cmake. This isn't a migration in progress — it's permanent. Only code
that genuinely needs to be a ROS 2 node (topics, services, `rclcpp`) pays
the colcon/ament tax.

**Individual packages symlinked into the workspace, not the repo root.**
Because colcon claims an entire directory tree as one package the moment it
finds a `CMakeLists.txt` at that directory's root, and does not recurse
into it looking for more packages, the repo root itself can never be
symlinked into `~/ros2_ws/src/` — only `simulation/` and `control/`
individually. This was discovered the hard way (see `progress.md`, Step 3)
and is now a fixed convention for any future ROS 2 package added to this
repo.

**Discrete commands over `/rover_command`, real velocity over `/cmd_vel`.**
`/cmd_vel` (a `Twist`) is the actual control surface Gazebo listens to.
`/rover_command` (a `String`) is a thin, human-friendly layer on top,
matching the README's explicit design: "discrete commands for now, a
velocity-based API underneath for later." Any future planner/autonomy code
should talk directly to `/cmd_vel` (or eventually, directly to
`CommandInterfaceNode::setVelocity()`), not to `/rover_command`.

---

## 6. Known Limitations (intentional, documented, not bugs)

- **Euler integration** in `integratePoseForward()`: assumes the rover
  travels in a straight line at its *starting* heading for the whole time
  step, then snaps to the new heading. Fine for small time steps / slow
  turning; drifts more otherwise. The README explicitly calls this out as
  something to revisit, not an oversight.
- **`RoverModel` and `CommandInterfaceNode` are disconnected.** See §4.
- **Fixed speeds for discrete commands** (`forwardSpeedMetersPerSecond_ =
  0.5`, `rotationSpeedRadiansPerSecond_ = 0.5` in `CommandInterfaceNode`)
  are hardcoded, not configurable via ROS 2 parameters yet.
