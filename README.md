# OpenRover

> **A simulation-first autonomous mobile robotics platform built for research, experimentation, and software engineering.**

OpenRover is a modular autonomous rover platform designed to be developed and evaluated **without requiring physical robotics hardware**.

The project begins with a fully simulated differential-drive rover and progressively introduces sensor simulation, localization, SLAM, path planning, local obstacle avoidance, computer vision, autonomous mission execution, telemetry, diagnostics, analytics, and a web-based control dashboard.

The architecture is intentionally designed around **hardware abstraction**. The autonomy stack should not fundamentally care whether sensor data comes from Gazebo or from a future physical rover. The simulation layer is therefore treated as the first deployment target rather than as a disposable prototype.

---

## Table of Contents

- [Project Overview](#project-overview)
- [Goals](#goals)
- [Non-Goals](#non-goals)
- [Why OpenRover](#why-openrover)
- [Core Design Principles](#core-design-principles)
- [System Architecture](#system-architecture)
- [Technology Stack](#technology-stack)
- [Repository Structure](#repository-structure)
- [Rover Model](#rover-model)
- [Development Roadmap](#development-roadmap)
  - [Phase 1 — Simulation](#phase-1--simulation)
  - [Phase 2 — Sensor Simulation](#phase-2--sensor-simulation)
  - [Phase 3 — Localization](#phase-3--localization)
  - [Phase 4 — SLAM](#phase-4--slam)
  - [Phase 5 — Path Planning](#phase-5--path-planning)
  - [Phase 6 — Local Obstacle Avoidance](#phase-6--local-obstacle-avoidance)
  - [Phase 7 — Computer Vision](#phase-7--computer-vision)
  - [Phase 8 — Autonomous Missions](#phase-8--autonomous-missions)
  - [Phase 9 — Telemetry and Dashboard](#phase-9--telemetry-and-dashboard)
- [Detailed Data Flow](#detailed-data-flow)
- [Software Interfaces](#software-interfaces)
- [Algorithms](#algorithms)
- [Simulation Scenarios](#simulation-scenarios)
- [Testing Strategy](#testing-strategy)
- [Benchmarking and Evaluation](#benchmarking-and-evaluation)
- [Fault Injection and Recovery](#fault-injection-and-recovery)
- [Telemetry and Observability](#telemetry-and-observability)
- [Database Design](#database-design)
- [Configuration](#configuration)
- [Build and Run](#build-and-run)
- [Development Workflow](#development-workflow)
- [CI/CD](#cicd)
- [Containerization](#containerization)
- [Documentation](#documentation)
- [AI-Assisted Development Disclosure](#ai-assisted-development-disclosure)
- [Code Quality and Engineering Standards](#code-quality-and-engineering-standards)
- [Security and Safety](#security-and-safety)
- [Future Hardware Support](#future-hardware-support)
- [Future Extensions](#future-extensions)
- [Milestones](#milestones)
- [Definition of Done](#definition-of-done)
- [Contributing](#contributing)
- [License](#license)
- [Project Status](#project-status)

---

# Project Overview

OpenRover is a software-first robotics platform centered around a simulated autonomous rover.

The project is intended to explore a complete autonomous robotics system, designed as a set of independently testable software components:

```text
                         ┌────────────────────────────┐
                         │       Web Dashboard        │
                         │   React + TypeScript       │
                         └─────────────┬──────────────┘
                                       │
                              REST / WebSocket
                                       │
                         ┌─────────────▼──────────────┐
                         │      ASP.NET Core API      │
                         │ Telemetry / Control / Data │
                         └─────────────┬──────────────┘
                                       │
                              Robot State / Commands
                                       │
    ┌──────────────────────────────────▼─────────────────────────────────────┐
    │                             OPENROVER CORE                             │
    │                                  C++20                                 │
    │                                                                        │
    │    Sensors → Localization → Mapping → Planning → Control → Missions    │
    │                                                                        │
    └─────────────────┬────────────┬────────────┬────────────┬───────────────┘
                      │            │            │            │
                      ▼            ▼            ▼            ▼
                   Camera        LiDAR         IMU        Encoders
                      │            │            │            │
                      └────────────┴────────────┴────────────┘
                                           │
                                           ▼
                                ┌──────────────────────┐
                                │   Gazebo Simulation  │
                                │ Physics + Environment│
                                └──────────────────────┘
```

The project is intended to become a reusable environment for:

- robotics algorithm development
- autonomous navigation research
- simulation experiments
- sensor-fusion experimentation
- computer-vision integration
- software architecture practice
- algorithm benchmarking
- reproducible robotics experiments
- telemetry and observability
- future hardware integration

---

# Goals

## Primary Goals

1. Build a modular autonomous rover entirely in simulation.
2. Keep autonomy algorithms independent from the simulator.
3. Implement important robotics algorithms explicitly rather than treating the entire system as a black box.
4. Provide deterministic and repeatable simulation scenarios where practical.
5. Create automated tests for core mathematical and algorithmic components.
6. Benchmark different algorithms using measurable metrics.
7. Build a professional telemetry and control interface.
8. Make the project extensible to physical hardware in the future.
9. Document assumptions, limitations, algorithms, and experimental results.

## Secondary Goals

- Explore C++ systems programming in a robotics context.
- Integrate C++, Python, .NET, and TypeScript in one coherent architecture.
- Learn practical robotics middleware and simulation workflows.
- Create a platform suitable for future research experiments.
- Develop reusable software interfaces for simulated and real sensors.
- Create datasets from simulation for computer-vision and ML experiments.
- And most important, have fun engineering!!

---

# Non-Goals

OpenRover is **not initially intended** to:

- replace mature production robotics stacks;
- implement a full physics engine;
- implement an entire ROS 2 replacement;
- support every robot topology;
- provide safety certification for physical robots;
- guarantee real-world navigation performance from simulation alone;
- claim research novelty without experimental evidence.

The project will reuse established infrastructure where that makes engineering sense.

For example:

- Gazebo provides the simulation and physics environment.
- ROS 2 can provide the middleware and message transport layer.
- OpenRover implements selected autonomy algorithms, orchestration, evaluation, and application-level software around them.

---

# Why OpenRover

Robotics projects often become demonstrations where a robot is wired together until it moves.

Since a lot of developers may not have access to hardware, OpenRover takes a different approach:

> **Treat the robot as a software system first.**

The project separates:

```text
Hardware / Simulation
        ↓
Sensor Abstraction
        ↓
Perception
        ↓
State Estimation
        ↓
Mapping
        ↓
Planning
        ↓
Control
        ↓
Mission Execution
        ↓
Telemetry / Observability
```

This separation makes it possible to:

- test a planner without running a camera;
- test localization with recorded sensor data;
- compare planners against the same environments;
- inject sensor faults;
- replay missions;
- change the simulator without rewriting autonomy code;
- eventually replace simulated hardware with physical hardware.

---

# Core Design Principles

## 1. Simulation-first

No physical hardware is required for the core project.

Simulation is treated as a first-class environment.

## 2. Hardware abstraction

Algorithms should depend on interfaces rather than directly on simulator-specific APIs.

Example:

```cpp
class ILidar {
public:
    virtual ~ILidar() = default;
    virtual LidarScan read() = 0;
};

class IMotorController {
public:
    virtual ~IMotorController() = default;
    virtual void setVelocity(double linear, double angular) = 0;
    virtual void stop() = 0;
};
```

A Gazebo implementation can satisfy these interfaces today.

A future real-hardware implementation can satisfy the same interfaces later.

## 3. Modular autonomy

Each major function should be independently replaceable:

```text
Localization
Mapping
Global Planning
Local Planning
Control
Perception
Mission Planning
```

## 4. Testability

Mathematical operations, state estimation, planning, collision checking, and mission logic should have automated tests.

## 5. Observability

Every autonomous run should be measurable and inspectable.

## 6. Reproducibility

Simulation runs should have:

- scenario IDs
- configuration files
- random seeds where relevant
- algorithm versions
- timestamps
- recorded metrics

## 7. Explicit failure handling

A robot should not silently fail.

The system should represent states such as:

```text
RUNNING
PAUSED
DEGRADED
REPLANNING
RECOVERY
FAILED
COMPLETE
```

## 8. Responsible AI-assisted development

AI tools may be used to assist with implementation, refactoring, debugging, documentation, test generation, and exploration.

But AI output must be reviewed and not automatically be considered correct.

See [AI-Assisted Development Disclosure](#ai-assisted-development-disclosure).

---

# System Architecture

OpenRover is divided into several logical layers.

```text
┌────────────────────────────────────────────────────────────┐
│                     PRESENTATION LAYER                     │
│                React + TypeScript Dashboard                │
└───────────────────────────┬────────────────────────────────┘
                            │
                    REST / WebSocket
                            │
┌───────────────────────────▼────────────────────────────────┐
│                     APPLICATION LAYER                      │
│                    ASP.NET Core Backend                    │
│                                                            │
│ Telemetry │ Commands │ Missions │ Runs │ Authentication*   │
└───────────────────────────┬────────────────────────────────┘
                            │
                       Robot API
                            │
┌───────────────────────────▼────────────────────────────────┐
│                     ROBOTICS CORE                          │
│                          C++20                             │
│                                                            │
│ Sensor Processing                                          │
│ State Estimation                                           │
│ Mapping                                                    │
│ Planning                                                   │
│ Control                                                    │
│ Perception                                                 │
│ Mission Execution                                          │
└───────────────────────────┬────────────────────────────────┘
                            │
                    ROS 2 / Adapters
                            │
┌───────────────────────────▼────────────────────────────────┐
│                     SIMULATION LAYER                       │
│                 Gazebo + Simulation Models                 │
└────────────────────────────────────────────────────────────┘
```

`*` Authentication and remote-access security are not prerequisites for the first milestones and should be introduced before exposing services outside a trusted development environment.

---

# Technology Stack

| Area | Technology |
|---|---|
| Core robotics | C++20 |
| Build system | CMake |
| Middleware | ROS 2 Jazzy |
| Simulation | Gazebo Harmonic |
| Linear algebra | Eigen |
| Computer vision | OpenCV |
| ML experiments | Python + PyTorch |
| API/backend | ASP.NET Core |
| Backend language | C# |
| Realtime transport | WebSockets |
| Frontend | React + TypeScript |
| Frontend styling | Tailwind CSS |
| Visualization | Three.js / browser-based canvas where appropriate |
| Database | PostgreSQL |
| Unit testing | GoogleTest |
| Python testing | pytest |
| API testing | .NET test tooling |
| Containerization | Docker |
| CI/CD | GitHub Actions |
| Documentation | Markdown |

This stack can evolve as the project develops. Technology choices should serve the architecture rather than become requirements for their own sake.

---

# Repository Structure

```text
OpenRover/
│
├── core/
│   ├── sensors/
│   │   ├── interfaces/
│   │   ├── lidar/
│   │   ├── imu/
│   │   ├── camera/
│   │   └── encoders/
│   │
│   ├── localization/
│   │   ├── odometry/
│   │   ├── ekf/
│   │   └── pose/
│   │
│   ├── mapping/
│   │   ├── occupancy_grid/
│   │   └── slam/
│   │
│   ├── planning/
│   │   ├── global/
│   │   │   ├── astar/
│   │   │   ├── dstar/
│   │   │   └── rrt/
│   │   └── local/
│   │
│   ├── control/
│   │   ├── pid/
│   │   └── trajectory/
│   │
│   ├── perception/
│   │   ├── detection/
│   │   ├── tracking/
│   │   └── geometry/
│   │
│   ├── missions/
│   │   ├── state_machine/
│   │   ├── task_planner/
│   │   └── recovery/
│   │
│   └── common/
│       ├── math/
│       ├── types/
│       ├── logging/
│       └── configuration/
│
├── simulation/
│   ├── worlds/
│   ├── models/
│   │   └── rover/
│   ├── sensors/
│   ├── plugins/
│   └── scenarios/
│
├── algorithms/
│   ├── benchmarks/
│   ├── reference/
│   └── experiments/
│
├── backend/
│   └── OpenRover.Api/
│       ├── Controllers/
│       ├── Services/
│       ├── Hubs/
│       ├── Models/
│       ├── Data/
│       └── Configuration/
│
├── dashboard/
│   └── OpenRover.Web/
│       ├── components/
│       ├── pages/
│       ├── hooks/
│       ├── services/
│       └── types/
│
├── python/
│   ├── perception/
│   ├── datasets/
│   ├── evaluation/
│   └── experiments/
│
├── tests/
│   ├── unit/
│   ├── integration/
│   ├── simulation/
│   └── end_to_end/
│
├── benchmarks/
│   ├── planning/
│   ├── localization/
│   ├── slam/
│   └── missions/
│
├── configs/
│   ├── rover/
│   ├── sensors/
│   ├── planners/
│   └── scenarios/
│
├── docs/
│   ├── architecture/
│   ├── algorithms/
│   ├── experiments/
│   └── development/
│
├── docker/
├── scripts/
├── assets/
├── CMakeLists.txt
├── docker-compose.yml
├── LICENSE
└── README.md
```

---

# Rover Model

## Drive Model

OpenRover initially uses a **two-wheel differential-drive rover**.

The rover has:

- left drive wheel;
- right drive wheel;
- optional passive caster(s);
- configurable wheel radius;
- configurable wheel separation;
- configurable maximum velocity;
- configurable acceleration and deceleration limits.

The robot state is represented as:

```text
x
y
theta
linear_velocity
angular_velocity
```

## Differential Drive

For left and right wheel velocities:

```text
v = (v_right + v_left) / 2

omega = (v_right - v_left) / wheel_separation
```

The pose is then integrated over time.

Exact integration and numerical stability should be handled in the implementation rather than assuming a simplified discrete update is always sufficient.

---

# Development Roadmap

The project is divided into nine primary phases.

The phases are ordered from foundational infrastructure to increasingly autonomous behavior.

---

# Phase 1 — Simulation

## Objective

Build a simulated rover that can be spawned, controlled, observed, and reset reliably.

## Deliverables

- Gazebo world.
- Rover model.
- Differential-drive controller.
- Basic command interface.
- Simulation startup scripts.
- Basic logging.
- Simulation reset functionality.

## Initial controls

The rover should support:

```text
FORWARD
BACKWARD
ROTATE_LEFT
ROTATE_RIGHT
STOP
```

A velocity-based API should eventually replace discrete keyboard actions:

```cpp
setVelocity(linear, angular);
```

## Success Criteria

A clean startup sequence should look like:

```text
Launch OpenRover
    ↓
Launch Gazebo
    ↓
Load environment
    ↓
Spawn rover
    ↓
Start C++ controller
    ↓
Send command
    ↓
Rover moves
```

The rover should be able to:

- move forward;
- rotate;
- stop;
- restart;
- reset to a known pose.

---

# Phase 2 — Sensor Simulation

## Objective

Make the simulated rover aware of the environment.

## Sensors

### LiDAR

Initial purposes:

- obstacle detection;
- occupancy mapping;
- localization;
- SLAM.

Expected data:

```text
timestamp
frame_id
angle_min
angle_max
angle_increment
range_min
range_max
ranges[]
```

### IMU

Expected data:

```text
orientation
angular_velocity
linear_acceleration
```

### Wheel Encoders

Used for odometry.

Expected data:

```text
left_wheel_position
right_wheel_position
left_wheel_velocity
right_wheel_velocity
```

### GPS

Used initially as a noisy global reference.

It may later be disabled for GPS-denied experiments.

### RGB Camera

Used in later perception phases.

### Depth Camera

Optional early sensor for future object localization and 3D perception.

---

# Phase 3 — Localization

## Objective

Estimate the rover's position and orientation.

## Step 1: Wheel Odometry

Use wheel motion to estimate pose.

Expected output:

```text
x
y
theta
```

Odometry should be deliberately tested with noise and drift.

## Step 2: IMU Integration

Use inertial measurements to supplement wheel odometry.

## Step 3: Extended Kalman Filter

Implement a sensor-fusion pipeline.

Conceptually:

```text
Wheel Odometry ─┐
                │
IMU ────────────┼──→ EKF ──→ Estimated Pose
                │
GPS ────────────┘
```

The implementation should explicitly represent:

- state vector;
- covariance;
- prediction step;
- measurement models;
- measurement covariance;
- update step.

## Localization Metrics

Record:

- position RMSE;
- heading error;
- drift over distance;
- convergence time;
- covariance behavior.

## Experiments

Compare:

```text
Odometry only
vs
Odometry + IMU
vs
Odometry + IMU + GPS
```

---

# Phase 4 — SLAM

## Objective

Allow the rover to build a map while estimating its own position.

## Initial Representation

Use a 2D occupancy grid.

Cells should represent at least:

```text
UNKNOWN
FREE
OCCUPIED
```

Optional probabilistic representation:

```text
P(occupied | observations)
```

## Initial Pipeline

```text
LiDAR Scan
    ↓
Scan Matching / Pose Update
    ↓
Pose Estimate
    ↓
Occupancy Grid Update
    ↓
Map
```

## SLAM Requirements

The system should support:

- map initialization;
- scan accumulation;
- pose updates;
- occupancy updates;
- map serialization;
- map reset;
- map visualization.

## SLAM Experiments

Measure:

- map quality;
- trajectory error;
- loop-closure behavior where supported;
- drift;
- computational cost.

---

# Phase 5 — Path Planning

## Objective

Generate a safe path between two locations.

## Global Planning

Initial algorithm:

### A*

Input:

```text
occupancy grid
start
goal
```

Output:

```text
ordered path cells / waypoints
```

## Additional Planners

The project should eventually support comparisons with:

- Dijkstra;
- D* Lite;
- RRT;
- RRT*.

## Planning Metrics

Every planning run should record:

- planning time;
- path length;
- number of nodes explored;
- number of replans;
- success/failure;
- minimum obstacle clearance;
- path smoothness where measurable.

## Path Smoothing

Raw grid paths should be converted into a trajectory appropriate for a mobile robot.

Possible stages:

```text
Grid path
    ↓
Waypoint reduction
    ↓
Geometric smoothing
    ↓
Velocity profile
    ↓
Controller
```

---

# Phase 6 — Local Obstacle Avoidance

## Objective

React to obstacles that are not represented in the original global path.

Global planning answers:

> Where should the robot go?

Local planning answers:

> What should the robot do right now?

Architecture:

```text
Global Planner
     ↓
Global Path
     ↓
Local Planner
     ↓
Velocity Command
     ↓
Robot
```

## Candidate Approaches

At least one local planning approach should be implemented first, followed by additional approaches for comparison.

Potential approaches:

- Dynamic Window Approach;
- Vector Field Histogram;
- potential fields;
- velocity-space sampling.

## Behavior

The rover should:

1. follow the global path;
2. detect nearby obstacles;
3. reduce speed when required;
4. temporarily deviate from the global path;
5. recover to the global trajectory;
6. trigger replanning when necessary.

---

# Phase 7 — Computer Vision

## Objective

Give the rover semantic perception.

## Initial Capability

Object detection from RGB camera frames.

Pipeline:

```text
Camera
   ↓
Image acquisition
   ↓
Pre-processing
   ↓
Object detector
   ↓
Bounding boxes / classes / confidence
   ↓
Object tracking / world association
```

## Example Output

```text
Object:
  class: red_box
  confidence: 0.91
  bounding_box: [...]
```

## Target-Based Navigation

The rover should eventually accept semantic goals:

```text
FIND(red_box)
```

rather than only geometric goals:

```text
GO_TO(x, y)
```

## Future Vision Features

- object tracking;
- segmentation;
- depth-assisted localization;
- visual landmarks;
- semantic maps;
- target confirmation;
- human detection;
- scene classification.

---

# Phase 8 — Autonomous Missions

## Objective

Move from navigation commands to high-level autonomous tasks.

## Mission Representation

A mission can contain:

```text
Mission
 ├── Task 1
 ├── Task 2
 ├── Task 3
 └── Completion condition
```

Example:

```text
MISSION: SEARCH_AND_RETURN

1. Navigate to Zone A
2. Search for target
3. Confirm target
4. Navigate to base
5. Complete mission
```

## State Machine

Expected high-level states:

```text
IDLE
 ↓
PLANNING
 ↓
NAVIGATING
 ↓
SEARCHING
 ↓
TARGET_FOUND
 ↓
RETURNING
 ↓
COMPLETE
```

Failure states may transition into:

```text
REPLANNING
RECOVERY
DEGRADED
FAILED
```

## Recovery Behavior

Examples:

```text
Path blocked
    ↓
Replan

Localization uncertain
    ↓
Slow down
    ↓
Attempt recovery
    ↓
If unresolved → mission degraded

Sensor unavailable
    ↓
Use fallback sensor model if possible
    ↓
Otherwise fail safely
```

## Mission Logging

Each mission should produce:

```text
Mission ID
Scenario ID
Algorithm versions
Start time
End time
Final state
Path length
Duration
Collisions
Replans
Failures
Sensor faults
```

---

# Phase 9 — Telemetry and Dashboard

## Objective

Provide a professional control, monitoring, and analytics interface.

## Backend

ASP.NET Core should expose:

### REST API

For:

- robot state;
- configuration;
- mission creation;
- mission history;
- scenario selection;
- run metadata;
- experiment results.

### WebSocket

For realtime:

- robot pose;
- velocity;
- sensor summaries;
- mission status;
- diagnostics;
- planner state;
- warnings.

## Dashboard Views

### 1. Mission Control

Controls:

```text
Start
Pause
Resume
Abort
Return to Base
Reset
```

### 2. Live Map

Display:

- SLAM map;
- robot pose;
- global path;
- local trajectory;
- goal;
- obstacles;
- detected objects.

### 3. Sensor Panel

Display:

- LiDAR status;
- camera status;
- IMU;
- GPS;
- encoders.

### 4. Diagnostics

Display:

```text
CPU
Memory
Simulation rate
Control frequency
Sensor health
Localization status
SLAM status
Planner status
Mission status
```

### 5. Analytics

Display:

```text
mission duration
path length
planning latency
collision count
replan count
localization error
goal error
```

---

# Detailed Data Flow

## Sensor Flow

```text
Gazebo
  │
  ├── LiDAR
  ├── IMU
  ├── GPS
  ├── Camera
  └── Encoders
          │
          ▼
   Sensor Interfaces
          │
          ▼
   Sensor Preprocessing
```

## Localization Flow

```text
Encoders ─────┐
              │
IMU ──────────┼──→ State Estimator → Robot Pose
              │
GPS ──────────┘
```

## Mapping Flow

```text
LiDAR + Robot Pose
        ↓
Scan Processing
        ↓
Occupancy Update
        ↓
Map
```

## Planning Flow

```text
Map + Start + Goal
        ↓
Global Planner
        ↓
Global Path
```

## Local Control Flow

```text
Global Path + Current Sensors
              ↓
       Local Planner
              ↓
      Velocity Command
              ↓
    Motion Controller
              ↓
        Drive System
```

## Mission Flow

```text
Mission
   ↓
Task
   ↓
Navigation / Perception
   ↓
Success?
 ┌─┴─┐
No  Yes
│    │
Recovery
│    ↓
└→ Next Task
```

---

# Software Interfaces

The internal architecture should use explicit interfaces wherever practical.

## Sensor Interfaces

```cpp
class ILidar {
public:
    virtual ~ILidar() = default;
    virtual LidarScan read() = 0;
};

class ICamera {
public:
    virtual ~ICamera() = default;
    virtual ImageFrame read() = 0;
};

class IIMU {
public:
    virtual ~IIMU() = default;
    virtual IMUReading read() = 0;
};
```

## Actuator Interface

```cpp
class IMotorController {
public:
    virtual ~IMotorController() = default;

    virtual void setVelocity(
        double linear,
        double angular
    ) = 0;

    virtual void stop() = 0;
};
```

## Localization Interface

```cpp
class ILocalization {
public:
    virtual ~ILocalization() = default;

    virtual PoseEstimate estimate(
        const SensorState& sensors
    ) = 0;
};
```

## Global Planner Interface

```cpp
class IGlobalPlanner {
public:
    virtual ~IGlobalPlanner() = default;

    virtual Path plan(
        const OccupancyGrid& map,
        const Pose& start,
        const Pose& goal
    ) = 0;
};
```

## Local Planner Interface

```cpp
class ILocalPlanner {
public:
    virtual ~ILocalPlanner() = default;

    virtual VelocityCommand compute(
        const Pose& pose,
        const Path& global_path,
        const SensorState& sensors
    ) = 0;
};
```

The purpose of these interfaces is not to maximize abstraction for its own sake. They exist to support:

- testing;
- simulation/hardware replacement;
- algorithm comparison;
- dependency injection;
- deterministic experiments.

---

# Algorithms

OpenRover is intended to contain both implementation code and comparative experiments.

## Localization

Planned:

- wheel odometry;
- IMU integration;
- Extended Kalman Filter;
- optional GPS fusion.

## Mapping

Planned:

- occupancy grids;
- ray-based LiDAR updates;
- scan matching;
- SLAM experiments.

## Planning

Planned:

- A*;
- Dijkstra;
- D* Lite;
- RRT;
- RRT*.

## Control

Planned:

- PID;
- trajectory following;
- optional model predictive control later.

## Perception

Planned:

- OpenCV preprocessing;
- neural object detection;
- object tracking;
- target localization.

## Mission Planning

Planned:

- finite-state machines;
- task graphs;
- retry policies;
- recovery behaviors.

---

# Simulation Scenarios

OpenRover should maintain a standardized scenario suite.

## Scenario 01 — Empty Room

Purpose:

- validate basic motion;
- validate localization;
- validate controller behavior.

## Scenario 02 — Corridor

Purpose:

- narrow-space navigation;
- wall proximity;
- local planning.

## Scenario 03 — Maze

Purpose:

- global planning;
- replanning;
- SLAM.

## Scenario 04 — Dynamic Obstacles

Purpose:

- local obstacle avoidance;
- dynamic replanning.

## Scenario 05 — Sensor Noise

Purpose:

- localization robustness;
- SLAM robustness.

## Scenario 06 — GPS Denied

Purpose:

- evaluate non-GPS localization.

## Scenario 07 — Blocked Route

Purpose:

- recovery behavior;
- planner failure handling.

## Scenario 08 — Search Area

Purpose:

- object detection;
- semantic navigation;
- mission execution.

## Scenario 09 — Multi-Goal Mission

Purpose:

- task sequencing;
- mission planning.

## Scenario 10 — Fault Injection

Purpose:

- degraded sensor operation;
- recovery;
- mission termination behavior.

---

# Testing Strategy

Testing is divided into four levels.

## Unit Tests

Test isolated components:

```text
Differential drive equations
PID controller
A*
RRT
Occupancy grid updates
EKF prediction
EKF update
Collision checking
Mission state transitions
```

## Integration Tests

Test interactions:

```text
LiDAR → Mapping
Sensors → Localization
Localization → Planner
Planner → Controller
Mission → Navigation
```

## Simulation Tests

Launch the simulated rover and evaluate actual behavior.

Examples:

```text
Does the rover reach the goal?
Does it avoid obstacles?
Does it recover when the route is blocked?
Does localization remain within tolerance?
```

## End-to-End Tests

Example:

```text
Create mission
    ↓
Start simulation
    ↓
Navigate
    ↓
Detect target
    ↓
Complete task
    ↓
Return
    ↓
Persist results
    ↓
Expose metrics through API
```

---

# Benchmarking and Evaluation

OpenRover should not only demonstrate algorithms visually.

The project should quantify performance.

## Planning Metrics

- planning time;
- path length;
- node expansions;
- path smoothness;
- obstacle clearance;
- success rate.

## Localization Metrics

- RMSE;
- maximum error;
- heading error;
- drift;
- covariance;
- convergence time.

## SLAM Metrics

- map accuracy;
- trajectory accuracy;
- map update latency;
- compute time;
- memory use.

## Navigation Metrics

- mission completion rate;
- collisions;
- replans;
- time to goal;
- total distance;
- average velocity;
- minimum clearance.

## Computer Vision Metrics

- precision;
- recall;
- F1;
- inference latency;
- confidence distribution.

## System Metrics

- CPU;
- RAM;
- control-loop frequency;
- simulation real-time factor;
- message latency;
- dropped messages.

---

# Fault Injection and Recovery

Robustness should be treated as a first-class feature.

Possible simulated failures:

```text
LiDAR unavailable
GPS unavailable
Camera unavailable
IMU noisy
Encoder drift
Delayed sensor messages
Dropped messages
Temporary obstacle
Invalid sensor values
Localization divergence
Planner failure
```

The system should expose its operational state.

Example:

```text
NORMAL
DEGRADED
RECOVERY
FAILED
```

A failure should be:

- detectable;
- logged;
- surfaced to telemetry;
- handled according to policy;
- reproducible.

---

# Telemetry and Observability

Every significant subsystem should emit structured telemetry.

## Example

```json
{
  "timestamp": "2026-08-22T10:00:00Z",
  "robot_id": "rover-01",
  "state": "NAVIGATING",
  "pose": {
    "x": 12.43,
    "y": 8.21,
    "theta": 1.42
  },
  "velocity": {
    "linear": 1.20,
    "angular": 0.14
  },
  "localization": {
    "status": "GOOD",
    "position_error_estimate": 0.18
  }
}
```

Logs should contain structured fields wherever practical rather than relying entirely on plain text.

---

# Database Design

PostgreSQL can be used for persistent experiment and telemetry metadata.

Possible entities:

```text
robots
scenarios
missions
mission_runs
telemetry_samples
sensor_events
planner_runs
localization_runs
fault_events
experiment_results
```

## Example Relationship

```text
Scenario
   │
   └── Mission Run
          │
          ├── Telemetry
          ├── Fault Events
          ├── Planner Results
          └── Final Metrics
```

Large binary sensor recordings should not automatically be stored directly in PostgreSQL. File-based or object-storage approaches can be used for large artifacts, while the database stores metadata and references.

---

# Configuration

Avoid hardcoding robotics parameters in source code.

Configuration should cover:

```yaml
robot:
  wheel_radius: 0.10
  wheel_separation: 0.45
  max_linear_velocity: 2.0
  max_angular_velocity: 2.5

localization:
  use_gps: true
  use_imu: true
  use_wheel_odometry: true

planning:
  algorithm: astar
  goal_tolerance: 0.25

control:
  controller: pid

mission:
  replan_attempts: 3
```

Configuration files should be version-controlled where they describe reproducible experiments.

Secrets should never be committed.

---

# Build and Run

The exact commands may evolve as the project matures. The intended developer workflow is approximately:

## Prerequisites

Install:

- Git
- C++20 compiler
- CMake
- ROS 2 Jazzy
- Gazebo Harmonic
- .NET SDK
- Node.js
- Python
- Docker

## Clone

```bash
git clone https://github.com/zishaan1911/OpenRover.git
cd OpenRover
```

## Build C++

```bash
cmake -S . -B build
cmake --build build
```

## Run Tests

```bash
ctest --test-dir build --output-on-failure
```

## Start Simulation

The eventual goal is a simple command such as:

```bash
./scripts/run_simulation.sh
```

or:

```bash
docker compose up
```

## Start Backend

```bash
dotnet run --project backend/OpenRover.Api
```

## Start Dashboard

```bash
cd dashboard/OpenRover.Web
npm install
npm run dev
```

Exact commands should be documented and kept synchronized with the repository as implementation progresses.

---

# Development Workflow

A feature should generally move through the following process:

```text
Issue / Experiment
      ↓
Design
      ↓
Implementation
      ↓
Unit Tests
      ↓
Integration Test
      ↓
Simulation Scenario
      ↓
Benchmark
      ↓
Documentation
      ↓
Pull Request
      ↓
Merge
```

For algorithm changes, record:

- assumptions;
- input/output;
- algorithm complexity where useful;
- parameter choices;
- test results;
- benchmark results;
- known limitations.

---

# CI/CD

GitHub Actions should eventually run:

## C++

```text
Format check
     ↓
Build
     ↓
Unit tests
     ↓
Static analysis
```

## .NET

```text
Restore
  ↓
Build
  ↓
Unit tests
  ↓
Integration tests
```

## Frontend

```text
Install
  ↓
Lint
  ↓
Type check
  ↓
Build
```

## Python

```text
Install dependencies
  ↓
Lint / type checks where configured
  ↓
pytest
```

## Repository-level checks

- broken documentation links where practical;
- configuration validation;
- Docker build;
- selected simulation smoke tests.

---

# Containerization

Docker should simplify the development environment.

Potential services:

```text
gazebo
openrover-core
openrover-api
postgres
dashboard
```

A development environment may eventually be launched through:

```bash
docker compose up
```

The containerized system should not hide important functionality from developers. Native execution should remain possible where practical, especially for graphics-heavy simulation workflows.

---

# Documentation

Documentation should exist at multiple levels.

## README

High-level project description and developer quick start.

## Architecture Documentation

Explain:

- module boundaries;
- data flow;
- interfaces;
- middleware;
- communication protocols.

## Algorithm Documentation

For each major algorithm:

```text
Purpose
Inputs
Outputs
Assumptions
Mathematical formulation
Implementation
Complexity
Parameters
Failure modes
Evaluation
```

## Experiment Documentation

Every major experiment should record:

- objective;
- environment;
- configuration;
- algorithm;
- parameters;
- random seed;
- results;
- interpretation;
- limitations.

---

# AI-Assisted Development Disclosure

OpenRover is an **AI-assisted software development project**.

AI coding tools may be used throughout development to assist with:

- code scaffolding;
- implementation suggestions;
- refactoring;
- debugging;
- test generation;
- documentation;
- API design exploration;
- algorithm explanations;
- boilerplate generation;
- code review assistance;
- development workflow automation.

## Important Principle

AI-generated or AI-suggested code will be reviewed and is **not automatically considered correct, safe, or authoritative**.

Code incorporated into the project should be:

1. reviewed by the project maintainer;
2. understood sufficiently to be maintained;
3. compiled and tested;
4. validated against the intended behavior;
5. evaluated through simulation where applicable;
6. corrected or rewritten when necessary.

AI assistance does not replace engineering judgment.

## Transparency

The use of AI assistance is disclosed intentionally because the project aims to demonstrate the process of developing a complex software system with modern development tools rather than imply that every line of code was manually authored from scratch.

The final repository should distinguish between:

```text
AI-assisted generation
Human review
Human decision-making
Automated validation
Experimental validation
```

where such distinctions are useful.

## Responsibility

The project maintainer remains responsible for:

- architectural decisions;
- code that is committed;
- dependency selection;
- algorithm choices;
- experimental conclusions;
- security decisions;
- licensing decisions;
- documentation accuracy.

AI tools may help produce an implementation, but they do not constitute project ownership or authoritative verification.

## Recommended Attribution Practice

When a generated or heavily AI-assisted contribution is significant enough to affect project history or reproducibility, the development notes may record:

```text
Tool used
Purpose
Human review performed
Tests executed
Result
```

No AI tool should be listed as the author, copyright holder, maintainer, or accountable party for the project unless a specific tool's license or policy explicitly requires otherwise.

---

# Code Quality and Engineering Standards

## C++

Target:

```text
C++20
```

Expected practices:

- RAII;
- const correctness;
- meaningful naming;
- small testable components;
- explicit ownership;
- limited global state;
- error handling;
- deterministic interfaces where appropriate;
- documentation for non-obvious mathematics.

## C#

Use standard .NET conventions.

Prefer:

- dependency injection;
- async APIs where appropriate;
- structured logging;
- explicit DTOs;
- validation;
- separation between API, services, and persistence.

## TypeScript

Prefer:

- strict type checking;
- reusable components;
- explicit API types;
- predictable state management;
- minimal `any`.

## Python

Python should primarily support:

- ML;
- data analysis;
- perception experiments;
- evaluation;
- dataset tooling.

---

# Security and Safety

OpenRover is primarily a simulation project.

However, the architecture may eventually control physical hardware.

Therefore, future hardware integration must consider:

- command validation;
- velocity limits;
- emergency stop;
- watchdog timers;
- communication loss;
- sensor failure;
- battery limits;
- mechanical constraints;
- safe startup states.

Remote control APIs should not be exposed to an untrusted network without appropriate authentication and authorization.

Simulation results must not be treated as proof that a physical rover is safe.

---

# Future Hardware Support

The project is explicitly intended to make future hardware integration possible.

The architecture should eventually look like:

```text
                        OpenRover Core
                              │
                    Hardware Abstraction
                              │
                ┌─────────────┴─────────────┐
                │                           │
           Simulation                  Physical Rover
                │                           │
              Gazebo              MCU / SBC / Sensors
```

Possible future platforms:

- Raspberry Pi;
- ESP32;
- Arduino-class microcontroller;
- USB LiDAR;
- USB camera;
- IMU;
- wheel encoders;
- motor controllers.

The autonomy software should avoid requiring hardware-specific code in high-level planning and mission modules.

---

# Future Extensions

Possible future features include:

## Advanced Control

- MPC;
- adaptive control;
- trajectory optimization.

## Advanced Perception

- instance segmentation;
- depth-based object localization;
- semantic segmentation;
- visual odometry.

## Learning-Based Navigation

- imitation learning;
- reinforcement learning;
- learned local planners;
- learned perception.

## Multi-Robot Systems

- multi-rover coordination;
- task allocation;
- swarm exploration;
- distributed mapping.

## Digital Twin

A richer digital twin can provide:

- 3D model;
- historical telemetry;
- component health;
- simulated faults;
- replayable missions.

## Cloud / Remote Operations

Potential future architecture:

```text
Robot
  ↓
Edge Runtime
  ↓
Cloud API
  ↓
Dashboard
```

This should only be introduced after the core local architecture is stable.

---

# Milestones

## Milestone 0 — Project Foundation

- [✔] Repository created
- [✔] License selected
- [✔] Initial architecture documented
- [✔] Build system configured
- [✔] CI skeleton configured

## Milestone 1 — Basic Rover

- [✔] Gazebo world
- [✔] Rover model
- [✔] Differential drive
- [✔] Basic control
- [✔] Reset functionality

## Milestone 2 — Sensors

- [ ] LiDAR
- [ ] IMU
- [ ] Wheel encoders
- [ ] GPS
- [ ] Camera
- [ ] Sensor interfaces
- [ ] Sensor logging

## Milestone 3 — Localization

- [ ] Wheel odometry
- [ ] IMU integration
- [ ] EKF
- [ ] Localization metrics
- [ ] Localization visualization

## Milestone 4 — SLAM

- [ ] Occupancy grid
- [ ] LiDAR map updates
- [ ] Scan matching
- [ ] SLAM experiments
- [ ] Map export

## Milestone 5 — Planning

- [ ] A*
- [ ] Path smoothing
- [ ] Collision checking
- [ ] Planner benchmark
- [ ] RRT
- [ ] Additional planners

## Milestone 6 — Local Navigation

- [ ] Local planner
- [ ] Dynamic obstacle detection
- [ ] Replanning
- [ ] Recovery
- [ ] Navigation benchmark

## Milestone 7 — Vision

- [ ] Camera pipeline
- [ ] Object detection
- [ ] Tracking
- [ ] Target localization
- [ ] Semantic goals

## Milestone 8 — Missions

- [ ] Mission representation
- [ ] Mission state machine
- [ ] Task execution
- [ ] Recovery behaviors
- [ ] Mission logging

## Milestone 9 — Dashboard

- [ ] ASP.NET Core API
- [ ] WebSocket telemetry
- [ ] Live map
- [ ] Mission control
- [ ] Diagnostics
- [ ] Analytics
- [ ] PostgreSQL persistence

## Milestone 10 — Engineering Hardening

- [ ] Comprehensive tests
- [ ] Benchmark suite
- [ ] Docker setup
- [ ] CI/CD
- [ ] Documentation
- [ ] Reproducibility tooling
- [ ] Fault injection suite

---

# Definition of Done

A feature should generally be considered complete only when:

- implementation exists;
- interfaces are documented;
- relevant tests exist;
- failure behavior is understood;
- simulation behavior has been verified;
- telemetry is available where useful;
- documentation is updated;
- configuration is reproducible;
- CI passes.

For algorithms, "done" should also mean that the algorithm has been evaluated against a representative scenario rather than only demonstrated once.

---

# Contributing

OpenRover accepts external contributions.

Contributors should:

1. read the architecture documentation;
2. keep modules focused;
3. add tests for algorithmic changes;
4. avoid hardcoding experimental parameters;
5. document non-obvious mathematical decisions;
6. report reproducible bugs;
7. distinguish benchmark changes from algorithm changes;
8. avoid committing secrets or generated artifacts unnecessarily.

For larger changes, a design issue or proposal should be opened before implementation.

---

# License

The final license should be selected before the project is published as an open-source repository.

The license should clearly define:

- modification rights;
- redistribution rights;
- attribution requirements;
- patent terms where applicable;
- warranty/liability limitations.

A common permissive option for software projects is the MIT License, but the final choice should be made deliberately based on the project's intended use.

---

# Project Status

OpenRover is currently in the **planning / initial development stage**.

The project is being built incrementally.

The roadmap is intentionally ambitious and will evolve as implementation and experimentation reveal new requirements.

The current priority is:

```text
Simulation
    ↓
Sensors
    ↓
Localization
    ↓
SLAM
    ↓
Planning
    ↓
Obstacle Avoidance
    ↓
Computer Vision
    ↓
Autonomous Missions
    ↓
Telemetry Dashboard
```

---

# Project Philosophy

OpenRover is built around a simple idea:

> **A robot is not just a machine that moves. It is a distributed, sensor-driven software system operating under uncertainty.**

The goal of this project is therefore not merely to make a simulated rover reach a destination.

The goal is to build a complete engineering system in which:

```text
Sensors
   ↓
Perception
   ↓
State Estimation
   ↓
World Model
   ↓
Planning
   ↓
Control
   ↓
Mission Execution
   ↓
Observability
   ↓
Evaluation
```

can be designed, tested, measured, visualized, and eventually transferred from simulation to physical hardware.

---

## Long-Term Vision

The long-term vision for OpenRover is a modular robotics platform where new algorithms can be plugged into the same autonomous stack and compared under controlled conditions.

Potential end state:

```text
                  ┌─────────────────────┐
                  │   Mission Planner   │
                  └──────────┬──────────┘
                             │
                  ┌──────────▼──────────┐
                  │ Autonomous Manager  │
                  └──────────┬──────────┘
                             │
             ┌───────────────┼────────────────┐
             │               │                │
             ▼               ▼                ▼
       Perception       Localization       Planning
             │               │                │
             └───────────────┼────────────────┘
                             ▼
                      Motion Control
                             │
                             ▼
                    Hardware Abstraction
                             │
                  ┌──────────┴──────────┐
                  │                     │
               Gazebo              Real Robot
```

The same high-level autonomy software should ultimately be capable of operating against different environments, scenarios, algorithms, and hardware adapters.

---

## Final Note

OpenRover is intended to be developed as a **software engineering project**, not merely as a robotics demo.

That means:

- architecture matters;
- tests matter;
- reproducibility matters;
- measurable results matter;
- failure handling matters;
- documentation matters;
- transparency about AI-assisted development matters.

The project may use AI during development, but **AI assistance should not replace human review, engineering decisions, experimentation, or responsibility for the resulting software.**
