#pragma once

#include "pose2d.hpp"

namespace openrover::math {

// The rover has two independently driven wheels: left and right.
// This struct holds how fast each wheel's rim is moving along the
// ground, measured in meters per second.
struct WheelVelocities
{
    double leftWheelVelocityMetersPerSecond = 0.0;
    double rightWheelVelocityMetersPerSecond = 0.0;
};

// The rover's body, as a whole, moves more simply than each wheel
// does individually: it moves forward/backward at some linear speed,
// and it turns at some angular speed.
struct BodyVelocity
{
    double linearVelocityMetersPerSecond = 0.0;   // called "v" in the README
    double angularVelocityRadiansPerSecond = 0.0; // called "omega" in the README
};

// Physical measurements of the rover chassis needed for the
// kinematics math below.
struct DifferentialDriveConfiguration
{
    // Distance between the centers of the left and right wheels,
    // measured in meters.
    double wheelSeparationMeters = 0.4;
};

// -----------------------------------------------------------------------
// TODO (you implement this in the .cpp file):
// Convert wheel velocities into a single body velocity.
//
// Formulas (also given in the README's "Differential Drive" section):
//   v     = (v_right + v_left) / 2
//   omega = (v_right - v_left) / wheel_separation
//
// Pseudocode:
//   1. Read leftWheelVelocity and rightWheelVelocity from
//      wheelVelocities.
//   2. Compute linearVelocity using the "v" formula above.
//   3. Compute angularVelocity using the "omega" formula above.
//   4. Build and return a BodyVelocity holding those two values.
// -----------------------------------------------------------------------
BodyVelocity computeBodyVelocityFromWheelVelocities(
    const WheelVelocities& wheelVelocities,
    const DifferentialDriveConfiguration& driveConfiguration);

// -----------------------------------------------------------------------
// TODO (you implement this in the .cpp file):
// Move the rover's pose forward in time, given where it started, how
// fast it's currently moving (body velocity), and how much time has
// passed since the last update.
//
// This is the "pose integration" step the README mentions:
//   "The pose is then integrated over time. Exact integration and
//    numerical stability should be handled in the implementation
//    rather than assuming a simplified discrete update is always
//    sufficient."
//
// For this first version, use the simplest approach ("Euler
// integration"):
//   newX     = oldX     + linearVelocity * cos(oldTheta) * deltaTimeSeconds
//   newY     = oldY     + linearVelocity * sin(oldTheta) * deltaTimeSeconds
//   newTheta = oldTheta + angularVelocity * deltaTimeSeconds
//
// Known limitation of this approach (just so you're aware, not
// something to fix yet): over one time step, it assumes the rover
// travels in a straight line at its *starting* heading, then snaps to
// the new heading at the very end of the step. For small
// deltaTimeSeconds and slow turning, this is a fine approximation.
// It drifts more as deltaTimeSeconds or angularVelocity grows. We'll
// come back to this later with a more accurate method — that's a
// deliberate future learning step, not a requirement right now.
//
// Pseudocode:
//   1. Start from startingPose.x, startingPose.y, startingPose.theta.
//   2. Compute newX and newY using the two formulas above.
//   3. Compute newTheta using the formula above.
//   4. Pass newTheta through normalizeAngleRadians() (declared in
//      pose2d.hpp) so it stays within (-pi, pi].
//   5. Build and return a Pose2D holding newX, newY, and the
//      normalized newTheta.
// -----------------------------------------------------------------------
Pose2D integratePoseForward(
    const Pose2D& startingPose,
    const BodyVelocity& bodyVelocity,
    double deltaTimeSeconds);

} // namespace openrover::math
