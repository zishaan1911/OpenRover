#include "differential_drive_kinematics.hpp"

#include <cmath>

namespace openrover::math {

BodyVelocity computeBodyVelocityFromWheelVelocities(
    const WheelVelocities& wheelVelocities,
    const DifferentialDriveConfiguration& driveConfiguration)
{
    // TODO: implement this function.
    // See the pseudocode above this function's declaration in
    // differential_drive_kinematics.hpp for step-by-step guidance.

    BodyVelocity bodyVelocity;
    return bodyVelocity;
}

Pose2D integratePoseForward(
    const Pose2D& startingPose,
    const BodyVelocity& bodyVelocity,
    double deltaTimeSeconds)
{
    // TODO: implement this function.
    // See the pseudocode above this function's declaration in
    // differential_drive_kinematics.hpp for step-by-step guidance.

    Pose2D newPose;
    return newPose;
}

} // namespace openrover::math
