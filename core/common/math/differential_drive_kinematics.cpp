#include "differential_drive_kinematics.hpp"

#include <cmath>

namespace openrover::math {

BodyVelocity computeBodyVelocityFromWheelVelocities(
    const WheelVelocities& wheelVelocities,
    const DifferentialDriveConfiguration& driveConfiguration)
{
    BodyVelocity bodyVelocity;
    bodyVelocity.linearVelocityMetersPerSecond = (wheelVelocities.rightWheelVelocityMetersPerSecond + wheelVelocities.leftWheelVelocityMetersPerSecond) / 2.0;
    bodyVelocity.angularVelocityRadiansPerSecond = (wheelVelocities.rightWheelVelocityMetersPerSecond - wheelVelocities.leftWheelVelocityMetersPerSecond) / driveConfiguration.wheelSeparationMeters;
    return bodyVelocity;
}

Pose2D integratePoseForward(
    const Pose2D& startingPose,
    const BodyVelocity& bodyVelocity,
    double deltaTimeSeconds)
{
    Pose2D newPose;
    newPose.x = startingPose.x + bodyVelocity.linearVelocityMetersPerSecond * std::cos(startingPose.theta) * deltaTimeSeconds;
    newPose.y = startingPose.y + bodyVelocity.linearVelocityMetersPerSecond * std::sin(startingPose.theta) * deltaTimeSeconds;
    newPose.theta = normalizeAngleRadians(startingPose.theta + bodyVelocity.angularVelocityRadiansPerSecond * deltaTimeSeconds);
    return newPose;
}

} // namespace openrover::math
