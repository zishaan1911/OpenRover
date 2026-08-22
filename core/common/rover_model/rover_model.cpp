#include "rover_model.hpp"

namespace openrover {

RoverModel::RoverModel(
    math::DifferentialDriveConfiguration driveConfiguration,
    math::Pose2D startingPose)
    : driveConfiguration_(driveConfiguration)
    , currentPose_(startingPose)
    , commandedVelocity_{}
{
}

void RoverModel::setVelocity(double linearVelocityMetersPerSecond, double angularVelocityRadiansPerSecond)
{
    commandedVelocity_.linearVelocityMetersPerSecond = linearVelocityMetersPerSecond;
    commandedVelocity_.angularVelocityRadiansPerSecond = angularVelocityRadiansPerSecond;
}

void RoverModel::setWheelVelocities(const math::WheelVelocities& wheelVelocities)
{
    math::BodyVelocity bodyVelocity =
        math::computeBodyVelocityFromWheelVelocities(wheelVelocities, driveConfiguration_);

    setVelocity(bodyVelocity.linearVelocityMetersPerSecond, bodyVelocity.angularVelocityRadiansPerSecond);
}

void RoverModel::stop()
{
    setVelocity(0.0, 0.0);
}

void RoverModel::tick(double deltaTimeSeconds)
{
    currentPose_ = math::integratePoseForward(currentPose_, commandedVelocity_, deltaTimeSeconds);
}

math::Pose2D RoverModel::getCurrentPose() const
{
    return currentPose_;
}

math::BodyVelocity RoverModel::getCurrentCommandedVelocity() const
{
    return commandedVelocity_;
}

void RoverModel::resetToPose(const math::Pose2D& pose)
{
    currentPose_ = pose;
    commandedVelocity_ = math::BodyVelocity{};
}

} // namespace openrover