#pragma once

#include "../math/differential_drive_kinematics.hpp"
#include "../math/pose2d.hpp"

namespace openrover {

// RoverModel represents the state of a single differential-drive
// rover: where it currently is (its pose), and how fast it is
// currently commanded to move (its body velocity).
//
// This class does not know anything about Gazebo, ROS 2, or real
// hardware, it is a pure, simulator-independent model of the
// robot's motion, built entirely on the kinematics functions
// already implemented. Later, a Gazebo-backed controller will call
// setVelocity() the same way a keyboard-teleop node or an autonomy
// stack eventually will, and the math underneath won't need to
// change.
class RoverModel
{
public:
    // Creates a RoverModel with the given physical configuration
    // (wheel separation, etc.), starting at the given pose. If no
    // starting pose is given, the rover starts at the origin, facing
    // along +x.
    explicit RoverModel(
        math::DifferentialDriveConfiguration driveConfiguration,
        math::Pose2D startingPose = math::Pose2D{});

    // Commands the rover to move at the given linear and angular
    // velocity, starting from the next tick() call. Calling this does
    // NOT move the rover immediately — it only records what velocity
    // to use going forward, until it's changed again.
    void setVelocity(double linearVelocityMetersPerSecond, double angularVelocityRadiansPerSecond);

    // Same idea as setVelocity(), but expressed as left/right wheel
    // velocities instead of a body velocity. Internally this reuses
    // computeBodyVelocityFromWheelVelocities() (the function
    // already implemented) to convert, then calls setVelocity().
    void setWheelVelocities(const math::WheelVelocities& wheelVelocities);

    // Convenience for setVelocity(0, 0).
    void stop();

    // Advances the rover's pose forward in time by the given number of seconds, using the currently commanded velocity.
    void tick(double deltaTimeSeconds);

    // Returns the rover's current pose (position + heading).
    math::Pose2D getCurrentPose() const;

    // Returns the velocity the rover is currently commanded to move at.
    math::BodyVelocity getCurrentCommandedVelocity() const;

    // Instantly moves the rover to a new pose, ignoring any physics,
    // and clears any commanded velocity.
    void resetToPose(const math::Pose2D& pose);

private:
    math::DifferentialDriveConfiguration driveConfiguration_;
    math::Pose2D currentPose_;
    math::BodyVelocity commandedVelocity_;
};

} // namespace openrover