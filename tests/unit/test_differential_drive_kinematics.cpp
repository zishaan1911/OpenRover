#include <gtest/gtest.h>

#include "differential_drive_kinematics.hpp"

using namespace openrover::math;

// -----------------------------------------------------------------------
// Tests for computeBodyVelocityFromWheelVelocities
// -----------------------------------------------------------------------

TEST(DifferentialDriveKinematics, BothWheelsSameSpeedMeansStraightLine)
{
    // If both wheels spin at the same speed, the rover should move
    // straight ahead (zero angular velocity) at that same speed.
    WheelVelocities wheelVelocities;
    wheelVelocities.leftWheelVelocityMetersPerSecond = 1.0;
    wheelVelocities.rightWheelVelocityMetersPerSecond = 1.0;

    DifferentialDriveConfiguration driveConfiguration;
    driveConfiguration.wheelSeparationMeters = 0.5;

    BodyVelocity bodyVelocity =
        computeBodyVelocityFromWheelVelocities(wheelVelocities, driveConfiguration);

    EXPECT_NEAR(bodyVelocity.linearVelocityMetersPerSecond, 1.0, 1e-9);
    EXPECT_NEAR(bodyVelocity.angularVelocityRadiansPerSecond, 0.0, 1e-9);
}

TEST(DifferentialDriveKinematics, OppositeWheelSpeedsMeansSpinInPlace)
{
    // If the right wheel spins forward and the left wheel spins
    // backward at the same speed, the rover spins in place: zero
    // linear velocity, non-zero angular velocity.
    WheelVelocities wheelVelocities;
    wheelVelocities.leftWheelVelocityMetersPerSecond = -1.0;
    wheelVelocities.rightWheelVelocityMetersPerSecond = 1.0;

    DifferentialDriveConfiguration driveConfiguration;
    driveConfiguration.wheelSeparationMeters = 0.5;

    BodyVelocity bodyVelocity =
        computeBodyVelocityFromWheelVelocities(wheelVelocities, driveConfiguration);

    EXPECT_NEAR(bodyVelocity.linearVelocityMetersPerSecond, 0.0, 1e-9);
    EXPECT_NEAR(bodyVelocity.angularVelocityRadiansPerSecond, 4.0, 1e-9);
}

// -----------------------------------------------------------------------
// Tests for integratePoseForward
// -----------------------------------------------------------------------

TEST(DifferentialDriveKinematics, DrivingStraightAlongXAxis)
{
    // Starting at the origin, facing along +x (theta = 0), driving
    // straight for 2 seconds at 1 m/s should move the rover 2 meters
    // along x, with no change in y or heading.
    Pose2D startingPose;
    startingPose.x = 0.0;
    startingPose.y = 0.0;
    startingPose.theta = 0.0;

    BodyVelocity bodyVelocity;
    bodyVelocity.linearVelocityMetersPerSecond = 1.0;
    bodyVelocity.angularVelocityRadiansPerSecond = 0.0;

    Pose2D newPose = integratePoseForward(startingPose, bodyVelocity, 2.0);

    EXPECT_NEAR(newPose.x, 2.0, 1e-9);
    EXPECT_NEAR(newPose.y, 0.0, 1e-9);
    EXPECT_NEAR(newPose.theta, 0.0, 1e-9);
}

TEST(DifferentialDriveKinematics, SpinningInPlaceChangesHeadingOnly)
{
    // Starting away from the origin, spinning in place (linear
    // velocity = 0) should only change theta, never x or y.
    Pose2D startingPose;
    startingPose.x = 1.0;
    startingPose.y = 1.0;
    startingPose.theta = 0.0;

    BodyVelocity bodyVelocity;
    bodyVelocity.linearVelocityMetersPerSecond = 0.0;
    bodyVelocity.angularVelocityRadiansPerSecond = 1.5707963267948966; // pi/2 per second

    Pose2D newPose = integratePoseForward(startingPose, bodyVelocity, 1.0);

    EXPECT_NEAR(newPose.x, 1.0, 1e-9);
    EXPECT_NEAR(newPose.y, 1.0, 1e-9);
    EXPECT_NEAR(newPose.theta, 1.5707963267948966, 1e-9);
}
