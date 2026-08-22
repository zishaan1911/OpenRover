#include <gtest/gtest.h>

#include "rover_model.hpp"

using namespace openrover;
using namespace openrover::math;

namespace {

// Small helper so every test doesn't have to repeat this setup.
RoverModel makeRoverAtOrigin()
{
    DifferentialDriveConfiguration driveConfiguration;
    driveConfiguration.wheelSeparationMeters = 0.5;

    Pose2D startingPose; // defaults to x=0, y=0, theta=0
    return RoverModel(driveConfiguration, startingPose);
}

} // namespace

TEST(RoverModel, StartsAtTheGivenPoseWithZeroVelocity)
{
    RoverModel rover = makeRoverAtOrigin();

    Pose2D pose = rover.getCurrentPose();
    EXPECT_NEAR(pose.x, 0.0, 1e-9);
    EXPECT_NEAR(pose.y, 0.0, 1e-9);
    EXPECT_NEAR(pose.theta, 0.0, 1e-9);

    BodyVelocity velocity = rover.getCurrentCommandedVelocity();
    EXPECT_NEAR(velocity.linearVelocityMetersPerSecond, 0.0, 1e-9);
    EXPECT_NEAR(velocity.angularVelocityRadiansPerSecond, 0.0, 1e-9);
}

TEST(RoverModel, TickingWithoutSettingVelocityDoesNotMoveTheRover)
{
    RoverModel rover = makeRoverAtOrigin();

    rover.tick(5.0); // 5 seconds pass, but nobody commanded a velocity

    Pose2D pose = rover.getCurrentPose();
    EXPECT_NEAR(pose.x, 0.0, 1e-9);
    EXPECT_NEAR(pose.y, 0.0, 1e-9);
    EXPECT_NEAR(pose.theta, 0.0, 1e-9);
}

TEST(RoverModel, DrivingStraightForOneTickMovesForward)
{
    RoverModel rover = makeRoverAtOrigin();

    rover.setVelocity(1.0, 0.0); // 1 m/s straight ahead, no turning
    rover.tick(2.0);             // 2 seconds pass

    Pose2D pose = rover.getCurrentPose();
    EXPECT_NEAR(pose.x, 2.0, 1e-9);
    EXPECT_NEAR(pose.y, 0.0, 1e-9);
    EXPECT_NEAR(pose.theta, 0.0, 1e-9);
}

TEST(RoverModel, MultipleTicksAccumulateDistance)
{
    RoverModel rover = makeRoverAtOrigin();

    rover.setVelocity(1.0, 0.0);
    rover.tick(1.0); // now at x = 1.0
    rover.tick(1.0); // now at x = 2.0
    rover.tick(1.0); // now at x = 3.0

    Pose2D pose = rover.getCurrentPose();
    EXPECT_NEAR(pose.x, 3.0, 1e-9);
    EXPECT_NEAR(pose.y, 0.0, 1e-9);
    EXPECT_NEAR(pose.theta, 0.0, 1e-9);
}

TEST(RoverModel, StopSetsVelocityToZeroSoFurtherTicksDoNothing)
{
    RoverModel rover = makeRoverAtOrigin();

    rover.setVelocity(1.0, 0.0);
    rover.tick(1.0); // moves to x = 1.0

    rover.stop();
    rover.tick(5.0); // should NOT move, since velocity is now zero

    Pose2D pose = rover.getCurrentPose();
    EXPECT_NEAR(pose.x, 1.0, 1e-9);
    EXPECT_NEAR(pose.y, 0.0, 1e-9);
}

TEST(RoverModel, ResetToPoseOverridesPositionAndClearsVelocity)
{
    RoverModel rover = makeRoverAtOrigin();

    rover.setVelocity(1.0, 0.0);
    rover.tick(1.0); // moves to x = 1.0

    Pose2D resetPose;
    resetPose.x = 5.0;
    resetPose.y = -3.0;
    resetPose.theta = 0.0;
    rover.resetToPose(resetPose);

    Pose2D pose = rover.getCurrentPose();
    EXPECT_NEAR(pose.x, 5.0, 1e-9);
    EXPECT_NEAR(pose.y, -3.0, 1e-9);

    BodyVelocity velocity = rover.getCurrentCommandedVelocity();
    EXPECT_NEAR(velocity.linearVelocityMetersPerSecond, 0.0, 1e-9);
    EXPECT_NEAR(velocity.angularVelocityRadiansPerSecond, 0.0, 1e-9);

    // A tick after reset should do nothing, since velocity was cleared.
    rover.tick(1.0);
    pose = rover.getCurrentPose();
    EXPECT_NEAR(pose.x, 5.0, 1e-9);
    EXPECT_NEAR(pose.y, -3.0, 1e-9);
}

TEST(RoverModel, SetWheelVelocitiesConvertsToBodyVelocityCorrectly)
{
    RoverModel rover = makeRoverAtOrigin(); // wheelSeparationMeters = 0.5

    // Both wheels at 1 m/s should mean: drive straight, no turning.
    WheelVelocities wheelVelocities;
    wheelVelocities.leftWheelVelocityMetersPerSecond = 1.0;
    wheelVelocities.rightWheelVelocityMetersPerSecond = 1.0;
    rover.setWheelVelocities(wheelVelocities);

    BodyVelocity velocity = rover.getCurrentCommandedVelocity();
    EXPECT_NEAR(velocity.linearVelocityMetersPerSecond, 1.0, 1e-9);
    EXPECT_NEAR(velocity.angularVelocityRadiansPerSecond, 0.0, 1e-9);

    rover.tick(1.0);
    Pose2D pose = rover.getCurrentPose();
    EXPECT_NEAR(pose.x, 1.0, 1e-9);
    EXPECT_NEAR(pose.y, 0.0, 1e-9);
}