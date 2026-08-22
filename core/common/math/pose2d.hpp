#pragma once

namespace openrover::math {

// Pose2D represents where the rover is and which way it is facing,
// in a flat 2D world (we ignore height/tilt for now).
//
// x and y are position coordinates, measured in meters.
// theta is the heading angle, measured in radians.
//   theta = 0             means facing along the positive x-axis.
//   theta increases counter-clockwise (the usual math convention).
struct Pose2D
{
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0;
};

// Keeps an angle inside the range (-pi, pi].
//
// Why this matters: if you keep adding small rotations together over
// many time steps, the raw angle value can grow past 360 degrees (or
// go very negative) even though the rover is physically only ever
// facing one direction at a time. "Wrapping" the angle back into a
// consistent range keeps later comparisons and math well-behaved.
double normalizeAngleRadians(double angleRadians);

} // namespace openrover::math
