#include "pose2d.hpp"

#include <numbers>

namespace openrover::math {

double normalizeAngleRadians(double angleRadians)
{
    constexpr double pi = std::numbers::pi;

    // Nudge the angle down until it's no bigger than pi
    while (angleRadians > pi)
    {
        angleRadians -= 2.0 * pi;
    }

    // and nudge it up until it's bigger than -pi.
    while (angleRadians <= -pi)
    {
        angleRadians += 2.0 * pi;
    }

    return angleRadians;
}

} // namespace openrover::math
