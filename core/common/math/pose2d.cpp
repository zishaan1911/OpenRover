#include "pose2d.hpp"

#include <cmath>

namespace openrover::math {

double normalizeAngleRadians(double angleRadians)
{
    // Nudge the angle down until it's no bigger than pi...
    while (angleRadians > M_PI)
    {
        angleRadians -= 2.0 * M_PI;
    }

    // ...and nudge it up until it's bigger than -pi.
    while (angleRadians <= -M_PI)
    {
        angleRadians += 2.0 * M_PI;
    }

    return angleRadians;
}

} // namespace openrover::math
