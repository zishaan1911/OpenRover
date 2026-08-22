#include "pose2d.hpp"

#include <numbers>

namespace openrover::math {

double normalizeAngleRadians(double angleRadians)
{
    // std::numbers::pi is a standard C++20 constant. We use it instead
    // of the older M_PI macro because M_PI is not part of the C++
    // standard — some compilers (like MSVC) don't define it unless you
    // ask for it in a special, non-portable way.
    constexpr double pi = std::numbers::pi;

    // Nudge the angle down until it's no bigger than pi...
    while (angleRadians > pi)
    {
        angleRadians -= 2.0 * pi;
    }

    // ...and nudge it up until it's bigger than -pi.
    while (angleRadians <= -pi)
    {
        angleRadians += 2.0 * pi;
    }

    return angleRadians;
}

} // namespace openrover::math
