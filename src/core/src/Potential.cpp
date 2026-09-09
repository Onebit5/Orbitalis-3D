#include <orbitalis/physics/Potential.hpp>

#include <orbitalis/physics/Constants.hpp>

#include <cmath>

namespace orbitalis {

double gravitational_potential(std::span<const Body> bodies,
                               const Vec3& point,
                               double softening) noexcept
{
    const double softening_squared = softening * softening;

    double total = 0.0;

    for (const Body& body : bodies) {
        const double d2 = distance_squared(point, body.position) + softening_squared;

        if (!(d2 > 0.0)) {
            // Only reachable with zero softening and a point exactly on a body. Skipping
            // that term is the least bad option: returning -infinity would poison every
            // vertex downstream of a single unlucky grid sample.
            continue;
        }

        total -= kGravitationalConstant * body.mass / std::sqrt(d2);
    }

    return total;
}

}  // namespace orbitalis
