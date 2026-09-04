#include <orbitalis/render/Picking.hpp>

#include <algorithm>
#include <cmath>

namespace orbitalis::render {

std::optional<double> ray_sphere_distance(const Vec3& origin,
                                          const Vec3& direction,
                                          const Vec3& centre,
                                          double radius) noexcept
{
    if (!(radius > 0.0)) {
        return std::nullopt;
    }

    const double a = direction.length_squared();
    if (!(a > 0.0)) {
        // A zero-length direction is not a ray. Rejecting it here avoids dividing by zero
        // below and returning a NaN that would compare false against everything and be
        // silently treated as a miss anyway, just more confusingly.
        return std::nullopt;
    }

    const Vec3 oc = origin - centre;

    const double half_b = dot(oc, direction);
    const double c = oc.length_squared() - radius * radius;

    // Using half of b throughout: the 2s cancel and it saves a multiply, which is the
    // standard form for this.
    const double discriminant = half_b * half_b - a * c;
    if (discriminant < 0.0) {
        return std::nullopt;
    }

    const double root = std::sqrt(discriminant);

    const double near_t = (-half_b - root) / a;
    if (near_t >= 0.0) {
        return near_t;
    }

    // Near hit is behind us. If the far one is ahead, the origin is inside the sphere.
    const double far_t = (-half_b + root) / a;
    if (far_t >= 0.0) {
        return far_t;
    }

    // Both behind: the sphere is entirely the wrong way.
    return std::nullopt;
}

std::optional<std::size_t> cycle_selection(std::optional<std::size_t> current,
                                           std::size_t count) noexcept
{
    if (count == 0) {
        return std::nullopt;
    }

    // Nothing selected, or an index left over from a smaller system: start the cycle.
    if (!current || *current >= count) {
        return std::size_t{0};
    }

    if (*current + 1 < count) {
        return *current + 1;
    }

    // That was the last body, so the next step is back to no selection.
    return std::nullopt;
}

std::optional<std::size_t> pick_nearest(const Vec3& origin,
                                        const Vec3& direction,
                                        std::span<const Vec3> centres,
                                        std::span<const double> radii) noexcept
{
    const std::size_t count = std::min(centres.size(), radii.size());

    std::optional<std::size_t> best;
    double best_distance = 0.0;

    for (std::size_t i = 0; i < count; ++i) {
        const std::optional<double> hit =
            ray_sphere_distance(origin, direction, centres[i], radii[i]);

        if (!hit) {
            continue;
        }
        if (!best || *hit < best_distance) {
            best = i;
            best_distance = *hit;
        }
    }

    return best;
}

}  // namespace orbitalis::render
