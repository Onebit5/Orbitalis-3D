#include <orbitalis/render/RenderFrame.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace orbitalis::render {

namespace {

/// Guards against a zero or negative scale, which would divide by nothing and put NaN
/// into every vertex. A bad scale is a configuration mistake rather than a physics
/// signal, so unlike Vec3::normalized() it gets corrected rather than left loud.
double sanitise_scale(double metres_per_unit) noexcept
{
    return (metres_per_unit > 0.0 && std::isfinite(metres_per_unit)) ? metres_per_unit : 1.0;
}

}  // namespace

RenderFrame::RenderFrame(const Vec3& focus, double metres_per_unit) noexcept
    : focus_(focus)
    , metres_per_unit_(sanitise_scale(metres_per_unit))
{
}

void RenderFrame::set_metres_per_unit(double metres_per_unit) noexcept
{
    metres_per_unit_ = sanitise_scale(metres_per_unit);
}

Vec3f RenderFrame::to_render(const Vec3& simulation_position) const noexcept
{
    // The whole point of this file is the order of these two lines. Subtract first, in
    // double, so the large common component cancels exactly. Only then narrow.
    const Vec3 relative = simulation_position - focus_;
    const Vec3 scaled = relative / metres_per_unit_;

    return Vec3f{static_cast<float>(scaled.x),
                 static_cast<float>(scaled.y),
                 static_cast<float>(scaled.z)};
}

Vec3 RenderFrame::to_simulation(const Vec3f& render_position) const noexcept
{
    // Widening back is lossless, so this recovers whatever survived the trip out. It does
    // not recover what to_render already discarded, and cannot.
    return Vec3{static_cast<double>(render_position.x) * metres_per_unit_ + focus_.x,
                static_cast<double>(render_position.y) * metres_per_unit_ + focus_.y,
                static_cast<double>(render_position.z) * metres_per_unit_ + focus_.z};
}

Vec3f RenderFrame::to_render_naive(const Vec3& simulation_position) const noexcept
{
    // Scale and narrow first, subtract afterwards, entirely in float. This is what you get
    // by writing the obvious thing, and it is wrong by tens of kilometres at solar-system
    // scale. Kept only so the tests can measure the damage.
    const auto sx = static_cast<float>(simulation_position.x / metres_per_unit_);
    const auto sy = static_cast<float>(simulation_position.y / metres_per_unit_);
    const auto sz = static_cast<float>(simulation_position.z / metres_per_unit_);

    const auto fx = static_cast<float>(focus_.x / metres_per_unit_);
    const auto fy = static_cast<float>(focus_.y / metres_per_unit_);
    const auto fz = static_cast<float>(focus_.z / metres_per_unit_);

    return Vec3f{sx - fx, sy - fy, sz - fz};
}

double RenderFrame::resolution_at(double render_distance) const noexcept
{
    const auto d = static_cast<float>(std::abs(render_distance));

    // Measure the actual gap to the next representable float rather than assuming
    // d * 2^-23. That assumption is only right within a binade, and it is wrong by up to
    // a factor of two elsewhere.
    const float next = std::nextafter(d, std::numeric_limits<float>::infinity());
    const double ulp_units = static_cast<double>(next) - static_cast<double>(d);

    return ulp_units * metres_per_unit_;
}

double RenderFrame::fit_scale(std::span<const Body> bodies,
                              const Vec3& focus,
                              double target_units) noexcept
{
    if (bodies.empty() || !(target_units > 0.0)) {
        return 1.0;
    }

    double furthest_squared = 0.0;
    for (const Body& b : bodies) {
        furthest_squared = std::max(furthest_squared, distance_squared(b.position, focus));
    }

    if (furthest_squared <= 0.0) {
        // Every body sits exactly on the focus. Any scale is as good as any other, so
        // return one that cannot cause a division problem downstream.
        return 1.0;
    }

    return std::sqrt(furthest_squared) / target_units;
}

}  // namespace orbitalis::render
