#include <orbitalis/render/OrbitCamera.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace orbitalis::render {

namespace {

constexpr double kDegreesToRadians = std::numbers::pi / 180.0;

/// Brings any angle into [0, 360). std::fmod alone is not enough: it keeps the sign of the
/// dividend, so fmod(-10, 360) is -10 rather than 350.
double wrap_degrees(double degrees) noexcept
{
    if (!std::isfinite(degrees)) {
        return 0.0;
    }

    double wrapped = std::fmod(degrees, 360.0);
    if (wrapped < 0.0) {
        wrapped += 360.0;
    }
    return wrapped;
}

double sane(double value, double fallback) noexcept
{
    return std::isfinite(value) ? value : fallback;
}

}  // namespace

OrbitCamera::OrbitCamera(double azimuth_degrees,
                         double elevation_degrees,
                         double distance) noexcept
{
    set_azimuth_degrees(azimuth_degrees);
    set_elevation_degrees(elevation_degrees);
    set_distance(distance);
}

void OrbitCamera::rotate(double delta_azimuth_degrees, double delta_elevation_degrees) noexcept
{
    set_azimuth_degrees(azimuth_ + sane(delta_azimuth_degrees, 0.0));
    set_elevation_degrees(elevation_ + sane(delta_elevation_degrees, 0.0));
}

void OrbitCamera::zoom(double factor) noexcept
{
    if (!(factor > 0.0) || !std::isfinite(factor)) {
        return;
    }
    set_distance(distance_ * factor);
}

void OrbitCamera::set_azimuth_degrees(double degrees) noexcept
{
    // Wrapped rather than clamped: spinning all the way round is a legitimate thing to do.
    azimuth_ = wrap_degrees(degrees);
}

void OrbitCamera::set_elevation_degrees(double degrees) noexcept
{
    // Clamped rather than wrapped: passing over the pole would flip the world upside down
    // mid-drag, which is disorienting even when the maths survives it.
    const double limit = std::abs(limits_.max_elevation_degrees);
    elevation_ = std::clamp(sane(degrees, 0.0), -limit, limit);
}

void OrbitCamera::set_distance(double distance) noexcept
{
    distance_ = std::clamp(sane(distance, limits_.min_distance),
                           limits_.min_distance,
                           limits_.max_distance);
}

void OrbitCamera::set_limits(const Limits& limits) noexcept
{
    limits_ = limits;

    if (!(limits_.min_distance > 0.0) || !std::isfinite(limits_.min_distance)) {
        limits_.min_distance = 0.05;
    }
    if (!(limits_.max_distance > limits_.min_distance) || !std::isfinite(limits_.max_distance)) {
        limits_.max_distance = limits_.min_distance * 1000.0;
    }
    if (!(limits_.max_elevation_degrees > 0.0) || limits_.max_elevation_degrees >= 90.0) {
        limits_.max_elevation_degrees = 89.0;
    }

    // Re-apply, so new limits cannot leave the camera somewhere they forbid.
    set_elevation_degrees(elevation_);
    set_distance(distance_);
}

Vec3f OrbitCamera::position() const noexcept
{
    const double azimuth = azimuth_ * kDegreesToRadians;
    const double elevation = elevation_ * kDegreesToRadians;

    const double horizontal = distance_ * std::cos(elevation);

    return Vec3f{static_cast<float>(horizontal * std::sin(azimuth)),
                 static_cast<float>(distance_ * std::sin(elevation)),
                 static_cast<float>(horizontal * std::cos(azimuth))};
}

}  // namespace orbitalis::render
