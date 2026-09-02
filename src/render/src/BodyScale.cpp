#include <orbitalis/render/BodyScale.hpp>

#include <cmath>
#include <numbers>

namespace orbitalis::render {

namespace {

/// A configuration with a non-positive reference radius or size would divide by zero or
/// raise a negative base to a fractional power. Both are configuration mistakes rather
/// than physics signals, so they get corrected the same way RenderFrame handles a bad
/// scale.
BodyScale::Config sanitise(BodyScale::Config config) noexcept
{
    if (!(config.reference_radius > 0.0) || !std::isfinite(config.reference_radius)) {
        config.reference_radius = 6.371e6;
    }
    if (!(config.reference_size > 0.0) || !std::isfinite(config.reference_size)) {
        config.reference_size = 0.08;
    }
    if (!(config.exponent > 0.0) || !std::isfinite(config.exponent)) {
        config.exponent = 1.0 / 3.0;
    }
    return config;
}

}  // namespace

double world_radius_for_pixels(double pixels,
                               double distance,
                               double vertical_fov_degrees,
                               int viewport_height) noexcept
{
    if (!(pixels > 0.0) || !(distance > 0.0) || viewport_height <= 0
        || !(vertical_fov_degrees > 0.0) || !(vertical_fov_degrees < 180.0)) {
        return 0.0;
    }

    const double half_fov = 0.5 * vertical_fov_degrees * std::numbers::pi / 180.0;

    // pixel_radius = R · H / (2 · d · tan(fovy/2)), solved for R.
    return pixels * 2.0 * distance * std::tan(half_fov) / static_cast<double>(viewport_height);
}

BodyScale::BodyScale(const Config& config) noexcept
    : config_(sanitise(config))
{
}

void BodyScale::set_config(const Config& config) noexcept
{
    config_ = sanitise(config);
}

double BodyScale::render_radius(double physical_radius_metres,
                                double metres_per_unit) const noexcept
{
    if (!(physical_radius_metres > 0.0) || !std::isfinite(physical_radius_metres)) {
        return 0.0;
    }

    if (config_.true_scale) {
        if (!(metres_per_unit > 0.0) || !std::isfinite(metres_per_unit)) {
            return 0.0;
        }
        return physical_radius_metres / metres_per_unit;
    }

    // Deliberately independent of metres_per_unit. Because RenderFrame::fit_scale
    // normalises every scenario to roughly the same render extent, a fixed size in render
    // units means a body looks the same whether it is a planet or a star cluster member.
    // Zooming is a camera movement, and perspective handles the apparent size for free.
    const double ratio = physical_radius_metres / config_.reference_radius;
    return config_.reference_size * std::pow(ratio, config_.exponent);
}

double BodyScale::exaggeration(double physical_radius_metres,
                               double metres_per_unit) const noexcept
{
    if (!(physical_radius_metres > 0.0) || !(metres_per_unit > 0.0)) {
        return 1.0;
    }

    const double honest = physical_radius_metres / metres_per_unit;
    if (!(honest > 0.0)) {
        return 1.0;
    }

    return render_radius(physical_radius_metres, metres_per_unit) / honest;
}

}  // namespace orbitalis::render
