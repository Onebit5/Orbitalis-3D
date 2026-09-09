#include <orbitalis/render/PotentialSurface.hpp>

#include <orbitalis/physics/Potential.hpp>

#include <algorithm>
#include <cmath>

namespace orbitalis::render {

namespace {

PotentialSurface::Config sanitise(PotentialSurface::Config config) noexcept
{
    config.resolution = std::clamp(config.resolution, 2, 512);

    if (!(config.extent_units > 0.0) || !std::isfinite(config.extent_units)) {
        config.extent_units = 14.0;
    }
    if (!(config.depth_units > 0.0) || !std::isfinite(config.depth_units)) {
        config.depth_units = 3.0;
    }
    if (!(config.softening_units >= 0.0) || !std::isfinite(config.softening_units)) {
        config.softening_units = 0.35;
    }
    return config;
}

}  // namespace

PotentialSurface::PotentialSurface(const Config& config)
    : config_(sanitise(config))
{
}

void PotentialSurface::set_config(const Config& config)
{
    config_ = sanitise(config);
    vertices_.clear();
    potentials_.clear();
    deepest_ = 0.0;
}

void PotentialSurface::update(std::span<const Body> bodies, const RenderFrame& frame)
{
    const int n = config_.resolution;
    const auto count = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);

    vertices_.assign(count, Vec3f{});
    potentials_.assign(count, 0.0);
    deepest_ = 0.0;

    if (bodies.empty()) {
        // Leave a flat sheet rather than an empty one, so the viewer has something to draw
        // and does not need a special case.
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                const double u = static_cast<double>(i) / (n - 1) * 2.0 - 1.0;
                const double v = static_cast<double>(j) / (n - 1) * 2.0 - 1.0;
                vertices_[static_cast<std::size_t>(j) * n + i] =
                    Vec3f{static_cast<float>(u * config_.extent_units), 0.0f,
                          static_cast<float>(v * config_.extent_units)};
            }
        }
        return;
    }

    const double softening_metres = config_.softening_units * frame.metres_per_unit();

    // First pass: sample the field. The sheet lies in the render XZ plane, so each grid
    // point is converted back into simulation space at render height zero, which is the
    // orbital plane by the axis convention in RenderFrame.
    for (int j = 0; j < n; ++j) {
        for (int i = 0; i < n; ++i) {
            const double u = static_cast<double>(i) / (n - 1) * 2.0 - 1.0;
            const double v = static_cast<double>(j) / (n - 1) * 2.0 - 1.0;

            const Vec3f render_point{static_cast<float>(u * config_.extent_units), 0.0f,
                                     static_cast<float>(v * config_.extent_units)};

            const double phi = gravitational_potential(bodies, frame.to_simulation(render_point),
                                                       softening_metres);

            const std::size_t index = static_cast<std::size_t>(j) * n + i;
            potentials_[index] = phi;
            deepest_ = std::min(deepest_, phi);

            vertices_[index] = render_point;
        }
    }

    // Second pass: normalise into a depth.
    //
    // Normalising against the deepest sample rather than against a fixed constant means the
    // sheet looks right whether this is a solar system or a star cluster, for the same
    // reason fit_scale derives the world unit from the data. It also means the depth axis
    // has no absolute meaning, only relative: the well is drawn at full depth whether the
    // central mass is a planet or a black hole.
    if (!(deepest_ < 0.0)) {
        return;
    }

    for (std::size_t index = 0; index < count; ++index) {
        const double fraction = potentials_[index] / deepest_;  // 0 far away, 1 at the deepest
        vertices_[index].y = static_cast<float>(-config_.depth_units * fraction);
    }
}

const Vec3f& PotentialSurface::vertex(int i, int j) const noexcept
{
    return vertices_[static_cast<std::size_t>(j) * config_.resolution + i];
}

double PotentialSurface::potential(int i, int j) const noexcept
{
    return potentials_[static_cast<std::size_t>(j) * config_.resolution + i];
}

}  // namespace orbitalis::render
