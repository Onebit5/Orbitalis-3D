#pragma once

#include <orbitalis/physics/Body.hpp>
#include <orbitalis/render/RenderFrame.hpp>

#include <span>
#include <vector>

namespace orbitalis::render {

/// A grid of the gravitational potential in the orbital plane, displaced downward into a
/// well. The rubber-sheet picture, done honestly.
///
/// # what it actually shows
///
/// The height of each vertex is the Newtonian gravitational potential Φ at that point,
/// which is a real scalar field with units of energy per kilogram: the work per unit mass
/// needed to escape from there to infinity. So the depth of the well is not a metaphor, it
/// is a measured quantity, and the slope of the surface is proportional to the acceleration
/// a body at that point would feel.
///
/// That last part is what makes it explain something. A planet orbits because it is moving
/// sideways fast enough to keep climbing the wall as fast as it falls; an asteroid that is
/// not spirals in. The steepness you can see *is* the pull.
///
/// # what it does not show
///
/// This is a Newtonian potential, not curved spacetime. The usual criticism of the
/// rubber-sheet analogy is that it explains gravity using gravity, since it relies on your
/// intuition that things roll downhill. That criticism is fair when the picture is sold as
/// general relativity. As a plot of Φ it is simply correct, and honest about being a plot.
///
/// The vertical axis is also not a spatial dimension. It is a scalar field drawn as height,
/// the same way a contour map draws elevation. Nothing physically moves up or down.
class PotentialSurface
{
public:
    struct Config
    {
        /// Vertices per side. The cost is resolution² potential evaluations per update,
        /// each of which is O(bodies), so this is the knob that matters for performance.
        int resolution = 56;

        /// Half-width of the sheet, in render units.
        double extent_units = 14.0;

        /// How far down the deepest point of the well is drawn, in render units.
        double depth_units = 3.0;

        /// Plummer softening for the sampling, in render units.
        ///
        /// Φ diverges at a body's centre, so without this a grid point landing near one
        /// would drag the whole normalisation to itself and flatten everything else to
        /// nothing. It is the same fudge as the force solver's, and it means the very
        /// bottom of each well is drawn shallower than it truly is.
        double softening_units = 0.35;
    };

    PotentialSurface() = default;
    explicit PotentialSurface(const Config& config);

    /// Resamples the field. Call when the bodies or the frame have moved.
    void update(std::span<const Body> bodies, const RenderFrame& frame);

    /// Vertex at grid coordinates (i, j), in render space, with the well in Y.
    ///
    /// Precondition: both indices are below resolution().
    [[nodiscard]] const Vec3f& vertex(int i, int j) const noexcept;

    /// Raw potential at (i, j), in J/kg, before it was scaled into a depth.
    [[nodiscard]] double potential(int i, int j) const noexcept;

    [[nodiscard]] int resolution() const noexcept { return config_.resolution; }

    /// Most negative potential found in the last update, which the depth scale is
    /// normalised against. Zero if there was nothing to sample.
    [[nodiscard]] double deepest() const noexcept { return deepest_; }

    [[nodiscard]] const Config& config() const noexcept { return config_; }
    void set_config(const Config& config);

private:
    Config config_{};
    std::vector<Vec3f> vertices_;
    std::vector<double> potentials_;
    double deepest_{0.0};
};

}  // namespace orbitalis::render
