#pragma once

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/Body.hpp>

#include <span>

namespace orbitalis::render {

/// A 3D vector of floats, for handing to the GPU.
///
/// This is the *only* place floats are allowed to appear in a position. Everything
/// upstream of here is double, for the reason established at 0.0.2: a float cannot resolve
/// Earth's orbital radius to better than about 16 km, which is further than Earth travels
/// in one timestep.
struct Vec3f
{
    float x{};
    float y{};
    float z{};
};

/// Converts simulation coordinates (metres, double, ~1e11) into render coordinates
/// (world units, float, ~1-100).
///
/// # why this is a whole class instead of a divide
///
/// OpenGL is a 32-bit float pipeline the whole way down: vertex positions, the model-view
/// matrix, the depth buffer. So somewhere a double has to become a float, and *where* that
/// happens decides whether the picture holds together when you zoom in.
///
/// The naive conversion scales first and subtracts later, effectively
///
///     render = float(position / scale) - float(focus / scale)
///
/// At solar-system scale both operands are near 1.0, where a float's spacing is 2^-23,
/// about 1.2e-7 world units. Multiply back up by an AU and that is **17.8 km**. Anything
/// closer together than that collapses to the same point. A spacecraft a kilometre from
/// the Earth renders exactly on top of it.
///
/// Doing the subtraction first, in double, fixes it:
///
///     render = float((position - focus) / scale)
///
/// The quantity being narrowed is no longer ~1.5e11, it is the offset from the camera,
/// which for anything you can actually see is small. That offset then gets the *relative*
/// precision of a float, roughly seven significant digits of whatever it happens to be,
/// rather than seven significant digits of an astronomical unit.
///
/// This is the same principle as computing `length_squared` instead of squaring a length,
/// and as dividing rather than multiplying by a reciprocal: **do the cancellation in the
/// wide type and narrow as late as possible.**
///
/// The trade is that precision now degrades with distance *from the camera* rather than
/// from the origin. That is exactly the right way round, because distant things are small
/// on screen and their error is subpixel.
class RenderFrame
{
public:
    RenderFrame() = default;

    /// `focus` is the point in simulation space that maps to the render origin, normally
    /// wherever the camera is looking. `metres_per_unit` is how many metres one render
    /// unit represents.
    RenderFrame(const Vec3& focus, double metres_per_unit) noexcept;

    /// Simulation position to render position. This is the one the viewer calls.
    [[nodiscard]] Vec3f to_render(const Vec3& simulation_position) const noexcept;

    /// Render position back to simulation space. Needed for click-to-select at 0.1.4,
    /// where a ray has to be turned back into metres.
    [[nodiscard]] Vec3 to_simulation(const Vec3f& render_position) const noexcept;

    /// The same conversion done the wrong way round, scaling before subtracting.
    ///
    /// Nothing in the viewer calls this. It exists so the tests can measure what the naive
    /// ordering actually costs rather than asserting that it is bad, and so that the
    /// difference is a number in a test file instead of a claim in a comment.
    [[nodiscard]] Vec3f to_render_naive(const Vec3& simulation_position) const noexcept;

    [[nodiscard]] const Vec3& focus() const noexcept { return focus_; }
    void set_focus(const Vec3& focus) noexcept { focus_ = focus; }

    [[nodiscard]] double metres_per_unit() const noexcept { return metres_per_unit_; }
    void set_metres_per_unit(double metres_per_unit) noexcept;

    /// How many metres one float ULP covers at the given distance from the render origin.
    ///
    /// This is the honest answer to "how precise is the picture here". At 1 render unit it
    /// is metres_per_unit * 2^-23; it doubles every time the distance doubles, because
    /// that is how floating point works.
    [[nodiscard]] double resolution_at(double render_distance) const noexcept;

    /// Picks a scale that fits every body within `target_units` of the focus.
    ///
    /// The answer to the question I left open at 0.1.1: what should a world unit *be*? an
    /// AU is the obvious choice for a planetary system and useless for a cluster spanning
    /// parsecs. So do not choose. Derive it from the data, and let a scenario of any size
    /// arrive on screen at a sensible size.
    ///
    /// Returns 1.0 for an empty span or a system with no extent, so the result is always
    /// safe to divide by.
    [[nodiscard]] static double fit_scale(std::span<const Body> bodies,
                                          const Vec3& focus,
                                          double target_units = 10.0) noexcept;

private:
    Vec3 focus_{};
    double metres_per_unit_{1.0};
};

}  // namespace orbitalis::render
