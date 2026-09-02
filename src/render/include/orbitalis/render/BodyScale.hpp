#pragma once

namespace orbitalis::render {

/// The world-space radius that covers `pixels` pixels on screen, for something `distance`
/// away from the camera.
///
/// Standard perspective geometry. The viewport half-height corresponds to half the vertical
/// field of view, so an object of radius R at distance d covers
///
///     pixel_radius = R · H / (2 · d · tan(fovy/2))
///
/// and this inverts that for R. It is the bridge between a screen-space intent ("never
/// smaller than three pixels") and the world-space number a draw call actually wants.
///
/// Returns 0 for nonsensical inputs rather than infinity, so the result is always safe to
/// hand to std::max.
[[nodiscard]] double world_radius_for_pixels(double pixels,
                                             double distance,
                                             double vertical_fov_degrees,
                                             int viewport_height) noexcept;

/// Turns physical body radii into sizes a human can actually see.
///
/// # the problem
///
/// Drawing bodies at true scale does not work, and it is not close. With the Sun-Earth
/// system fitted to six render units:
///
///     Sun       radius 6.957e8 m   ->  1.67 pixels
///     Jupiter   radius 6.991e7 m   ->  0.168 pixels
///     Earth     radius 6.371e6 m   ->  0.0153 pixels
///     Ceres     radius 4.730e5 m   ->  0.0011 pixels
///
/// Earth is one sixty-fifth of a pixel. The solar system is overwhelmingly empty space, and
/// an honest picture of it is a black screen. So the radii have to be a lie, and the only
/// question is which lie.
///
/// # the compression
///
/// Multiplying every radius by a constant keeps the proportions honest and does not help:
/// the Sun is 109 times Earth and 1471 times Ceres, so any factor that makes Ceres visible
/// makes the Sun swallow the screen.
///
/// So sizes go through a power law instead,
///
///     render_radius = reference_size · (r / reference_radius) ^ exponent
///
/// with an exponent below one. A cube root turns 1471:1 into 11.4:1 and 109:1 into 4.8:1,
/// which still reads as "the Sun is much bigger than the Earth" while keeping both on
/// screen. An exponent of 1.0 gives true proportions back for anyone who wants them.
///
/// The relationship stays strictly monotonic, so a bigger body is never drawn smaller than
/// a smaller one. That matters more than the exact exponent: it is what keeps the picture
/// honest about ordering even while lying about ratios.
class BodyScale
{
public:
    struct Config
    {
        /// The physical radius that maps exactly to `reference_size`. Earth's, by default,
        /// because it is the body most people have an intuition for.
        double reference_radius = 6.371e6;  // m

        /// Render units a body of `reference_radius` is drawn at.
        double reference_size = 0.08;

        /// Compression exponent. 1.0 preserves true proportions, 1/3 compresses hard.
        double exponent = 1.0 / 3.0;

        /// Draw physical sizes instead. Correct, and almost entirely invisible; useful for
        /// showing exactly how much the normal mode is exaggerating.
        bool true_scale = false;
    };

    BodyScale() = default;
    explicit BodyScale(const Config& config) noexcept;

    /// Radius to draw a body at, in render units.
    ///
    /// `metres_per_unit` comes from the RenderFrame and is only consulted in true-scale
    /// mode, where the answer is just the physical radius converted.
    ///
    /// A zero or negative physical radius returns 0: a point mass has no size, and the
    /// minimum-pixel floor applied by the caller is what keeps it visible.
    [[nodiscard]] double render_radius(double physical_radius_metres,
                                       double metres_per_unit) const noexcept;

    /// How much bigger than reality a body is being drawn. 1.0 means honest.
    ///
    /// Worth surfacing in the HUD: a viewer that silently exaggerates by 200x should say
    /// so, otherwise it is teaching people something false about the solar system.
    [[nodiscard]] double exaggeration(double physical_radius_metres,
                                      double metres_per_unit) const noexcept;

    [[nodiscard]] const Config& config() const noexcept { return config_; }
    void set_config(const Config& config) noexcept;

    [[nodiscard]] bool true_scale() const noexcept { return config_.true_scale; }
    void set_true_scale(bool enabled) noexcept { config_.true_scale = enabled; }

private:
    Config config_{};
};

}  // namespace orbitalis::render
