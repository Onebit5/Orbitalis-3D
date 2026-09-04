#pragma once

#include <orbitalis/render/RenderFrame.hpp>

namespace orbitalis::render {

/// A camera that orbits the render origin on a sphere.
///
/// # why the target is always the origin
///
/// It looks like a limitation and is the opposite. `RenderFrame::focus` decides *what sits
/// at the render origin*, so pointing the camera permanently at the origin means "follow
/// that body" is not a camera operation at all: it is setting the frame's focus to the
/// body's position each frame, and the camera never learns anything about it.
///
/// That falls out well for precision too. Following a body puts it at the origin, which is
/// exactly where camera-relative conversion has the most resolution (0.1.2). So the mode
/// where you most want detail is automatically the mode where you get it, without a special
/// case anywhere.
///
/// # conventions
///
/// Right-handed, Y up, matching raylib. Azimuth is measured from +Z toward +X, so azimuth 0
/// puts the camera on +Z and azimuth 90° puts it on +X. Elevation is the angle above the XZ
/// plane.
///
///     x = d · cos(elevation) · sin(azimuth)
///     y = d · sin(elevation)
///     z = d · cos(elevation) · cos(azimuth)
class OrbitCamera
{
public:
    struct Limits
    {
        /// Stops the camera being pushed through whatever it is looking at.
        double min_distance = 0.05;

        /// Stops it being flung so far out that the scene collapses to a point and float
        /// precision in the projection starts to matter.
        double max_distance = 1000.0;

        /// Elevation is clamped just short of the pole. At exactly ±90° the view direction
        /// is parallel to the up vector, the cross product used to build the view basis is
        /// zero, and the camera's roll becomes undefined: the picture flips or vanishes.
        /// Every orbit camera has some version of this clamp.
        double max_elevation_degrees = 89.0;
    };

    OrbitCamera() = default;
    OrbitCamera(double azimuth_degrees, double elevation_degrees, double distance) noexcept;

    /// Adds to the current angles. Elevation clamps, azimuth wraps.
    void rotate(double delta_azimuth_degrees, double delta_elevation_degrees) noexcept;

    /// Multiplies the distance. Values below 1 move closer, above 1 move further away.
    ///
    /// Multiplicative rather than additive on purpose: a scroll notch should mean "10%
    /// closer" at every scale, otherwise zooming is unusably coarse when far out and
    /// unusably slow when close in. A non-positive or non-finite factor is ignored.
    void zoom(double factor) noexcept;

    void set_azimuth_degrees(double degrees) noexcept;
    void set_elevation_degrees(double degrees) noexcept;
    void set_distance(double distance) noexcept;

    /// Camera position in render space, with the target at the origin.
    [[nodiscard]] Vec3f position() const noexcept;

    [[nodiscard]] double azimuth_degrees() const noexcept { return azimuth_; }
    [[nodiscard]] double elevation_degrees() const noexcept { return elevation_; }
    [[nodiscard]] double distance() const noexcept { return distance_; }

    [[nodiscard]] const Limits& limits() const noexcept { return limits_; }

    /// Re-applies the clamps, so changing limits cannot leave the camera outside them.
    void set_limits(const Limits& limits) noexcept;

private:
    Limits limits_{};
    double azimuth_{45.0};
    double elevation_{25.0};
    double distance_{14.0};
};

}  // namespace orbitalis::render
