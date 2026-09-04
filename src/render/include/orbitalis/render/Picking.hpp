#pragma once

#include <orbitalis/math/Vec3.hpp>

#include <cstddef>
#include <optional>
#include <span>

namespace orbitalis::render {

/// Distance along a ray to where it first enters a sphere, or nothing if it misses.
///
/// Substituting the ray `o + t·d` into `|p − c|² = r²` gives a quadratic in t:
///
///     (d·d) t² + 2(oc·d) t + (oc·oc − r²) = 0      where oc = o − c
///
/// A negative discriminant means the ray misses. Otherwise the smaller root is the near
/// surface; if that is behind the ray origin the origin is inside the sphere, and the far
/// root is returned instead so clicking from inside a body still selects it.
///
/// Everything is double even though render space is float, because the arithmetic is cheap
/// here and there is no reason to inherit float's precision for an intermediate result.
[[nodiscard]] std::optional<double> ray_sphere_distance(const Vec3& origin,
                                                        const Vec3& direction,
                                                        const Vec3& centre,
                                                        double radius) noexcept;

/// Index of the first body a ray hits, or nothing.
///
/// `centres` and `radii` are parallel and must be the same length. Radii are the *drawn*
/// radii, not physical ones, which is the point: bodies are drawn with a minimum on-screen
/// size (0.1.3), so passing the drawn radius here means **anything you can see, you can
/// click**. Picking with true radii would make Earth a 0.015-pixel target.
///
/// Ties are broken by distance along the ray, so a body in front of another wins.
[[nodiscard]] std::optional<std::size_t> pick_nearest(const Vec3& origin,
                                                      const Vec3& direction,
                                                      std::span<const Vec3> centres,
                                                      std::span<const double> radii) noexcept;

/// Advances a selection one step: nothing -> 0 -> 1 -> ... -> count-1 -> nothing.
///
/// Three lines of state machine, and it lives here rather than in the viewer's input
/// handling for a specific reason: driving a real key press into a GPU window from a test
/// turns out to be unreliable (only the first synthetic TAB ever arrived), so the only way
/// to actually know this cycles correctly is to make it a pure function and assert on it.
///
/// An out-of-range selection is treated as the start of the cycle rather than clamped,
/// since it can only mean the system changed size underneath the caller.
[[nodiscard]] std::optional<std::size_t> cycle_selection(std::optional<std::size_t> current,
                                                         std::size_t count) noexcept;

}  // namespace orbitalis::render
