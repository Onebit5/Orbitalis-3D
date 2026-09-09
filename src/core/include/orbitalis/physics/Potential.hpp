#pragma once

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/Body.hpp>

#include <span>

namespace orbitalis {

/// Gravitational potential at a point, in joules per kilogram.
///
///     Φ(r⃗) = −Σ G·mᵢ / |r⃗ − r⃗ᵢ|
///
/// The potential is the energy per unit mass it would take to haul a test particle from
/// that point out to infinity, which is why it is negative everywhere and rises toward
/// zero as you leave. It is a *scalar* field, unlike the acceleration, so it superposes by
/// simple addition with no directions to keep track of.
///
/// Two things make it worth having:
///
///   - the potential energy term in the total energy at 0.2.3 is built from it, and total
///     energy is how integrator drift gets measured
///   - it is the honest thing to draw for a "gravity well" surface, because the depth of
///     the well at a point *is* the potential there
///
/// `softening` is the same Plummer ε as the force solver, and for the same reason: Φ
/// diverges at a body's centre, so a grid point that lands on one would be −∞. Softened,
///
///     Φ(r⃗) = −Σ G·mᵢ / √(|r⃗ − r⃗ᵢ|² + ε²)
///
/// which is exactly the potential whose gradient gives the softened force. Using the
/// unsoftened potential alongside softened forces would show an energy drift that is pure
/// bookkeeping, as noted at 0.0.4.
[[nodiscard]] double gravitational_potential(std::span<const Body> bodies,
                                             const Vec3& point,
                                             double softening = 0.0) noexcept;

}  // namespace orbitalis
