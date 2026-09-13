#pragma once

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/Body.hpp>

#include <span>

namespace orbitalis {

/// Computes gravitational accelerations for a set of bodies.
///
/// This interface exists so that Barnes-Hut can replace brute force at 0.4.0 without any
/// other part of the project noticing. The integrators at 0.3.0 hold one of these and
/// never care which it is.
///
/// **It produces accelerations, not forces.** That is deliberate. Newton gives the force
/// on body i from body j as
///
///     F = G · mᵢ · mⱼ · d⃗ / |d⃗|³
///
/// but every caller immediately wants a = F/mᵢ, and that division cancels the mᵢ that was
/// just multiplied in. Computing acceleration directly skips both operations:
///
///     a⃗ᵢ = G · mⱼ · d⃗ / |d⃗|³
///
/// which is one fewer multiply and one fewer divide per body, on the hottest loop in the
/// project. It also means a massless test particle costs nothing special: its acceleration
/// simply does not depend on its own mass, which is exactly what the equivalence principle
/// says.
///
/// **Takes a span of bodies rather than a System** because the integrators need to
/// evaluate accelerations at *trial* states, not just the current one. RK4 evaluates four
/// times per step at four different sets of positions, none of which are the System's real
/// state.
class IForceSolver
{
public:
    virtual ~IForceSolver() = default;

    /// Writes the acceleration of every body into `accelerations`, overwriting whatever
    /// was there. Only mass and position are read; velocity and radius are ignored,
    /// because Newtonian gravity depends on neither.
    ///
    /// Precondition: `accelerations.size() == bodies.size()`.
    virtual void compute_accelerations(std::span<const Body> bodies,
                                       std::span<Vec3> accelerations) const = 0;

    /// Total gravitational potential energy of this configuration, in joules.
    ///
    /// **This is on the solver, and not a free function, on purpose.** The potential has to
    /// be the one whose gradient is the acceleration *this same object* produces. Any other
    /// potential produces a measured energy drift that is pure bookkeeping: it says nothing
    /// about the integrator and everything about two formulas disagreeing.
    ///
    /// The specific trap is softening. A solver with Plummer ε gives
    ///
    ///     a⃗ = G·mⱼ·d⃗ / (|d⃗|² + ε²)^(3/2)
    ///
    /// whose potential is −G·mᵢ·mⱼ / √(|d⃗|² + ε²), not the Newtonian −G·mᵢ·mⱼ / |d⃗|.
    /// Pairing the wrong one would show a drift that changes when ε changes and stays put
    /// when the integrator changes, which is exactly backwards from what the diagnostics at
    /// 0.2.3 are for. Asking the solver makes the mismatch impossible rather than merely
    /// documented, and I have already once failed to act on a note that said "remember to
    /// match these".
    ///
    /// Sign convention: negative for a bound configuration, approaching zero as the bodies
    /// are separated to infinity. Each unordered pair is counted once.
    ///
    /// Cost is O(n²) for a direct solver, the same as one force evaluation, so this is a
    /// once-per-frame diagnostic rather than a once-per-step one.
    [[nodiscard]] virtual double potential_energy(std::span<const Body> bodies) const = 0;

    /// Short identifier for logs, benchmark tables and the comparison harness at 0.2.6.
    [[nodiscard]] virtual const char* name() const noexcept = 0;

protected:
    // Protected rather than public, and defaulted rather than deleted: derived classes
    // stay copyable, but nobody can slice one through a base reference by accident.
    IForceSolver() = default;
    IForceSolver(const IForceSolver&) = default;
    IForceSolver(IForceSolver&&) = default;
    IForceSolver& operator=(const IForceSolver&) = default;
    IForceSolver& operator=(IForceSolver&&) = default;
};

}  // namespace orbitalis
