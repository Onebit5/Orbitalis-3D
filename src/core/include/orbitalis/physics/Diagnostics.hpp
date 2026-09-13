#pragma once

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/Body.hpp>
#include <orbitalis/physics/ForceSolver.hpp>
#include <orbitalis/physics/System.hpp>

#include <span>

namespace orbitalis {

// =======================================================================================
// The quantities that are supposed to stay constant, and therefore the only honest way to
// tell whether an integrator is working.
//
// Up to here I have been checking integrators by looking at them. An orbit that comes back
// to roughly where it started looks correct, and forward Euler's spiral is obvious, but
// "roughly" is doing an enormous amount of work in that sentence and the eye cannot tell
// 1e-8 from 1e-4. Worse, the eye cannot tell a *bounded* error from a slowly growing one,
// and that distinction is the entire point of milestone 0.3.0.
//
// Three conserved quantities, and they fail in different ways, which is what makes having
// all three worth the trouble:
//
//   energy            conserved by the *integrator*, or not. this is the one that
//                     distinguishes methods. symplectic methods keep the error bounded,
//                     everything else drifts one way forever
//
//   linear momentum   conserved by the *force solver*, via Newton's third law inside the
//                     pair loop. an integrator essentially cannot break it, so a momentum
//                     drift above roundoff means the force law is wrong, not the method
//
//   angular momentum  conserved by the force solver and the integrator *jointly*, which is
//                     the one I got wrong and had to be corrected by a failing test. zero
//                     net torque is what makes the continuous system conserve it, and that
//                     is the solver's doing: gravity is central, so each pair's torques
//                     cancel. but whether a discrete step preserves it depends on the order
//                     the position and velocity updates happen in.
//
//                     semi-implicit Euler and velocity Verlet both conserve it exactly,
//                     because r⃗ₙ₊₁ × v⃗ₙ₊₁ collapses to r⃗ₙ × v⃗ₙ₊₁ when the position update
//                     uses the new velocity, leaving a torque term that is zero. forward
//                     Euler does not: its cross terms leave dt²·Σ mᵢ(v⃗ᵢ × a⃗ᵢ), which is not
//                     a torque and does not vanish. measured, it loses 55% of L over twenty
//                     orbits while its linear momentum stays at 3e-15.
//
//                     so a violation means either the force stopped being central, which is
//                     what a Barnes-Hut bug at 0.4.0 will look like, or the integrator's
//                     update ordering is wrong.
//
// So they localise a fault rather than just detecting one. Linear momentum accuses the
// solver, energy accuses the integrator, and angular momentum can accuse either, which is
// why all three are here instead of just the energy the milestone asked for.
// =======================================================================================

/// Total kinetic energy, J.  T = Σ ½·mᵢ·|v⃗ᵢ|²
///
/// Frame dependent, unlike the potential. A system drifting through space carries extra
/// kinetic energy that has nothing to do with its internal dynamics, which is one more
/// reason `remove_net_drift()` is worth calling on hand-written scenarios.
[[nodiscard]] double kinetic_energy(std::span<const Body> bodies) noexcept;

/// Total angular momentum about `origin`, kg·m²/s.  L⃗ = Σ mᵢ·(r⃗ᵢ − origin) × v⃗ᵢ
///
/// For an isolated system this is conserved about any fixed origin, and separately
/// conserved about the instantaneous barycentre even though the barycentre is moving. The
/// two are equal only when the total linear momentum is zero: in general they differ by
/// R⃗ × P⃗, the angular momentum the system's bulk motion carries about that origin.
///
/// `measure()` uses the barycentre, so the number does not change if you view the same
/// physics from a different inertial frame.
[[nodiscard]] Vec3 angular_momentum(std::span<const Body> bodies,
                                    const Vec3& origin = Vec3{}) noexcept;

/// Everything worth watching, measured in one place.
struct Diagnostics
{
    double kinetic{};    ///< T, J
    double potential{};  ///< U, J. Negative for a bound system.
    double total{};      ///< E = T + U, J. Negative means bound.

    Vec3 momentum{};          ///< P⃗, kg·m/s
    Vec3 angular_momentum{};  ///< L⃗ about the barycentre, kg·m²/s

    Vec3 center_of_mass{};           ///< R⃗, m
    Vec3 center_of_mass_velocity{};  ///< Ṙ⃗, m/s. Constant for an isolated system.

    /// Largest |mᵢ·v⃗ᵢ| of any single body, kg·m/s.
    ///
    /// Carried because it is the only sensible denominator for a momentum drift. Total
    /// momentum is usually zero by construction, so a *relative* drift has nothing to
    /// divide by, and an absolute figure at solar system masses is a huge number that means
    /// nothing on its own. The largest single term is the size of the cancellation being
    /// asked for, so drift measured against it says how well that cancellation held.
    double largest_body_momentum{};

    /// Largest |mᵢ·r⃗ᵢ × v⃗ᵢ| about the barycentre. Same reasoning as above.
    double largest_body_angular_momentum{};
};

/// Measures everything in the struct above.
///
/// The potential comes from the solver rather than from a formula here, so it is always the
/// one whose gradient is the force actually being integrated. See `IForceSolver`.
///
/// **Cost is O(n²)**, dominated by the potential, which is the same order as a single force
/// evaluation. That makes this a per-frame diagnostic and not a per-step one: calling it
/// every step would roughly double the cost of a run to measure it.
[[nodiscard]] Diagnostics measure(const System& system, const IForceSolver& solver);

/// Tracks how far the conserved quantities have moved since a baseline.
///
/// Held separately from `Diagnostics` because "what is true now" and "how far has it
/// wandered" are different questions, and the second one needs somewhere to remember.
///
/// # on sampling
///
/// The viewer samples this once per frame while running many steps per frame, so
/// `worst_relative_energy_error()` is the worst of what was *looked at*, not the worst that
/// happened. It is a lower bound. That is an acceptable trade for a live display, and it is
/// stated rather than hidden because a diagnostic that quietly under-reports is worse than
/// no diagnostic at all. The tests sample every step, where it is exact.
class ConservationMonitor
{
public:
    /// Captures a new baseline and clears the history.
    ///
    /// Call this whenever the comparison stops meaning anything: a scenario reload, a
    /// rewind, or a change of integrator. Continuing to measure a new method against the
    /// old method's starting energy would attribute one method's drift to another.
    void reset(const System& system, const IForceSolver& solver);

    /// Measures the current state and folds it into the running worst case.
    ///
    /// Takes a baseline automatically on the first call, so a caller that forgets to
    /// `reset()` gets a sensible answer rather than a comparison against zero.
    void sample(const System& system, const IForceSolver& solver);

    [[nodiscard]] bool has_baseline() const noexcept { return has_baseline_; }
    [[nodiscard]] int samples() const noexcept { return samples_; }

    [[nodiscard]] const Diagnostics& baseline() const noexcept { return baseline_; }
    [[nodiscard]] const Diagnostics& latest() const noexcept { return latest_; }

    /// (E − E₀) / |E₀|, **signed**.
    ///
    /// Signed rather than absolute, and divided by |E₀| rather than E₀, because a bound
    /// system has E₀ < 0 and dividing by it flips the sign: an energy *gain* would report
    /// as a loss. Positive here always means the integrator is pumping energy in, which for
    /// a bound orbit means it is climbing outward.
    ///
    /// Worth knowing what this number cannot do: it saturates. A method that unbinds the
    /// system drives E toward zero, so the magnitude flattens out near 1 no matter how
    /// wrong the trajectory has become. Fine for spotting drift, useless for measuring how
    /// bad it got. Zero if there is no baseline yet, or if E₀ was exactly zero.
    [[nodiscard]] double relative_energy_error() const noexcept;

    /// Largest |relative_energy_error()| across every sample taken since the baseline.
    ///
    /// This is the number that separates a symplectic method from a merely accurate one: it
    /// stops growing for the first and does not for the second.
    [[nodiscard]] double worst_relative_energy_error() const noexcept { return worst_energy_; }

    /// |P⃗ − P⃗₀| scaled by the largest single body momentum. Dimensionless.
    ///
    /// Expect this at roundoff, around 1e-14, for any integrator. It is a property of the
    /// force solver, so anything larger means the pair loop is broken rather than the
    /// method.
    [[nodiscard]] double relative_momentum_drift() const noexcept;

    /// |L⃗ − L⃗₀| scaled the same way. Also expected at roundoff.
    [[nodiscard]] double relative_angular_momentum_drift() const noexcept;

private:
    Diagnostics baseline_{};
    Diagnostics latest_{};
    double worst_energy_{};
    int samples_{};
    bool has_baseline_{false};
};

}  // namespace orbitalis
