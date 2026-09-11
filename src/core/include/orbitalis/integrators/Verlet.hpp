#pragma once

#include <orbitalis/integrators/Integrator.hpp>
#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/ForceSolver.hpp>
#include <orbitalis/physics/System.hpp>

#include <cstddef>
#include <vector>

namespace orbitalis {

// =======================================================================================
// Velocity Verlet. The workhorse.
//
// Second order, symplectic, time-reversible, and still only one force evaluation per step.
// That combination is why essentially every serious n-body code in the world runs some
// version of this, and why the rest of milestone 0.3.0 is largely about measuring what it
// gives up to the fancier methods and what they give up to it.
//
// The update, in the kick-drift-kick form this class implements:
//
//     v⃗ₙ₊½ = v⃗ₙ    + ½ · a⃗ₙ   · dt        kick   (half)
//     x⃗ₙ₊₁ = x⃗ₙ    +      v⃗ₙ₊½ · dt        drift  (full)
//     a⃗ₙ₊₁ = f(x⃗ₙ₊₁)                       the one force evaluation
//     v⃗ₙ₊₁ = v⃗ₙ₊½  + ½ · a⃗ₙ₊₁ · dt        kick   (half)
//
// Substituting the first line into the second gives the form usually written in textbooks,
// and they are the same method:
//
//     x⃗ₙ₊₁ = x⃗ₙ + v⃗ₙ · dt + ½ · a⃗ₙ · dt²
//     v⃗ₙ₊₁ = v⃗ₙ + ½ · (a⃗ₙ + a⃗ₙ₊₁) · dt
//
// # why this is second order when Euler is first
//
// Look at the position update. It carries the dt² term, which is exactly the second term
// of the Taylor expansion of x(t+dt), so the position error per step starts at dt³. Euler
// truncates one term earlier and its per-step error starts at dt². The velocity update is
// a trapezoid rather than a rectangle, which buys the same order there.
//
// In practice: halve the timestep and Verlet's error divides by four where Euler's merely
// halves. Test `test_verlet.cpp` measures that ratio rather than trusting it.
//
// # why it is symplectic and time-reversible
//
// The three lines above are half a kick, a whole drift, and half a kick. Each of those is
// individually an exact solution of half the problem (a drift is what a body does with
// gravity switched off; a kick is what it does with motion switched off), and each is
// therefore individually symplectic. A composition of symplectic maps is symplectic. That
// is the entire argument, and it is why the method conserves a Hamiltonian very close to
// the true one, forever, instead of accumulating energy in one direction.
//
// The symmetry of the composition (½, 1, ½ rather than 1, 1) buys something extra: run the
// method backwards and you retrace the same states. `test_verlet.cpp` checks that by
// stepping forward, negating every velocity, and stepping back to the start.
//
// Both properties are properties of the method **at a fixed timestep**. Vary the step and
// they are gone, which is why RKF45 at 0.2.5 is not symplectic despite being far more
// accurate per step, and why frame delta-time must never reach an integrator.
//
// # the cached acceleration, and the one way to break this class
//
// a⃗ₙ₊₁ is computed at the positions the step ends on, which are the positions the *next*
// step begins on. So it is kept, and the next step's opening kick reuses it. That is what
// makes this one evaluation per step rather than two; without the cache, velocity Verlet
// costs exactly what RK2 costs and is a much worse deal.
//
// The cache is valid only while nothing has moved the bodies behind this object's back.
// Constructing a fresh integrator, calling `reset()`, or changing the number of bodies all
// cause the opening acceleration to be recomputed. Anything else that writes positions or
// masses directly, that swaps in a different force law, or that points the same integrator
// at a different System, must call `reset()`.
//
// Velocities are not on that list, and the omission is deliberate rather than an oversight:
// Newtonian acceleration depends on positions and masses only. Negating every velocity to
// run the system backwards therefore leaves the cache perfectly valid, which is convenient,
// because that is exactly what the time-reversibility test does.
//
// **The failure mode if you forget is not a crash.** It is one step taken with the forces
// from somewhere else, which looks entirely plausible and quietly corrupts the trajectory.
// That is the reason `reset()` went onto the interface at 0.2.1, before anything needed it:
// so that callers were already calling it by the time forgetting mattered.
// =======================================================================================

/// Velocity Verlet, in kick-drift-kick form.
///
/// The default choice for long runs. Second order, symplectic, one force evaluation per
/// step. RK4 at 0.2.4 is more accurate per step and will still lose over a million of them.
class VelocityVerlet final : public IIntegrator
{
public:
    /// The solver must outlive this integrator. Held by pointer rather than reference so
    /// the class stays copyable and assignable.
    explicit VelocityVerlet(const IForceSolver& solver) noexcept : solver_(&solver) {}

    /// Advances every body by `dt` seconds, evaluating the force exactly once.
    ///
    /// The first call after construction or `reset()` evaluates twice: once to prime the
    /// cached opening acceleration, once for the step itself.
    void step(System& system, double dt) override;

    [[nodiscard]] const char* name() const noexcept override { return "velocity-verlet"; }

    /// Second order: halving the timestep divides the error by four.
    [[nodiscard]] int order() const noexcept override { return 2; }

    /// Yes, and at a fixed timestep it stays that way indefinitely.
    [[nodiscard]] bool is_symplectic() const noexcept override { return true; }

    /// Discards the cached acceleration, so the next step recomputes it.
    ///
    /// Call this after anything writes positions or masses without going through
    /// `step()`, or before pointing this integrator at a different System. Velocities do
    /// not need it: acceleration does not depend on them. See the note above on what
    /// happens if you forget.
    void reset() noexcept override { has_cached_acceleration_ = false; }

    /// Whether an opening acceleration is currently cached.
    ///
    /// Exposed for the tests, which need to distinguish "the cache was rebuilt" from "the
    /// cache happened to still be right".
    [[nodiscard]] bool has_cached_acceleration() const noexcept
    {
        return has_cached_acceleration_;
    }

private:
    const IForceSolver* solver_;

    /// a⃗ at the start of the next step, left over from the end of the previous one. Held
    /// as a member rather than reallocated per step: a long run allocates once.
    std::vector<Vec3> accelerations_;

    /// False until `accelerations_` holds something computed at the current positions.
    bool has_cached_acceleration_{false};
};

}  // namespace orbitalis
