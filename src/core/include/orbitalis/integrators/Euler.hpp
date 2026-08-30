#pragma once

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/ForceSolver.hpp>
#include <orbitalis/physics/System.hpp>

#include <vector>

namespace orbitalis {

// =======================================================================================
// Two integrators that differ by one word, and behave completely differently.
//
// Both advance the system by a fixed timestep using nothing but the current acceleration.
// Given a⃗ = f(x⃗), the update is:
//
//   forward Euler          x⃗ₙ₊₁ = x⃗ₙ + v⃗ₙ · dt          <- OLD velocity
//                          v⃗ₙ₊₁ = v⃗ₙ + a⃗ₙ · dt
//
//   semi-implicit Euler    v⃗ₙ₊₁ = v⃗ₙ + a⃗ₙ · dt
//                          x⃗ₙ₊₁ = x⃗ₙ + v⃗ₙ₊₁ · dt        <- NEW velocity
//
// That is the entire difference: which velocity the position update uses. Both are first
// order, both cost exactly one force evaluation per step, and they take the same number of
// instructions.
//
// Semi-implicit Euler is *symplectic*: it exactly conserves a quantity very close to the
// true energy, so its error oscillates instead of accumulating. Forward Euler is not, and
// systematically injects energy into a bound orbit until the body escapes.
//
// The measured difference on a circular orbit at 1 AU with dt = 1 day, after one orbit:
//
//   forward Euler        radius 1.000 AU -> 1.206 AU   (spirals out 20% per orbit)
//   semi-implicit Euler  radius 1.000 AU -> 1.000038 AU
//
// Both live here on purpose. Forward Euler is the control group that milestone 0.3.0 needs
// something to beat. Semi-implicit is here because when 0.0.6 asks "did Earth come back to
// where it started", a second integrator is what distinguishes *the method is bad* (which
// is expected) from *the force solver is wrong* (which is not). One of them costs 15 lines
// and answers that question immediately.
// =======================================================================================

/// Forward (explicit) Euler. The bad one, deliberately.
///
/// Do not use this for anything you care about. It is here as a baseline, and because
/// seeing an orbit spiral outward is a much better argument for symplectic integrators
/// than reading that they are better.
class ForwardEuler
{
public:
    /// The solver must outlive this integrator. Held by pointer rather than reference so
    /// the class stays copyable and assignable.
    explicit ForwardEuler(const IForceSolver& solver) noexcept : solver_(&solver) {}

    /// Advances every body by `dt` seconds.
    void step(System& system, double dt);

    [[nodiscard]] const char* name() const noexcept { return "forward-euler"; }

private:
    const IForceSolver* solver_;

    /// Scratch. Owned by the integrator, not by System or Body: settled at 0.0.3, on the
    /// grounds that different methods need different numbers of these (RK4 needs four).
    /// Kept as a member so a long run does not allocate once per step.
    std::vector<Vec3> accelerations_;
};

/// Semi-implicit (symplectic) Euler, also called Euler-Cromer.
///
/// Same cost as forward Euler, dramatically better on orbits. Still only first order, so
/// velocity Verlet at 0.2.2 will beat it, but it is a genuinely usable method rather than
/// a cautionary tale.
class SemiImplicitEuler
{
public:
    explicit SemiImplicitEuler(const IForceSolver& solver) noexcept : solver_(&solver) {}

    void step(System& system, double dt);

    [[nodiscard]] const char* name() const noexcept { return "semi-implicit-euler"; }

private:
    const IForceSolver* solver_;
    std::vector<Vec3> accelerations_;
};

// No IIntegrator base class yet. That arrives at 0.2.1, when there are enough methods for
// runtime switching to be worth the indirection. Both classes deliberately have the same
// shape, so extracting the interface later is mechanical rather than a redesign.

}  // namespace orbitalis
