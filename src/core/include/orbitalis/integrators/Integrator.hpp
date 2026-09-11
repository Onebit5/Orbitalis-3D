#pragma once

#include <orbitalis/physics/ForceSolver.hpp>
#include <orbitalis/physics/System.hpp>

#include <memory>
#include <span>
#include <string_view>

namespace orbitalis {

/// Advances a System through time.
///
/// # the contract
///
/// `step(system, dt)` advances the system by **exactly** `dt` seconds. That wording is
/// deliberate and it is the decision that shapes everything downstream.
///
/// An adaptive method like RKF45 (0.2.5) does not naturally take a fixed step: it picks its
/// own, based on a local error estimate. The tempting interface therefore returns how much
/// time it actually managed. That would push the bookkeeping onto every caller, and it
/// would break `SimClock`, whose whole job is handing out identical fixed steps so that a
/// run is reproducible across framerates (0.1.6).
///
/// So an adaptive integrator subdivides *internally* and still advances exactly `dt`. The
/// caller never learns how many evaluations that took, which is right: the number of
/// internal stages is an implementation detail, and the one thing that must stay constant
/// is the interval the outside world sees.
///
/// # why the metadata is part of the interface
///
/// `order()` and `is_symplectic()` are not decoration. They are the two properties that
/// actually distinguish these methods, and both are needed by things that should not have
/// to know which concrete type they are holding:
///
///   - the comparison harness at 0.2.6 plots error against timestep, and the expected slope
///     *is* the order
///   - `is_symplectic()` is the difference between an error that oscillates forever and one
///     that accumulates. it is what makes velocity Verlet the right choice for a
///     million-step run and RKF45 the wrong one, despite RKF45 being far more accurate per
///     step
class IIntegrator
{
public:
    virtual ~IIntegrator() = default;

    /// Advances every body by exactly `dt` seconds.
    virtual void step(System& system, double dt) = 0;

    /// Short identifier, matching the name `make_integrator` accepts.
    [[nodiscard]] virtual const char* name() const noexcept = 0;

    /// Order of convergence: halving the timestep divides the error by 2^order.
    ///
    /// Euler is 1, velocity Verlet is 2, RK4 is 4. Measured at 0.2.6, where a log-log plot
    /// of error against timestep should come out with this as its slope. A method whose
    /// measured order does not match what it claims has a bug.
    [[nodiscard]] virtual int order() const noexcept = 0;

    /// Whether the method conserves a nearby Hamiltonian exactly, and therefore keeps its
    /// energy error bounded rather than accumulating.
    ///
    /// Note this is a property of the method *at a fixed timestep*. Varying the step
    /// destroys it, which is why adaptive methods are not symplectic even when built on a
    /// symplectic core, and why frame delta-time must never reach an integrator.
    [[nodiscard]] virtual bool is_symplectic() const noexcept = 0;

    /// Discards any state carried between steps.
    ///
    /// Most methods have none and the default does nothing. Velocity Verlet at 0.2.2 does:
    /// it reuses the acceleration computed at the end of one step as the start of the next,
    /// which is what makes it one force evaluation per step instead of two. That cache is
    /// only valid if the positions have not changed behind its back, so anything that
    /// reloads a scenario, rewinds, or swaps the force solver must call this.
    virtual void reset() noexcept {}

protected:
    // Protected and defaulted rather than deleted: concrete integrators stay copyable, but
    // nobody can slice one through a base reference by accident.
    IIntegrator() = default;
    IIntegrator(const IIntegrator&) = default;
    IIntegrator(IIntegrator&&) = default;
    IIntegrator& operator=(const IIntegrator&) = default;
    IIntegrator& operator=(IIntegrator&&) = default;
};

/// The method to use when nobody has expressed a preference.
///
/// Lives here rather than in the viewer because "which integrator should I use" is a
/// physics answer, not a UI one, and the same answer is wanted by the scenario loader at
/// 0.5.3 when a file omits the field. Velocity Verlet because it is second order and
/// symplectic at one force evaluation per step, which is the right default for a long run
/// even though RKF45 will beat it over a short arc.
///
/// A test asserts this name actually constructs. That check exists because the viewer used
/// to pick its starting method by index, and a lookup that quietly failed would have fallen
/// back to whatever sat at index 0, which is forward Euler: the one method in the project
/// that is deliberately bad. Silently defaulting to the worst option is the sort of failure
/// that looks like a physics bug for a week.
inline constexpr std::string_view kDefaultIntegratorName = "velocity-verlet";

/// Every name `make_integrator` understands, in a stable order suitable for cycling through
/// in a menu.
///
/// Stable because scenario files at 0.5.3 will name an integrator in text, and a benchmark
/// table is easier to read when the rows do not move around between runs.
[[nodiscard]] std::span<const std::string_view> integrator_names() noexcept;

/// Creates an integrator by name, or nullptr if the name is not recognised.
///
/// `solver` must outlive the returned integrator.
///
/// Returning nullptr rather than throwing or falling back to a default: an unknown name in
/// a scenario file is a mistake the user needs told about, and quietly substituting some
/// other method would produce results attributed to an integrator that never ran.
[[nodiscard]] std::unique_ptr<IIntegrator> make_integrator(std::string_view name,
                                                           const IForceSolver& solver);

}  // namespace orbitalis
