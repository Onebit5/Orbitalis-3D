#include <doctest/doctest.h>

#include <orbitalis/integrators/Euler.hpp>
#include <orbitalis/integrators/Integrator.hpp>
#include <orbitalis/integrators/Verlet.hpp>
#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/Diagnostics.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <span>
#include <string_view>

using orbitalis::Body;
using orbitalis::BruteForceSolver;
using orbitalis::distance;
using orbitalis::ForwardEuler;
using orbitalis::IForceSolver;
using orbitalis::kAstronomicalUnit;
using orbitalis::kGravitationalConstant;
using orbitalis::kSolarMass;
using orbitalis::make_integrator;
using orbitalis::SemiImplicitEuler;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::VelocityVerlet;
using orbitalis::scenarios::circular_period;
using orbitalis::scenarios::sun_earth;

namespace {

// =======================================================================================
// helpers
// =======================================================================================

/// A solver that forwards to brute force and counts how many times it was asked.
///
/// The claim "one force evaluation per step" is the entire reason velocity Verlet caches
/// an acceleration, and it is the sort of claim that stays true in a comment long after it
/// has stopped being true in the code. This turns it into a number.
class CountingSolver final : public IForceSolver
{
public:
    void compute_accelerations(std::span<const Body> bodies,
                               std::span<Vec3> accelerations) const override
    {
        ++calls_;
        inner_.compute_accelerations(bodies, accelerations);
    }

    [[nodiscard]] double potential_energy(std::span<const Body> bodies) const override
    {
        return inner_.potential_energy(bodies);
    }

    [[nodiscard]] const char* name() const noexcept override { return "counting"; }

    [[nodiscard]] int calls() const noexcept { return calls_; }

private:
    BruteForceSolver inner_;
    mutable int calls_{0};  // compute_accelerations is const, and should be
};

/// A Sun and a 1 kg probe on a circular orbit at one AU, in the XY plane.
///
/// The probe is light enough that the Sun's own acceleration is about 3e-33 m/s2, which
/// displaces it by roughly 1e-18 m over a year. That is far below the integration errors
/// being measured here, so the analytic fixed-centre circular solution is exact for these
/// purposes and the measured error really is the integrator's.
struct CircularOrbit
{
    static constexpr double radius = kAstronomicalUnit;

    // G*M rather than kSunGM, so this matches exactly what the solver computes from the
    // mass it is handed. The difference is one rounding, but it costs nothing to be exact.
    static inline const double gm = kGravitationalConstant * kSolarMass;
    static inline const double omega = std::sqrt(gm / (radius * radius * radius));
    static inline const double period = circular_period(gm, radius);

    [[nodiscard]] static System state()
    {
        System system;
        system.add(Body{kSolarMass, 6.957e8, Vec3{}, Vec3{}}, "Sun");
        system.add(Body{1.0, 1.0, Vec3{radius, 0.0, 0.0}, Vec3{0.0, radius * omega, 0.0}},
                   "probe");
        return system;
    }

    /// Where the probe is at time t, exactly.
    [[nodiscard]] static Vec3 true_position(double t) noexcept
    {
        return Vec3{radius * std::cos(omega * t), radius * std::sin(omega * t), 0.0};
    }
};

/// Distance between where the integrator put the probe and where it should be, after
/// covering a quarter of an orbit in `steps` equal steps.
///
/// A quarter orbit rather than a whole one on purpose: at exactly one period the dominant
/// error is a phase error that has come most of the way back around, and measuring an
/// error where part of it has cancelled makes a convergence order look better than it is.
[[nodiscard]] double quarter_orbit_error(std::string_view name, int steps)
{
    const BruteForceSolver solver;
    const auto integrator = make_integrator(name, solver);
    REQUIRE(integrator != nullptr);

    const double arc = 0.25 * CircularOrbit::period;
    const double dt = arc / steps;

    System system = CircularOrbit::state();
    for (int i = 0; i < steps; ++i) {
        integrator->step(system, dt);
    }

    return distance(system[1].position, CircularOrbit::true_position(arc));
}

/// Total mechanical energy, J.
///
/// A thin call into the library now. At 0.2.2 this was written out by hand here because core
/// did not have it yet; 0.2.3 moved it in, and asking the solver for the potential is what
/// guarantees it matches the force being integrated.
[[nodiscard]] double total_energy(const System& system, const IForceSolver& solver)
{
    return orbitalis::measure(system, solver).total;
}

/// How the energy error behaves over a run.
///
/// Two different questions, because one number cannot answer both. `worst_early` against
/// `worst_late` says whether the error stays inside a fixed envelope. `rising` against
/// `samples` says whether it has a preferred direction, sampled once per orbit.
///
/// The direction measure exists because |E(t) - E0| / |E0| saturates: as a non-symplectic
/// method pumps energy into a bound orbit, E climbs toward zero, so the ratio approaches 1
/// and flattens out no matter how badly the trajectory is being destroyed. Asking how much
/// the error grew would therefore be asking about a ceiling rather than about the method.
/// Asking whether it ever goes back down is not fooled by that.
struct EnergyProfile
{
    double worst{};
    double worst_early{};
    double worst_late{};
    int samples{};  ///< orbit boundaries compared against the previous one
    int rising{};   ///< of those, how many had a larger signed error than the last
};

[[nodiscard]] EnergyProfile energy_profile(std::string_view name, int orbits, int per_orbit)
{
    const BruteForceSolver solver;
    const auto integrator = make_integrator(name, solver);
    REQUIRE(integrator != nullptr);

    System system = sun_earth();
    const double period =
        circular_period(orbitalis::kSunGM + orbitalis::kEarthGM, kAstronomicalUnit);
    const double dt = period / per_orbit;

    const double e0 = total_energy(system, solver);
    const int steps = orbits * per_orbit;

    EnergyProfile profile;
    double previous_signed = 0.0;
    bool have_previous = false;

    for (int i = 0; i < steps; ++i) {
        integrator->step(system, dt);

        // Divided by |E0|, not by E0. A bound orbit has E0 < 0, so dividing by it signed
        // would report an energy *gain* as a decrease, which is how the first version of
        // this got forward Euler exactly backwards.
        const double signed_error = (total_energy(system, solver) - e0) / std::abs(e0);
        const double error = std::abs(signed_error);

        profile.worst = std::max(profile.worst, error);

        if (i < steps / 10) {
            profile.worst_early = std::max(profile.worst_early, error);
        } else if (i >= steps - steps / 10) {
            profile.worst_late = std::max(profile.worst_late, error);
        }

        // Once per orbit, at the same phase every time, so a within-orbit oscillation
        // cannot be mistaken for a trend and vice versa.
        if ((i + 1) % per_orbit == 0) {
            if (have_previous) {
                ++profile.samples;
                if (signed_error > previous_signed) {
                    ++profile.rising;
                }
            }
            previous_signed = signed_error;
            have_previous = true;
        }
    }
    return profile;
}

}  // namespace

// =======================================================================================
// it exists and says what it is
// =======================================================================================

TEST_CASE("velocity Verlet is reachable by name and reports second order and symplectic")
{
    const BruteForceSolver solver;
    const auto integrator = make_integrator("velocity-verlet", solver);

    REQUIRE(integrator != nullptr);
    CHECK(std::string_view{integrator->name()} == "velocity-verlet");
    CHECK(integrator->order() == 2);
    CHECK(integrator->is_symplectic());

    // And it turns up in the list, which is what puts it in the viewer's cycle key and in
    // the comparison harness at 0.2.6. The registry derives the name list from the factory
    // table precisely so these two cannot disagree.
    const auto names = orbitalis::integrator_names();
    CHECK(std::find(names.begin(), names.end(), "velocity-verlet") != names.end());
}

// =======================================================================================
// order of convergence -- the claim order() makes
// =======================================================================================

TEST_CASE("halving the timestep divides Verlet's error by four")
{
    // This is the definition of second order, measured rather than asserted. If the
    // kick-drift-kick split were written wrong, a whole kick before the drift instead of
    // two halves around it, the method would still run and still look like an orbit, and
    // this ratio would drop to 2. That is the bug this test exists to catch.
    const double e1 = quarter_orbit_error("velocity-verlet", 64);
    const double e2 = quarter_orbit_error("velocity-verlet", 128);
    const double e3 = quarter_orbit_error("velocity-verlet", 256);

    MESSAGE("verlet errors (m): " << e1 << " " << e2 << " " << e3);
    MESSAGE("verlet ratios: " << e1 / e2 << " " << e2 / e3);

    CHECK(e1 / e2 == doctest::Approx(4.0).epsilon(0.01));
    CHECK(e2 / e3 == doctest::Approx(4.0).epsilon(0.01));
}

TEST_CASE("the same measurement gives two for semi-implicit Euler")
{
    // The control. A harness with a subtle bug could land near 4 by accident; measuring a
    // method whose order is already known, with the same code, is what makes the result
    // above mean something.
    const double e1 = quarter_orbit_error("semi-implicit-euler", 64);
    const double e2 = quarter_orbit_error("semi-implicit-euler", 128);
    const double e3 = quarter_orbit_error("semi-implicit-euler", 256);

    MESSAGE("semi-implicit errors (m): " << e1 << " " << e2 << " " << e3);
    MESSAGE("semi-implicit ratios: " << e1 / e2 << " " << e2 / e3);

    CHECK(e1 / e2 == doctest::Approx(2.0).epsilon(0.01));
    CHECK(e2 / e3 == doctest::Approx(2.0).epsilon(0.01));
}

TEST_CASE("at equal cost Verlet is far more accurate than either Euler")
{
    // Same number of force evaluations, same timestep, same scenario. This is the whole
    // argument for the method in one comparison.
    const int steps = 256;

    const double verlet = quarter_orbit_error("velocity-verlet", steps);
    const double semi = quarter_orbit_error("semi-implicit-euler", steps);
    const double forward = quarter_orbit_error("forward-euler", steps);

    MESSAGE("quarter orbit error at " << steps << " steps -- verlet " << verlet << " m, semi "
                                      << semi << " m, forward " << forward << " m");

    CHECK(verlet < semi);
    CHECK(verlet < forward);

    // Measured 725x on this arc, identical in Debug and Release. Asserted a little below
    // that rather than at 2x, because "somewhat better" would pass even if the method had
    // quietly degraded to first order.
    CHECK(semi / verlet > 500.0);
}

// =======================================================================================
// energy: bounded rather than accumulating
// =======================================================================================

TEST_CASE("Verlet's energy error stays bounded over hundreds of orbits")
{
    // Symplecticity, stated as something measurable. The error oscillates within a bound
    // set by the timestep, and crucially the bound does not grow: the worst excursion in
    // the last tenth of a 300 orbit run is the same size as in the first tenth.
    const EnergyProfile verlet = energy_profile("velocity-verlet", 300, 365);

    MESSAGE("verlet energy error -- worst " << verlet.worst << ", early " << verlet.worst_early
                                            << ", late " << verlet.worst_late << ", rising "
                                            << verlet.rising << "/" << verlet.samples);

    CHECK(verlet.worst < 1e-7);          // measured 2.19e-8
    CHECK(verlet.worst_late < verlet.worst_early * 1.001);  // measured ratio 1.0000005

    // The per-orbit direction count is reported above but deliberately not asserted here,
    // and the reason is worth writing down. It comes out near-monotonic for Verlet, which
    // looks alarming until you notice what is being sampled: the numerical orbit's period
    // is not exactly the analytic one, so "every 365 steps" creeps slowly in phase, and the
    // sampled value slides along a within-orbit oscillation rather than tracking a drift.
    // That is measuring the sampling, not the method. The envelope check above is immune to
    // it because it takes a maximum over a whole tenth of the run. Same mistake as the
    // 0.0.5 envelope test, caught earlier this time.
}

TEST_CASE("and it is far tighter than semi-implicit Euler's at the same timestep")
{
    // Both methods are symplectic, so both are bounded. What second order buys is the size
    // of the bound, and the difference is not subtle.
    const EnergyProfile verlet = energy_profile("velocity-verlet", 20, 365);
    const EnergyProfile semi = energy_profile("semi-implicit-euler", 20, 365);

    MESSAGE("bounds -- verlet " << verlet.worst << ", semi-implicit " << semi.worst
                                << ", ratio " << semi.worst / verlet.worst << "; semi early "
                                << semi.worst_early << " late " << semi.worst_late);

    CHECK(semi.worst > verlet.worst * 8000.0);  // measured 13,506
    CHECK(semi.worst_late < semi.worst_early * 1.001);  // measured equal to six digits
}

TEST_CASE("forward Euler's error only ever goes one way, which is what bounded means")
{
    // The contrast that makes "bounded" a claim rather than a word.
    //
    // Note this asks about direction, not size. The first version of this test asserted
    // that the error grew by at least 5x from the first orbits to the last, and it failed:
    // measured 0.246 early against 0.586 late, a factor of 2.4. That is not the method
    // behaving well. |E - E0| / |E0| simply cannot exceed 1 while the orbit is still bound,
    // because forward Euler drives E upward toward zero, so the ratio flattens against a
    // ceiling that has nothing to do with how wrong the trajectory is. Sampling the signed
    // error once per orbit and asking whether it ever decreases is immune to that.
    const EnergyProfile forward = energy_profile("forward-euler", 20, 365);

    MESSAGE("forward euler -- early " << forward.worst_early << ", late " << forward.worst_late
                                      << ", rising " << forward.rising << "/" << forward.samples);

    // Every single orbit leaves the system with more energy than the one before. No
    // oscillation at all: this is a one-way street, and phase creep cannot fake it because
    // the magnitude grows by more than the whole within-orbit variation.
    REQUIRE(forward.samples == 19);
    CHECK(forward.rising == forward.samples);

    // The envelope grows too, and lands somewhere a bounded method never goes: a quarter of
    // the system's binding energy gone within two orbits, well over half of it by twenty.
    // Verlet at the same timestep sits at 2e-8.
    CHECK(forward.worst_late > forward.worst_early * 2.0);
    CHECK(forward.worst > 0.5);
}

// =======================================================================================
// one force evaluation per step
// =======================================================================================

TEST_CASE("Verlet evaluates the force once per step after priming")
{
    const CountingSolver solver;
    VelocityVerlet integrator{solver};

    System system = sun_earth();
    const double dt = 3600.0;

    integrator.step(system, dt);
    CHECK(solver.calls() == 2);  // one to prime the cache, one for the step itself

    for (int i = 0; i < 99; ++i) {
        integrator.step(system, dt);
    }
    CHECK(solver.calls() == 101);  // 100 steps, plus the single priming evaluation

    SUBCASE("and reset costs one more priming evaluation, not one per step")
    {
        integrator.reset();
        CHECK_FALSE(integrator.has_cached_acceleration());

        for (int i = 0; i < 10; ++i) {
            integrator.step(system, dt);
        }
        CHECK(solver.calls() == 112);  // 101 + 1 priming + 10 steps
    }
}

TEST_CASE("semi-implicit Euler also costs one evaluation per step, with nothing to prime")
{
    // Worth pinning next to the above, because "one evaluation per step" is only
    // interesting alongside the order. Verlet gets second order at first order's price.
    const CountingSolver solver;
    SemiImplicitEuler integrator{solver};

    System system = sun_earth();
    for (int i = 0; i < 100; ++i) {
        integrator.step(system, 3600.0);
    }

    CHECK(solver.calls() == 100);
}

// =======================================================================================
// the cache, and the ways it can go stale
// =======================================================================================

TEST_CASE("carrying the acceleration forward gives bit-identical results to recomputing it")
{
    // The cache is not an approximation. The acceleration at the end of a step is computed
    // from the same positions the next step starts at, so reusing it must give exactly the
    // same bits as computing it again. Approx() would hide a real difference here, so this
    // is exact equality on purpose.
    const BruteForceSolver solver;
    const double dt = 3600.0 * 6.0;

    System cached = sun_earth();
    VelocityVerlet with_cache{solver};

    System recomputed = sun_earth();
    VelocityVerlet without_cache{solver};

    for (int i = 0; i < 500; ++i) {
        with_cache.step(cached, dt);

        without_cache.reset();  // forces a fresh opening acceleration every single step
        without_cache.step(recomputed, dt);
    }

    CHECK(cached[1].position == recomputed[1].position);
    CHECK(cached[1].velocity == recomputed[1].velocity);
}

TEST_CASE("changing the body count invalidates the cache without being asked")
{
    // The one stale-cache case that can be detected for free, so it is. Adding a body
    // changes the acceleration of every other body, so keeping the old entries and filling
    // in a new one would be wrong; the whole cache goes.
    const BruteForceSolver solver;
    const double dt = 3600.0;

    System system = sun_earth();
    VelocityVerlet reused{solver};
    for (int i = 0; i < 20; ++i) {
        reused.step(system, dt);
    }

    system.add(Body{1.0e24, 3.4e6, Vec3{2.3 * kAstronomicalUnit, 0.0, 0.0},
                    Vec3{0.0, 19000.0, 0.0}},
               "interloper");

    System reference = system;  // same state, stepped by an integrator with no history
    VelocityVerlet fresh{solver};

    reused.step(system, dt);
    fresh.step(reference, dt);

    CHECK(system[1].position == reference[1].position);
    CHECK(system[2].position == reference[2].position);
}

TEST_CASE("moving bodies behind the integrator's back needs an explicit reset")
{
    // The documented limitation, written down as a test so it is a known shape rather than
    // a surprise. A teleported body with the body count unchanged cannot be detected
    // cheaply, so the next step is taken with the forces from where it used to be.
    //
    // Both integrators below are given the same twenty steps of history first. That matters:
    // an earlier version of this test called reset() on a brand new integrator, which has
    // nothing to clear, so the comparison was vacuous and a reset() that did nothing at all
    // still passed. Mutation testing is what surfaced that.
    const BruteForceSolver solver;
    const double dt = 3600.0;

    System kept = sun_earth();
    System healed = sun_earth();
    VelocityVerlet keeps_cache{solver};
    VelocityVerlet gets_reset{solver};

    for (int i = 0; i < 20; ++i) {
        keeps_cache.step(kept, dt);
        gets_reset.step(healed, dt);
    }
    REQUIRE(kept[1].position == healed[1].position);  // identical histories so far

    // The interference: drag Earth a long way inward, identically in both.
    const Vec3 teleport{0.4 * kAstronomicalUnit, 0.0, 0.0};
    kept[1].position = teleport;
    healed[1].position = teleport;

    System correct = healed;
    VelocityVerlet fresh{solver};

    keeps_cache.step(kept, dt);  // cache still holds accelerations from 1 AU

    gets_reset.reset();
    CHECK_FALSE(gets_reset.has_cached_acceleration());
    gets_reset.step(healed, dt);

    fresh.step(correct, dt);

    // Resetting an integrator that *had* a cache gives exactly what a new one gives.
    CHECK(healed[1].velocity == correct[1].velocity);
    CHECK(healed[1].position == correct[1].position);

    // Not resetting does not, and the difference is real rather than a rounding.
    const double error = distance(kept[1].velocity, correct[1].velocity);
    MESSAGE("stale cache costs " << error << " m/s on one step");
    CHECK(error > 10.0);  // measured 56 m/s, against an orbital speed of 3e4
}

// =======================================================================================
// time reversibility
// =======================================================================================

TEST_CASE("running Verlet backwards retraces its own path")
{
    // Time symmetry, which is the structural reason the method is second order: the
    // composition half-kick, drift, half-kick reads the same in both directions, so the
    // odd-order error terms cancel. Step forward, flip every velocity, step the same
    // number of times, flip back, and the system is where it started to within roundoff.
    const BruteForceSolver solver;
    const double dt = 3600.0 * 6.0;
    const int steps = 2000;

    System system = sun_earth();
    const Vec3 start_position = system[1].position;
    const Vec3 start_velocity = system[1].velocity;

    VelocityVerlet integrator{solver};
    for (int i = 0; i < steps; ++i) {
        integrator.step(system, dt);
    }

    // No reset needed: acceleration depends on positions and masses, and neither changed.
    for (Body& b : system.bodies()) {
        b.velocity = -b.velocity;
    }
    for (int i = 0; i < steps; ++i) {
        integrator.step(system, dt);
    }
    for (Body& b : system.bodies()) {
        b.velocity = -b.velocity;
    }

    const double position_error = distance(system[1].position, start_position);
    const double velocity_error = distance(system[1].velocity, start_velocity);

    MESSAGE("after " << steps << " steps out and back: " << position_error << " m, "
                     << velocity_error << " m/s");

    // Tolerances set from measurement. What is left is accumulated floating-point rounding
    // over 4000 steps at 1e11 metres, not anything the method did wrong.
    CHECK(position_error < 0.01);   // measured 5.9e-4 m, on an orbit of 1.5e11 m
    CHECK(velocity_error < 1e-9);   // measured 8.6e-11 m/s, on a speed of 3e4 m/s
}

TEST_CASE("forward Euler does not, which is what makes that test worth running")
{
    const BruteForceSolver solver;
    const double dt = 3600.0 * 6.0;
    const int steps = 2000;

    System system = sun_earth();
    const Vec3 start_position = system[1].position;

    ForwardEuler integrator{solver};
    for (int i = 0; i < steps; ++i) {
        integrator.step(system, dt);
    }
    for (Body& b : system.bodies()) {
        b.velocity = -b.velocity;
    }
    for (int i = 0; i < steps; ++i) {
        integrator.step(system, dt);
    }

    const double position_error = distance(system[1].position, start_position);
    MESSAGE("forward Euler out and back: " << position_error << " m");

    // Measured 1.1e11 m, which is 0.74 AU: not off by a rounding, off by most of the
    // orbit. Verlet manages 5.9e-4 m on the identical run.
    CHECK(position_error > 1e10);
}

// =======================================================================================
// edges
// =======================================================================================

TEST_CASE("a zero timestep changes nothing but does prime the cache")
{
    const CountingSolver solver;
    VelocityVerlet integrator{solver};

    System system = sun_earth();
    const Vec3 position = system[1].position;
    const Vec3 velocity = system[1].velocity;

    integrator.step(system, 0.0);

    CHECK(system[1].position == position);
    CHECK(system[1].velocity == velocity);
    CHECK(integrator.has_cached_acceleration());
    CHECK(solver.calls() == 2);
}

TEST_CASE("an empty system steps without complaint")
{
    const BruteForceSolver solver;
    VelocityVerlet integrator{solver};

    System system;
    integrator.step(system, 3600.0);

    CHECK(system.empty());
}

TEST_CASE("Verlet conserves linear momentum to roundoff")
{
    // Momentum conservation is a property of the force solver, not the integrator: it
    // follows from Newton's third law inside the pair loop. Checked here anyway, because
    // the symmetric velocity update is where an integrator could break it, and because a
    // momentum drift would otherwise surface much later looking like a physics bug.
    const BruteForceSolver solver;
    VelocityVerlet integrator{solver};

    System system = sun_earth();
    const Vec3 initial = system.total_momentum();

    for (int i = 0; i < 5000; ++i) {
        integrator.step(system, 3600.0);
    }

    const Vec3 drift = system.total_momentum() - initial;

    // Scaled against the largest single body momentum, because an absolute figure at solar
    // system masses is a big number that means nothing on its own.
    double largest = 0.0;
    for (const Body& b : system.bodies()) {
        largest = std::max(largest, (b.mass * b.velocity).length());
    }

    MESSAGE("momentum drift, relative: " << drift.length() / largest);
    CHECK(drift.length() / largest < 3e-14);  // measured 1.05e-14 over 5000 steps
}
