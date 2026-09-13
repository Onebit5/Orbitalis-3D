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

#include <cmath>
#include <cstddef>
#include <string_view>
#include <vector>

using orbitalis::angular_momentum;
using orbitalis::Body;
using orbitalis::BruteForceSolver;
using orbitalis::ConservationMonitor;
using orbitalis::cross;
using orbitalis::Diagnostics;
using orbitalis::kAstronomicalUnit;
using orbitalis::kGravitationalConstant;
using orbitalis::kinetic_energy;
using orbitalis::make_integrator;
using orbitalis::measure;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::VelocityVerlet;
using orbitalis::scenarios::circular_period;
using orbitalis::scenarios::sun_earth;
using orbitalis::scenarios::sun_earth_drifting;

namespace {

/// Two masses a known distance apart, so every quantity has a closed form.
System two_masses(double separation = 1.0e9)
{
    System system;
    system.add(Body{3.0e24, 1.0e6, Vec3{0.0, 0.0, 0.0}, Vec3{0.0, 100.0, 0.0}}, "a");
    system.add(Body{5.0e24, 1.0e6, Vec3{separation, 0.0, 0.0}, Vec3{0.0, -60.0, 0.0}}, "b");
    return system;
}

}  // namespace

// =======================================================================================
// the pieces, against closed forms
// =======================================================================================

TEST_CASE("kinetic energy is the sum of half m v squared")
{
    System system;
    system.add(Body{2.0, 1.0, Vec3{}, Vec3{3.0, 4.0, 0.0}});   // |v| = 5, T = 25
    system.add(Body{10.0, 1.0, Vec3{}, Vec3{0.0, 0.0, 1.0}});  // T = 5

    CHECK(kinetic_energy(system.bodies()) == doctest::Approx(30.0));

    SUBCASE("and it is never negative")
    {
        for (Body& b : system.bodies()) {
            b.velocity = -b.velocity;
        }
        CHECK(kinetic_energy(system.bodies()) == doctest::Approx(30.0));
    }

    SUBCASE("an empty system has none")
    {
        System empty;
        CHECK(kinetic_energy(empty.bodies()) == 0.0);
    }
}

TEST_CASE("potential energy of a pair is minus G m1 m2 over r")
{
    const BruteForceSolver solver;
    const double separation = 1.0e9;
    const System system = two_masses(separation);

    const double expected = -kGravitationalConstant * 3.0e24 * 5.0e24 / separation;

    CHECK(solver.potential_energy(system.bodies()) == doctest::Approx(expected));

    SUBCASE("counted once per pair, not once per ordered pair")
    {
        // The classic way to be exactly 2x wrong and still look right, because a doubled
        // energy is conserved just as well as the correct one.
        CHECK(solver.potential_energy(system.bodies()) > 2.0 * expected);
    }

    SUBCASE("a single body has no potential energy at all")
    {
        System lone;
        lone.add(Body{1.0e30, 1.0, Vec3{}, Vec3{}});
        CHECK(solver.potential_energy(lone.bodies()) == 0.0);
    }

    SUBCASE("and it rises toward zero as the pair separates")
    {
        const System far = two_masses(separation * 1000.0);
        const double near_energy = solver.potential_energy(system.bodies());
        const double far_energy = solver.potential_energy(far.bodies());

        CHECK(far_energy > near_energy);
        CHECK(far_energy < 0.0);
    }
}

TEST_CASE("softening changes the potential to match the softened force")
{
    const double separation = 1.0e9;
    const double epsilon = 4.0e8;
    const BruteForceSolver solver{epsilon};
    const System system = two_masses(separation);

    const double expected = -kGravitationalConstant * 3.0e24 * 5.0e24 /
                            std::sqrt(separation * separation + epsilon * epsilon);

    CHECK(solver.potential_energy(system.bodies()) == doctest::Approx(expected));

    // Softening makes the well shallower, never deeper, which is the whole point: it caps
    // what happens at zero separation instead of letting it diverge.
    const BruteForceSolver unsoftened;
    CHECK(solver.potential_energy(system.bodies()) >
          unsoftened.potential_energy(system.bodies()));
}

// =======================================================================================
// the one that matters: potential and force are the same physics
// =======================================================================================

TEST_CASE("the force really is minus the gradient of this potential")
{
    // This is the test the whole design of putting potential_energy on IForceSolver exists
    // to make possible. If the two disagree, every energy figure downstream is measuring
    // the disagreement rather than the integrator, and it would look exactly like drift.
    //
    // Numerically: shift one body by ±h along an axis and difference the total potential
    // energy. dU/dx_i should equal -m_i * a_i, because F = -grad U and F = m*a.
    const BruteForceSolver solver;

    System system;
    system.add(Body{2.0e30, 7.0e8, Vec3{0.0, 0.0, 0.0}, Vec3{}}, "star");
    system.add(Body{6.0e24, 6.4e6, Vec3{1.5e11, 2.0e10, -3.0e9}, Vec3{}}, "planet");
    system.add(Body{1.9e27, 7.1e7, Vec3{-4.0e11, 5.0e10, 8.0e9}, Vec3{}}, "giant");

    std::vector<Vec3> accelerations(system.size());
    solver.compute_accelerations(system.bodies(), accelerations);

    // Step chosen against the scale of the problem. Too small and the difference of two
    // nearly equal potentials is pure roundoff; too large and the second-order term of the
    // expansion shows up. 1e4 m against separations of 1e11 m sits in the middle.
    const double h = 1.0e4;

    for (std::size_t i = 0; i < system.size(); ++i) {
        CAPTURE(i);
        for (int axis = 0; axis < 3; ++axis) {
            CAPTURE(axis);

            Vec3 offset{};
            offset[axis] = h;

            System plus = system;
            plus[i].position += offset;
            System minus = system;
            minus[i].position -= offset;

            const double gradient = (solver.potential_energy(plus.bodies()) -
                                     solver.potential_energy(minus.bodies())) /
                                    (2.0 * h);

            const double expected = -system[i].mass * accelerations[i][axis];

            // Relative comparison: these are 1e17-scale numbers and an absolute tolerance
            // would be meaningless. Central differences are second order, so the residual
            // here is (h/L)² plus roundoff, measured at a few parts in 1e7.
            CHECK(gradient == doctest::Approx(expected).epsilon(1e-5));
        }
    }
}

TEST_CASE("and it still holds with softening switched on")
{
    // The case that actually bites. The softened force has (d² + e²) to the 3/2, the
    // softened potential has it to the 1/2, and pairing either with the unsoftened version
    // of the other produces a drift that changes when epsilon changes rather than when the
    // integrator does.
    const BruteForceSolver solver{2.0e10};

    System system;
    system.add(Body{2.0e30, 7.0e8, Vec3{0.0, 0.0, 0.0}, Vec3{}});
    system.add(Body{6.0e24, 6.4e6, Vec3{1.5e11, 2.0e10, 0.0}, Vec3{}});

    std::vector<Vec3> accelerations(system.size());
    solver.compute_accelerations(system.bodies(), accelerations);

    const double h = 1.0e4;
    for (std::size_t i = 0; i < system.size(); ++i) {
        CAPTURE(i);
        Vec3 offset{};
        offset[0] = h;

        System plus = system;
        plus[i].position += offset;
        System minus = system;
        minus[i].position -= offset;

        const double gradient = (solver.potential_energy(plus.bodies()) -
                                 solver.potential_energy(minus.bodies())) /
                                (2.0 * h);

        CHECK(gradient == doctest::Approx(-system[i].mass * accelerations[i][0]).epsilon(1e-5));
    }
}

// =======================================================================================
// angular momentum
// =======================================================================================

TEST_CASE("angular momentum of a body in the XY plane points along Z")
{
    System system;
    system.add(Body{4.0, 1.0, Vec3{3.0, 0.0, 0.0}, Vec3{0.0, 5.0, 0.0}});

    // L = m (r x v) = 4 * (3 x_hat) x (5 y_hat) = 60 z_hat
    const Vec3 l = angular_momentum(system.bodies());
    CHECK(l.x == doctest::Approx(0.0));
    CHECK(l.y == doctest::Approx(0.0));
    CHECK(l.z == doctest::Approx(60.0));

    SUBCASE("and reverses when the orbit does")
    {
        system[0].velocity = -system[0].velocity;
        CHECK(angular_momentum(system.bodies()).z == doctest::Approx(-60.0));
    }
}

TEST_CASE("the origin only matters when the system carries net momentum")
{
    const Vec3 elsewhere{7.0e11, -2.0e11, 5.0e10};

    SUBCASE("with zero total momentum the choice of origin is irrelevant")
    {
        const System system = sun_earth();

        // Relative, not absolute. |P| here is 3.5e13 kg m/s, which sounds enormous and is
        // 2e-16 of Earth's own momentum: the residue of building the scenario in floating
        // point. An absolute threshold on a solar-system quantity is not a measurement, and
        // writing one here anyway is a mistake I have made before.
        const Diagnostics d = measure(system, BruteForceSolver{});
        REQUIRE(d.momentum.length() / d.largest_body_momentum < 1e-14);

        const Vec3 about_origin = angular_momentum(system.bodies());
        const Vec3 about_elsewhere = angular_momentum(system.bodies(), elsewhere);

        // They differ by R x P, and P is zero.
        CHECK(about_origin.z == doctest::Approx(about_elsewhere.z));
    }

    SUBCASE("with net momentum it is not")
    {
        const System system = sun_earth_drifting();
        const Diagnostics d = measure(system, BruteForceSolver{});
        REQUIRE(d.momentum.length() / d.largest_body_momentum > 0.5);

        const Vec3 about_origin = angular_momentum(system.bodies());
        const Vec3 about_elsewhere = angular_momentum(system.bodies(), elsewhere);

        CHECK(about_origin.z != doctest::Approx(about_elsewhere.z));
    }
}

// =======================================================================================
// measure(), as a bundle
// =======================================================================================

TEST_CASE("measure reports a bound system as bound")
{
    const BruteForceSolver solver;
    const System system = sun_earth();
    const Diagnostics d = measure(system, solver);

    CHECK(d.kinetic > 0.0);
    CHECK(d.potential < 0.0);
    CHECK(d.total == doctest::Approx(d.kinetic + d.potential));

    // Bound means negative total energy, and for a circular orbit the virial theorem is
    // sharp: 2T + U = 0, so E = -T exactly.
    CHECK(d.total < 0.0);
    CHECK(d.total == doctest::Approx(-d.kinetic).epsilon(1e-6));

    // Barycentric scenario: no net momentum, barycentre at the origin. Momentum measured
    // against a body rather than in absolute terms, for the reason spelled out below.
    CHECK(d.momentum.length() / d.largest_body_momentum < 1e-14);
    CHECK(d.center_of_mass.length() < 1.0);

    // The orbit is in the XY plane, so L points along Z and nowhere else.
    CHECK(d.angular_momentum.z > 0.0);
    CHECK(std::abs(d.angular_momentum.x) < std::abs(d.angular_momentum.z) * 1e-12);

    // Scales exist and are positive, so nothing downstream divides by zero.
    CHECK(d.largest_body_momentum > 0.0);
    CHECK(d.largest_body_angular_momentum > 0.0);
}

TEST_CASE("an unbound pair reports positive total energy")
{
    const BruteForceSolver solver;

    System system;
    system.add(Body{1.0e24, 1.0e6, Vec3{}, Vec3{0.0, 0.0, 0.0}});
    // Far apart and moving fast: kinetic wins.
    system.add(Body{1.0e3, 1.0, Vec3{1.0e12, 0.0, 0.0}, Vec3{0.0, 1.0e5, 0.0}});

    CHECK(measure(system, solver).total > 0.0);
}

TEST_CASE("measure takes angular momentum about the barycentre, not about the origin")
{
    // Mutation testing forced this test to be rewritten. The first version boosted
    // sun_earth() and checked the angular momentum did not move, which passed happily even
    // when measure() was changed to use the origin. The reason is that sun_earth() puts its
    // barycentre *at* the origin, so the two choices are the same number there and the
    // scenario could not tell them apart. The test was asserting the right idea against a
    // case with no opinion about it.
    //
    // This needs a system that is both drifting, so P is non-zero and the origin genuinely
    // matters, and displaced, so R x P is comparable to the internal rotation rather than a
    // rounding on top of it.
    const BruteForceSolver solver;

    System home = sun_earth_drifting();
    System moved = sun_earth_drifting();

    const Vec3 shift{5.0 * kAstronomicalUnit, -3.0 * kAstronomicalUnit, 0.0};
    for (Body& b : moved.bodies()) {
        b.position += shift;
        b.velocity += Vec3{1.0e4, -5.0e3, 2.0e3};
    }

    const Vec3 internal_home = measure(home, solver).angular_momentum;
    const Vec3 internal_moved = measure(moved, solver).angular_momentum;

    // Same physics, written down in a translated and boosted frame. The internal rotation
    // is what it always was.
    CHECK(internal_moved.z == doctest::Approx(internal_home.z).epsilon(1e-10));

    // Measured about a fixed origin it is a completely different number, differing by
    // R x P, which here is five times the internal value.
    const Vec3 about_origin = angular_momentum(moved.bodies());
    MESSAGE("about barycentre " << internal_moved.z << ", about origin " << about_origin.z);
    CHECK(std::abs(about_origin.z - internal_moved.z) > std::abs(internal_moved.z));
}

TEST_CASE("a boost carries real kinetic energy in, so energy is frame dependent")
{
    // The other half of the same point: angular momentum about the barycentre is a frame
    // invariant here and the total energy is not. Worth pinning so that a future me reading
    // an energy figure off the HUD remembers it describes this frame and not the physics.
    const BruteForceSolver solver;

    System at_rest = sun_earth();
    System boosted = sun_earth();
    for (Body& b : boosted.bodies()) {
        b.velocity += Vec3{1.0e4, -5.0e3, 2.0e3};
    }

    const Diagnostics a = measure(at_rest, solver);
    const Diagnostics b = measure(boosted, solver);

    CHECK(b.kinetic > a.kinetic);
    CHECK(b.potential == doctest::Approx(a.potential));  // positions unchanged
    CHECK(b.total > a.total);

    // Enough of a boost to unbind it on paper, which is exactly why the sign of the total
    // energy is only meaningful in the barycentric frame.
    CHECK(b.total > 0.0);
}

// =======================================================================================
// the monitor
// =======================================================================================

TEST_CASE("a fresh monitor has no baseline and reports no drift")
{
    ConservationMonitor monitor;

    CHECK_FALSE(monitor.has_baseline());
    CHECK(monitor.samples() == 0);
    CHECK(monitor.relative_energy_error() == 0.0);
    CHECK(monitor.worst_relative_energy_error() == 0.0);
    CHECK(monitor.relative_momentum_drift() == 0.0);
    CHECK(monitor.relative_angular_momentum_drift() == 0.0);
}

TEST_CASE("the first sample takes its own baseline rather than comparing against zero")
{
    const BruteForceSolver solver;
    const System system = sun_earth();

    ConservationMonitor monitor;
    monitor.sample(system, solver);

    CHECK(monitor.has_baseline());
    CHECK(monitor.relative_energy_error() == 0.0);
    CHECK(monitor.baseline().total == monitor.latest().total);
}

TEST_CASE("the monitor measures Verlet's drift as bounded and Euler's as growing")
{
    // The milestone's whole purpose in one test. Same scenario, same timestep, same number
    // of steps, sampled every step so the worst case is exact rather than a lower bound.
    const BruteForceSolver solver;
    const double period =
        circular_period(orbitalis::kSunGM + orbitalis::kEarthGM, kAstronomicalUnit);

    auto run = [&](std::string_view name, int orbits) {
        const auto integrator = make_integrator(name, solver);
        REQUIRE(integrator != nullptr);

        System system = sun_earth();
        ConservationMonitor monitor;
        monitor.reset(system, solver);

        for (int i = 0; i < orbits * 365; ++i) {
            integrator->step(system, period / 365.0);
            monitor.sample(system, solver);
        }
        return monitor;
    };

    const ConservationMonitor verlet = run("velocity-verlet", 20);
    const ConservationMonitor forward = run("forward-euler", 20);

    MESSAGE("verlet worst " << verlet.worst_relative_energy_error() << ", forward worst "
                            << forward.worst_relative_energy_error());

    CHECK(verlet.worst_relative_energy_error() < 1e-7);
    CHECK(forward.worst_relative_energy_error() > 0.5);

    // Sign carries meaning: forward Euler pumps energy *in*, which for a bound orbit means
    // climbing outward. Getting this backwards is a mistake I have already made once, by
    // dividing by E0 instead of |E0|.
    CHECK(forward.relative_energy_error() > 0.0);

    SUBCASE("both conserve linear momentum, whatever else they do to the orbit")
    {
        // Linear momentum is a property of the force solver alone. Every integrator here
        // updates velocity as some weighted sum of accelerations times dt, and Newton's
        // third law makes the mass-weighted sum of accelerations exactly zero, so P comes
        // out unchanged no matter how badly the method is behaving. That is what makes it
        // useful: a violation points at the pair loop, never at the method.
        MESSAGE("linear momentum drift -- verlet " << verlet.relative_momentum_drift()
                                                   << ", forward "
                                                   << forward.relative_momentum_drift());

        CHECK(verlet.relative_momentum_drift() < 1e-12);
        CHECK(forward.relative_momentum_drift() < 1e-12);
    }

    SUBCASE("but angular momentum separates them, and more sharply than energy does")
    {
        // I expected this to behave like linear momentum and it does not, which is the most
        // interesting thing 0.2.3 turned up.
        //
        // Zero net torque makes the *continuous* system conserve L. Whether a discrete step
        // does depends on the order the position and velocity updates happen in:
        //
        //   semi-implicit  r(n+1) = r(n) + v(n+1)*dt, so r(n+1) x v(n+1) = r(n) x v(n+1),
        //                  and sum m r(n) x v(n+1) = L(n) + dt * torque = L(n). exact.
        //
        //   Verlet         the same cancellation twice, once around each half kick. exact.
        //
        //   forward Euler  r(n+1) x v(n+1) = (r + v dt) x (v + a dt), whose cross terms
        //                  leave dt^2 * sum m (v x a). not a torque, not zero.
        //
        // So L is conserved by the solver *and* the integrator jointly, not by the solver
        // alone as I had written in the header. Forward Euler loses more than half of it.
        MESSAGE("angular momentum drift -- verlet "
                << verlet.relative_angular_momentum_drift() << ", forward "
                << forward.relative_angular_momentum_drift());

        CHECK(verlet.relative_angular_momentum_drift() < 1e-12);
        CHECK(forward.relative_angular_momentum_drift() > 0.5);
    }
}

TEST_CASE("semi-implicit Euler conserves angular momentum exactly despite being first order")
{
    // The pairing that shows angular momentum is measuring something energy is not.
    // Semi-implicit Euler is a poor method by every accuracy measure here, 725x worse than
    // Verlet on position and four orders of magnitude worse on energy, and it still holds
    // angular momentum to roundoff, because its update ordering cancels the cross term
    // exactly. Accuracy and conservation are different questions.
    const BruteForceSolver solver;
    const double period =
        circular_period(orbitalis::kSunGM + orbitalis::kEarthGM, kAstronomicalUnit);

    const auto integrator = make_integrator("semi-implicit-euler", solver);
    REQUIRE(integrator != nullptr);

    System system = sun_earth();
    ConservationMonitor monitor;
    monitor.reset(system, solver);

    for (int i = 0; i < 20 * 365; ++i) {
        integrator->step(system, period / 365.0);
        monitor.sample(system, solver);
    }

    MESSAGE("semi-implicit -- energy " << monitor.worst_relative_energy_error() << ", L "
                                       << monitor.relative_angular_momentum_drift());

    CHECK(monitor.relative_angular_momentum_drift() < 1e-12);
    CHECK(monitor.worst_relative_energy_error() > 1e-5);  // and yet the energy is poor
}

TEST_CASE("the worst case never decreases, and reset clears it")
{
    const BruteForceSolver solver;
    const auto integrator = make_integrator("forward-euler", solver);
    REQUIRE(integrator != nullptr);

    System system = sun_earth();
    ConservationMonitor monitor;
    monitor.reset(system, solver);

    double previous_worst = 0.0;
    for (int i = 0; i < 500; ++i) {
        integrator->step(system, 3600.0 * 12.0);
        monitor.sample(system, solver);

        CHECK(monitor.worst_relative_energy_error() >= previous_worst);
        previous_worst = monitor.worst_relative_energy_error();
    }
    REQUIRE(previous_worst > 0.0);
    CHECK(monitor.samples() == 500);

    SUBCASE("reset re-baselines against the current state")
    {
        monitor.reset(system, solver);

        CHECK(monitor.samples() == 0);
        CHECK(monitor.worst_relative_energy_error() == 0.0);
        CHECK(monitor.relative_energy_error() == 0.0);

        // Re-baselining does not undo the drift, it just stops attributing it to whatever
        // comes next. The energy is still the drifted one.
        CHECK(monitor.baseline().total == doctest::Approx(measure(system, solver).total));
    }
}

TEST_CASE("a zero baseline energy is reported as no drift rather than as NaN")
{
    // A single motionless body has E = 0 exactly. Dividing by it would give a NaN that then
    // poisons the running worst case forever, and a NaN in a HUD is much harder to trace
    // back than a zero.
    const BruteForceSolver solver;

    System system;
    system.add(Body{1.0e30, 7.0e8, Vec3{}, Vec3{}}, "lone");

    ConservationMonitor monitor;
    monitor.reset(system, solver);
    REQUIRE(monitor.baseline().total == 0.0);

    system[0].position += Vec3{1.0e9, 0.0, 0.0};
    monitor.sample(system, solver);

    CHECK(monitor.relative_energy_error() == 0.0);
    CHECK_FALSE(std::isnan(monitor.worst_relative_energy_error()));
    CHECK(monitor.relative_momentum_drift() == 0.0);
}

TEST_CASE("momentum drift is scaled against a body, not against the total")
{
    // The total is zero by construction in a barycentric scenario, so a relative drift has
    // nothing of its own to divide by. Scaling against the largest single body momentum
    // asks how well the cancellation held, which is the question worth asking.
    const BruteForceSolver solver;
    const System system = sun_earth();
    const Diagnostics d = measure(system, solver);

    // Measured: |P| = 3.5e13 kg m/s against a single body's 1.78e29. The absolute figure
    // looks catastrophic and is 2e-16 of the quantity being cancelled.
    CHECK(d.momentum.length() > 1.0e12);                                 // not "small"
    CHECK(d.largest_body_momentum > 1.0e29);                             // this is the scale
    CHECK(d.momentum.length() / d.largest_body_momentum < 1e-14);        // and this is small

    // Earth's own momentum is the larger of the two, since the pair's momenta cancel and
    // are therefore equal in magnitude to within the mass ratio rounding.
    CHECK(d.largest_body_momentum ==
          doctest::Approx((system[1].mass * system[1].velocity).length()).epsilon(1e-9));
}
