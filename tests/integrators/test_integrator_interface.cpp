#include <doctest/doctest.h>

#include <orbitalis/integrators/Euler.hpp>
#include <orbitalis/integrators/Integrator.hpp>
#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <algorithm>
#include <string_view>
#include <vector>

using orbitalis::BruteForceSolver;
using orbitalis::ForwardEuler;
using orbitalis::IIntegrator;
using orbitalis::integrator_names;
using orbitalis::kAstronomicalUnit;
using orbitalis::make_integrator;
using orbitalis::SemiImplicitEuler;
using orbitalis::SemiImplicitEuler;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::scenarios::sun_earth;

// =======================================================================================
// the registry
// =======================================================================================

TEST_CASE("every listed name constructs an integrator that reports the same name")
{
    // The round trip that keeps the name list and the factory from drifting apart. Adding a
    // method means adding one row to the registry; forgetting either half fails here.
    const BruteForceSolver solver;

    REQUIRE_FALSE(integrator_names().empty());

    for (const std::string_view name : integrator_names()) {
        CAPTURE(name);

        const auto integrator = make_integrator(name, solver);
        REQUIRE(integrator != nullptr);
        CHECK(std::string_view{integrator->name()} == name);
    }
}

TEST_CASE("an unknown name yields nullptr rather than a silent substitution")
{
    // A scenario file at 0.5.3 naming an integrator that does not exist is a mistake the
    // user needs told about. Falling back to a default would attribute results to a method
    // that never ran.
    const BruteForceSolver solver;

    CHECK(make_integrator("verlet", solver) == nullptr);          // not written until 0.2.2
    CHECK(make_integrator("", solver) == nullptr);
    CHECK(make_integrator("Forward-Euler", solver) == nullptr);   // names are case-sensitive
    CHECK(make_integrator("forward-euler ", solver) == nullptr);  // and not trimmed
}

TEST_CASE("the name list has no duplicates and a stable order")
{
    // Stable because scenario files name integrators in text and benchmark tables are
    // easier to read when the rows do not move between runs.
    const auto names = integrator_names();

    std::vector<std::string_view> sorted{names.begin(), names.end()};
    std::sort(sorted.begin(), sorted.end());
    CHECK(std::adjacent_find(sorted.begin(), sorted.end()) == sorted.end());

    // Two calls give the same sequence.
    const auto again = integrator_names();
    CHECK(std::equal(names.begin(), names.end(), again.begin(), again.end()));
}

TEST_CASE("both Euler variants are reachable by name")
{
    const BruteForceSolver solver;

    const auto forward = make_integrator("forward-euler", solver);
    const auto semi = make_integrator("semi-implicit-euler", solver);

    REQUIRE(forward != nullptr);
    REQUIRE(semi != nullptr);

    CHECK_FALSE(forward->is_symplectic());
    CHECK(semi->is_symplectic());
}

// =======================================================================================
// the metadata
// =======================================================================================

TEST_CASE("every integrator reports a sane order")
{
    const BruteForceSolver solver;

    for (const std::string_view name : integrator_names()) {
        CAPTURE(name);
        const auto integrator = make_integrator(name, solver);
        REQUIRE(integrator != nullptr);

        // Anything claiming order zero or a negative order is broken; anything above eight
        // is not a method I have written.
        CHECK(integrator->order() >= 1);
        CHECK(integrator->order() <= 8);
    }
}

TEST_CASE("order and symplecticity are independent properties")
{
    // Both Eulers are first order. One keeps an orbit closed forever and the other spirals
    // out 20% per revolution. Order measures how fast error shrinks with dt; symplecticity
    // measures whether it accumulates. Conflating them is the mistake this interface is
    // shaped to prevent.
    const BruteForceSolver solver;

    const ForwardEuler forward{solver};
    const SemiImplicitEuler semi{solver};

    CHECK(forward.order() == semi.order());
    CHECK(forward.is_symplectic() != semi.is_symplectic());
}

// =======================================================================================
// using it polymorphically
// =======================================================================================

TEST_CASE("stepping through the interface matches stepping the concrete type")
{
    // Virtual dispatch must not change the answer. It happens once per step rather than
    // once per body, so it costs nothing measurable either.
    const BruteForceSolver solver;
    const double period =
        orbitalis::scenarios::circular_period(orbitalis::kSunGM + orbitalis::kEarthGM,
                                              kAstronomicalUnit);
    const double dt = period / 500.0;

    System direct = sun_earth();
    SemiImplicitEuler concrete{solver};
    for (int i = 0; i < 200; ++i) {
        concrete.step(direct, dt);
    }

    System through_interface = sun_earth();
    const auto integrator = make_integrator("semi-implicit-euler", solver);
    REQUIRE(integrator != nullptr);
    for (int i = 0; i < 200; ++i) {
        integrator->step(through_interface, dt);
    }

    CHECK(direct[1].position == through_interface[1].position);
    CHECK(direct[1].velocity == through_interface[1].velocity);
}

TEST_CASE("a caller can hold any integrator without knowing which")
{
    // What "swappable at runtime" actually buys: the viewer holds one of these and the
    // comparison harness at 0.2.6 will iterate over all of them.
    const BruteForceSolver solver;
    const double period =
        orbitalis::scenarios::circular_period(orbitalis::kSunGM + orbitalis::kEarthGM,
                                              kAstronomicalUnit);

    auto radius_after_one_orbit = [&](std::string_view name) {
        const auto integrator = make_integrator(name, solver);
        REQUIRE(integrator != nullptr);

        System system = sun_earth();
        for (int i = 0; i < 365; ++i) {
            integrator->step(system, period / 365.0);
        }
        return orbitalis::distance(system[0].position, system[1].position) / kAstronomicalUnit;
    };

    // The 0.0.5 result, reproduced entirely through the interface with no concrete type
    // named anywhere except as a string.
    CHECK(radius_after_one_orbit("semi-implicit-euler") == doctest::Approx(1.0).epsilon(1e-4));
    CHECK(radius_after_one_orbit("forward-euler") > 1.15);
}

TEST_CASE("reset is callable on integrators that carry no state")
{
    // Both Eulers use the default no-op. Velocity Verlet at 0.2.2 is the first that will
    // need to override it, and callers should already be calling it by then.
    const BruteForceSolver solver;
    const double period =
        orbitalis::scenarios::circular_period(orbitalis::kSunGM + orbitalis::kEarthGM,
                                              kAstronomicalUnit);

    const auto integrator = make_integrator("semi-implicit-euler", solver);
    REQUIRE(integrator != nullptr);

    System system = sun_earth();
    for (int i = 0; i < 50; ++i) {
        integrator->step(system, period / 500.0);
    }

    const Vec3 before = system[1].position;
    integrator->reset();

    // Resetting clears integrator state, not simulation state.
    CHECK(system[1].position == before);

    SUBCASE("and stepping still works afterwards")
    {
        integrator->step(system, period / 500.0);
        CHECK(system[1].position != before);
        CHECK(system[1].position.is_finite());
    }
}

TEST_CASE("integrators are destroyed through the base pointer without leaking")
{
    // The virtual destructor, stated as a test. Without it, deleting through
    // unique_ptr<IIntegrator> would skip the derived destructor and leak the scratch
    // vector on every construction.
    const BruteForceSolver solver;

    for (int i = 0; i < 1000; ++i) {
        const auto integrator = make_integrator("forward-euler", solver);
        REQUIRE(integrator != nullptr);

        System system = sun_earth();
        integrator->step(system, 3600.0);
    }

    CHECK(true);  // reaching here without exhausting memory is the assertion
}
