#include <doctest/doctest.h>

#include <orbitalis/integrators/Euler.hpp>
#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <algorithm>
#include <cmath>

using orbitalis::approx_equal;
using orbitalis::Body;
using orbitalis::BruteForceSolver;
using orbitalis::distance;
using orbitalis::ForwardEuler;
using orbitalis::kAstronomicalUnit;
using orbitalis::kDay;
using orbitalis::kEarthGM;
using orbitalis::kSiderealYear;
using orbitalis::kSunGM;
using orbitalis::SemiImplicitEuler;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::scenarios::circular_period;
using orbitalis::scenarios::sun_earth;
using orbitalis::scenarios::sun_earth_drifting;

// =======================================================================================
// the period
// =======================================================================================

TEST_CASE("the computed year matches the real sidereal year")
{
    // The headline check for this step, and the one that caught a bad constant.
    //
    // A two-body circular orbit at exactly one AU, using the IAU standard gravitational
    // parameters, should take one sidereal year: 365.256363 days. Getting within a couple
    // of seconds of that means the force law, the constants and the period formula all
    // agree with reality rather than merely with each other.
    const double period = circular_period(kSunGM + kEarthGM, kAstronomicalUnit);

    CHECK(period / kDay == doctest::Approx(365.256350).epsilon(1e-9));

    SUBCASE("within a couple of seconds of the real thing")
    {
        CHECK(std::abs(period - kSiderealYear) < 5.0);
    }
}

TEST_CASE("period obeys Kepler's third law")
{
    // T² ∝ a³, so quadrupling the separation multiplies the period by 8.
    const double t1 = circular_period(kSunGM, kAstronomicalUnit);
    const double t4 = circular_period(kSunGM, 4.0 * kAstronomicalUnit);

    CHECK(t4 / t1 == doctest::Approx(8.0).epsilon(1e-12));

    SUBCASE("and the general form holds for an awkward ratio")
    {
        const double a = 2.7 * kAstronomicalUnit;
        const double t = circular_period(kSunGM, a);
        CHECK(t / t1 == doctest::Approx(std::pow(2.7, 1.5)).epsilon(1e-12));
    }
}

TEST_CASE("period uses the combined GM, not just the central body's")
{
    // Wrong for Sun and Earth by only a part in 300,000, but catastrophically wrong for a
    // near-equal-mass binary. Checking the formula uses what it says it uses.
    const double combined = circular_period(kSunGM + kEarthGM, kAstronomicalUnit);
    const double central_only = circular_period(kSunGM, kAstronomicalUnit);

    CHECK(combined < central_only);

    SUBCASE("equal masses halve nothing but shorten the period by sqrt(2)")
    {
        const double single = circular_period(kSunGM, kAstronomicalUnit);
        const double binary = circular_period(2.0 * kSunGM, kAstronomicalUnit);
        CHECK(single / binary == doctest::Approx(std::sqrt(2.0)).epsilon(1e-12));
    }
}

// =======================================================================================
// the barycentric scenario
// =======================================================================================

TEST_CASE("sun_earth is built in the barycentric frame")
{
    const System s = sun_earth();

    REQUIRE(s.size() == 2);
    CHECK(s.name(0) == "Sun");
    CHECK(s.name(1) == "Earth");

    SUBCASE("total momentum is zero")
    {
        // Not approximately zero because the velocities were split by the mass ratio on
        // purpose. Scaled against Earth's own momentum, which is the magnitude the
        // cancellation happens at.
        const double scale = (s[1].mass * s[1].velocity).length();
        CHECK(s.total_momentum().length() / scale < 1e-15);
    }

    SUBCASE("the barycentre sits at the origin")
    {
        CHECK(s.center_of_mass().length() < 1.0);  // within a metre, of 1.5e11
    }

    SUBCASE("the barycentre is not moving")
    {
        CHECK(s.center_of_mass_velocity().length() < 1e-12);
    }

    SUBCASE("the separation is one AU")
    {
        CHECK(distance(s[0].position, s[1].position)
              == doctest::Approx(kAstronomicalUnit).epsilon(1e-12));
    }
}

TEST_CASE("the Sun moves too")
{
    // The part the naive setup gets wrong. The Sun is not a fixed post: it circles the
    // barycentre 449 km from its own centre, at about 9 cm/s.
    const System s = sun_earth();

    CHECK(s[0].position.length() == doctest::Approx(449314.3).epsilon(1e-4));
    CHECK(s[0].velocity.length() == doctest::Approx(0.089458).epsilon(1e-4));

    SUBCASE("and it moves opposite to the Earth")
    {
        CHECK(s[0].velocity.y < 0.0);
        CHECK(s[1].velocity.y > 0.0);
    }

    SUBCASE("in proportion to the inverse mass ratio")
    {
        const double ratio = s[1].velocity.length() / s[0].velocity.length();
        CHECK(ratio == doctest::Approx(kSunGM / kEarthGM).epsilon(1e-9));
    }
}

TEST_CASE("sun_earth honours a different separation")
{
    const System s = sun_earth(5.2 * kAstronomicalUnit);

    CHECK(distance(s[0].position, s[1].position)
          == doctest::Approx(5.2 * kAstronomicalUnit).epsilon(1e-12));

    // Further out means slower: v = sqrt(GM/a).
    CHECK(s[1].velocity.length() < sun_earth()[1].velocity.length());
}

// =======================================================================================
// the drifting scenario, and the fix
// =======================================================================================

TEST_CASE("the naive setup carries net momentum")
{
    // Sun parked at the origin, Earth given the full orbital velocity. This is what
    // almost everyone writes first, and the pair slides across the universe forever.
    const System s = sun_earth_drifting();

    CHECK(s.total_momentum().length() > 0.0);
    CHECK(s.center_of_mass_velocity().length() == doctest::Approx(0.089458).epsilon(1e-4));

    SUBCASE("which is about 2800 km per year")
    {
        const double per_year = s.center_of_mass_velocity().length() * kSiderealYear;
        CHECK(per_year / 1000.0 == doctest::Approx(2823.0).epsilon(1e-3));
    }
}

TEST_CASE("remove_net_drift zeroes the drift")
{
    System s = sun_earth_drifting();
    REQUIRE(s.center_of_mass_velocity().length() > 0.01);

    s.remove_net_drift();

    const double scale = (s[1].mass * s[1].velocity).length();
    CHECK(s.total_momentum().length() / scale < 1e-15);
    CHECK(s.center_of_mass_velocity().length() < 1e-12);
}

TEST_CASE("remove_net_drift changes the frame, not the physics")
{
    // Subtracting a constant from every velocity is a change of inertial frame. Relative
    // velocities, and therefore the whole dynamics, are untouched.
    System s = sun_earth_drifting();
    const Vec3 relative_before = s[1].velocity - s[0].velocity;
    const Vec3 positions_before = s[1].position - s[0].position;

    s.remove_net_drift();

    CHECK(approx_equal(s[1].velocity - s[0].velocity, relative_before, 1e-9));
    CHECK(s[1].position - s[0].position == positions_before);
}

TEST_CASE("remove_net_drift is idempotent and safe on an empty system")
{
    System empty;
    empty.remove_net_drift();
    CHECK(empty.empty());

    System s = sun_earth_drifting();
    s.remove_net_drift();
    const Vec3 after_once = s[1].velocity;
    s.remove_net_drift();
    CHECK(approx_equal(s[1].velocity, after_once, 1e-9));
}

// =======================================================================================
// the milestone 0.1.0 test: does Earth actually come back?
// =======================================================================================

TEST_CASE("Earth returns to where it started after one year")
{
    // The whole point of milestone 0.1.0, stated as an assertion.
    //
    // One sidereal year split into a whole number of steps, so "one period" is exact and
    // the distance from the starting point measures integrator error and nothing else.
    const BruteForceSolver solver;
    const double period = circular_period(kSunGM + kEarthGM, kAstronomicalUnit);
    constexpr int kSteps = 365;
    const double dt = period / kSteps;

    System s = sun_earth();
    const Vec3 start = s[1].position - s[0].position;

    SemiImplicitEuler integrator{solver};
    for (int i = 0; i < kSteps; ++i) {
        integrator.step(s, dt);
    }

    const Vec3 finish = s[1].position - s[0].position;
    const double return_distance = (finish - start).length() / kAstronomicalUnit;

    // Measured: 0.001319 AU, about 197,000 km, on an orbit 940 million km around. That is
    // an error of roughly 0.02% of the path travelled, from a first-order method taking
    // day-long steps.
    CHECK(return_distance == doctest::Approx(0.001319).epsilon(1e-3));
    CHECK(return_distance < 0.01);

    SUBCASE("and the orbit has not changed size")
    {
        CHECK(finish.length() / kAstronomicalUnit == doctest::Approx(1.0).epsilon(1e-4));
    }

    SUBCASE("and it did not fly off into nowhere")
    {
        CHECK(finish.is_finite());
        CHECK(finish.length() < 2.0 * kAstronomicalUnit);
    }
}

TEST_CASE("forward Euler does not bring Earth back")
{
    // The control group, and the reason two integrators exist. The physics is identical;
    // only the method differs. If both failed, the force solver would be the suspect.
    const BruteForceSolver solver;
    const double period = circular_period(kSunGM + kEarthGM, kAstronomicalUnit);
    constexpr int kSteps = 365;

    System s = sun_earth();
    const Vec3 start = s[1].position - s[0].position;

    ForwardEuler integrator{solver};
    for (int i = 0; i < kSteps; ++i) {
        integrator.step(s, period / kSteps);
    }

    const Vec3 finish = s[1].position - s[0].position;
    const double return_distance = (finish - start).length() / kAstronomicalUnit;

    // 0.887 AU away from where it started, against semi-implicit's 0.0013. Same cost.
    CHECK(return_distance > 0.5);
    CHECK(finish.length() / kAstronomicalUnit == doctest::Approx(1.205782).epsilon(1e-4));
}

TEST_CASE("momentum survives a full orbit under both integrators")
{
    const BruteForceSolver solver;
    const double period = circular_period(kSunGM + kEarthGM, kAstronomicalUnit);
    constexpr int kSteps = 365;

    auto drift_after_orbit = [&](auto integrator_tag) {
        using Integrator = decltype(integrator_tag);
        System s = sun_earth();
        Integrator integrator{solver};
        for (int i = 0; i < kSteps; ++i) {
            integrator.step(s, period / kSteps);
        }
        double scale = 0.0;
        for (const Body& b : s.bodies()) {
            scale = std::max(scale, (b.mass * b.velocity).length());
        }
        return s.total_momentum().length() / scale;
    };

    // Measured at 3.3e-16 and 4.3e-17: machine precision, for both. Momentum conservation
    // is a property of the force solver, so even the bad integrator inherits it.
    CHECK(drift_after_orbit(ForwardEuler{solver}) < 1e-14);
    CHECK(drift_after_orbit(SemiImplicitEuler{solver}) < 1e-14);
}
