#include <doctest/doctest.h>

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/Potential.hpp>
#include <orbitalis/physics/System.hpp>

#include <cmath>
#include <vector>

using orbitalis::Body;
using orbitalis::BruteForceSolver;
using orbitalis::gravitational_potential;
using orbitalis::kAstronomicalUnit;
using orbitalis::kGravitationalConstant;
using orbitalis::kSolarMass;
using orbitalis::kSunGM;
using orbitalis::System;
using orbitalis::Vec3;

TEST_CASE("the potential of a single mass is -GM/r")
{
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});

    const double phi = gravitational_potential(s.bodies(), Vec3{kAstronomicalUnit, 0.0, 0.0});

    CHECK(phi == doctest::Approx(-kSunGM / kAstronomicalUnit).epsilon(1e-9));
}

TEST_CASE("the potential is negative everywhere and rises toward zero with distance")
{
    // It is the work per unit mass needed to escape to infinity, so it is a debt that gets
    // smaller the further out you already are.
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});

    const double near = gravitational_potential(s.bodies(), Vec3{kAstronomicalUnit, 0.0, 0.0});
    const double far = gravitational_potential(s.bodies(), Vec3{10.0 * kAstronomicalUnit, 0.0, 0.0});

    CHECK(near < 0.0);
    CHECK(far < 0.0);
    CHECK(far > near);

    SUBCASE("and falls off as 1/r, not 1/r squared")
    {
        // The force is inverse-square; the potential it integrates to is inverse-linear.
        // Ten times further out is exactly one tenth the potential.
        CHECK(near / far == doctest::Approx(10.0).epsilon(1e-12));
    }
}

TEST_CASE("potentials superpose by plain addition")
{
    // Being a scalar field is the whole convenience: no directions to cancel.
    const Body a{kSolarMass, 0.0, Vec3{}, Vec3{}};
    const Body b{kSolarMass * 0.5, 0.0, Vec3{3.0 * kAstronomicalUnit, 0.0, 0.0}, Vec3{}};

    const Vec3 point{kAstronomicalUnit, 0.4 * kAstronomicalUnit, 0.0};

    const std::vector<Body> just_a{a};
    const std::vector<Body> just_b{b};
    const std::vector<Body> both{a, b};

    CHECK(gravitational_potential(both, point)
          == doctest::Approx(gravitational_potential(just_a, point)
                             + gravitational_potential(just_b, point)));
}

TEST_CASE("the midpoint between equal masses is a potential maximum along the axis")
{
    // Not zero: the potential there is deep, being close to two masses at once. But it is
    // the shallowest point on the line between them, which is why L1 sits near there.
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{-kAstronomicalUnit, 0.0, 0.0}, Vec3{}});
    s.add(Body{kSolarMass, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});

    const double middle = gravitational_potential(s.bodies(), Vec3{});
    const double offset = gravitational_potential(s.bodies(), Vec3{0.3 * kAstronomicalUnit, 0.0, 0.0});

    CHECK(middle < 0.0);
    CHECK(middle > offset);
}

TEST_CASE("the gradient of the potential is the acceleration")
{
    // The relationship that makes the drawn surface meaningful: the slope you can see is
    // the pull a body would feel. Checked numerically against the force solver.
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});

    const Vec3 point{2.0 * kAstronomicalUnit, 0.7 * kAstronomicalUnit, 0.0};
    const double h = 1.0e6;  // small next to an AU, large enough to avoid cancellation noise

    // Central difference: a = -grad(phi)
    auto phi_at = [&](const Vec3& p) { return gravitational_potential(s.bodies(), p); };

    const Vec3 numeric{
        -(phi_at(point + Vec3{h, 0.0, 0.0}) - phi_at(point - Vec3{h, 0.0, 0.0})) / (2.0 * h),
        -(phi_at(point + Vec3{0.0, h, 0.0}) - phi_at(point - Vec3{0.0, h, 0.0})) / (2.0 * h),
        -(phi_at(point + Vec3{0.0, 0.0, h}) - phi_at(point - Vec3{0.0, 0.0, h})) / (2.0 * h),
    };

    // What the solver says, using a massless probe at the same point.
    System probe_system;
    probe_system.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    probe_system.add(Body{1.0, 0.0, point, Vec3{}});

    std::vector<Vec3> accel(2);
    BruteForceSolver{}.compute_accelerations(probe_system.bodies(), accel);

    CHECK(numeric.x == doctest::Approx(accel[1].x).epsilon(1e-6));
    CHECK(numeric.y == doctest::Approx(accel[1].y).epsilon(1e-6));
    CHECK(std::abs(numeric.z) < 1e-12);
}

TEST_CASE("softening bounds the well instead of letting it diverge")
{
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});

    SUBCASE("unsoftened, the centre is skipped rather than returning -infinity")
    {
        // A grid sample landing exactly on a body would otherwise poison everything
        // downstream of it.
        const double phi = gravitational_potential(s.bodies(), Vec3{}, 0.0);
        CHECK(std::isfinite(phi));
    }

    SUBCASE("softened, the centre is the deepest finite value -GM/eps")
    {
        const double eps = 1.0e9;
        const double phi = gravitational_potential(s.bodies(), Vec3{}, eps);

        CHECK(std::isfinite(phi));
        CHECK(phi == doctest::Approx(-kSunGM / eps).epsilon(1e-9));
    }

    SUBCASE("and softening is negligible far away")
    {
        const double sharp = gravitational_potential(s.bodies(), Vec3{kAstronomicalUnit, 0.0, 0.0}, 0.0);
        const double soft = gravitational_potential(s.bodies(), Vec3{kAstronomicalUnit, 0.0, 0.0}, 1.0e6);
        CHECK(std::abs(soft - sharp) / std::abs(sharp) < 1e-9);
    }
}

TEST_CASE("an empty system has no potential")
{
    CHECK(gravitational_potential({}, Vec3{1.0, 2.0, 3.0}) == 0.0);
}

TEST_CASE("escape velocity falls out of the potential")
{
    // A nice check that the units are right: escaping means kinetic energy per unit mass
    // exactly cancels the potential debt, so v_escape = sqrt(2|phi|).
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});

    const double phi = gravitational_potential(s.bodies(), Vec3{kAstronomicalUnit, 0.0, 0.0});
    const double escape = std::sqrt(2.0 * std::abs(phi));

    // Escape velocity from Earth's orbit is about 42.1 km/s, and it is sqrt(2) times the
    // circular speed of 29.78 km/s, which is the standard result.
    CHECK(escape == doctest::Approx(42122.0).epsilon(1e-3));
    CHECK(escape / std::sqrt(kSunGM / kAstronomicalUnit) == doctest::Approx(std::sqrt(2.0)));
}
