#include <doctest/doctest.h>

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>

#include <algorithm>
#include <cmath>
#include <string_view>
#include <vector>

using orbitalis::approx_equal;
using orbitalis::Body;
using orbitalis::BruteForceSolver;
using orbitalis::IForceSolver;
using orbitalis::kAstronomicalUnit;
using orbitalis::kEarthMass;
using orbitalis::kGravitationalConstant;
using orbitalis::kSolarMass;
using orbitalis::System;
using orbitalis::Vec3;

namespace {

/// Runs a solver over a System and hands back the accelerations.
std::vector<Vec3> solve(const System& s, const BruteForceSolver& solver)
{
    std::vector<Vec3> accel(s.size());
    solver.compute_accelerations(s.bodies(), accel);
    return accel;
}

}  // namespace

// =======================================================================================
// degenerate cases
// =======================================================================================

TEST_CASE("an empty system produces no accelerations and does not hang")
{
    // The loop bound is written `i + 1 < n` rather than `i < n - 1` specifically because
    // n is unsigned and n - 1 would wrap to SIZE_MAX here. If that ever regresses, this
    // test hangs rather than failing, which is its own kind of signal.
    const System s;
    const BruteForceSolver solver;

    std::vector<Vec3> accel;
    solver.compute_accelerations(s.bodies(), accel);

    CHECK(accel.empty());
}

TEST_CASE("a lone body does not accelerate itself")
{
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{1.0, 2.0, 3.0}, Vec3{}});

    const auto accel = solve(s, BruteForceSolver{});

    REQUIRE(accel.size() == 1);
    CHECK(accel[0] == Vec3{});
}

TEST_CASE("accelerations are overwritten, not accumulated")
{
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    s.add(Body{kEarthMass, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});

    const BruteForceSolver solver;

    // Pre-fill with nonsense to prove it gets cleared.
    std::vector<Vec3> accel(2, Vec3{1e9, 1e9, 1e9});
    solver.compute_accelerations(s.bodies(), accel);
    const Vec3 first = accel[1];

    // And calling twice into the same buffer gives the same answer, not double.
    solver.compute_accelerations(s.bodies(), accel);
    CHECK(accel[1] == first);
}

// =======================================================================================
// the two-body case, against numbers computed independently
// =======================================================================================

TEST_CASE("Sun and Earth at one AU produce the textbook accelerations")
{
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}}, "Sun");
    s.add(Body{kEarthMass, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}}, "Earth");

    const auto accel = solve(s, BruteForceSolver{});
    REQUIRE(accel.size() == 2);

    // a = GM / r², worked out separately from the IAU GM values: 5.9300835190e-03 m/s²
    // on the Earth and 1.7810944552e-08 m/s² on the Sun.
    CHECK(accel[1].x == doctest::Approx(-5.9300835190e-03).epsilon(1e-9));
    CHECK(accel[0].x == doctest::Approx(1.7810944552e-08).epsilon(1e-9));

    SUBCASE("and they point at each other")
    {
        // Earth sits at +x, so it accelerates toward -x and the Sun toward +x.
        CHECK(accel[1].x < 0.0);
        CHECK(accel[0].x > 0.0);
        CHECK(accel[0].y == 0.0);
        CHECK(accel[0].z == 0.0);
    }
}

TEST_CASE("the acceleration ratio is the inverse mass ratio")
{
    // The clean statement of Newton's third law: equal and opposite *forces* mean the
    // lighter body gets the larger acceleration, by exactly the mass ratio. Using wildly
    // unequal masses on purpose, because equal masses would make this test unable to
    // fail (the 0.0.2 and 0.0.3 lesson).
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    s.add(Body{kEarthMass, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});

    const auto accel = solve(s, BruteForceSolver{});

    const double ratio = accel[1].length() / accel[0].length();
    CHECK(ratio == doctest::Approx(kSolarMass / kEarthMass).epsilon(1e-12));
}

TEST_CASE("momentum is conserved to machine precision")
{
    // Σ mᵢ·a⃗ᵢ must vanish: internal forces cancel in pairs, so an isolated system cannot
    // accelerate its own centre of mass. This is the single sharpest check that the
    // pair-once accumulation is right.
    //
    // "To machine precision" rather than "exactly", and the distinction is real. The
    // solver computes accelerations directly, so body i accumulates mⱼ·c⃗ and body j
    // accumulates -mᵢ·c⃗. Weighting those by mass gives mᵢ·(mⱼ·c⃗) against mⱼ·(mᵢ·c⃗),
    // which are the same value grouped differently, and floating-point multiplication is
    // commutative but not associative. Accumulating *forces* instead would cancel
    // bitwise, at the cost of n extra divisions to recover accelerations.
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    s.add(Body{kEarthMass, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});
    s.add(Body{1.898e27, 0.0, Vec3{0.0, 5.2 * kAstronomicalUnit, 0.0}, Vec3{}});
    s.add(Body{6.417e23, 0.0, Vec3{-1.52 * kAstronomicalUnit, 0.3e11, 1e10}, Vec3{}});

    const auto accel = solve(s, BruteForceSolver{});

    Vec3 net{};
    double scale = 0.0;
    for (std::size_t i = 0; i < s.size(); ++i) {
        const Vec3 term = s[i].mass * accel[i];
        net += term;
        scale = std::max(scale, term.length());
    }

    // Measured: 1.33e-16, which is about half a double epsilon (2.22e-16). So the
    // grouping argument above costs exactly one ULP and nothing more. The threshold is
    // kept two orders looser than that so it does not turn flaky under a different
    // compiler or optimisation level.
    CHECK(net.length() / scale < 1e-14);
}

// =======================================================================================
// the force law itself
// =======================================================================================

TEST_CASE("gravity falls off as the inverse square")
{
    const BruteForceSolver solver;

    auto accel_at = [&](double r) {
        System s;
        s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
        s.add(Body{1.0, 0.0, Vec3{r, 0.0, 0.0}, Vec3{}});
        return solve(s, solver)[1].length();
    };

    const double a1 = accel_at(kAstronomicalUnit);
    const double a2 = accel_at(2.0 * kAstronomicalUnit);
    const double a3 = accel_at(3.0 * kAstronomicalUnit);

    // Double the distance, quarter the pull. Triple it, one ninth.
    CHECK(a1 / a2 == doctest::Approx(4.0).epsilon(1e-12));
    CHECK(a1 / a3 == doctest::Approx(9.0).epsilon(1e-12));
}

TEST_CASE("acceleration is proportional to the attracting mass and independent of the attracted one")
{
    const BruteForceSolver solver;

    auto probe_accel = [&](double attracting, double probe) {
        System s;
        s.add(Body{attracting, 0.0, Vec3{}, Vec3{}});
        s.add(Body{probe, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});
        return solve(s, solver)[1].length();
    };

    SUBCASE("doubling the attracting mass doubles the acceleration")
    {
        CHECK(probe_accel(2.0 * kSolarMass, 1.0) / probe_accel(kSolarMass, 1.0)
              == doctest::Approx(2.0).epsilon(1e-12));
    }

    SUBCASE("the falling body's own mass does not matter")
    {
        // Galileo's result, and the equivalence principle. A grain of dust and a moon
        // fall toward the Sun at identical rates. This falls straight out of computing
        // accelerations rather than forces: the probe's mass never enters the expression.
        const double dust = probe_accel(kSolarMass, 1e-6);
        const double moon = probe_accel(kSolarMass, 7.35e22);
        CHECK(dust == doctest::Approx(moon).epsilon(1e-15));
    }
}

TEST_CASE("the circular-orbit speed at one AU matches Earth's actual speed")
{
    // Ties the force law to a number I can look up. For a circular orbit the gravitational
    // acceleration is exactly the centripetal acceleration, a = v²/r, so v = √(a·r).
    //
    // Earth's mean orbital speed is about 29,780 m/s. Getting ~29,785 out of this is the
    // right answer: Earth's orbit is mildly elliptical, so its actual speed varies either
    // side of the circular value.
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    s.add(Body{kEarthMass, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});

    const auto accel = solve(s, BruteForceSolver{});
    const double v_circular = std::sqrt(accel[1].length() * kAstronomicalUnit);

    CHECK(v_circular == doctest::Approx(29784.69).epsilon(1e-5));
    CHECK(v_circular > 29000.0);
    CHECK(v_circular < 30500.0);
}

// =======================================================================================
// many bodies
// =======================================================================================

TEST_CASE("accelerations superpose")
{
    // The pull from several bodies is the vector sum of the individual pulls. Nothing in
    // Newtonian gravity is nonlinear, and this checks the accumulation does not lose that.
    const BruteForceSolver solver;

    const Body a{kSolarMass, 0.0, Vec3{}, Vec3{}};
    const Body b{1.898e27, 0.0, Vec3{5.2 * kAstronomicalUnit, 0.0, 0.0}, Vec3{}};
    const Body c{5.683e26, 0.0, Vec3{0.0, 9.5 * kAstronomicalUnit, 2e11}, Vec3{}};

    // Target body, deliberately off-axis so no component is accidentally zero.
    const Body target{1.0, 0.0, Vec3{kAstronomicalUnit, 0.4e11, -0.2e11}, Vec3{}};

    auto accel_on_target = [&](std::vector<Body> others) {
        System s;
        s.add(target);
        for (const Body& o : others) {
            s.add(o);
        }
        return solve(s, solver)[0];
    };

    const Vec3 from_a = accel_on_target({a});
    const Vec3 from_b = accel_on_target({b});
    const Vec3 from_c = accel_on_target({c});
    const Vec3 from_all = accel_on_target({a, b, c});

    CHECK(approx_equal(from_all, from_a + from_b + from_c, 1e-18));
}

TEST_CASE("a body at the centre of a symmetric arrangement feels nothing")
{
    System s;
    s.add(Body{1.0, 0.0, Vec3{}, Vec3{}});  // the probe, at the centre
    s.add(Body{kSolarMass, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});
    s.add(Body{kSolarMass, 0.0, Vec3{-kAstronomicalUnit, 0.0, 0.0}, Vec3{}});
    s.add(Body{kSolarMass, 0.0, Vec3{0.0, kAstronomicalUnit, 0.0}, Vec3{}});
    s.add(Body{kSolarMass, 0.0, Vec3{0.0, -kAstronomicalUnit, 0.0}, Vec3{}});

    const auto accel = solve(s, BruteForceSolver{});

    CHECK(accel[0].length() < 1e-20);
}

// =======================================================================================
// softening
// =======================================================================================

TEST_CASE("without softening, coincident bodies blow up")
{
    // Pinning the thing softening exists to prevent. Two bodies at the same point give a
    // zero denominator, so the acceleration is non-finite, and because every body pulls
    // on every other, one NaN contaminates the whole system within a step.
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});

    const auto accel = solve(s, BruteForceSolver{0.0});

    CHECK_FALSE(accel[0].is_finite());
    CHECK_FALSE(accel[1].is_finite());
}

TEST_CASE("with softening, coincident bodies stay finite")
{
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});

    const auto accel = solve(s, BruteForceSolver{1e9});

    CHECK(accel[0].is_finite());
    CHECK(accel[1].is_finite());

    SUBCASE("and at exactly zero separation the softened force is zero, not merely small")
    {
        // d⃗ is the zero vector, so the whole expression vanishes regardless of the
        // denominator. Physically this is the Plummer sphere: at its own centre the
        // enclosed mass is zero, so there is nothing to pull you anywhere.
        CHECK(accel[0] == Vec3{});
    }
}

TEST_CASE("softening weakens gravity at close range")
{
    const double separation = 1e8;  // well inside the softening length below

    auto accel_with = [&](double eps) {
        System s;
        s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
        s.add(Body{1.0, 0.0, Vec3{separation, 0.0, 0.0}, Vec3{}});
        return solve(s, BruteForceSolver{eps})[1].length();
    };

    const double sharp = accel_with(0.0);
    const double soft = accel_with(1e9);

    CHECK(soft < sharp);
    CHECK(soft > 0.0);
}

TEST_CASE("softening is negligible when the separation is large compared to epsilon")
{
    // The rule that makes softening usable: ε must be small next to the distances that
    // matter. At one AU an ε of 1000 km changes the answer by about 7 parts in 10^11.
    auto accel_with = [&](double eps) {
        System s;
        s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
        s.add(Body{1.0, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});
        return solve(s, BruteForceSolver{eps})[1].length();
    };

    const double sharp = accel_with(0.0);
    const double soft = accel_with(1e6);

    CHECK(std::abs(soft - sharp) / sharp < 1e-9);
}

TEST_CASE("softening length is reported back")
{
    CHECK(BruteForceSolver{}.softening() == 0.0);
    CHECK(BruteForceSolver{1234.5}.softening() == 1234.5);
}

TEST_CASE("the solver identifies itself")
{
    // Used by the comparison harness at 0.2.6 and the benchmark tables at 0.3.6.
    CHECK(std::string_view{BruteForceSolver{}.name()} == "brute-force");
}

TEST_CASE("works through the IForceSolver interface")
{
    // The whole reason the interface exists: at 0.4.0 a Barnes-Hut solver replaces this
    // one and the integrators never notice. Exercise the virtual path so the base class
    // is not merely decorative.
    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    s.add(Body{kEarthMass, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});

    const BruteForceSolver concrete{};
    const IForceSolver& solver = concrete;

    std::vector<Vec3> accel(s.size());
    solver.compute_accelerations(s.bodies(), accel);

    CHECK(accel[1].x == doctest::Approx(-5.9300835190e-03).epsilon(1e-9));
    CHECK(std::string_view{solver.name()} == "brute-force");
}
