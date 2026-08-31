#include <doctest/doctest.h>

#include <orbitalis/integrators/Euler.hpp>
#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>

#include <algorithm>
#include <cmath>
#include <span>
#include <string_view>
#include <utility>

using orbitalis::approx_equal;
using orbitalis::Body;
using orbitalis::BruteForceSolver;
using orbitalis::distance;
using orbitalis::ForwardEuler;
using orbitalis::IForceSolver;
using orbitalis::kAstronomicalUnit;
using orbitalis::kDay;
using orbitalis::kGravitationalConstant;
using orbitalis::kSolarMass;
using orbitalis::SemiImplicitEuler;
using orbitalis::System;
using orbitalis::Vec3;

namespace {

/// A solver that ignores gravity entirely and applies the same acceleration to everything.
///
/// Exists so the integrators can be checked against closed-form answers. With a constant
/// acceleration the exact trajectory is x = x₀ + v₀t + ½at², so the integrator error can
/// be compared to an analytic formula rather than to another approximation. It also
/// exercises IForceSolver from outside the library, which is the interface's whole point.
class ConstantAccelerationSolver final : public IForceSolver
{
public:
    explicit ConstantAccelerationSolver(Vec3 a) noexcept : a_(a) {}

    void compute_accelerations(std::span<const Body> bodies,
                               std::span<Vec3> accelerations) const override
    {
        (void)bodies;
        std::fill(accelerations.begin(), accelerations.end(), a_);
    }

    [[nodiscard]] const char* name() const noexcept override { return "constant"; }

private:
    Vec3 a_;
};

/// Sun plus a 1 kg probe on a circular orbit at one AU.
///
/// The probe is light enough that the Sun's own acceleration is around 3e-33 m/s², so this
/// is effectively a fixed central mass and can be compared against the textbook two-body
/// result without worrying about barycentre motion.
System circular_orbit_system()
{
    const double v = std::sqrt(kGravitationalConstant * kSolarMass / kAstronomicalUnit);

    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}}, "Sun");
    s.add(Body{1.0, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{0.0, v, 0.0}}, "probe");
    return s;
}

double orbital_radius(const System& s)
{
    return distance(s[0].position, s[1].position);
}

/// One orbit at this radius takes 365.21 days, so 365 steps of one day is very nearly a
/// full revolution.
constexpr int kStepsPerOrbit = 365;

}  // namespace

// =======================================================================================
// exactness where these methods are exact
// =======================================================================================

TEST_CASE("with no forces a body travels in a straight line")
{
    // Both methods are exact here: with zero acceleration the velocity never changes, so
    // there is nothing for either ordering to get wrong.
    const ConstantAccelerationSolver none{Vec3{}};

    System s;
    s.add(Body{1.0, 0.0, Vec3{}, Vec3{3.0, -4.0, 12.0}});

    SUBCASE("forward Euler")
    {
        ForwardEuler integrator{none};
        for (int i = 0; i < 100; ++i) {
            integrator.step(s, 0.5);
        }
        CHECK(approx_equal(s[0].position, Vec3{150.0, -200.0, 600.0}, 1e-9));
        CHECK(approx_equal(s[0].velocity, Vec3{3.0, -4.0, 12.0}));
    }

    SUBCASE("semi-implicit Euler")
    {
        SemiImplicitEuler integrator{none};
        for (int i = 0; i < 100; ++i) {
            integrator.step(s, 0.5);
        }
        CHECK(approx_equal(s[0].position, Vec3{150.0, -200.0, 600.0}, 1e-9));
        CHECK(approx_equal(s[0].velocity, Vec3{3.0, -4.0, 12.0}));
    }
}

TEST_CASE("under constant acceleration both get velocity exactly right")
{
    // v = v₀ + at is linear in t, and both methods integrate it the same way, so the
    // velocity is exact for both regardless of ordering. All the error is in position.
    const ConstantAccelerationSolver gravity{Vec3{0.0, -10.0, 0.0}};

    System sf;
    sf.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
    System ss = sf;

    ForwardEuler fwd{gravity};
    SemiImplicitEuler semi{gravity};

    for (int i = 0; i < 100; ++i) {  // 100 steps of 0.1 s = 10 s
        fwd.step(sf, 0.1);
        semi.step(ss, 0.1);
    }

    // v = a·t = -10 · 10 = -100 m/s, both of them.
    CHECK(sf[0].velocity.y == doctest::Approx(-100.0).epsilon(1e-12));
    CHECK(ss[0].velocity.y == doctest::Approx(-100.0).epsilon(1e-12));
}

TEST_CASE("the two position errors are equal and opposite")
{
    // With v₀ = 0 and constant a, n steps of dt give in closed form:
    //
    //   exact           x = ½·a·(n·dt)²        = ½·a·n²·dt²
    //   forward Euler   x = a·dt²·n(n-1)/2     = exact - ½·a·n·dt²
    //   semi-implicit   x = a·dt²·n(n+1)/2     = exact + ½·a·n·dt²
    //
    // So forward lags and semi-implicit leads, by exactly the same amount. Nothing about
    // that is special to gravity; it is a property of which velocity the position update
    // uses.
    //
    // a = -10, dt = 0.1, n = 100, so t = 10 s:
    //   exact    = -500 m
    //   forward  = -495 m
    //   semi     = -505 m
    const ConstantAccelerationSolver gravity{Vec3{0.0, -10.0, 0.0}};

    System sf;
    sf.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
    System ss = sf;

    ForwardEuler fwd{gravity};
    SemiImplicitEuler semi{gravity};

    for (int i = 0; i < 100; ++i) {
        fwd.step(sf, 0.1);
        semi.step(ss, 0.1);
    }

    constexpr double exact = -500.0;

    CHECK(sf[0].position.y == doctest::Approx(-495.0).epsilon(1e-12));
    CHECK(ss[0].position.y == doctest::Approx(-505.0).epsilon(1e-12));

    SUBCASE("and so their average is the exact answer")
    {
        const double mean = 0.5 * (sf[0].position.y + ss[0].position.y);
        CHECK(mean == doctest::Approx(exact).epsilon(1e-12));
    }

    SUBCASE("both errors are half·a·n·dt², the first-order truncation term")
    {
        constexpr double predicted = 0.5 * 10.0 * 100.0 * 0.1 * 0.1;  // = 5 m
        CHECK(std::abs(sf[0].position.y - exact) == doctest::Approx(predicted).epsilon(1e-12));
        CHECK(std::abs(ss[0].position.y - exact) == doctest::Approx(predicted).epsilon(1e-12));
    }
}

// =======================================================================================
// the headline result: what happens to a real orbit
// =======================================================================================

TEST_CASE("forward Euler spirals a circular orbit outward")
{
    // The reason this integrator exists in the project at all. Over the step the body
    // genuinely curves, but forward Euler moves it along the straight line it was on at
    // the start of the step, which always lands it slightly too far out. The error is
    // one-directional, so it accumulates.
    System s = circular_orbit_system();
    const BruteForceSolver solver;
    ForwardEuler integrator{solver};

    REQUIRE(orbital_radius(s) == doctest::Approx(kAstronomicalUnit).epsilon(1e-12));

    for (int i = 0; i < kStepsPerOrbit; ++i) {
        integrator.step(s, kDay);
    }

    // Computed independently: 1.205620 AU after one orbit at dt = 1 day.
    const double radii = orbital_radius(s) / kAstronomicalUnit;
    CHECK(radii == doctest::Approx(1.205620).epsilon(1e-4));

    SUBCASE("and it keeps going, orbit after orbit")
    {
        for (int i = 0; i < 9 * kStepsPerOrbit; ++i) {
            integrator.step(s, kDay);
        }
        // 1.894430 AU after ten orbits. Not a wobble, a spiral.
        CHECK(orbital_radius(s) / kAstronomicalUnit == doctest::Approx(1.894430).epsilon(1e-3));
    }
}

TEST_CASE("forward Euler's radius never decreases")
{
    // Sharper than checking the endpoint: the error has a sign. A circular orbit under
    // this method never comes back in, not even briefly.
    System s = circular_orbit_system();
    const BruteForceSolver solver;
    ForwardEuler integrator{solver};

    double previous = orbital_radius(s);
    bool monotonic = true;

    for (int i = 0; i < kStepsPerOrbit; ++i) {
        integrator.step(s, kDay);
        const double r = orbital_radius(s);
        if (r < previous) {
            monotonic = false;
        }
        previous = r;
    }

    CHECK(monotonic);
}

TEST_CASE("semi-implicit Euler keeps the orbit closed")
{
    System s = circular_orbit_system();
    const BruteForceSolver solver;
    SemiImplicitEuler integrator{solver};

    for (int i = 0; i < kStepsPerOrbit; ++i) {
        integrator.step(s, kDay);
    }

    // 1.000038 AU: back within four parts in a hundred thousand, using an identical
    // timestep and an identical number of force evaluations to the run above.
    CHECK(orbital_radius(s) / kAstronomicalUnit == doctest::Approx(1.0).epsilon(1e-4));
}

TEST_CASE("semi-implicit Euler's error is bounded, not accumulating")
{
    // This is the symplectic property stated as directly as I can manage. The method does
    // not track the true circular orbit: it turns it into a slightly eccentric one that
    // breathes between 0.9915 and 1.0088 AU. But those bounds are the *same* after ten
    // orbits as after one. The error oscillates instead of growing.
    const BruteForceSolver solver;

    auto radius_bounds = [&](int orbits) {
        System s = circular_orbit_system();
        SemiImplicitEuler integrator{solver};

        double lo = orbital_radius(s);
        double hi = lo;
        for (int i = 0; i < orbits * kStepsPerOrbit; ++i) {
            integrator.step(s, kDay);
            const double r = orbital_radius(s);
            lo = std::min(lo, r);
            hi = std::max(hi, r);
        }
        return std::pair{lo / kAstronomicalUnit, hi / kAstronomicalUnit};
    };

    const auto [lo1, hi1] = radius_bounds(1);

    CHECK(lo1 == doctest::Approx(0.991547).epsilon(1e-4));
    CHECK(hi1 == doctest::Approx(1.008749).epsilon(1e-4));

    SUBCASE("the envelope converges and then stops moving entirely")
    {
        // Comparing one orbit against ten is not quite the right test, and finding out
        // why was the interesting part of this step. Those two disagree by about 2.5e-7,
        // which looks like slow growth but is not: 365 steps of one day is 365.21 days of
        // orbit, so the sampling phase creeps forward a little each revolution. Over more
        // orbits you simply sample nearer to the true extremum. It is a measurement
        // artefact, not the error growing.
        //
        // Once enough phases have been sampled the envelope stops moving altogether. Ten
        // orbits and fifty give bit-identical bounds, and that is the symplectic property
        // stated properly: the error is bounded forever, not merely small for a while.
        const auto [lo10, hi10] = radius_bounds(10);
        const auto [lo50, hi50] = radius_bounds(50);

        // Measured: the upper bound is bit-identical between 10 and 50 orbits; the lower
        // bound is still settling by 7e-9 relative, so 1e-7 covers it with room. That is
        // still the envelope holding steady to seven significant figures across fifty
        // revolutions, against forward Euler growing by 0.9 AU over ten.
        CHECK(lo50 == doctest::Approx(lo10).epsilon(1e-7));
        CHECK(hi50 == doctest::Approx(hi10).epsilon(1e-7));

        // And it never escapes a 1% band, over any of those durations.
        CHECK(lo50 > 0.99);
        CHECK(hi50 < 1.01);
    }
}

TEST_CASE("same cost, wildly different answers")
{
    // Both methods perform exactly one force evaluation per step and the same number of
    // arithmetic operations. The entire difference is which velocity the position update
    // reads. Stating it as a test because it is the single most useful thing in this step.
    const BruteForceSolver solver;

    System a = circular_orbit_system();
    System b = circular_orbit_system();

    ForwardEuler fwd{solver};
    SemiImplicitEuler semi{solver};

    for (int i = 0; i < kStepsPerOrbit; ++i) {
        fwd.step(a, kDay);
        semi.step(b, kDay);
    }

    const double error_fwd = std::abs(orbital_radius(a) - kAstronomicalUnit);
    const double error_semi = std::abs(orbital_radius(b) - kAstronomicalUnit);

    CHECK(error_semi * 1000.0 < error_fwd);
}

// =======================================================================================
// what Euler does get right
// =======================================================================================

TEST_CASE("both integrators conserve linear momentum")
{
    // Worth separating from energy, because it is a different kind of guarantee.
    //
    // Every step adds a⃗ᵢ·dt to each velocity, so the change in total momentum is
    // dt·Σ mᵢa⃗ᵢ, and that sum is zero because internal forces cancel in pairs. Momentum
    // conservation is therefore a property of the *force solver*, and holds for any
    // integrator, including a bad one. Energy conservation is a property of the
    // *integrator*, and forward Euler does not have it.
    //
    // Which means: if momentum ever drifts, the bug is in the force accumulation, not in
    // the integrator. That is a genuinely useful thing to be able to conclude.
    const BruteForceSolver solver;

    auto make_system = [] {
        System s;
        s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
        s.add(Body{5.9722e24, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{0.0, 29780.0, 0.0}});
        s.add(Body{1.898e27, 0.0, Vec3{0.0, 5.2 * kAstronomicalUnit, 0.0},
                   Vec3{-13070.0, 0.0, 0.0}});
        return s;
    };

    SUBCASE("forward Euler")
    {
        System s = make_system();
        const Vec3 p0 = s.total_momentum();
        ForwardEuler integrator{solver};
        for (int i = 0; i < 100; ++i) {
            integrator.step(s, kDay);
        }
        CHECK((s.total_momentum() - p0).length() / p0.length() < 1e-12);
    }

    SUBCASE("semi-implicit Euler")
    {
        System s = make_system();
        const Vec3 p0 = s.total_momentum();
        SemiImplicitEuler integrator{solver};
        for (int i = 0; i < 100; ++i) {
            integrator.step(s, kDay);
        }
        CHECK((s.total_momentum() - p0).length() / p0.length() < 1e-12);
    }
}

// =======================================================================================
// mechanics
// =======================================================================================

TEST_CASE("a zero timestep changes nothing")
{
    const BruteForceSolver solver;
    System s = circular_orbit_system();
    const Vec3 p = s[1].position;
    const Vec3 v = s[1].velocity;

    ForwardEuler fwd{solver};
    SemiImplicitEuler semi{solver};
    fwd.step(s, 0.0);
    semi.step(s, 0.0);

    CHECK(s[1].position == p);
    CHECK(s[1].velocity == v);
}

TEST_CASE("an empty system steps without complaint")
{
    const BruteForceSolver solver;
    System s;

    ForwardEuler fwd{solver};
    SemiImplicitEuler semi{solver};
    fwd.step(s, kDay);
    semi.step(s, kDay);

    CHECK(s.empty());
}

TEST_CASE("the scratch buffer follows the system when it grows")
{
    // The integrator caches an accelerations vector across steps so a long run does not
    // allocate every step. It has to notice when the system changes size.
    const BruteForceSolver solver;
    ForwardEuler integrator{solver};

    System s;
    s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
    integrator.step(s, kDay);

    s.add(Body{1.0, 0.0, Vec3{kAstronomicalUnit, 0.0, 0.0}, Vec3{}});
    integrator.step(s, kDay);

    // The new body must have felt the Sun on that second step.
    CHECK(s[1].velocity.x < 0.0);

    SUBCASE("and when it shrinks")
    {
        s.clear();
        s.add(Body{kSolarMass, 0.0, Vec3{}, Vec3{}});
        integrator.step(s, kDay);
        CHECK(s.size() == 1);
    }
}

TEST_CASE("integrators identify themselves")
{
    const BruteForceSolver solver;
    CHECK(std::string_view{ForwardEuler{solver}.name()} == "forward-euler");
    CHECK(std::string_view{SemiImplicitEuler{solver}.name()} == "semi-implicit-euler");
}
