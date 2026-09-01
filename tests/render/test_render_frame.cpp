#include <doctest/doctest.h>

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/render/RenderFrame.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <cmath>
#include <vector>

using orbitalis::Body;
using orbitalis::kAstronomicalUnit;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::render::RenderFrame;
using orbitalis::render::Vec3f;
using orbitalis::scenarios::sun_earth;

namespace {

/// Distance between two render positions, in render units.
double separation(const Vec3f& a, const Vec3f& b)
{
    const double dx = static_cast<double>(a.x) - static_cast<double>(b.x);
    const double dy = static_cast<double>(a.y) - static_cast<double>(b.y);
    const double dz = static_cast<double>(a.z) - static_cast<double>(b.z);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

}  // namespace

// =======================================================================================
// the basic transform
// =======================================================================================

TEST_CASE("the focus maps to the render origin")
{
    const Vec3 focus{kAstronomicalUnit, 0.0, 0.0};
    const RenderFrame frame{focus, kAstronomicalUnit};

    const Vec3f origin = frame.to_render(focus);

    CHECK(origin.x == 0.0f);
    CHECK(origin.y == 0.0f);
    CHECK(origin.z == 0.0f);
}

TEST_CASE("one scale unit away maps to one render unit")
{
    const RenderFrame frame{Vec3{}, kAstronomicalUnit};

    const Vec3f p = frame.to_render(Vec3{kAstronomicalUnit, 0.0, 0.0});

    CHECK(p.x == doctest::Approx(1.0f));
    CHECK(p.y == 0.0f);
    CHECK(p.z == 0.0f);
}

TEST_CASE("the transform is a translation followed by a uniform scale")
{
    const RenderFrame frame{Vec3{3.0e11, -1.0e11, 5.0e10}, 1.0e10};

    SUBCASE("distances scale uniformly")
    {
        const Vec3 a{3.0e11, -1.0e11, 5.0e10};
        const Vec3 b{3.0e11 + 2.0e10, -1.0e11, 5.0e10};

        CHECK(separation(frame.to_render(a), frame.to_render(b)) == doctest::Approx(2.0));
    }

    SUBCASE("moving the focus does not change relative geometry")
    {
        const Vec3 a{1.0e11, 2.0e11, 3.0e11};
        const Vec3 b{1.5e11, 2.0e11, 3.0e11};

        RenderFrame moved = frame;
        moved.set_focus(Vec3{-7.0e11, 4.0e11, 0.0});

        CHECK(separation(frame.to_render(a), frame.to_render(b))
              == doctest::Approx(separation(moved.to_render(a), moved.to_render(b))));
    }
}

TEST_CASE("to_simulation inverts to_render")
{
    const RenderFrame frame{Vec3{kAstronomicalUnit, 0.0, 0.0}, kAstronomicalUnit};

    // A point a few million km from the focus: far enough to be a real offset, close
    // enough that float has plenty of relative precision for it.
    const Vec3 original{kAstronomicalUnit + 4.0e9, 2.0e9, -1.0e9};

    const Vec3 recovered = frame.to_simulation(frame.to_render(original));

    // Round-trip through float, so this cannot be exact. A part in 10^7 is float's
    // relative precision and is what "lossless widening of a lossy narrowing" gets you.
    CHECK(recovered.x == doctest::Approx(original.x).epsilon(1e-6));
    CHECK(recovered.y == doctest::Approx(original.y).epsilon(1e-6));
    CHECK(recovered.z == doctest::Approx(original.z).epsilon(1e-6));
}

// =======================================================================================
// the precision argument, measured
// =======================================================================================

TEST_CASE("the naive ordering loses a kilometre entirely")
{
    // The headline result of this step, and the reason RenderFrame is a class rather than
    // a divide.
    //
    // Camera parked on the Earth, one AU per render unit. Put a spacecraft 1 km away.
    // Both operands of the naive subtraction are then within 7e-9 of 1.0, and a float's
    // spacing near 1.0 is 1.2e-7. The two values round to the *same float*, so the
    // subtraction yields exactly zero and the spacecraft renders inside the planet.
    const Vec3 earth{kAstronomicalUnit, 0.0, 0.0};
    const RenderFrame frame{earth, kAstronomicalUnit};

    const Vec3 spacecraft{kAstronomicalUnit + 1000.0, 0.0, 0.0};

    const Vec3f naive = frame.to_render_naive(spacecraft);
    const Vec3f correct = frame.to_render(spacecraft);

    SUBCASE("naive collapses it onto the camera")
    {
        CHECK(naive.x == 0.0f);
    }

    SUBCASE("camera-relative keeps it")
    {
        // 1000 m / 1 AU = 6.684e-9 render units.
        CHECK(correct.x == doctest::Approx(1000.0 / kAstronomicalUnit).epsilon(1e-6));
        CHECK(correct.x > 0.0f);
    }
}

TEST_CASE("the naive ordering quantises to tens of kilometres")
{
    // Sweep a body outward from the camera and find the smallest offset the naive
    // transform can represent at all. Near 1.0 a float's spacing is 2^-23, so the answer
    // should be about 1.2e-7 AU, which is ~17.8 km.
    const Vec3 earth{kAstronomicalUnit, 0.0, 0.0};
    const RenderFrame frame{earth, kAstronomicalUnit};

    double smallest_visible = 0.0;
    for (double metres = 1.0; metres < 1.0e6; metres *= 1.05) {
        const Vec3f naive = frame.to_render_naive(Vec3{kAstronomicalUnit + metres, 0.0, 0.0});
        if (naive.x != 0.0f) {
            smallest_visible = metres;
            break;
        }
    }

    REQUIRE(smallest_visible > 0.0);

    // Somewhere in the region of 9-18 km depending on where rounding falls. Either way it
    // is kilometres, and the Earth is only 12,742 km across.
    CHECK(smallest_visible > 5000.0);
    CHECK(smallest_visible < 30000.0);

    SUBCASE("while camera-relative resolves a single metre there")
    {
        const Vec3f correct = frame.to_render(Vec3{kAstronomicalUnit + 1.0, 0.0, 0.0});
        CHECK(correct.x > 0.0f);
        CHECK(static_cast<double>(correct.x) * kAstronomicalUnit == doctest::Approx(1.0).epsilon(1e-3));
    }
}

TEST_CASE("precision degrades with distance from the camera, not from the origin")
{
    // The trade that camera-relative rendering makes, stated as a test. This is the right
    // way round: distant things are small on screen, so their error is subpixel.
    const RenderFrame frame{Vec3{}, kAstronomicalUnit};

    const double near = frame.resolution_at(1.0);
    const double far = frame.resolution_at(1000.0);

    CHECK(near < far);

    SUBCASE("and it doubles with every doubling of distance")
    {
        // Floating point spacing is constant within a binade and doubles across one, so
        // powers of two are where the relationship is exact.
        CHECK(frame.resolution_at(2.0) == doctest::Approx(2.0 * frame.resolution_at(1.0)));
        CHECK(frame.resolution_at(4.0) == doctest::Approx(4.0 * frame.resolution_at(1.0)));
    }

    SUBCASE("one AU per unit gives about 18 km of resolution at one unit out")
    {
        // 2^-23 AU = 1.19e-7 * 1.496e11 m.
        CHECK(near == doctest::Approx(17836.0).epsilon(1e-3));
    }

    SUBCASE("a tighter scale buys proportionally more precision")
    {
        // Zooming in is not just a camera change: it is a smaller metres_per_unit, and
        // that is what actually recovers detail.
        RenderFrame close{Vec3{}, 1.0e6};  // 1000 km per unit
        CHECK(close.resolution_at(1.0) < 1.0);  // sub-metre
    }
}

// =======================================================================================
// choosing a scale
// =======================================================================================

TEST_CASE("fit_scale sizes a system to the target extent")
{
    System s;
    s.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
    s.add(Body{1.0, 0.0, Vec3{5.0e11, 0.0, 0.0}, Vec3{}});

    const double scale = RenderFrame::fit_scale(s.bodies(), Vec3{}, 10.0);
    const RenderFrame frame{Vec3{}, scale};

    // The furthest body should land right at the target radius.
    const Vec3f furthest = frame.to_render(Vec3{5.0e11, 0.0, 0.0});
    CHECK(furthest.x == doctest::Approx(10.0).epsilon(1e-6));
}

TEST_CASE("fit_scale handles a real scenario")
{
    // The answer to "what should a world unit be": do not choose, derive it. A solar
    // system and a star cluster both arrive on screen at a usable size.
    const System s = sun_earth();
    const Vec3 focus = s.center_of_mass();

    const double scale = RenderFrame::fit_scale(s.bodies(), focus, 10.0);
    const RenderFrame frame{focus, scale};

    for (const Body& b : s.bodies()) {
        const Vec3f p = frame.to_render(b.position);
        CHECK(std::abs(p.x) <= 10.001f);
        CHECK(std::abs(p.y) <= 10.001f);
        CHECK(std::abs(p.z) <= 10.001f);
    }
}

TEST_CASE("fit_scale degenerate cases return something safe to divide by")
{
    SUBCASE("empty system")
    {
        CHECK(RenderFrame::fit_scale({}, Vec3{}, 10.0) == 1.0);
    }

    SUBCASE("every body on the focus")
    {
        System s;
        s.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
        s.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
        CHECK(RenderFrame::fit_scale(s.bodies(), Vec3{}, 10.0) == 1.0);
    }

    SUBCASE("nonsensical target")
    {
        System s;
        s.add(Body{1.0, 0.0, Vec3{1.0e11, 0.0, 0.0}, Vec3{}});
        CHECK(RenderFrame::fit_scale(s.bodies(), Vec3{}, 0.0) == 1.0);
        CHECK(RenderFrame::fit_scale(s.bodies(), Vec3{}, -5.0) == 1.0);
    }
}

// =======================================================================================
// robustness
// =======================================================================================

TEST_CASE("a bad scale is corrected rather than producing NaN")
{
    // Unlike Vec3::normalized(), which deliberately yields NaN because a zero-length
    // vector there means a bug, a zero scale here is a configuration mistake. Silently
    // filling every vertex with NaN would be a much worse outcome than clamping.
    SUBCASE("zero")
    {
        const RenderFrame frame{Vec3{}, 0.0};
        CHECK(frame.metres_per_unit() == 1.0);
        CHECK(frame.to_render(Vec3{5.0, 0.0, 0.0}).x == 5.0f);
    }

    SUBCASE("negative")
    {
        CHECK(RenderFrame{Vec3{}, -1000.0}.metres_per_unit() == 1.0);
    }

    SUBCASE("through the setter too")
    {
        RenderFrame frame{Vec3{}, kAstronomicalUnit};
        frame.set_metres_per_unit(0.0);
        CHECK(frame.metres_per_unit() == 1.0);
    }
}

TEST_CASE("a default-constructed frame is the identity")
{
    const RenderFrame frame;

    CHECK(frame.metres_per_unit() == 1.0);
    CHECK(frame.focus() == Vec3{});

    const Vec3f p = frame.to_render(Vec3{1.0, 2.0, 3.0});
    CHECK(p.x == 1.0f);
    CHECK(p.y == 2.0f);
    CHECK(p.z == 3.0f);
}
