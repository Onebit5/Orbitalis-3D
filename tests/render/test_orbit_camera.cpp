#include <doctest/doctest.h>

#include <orbitalis/render/OrbitCamera.hpp>

#include <cmath>
#include <limits>
#include <numbers>

using orbitalis::render::OrbitCamera;
using orbitalis::render::Vec3f;

namespace {

double length(const Vec3f& v)
{
    return std::sqrt(static_cast<double>(v.x) * v.x + static_cast<double>(v.y) * v.y
                     + static_cast<double>(v.z) * v.z);
}

}  // namespace

// =======================================================================================
// the spherical placement
// =======================================================================================

TEST_CASE("the camera sits at its distance from the origin")
{
    // The target is always the render origin, so this is the defining property.
    const OrbitCamera camera{37.0, 21.0, 8.5};

    CHECK(length(camera.position()) == doctest::Approx(8.5).epsilon(1e-6));
}

TEST_CASE("the angle conventions are the ones documented")
{
    SUBCASE("azimuth 0 puts the camera on +Z")
    {
        const OrbitCamera camera{0.0, 0.0, 10.0};
        const Vec3f p = camera.position();
        CHECK(p.x == doctest::Approx(0.0).epsilon(1e-6));
        CHECK(p.y == doctest::Approx(0.0).epsilon(1e-6));
        CHECK(p.z == doctest::Approx(10.0).epsilon(1e-6));
    }

    SUBCASE("azimuth 90 puts it on +X")
    {
        const OrbitCamera camera{90.0, 0.0, 10.0};
        const Vec3f p = camera.position();
        CHECK(p.x == doctest::Approx(10.0).epsilon(1e-6));
        CHECK(std::abs(p.z) < 1e-5);
    }

    SUBCASE("azimuth 180 puts it on -Z")
    {
        const OrbitCamera camera{180.0, 0.0, 10.0};
        CHECK(camera.position().z == doctest::Approx(-10.0).epsilon(1e-6));
    }

    SUBCASE("elevation 0 keeps it in the XZ plane")
    {
        const OrbitCamera camera{123.0, 0.0, 10.0};
        CHECK(std::abs(camera.position().y) < 1e-5);
    }

    SUBCASE("positive elevation lifts it")
    {
        CHECK(OrbitCamera{0.0, 45.0, 10.0}.position().y > 0.0f);
        CHECK(OrbitCamera{0.0, -45.0, 10.0}.position().y < 0.0f);
    }

    SUBCASE("elevation 45 splits the distance the way trigonometry says")
    {
        const OrbitCamera camera{0.0, 45.0, 10.0};
        const Vec3f p = camera.position();
        CHECK(p.y == doctest::Approx(10.0 * std::sin(45.0 * std::numbers::pi / 180.0)));
        CHECK(p.z == doctest::Approx(10.0 * std::cos(45.0 * std::numbers::pi / 180.0)));
    }
}

TEST_CASE("distance is preserved under rotation")
{
    // Rotating must not translate. Worth pinning because it is exactly what goes wrong if
    // the trig is assembled slightly wrongly.
    OrbitCamera camera{0.0, 0.0, 12.0};

    for (int i = 0; i < 20; ++i) {
        camera.rotate(37.0, 7.0);
        CHECK(length(camera.position()) == doctest::Approx(12.0).epsilon(1e-6));
    }
}

// =======================================================================================
// rotation
// =======================================================================================

TEST_CASE("rotation accumulates")
{
    OrbitCamera camera{0.0, 0.0, 10.0};

    camera.rotate(30.0, 10.0);
    camera.rotate(30.0, 10.0);

    CHECK(camera.azimuth_degrees() == doctest::Approx(60.0));
    CHECK(camera.elevation_degrees() == doctest::Approx(20.0));
}

TEST_CASE("azimuth wraps rather than clamping")
{
    // Spinning all the way round is a legitimate thing to do with a mouse.
    OrbitCamera camera{350.0, 0.0, 10.0};
    camera.rotate(20.0, 0.0);

    CHECK(camera.azimuth_degrees() == doctest::Approx(10.0));

    SUBCASE("and backwards past zero")
    {
        OrbitCamera back{10.0, 0.0, 10.0};
        back.rotate(-20.0, 0.0);
        // std::fmod alone would give -10 here, because it keeps the dividend's sign.
        CHECK(back.azimuth_degrees() == doctest::Approx(350.0));
    }

    SUBCASE("and many turns at once")
    {
        OrbitCamera spun{0.0, 0.0, 10.0};
        spun.rotate(360.0 * 5.0 + 45.0, 0.0);
        CHECK(spun.azimuth_degrees() == doctest::Approx(45.0));
    }
}

TEST_CASE("elevation clamps short of the pole")
{
    // At exactly +-90 the view direction is parallel to the up vector and the view basis
    // degenerates. Every orbit camera clamps here.
    OrbitCamera camera{0.0, 0.0, 10.0};

    camera.rotate(0.0, 1000.0);
    CHECK(camera.elevation_degrees() == doctest::Approx(89.0));
    CHECK(camera.elevation_degrees() < 90.0);

    camera.rotate(0.0, -2000.0);
    CHECK(camera.elevation_degrees() == doctest::Approx(-89.0));
    CHECK(camera.elevation_degrees() > -90.0);

    SUBCASE("and the position stays well defined at the clamp")
    {
        OrbitCamera high{0.0, 89.0, 10.0};
        const Vec3f p = high.position();
        CHECK(std::isfinite(p.x));
        CHECK(std::isfinite(p.y));
        CHECK(std::isfinite(p.z));
        CHECK(length(p) == doctest::Approx(10.0).epsilon(1e-6));
        // Still a little horizontal offset left, which is what keeps the basis valid.
        CHECK(std::sqrt(p.x * p.x + p.z * p.z) > 0.0f);
    }
}

// =======================================================================================
// zoom
// =======================================================================================

TEST_CASE("zoom is multiplicative")
{
    // A scroll notch should mean the same relative change at every scale, otherwise zoom is
    // unusably coarse far out and unusably slow close in.
    OrbitCamera camera{0.0, 0.0, 100.0};

    camera.zoom(0.5);
    CHECK(camera.distance() == doctest::Approx(50.0));

    camera.zoom(0.5);
    CHECK(camera.distance() == doctest::Approx(25.0));

    camera.zoom(4.0);
    CHECK(camera.distance() == doctest::Approx(100.0));
}

TEST_CASE("zoom by one changes nothing")
{
    OrbitCamera camera{0.0, 0.0, 7.5};
    camera.zoom(1.0);
    CHECK(camera.distance() == doctest::Approx(7.5));
}

TEST_CASE("zoom respects the distance limits")
{
    OrbitCamera camera{0.0, 0.0, 10.0};

    SUBCASE("cannot be pushed through the target")
    {
        for (int i = 0; i < 100; ++i) {
            camera.zoom(0.5);
        }
        CHECK(camera.distance() == doctest::Approx(camera.limits().min_distance));
        CHECK(camera.distance() > 0.0);
    }

    SUBCASE("cannot be flung to infinity")
    {
        for (int i = 0; i < 100; ++i) {
            camera.zoom(2.0);
        }
        CHECK(camera.distance() == doctest::Approx(camera.limits().max_distance));
        CHECK(std::isfinite(camera.distance()));
    }
}

TEST_CASE("nonsense zoom factors are ignored rather than destroying the camera")
{
    OrbitCamera camera{0.0, 0.0, 10.0};

    camera.zoom(0.0);
    CHECK(camera.distance() == doctest::Approx(10.0));

    camera.zoom(-2.0);
    CHECK(camera.distance() == doctest::Approx(10.0));

    camera.zoom(std::numeric_limits<double>::quiet_NaN());
    CHECK(camera.distance() == doctest::Approx(10.0));
}

TEST_CASE("nonsense rotations are ignored")
{
    OrbitCamera camera{45.0, 20.0, 10.0};

    camera.rotate(std::numeric_limits<double>::quiet_NaN(), 0.0);
    CHECK(camera.azimuth_degrees() == doctest::Approx(45.0));

    camera.rotate(0.0, std::numeric_limits<double>::infinity());
    CHECK(camera.elevation_degrees() == doctest::Approx(20.0));
}

// =======================================================================================
// limits
// =======================================================================================

TEST_CASE("changing limits re-applies them immediately")
{
    // Otherwise tightening the limits leaves the camera somewhere they forbid, and it only
    // snaps back the next time the user happens to touch it.
    OrbitCamera camera{0.0, 80.0, 500.0};

    OrbitCamera::Limits limits;
    limits.min_distance = 1.0;
    limits.max_distance = 100.0;
    limits.max_elevation_degrees = 45.0;
    camera.set_limits(limits);

    CHECK(camera.distance() == doctest::Approx(100.0));
    CHECK(camera.elevation_degrees() == doctest::Approx(45.0));
}

TEST_CASE("broken limits are repaired instead of trusted")
{
    OrbitCamera camera;

    OrbitCamera::Limits limits;
    limits.min_distance = -5.0;
    limits.max_distance = -10.0;
    limits.max_elevation_degrees = 90.0;  // exactly the degenerate value
    camera.set_limits(limits);

    CHECK(camera.limits().min_distance > 0.0);
    CHECK(camera.limits().max_distance > camera.limits().min_distance);
    CHECK(camera.limits().max_elevation_degrees < 90.0);
    CHECK(camera.distance() > 0.0);
}
