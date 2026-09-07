#include <doctest/doctest.h>

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/render/RenderFrame.hpp>
#include <orbitalis/render/Trail.hpp>

#include <cmath>
#include <limits>

using orbitalis::Body;
using orbitalis::kAstronomicalUnit;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::render::RenderFrame;
using orbitalis::render::Trail;
using orbitalis::render::TrailSet;
using orbitalis::render::Vec3f;

// =======================================================================================
// the ring buffer
// =======================================================================================

TEST_CASE("a new trail is empty")
{
    const Trail trail{16};

    CHECK(trail.empty());
    CHECK(trail.size() == 0);
    CHECK(trail.capacity() == 16);
    CHECK_FALSE(trail.full());
}

TEST_CASE("points come back in chronological order")
{
    Trail trail{8};

    for (int i = 0; i < 5; ++i) {
        trail.push(Vec3{static_cast<double>(i), 0.0, 0.0});
    }

    REQUIRE(trail.size() == 5);
    for (std::size_t i = 0; i < 5; ++i) {
        CHECK(trail[i].x == doctest::Approx(static_cast<double>(i)));
    }

    CHECK(trail.oldest().x == doctest::Approx(0.0));
    CHECK(trail.newest().x == doctest::Approx(4.0));
}

TEST_CASE("the buffer wraps and drops the oldest point")
{
    Trail trail{4};

    for (int i = 0; i < 10; ++i) {
        trail.push(Vec3{static_cast<double>(i), 0.0, 0.0});
    }

    REQUIRE(trail.size() == 4);
    CHECK(trail.full());

    // Only 6, 7, 8, 9 survive, still oldest-first.
    CHECK(trail.oldest().x == doctest::Approx(6.0));
    CHECK(trail.newest().x == doctest::Approx(9.0));
    for (std::size_t i = 0; i < 4; ++i) {
        CHECK(trail[i].x == doctest::Approx(static_cast<double>(6 + i)));
    }
}

TEST_CASE("order survives many wraps")
{
    // The indexing has two cases, filling and wrapped, and the wrapped one is where an
    // off-by-one hides. Push a prime number of points through a small buffer so head_ lands
    // somewhere awkward.
    Trail trail{7};

    for (int i = 0; i < 1000; ++i) {
        trail.push(Vec3{static_cast<double>(i), 0.0, 0.0});
    }

    REQUIRE(trail.size() == 7);
    for (std::size_t i = 0; i < 7; ++i) {
        CHECK(trail[i].x == doctest::Approx(static_cast<double>(993 + i)));
    }
}

TEST_CASE("clear empties without changing capacity")
{
    Trail trail{8};
    for (int i = 0; i < 8; ++i) {
        trail.push(Vec3{static_cast<double>(i), 0.0, 0.0});
    }

    trail.clear();

    CHECK(trail.empty());
    CHECK(trail.capacity() == 8);

    SUBCASE("and it still records correctly afterwards")
    {
        trail.push(Vec3{99.0, 0.0, 0.0});
        REQUIRE(trail.size() == 1);
        CHECK(trail.newest().x == doctest::Approx(99.0));
        CHECK(trail.oldest().x == doctest::Approx(99.0));
    }
}

TEST_CASE("a zero-capacity trail accepts nothing")
{
    // A legitimate way to say "do not trail this body", which matters at 0.4.0 where
    // trailing 100,000 of them is not an option.
    Trail trail{0};

    trail.push(Vec3{1.0, 2.0, 3.0});

    CHECK(trail.empty());
    CHECK(trail.size() == 0);
    CHECK_FALSE(trail.sample(Vec3{1.0, 2.0, 3.0}, 0.0));
}

TEST_CASE("set_capacity discards rather than partially keeping")
{
    // A trail missing its middle is worse than no trail: it draws a straight line across a
    // gap that the body never travelled.
    Trail trail{8};
    for (int i = 0; i < 8; ++i) {
        trail.push(Vec3{static_cast<double>(i), 0.0, 0.0});
    }

    trail.set_capacity(4);

    CHECK(trail.capacity() == 4);
    CHECK(trail.empty());
}

// =======================================================================================
// distance-based sampling
// =======================================================================================

TEST_CASE("sampling ignores points that have not moved far enough")
{
    Trail trail{16};

    CHECK(trail.sample(Vec3{0.0, 0.0, 0.0}, 10.0));   // first point always lands
    CHECK_FALSE(trail.sample(Vec3{5.0, 0.0, 0.0}, 10.0));
    CHECK_FALSE(trail.sample(Vec3{9.9, 0.0, 0.0}, 10.0));
    CHECK(trail.sample(Vec3{10.0, 0.0, 0.0}, 10.0));

    CHECK(trail.size() == 2);
}

TEST_CASE("separation is measured from the last kept point, not the last offered one")
{
    // Otherwise a body creeping forward in tiny steps would never record anything, because
    // each step is individually too small.
    Trail trail{16};

    trail.sample(Vec3{}, 10.0);
    for (int i = 1; i <= 20; ++i) {
        trail.sample(Vec3{static_cast<double>(i), 0.0, 0.0}, 10.0);
    }

    // Points at 0, 10, 20.
    REQUIRE(trail.size() == 3);
    CHECK(trail[0].x == doctest::Approx(0.0));
    CHECK(trail[1].x == doctest::Approx(10.0));
    CHECK(trail[2].x == doctest::Approx(20.0));
}

TEST_CASE("zero separation records everything")
{
    Trail trail{16};

    for (int i = 0; i < 5; ++i) {
        CHECK(trail.sample(Vec3{}, 0.0));
    }
    CHECK(trail.size() == 5);
}

TEST_CASE("non-finite positions are refused")
{
    // A NaN would poison the drawn line strip, and NaN comparisons are all false so the
    // separation check would let it straight through.
    Trail trail{16};

    const double nan = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(trail.sample(Vec3{nan, 0.0, 0.0}, 0.0));
    CHECK_FALSE(trail.sample(Vec3{std::numeric_limits<double>::infinity(), 0.0, 0.0}, 0.0));
    CHECK(trail.empty());
}

TEST_CASE("sampling measures true distance, not one axis")
{
    Trail trail{16};

    trail.sample(Vec3{}, 5.0);

    // 3-4-5: a displacement of 5, made of components smaller than 5.
    CHECK(trail.sample(Vec3{3.0, 4.0, 0.0}, 5.0));
    CHECK(trail.size() == 2);
}

// =======================================================================================
// the reason trails hold metres
// =======================================================================================

TEST_CASE("a trail means the same shape in any reference frame")
{
    // The whole argument for storing simulation coordinates. Record a path, then project it
    // through two very different frames and check the geometry survives.
    //
    // Storing render coordinates instead would leave every historical point expressed
    // against whatever focus was current when it was recorded, so switching the followed
    // body would smear the trail across the screen.
    Trail trail{8};
    for (int i = 0; i < 5; ++i) {
        trail.push(Vec3{kAstronomicalUnit + i * 1.0e9, 0.0, 0.0});
    }

    const RenderFrame system_frame{Vec3{}, kAstronomicalUnit};
    const RenderFrame follow_frame{Vec3{kAstronomicalUnit, 0.0, 0.0}, kAstronomicalUnit};

    auto span_of = [](const RenderFrame& frame, const Trail& t) {
        const Vec3f first = frame.to_render(t.oldest());
        const Vec3f last = frame.to_render(t.newest());
        return static_cast<double>(last.x) - static_cast<double>(first.x);
    };

    CHECK(span_of(system_frame, trail) == doctest::Approx(span_of(follow_frame, trail)));

    SUBCASE("and the follow frame puts the path right next to the origin")
    {
        CHECK(std::abs(follow_frame.to_render(trail.oldest()).x) < 0.001f);
        CHECK(std::abs(system_frame.to_render(trail.oldest()).x) > 0.9f);
    }
}

// =======================================================================================
// TrailSet
// =======================================================================================

TEST_CASE("a trail set keeps one trail per body")
{
    System system;
    system.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
    system.add(Body{1.0, 0.0, Vec3{100.0, 0.0, 0.0}, Vec3{}});

    TrailSet trails{system.size(), 32};
    REQUIRE(trails.size() == 2);

    trails.sample(system.bodies(), 0.0);

    CHECK(trails[0].size() == 1);
    CHECK(trails[1].size() == 1);
    CHECK(trails[1].newest().x == doctest::Approx(100.0));
}

TEST_CASE("resizing preserves the history of surviving trails")
{
    TrailSet trails{2, 32};

    System system;
    system.add(Body{1.0, 0.0, Vec3{1.0, 0.0, 0.0}, Vec3{}});
    system.add(Body{1.0, 0.0, Vec3{2.0, 0.0, 0.0}, Vec3{}});
    trails.sample(system.bodies(), 0.0);

    SUBCASE("growing appends empty trails")
    {
        trails.resize(4);
        CHECK(trails.size() == 4);
        CHECK(trails[0].size() == 1);  // kept
        CHECK(trails[3].empty());      // new
        CHECK(trails[3].capacity() == 32);
    }

    SUBCASE("shrinking drops the tail")
    {
        trails.resize(1);
        CHECK(trails.size() == 1);
        CHECK(trails[0].size() == 1);
    }

    SUBCASE("resizing to the same count is a no-op")
    {
        trails.resize(2);
        CHECK(trails[0].size() == 1);
        CHECK(trails[1].size() == 1);
    }
}

TEST_CASE("sampling a mismatched system uses the shorter side")
{
    TrailSet trails{2, 32};

    System system;
    system.add(Body{1.0, 0.0, Vec3{1.0, 0.0, 0.0}, Vec3{}});
    system.add(Body{1.0, 0.0, Vec3{2.0, 0.0, 0.0}, Vec3{}});
    system.add(Body{1.0, 0.0, Vec3{3.0, 0.0, 0.0}, Vec3{}});

    trails.sample(system.bodies(), 0.0);  // three bodies, two trails

    CHECK(trails.size() == 2);
    CHECK(trails[1].newest().x == doctest::Approx(2.0));
}

TEST_CASE("changing capacity applies to every trail")
{
    TrailSet trails{3, 32};
    trails.set_capacity_per_body(8);

    CHECK(trails.capacity_per_body() == 8);
    for (std::size_t i = 0; i < trails.size(); ++i) {
        CHECK(trails[i].capacity() == 8);
    }
}

TEST_CASE("clearing empties every trail but keeps the count")
{
    System system;
    system.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
    system.add(Body{1.0, 0.0, Vec3{}, Vec3{}});

    TrailSet trails{2, 32};
    trails.sample(system.bodies(), 0.0);
    trails.clear();

    CHECK(trails.size() == 2);
    CHECK(trails[0].empty());
    CHECK(trails[1].empty());
}

TEST_CASE("a long run stays bounded")
{
    // Trails run forever in a live viewer, so this is the property that matters most: the
    // memory does not grow, whatever happens.
    System system;
    system.add(Body{1.0, 0.0, Vec3{}, Vec3{}});

    TrailSet trails{1, 64};

    for (int i = 0; i < 100000; ++i) {
        system[0].position = Vec3{static_cast<double>(i), 0.0, 0.0};
        trails.sample(system.bodies(), 0.5);
    }

    CHECK(trails[0].size() == 64);
    CHECK(trails[0].newest().x == doctest::Approx(99999.0));
    CHECK(trails[0].oldest().x == doctest::Approx(99936.0));
}
