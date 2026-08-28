#include <doctest/doctest.h>

#include <orbitalis/math/Vec3.hpp>

#include <cmath>
#include <limits>

using orbitalis::Vec3;
using orbitalis::approx_equal;
using orbitalis::cross;
using orbitalis::distance;
using orbitalis::distance_squared;
using orbitalis::dot;

// ---------------------------------------------------------------------------------
// Compile-time checks. These cost nothing at runtime and they fail the BUILD rather
// than a test run, which is the strongest guarantee available. They also prove the
// constexpr claim in the header is real: if any of these operations quietly stopped
// being usable in a constant expression, this file would not compile.
// ---------------------------------------------------------------------------------

static_assert(Vec3{}.x == 0.0 && Vec3{}.y == 0.0 && Vec3{}.z == 0.0,
              "Vec3{} must zero-initialise");

static_assert(Vec3{1.0, 2.0, 3.0} + Vec3{4.0, 5.0, 6.0} == Vec3{5.0, 7.0, 9.0});
static_assert(Vec3{4.0, 5.0, 6.0} - Vec3{1.0, 2.0, 3.0} == Vec3{3.0, 3.0, 3.0});
static_assert(Vec3{1.0, 2.0, 3.0} * 2.0 == Vec3{2.0, 4.0, 6.0});
static_assert(2.0 * Vec3{1.0, 2.0, 3.0} == Vec3{2.0, 4.0, 6.0});
static_assert(-Vec3{1.0, -2.0, 3.0} == Vec3{-1.0, 2.0, -3.0});
static_assert(dot(Vec3{1.0, 2.0, 3.0}, Vec3{4.0, -5.0, 6.0}) == 12.0);
static_assert(cross(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}) == Vec3{0.0, 0.0, 1.0});
static_assert(Vec3{3.0, 4.0, 0.0}.length_squared() == 25.0);
static_assert(Vec3{1.0, 2.0, 3.0}[1] == 2.0);

// ---------------------------------------------------------------------------------

TEST_CASE("aggregate initialisation and defaults")
{
    const Vec3 zero{};
    CHECK(zero.x == 0.0);
    CHECK(zero.y == 0.0);
    CHECK(zero.z == 0.0);

    const Vec3 v{1.5, -2.5, 3.5};
    CHECK(v.x == 1.5);
    CHECK(v.y == -2.5);
    CHECK(v.z == 3.5);
}

TEST_CASE("indexed access")
{
    SUBCASE("reads every axis")
    {
        const Vec3 v{7.0, 8.0, 9.0};
        CHECK(v[0] == 7.0);
        CHECK(v[1] == 8.0);
        CHECK(v[2] == 9.0);
    }

    SUBCASE("writes through the non-const overload")
    {
        Vec3 v{};
        v[0] = 1.0;
        v[1] = 2.0;
        v[2] = 3.0;
        CHECK(v == Vec3{1.0, 2.0, 3.0});
    }

    SUBCASE("agrees with named members")
    {
        // The octree at 0.3.2 will select child octants by looping over axes, so these
        // two access paths must never disagree.
        const Vec3 v{-1.0, 4.0, 0.5};
        CHECK(v[0] == v.x);
        CHECK(v[1] == v.y);
        CHECK(v[2] == v.z);
    }
}

TEST_CASE("compound assignment")
{
    Vec3 v{1.0, 2.0, 3.0};

    v += Vec3{1.0, 1.0, 1.0};
    CHECK(v == Vec3{2.0, 3.0, 4.0});

    v -= Vec3{0.5, 0.5, 0.5};
    CHECK(v == Vec3{1.5, 2.5, 3.5});

    v *= 2.0;
    CHECK(v == Vec3{3.0, 5.0, 7.0});

    v /= 2.0;
    CHECK(v == Vec3{1.5, 2.5, 3.5});
}

TEST_CASE("addition is commutative and has an identity")
{
    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{-4.0, 5.5, 0.25};

    CHECK(a + b == b + a);
    CHECK(a + Vec3{} == a);
    CHECK(a - a == Vec3{});
}

TEST_CASE("division is not reciprocal multiplication")
{
    // This is the test that pins down the decision documented on operator/=.
    //
    // Dividing a value by itself must give exactly 1.0. Multiplying by the reciprocal
    // does not: 1.0/49.0 is inexact, and 49.0 * (1.0/49.0) comes out as
    // 0.9999999999999999. So if anyone ever "optimises" operator/= into a reciprocal
    // multiply, this fails immediately.
    const Vec3 v{49.0, 49.0, 49.0};
    const Vec3 divided = v / 49.0;

    CHECK(divided.x == 1.0);
    CHECK(divided.y == 1.0);
    CHECK(divided.z == 1.0);

    // And the general property: each component matches plain scalar division exactly.
    const Vec3 w{7.0, 10.0, 100.0};
    const Vec3 q = w / 3.0;

    CHECK(q.x == 7.0 / 3.0);
    CHECK(q.y == 10.0 / 3.0);
    CHECK(q.z == 100.0 / 3.0);
}

TEST_CASE("dot product")
{
    SUBCASE("known value")
    {
        CHECK(dot(Vec3{1.0, 2.0, 3.0}, Vec3{4.0, -5.0, 6.0}) == 12.0);
    }

    SUBCASE("orthogonal vectors give zero")
    {
        CHECK(dot(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 1.0, 0.0}) == 0.0);
        CHECK(dot(Vec3{1.0, 0.0, 0.0}, Vec3{0.0, 0.0, 1.0}) == 0.0);
    }

    SUBCASE("is commutative")
    {
        const Vec3 a{1.5, -2.0, 0.25};
        const Vec3 b{3.0, 4.0, -5.0};
        CHECK(dot(a, b) == dot(b, a));
    }

    SUBCASE("dot with self is length squared")
    {
        const Vec3 v{3.0, 4.0, 12.0};
        CHECK(dot(v, v) == v.length_squared());
    }
}

TEST_CASE("cross product")
{
    const Vec3 ex{1.0, 0.0, 0.0};
    const Vec3 ey{0.0, 1.0, 0.0};
    const Vec3 ez{0.0, 0.0, 1.0};

    SUBCASE("is right-handed")
    {
        CHECK(cross(ex, ey) == ez);
        CHECK(cross(ey, ez) == ex);
        CHECK(cross(ez, ex) == ey);
    }

    SUBCASE("is anticommutative")
    {
        const Vec3 a{1.0, 2.0, 3.0};
        const Vec3 b{4.0, 5.0, 6.0};
        CHECK(cross(a, b) == -cross(b, a));
    }

    SUBCASE("cross with self is zero")
    {
        const Vec3 v{1.5, -2.0, 7.0};
        CHECK(cross(v, v) == Vec3{});
    }

    SUBCASE("result is orthogonal to both inputs")
    {
        const Vec3 a{1.0, 2.0, 3.0};
        const Vec3 b{-4.0, 5.0, 6.0};
        const Vec3 c = cross(a, b);

        CHECK(dot(c, a) == doctest::Approx(0.0));
        CHECK(dot(c, b) == doctest::Approx(0.0));
    }

    SUBCASE("satisfies the BAC-CAB identity")
    {
        // a x (b x c) == b(a.c) - c(a.b). A genuine correctness check: it fails for
        // almost any sign error or transposed term in the cross product.
        const Vec3 a{1.0, 2.0, 3.0};
        const Vec3 b{-4.0, 5.0, 6.0};
        const Vec3 c{7.0, -8.0, 9.0};

        const Vec3 lhs = cross(a, cross(b, c));
        const Vec3 rhs = b * dot(a, c) - c * dot(a, b);

        CHECK(approx_equal(lhs, rhs, 1e-9));
    }
}

TEST_CASE("length and length_squared")
{
    SUBCASE("3-4-5 triangle in a plane")
    {
        CHECK(Vec3{3.0, 4.0, 0.0}.length() == 5.0);
        CHECK(Vec3{3.0, 4.0, 0.0}.length_squared() == 25.0);
    }

    SUBCASE("Pythagorean quadruples come out exact")
    {
        // 1^2 + 2^2 + 2^2 = 3^2, and 2^2 + 3^2 + 6^2 = 7^2. Both are exactly
        // representable, so these can be checked without any tolerance.
        CHECK(Vec3{1.0, 2.0, 2.0}.length() == 3.0);
        CHECK(Vec3{2.0, 3.0, 6.0}.length() == 7.0);
    }

    SUBCASE("length is unaffected by sign")
    {
        CHECK(Vec3{-3.0, -4.0, 0.0}.length() == 5.0);
    }

    SUBCASE("zero vector has zero length")
    {
        CHECK(Vec3{}.length() == 0.0);
        CHECK(Vec3{}.length_squared() == 0.0);
    }
}

TEST_CASE("normalized")
{
    SUBCASE("produces a unit vector")
    {
        const Vec3 v{3.0, 4.0, 12.0};  // length 13
        const Vec3 u = v.normalized();

        CHECK(u.length() == doctest::Approx(1.0));
        CHECK(approx_equal(u, Vec3{3.0 / 13.0, 4.0 / 13.0, 12.0 / 13.0}));
    }

    SUBCASE("preserves direction")
    {
        const Vec3 v{2.0, 0.0, 0.0};
        CHECK(approx_equal(v.normalized(), Vec3{1.0, 0.0, 0.0}));
    }

    SUBCASE("an already-unit vector is unchanged")
    {
        CHECK(approx_equal(Vec3{0.0, 1.0, 0.0}.normalized(), Vec3{0.0, 1.0, 0.0}));
    }

    SUBCASE("normalizing zero yields non-finite components, as documented")
    {
        // Pinning the documented behaviour rather than pretending it is safe. 0/0 is
        // NaN, so the result is detectable with is_finite() instead of silently being
        // a plausible-looking vector.
        const Vec3 u = Vec3{}.normalized();
        CHECK_FALSE(u.is_finite());
    }
}

TEST_CASE("is_finite")
{
    CHECK(Vec3{1.0, 2.0, 3.0}.is_finite());
    CHECK(Vec3{}.is_finite());

    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();

    CHECK_FALSE(Vec3{inf, 0.0, 0.0}.is_finite());
    CHECK_FALSE(Vec3{0.0, nan, 0.0}.is_finite());
    CHECK_FALSE(Vec3{0.0, 0.0, -inf}.is_finite());
}

TEST_CASE("distance")
{
    const Vec3 a{1.0, 2.0, 3.0};
    const Vec3 b{4.0, 6.0, 3.0};  // 3-4-5 again, offset

    CHECK(distance(a, b) == 5.0);
    CHECK(distance_squared(a, b) == 25.0);

    SUBCASE("is symmetric")
    {
        CHECK(distance(a, b) == distance(b, a));
        CHECK(distance_squared(a, b) == distance_squared(b, a));
    }

    SUBCASE("distance to self is zero")
    {
        CHECK(distance(a, a) == 0.0);
    }
}

TEST_CASE("approx_equal")
{
    CHECK(approx_equal(Vec3{1.0, 2.0, 3.0}, Vec3{1.0, 2.0, 3.0}));
    CHECK(approx_equal(Vec3{1.0, 0.0, 0.0}, Vec3{1.0 + 1e-15, 0.0, 0.0}));
    CHECK_FALSE(approx_equal(Vec3{1.0, 0.0, 0.0}, Vec3{1.1, 0.0, 0.0}));

    SUBCASE("tolerance is honoured")
    {
        CHECK(approx_equal(Vec3{1.0, 0.0, 0.0}, Vec3{1.05, 0.0, 0.0}, 0.1));
        CHECK_FALSE(approx_equal(Vec3{1.0, 0.0, 0.0}, Vec3{1.05, 0.0, 0.0}, 0.01));
    }
}

TEST_CASE("exact equality is exact")
{
    // operator== compares with ==, deliberately. This documents that it does NOT
    // tolerate rounding, so nobody reaches for it on computed values by mistake.
    const Vec3 a{0.1 + 0.2, 0.0, 0.0};
    const Vec3 b{0.3, 0.0, 0.0};

    CHECK(a != b);                    // 0.1 + 0.2 != 0.3 in binary floating point
    CHECK(approx_equal(a, b));        // but they are equal to within tolerance
}

// ---------------------------------------------------------------------------------
// A first look at the numbers this project will actually deal with. Not a physics
// test (there is no physics yet) but a check that Vec3 behaves at solar-system
// magnitudes, which is where a float type would already have fallen apart.
// ---------------------------------------------------------------------------------

TEST_CASE("survives astronomical magnitudes")
{
    const double au = 1.495978707e11;  // metres
    const Vec3 earth{au, 0.0, 0.0};

    CHECK(earth.length() == doctest::Approx(au));

    SUBCASE("a one-metre step is still representable")
    {
        // The whole argument for double over float. At this magnitude a float's
        // spacing is about 16 km, so adding one metre would be a no-op. A double
        // resolves about 31 micrometres here.
        const Vec3 nudged = earth + Vec3{1.0, 0.0, 0.0};
        CHECK(nudged.x != earth.x);
        CHECK(nudged.x - earth.x == 1.0);
    }

    SUBCASE("orbital velocity times a small timestep still moves the body")
    {
        const Vec3 velocity{0.0, 29780.0, 0.0};  // m/s, roughly Earth's
        const double dt = 0.1;                   // seconds

        const Vec3 moved = earth + velocity * dt;
        CHECK(moved.y == doctest::Approx(2978.0));
        CHECK(moved != earth);
    }
}
