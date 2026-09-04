#include <doctest/doctest.h>

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/render/Picking.hpp>

#include <vector>

using orbitalis::Vec3;
using orbitalis::render::pick_nearest;
using orbitalis::render::ray_sphere_distance;

// =======================================================================================
// ray against one sphere
// =======================================================================================

TEST_CASE("a ray straight at a sphere hits its near surface")
{
    // Sphere of radius 1 centred 10 along +Z. The near surface is at 9.
    const auto hit = ray_sphere_distance(Vec3{}, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 10.0}, 1.0);

    REQUIRE(hit.has_value());
    CHECK(*hit == doctest::Approx(9.0));
}

TEST_CASE("a ray pointing away misses")
{
    const auto hit = ray_sphere_distance(Vec3{}, Vec3{0.0, 0.0, -1.0}, Vec3{0.0, 0.0, 10.0}, 1.0);
    CHECK_FALSE(hit.has_value());
}

TEST_CASE("a ray passing beside the sphere misses")
{
    // Offset by 2 on a sphere of radius 1.
    const auto hit = ray_sphere_distance(Vec3{2.0, 0.0, 0.0}, Vec3{0.0, 0.0, 1.0},
                                         Vec3{0.0, 0.0, 10.0}, 1.0);
    CHECK_FALSE(hit.has_value());
}

TEST_CASE("a grazing ray is a hit and a just-missing one is not")
{
    // The discriminant changes sign right at the radius, so this is where the maths is
    // most likely to be subtly wrong.
    const Vec3 direction{0.0, 0.0, 1.0};
    const Vec3 centre{0.0, 0.0, 10.0};

    CHECK(ray_sphere_distance(Vec3{0.999, 0.0, 0.0}, direction, centre, 1.0).has_value());
    CHECK_FALSE(ray_sphere_distance(Vec3{1.001, 0.0, 0.0}, direction, centre, 1.0).has_value());

    SUBCASE("exactly tangent lands at the closest approach")
    {
        const auto hit = ray_sphere_distance(Vec3{1.0, 0.0, 0.0}, direction, centre, 1.0);
        REQUIRE(hit.has_value());
        CHECK(*hit == doctest::Approx(10.0).epsilon(1e-6));
    }
}

TEST_CASE("a ray starting inside the sphere hits the far surface")
{
    // So clicking while the camera is inside a body still selects it rather than falling
    // through to whatever is behind.
    const auto hit = ray_sphere_distance(Vec3{}, Vec3{0.0, 0.0, 1.0}, Vec3{}, 5.0);

    REQUIRE(hit.has_value());
    CHECK(*hit == doctest::Approx(5.0));
}

TEST_CASE("the direction need not be normalised, and t scales with its length")
{
    // t is measured in units of the direction vector, which is worth pinning because
    // callers hand over whatever the projection gave them.
    const auto unit = ray_sphere_distance(Vec3{}, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 10.0}, 1.0);
    const auto doubled = ray_sphere_distance(Vec3{}, Vec3{0.0, 0.0, 2.0}, Vec3{0.0, 0.0, 10.0}, 1.0);

    REQUIRE(unit.has_value());
    REQUIRE(doubled.has_value());
    CHECK(*doubled == doctest::Approx(*unit / 2.0));
}

TEST_CASE("degenerate inputs are rejected rather than returning NaN")
{
    SUBCASE("zero radius")
    {
        CHECK_FALSE(ray_sphere_distance(Vec3{}, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 10.0}, 0.0)
                        .has_value());
    }

    SUBCASE("negative radius")
    {
        CHECK_FALSE(ray_sphere_distance(Vec3{}, Vec3{0.0, 0.0, 1.0}, Vec3{0.0, 0.0, 10.0}, -1.0)
                        .has_value());
    }

    SUBCASE("zero-length direction is not a ray")
    {
        CHECK_FALSE(ray_sphere_distance(Vec3{}, Vec3{}, Vec3{0.0, 0.0, 10.0}, 1.0).has_value());
    }
}

// =======================================================================================
// picking among several
// =======================================================================================

TEST_CASE("pick_nearest returns the closest hit along the ray")
{
    // Three spheres in a line. The ray should select the near one, not the first in the
    // array or the biggest.
    const std::vector<Vec3> centres{
        Vec3{0.0, 0.0, 30.0},
        Vec3{0.0, 0.0, 10.0},
        Vec3{0.0, 0.0, 20.0},
    };
    const std::vector<double> radii{3.0, 1.0, 2.0};

    const auto picked = pick_nearest(Vec3{}, Vec3{0.0, 0.0, 1.0}, centres, radii);

    REQUIRE(picked.has_value());
    CHECK(*picked == 1);
}

TEST_CASE("pick_nearest ignores spheres the ray misses")
{
    const std::vector<Vec3> centres{
        Vec3{50.0, 0.0, 5.0},   // far off to the side
        Vec3{0.0, 0.0, 20.0},   // on the ray
    };
    const std::vector<double> radii{1.0, 1.0};

    const auto picked = pick_nearest(Vec3{}, Vec3{0.0, 0.0, 1.0}, centres, radii);

    REQUIRE(picked.has_value());
    CHECK(*picked == 1);
}

TEST_CASE("a small near body beats a large far one")
{
    // The behaviour that matters when clicking a planet in front of a star. Sorting by
    // radius or by array order would both get this wrong.
    const std::vector<Vec3> centres{
        Vec3{0.0, 0.0, 40.0},
        Vec3{0.0, 0.0, 5.0},
    };
    const std::vector<double> radii{10.0, 0.2};

    const auto picked = pick_nearest(Vec3{}, Vec3{0.0, 0.0, 1.0}, centres, radii);

    REQUIRE(picked.has_value());
    CHECK(*picked == 1);
}

TEST_CASE("pick_nearest finds nothing when nothing is hit")
{
    const std::vector<Vec3> centres{Vec3{0.0, 50.0, 10.0}};
    const std::vector<double> radii{1.0};

    CHECK_FALSE(pick_nearest(Vec3{}, Vec3{0.0, 0.0, 1.0}, centres, radii).has_value());
}

TEST_CASE("pick_nearest handles empty and mismatched inputs")
{
    SUBCASE("empty")
    {
        CHECK_FALSE(pick_nearest(Vec3{}, Vec3{0.0, 0.0, 1.0}, {}, {}).has_value());
    }

    SUBCASE("mismatched lengths use the shorter one rather than reading past the end")
    {
        const std::vector<Vec3> centres{Vec3{0.0, 0.0, 10.0}, Vec3{0.0, 0.0, 20.0}};
        const std::vector<double> radii{1.0};  // one short

        const auto picked = pick_nearest(Vec3{}, Vec3{0.0, 0.0, 1.0}, centres, radii);
        REQUIRE(picked.has_value());
        CHECK(*picked == 0);
    }
}

TEST_CASE("a tiny body is still pickable when drawn at its floor size")
{
    // The reason pick_nearest takes drawn radii rather than physical ones. Earth's true
    // radius in render units is around 2.6e-4, which is an impossible click target; the
    // three-pixel floor from 0.1.3 makes it roughly 0.05 units, which is not.
    const std::vector<Vec3> centres{Vec3{0.0, 0.0, 6.0}};

    const std::vector<double> physical{2.555e-4};
    const std::vector<double> drawn{0.05};

    const Vec3 slightly_off{0.01, 0.0, 0.0};

    CHECK_FALSE(pick_nearest(slightly_off, Vec3{0.0, 0.0, 1.0}, centres, physical).has_value());
    CHECK(pick_nearest(slightly_off, Vec3{0.0, 0.0, 1.0}, centres, drawn).has_value());
}

// =======================================================================================
// selection cycling
// =======================================================================================

TEST_CASE("cycle_selection walks the bodies and returns to nothing")
{
    using orbitalis::render::cycle_selection;

    // The full loop for a three-body system.
    auto s = cycle_selection(std::nullopt, 3);
    REQUIRE(s.has_value());
    CHECK(*s == 0);

    s = cycle_selection(s, 3);
    REQUIRE(s.has_value());
    CHECK(*s == 1);

    s = cycle_selection(s, 3);
    REQUIRE(s.has_value());
    CHECK(*s == 2);

    s = cycle_selection(s, 3);
    CHECK_FALSE(s.has_value());  // past the last one, back to the system frame

    s = cycle_selection(s, 3);
    REQUIRE(s.has_value());
    CHECK(*s == 0);  // and round again
}

TEST_CASE("cycle_selection handles a single body")
{
    using orbitalis::render::cycle_selection;

    const auto first = cycle_selection(std::nullopt, 1);
    REQUIRE(first.has_value());
    CHECK(*first == 0);

    CHECK_FALSE(cycle_selection(first, 1).has_value());
}

TEST_CASE("cycle_selection on an empty system selects nothing")
{
    using orbitalis::render::cycle_selection;

    CHECK_FALSE(cycle_selection(std::nullopt, 0).has_value());
    CHECK_FALSE(cycle_selection(std::size_t{0}, 0).has_value());
}

TEST_CASE("a stale index from a larger system restarts the cycle")
{
    using orbitalis::render::cycle_selection;

    // Following body 7 when the system now has two bodies. Clamping would be a lie about
    // which body is selected, so it starts over instead.
    const auto s = cycle_selection(std::size_t{7}, 2);
    REQUIRE(s.has_value());
    CHECK(*s == 0);
}
