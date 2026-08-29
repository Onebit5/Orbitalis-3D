#include <doctest/doctest.h>

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/System.hpp>

using orbitalis::approx_equal;
using orbitalis::Body;
using orbitalis::BodyId;
using orbitalis::System;
using orbitalis::Vec3;

namespace {

// Real values, because the tests below are more interesting when the numbers mean
// something. SI units throughout.
constexpr double kSunMass = 1.98892e30;    // kg
constexpr double kSunRadius = 6.957e8;     // m
constexpr double kEarthMass = 5.9722e24;   // kg
constexpr double kAu = 1.495978707e11;     // m

}  // namespace

TEST_CASE("a new System is empty")
{
    const System s;

    CHECK(s.empty());
    CHECK(s.size() == 0);
    CHECK(s.total_mass() == 0.0);
    CHECK(s.center_of_mass() == Vec3{});
    CHECK(s.total_momentum() == Vec3{});
    CHECK(s.center_of_mass_velocity() == Vec3{});
}

TEST_CASE("add returns sequential ids and grows the system")
{
    System s;

    const BodyId a = s.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
    const BodyId b = s.add(Body{2.0, 0.0, Vec3{}, Vec3{}});
    const BodyId c = s.add(Body{3.0, 0.0, Vec3{}, Vec3{}});

    CHECK(a == 0);
    CHECK(b == 1);
    CHECK(c == 2);
    CHECK(s.size() == 3);
    CHECK_FALSE(s.empty());
}

TEST_CASE("add stores the body unchanged")
{
    System s;
    const Body earth{kEarthMass, 6.371e6, Vec3{kAu, 0.0, 0.0}, Vec3{0.0, 29780.0, 0.0}};

    const BodyId id = s.add(earth);

    CHECK(s[id].mass == earth.mass);
    CHECK(s[id].radius == earth.radius);
    CHECK(s[id].position == earth.position);
    CHECK(s[id].velocity == earth.velocity);
}

TEST_CASE("names")
{
    System s;
    const BodyId sun = s.add(Body{kSunMass, kSunRadius, Vec3{}, Vec3{}}, "Sun");
    const BodyId rock = s.add(Body{1.0, 1.0, Vec3{}, Vec3{}});

    CHECK(s.name(sun) == "Sun");

    SUBCASE("a body added without a name has an empty one")
    {
        CHECK(s.name(rock).empty());
    }

    SUBCASE("names can be set afterwards")
    {
        s.set_name(rock, "Ceres");
        CHECK(s.name(rock) == "Ceres");
        CHECK(s.name(sun) == "Sun");  // and the neighbour is untouched
    }
}

TEST_CASE("bodies are mutable through the system")
{
    System s;
    const BodyId id = s.add(Body{1.0, 0.0, Vec3{}, Vec3{}});

    SUBCASE("through operator[]")
    {
        s[id].position = Vec3{1.0, 2.0, 3.0};
        CHECK(s[id].position == Vec3{1.0, 2.0, 3.0});
    }

    SUBCASE("through the span, which is how the integrators will do it")
    {
        for (Body& b : s.bodies()) {
            b.velocity = Vec3{0.0, 100.0, 0.0};
        }
        CHECK(s[id].velocity == Vec3{0.0, 100.0, 0.0});
    }
}

TEST_CASE("the bodies span sees the whole system contiguously")
{
    System s;
    s.add(Body{1.0, 0.0, Vec3{1.0, 0.0, 0.0}, Vec3{}});
    s.add(Body{2.0, 0.0, Vec3{2.0, 0.0, 0.0}, Vec3{}});

    const auto view = s.bodies();
    REQUIRE(view.size() == 2);
    CHECK(view[0].mass == 1.0);
    CHECK(view[1].mass == 2.0);
    CHECK(view.data() == &s[0]);
}

TEST_CASE("clear empties everything")
{
    System s;
    s.add(Body{1.0, 0.0, Vec3{}, Vec3{}}, "one");
    s.add(Body{2.0, 0.0, Vec3{}, Vec3{}}, "two");

    s.clear();

    CHECK(s.empty());
    CHECK(s.size() == 0);
    CHECK(s.total_mass() == 0.0);

    SUBCASE("and ids restart after clearing")
    {
        CHECK(s.add(Body{3.0, 0.0, Vec3{}, Vec3{}}, "three") == 0);
        CHECK(s.name(0) == "three");
    }
}

TEST_CASE("reserve changes capacity but not size")
{
    System s;
    s.reserve(1000);

    CHECK(s.empty());
    CHECK(s.size() == 0);

    // And it still behaves normally afterwards.
    CHECK(s.add(Body{1.0, 0.0, Vec3{}, Vec3{}}) == 0);
    CHECK(s.size() == 1);
}

TEST_CASE("total_mass sums every body")
{
    System s;
    s.add(Body{1.0, 0.0, Vec3{}, Vec3{}});
    s.add(Body{2.5, 0.0, Vec3{}, Vec3{}});
    s.add(Body{0.5, 0.0, Vec3{}, Vec3{}});

    CHECK(s.total_mass() == 4.0);
}

TEST_CASE("center_of_mass")
{
    SUBCASE("two equal masses balance at the midpoint")
    {
        System s;
        s.add(Body{1.0, 0.0, Vec3{-2.0, 0.0, 0.0}, Vec3{}});
        s.add(Body{1.0, 0.0, Vec3{2.0, 0.0, 0.0}, Vec3{}});

        CHECK(approx_equal(s.center_of_mass(), Vec3{}));
    }

    SUBCASE("unequal masses balance nearer the heavier one")
    {
        // 1 kg at x=0 and 2 kg at x=3 balance at x=2, not x=1.5.
        System s;
        s.add(Body{1.0, 0.0, Vec3{0.0, 0.0, 0.0}, Vec3{}});
        s.add(Body{2.0, 0.0, Vec3{3.0, 0.0, 0.0}, Vec3{}});

        CHECK(approx_equal(s.center_of_mass(), Vec3{2.0, 0.0, 0.0}));
    }

    SUBCASE("a single body is its own centre of mass")
    {
        System s;
        s.add(Body{7.0, 0.0, Vec3{1.0, 2.0, 3.0}, Vec3{}});

        CHECK(approx_equal(s.center_of_mass(), Vec3{1.0, 2.0, 3.0}));
    }

    SUBCASE("works in all three dimensions at once")
    {
        System s;
        s.add(Body{1.0, 0.0, Vec3{0.0, 0.0, 0.0}, Vec3{}});
        s.add(Body{1.0, 0.0, Vec3{2.0, 4.0, 6.0}, Vec3{}});

        CHECK(approx_equal(s.center_of_mass(), Vec3{1.0, 2.0, 3.0}));
    }
}

TEST_CASE("the Sun-Earth barycentre lies inside the Sun")
{
    // A real physical result rather than an arithmetic one, and a good check that the
    // mass weighting is the right way round. The Sun does not sit still while the Earth
    // orbits it: both orbit their common barycentre. But the Sun outweighs the Earth by
    // a factor of about 333,000, so that barycentre falls only ~449 km from the Sun's
    // centre, while the Sun's radius is ~696,000 km. The wobble is real and it is
    // entirely inside the star.
    //
    // If the weighting were inverted, this would land 149 million km away instead, so
    // the test has plenty of room to notice.
    System s;
    s.add(Body{kSunMass, kSunRadius, Vec3{}, Vec3{}}, "Sun");
    s.add(Body{kEarthMass, 6.371e6, Vec3{kAu, 0.0, 0.0}, Vec3{}}, "Earth");

    const Vec3 barycentre = s.center_of_mass();

    CHECK(barycentre.x == doctest::Approx(449201.4).epsilon(1e-4));
    CHECK(barycentre.y == 0.0);
    CHECK(barycentre.z == 0.0);

    CHECK(barycentre.length() < kSunRadius);
    CHECK(barycentre.length() / kSunRadius < 0.001);  // well under 1% of the way out
}

TEST_CASE("total_momentum")
{
    SUBCASE("equal and opposite momenta cancel")
    {
        System s;
        s.add(Body{2.0, 0.0, Vec3{}, Vec3{3.0, 0.0, 0.0}});
        s.add(Body{3.0, 0.0, Vec3{}, Vec3{-2.0, 0.0, 0.0}});

        // 2*3 = 6 and 3*(-2) = -6.
        CHECK(approx_equal(s.total_momentum(), Vec3{}));
    }

    SUBCASE("is mass-weighted, not a plain velocity sum")
    {
        System s;
        s.add(Body{10.0, 0.0, Vec3{}, Vec3{1.0, 0.0, 0.0}});
        s.add(Body{1.0, 0.0, Vec3{}, Vec3{1.0, 0.0, 0.0}});

        CHECK(approx_equal(s.total_momentum(), Vec3{11.0, 0.0, 0.0}));
    }

    SUBCASE("stationary bodies carry none")
    {
        System s;
        s.add(Body{5.0, 0.0, Vec3{1.0, 2.0, 3.0}, Vec3{}});

        CHECK(s.total_momentum() == Vec3{});
    }
}

TEST_CASE("center_of_mass_velocity is the drift of the whole system")
{
    SUBCASE("a balanced system does not drift")
    {
        System s;
        s.add(Body{2.0, 0.0, Vec3{}, Vec3{3.0, 0.0, 0.0}});
        s.add(Body{3.0, 0.0, Vec3{}, Vec3{-2.0, 0.0, 0.0}});

        CHECK(approx_equal(s.center_of_mass_velocity(), Vec3{}));
    }

    SUBCASE("bodies moving together drift at their shared velocity")
    {
        System s;
        s.add(Body{1.0, 0.0, Vec3{}, Vec3{0.0, 10.0, 0.0}});
        s.add(Body{99.0, 0.0, Vec3{}, Vec3{0.0, 10.0, 0.0}});

        CHECK(approx_equal(s.center_of_mass_velocity(), Vec3{0.0, 10.0, 0.0}));
    }

    SUBCASE("a heavy slow body dominates a light fast one")
    {
        // 1000 kg at 1 m/s and 1 kg at 100 m/s: total p = 1100, total m = 1001.
        System s;
        s.add(Body{1000.0, 0.0, Vec3{}, Vec3{1.0, 0.0, 0.0}});
        s.add(Body{1.0, 0.0, Vec3{}, Vec3{100.0, 0.0, 0.0}});

        CHECK(s.center_of_mass_velocity().x == doctest::Approx(1100.0 / 1001.0));
    }
}
