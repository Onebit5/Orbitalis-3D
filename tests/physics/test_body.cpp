#include <doctest/doctest.h>

#include <orbitalis/physics/Body.hpp>

#include <type_traits>
#include <vector>

using orbitalis::Body;
using orbitalis::Vec3;

// The layout guarantees live as static_asserts in Body.hpp itself, so they fail the build
// of anything that includes the header rather than only the test binary. Restating the
// reasoning here would just be duplication; what follows tests behaviour instead.

TEST_CASE("Body default-initialises to zero")
{
    const Body b{};

    CHECK(b.mass == 0.0);
    CHECK(b.radius == 0.0);
    CHECK(b.position == Vec3{});
    CHECK(b.velocity == Vec3{});
}

TEST_CASE("Body is an aggregate")
{
    // No user-declared constructors, so this brace form works and the field order is
    // part of the interface: mass, radius, position, velocity.
    const Body earth{
        5.9722e24,
        6.371e6,
        Vec3{1.495978707e11, 0.0, 0.0},
        Vec3{0.0, 29780.0, 0.0},
    };

    CHECK(earth.mass == 5.9722e24);
    CHECK(earth.radius == 6.371e6);
    CHECK(earth.position.x == 1.495978707e11);
    CHECK(earth.velocity.y == 29780.0);
}

TEST_CASE("Body copies by value")
{
    const Body a{1.0, 2.0, Vec3{3.0, 4.0, 5.0}, Vec3{6.0, 7.0, 8.0}};
    Body b = a;

    CHECK(b.mass == a.mass);
    CHECK(b.position == a.position);

    // Copies are independent. Worth pinning: the integrators at 0.3.0 take copies of
    // state to evaluate trial steps, and RK4 in particular depends on this.
    b.position = Vec3{};
    CHECK(a.position == Vec3{3.0, 4.0, 5.0});
}

TEST_CASE("Body survives being stored contiguously")
{
    // The force loop at 0.0.4 and the tree at 0.4.0 both iterate an array of these, and
    // the binary writer at 0.5.0 memcpys one. Check the contiguity assumption directly.
    std::vector<Body> bodies;
    bodies.push_back(Body{1.0, 0.0, Vec3{1.0, 0.0, 0.0}, Vec3{}});
    bodies.push_back(Body{2.0, 0.0, Vec3{2.0, 0.0, 0.0}, Vec3{}});
    bodies.push_back(Body{3.0, 0.0, Vec3{3.0, 0.0, 0.0}, Vec3{}});

    const Body* raw = bodies.data();
    CHECK(raw[0].mass == 1.0);
    CHECK(raw[1].mass == 2.0);
    CHECK(raw[2].mass == 3.0);

    // Stride between elements is exactly sizeof(Body), i.e. no hidden padding between
    // array entries.
    const auto stride = reinterpret_cast<const char*>(&bodies[1])
                        - reinterpret_cast<const char*>(&bodies[0]);
    CHECK(stride == static_cast<std::ptrdiff_t>(sizeof(Body)));
}
