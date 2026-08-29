#pragma once

#include <orbitalis/math/Vec3.hpp>

#include <type_traits>

namespace orbitalis {

/// A single gravitating body: a point mass that happens to have a radius.
///
/// This holds only what the simulation *is*, not what it computes. Specifically it holds
/// no acceleration and no name. Both of those were tempting and both are wrong here:
///
///   - acceleration is derived, recomputed from scratch every step, and different
///     integrators need different numbers of acceleration buffers (RK4 needs four).
///     So it belongs to whoever is doing the integrating, not to the state. See the
///     0.0.3 devlog entry.
///   - a name is metadata. Nothing in the physics reads it. Putting a std::string here
///     would add 32 bytes to a struct that gets streamed through the n-body loop, and
///     would stop Body being trivially copyable, which the binary writer at 0.5.0 needs.
///     Names live alongside the bodies in System instead.
///
/// Units are SI throughout: kilograms, metres, seconds. Conversion from AU / days /
/// solar masses happens at the I/O boundary and nowhere else.
struct Body
{
    /// Gravitational mass, kg. Must be positive; a zero or negative mass is not
    /// physically meaningful and will produce nonsense rather than an error.
    double mass{};

    /// Radius, m. Not used by gravity, which treats every body as a point mass. It is
    /// here for rendering at 0.1.3 and for collision detection at 0.7.1.
    double radius{};

    /// Position, m, in the simulation's inertial frame.
    Vec3 position{};

    /// Velocity, m/s, in the same frame.
    Vec3 velocity{};
};

// 8 + 8 + 24 + 24 = 64 bytes, with no padding, which is exactly one cache line on x86-64.
// That is not a coincidence and it is worth protecting: at 0.4.0 the force loop streams
// through an array of these, and a body that straddles two cache lines doubles the memory
// traffic of the hottest loop in the project.
//
// If this fires, someone added a field. That is allowed, but it is a decision to make
// deliberately rather than by accident.
static_assert(sizeof(Body) == 64,
              "Body should be exactly one 64-byte cache line. Adding a field has a real "
              "cost in the n-body loop; if the new field is worth it, update this assert.");

// Required by the bulk binary writer at 0.5.0, which memcpys arrays of these straight to
// disk. Adding a std::string, a virtual function or a custom destructor would break it.
static_assert(std::is_trivially_copyable_v<Body>,
              "Body must stay trivially copyable so trajectories can be written in bulk.");

}  // namespace orbitalis
