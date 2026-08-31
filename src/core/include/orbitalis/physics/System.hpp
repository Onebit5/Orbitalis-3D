#pragma once

#include <orbitalis/physics/Body.hpp>

#include <cstddef>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace orbitalis {

/// Identifies a body within a System. This is its index, nothing cleverer.
///
/// Known limitation, flagged now so it does not surprise me later: because it is an
/// index, removing a body invalidates every id above it. Nothing removes bodies yet, but
/// collision merging at 0.7.1 will, and at that point anything holding an id across a
/// step (the viewer's "follow this body", trail buffers) breaks. The fix is stable
/// handles with a generation counter, and it is deliberately not being built now for a
/// problem eight milestones away.
using BodyId = std::size_t;

/// The complete state of a simulation: every gravitating body, plus their names.
///
/// This is *state only*. It holds no accelerations, no forces, no timestep and no
/// integrator. That is what makes it the thing checkpoint/restore serialises at 0.5.6:
/// positions, velocities and masses are the whole story, and everything else can be
/// recomputed from them.
class System
{
public:
    // ---- construction ---------------------------------------------------------------

    /// Adds a body and returns its id. The name is optional and purely for display; the
    /// physics never reads it.
    BodyId add(const Body& body, std::string name = {});

    /// Reserves capacity for n bodies. Worth calling before loading a large scenario:
    /// the Plummer-sphere generator at 0.5.5 adds bodies in a tight loop.
    void reserve(std::size_t n);

    void clear() noexcept;

    // ---- access ---------------------------------------------------------------------

    [[nodiscard]] std::size_t size() const noexcept { return bodies_.size(); }
    [[nodiscard]] bool empty() const noexcept { return bodies_.empty(); }

    [[nodiscard]] Body& operator[](BodyId id) noexcept { return bodies_[id]; }
    [[nodiscard]] const Body& operator[](BodyId id) const noexcept { return bodies_[id]; }

    /// A view of every body, contiguous. This is what the force solver at 0.0.4 and the
    /// integrators at 0.3.0 iterate over.
    ///
    /// Deliberately a span rather than a reference to the underlying vector: a span
    /// cannot resize, so callers can read and mutate bodies but cannot push_back behind
    /// the System's back and desynchronise the parallel name array. add() is the only
    /// way in.
    [[nodiscard]] std::span<Body> bodies() noexcept { return bodies_; }
    [[nodiscard]] std::span<const Body> bodies() const noexcept { return bodies_; }

    /// Display name, or an empty view if the body was added without one.
    [[nodiscard]] std::string_view name(BodyId id) const noexcept { return names_[id]; }

    void set_name(BodyId id, std::string name) { names_[id] = std::move(name); }

    // ---- aggregate quantities -------------------------------------------------------
    //
    // These are the cheap ones, the ones that need no force law. Energy needs the
    // potential, which needs G and the pair loop, so it arrives with the diagnostics at
    // 0.2.3 rather than here.

    /// Sum of all masses, kg.
    [[nodiscard]] double total_mass() const noexcept;

    /// Mass-weighted mean position, m. The barycentre.
    ///
    /// Returns the zero vector for an empty system. If the system is non-empty but the
    /// masses sum to zero, the result is NaN, which is correct: that is a broken system
    /// and it should be loud rather than plausible.
    [[nodiscard]] Vec3 center_of_mass() const noexcept;

    /// Total linear momentum, kg m/s. In an isolated system under gravity this is exactly
    /// conserved, which makes it one of the sharpest correctness checks available for an
    /// integrator at 0.2.3.
    [[nodiscard]] Vec3 total_momentum() const noexcept;

    /// Subtracts the barycentre velocity from every body, so the system as a whole stops
    /// drifting. Physics is unchanged: momentum conservation means the drift is constant,
    /// so removing it is a change of inertial frame and nothing more.
    ///
    /// Worth doing to almost any hand-written scenario. Giving a planet its orbital
    /// velocity while leaving the star at rest leaves the pair carrying net momentum, and
    /// the whole system then slides across the universe forever.
    ///
    /// Only touches velocities. Where the barycentre *sits* is a separate choice.
    void remove_net_drift() noexcept;

    /// Velocity of the barycentre, m/s. Non-zero means the whole system is drifting,
    /// which is usually an artefact of hand-written initial conditions rather than
    /// physics. Subtracting it gives a centre-of-mass frame that keeps everything on
    /// screen.
    [[nodiscard]] Vec3 center_of_mass_velocity() const noexcept;

private:
    // Parallel arrays, kept the same length by add() and clear(). Names are separated out
    // so that Body stays 64 bytes and trivially copyable; see the comment in Body.hpp.
    std::vector<Body> bodies_;
    std::vector<std::string> names_;
};

}  // namespace orbitalis
