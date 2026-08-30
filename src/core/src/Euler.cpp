#include <orbitalis/integrators/Euler.hpp>

#include <cstddef>

namespace orbitalis {

void ForwardEuler::step(System& system, double dt)
{
    const std::span<Body> bodies = system.bodies();

    // resize() is a no-op once the vector is big enough, so a long run allocates once.
    accelerations_.resize(bodies.size());

    solver_->compute_accelerations(bodies, accelerations_);

    for (std::size_t i = 0; i < bodies.size(); ++i) {
        // Position is updated FIRST, using the velocity from the start of the step. That
        // ordering is the whole definition of forward Euler, and it is why the method
        // pumps energy into a bound orbit: over the step the body really does curve, but
        // this moves it along the straight line it was travelling on at the beginning,
        // which always lands it slightly further out than it should be. Repeat 365 times
        // and a circular orbit at 1 AU has become 1.206 AU.
        bodies[i].position += bodies[i].velocity * dt;
        bodies[i].velocity += accelerations_[i] * dt;
    }
}

void SemiImplicitEuler::step(System& system, double dt)
{
    const std::span<Body> bodies = system.bodies();

    accelerations_.resize(bodies.size());

    solver_->compute_accelerations(bodies, accelerations_);

    for (std::size_t i = 0; i < bodies.size(); ++i) {
        // Velocity first, then position using the velocity that was just updated. One
        // word different from the above, and it makes the method symplectic: the error no
        // longer accumulates in one direction, it oscillates around the true orbit.
        bodies[i].velocity += accelerations_[i] * dt;
        bodies[i].position += bodies[i].velocity * dt;
    }
}

}  // namespace orbitalis
