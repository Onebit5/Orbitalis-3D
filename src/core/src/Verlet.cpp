#include <orbitalis/integrators/Verlet.hpp>

#include <cstddef>

namespace orbitalis {

void VelocityVerlet::step(System& system, double dt)
{
    const std::span<Body> bodies = system.bodies();

    // A change in body count invalidates the whole cache, not just the tail. Adding one
    // body changes the acceleration of every other body, so keeping the prefix and filling
    // in the new entry would be wrong in a way that looks right.
    if (accelerations_.size() != bodies.size()) {
        accelerations_.resize(bodies.size());
        has_cached_acceleration_ = false;
    }

    // Only on the first step after construction or reset(). Every other step inherits this
    // from the closing evaluation of the previous one, which is the entire trick: the
    // acceleration at the end of a step is the acceleration at the start of the next.
    if (!has_cached_acceleration_) {
        solver_->compute_accelerations(bodies, accelerations_);
        has_cached_acceleration_ = true;
    }

    const double half_dt = 0.5 * dt;

    for (std::size_t i = 0; i < bodies.size(); ++i) {
        // Kick by half a step, then drift by a whole one. Splitting the kick around the
        // drift is what makes the composition symmetric, and the symmetry is what makes
        // the method second order and time-reversible. Kicking a whole step before the
        // drift instead would give semi-implicit Euler, which is first order.
        bodies[i].velocity += accelerations_[i] * half_dt;
        bodies[i].position += bodies[i].velocity * dt;
    }

    // The one force evaluation, at the positions the step ends on. This is also what gets
    // carried into the next step's opening kick.
    solver_->compute_accelerations(bodies, accelerations_);

    for (std::size_t i = 0; i < bodies.size(); ++i) {
        bodies[i].velocity += accelerations_[i] * half_dt;
    }
}

}  // namespace orbitalis
