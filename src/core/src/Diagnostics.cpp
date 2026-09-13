#include <orbitalis/physics/Diagnostics.hpp>

#include <algorithm>
#include <cmath>

namespace orbitalis {

double kinetic_energy(std::span<const Body> bodies) noexcept
{
    double energy = 0.0;
    for (const Body& body : bodies) {
        // length_squared() rather than length(), so no square root is taken only to be
        // squared again. Same reasoning as the gravity kernel at 0.0.4.
        energy += 0.5 * body.mass * body.velocity.length_squared();
    }
    return energy;
}

Vec3 angular_momentum(std::span<const Body> bodies, const Vec3& origin) noexcept
{
    Vec3 total{};
    for (const Body& body : bodies) {
        total += body.mass * cross(body.position - origin, body.velocity);
    }
    return total;
}

Diagnostics measure(const System& system, const IForceSolver& solver)
{
    const std::span<const Body> bodies = system.bodies();

    Diagnostics d;

    d.kinetic = kinetic_energy(bodies);
    d.potential = solver.potential_energy(bodies);
    d.total = d.kinetic + d.potential;

    d.momentum = system.total_momentum();
    d.center_of_mass = system.center_of_mass();
    d.center_of_mass_velocity = system.center_of_mass_velocity();

    // About the barycentre, so the answer is the system's internal rotation and does not
    // change when the whole thing is viewed from a different inertial frame.
    d.angular_momentum = angular_momentum(bodies, d.center_of_mass);

    // The scales the drift figures get divided by. Computed here rather than at comparison
    // time because they belong to the state being measured, and because the baseline's
    // scale is the one a drift should be judged against.
    for (const Body& body : bodies) {
        d.largest_body_momentum =
            std::max(d.largest_body_momentum, (body.mass * body.velocity).length());

        const Vec3 l = body.mass * cross(body.position - d.center_of_mass, body.velocity);
        d.largest_body_angular_momentum = std::max(d.largest_body_angular_momentum, l.length());
    }

    return d;
}

void ConservationMonitor::reset(const System& system, const IForceSolver& solver)
{
    baseline_ = measure(system, solver);
    latest_ = baseline_;
    worst_energy_ = 0.0;
    samples_ = 0;
    has_baseline_ = true;
}

void ConservationMonitor::sample(const System& system, const IForceSolver& solver)
{
    if (!has_baseline_) {
        reset(system, solver);
        return;
    }

    latest_ = measure(system, solver);
    ++samples_;
    worst_energy_ = std::max(worst_energy_, std::abs(relative_energy_error()));
}

double ConservationMonitor::relative_energy_error() const noexcept
{
    // Guard rather than assume: an unbound system passing through E = 0, or a system of one
    // motionless body, both give a zero baseline and would otherwise produce a NaN that
    // then poisons the running worst case forever.
    if (!has_baseline_ || baseline_.total == 0.0) {
        return 0.0;
    }
    return (latest_.total - baseline_.total) / std::abs(baseline_.total);
}

double ConservationMonitor::relative_momentum_drift() const noexcept
{
    if (!has_baseline_ || baseline_.largest_body_momentum == 0.0) {
        return 0.0;
    }
    return (latest_.momentum - baseline_.momentum).length() / baseline_.largest_body_momentum;
}

double ConservationMonitor::relative_angular_momentum_drift() const noexcept
{
    if (!has_baseline_ || baseline_.largest_body_angular_momentum == 0.0) {
        return 0.0;
    }
    return (latest_.angular_momentum - baseline_.angular_momentum).length() /
           baseline_.largest_body_angular_momentum;
}

}  // namespace orbitalis
