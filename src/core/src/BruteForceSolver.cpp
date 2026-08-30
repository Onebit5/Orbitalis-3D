#include <orbitalis/physics/BruteForceSolver.hpp>

#include <orbitalis/physics/Constants.hpp>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>

namespace orbitalis {

BruteForceSolver::BruteForceSolver(double softening) noexcept
    : softening_(softening)
    , softening_squared_(softening * softening)
{
}

void BruteForceSolver::compute_accelerations(std::span<const Body> bodies,
                                             std::span<Vec3> accelerations) const
{
    assert(accelerations.size() == bodies.size()
           && "accelerations span must be the same length as bodies");

    // Overwrite rather than accumulate. The caller gets a fresh answer every call, which
    // is what the integrators want: acceleration is recomputed from scratch each step and
    // never carried over.
    std::fill(accelerations.begin(), accelerations.end(), Vec3{});

    const std::size_t n = bodies.size();

    // Each unordered pair once, applying the result to both bodies. The alternative, a
    // full n² loop skipping i == j, computes every separation vector twice and does twice
    // the square roots for an identical answer.
    //
    // Written as `i + 1 < n` rather than `i < n - 1` on purpose: n is unsigned, so with an
    // empty span `n - 1` wraps to a very large number and the loop runs forever.
    for (std::size_t i = 0; i + 1 < n; ++i) {
        for (std::size_t j = i + 1; j < n; ++j) {
            // Points from i toward j, so it is the direction i accelerates in.
            const Vec3 d = bodies[j].position - bodies[i].position;

            // Softening lives here, as one added term. With softening_squared_ == 0 this
            // is exactly the Newtonian expression.
            const double d2 = d.length_squared() + softening_squared_;

            // |d|³ = d² · √(d²). One square root and one division for the pair.
            //
            // Deliberately not std::pow(d2, 1.5): pow is a general-purpose function that
            // goes through logarithms, and it is both far slower and less accurate than a
            // multiply by a square root for this specific exponent.
            const double inv_r3 = 1.0 / (d2 * std::sqrt(d2));

            // Everything both bodies share. Only the masses differ from here.
            const Vec3 common = kGravitationalConstant * inv_r3 * d;

            // Newton's third law. i is pulled toward j by j's mass, j is pulled back
            // toward i by i's mass. Note the accelerations are not equal and opposite;
            // the *forces* are. The lighter body accelerates more.
            accelerations[i] += bodies[j].mass * common;
            accelerations[j] -= bodies[i].mass * common;
        }
    }
}

}  // namespace orbitalis
