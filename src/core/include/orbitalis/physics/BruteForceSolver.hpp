#pragma once

#include <orbitalis/physics/ForceSolver.hpp>

namespace orbitalis {

/// Every pair, no approximation. O(n²).
///
/// This is the reference implementation, and it stays in the project permanently even
/// after Barnes-Hut arrives at 0.4.0. Barnes-Hut is an *approximation* with a tunable
/// error, and the only way to know that error is to compare it against an exact answer.
/// Step 0.3.5 is precisely that comparison, and it needs this class to be trustworthy.
///
/// It computes each pair once and applies the result to both bodies via Newton's third
/// law, so the inner loop runs n(n-1)/2 times rather than n².
class BruteForceSolver final : public IForceSolver
{
public:
    /// `softening` is the Plummer softening length ε, in metres.
    ///
    /// Zero means pure Newtonian gravity, which is the default because softening is a
    /// fudge and opting into a fudge should be explicit. See the note on the class for
    /// what it actually does to the physics.
    explicit BruteForceSolver(double softening = 0.0) noexcept;

    void compute_accelerations(std::span<const Body> bodies,
                               std::span<Vec3> accelerations) const override;

    [[nodiscard]] const char* name() const noexcept override { return "brute-force"; }

    /// The softening length ε, in metres.
    [[nodiscard]] double softening() const noexcept { return softening_; }

private:
    double softening_{};
    double softening_squared_{};
};

// ---------------------------------------------------------------------------------------
// On softening.
//
// The unsoftened acceleration of body i due to body j is
//
//     a⃗ = G · mⱼ · d⃗ / |d⃗|³        where d⃗ = r⃗ⱼ - r⃗ᵢ
//
// As |d⃗| goes to zero this goes to infinity. Not "large": infinity, then NaN, and then
// every body in the simulation is NaN within one step, because they all pull on each
// other. Two bodies passing close is not an exotic case; it is Tuesday in an n-body run.
//
// Plummer softening replaces the denominator with
//
//     a⃗ = G · mⱼ · d⃗ / (|d⃗|² + ε²)^(3/2)
//
// which can never be zero. Notice it costs one addition and no branch, which is a large
// part of why it is the standard choice.
//
// **This is a fudge, and it is important to stay honest about which one.** It does not
// make close encounters accurate; it makes them not explode. Physically it is equivalent
// to replacing each point mass with a Plummer sphere of scale length ε, so at separations
// well inside ε the force is *wrong* on purpose: it falls to zero at zero separation
// instead of diverging.
//
// The rule that follows: ε must be small compared to the separations that matter. For a
// star cluster where individual close encounters are noise, a large ε is the right call.
// For solar-system orbital mechanics, where the whole point is getting individual
// trajectories right, ε should be zero and close encounters should be handled by shrinking
// the timestep instead. That is what adaptive RKF45 is for at 0.2.5.
//
// One consequence to remember at 0.2.3: the potential consistent with this force is
// Φ = -G·m / √(|d⃗|² + ε²), *not* the Newtonian -G·m/|d⃗|. Measuring energy conservation
// with the unsoftened potential while integrating softened forces would show a drift that
// is an artefact of mismatched formulas rather than a real integrator problem.
// ---------------------------------------------------------------------------------------

}  // namespace orbitalis
