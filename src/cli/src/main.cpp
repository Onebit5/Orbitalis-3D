#include <orbitalis/core/Version.hpp>
#include <orbitalis/integrators/Euler.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <algorithm>
#include <cstdio>
#include <string_view>

// Milestone 0.0.6: run the Sun-Earth system for one orbit and report what happened.
//
// No argument parsing yet; a real CLI is 0.6.0 (step 0.5.4). This exists to close
// milestone 0.1.0, whose test is "Earth goes around the Sun once and doesn't fly off into
// nowhere, and the energy drift is bad and I know it's bad".

namespace {

using namespace orbitalis;

struct Result
{
    const char* integrator;
    double final_radius_au;
    double return_distance_au;
    double momentum_drift;
};

/// Advances a copy of `system` for one full orbit and measures the damage.
///
/// "Return distance" is how far Earth ends up from where it began. After exactly one
/// period a perfect integrator would put it back at zero, so this is a direct measure of
/// accumulated error rather than a proxy for it.
template <typename Integrator>
Result run(System system, const IForceSolver& solver, double dt, int steps,
           const char* label)
{
    const Vec3 start = system[1].position - system[0].position;
    const Vec3 p0 = system.total_momentum();

    Integrator integrator{solver};
    for (int i = 0; i < steps; ++i) {
        integrator.step(system, dt);
    }

    const Vec3 finish = system[1].position - system[0].position;

    // Momentum should be conserved by any integrator, since it is a property of the force
    // solver rather than the method.
    //
    // Scaling this correctly matters more than it looks. The barycentric scenario starts
    // with total momentum of exactly zero, so a relative-to-initial figure would divide by
    // nothing. But the raw absolute drift is just as useless: it comes out around 5e13,
    // which sounds catastrophic until you notice Earth alone carries 1.8e29 kg m/s. So it
    // is normalised against the largest individual momentum, which is the scale the
    // cancellation actually happens at.
    double scale = 0.0;
    for (const Body& b : system.bodies()) {
        scale = std::max(scale, (b.mass * b.velocity).length());
    }
    const double drift = (system.total_momentum() - p0).length() / scale;

    return Result{label,
                  finish.length() / kAstronomicalUnit,
                  (finish - start).length() / kAstronomicalUnit,
                  drift};
}

}  // namespace

int main()
{
    std::printf("%s\n", orbitalis::version_banner());
    std::printf("milestone : %s\n\n", orbitalis::milestone_name());

    constexpr double separation = kAstronomicalUnit;

    System system = scenarios::sun_earth(separation);

    const double period = scenarios::circular_period(kSunGM + kEarthGM, separation);

    // One orbit split into a whole number of steps, so "one period" is exact and the
    // return distance measures integrator error and nothing else.
    constexpr int kSteps = 365;
    const double dt = period / kSteps;

    std::printf("scenario   : Sun-Earth, circular, barycentric frame\n");
    std::printf("bodies     : %zu\n", system.size());
    std::printf("separation : %.6f AU\n", separation / kAstronomicalUnit);
    std::printf("period     : %.6f days   (sidereal year is %.6f)\n",
                period / kDay, kSiderealYear / kDay);
    std::printf("timestep   : %.4f days\n", dt / kDay);
    std::printf("steps      : %d  (exactly one orbit)\n\n", kSteps);

    std::printf("initial state\n");
    for (BodyId i = 0; i < system.size(); ++i) {
        // %.*s rather than %s: a string_view is not guaranteed null-terminated, even
        // when it happens to be here because the names came from std::string.
        const std::string_view nm = system.name(i);
        std::printf("  %-6.*s m = %.6e kg   x = %+.6e m   v = %+.6f m/s\n",
                    static_cast<int>(nm.size()), nm.data(), system[i].mass,
                    system[i].position.x, system[i].velocity.y);
    }
    std::printf("  net drift  : %.6e m/s\n\n", system.center_of_mass_velocity().length());

    const BruteForceSolver solver;

    const Result results[] = {
        run<ForwardEuler>(system, solver, dt, kSteps, "forward-euler"),
        run<SemiImplicitEuler>(system, solver, dt, kSteps, "semi-implicit-euler"),
    };

    std::printf("after one orbit\n");
    std::printf("  %-22s %14s %18s %16s\n", "integrator", "final r (AU)",
                "return dist (AU)", "rel. |dp| drift");
    for (const Result& r : results) {
        std::printf("  %-22s %14.6f %18.6f %16.3e\n", r.integrator, r.final_radius_au,
                    r.return_distance_au, r.momentum_drift);
    }

    std::printf("\n");
    std::printf("forward Euler moves the body along the tangent it started the step on, and a\n");
    std::printf("tangent always falls outside the circle, so it gains radius every single step.\n");
    std::printf("semi-implicit costs exactly the same and keeps the orbit closed.\n\n");

    // The drifting scenario, kept as a demonstration of what goes wrong when the star is
    // pinned at the origin.
    System drifting = scenarios::sun_earth_drifting();
    const double drift_speed = drifting.center_of_mass_velocity().length();
    std::printf("for comparison, the same pair with the Sun parked at rest:\n");
    std::printf("  barycentre drifts at %.6f m/s, which is %.1f km per year\n",
                drift_speed, drift_speed * kSiderealYear / 1000.0);
    drifting.remove_net_drift();
    std::printf("  after remove_net_drift(): %.3e m/s\n", drifting.center_of_mass_velocity().length());

    return 0;
}
