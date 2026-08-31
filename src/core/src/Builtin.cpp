#include <orbitalis/scenarios/Builtin.hpp>

#include <cmath>
#include <numbers>

namespace orbitalis::scenarios {

namespace {

// Radii are for display only; gravity treats every body as a point mass.
constexpr double kSunRadius = 6.957e8;    // m
constexpr double kEarthRadius = 6.371e6;  // m

}  // namespace

double circular_period(double gm_total, double separation) noexcept
{
    return 2.0 * std::numbers::pi
           * std::sqrt(separation * separation * separation / gm_total);
}

System sun_earth(double separation)
{
    // Work in GM rather than mass wherever possible. GM is measured directly from orbits
    // to about ten significant figures; mass in kilograms inherits G's uncertainty of
    // 2.2e-5. Deriving the period from masses instead of GM is what put the year 68
    // minutes out before 0.0.6, see Constants.hpp.
    const double gm_total = kSunGM + kEarthGM;

    // Relative speed for a circular orbit at this separation: v² = GM/a.
    const double v_relative = std::sqrt(gm_total / separation);

    // Split position and velocity by the mass ratio so the barycentre is at the origin
    // and at rest. Each body gets the *other* body's share, which is the part that looks
    // backwards at first glance and is the whole content of the two-body reduction.
    const double sun_share = kEarthGM / gm_total;
    const double earth_share = kSunGM / gm_total;

    System s;
    s.add(Body{kSolarMass,
               kSunRadius,
               Vec3{-separation * sun_share, 0.0, 0.0},
               Vec3{0.0, -v_relative * sun_share, 0.0}},
          "Sun");
    s.add(Body{kEarthMass,
               kEarthRadius,
               Vec3{separation * earth_share, 0.0, 0.0},
               Vec3{0.0, v_relative * earth_share, 0.0}},
          "Earth");
    return s;
}

System sun_earth_drifting(double separation)
{
    const double v_relative = std::sqrt((kSunGM + kEarthGM) / separation);

    System s;
    s.add(Body{kSolarMass, kSunRadius, Vec3{}, Vec3{}}, "Sun");
    s.add(Body{kEarthMass,
               kEarthRadius,
               Vec3{separation, 0.0, 0.0},
               Vec3{0.0, v_relative, 0.0}},
          "Earth");
    return s;
}

}  // namespace orbitalis::scenarios
