#pragma once

#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>

namespace orbitalis::scenarios {

/// Period of a circular two-body orbit, seconds.
///
///     T = 2π · √(a³ / GM)
///
/// where GM is the *combined* standard gravitational parameter of both bodies and `a` is
/// their separation. Using only the central body's GM is the common mistake and is wrong
/// whenever the second mass is not negligible.
///
/// Kepler's third law is the a³ in there: period squared goes as separation cubed. That
/// gets asserted properly at 0.6.3.
[[nodiscard]] double circular_period(double gm_total, double separation) noexcept;

/// Sun and Earth on a circular orbit, in the barycentric frame.
///
/// Both bodies orbit their common centre of mass, which is where they actually are: the
/// Sun is not a fixed post. Positions and velocities are split by the mass ratio so that
/// total momentum is exactly zero and the barycentre sits at the origin and stays there.
///
/// The Sun's share is small but not nothing. At one AU it circles a point 449 km from its
/// own centre at about 9 cm/s.
[[nodiscard]] System sun_earth(double separation = kAstronomicalUnit);

/// The same pair set up the tempting way: Sun parked at the origin at rest, Earth given
/// the full orbital velocity.
///
/// Kept deliberately, because it is what almost everyone writes first and the failure is
/// instructive rather than obvious. The pair carries a net momentum of m⊕·v, so the
/// barycentre slides at about 9 cm/s forever. That is 2,800 km per year: harmless for one
/// orbit, and a steadily growing embarrassment over a long run.
///
/// Calling `remove_net_drift()` on the result converts it into an equivalent
/// non-drifting system.
[[nodiscard]] System sun_earth_drifting(double separation = kAstronomicalUnit);

}  // namespace orbitalis::scenarios
