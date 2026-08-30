#pragma once

namespace orbitalis {

/// Newton's gravitational constant, m^3 kg^-1 s^-2. CODATA 2018.
///
/// Worth knowing: this is by far the least precisely measured of the fundamental
/// constants, with a relative uncertainty around 2.2e-5. Every other number in this
/// simulation is known to far better than that, so G is the accuracy ceiling on any
/// absolute prediction the code makes.
///
/// In practice it matters less than it sounds, because what astronomers actually measure
/// is not G and not a mass, but the *product* GM (the "standard gravitational parameter"),
/// and that is known to about ten significant figures. So a solar-system simulation is far
/// more accurate than G alone would suggest, as long as masses come from published GM
/// values rather than from independently measured kilograms.
inline constexpr double kGravitationalConstant = 6.67430e-11;

// ---- unit conversions ---------------------------------------------------------------
//
// The simulation is SI internally, always. These exist so that scenario data written the
// way astronomers write it can be converted at the boundary and nowhere else.

/// Astronomical unit, m. Exact by definition since the IAU fixed it in 2012.
inline constexpr double kAstronomicalUnit = 1.495978707e11;

/// Seconds in a day.
inline constexpr double kDay = 86400.0;

/// Seconds in a Julian year, which is exactly 365.25 days by definition. Not the same as
/// a sidereal or tropical year; when 0.0.6 checks that Earth returns to its starting
/// point, the relevant period is the sidereal year of about 365.256 days.
inline constexpr double kJulianYear = 365.25 * kDay;

// ---- reference masses ---------------------------------------------------------------

inline constexpr double kSolarMass = 1.98892e30;   // kg
inline constexpr double kEarthMass = 5.9722e24;    // kg

}  // namespace orbitalis
