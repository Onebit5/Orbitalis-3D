#pragma once

namespace orbitalis {

/// Newton's gravitational constant, m³ kg⁻¹ s⁻². CODATA 2018.
///
/// By far the least precisely measured of the fundamental constants, with a relative
/// uncertainty around 2.2e-5. Everything else here is known far better than that.
inline constexpr double kGravitationalConstant = 6.67430e-11;

// ---- standard gravitational parameters ------------------------------------------------
//
// GM, not M. These are the primary constants and the masses below are derived from them,
// which is the opposite of how it usually gets written down. The reason is that nobody
// measures the Sun in kilograms: what comes out of orbital observations is the *product*
// GM, and it is known to about ten significant figures while G alone is known to five.
//
// This is not a theoretical nicety, it cost me the length of the year. Up to 0.0.5 this
// file had kSolarMass = 1.98892e30 taken straight from a reference table, which implies
// G·M = 1.32746e20 against the measured 1.32712e20: high by 0.026%. Since T ∝ 1/√(GM),
// that put the Earth's orbital period out by 68 minutes.
//
// Deriving mass from GM instead gives a two-body period at a = 1 AU of 365.256350 days
// against the real sidereal year of 365.256363. A difference of 1.1 seconds over a year.
//
// Values are IAU 2009 / JPL DE430.

/// GM of the Sun, m³ s⁻².
inline constexpr double kSunGM = 1.32712440018e20;

/// GM of the Earth, m³ s⁻².
inline constexpr double kEarthGM = 3.986004418e14;

// ---- masses, derived ------------------------------------------------------------------
//
// Kept because Body stores a mass and the force solver multiplies by it. But they inherit
// G's uncertainty, so anything that can be phrased in terms of GM should be.

inline constexpr double kSolarMass = kSunGM / kGravitationalConstant;    // ~1.98841e30 kg
inline constexpr double kEarthMass = kEarthGM / kGravitationalConstant;  // ~5.97217e24 kg

// ---- unit conversions -----------------------------------------------------------------
//
// The simulation is SI internally, always. These exist so scenario data written the way
// astronomers write it can be converted at the boundary and nowhere else.

/// Astronomical unit, m. Exact by definition since the IAU fixed it in 2012.
inline constexpr double kAstronomicalUnit = 1.495978707e11;

/// Seconds in a day.
inline constexpr double kDay = 86400.0;

/// Seconds in a Julian year: exactly 365.25 days by definition. A unit of time, not an
/// orbital period. Do not use it to decide when Earth has gone round once.
inline constexpr double kJulianYear = 365.25 * kDay;

/// Seconds in a sidereal year, 365.256363 days: the time for Earth to return to the same
/// position against the fixed stars. This is the one that means "one orbit".
///
/// It is not the same as the tropical year of 365.2422 days, which tracks the seasons and
/// is shorter because the equinoxes precess. Calendars care about the tropical year;
/// orbital mechanics cares about the sidereal one.
inline constexpr double kSiderealYear = 365.256363 * kDay;

}  // namespace orbitalis
