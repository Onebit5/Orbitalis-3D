#include <doctest/doctest.h>

#include <orbitalis/integrators/Euler.hpp>
#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/render/SimClock.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <cmath>
#include <limits>
#include <random>

using orbitalis::BruteForceSolver;
using orbitalis::SemiImplicitEuler;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::render::SimClock;

// =======================================================================================
// the property the whole class exists for
// =======================================================================================

TEST_CASE("the same real time produces the same number of steps at any framerate")
{
    // This is milestone 0.2.0's stated test, as an assertion rather than a squint at the
    // screen: the simulation must not care how fast the display is.
    SimClock::Config config;
    config.timestep = 1.0;
    config.time_scale = 1.0;
    config.max_steps_per_frame = 100000;  // not testing the cap here

    auto steps_over_ten_seconds = [&](double frame_seconds) {
        SimClock clock{config};
        int total = 0;
        const int frames = static_cast<int>(std::round(10.0 / frame_seconds));
        for (int i = 0; i < frames; ++i) {
            total += clock.advance(frame_seconds);
        }
        return total;
    };

    // Within one step, not exactly equal, and the difference is not a defect.
    //
    // Summing 1/30 three hundred times gives 9.9999999999999751, while 1/60 six hundred
    // times gives 10.000000000000076. Ten seconds is exactly a multiple of the timestep
    // here, so it sits on a floor() boundary and a shortfall of 1e-14 costs a whole step.
    // No accumulator built on doubles avoids that.
    //
    // The guarantee that matters is not the count. It is that every step is the SAME
    // LENGTH, so the physics is identical and at most one step of simulated time separates
    // two framerates. The end-to-end test at the bottom of this file checks the part that
    // actually counts.
    const int at30 = steps_over_ten_seconds(1.0 / 30.0);
    const int at60 = steps_over_ten_seconds(1.0 / 60.0);
    const int at144 = steps_over_ten_seconds(1.0 / 144.0);
    const int at300 = steps_over_ten_seconds(1.0 / 300.0);

    CHECK(std::abs(at30 - at60) <= 1);
    CHECK(std::abs(at60 - at144) <= 1);
    CHECK(std::abs(at144 - at300) <= 1);
    CHECK(std::abs(at30 - at300) <= 1);

    for (int steps : {at30, at60, at144, at300}) {
        CHECK(steps >= 9);
        CHECK(steps <= 10);
    }
}

TEST_CASE("away from a floor boundary the count is identical at every framerate")
{
    // The same sweep, arranged so the total is not an exact multiple of the timestep.
    // Accumulator drift of 1e-14 then cannot change the answer at all.
    SimClock::Config config;
    config.timestep = 1.0;
    config.max_steps_per_frame = 100000;

    auto steps_over = [&](double frame_seconds, double total_seconds) {
        SimClock clock{config};
        int total = 0;
        const int frames = static_cast<int>(std::round(total_seconds / frame_seconds));
        for (int i = 0; i < frames; ++i) {
            total += clock.advance(frame_seconds);
        }
        return total;
    };

    CHECK(steps_over(1.0 / 30.0, 10.5) == 10);
    CHECK(steps_over(1.0 / 60.0, 10.5) == 10);
    CHECK(steps_over(1.0 / 144.0, 10.5) == 10);
    CHECK(steps_over(1.0 / 300.0, 10.5) == 10);
}

TEST_CASE("an erratic framerate still gives the same total")
{
    // Real frames are not uniform: something else grabs the CPU, a window gets resized, the
    // compositor stutters. The accumulator has to absorb all of it.
    SimClock::Config config;
    config.timestep = 0.1;
    config.max_steps_per_frame = 100000;

    SimClock clock{config};

    std::mt19937 rng{12345};
    std::uniform_real_distribution<double> jitter{0.001, 0.08};

    int total = 0;
    double real_elapsed = 0.0;
    while (real_elapsed < 10.0) {
        const double frame = std::min(jitter(rng), 10.0 - real_elapsed);
        real_elapsed += frame;
        total += clock.advance(frame);
    }

    // 10 seconds at 0.1 s per step, within the one-step boundary tolerance described above.
    CHECK(total >= 99);
    CHECK(total <= 100);
    CHECK(clock.pending() < config.timestep);
}

TEST_CASE("the remainder is carried, not discarded")
{
    // Dropping the sub-step remainder each frame would make the simulation run slow by up
    // to one timestep per frame, which at 60 fps is a large systematic error.
    SimClock::Config config;
    config.timestep = 1.0;
    SimClock clock{config};

    // Nine frames of 0.9 s: 8.1 s of simulated time, so 8 steps and 0.1 carried.
    int total = 0;
    for (int i = 0; i < 9; ++i) {
        total += clock.advance(0.9);
    }

    CHECK(total == 8);
    CHECK(clock.pending() == doctest::Approx(0.1));
}

// =======================================================================================
// speed, pause, single step
// =======================================================================================

TEST_CASE("time scale multiplies the simulated rate")
{
    SimClock::Config config;
    config.timestep = 1.0;
    config.max_steps_per_frame = 100000;

    auto steps_at = [&](double scale) {
        SimClock clock{config};
        clock.set_time_scale(scale);
        int total = 0;
        for (int i = 0; i < 100; ++i) {
            total += clock.advance(0.1);  // 10 real seconds
        }
        return total;
    };

    // Ratios rather than absolutes, so a floor boundary cannot make this flaky.
    const int base = steps_at(1.0);
    CHECK(base >= 9);
    CHECK(base <= 10);
    CHECK(steps_at(2.0) >= 2 * base - 1);
    CHECK(steps_at(10.0) >= 10 * base - 1);
    CHECK(steps_at(0.5) >= base / 2 - 1);
    CHECK(steps_at(0.5) <= base / 2 + 1);
}

TEST_CASE("scale_speed is multiplicative and rejects nonsense")
{
    SimClock clock;
    clock.set_time_scale(4.0);

    clock.scale_speed(2.0);
    CHECK(clock.time_scale() == doctest::Approx(8.0));

    clock.scale_speed(0.25);
    CHECK(clock.time_scale() == doctest::Approx(2.0));

    clock.scale_speed(0.0);
    clock.scale_speed(-1.0);
    clock.scale_speed(std::numeric_limits<double>::quiet_NaN());
    CHECK(clock.time_scale() == doctest::Approx(2.0));
}

TEST_CASE("pausing stops the simulation without losing the clock")
{
    SimClock::Config config;
    config.timestep = 1.0;
    SimClock clock{config};

    clock.set_paused(true);
    for (int i = 0; i < 100; ++i) {
        CHECK(clock.advance(1.0) == 0);
    }

    CHECK(clock.pending() == doctest::Approx(0.0));

    SUBCASE("and unpausing does not release a flood of accumulated steps")
    {
        // Time spent paused is not owed. If the accumulator ran while paused, unpausing
        // after a minute would fire sixty steps at once.
        clock.set_paused(false);
        CHECK(clock.advance(1.0) == 1);
    }
}

TEST_CASE("single stepping works while paused")
{
    SimClock::Config config;
    config.timestep = 1.0;
    SimClock clock{config};
    clock.set_paused(true);

    clock.request_single_step();
    CHECK(clock.advance(1.0) == 1);

    // And only once.
    CHECK(clock.advance(1.0) == 0);
}

TEST_CASE("single stepping leaves the accumulator alone")
{
    // Otherwise stepping through a few frames would shift the phase of ordinary stepping
    // when play resumes, which makes single-stepping a debugging tool that changes what it
    // is debugging.
    SimClock::Config config;
    config.timestep = 1.0;
    SimClock clock{config};

    (void)clock.advance(0.4);  // 0.4 carried
    REQUIRE(clock.pending() == doctest::Approx(0.4));

    clock.request_single_step();
    (void)clock.advance(0.0);

    CHECK(clock.pending() == doctest::Approx(0.4));
}

// =======================================================================================
// the spiral of death
// =======================================================================================

TEST_CASE("a huge frame is capped rather than running forever")
{
    SimClock::Config config;
    config.timestep = 0.001;
    config.max_steps_per_frame = 16;
    SimClock clock{config};

    // Ten seconds of real time at a millisecond a step is ten thousand steps owed.
    CHECK(clock.advance(10.0) == 16);
    CHECK(clock.fell_behind());
}

TEST_CASE("the backlog is dropped, not carried")
{
    // Carrying it is what makes the spiral a spiral: every capped frame leaves more owed
    // than before, so the loop never recovers.
    SimClock::Config config;
    config.timestep = 0.001;
    config.max_steps_per_frame = 16;
    SimClock clock{config};

    (void)clock.advance(10.0);
    REQUIRE(clock.fell_behind());

    // Back to a normal frame: it should behave normally, not still be paying off a debt.
    CHECK(clock.advance(0.016) == 16);
    CHECK(clock.pending() < config.timestep);
}

TEST_CASE("fell_behind clears once the frame rate recovers")
{
    SimClock::Config config;
    config.timestep = 0.01;
    config.max_steps_per_frame = 8;
    SimClock clock{config};

    (void)clock.advance(5.0);
    REQUIRE(clock.fell_behind());

    (void)clock.advance(0.016);
    CHECK_FALSE(clock.fell_behind());
}

// =======================================================================================
// degenerate input
// =======================================================================================

TEST_CASE("nonsense deltas contribute nothing")
{
    SimClock::Config config;
    config.timestep = 1.0;
    SimClock clock{config};

    CHECK(clock.advance(0.0) == 0);
    CHECK(clock.advance(-5.0) == 0);  // a clock that jumps back is not a reason to rewind
    CHECK(clock.advance(std::numeric_limits<double>::quiet_NaN()) == 0);
    CHECK(clock.advance(std::numeric_limits<double>::infinity()) == 0);
    CHECK(clock.pending() == doctest::Approx(0.0));

    SUBCASE("and the clock still works afterwards")
    {
        CHECK(clock.advance(3.0) == 3);
    }
}

TEST_CASE("a broken config is repaired rather than trusted")
{
    SimClock::Config config;
    config.timestep = 0.0;
    config.time_scale = -1.0;
    config.max_steps_per_frame = 0;

    const SimClock clock{config};

    CHECK(clock.timestep() > 0.0);
    CHECK(clock.time_scale() > 0.0);
    CHECK(clock.config().max_steps_per_frame >= 1);
}

// =======================================================================================
// end to end: the thing the milestone actually promises
// =======================================================================================

TEST_CASE("a simulation driven at 30 and at 300 fps lands in the same place")
{
    // Not just the same step count: the same physical answer. This is the milestone test
    // for 0.2.0, and the reason frame delta-time never reaches the integrator.
    const BruteForceSolver solver;
    const double period =
        orbitalis::scenarios::circular_period(orbitalis::kSunGM + orbitalis::kEarthGM,
                                              orbitalis::kAstronomicalUnit);

    auto run_at = [&](double frame_seconds) {
        SimClock::Config config;
        config.timestep = period / 2000.0;
        config.time_scale = period / 10.0;  // one orbit per ten real seconds
        config.max_steps_per_frame = 100000;

        SimClock clock{config};
        System system = orbitalis::scenarios::sun_earth();
        SemiImplicitEuler integrator{solver};

        const int frames = static_cast<int>(std::round(5.0 / frame_seconds));
        for (int i = 0; i < frames; ++i) {
            const int steps = clock.advance(frame_seconds);
            for (int s = 0; s < steps; ++s) {
                integrator.step(system, clock.timestep());
            }
        }
        return system[1].position;
    };

    const Vec3 slow = run_at(1.0 / 30.0);
    const Vec3 fast = run_at(1.0 / 300.0);

    // Ten times the frames, identical trajectory. Both ran the same whole number of
    // identical fixed steps, so this is exact rather than merely close.
    CHECK(slow == fast);

    SUBCASE("and it actually moved")
    {
        const System start = orbitalis::scenarios::sun_earth();
        CHECK(orbitalis::distance(slow, start[1].position) > 0.1 * orbitalis::kAstronomicalUnit);
    }
}
