#include <orbitalis/render/SimClock.hpp>

#include <algorithm>
#include <cmath>

namespace orbitalis::render {

namespace {

SimClock::Config sanitise(SimClock::Config config) noexcept
{
    if (!(config.timestep > 0.0) || !std::isfinite(config.timestep)) {
        config.timestep = 3600.0;
    }
    if (!(config.time_scale > 0.0) || !std::isfinite(config.time_scale)) {
        config.time_scale = 1.0;
    }
    config.max_steps_per_frame = std::clamp(config.max_steps_per_frame, 1, 100000);
    return config;
}

}  // namespace

SimClock::SimClock(const Config& config) noexcept
    : config_(sanitise(config))
{
}

void SimClock::set_config(const Config& config) noexcept
{
    config_ = sanitise(config);
}

void SimClock::set_timestep(double seconds) noexcept
{
    Config config = config_;
    config.timestep = seconds;
    config_ = sanitise(config);
}

void SimClock::set_time_scale(double scale) noexcept
{
    Config config = config_;
    config.time_scale = scale;
    config_ = sanitise(config);
}

void SimClock::scale_speed(double factor) noexcept
{
    if (!(factor > 0.0) || !std::isfinite(factor)) {
        return;
    }
    set_time_scale(config_.time_scale * factor);
}

int SimClock::advance(double real_delta_seconds) noexcept
{
    fell_behind_ = false;

    // Single-stepping wins over pause, and deliberately does not touch the accumulator: it
    // is a debugging action, not a slice of elapsed time, so it must not shift the phase of
    // the ordinary stepping when play resumes.
    if (single_step_) {
        single_step_ = false;
        return 1;
    }

    if (paused_) {
        return 0;
    }

    if (!std::isfinite(real_delta_seconds) || real_delta_seconds <= 0.0) {
        return 0;
    }

    accumulator_ += real_delta_seconds * config_.time_scale;

    const double whole = std::floor(accumulator_ / config_.timestep);

    // The division is guarded against a delta so enormous it overflows an int, which is
    // reachable after a breakpoint or a laptop waking from sleep.
    const double capped = std::min(whole, static_cast<double>(config_.max_steps_per_frame));
    const int steps = static_cast<int>(std::max(0.0, capped));

    if (whole > static_cast<double>(config_.max_steps_per_frame)) {
        // Drop the backlog rather than carry it. Carrying it is what turns one slow frame
        // into a permanent one.
        accumulator_ = 0.0;
        fell_behind_ = true;
    } else {
        accumulator_ -= steps * config_.timestep;
    }

    return steps;
}

}  // namespace orbitalis::render
