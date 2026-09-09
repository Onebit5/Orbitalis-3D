#pragma once

namespace orbitalis::render {

/// Decides how many fixed simulation steps to run for a given amount of real time.
///
/// # why frame delta-time must never reach the integrator
///
/// The obvious loop is `integrator.step(system, frame_delta * scale)`, and it is wrong in
/// two separate ways.
///
/// It destroys **reproducibility**: the timestep is then whatever the frame happened to
/// take, so the same scenario gives different answers on a 60 Hz machine, a 144 Hz one, and
/// the same machine when something else grabs the CPU for a moment. There is no run to
/// compare against because there is no run twice.
///
/// It also destroys **energy behaviour**. Everything measured at 0.0.5 assumed a constant
/// dt: velocity Verlet is symplectic only for a fixed step, and a varying one turns its
/// bounded, oscillating error into a drifting one. A method chosen precisely because its
/// error does not accumulate quietly stops having that property.
///
/// So the fix is an accumulator. Real time goes in, whole fixed steps come out, and the
/// remainder is carried to the next frame. The timestep the integrator sees is always
/// exactly `timestep()`, and the *number* of steps absorbs the variation.
///
/// # the spiral of death
///
/// If a frame takes long enough to owe more steps than can be run in time, running them all
/// makes the next frame later still, which owes even more. The loop never catches up and
/// the application locks solid.
///
/// `max_steps_per_frame` caps it, and the backlog beyond the cap is **dropped** rather than
/// carried. That means simulated time falls behind real time under load, which is the right
/// trade: a simulation that runs slow is usable and one that hangs is not.
class SimClock
{
public:
    struct Config
    {
        /// Simulation seconds per fixed step. This is what the integrator receives, and it
        /// never varies.
        double timestep = 3600.0;

        /// Simulation seconds per real second. The user-facing speed control.
        double time_scale = 1.0;

        /// Upper bound on steps per call, guarding the spiral of death above.
        int max_steps_per_frame = 64;
    };

    SimClock() = default;
    explicit SimClock(const Config& config) noexcept;

    /// Returns how many fixed steps to run for `real_delta_seconds` of wall clock.
    ///
    /// Non-finite or negative deltas contribute nothing: a clock that jumps backwards is a
    /// platform problem, not a reason to run the simulation in reverse.
    [[nodiscard]] int advance(double real_delta_seconds) noexcept;

    /// Runs exactly one step on the next `advance`, even while paused, and leaves the
    /// accumulator alone so single-stepping does not drift the phase.
    void request_single_step() noexcept { single_step_ = true; }

    void set_paused(bool paused) noexcept { paused_ = paused; }
    [[nodiscard]] bool paused() const noexcept { return paused_; }

    /// Multiplies the current speed. Ignored if the factor is not positive and finite.
    void scale_speed(double factor) noexcept;

    void set_time_scale(double scale) noexcept;
    [[nodiscard]] double time_scale() const noexcept { return config_.time_scale; }

    void set_timestep(double seconds) noexcept;
    [[nodiscard]] double timestep() const noexcept { return config_.timestep; }

    /// Simulation seconds not yet handed out as whole steps. Always below `timestep()`.
    [[nodiscard]] double pending() const noexcept { return accumulator_; }

    /// Whether the last `advance` hit the cap and discarded backlog, meaning simulated time
    /// is now behind real time. Worth surfacing, because a viewer that silently runs slow
    /// looks like a physics bug.
    [[nodiscard]] bool fell_behind() const noexcept { return fell_behind_; }

    [[nodiscard]] const Config& config() const noexcept { return config_; }
    void set_config(const Config& config) noexcept;

private:
    Config config_{};
    double accumulator_{0.0};
    bool paused_{false};
    bool single_step_{false};
    bool fell_behind_{false};
};

}  // namespace orbitalis::render
