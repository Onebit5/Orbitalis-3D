#pragma once

#include <orbitalis/math/Vec3.hpp>
#include <orbitalis/physics/Body.hpp>

#include <cstddef>
#include <span>
#include <vector>

namespace orbitalis::render {

/// A fixed-capacity ring buffer of a body's past positions.
///
/// # why these are simulation coordinates, not render ones
///
/// The obvious implementation stores whatever went to the GPU, which is a `Vec3f` in render
/// space. It is wrong, and the failure is instructive.
///
/// Render coordinates are relative to `RenderFrame::focus`. Click a different body to follow
/// and the focus jumps by an astronomical unit, so every historical point in the trail is
/// now expressed against a different origin than the one it was recorded in. The trail does
/// not move with the body: it smears across the screen, and the shape you were looking at
/// stops being a shape.
///
/// Storing metres and converting at draw time makes a trail mean the same thing in every
/// frame of reference. It costs one conversion per point per frame, and it is the same rule
/// that has decided several designs already: **keep the wide, frame-independent
/// representation and convert late.**
///
/// # sampling
///
/// Points are recorded by distance travelled rather than once per frame. A body near
/// perihelion moves several times faster than at aphelion, so time-based sampling bunches
/// points where the body is slow and stretches them where it is fast, which is exactly
/// backwards: the fast part is where the curve bends most and needs the most points.
/// Distance-based sampling gives a uniform spatial resolution and a smooth line at any
/// speed, and it costs nothing when a body is barely moving.
class Trail
{
public:
    Trail() = default;
    explicit Trail(std::size_t capacity);

    /// Records `position` only if it is at least `min_separation` metres from the last
    /// point recorded. Returns whether it was kept.
    ///
    /// A `min_separation` of zero records unconditionally.
    bool sample(const Vec3& position, double min_separation);

    /// Records unconditionally, dropping the oldest point when full.
    void push(const Vec3& position);

    void clear() noexcept;

    /// Changes capacity. Existing points are discarded rather than partially kept, because
    /// a trail missing its middle is worse than no trail.
    void set_capacity(std::size_t capacity);

    [[nodiscard]] std::size_t size() const noexcept { return count_; }
    [[nodiscard]] std::size_t capacity() const noexcept { return points_.size(); }
    [[nodiscard]] bool empty() const noexcept { return count_ == 0; }
    [[nodiscard]] bool full() const noexcept { return count_ == points_.size(); }

    /// Chronological: index 0 is the oldest point still retained.
    ///
    /// Precondition: `index < size()`.
    [[nodiscard]] const Vec3& operator[](std::size_t index) const noexcept;

    [[nodiscard]] const Vec3& oldest() const noexcept { return (*this)[0]; }
    [[nodiscard]] const Vec3& newest() const noexcept { return (*this)[count_ - 1]; }

private:
    std::vector<Vec3> points_;

    /// Where the next push goes. When the buffer is full this is also the oldest point.
    std::size_t head_{0};

    std::size_t count_{0};
};

/// One Trail per body, kept in step with the System.
class TrailSet
{
public:
    TrailSet() = default;
    TrailSet(std::size_t body_count, std::size_t capacity_per_body);

    /// Matches the trail count to the system. Existing trails keep their history; new ones
    /// start empty.
    void resize(std::size_t body_count);

    /// Samples every body. Bodies whose trails are all at capacity simply drop their oldest
    /// point, so this is safe to call every frame forever.
    void sample(std::span<const Body> bodies, double min_separation);

    void clear() noexcept;

    void set_capacity_per_body(std::size_t capacity);

    [[nodiscard]] std::size_t size() const noexcept { return trails_.size(); }
    [[nodiscard]] std::size_t capacity_per_body() const noexcept { return capacity_; }

    [[nodiscard]] const Trail& operator[](std::size_t index) const noexcept
    {
        return trails_[index];
    }

private:
    std::vector<Trail> trails_;
    std::size_t capacity_{512};
};

}  // namespace orbitalis::render
