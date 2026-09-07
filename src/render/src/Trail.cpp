#include <orbitalis/render/Trail.hpp>

#include <algorithm>
#include <cmath>

namespace orbitalis::render {

Trail::Trail(std::size_t capacity)
    : points_(capacity)
{
}

void Trail::set_capacity(std::size_t capacity)
{
    points_.assign(capacity, Vec3{});
    head_ = 0;
    count_ = 0;
}

void Trail::clear() noexcept
{
    head_ = 0;
    count_ = 0;
}

void Trail::push(const Vec3& position)
{
    if (points_.empty()) {
        // A zero-capacity trail is a legitimate way to say "do not trail this body", which
        // matters at 0.4.0 where trailing all 100,000 of them is not an option.
        return;
    }

    points_[head_] = position;
    head_ = (head_ + 1) % points_.size();

    if (count_ < points_.size()) {
        ++count_;
    }
}

bool Trail::sample(const Vec3& position, double min_separation)
{
    if (points_.empty() || !position.is_finite()) {
        return false;
    }

    if (count_ > 0 && min_separation > 0.0) {
        // Compared squared, so no square root on a path that runs every body every frame.
        if (distance_squared(newest(), position) < min_separation * min_separation) {
            return false;
        }
    }

    push(position);
    return true;
}

const Vec3& Trail::operator[](std::size_t index) const noexcept
{
    // While the buffer is still filling, points sit at 0..count_-1 and the oldest is at 0.
    // Once it has wrapped, head_ points at the oldest.
    const std::size_t start = (count_ < points_.size()) ? 0 : head_;
    return points_[(start + index) % points_.size()];
}

// ---------------------------------------------------------------------------------------

TrailSet::TrailSet(std::size_t body_count, std::size_t capacity_per_body)
    : capacity_(capacity_per_body)
{
    resize(body_count);
}

void TrailSet::resize(std::size_t body_count)
{
    if (body_count == trails_.size()) {
        return;
    }

    // Shrinking drops the tail; growing appends empty trails. Either way the trails that
    // survive keep their history, because a body that did not change index did not change
    // identity.
    //
    // That assumption is exactly the BodyId-is-an-index limitation written down at 0.0.3,
    // and collision merging at 0.7.1 will break it: removing a body shifts every id above
    // it, and the trails would silently attach to the wrong bodies. Noted there, noted here.
    trails_.resize(body_count, Trail{capacity_});
}

void TrailSet::sample(std::span<const Body> bodies, double min_separation)
{
    const std::size_t count = std::min(bodies.size(), trails_.size());

    for (std::size_t i = 0; i < count; ++i) {
        trails_[i].sample(bodies[i].position, min_separation);
    }
}

void TrailSet::clear() noexcept
{
    for (Trail& trail : trails_) {
        trail.clear();
    }
}

void TrailSet::set_capacity_per_body(std::size_t capacity)
{
    capacity_ = capacity;
    for (Trail& trail : trails_) {
        trail.set_capacity(capacity);
    }
}

}  // namespace orbitalis::render
