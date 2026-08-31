#include <orbitalis/physics/System.hpp>

#include <utility>

namespace orbitalis {

BodyId System::add(const Body& body, std::string name)
{
    const BodyId id = bodies_.size();
    bodies_.push_back(body);
    names_.push_back(std::move(name));
    return id;
}

void System::reserve(std::size_t n)
{
    bodies_.reserve(n);
    names_.reserve(n);
}

void System::clear() noexcept
{
    bodies_.clear();
    names_.clear();
}

void System::remove_net_drift() noexcept
{
    if (bodies_.empty()) {
        return;
    }

    const Vec3 drift = center_of_mass_velocity();
    for (Body& b : bodies_) {
        b.velocity -= drift;
    }
}

double System::total_mass() const noexcept
{
    double total = 0.0;
    for (const Body& b : bodies_) {
        total += b.mass;
    }
    return total;
}

Vec3 System::center_of_mass() const noexcept
{
    if (bodies_.empty()) {
        return Vec3{};
    }

    // Sum m*r first, divide once at the end. Dividing per body would be both slower and
    // less accurate, since every division rounds.
    Vec3 weighted{};
    double total = 0.0;
    for (const Body& b : bodies_) {
        weighted += b.mass * b.position;
        total += b.mass;
    }
    return weighted / total;
}

Vec3 System::total_momentum() const noexcept
{
    Vec3 p{};
    for (const Body& b : bodies_) {
        p += b.mass * b.velocity;
    }
    return p;
}

Vec3 System::center_of_mass_velocity() const noexcept
{
    if (bodies_.empty()) {
        return Vec3{};
    }
    return total_momentum() / total_mass();
}

}  // namespace orbitalis
