#include <orbitalis/integrators/Integrator.hpp>

#include <orbitalis/integrators/Euler.hpp>
#include <orbitalis/integrators/Verlet.hpp>

#include <array>
#include <cstddef>

namespace orbitalis {

namespace {

// The single source of truth for which integrators exist. integrator_names() and
// make_integrator() both read from it, so the list and the factory cannot drift apart:
// adding a method means adding one row here, and there is a round-trip test asserting that
// every listed name constructs something that reports that same name back.
struct Entry
{
    std::string_view name;
    std::unique_ptr<IIntegrator> (*create)(const IForceSolver&);
};

constexpr std::array<Entry, 3> kRegistry{{
    {"forward-euler",
     [](const IForceSolver& solver) -> std::unique_ptr<IIntegrator> {
         return std::make_unique<ForwardEuler>(solver);
     }},
    {"semi-implicit-euler",
     [](const IForceSolver& solver) -> std::unique_ptr<IIntegrator> {
         return std::make_unique<SemiImplicitEuler>(solver);
     }},
    {"velocity-verlet",
     [](const IForceSolver& solver) -> std::unique_ptr<IIntegrator> {
         return std::make_unique<VelocityVerlet>(solver);
     }},
}};

// Held separately so integrator_names() can hand back a span without building anything.
//
// Derived from kRegistry rather than written out, because listing the names by hand made
// "adding a method is one row" quietly false: velocity Verlet at 0.2.2 needed two edits,
// and forgetting the second would have compiled and produced an integrator reachable by
// make_integrator() but invisible to the viewer's cycle key and to the 0.2.6 harness.
constexpr auto kNames = [] {
    std::array<std::string_view, kRegistry.size()> names{};
    for (std::size_t i = 0; i < kRegistry.size(); ++i) {
        names[i] = kRegistry[i].name;
    }
    return names;
}();

}  // namespace

std::span<const std::string_view> integrator_names() noexcept
{
    return kNames;
}

std::unique_ptr<IIntegrator> make_integrator(std::string_view name, const IForceSolver& solver)
{
    for (const Entry& entry : kRegistry) {
        if (entry.name == name) {
            return entry.create(solver);
        }
    }
    return nullptr;
}

}  // namespace orbitalis
