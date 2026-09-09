#include <orbitalis/integrators/Integrator.hpp>

#include <orbitalis/integrators/Euler.hpp>

#include <array>

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

constexpr std::array<Entry, 2> kRegistry{{
    {"forward-euler",
     [](const IForceSolver& solver) -> std::unique_ptr<IIntegrator> {
         return std::make_unique<ForwardEuler>(solver);
     }},
    {"semi-implicit-euler",
     [](const IForceSolver& solver) -> std::unique_ptr<IIntegrator> {
         return std::make_unique<SemiImplicitEuler>(solver);
     }},
}};

// Held separately so integrator_names() can hand back a span without building anything.
constexpr std::array<std::string_view, kRegistry.size()> kNames{
    kRegistry[0].name,
    kRegistry[1].name,
};

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
