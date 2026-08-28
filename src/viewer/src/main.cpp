#include <orbitalis/core/Version.hpp>

#include <cstdio>

// milestone 0.0.1: a stub. there is no window yet -- raylib arrives at 0.1.1 and the
// first orbit I can actually watch is 0.2.0. the target exists now so that the
// core/viewer/cli separation is enforced by the build from commit one.
int main()
{
    std::printf("%s\n", orbitalis::version_banner());
    std::printf("  milestone : %s\n", orbitalis::milestone_name());
    std::printf("  target    : orbitalis-viewer (stub, no window yet)\n");
    std::printf("  next      : 0.1.1 -- raylib via FetchContent, an actual window\n");
    return 0;
}
