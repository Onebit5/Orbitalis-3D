#include <orbitalis/core/Version.hpp>

#include <cstdio>

// milestone 0.0.1: prove the build system works and that an executable can link
// orbitalis-core. real argument parsing is 0.6.0 (step 0.5.4).
int main()
{
    std::printf("%s\n", orbitalis::version_banner());
    std::printf("  milestone : %s\n", orbitalis::milestone_name());
    std::printf("  target    : orbitalis-cli (headless)\n");
    std::printf("  next      : 0.0.2 -- Vec3 and the first real tests\n");
    return 0;
}
