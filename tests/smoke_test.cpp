#include <orbitalis/core/Version.hpp>

#include <cstdio>
#include <cstring>

// milestone 0.0.1 claims "C++20". this turns that claim into something the compiler
// enforces instead of something I assume. note this only works because the build passes
// /Zc:__cplusplus -- without that flag MSVC reports 199711L no matter what standard is
// actually selected, which makes the macro useless and is why so many people think it
// is unreliable.
static_assert(__cplusplus >= 202002L,
              "Orbitalis needs C++20. If this fires on MSVC, check that /Zc:__cplusplus "
              "is still in cmake/OrbitalisWarnings.cmake.");

// deliberately NOT using <cassert>: NDEBUG is defined in Release builds, which turns
// assert() into a no-op and would make this test pass unconditionally. a test that
// cannot fail is worse than no test, because it looks like coverage.

namespace {

int failures = 0;

void check(bool condition, const char* what)
{
    if (!condition) {
        std::printf("  FAIL  %s\n", what);
        ++failures;
    } else {
        std::printf("  ok    %s\n", what);
    }
}

}  // namespace

int main()
{
    std::printf("orbitalis smoke test\n");

    const char* banner = orbitalis::version_banner();

    check(banner != nullptr && banner[0] != '\0', "version_banner() returns a non-empty string");
    check(orbitalis::milestone_name() != nullptr, "milestone_name() returns a string");

    // the banner must actually contain the version. this catches the generated header
    // going stale, which is the realistic failure mode here.
    check(banner != nullptr && std::strstr(banner, orbitalis::kVersionString) != nullptr,
          "banner contains kVersionString");

    // the split-out integers must agree with the string CMake generated. no hardcoded
    // "0.0.1" anywhere, so this test never needs touching on a version bump.
    char expected[64];
    std::snprintf(expected, sizeof(expected), "%d.%d.%d",
                  orbitalis::kVersionMajor, orbitalis::kVersionMinor, orbitalis::kVersionPatch);
    check(std::strcmp(expected, orbitalis::kVersionString) == 0,
          "major/minor/patch agree with kVersionString");

    std::printf("%s (%d failure(s))\n", failures == 0 ? "PASSED" : "FAILED", failures);
    return failures == 0 ? 0 : 1;
}
