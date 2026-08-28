#include <doctest/doctest.h>

#include <orbitalis/core/Version.hpp>

#include <cstdio>
#include <cstring>

// The project claims C++20. This turns that claim into something the compiler enforces
// rather than something I assume. It only works because the build passes
// /Zc:__cplusplus; without that flag MSVC reports 199711L regardless of which standard
// is actually selected, which is why the macro has a reputation for being useless.
static_assert(__cplusplus >= 202002L,
              "Orbitalis needs C++20. If this fires on MSVC, check that /Zc:__cplusplus "
              "is still in cmake/OrbitalisWarnings.cmake.");

TEST_CASE("version banner is populated")
{
    const char* banner = orbitalis::version_banner();

    REQUIRE(banner != nullptr);
    CHECK(banner[0] != '\0');
    CHECK(orbitalis::milestone_name() != nullptr);
}

TEST_CASE("version banner contains the version string")
{
    // Catches the generated header going stale, which is the realistic failure mode.
    const char* banner = orbitalis::version_banner();

    REQUIRE(banner != nullptr);
    CHECK(std::strstr(banner, orbitalis::kVersionString) != nullptr);
}

TEST_CASE("split version integers agree with the version string")
{
    // No hardcoded version number anywhere, so this test never needs touching on a bump.
    char expected[64];
    std::snprintf(expected, sizeof(expected), "%d.%d.%d",
                  orbitalis::kVersionMajor,
                  orbitalis::kVersionMinor,
                  orbitalis::kVersionPatch);

    CHECK(std::strcmp(expected, orbitalis::kVersionString) == 0);
}
