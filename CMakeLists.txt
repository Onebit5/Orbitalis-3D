cmake_minimum_required(VERSION 3.25)

project(Orbitalis3D
    VERSION     0.0.4
    DESCRIPTION "3D newtonian n-body gravitational simulator"
    LANGUAGES   CXX
)

# ------------------------------------------------------------------------------
# options
# ------------------------------------------------------------------------------
option(ORBITALIS_BUILD_VIEWER       "Build the real-time 3D viewer"     ON)
option(ORBITALIS_BUILD_CLI          "Build the headless CLI"            ON)
option(ORBITALIS_BUILD_TESTS        "Build the test suite"              ON)
option(ORBITALIS_WARNINGS_AS_ERRORS "Treat compiler warnings as errors" OFF)

# ------------------------------------------------------------------------------
# language settings
#
# C++20 is required, not "preferred" -- STANDARD_REQUIRED makes CMake hard-fail on a
# compiler that can't do it instead of silently falling back to C++14 and handing me a
# thousand confusing errors later. EXTENSIONS OFF keeps me on ISO C++ (no /Za-style
# vendor extras) so the code stays portable if I ever leave MSVC.
# ------------------------------------------------------------------------------
set(CMAKE_CXX_STANDARD          20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS        OFF)

# single-config generators (Ninja, Makefiles) default to *no* build type, which means no
# optimisation AND no debug info. multi-config generators (Visual Studio) ignore this.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# put binaries somewhere predictable. $<CONFIG> keeps Debug and Release from stomping on
# each other, which they otherwise do under the Visual Studio generator.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/$<CONFIG>")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/$<CONFIG>")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib/$<CONFIG>")

set_property(GLOBAL PROPERTY USE_FOLDERS ON)


# ------------------------------------------------------------------------------
# turn OFF vcpkg's global MSBuild integration for this project.
#
# there is a vcpkg at C:/Users/azpij/vcpkg with `vcpkg integrate install` active,
# which injects itself into EVERY MSBuild C++ project on this machine. two problems:
#
#   1. it appends "<vcpkg>/installed/x64-windows/lib/*.lib" to my link line -- a
#      wildcard. so whatever happens to be installed in vcpkg gets handed to the
#      linker. right now that is lua.lib. my core library is supposed to depend on
#      nothing, and "nothing" does not include a scripting language I never asked for.
#   2. its applocal step shells out to pwsh.exe (PowerShell 7), which is not installed
#      here, so every single build printed a 9009 error before falling back to
#      Windows PowerShell 5.1.
#
# CMAKE_VS_GLOBALS sets an MSBuild global property on every target generated after
# this point. this is project-local: it does not touch the machine-wide vcpkg
# integration, so my other projects keep working exactly as before.
#
# if I ever DO want vcpkg here, flip this off deliberately rather than by accident.
# ------------------------------------------------------------------------------
set(CMAKE_VS_GLOBALS "VcpkgEnabled=false")

include(cmake/OrbitalisWarnings.cmake)

# ------------------------------------------------------------------------------
# targets
#
# the dependency direction is the whole architecture and it only points one way:
#
#     orbitalis-core  (static lib, physics, ZERO graphics dependencies)
#         ^      ^
#         |      |
#      viewer   cli
#
# core must never link raylib. the day it does, I can't benchmark or run headless on a
# machine without a GPU -- which is exactly what milestone 0.4.0 needs. see
# docs/devlog/0.0.1.md for the longer rant.
# ------------------------------------------------------------------------------
add_subdirectory(src/core)

if(ORBITALIS_BUILD_CLI)
    add_subdirectory(src/cli)
endif()

if(ORBITALIS_BUILD_VIEWER)
    add_subdirectory(src/viewer)
endif()

if(ORBITALIS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# so hitting F5 in Visual Studio runs something useful instead of ALL_BUILD
if(MSVC AND ORBITALIS_BUILD_CLI)
    set_property(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}"
                 PROPERTY VS_STARTUP_PROJECT orbitalis-cli)
endif()

message(STATUS "Orbitalis3D ${PROJECT_VERSION} | viewer=${ORBITALIS_BUILD_VIEWER} cli=${ORBITALIS_BUILD_CLI} tests=${ORBITALIS_BUILD_TESTS}")
