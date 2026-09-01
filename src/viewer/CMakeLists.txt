# ------------------------------------------------------------------------------
# orbitalis-viewer -- the real-time 3D window.
#
# This is the ONLY target that links raylib. orbitalis-core must never gain a
# graphics dependency: milestone 0.4.0 benchmarks Barnes-Hut at 100k bodies and
# those numbers are only trustworthy produced headless, on a machine with no GPU.
# There is a configure-time assertion enforcing that at the bottom of this file.
# ------------------------------------------------------------------------------
include(FetchContent)

# raylib's own CMake builds its examples by default, which is roughly a hundred
# extra targets I will never run. Set before MakeAvailable so they take effect.
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_GAMES    OFF CACHE BOOL "" FORCE)

# Static, so the viewer is a single file to hand someone. Explicit rather than
# inherited, because BUILD_SHARED_LIBS is a global and I do not want raylib's
# linkage to change because something else set it.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG        6.0        # pinned, same reasoning as doctest: a floating tag
                              # makes my build depend on what upstream did today.
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(raylib)

add_executable(orbitalis-viewer src/main.cpp)

target_link_libraries(orbitalis-viewer PRIVATE
    orbitalis::core
    orbitalis::render
    orbitalis::warnings
    raylib
)

# raylib's headers are not mine, so I do not want /W4 findings from them mixed in
# with findings about my own code.
get_target_property(raylib_includes raylib INTERFACE_INCLUDE_DIRECTORIES)
if(raylib_includes)
    target_include_directories(orbitalis-viewer SYSTEM PRIVATE ${raylib_includes})
endif()

set_target_properties(orbitalis-viewer PROPERTIES FOLDER "orbitalis")

# raylib is a third-party target; keep it out of the top level of the IDE tree.
foreach(dep IN ITEMS raylib glfw)
    if(TARGET ${dep})
        set_target_properties(${dep} PROPERTIES FOLDER "external")
    endif()
endforeach()

# --version exits before InitWindow, so this runs fine with no display attached.
# It is the only part of the viewer that CI can exercise, and it is enough to
# catch the link breaking.
#
# Requires enable_testing() to have run already, which the root CMakeLists now does
# before descending into any subdirectory. Without that, add_test here is silently
# ignored -- no warning, no error, the test just never shows up in ctest.
if(ORBITALIS_BUILD_TESTS)
    add_test(NAME viewer-starts COMMAND orbitalis-viewer --version)
endif()

# ------------------------------------------------------------------------------
# The architecture rule, enforced instead of merely documented.
#
# It has been a comment since 0.0.1. A comment does not stop anyone (me, in eight
# months) from adding raylib to core's link line to solve some small problem, and
# by the time the benchmarks fail the change is buried. This makes it a configure
# error.
# ------------------------------------------------------------------------------
# orbitalis-render is guarded too, as of 0.1.2. It is view-layer code, but the test
# suite links it and the test suite has to keep running on a machine with no display.
foreach(_guarded IN ITEMS orbitalis-core orbitalis-render)
    if(NOT TARGET ${_guarded})
        continue()
    endif()

    get_target_property(_links ${_guarded} LINK_LIBRARIES)
    if(_links MATCHES "raylib|glfw|opengl|OpenGL|SDL")
        message(FATAL_ERROR
            "${_guarded} links a graphics library (${_links}).\n"
            "it must stay headless: the Barnes-Hut benchmarks at 0.4.0 and the whole "
            "test suite have to run without a display. Link graphics into "
            "orbitalis-viewer instead.")
    endif()
endforeach()
