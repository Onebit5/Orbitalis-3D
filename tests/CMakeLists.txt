# ------------------------------------------------------------------------------
# tests.
#
# doctest arrives here at 0.0.2, replacing the hand-rolled smoke test from 0.0.1.
# it is a single header, compiles fast, and pulls in nothing else, which is the whole
# reason it won over Catch2 and GoogleTest. the hand-written physics stays hand-written
# (that is the project); the test harness is a chore and I am not writing one.
# ------------------------------------------------------------------------------
include(FetchContent)

# doctest builds its own test suite and install rules by default when it is the top
# level project. it is not, but set these explicitly so a future doctest version cannot
# surprise me with several hundred extra targets.
set(DOCTEST_WITH_TESTS            OFF CACHE BOOL "" FORCE)
set(DOCTEST_WITH_MAIN_IN_STATIC_LIB OFF CACHE BOOL "" FORCE)
set(DOCTEST_NO_INSTALL            ON  CACHE BOOL "" FORCE)

FetchContent_Declare(
    doctest
    GIT_REPOSITORY https://github.com/doctest/doctest.git
    GIT_TAG        v2.5.3      # pinned. a floating tag would make my builds depend on
                               # what upstream did this morning.
    GIT_SHALLOW    TRUE
)

FetchContent_MakeAvailable(doctest)

add_executable(orbitalis-tests
    test_main.cpp
    test_version.cpp
    math/test_vec3.cpp
    physics/test_body.cpp
    physics/test_system.cpp
    physics/test_brute_force_solver.cpp
    integrators/test_euler.cpp
    scenarios/test_sun_earth.cpp
    render/test_render_frame.cpp
)

target_link_libraries(orbitalis-tests PRIVATE
    orbitalis::core
    orbitalis::render
    orbitalis::warnings
    doctest::doctest
)

# doctest forward-declares std::basic_ostream rather than including <ostream>, to keep its
# own compile times down. that breaks the moment something needs the complete type: MSVC's
# string_view header declares an operator<< for string_view, so simply writing
# CHECK(system.name(id) == "Sun") fails to compile with a wall of errors pointing into
# __msvc_string_view.hpp rather than at my code. this tells doctest to include the real
# standard headers instead.
target_compile_definitions(orbitalis-tests PRIVATE DOCTEST_CONFIG_USE_STD_HEADERS)

# doctest's headers are not mine, so I do not want /W4 findings from them mixed in with
# findings about my own code. SYSTEM suppresses warnings originating inside them.
get_target_property(doctest_includes doctest::doctest INTERFACE_INCLUDE_DIRECTORIES)
if(doctest_includes)
    target_include_directories(orbitalis-tests SYSTEM PRIVATE ${doctest_includes})
endif()

set_target_properties(orbitalis-tests PROPERTIES FOLDER "orbitalis/tests")

# registers each TEST_CASE as its own ctest entry, so a failure names the case rather
# than just saying "the test executable returned 1".
include("${doctest_SOURCE_DIR}/scripts/cmake/doctest.cmake")
doctest_discover_tests(orbitalis-tests)
