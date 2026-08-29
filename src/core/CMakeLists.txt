# ------------------------------------------------------------------------------
# orbitalis-core -- the physics. no graphics, no I/O frameworks, no viewer.
# everything else in the project depends on this and it depends on nothing.
# ------------------------------------------------------------------------------

# expand Version.hpp.in -> build tree. generated headers go in the build dir, never
# the source dir, so a fresh clone is never polluted with build artefacts.
configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/include/orbitalis/core/Version.hpp.in"
    "${CMAKE_CURRENT_BINARY_DIR}/generated/orbitalis/core/Version.hpp"
    @ONLY
)

add_library(orbitalis-core STATIC
    src/Version.cpp
    src/System.cpp
    include/orbitalis/math/Vec3.hpp
    include/orbitalis/physics/Body.hpp
    include/orbitalis/physics/System.hpp
    "${CMAKE_CURRENT_BINARY_DIR}/generated/orbitalis/core/Version.hpp"
)
add_library(orbitalis::core ALIAS orbitalis-core)

target_include_directories(orbitalis-core
    PUBLIC
        "${CMAKE_CURRENT_SOURCE_DIR}/include"
        "${CMAKE_CURRENT_BINARY_DIR}/generated"
)

# PRIVATE: the warning flags apply to core's own translation units and do not leak
# into anything that links core.
target_link_libraries(orbitalis-core PRIVATE orbitalis::warnings)

set_target_properties(orbitalis-core PROPERTIES FOLDER "orbitalis")
