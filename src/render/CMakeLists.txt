# ------------------------------------------------------------------------------
# orbitalis-render -- the view layer, minus the graphics API.
#
# New at 0.1.2, and worth explaining. Everything the viewer does that is not an
# actual draw call is arithmetic: converting simulation coordinates into render
# coordinates without losing precision (here), sizing bodies so they are visible
# at true scale (0.1.3), orbiting a camera (0.1.4), keeping a ring buffer of past
# positions (0.1.5), accumulating a fixed timestep (0.1.6).
#
# All of that has real edge cases and deserves tests, and none of it needs raylib.
# But orbitalis-tests cannot link raylib: there is no display on a build agent.
#
# So the view layer splits in two. This library holds the logic and links only
# orbitalis-core. orbitalis-viewer is the thin raylib shell over it, and is the
# only target that needs a GPU.
#
# It also means the same rule that protects orbitalis-core protects this: no
# graphics dependency, enforced at the bottom of src/viewer/CMakeLists.txt.
# ------------------------------------------------------------------------------
add_library(orbitalis-render STATIC
    src/RenderFrame.cpp
    include/orbitalis/render/RenderFrame.hpp
)
add_library(orbitalis::render ALIAS orbitalis-render)

target_include_directories(orbitalis-render
    PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(orbitalis-render
    PUBLIC  orbitalis::core        # PUBLIC: Vec3 and Body are in this library's headers
    PRIVATE orbitalis::warnings
)

set_target_properties(orbitalis-render PROPERTIES FOLDER "orbitalis")
