# ------------------------------------------------------------------------------
# orbitalis-viewer -- the real-time 3D window.
#
# right now it is a console stub: raylib does not arrive until milestone 0.2.0
# (step 0.1.1). the target exists this early on purpose, so the three-way split is
# real from the first commit instead of being a refactor I keep postponing.
# ------------------------------------------------------------------------------
add_executable(orbitalis-viewer src/main.cpp)

target_link_libraries(orbitalis-viewer PRIVATE orbitalis::core orbitalis::warnings)

set_target_properties(orbitalis-viewer PROPERTIES FOLDER "orbitalis")
