# ------------------------------------------------------------------------------
# orbitalis-cli -- headless driver. batch runs and binary export later on.
# links core ONLY. this is the target that has to keep working on a machine with
# no GPU and no display, which is what makes benchmarking honest.
# ------------------------------------------------------------------------------
add_executable(orbitalis-cli src/main.cpp)

target_link_libraries(orbitalis-cli PRIVATE orbitalis::core orbitalis::warnings)

set_target_properties(orbitalis-cli PROPERTIES FOLDER "orbitalis")
