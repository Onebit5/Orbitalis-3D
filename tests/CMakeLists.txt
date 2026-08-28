# ------------------------------------------------------------------------------
# tests.
#
# no doctest yet -- that lands at 0.0.2 together with Vec3, which is the first
# thing actually worth asserting on. for now this is a hand-rolled smoke test whose
# only job is to prove the ctest plumbing works, so that 0.0.2 is pure Vec3 work
# and not a fight with CMake.
# ------------------------------------------------------------------------------
add_executable(orbitalis-smoketest smoke_test.cpp)

target_link_libraries(orbitalis-smoketest PRIVATE orbitalis::core orbitalis::warnings)

set_target_properties(orbitalis-smoketest PROPERTIES FOLDER "orbitalis/tests")

add_test(NAME smoke COMMAND orbitalis-smoketest)
