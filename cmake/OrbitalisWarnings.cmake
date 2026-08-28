# ------------------------------------------------------------------------------
# one INTERFACE target carrying the compiler flags every orbitalis target wants.
# targets link it PRIVATE, so the flags apply to my code and never leak out to
# anything that consumes the library.
# ------------------------------------------------------------------------------
add_library(orbitalis_warnings INTERFACE)
add_library(orbitalis::warnings ALIAS orbitalis_warnings)

if(MSVC)
    target_compile_options(orbitalis_warnings INTERFACE
        /W4               # high but usable. NOT /Wall -- that warns inside <vector> and
                          # buries real problems under thousands of system-header lines.
        /permissive-      # actually enforce the standard, no MSVC-isms
        /utf-8            # source + execution charset = UTF-8. without this MSVC reads
                          # files in the system codepage and mangles the r^2, theta and
                          # epsilon characters in my physics comments (warning C4819).
        /Zc:__cplusplus   # otherwise __cplusplus lies and reports 199711L forever
        /Zc:preprocessor  # conforming preprocessor
        /MP               # compile translation units in parallel
    )
    if(ORBITALIS_WARNINGS_AS_ERRORS)
        target_compile_options(orbitalis_warnings INTERFACE /WX)
    endif()
else()
    # dormant for now (I'm on MSVC), but keeping it means the day I try clang or gcc
    # I find out immediately instead of discovering 200 portability bugs at once.
    target_compile_options(orbitalis_warnings INTERFACE
        -Wall -Wextra -Wpedantic
        -Wshadow             # shadowed variables, an actual source of physics bugs
        -Wconversion         # silent narrowing, e.g. double -> float at the render edge
        -Wdouble-promotion   # accidental float -> double, kills vectorisation
        -Wnon-virtual-dtor
        -Wold-style-cast
    )
    if(ORBITALIS_WARNINGS_AS_ERRORS)
        target_compile_options(orbitalis_warnings INTERFACE -Werror)
    endif()
endif()

# deliberately OFF by default. warnings-as-errors is milestone 0.7.0 (0.6.7) work -- I do
# not want a stray C4244 blocking me from seeing whether Earth orbits the Sun.
