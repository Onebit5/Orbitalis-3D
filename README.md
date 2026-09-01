# Orbitalis-3D

a 3D n-body simulator in C++ that computes gravitational trajectories for planetary
systems by numerically solving the equations of motion. newtonian physics, `double`
precision, written from scratch.

**current version: 0.1.2** — the render scale layer. see [status](#status) for exactly what
does and doesn't exist yet.

---

## status

this is early. what works today:

| | |
|---|---|
| build system | [x] CMake, three targets, Debug + Release, zero warnings |
| tests | [x] doctest, 91 cases |
| vector maths | [x] Vec3, header-only |
| bodies | [x] Body, System, barycentre |
| gravity | [x] brute force O(n^2), softening |
| integrators | [x] Euler, forward + semi-implicit |
| 3D viewer | [~] bodies on screen, no camera controls yet |
| Barnes-Hut | [ ] 0.4.0 |
| binary export | [ ] 0.5.0 |

the physics core is complete. `orbitalis-cli` runs a Sun-Earth system for one sidereal
year and Earth returns to within 0.0013 AU of where it started, on a path 940 million km
long. the computed orbital period lands within 1.1 seconds of the real sidereal year.

`orbitalis-viewer` draws the Sun-Earth system through a camera-relative render frame. the
camera is fixed and nothing moves yet; controls arrive at 0.1.4 and the sim loop at 0.1.6. the full plan is in [ROADMAP.md](ROADMAP.md).

## how it's put together

three build targets, and the dependency arrow only points one way:

```
              orbitalis-core
              static library
        the physics — knows nothing
            about pixels or I/O
                ^         ^
                |         |
      orbitalis-viewer   orbitalis-cli
       real-time 3D       headless runner
       (raylib, 0.2.0)    (batch + export)
```

`orbitalis-core` has **zero graphics dependencies** and never will. that's not tidiness
for its own sake — milestone 0.4.0 benchmarks Barnes-Hut at 100k bodies, and those numbers
are only trustworthy if they can be produced headless, with no window and no GPU.
`orbitalis-cli` exists to keep that path honest.

a few conventions that run through everything:

- **`double` in the simulation, `float` only at the render boundary.** a `float` can't
  represent Earth's orbital radius to better than ~16 km, which is larger than the
  distance Earth moves in a single timestep — so the position update would round to no
  change at all and the planet would never move.
- **SI units internally** (metres, kilograms, seconds). scenario files may use AU, days
  and solar masses; conversion happens at the I/O boundary only.
- **fixed simulation timestep, decoupled from framerate.** frame delta-time never reaches
  the integrator — that would make runs unreproducible and wreck energy conservation.

## requirements

- **CMake** ≥ 3.25
- a **C++20** compiler

developed against Visual Studio Community 2026 (MSVC 19.51) on Windows 11. nothing in the
code is MSVC-specific, but the supplied preset targets the VS generator.

## build

```sh
cmake --preset default
cmake --build --preset debug      # or: --preset release
```

binaries land in `build/bin/<config>/`:

```sh
./build/bin/Debug/orbitalis-cli.exe
./build/bin/Debug/orbitalis-viewer.exe
```

### tests

```sh
ctest --preset debug              # or: --preset release
```

### without the preset

if the `Visual Studio 18 2026` generator isn't what you have, skip the preset and pick
your own:

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build
```

### build options

| option | default | |
|---|---|---|
| `ORBITALIS_BUILD_VIEWER` | `ON` | build the 3D viewer |
| `ORBITALIS_BUILD_CLI` | `ON` | build the headless runner |
| `ORBITALIS_BUILD_TESTS` | `ON` | build the test suite |
| `ORBITALIS_WARNINGS_AS_ERRORS` | `OFF` | `/WX` — turned on deliberately at 0.7.0 |

## layout

```
src/core/      orbitalis-core — physics, no dependencies
src/render/    orbitalis-render — view maths, no graphics API
src/viewer/    orbitalis-viewer — real-time 3D
src/cli/       orbitalis-cli — headless
tests/         ctest suite
scenarios/     initial-condition files (0.6.0)
tools/         python readers for exported trajectories (0.5.0)
cmake/         shared CMake modules
```

## note on vcpkg

if you have vcpkg installed with `vcpkg integrate install`, its machine-wide MSBuild hook
appends `installed/<triplet>/lib/*.lib` — a wildcard — to the link line of every MSBuild
C++ project on the system. this build sets `VcpkgEnabled=false` via `CMAKE_VS_GLOBALS` to
opt out, so that `orbitalis-core` genuinely depends on nothing. it's project-local and
doesn't disturb your global vcpkg setup.
