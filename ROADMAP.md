# roadmap

**where I am: 0.1.4 done.** drag, zoom, click or TAB to follow a body. next up is 0.1.5
(trails).

a milestone is **done** when the thing it promises actually works and I wrote down how I
proved it. not when the code compiles.

| milestone | | |
| --- | --- | --- |
| 0.1.0 | the core — Vec3, gravity, first orbit | **done** |
| 0.2.0 | live 3D viewer | in progress |
| 0.3.0 | integrators — Verlet, RK4, RKF45 | |
| 0.4.0 | Barnes-Hut octree + threading | |
| 0.5.0 | binary trajectory export | |
| 0.6.0 | orbital elements, scenarios, CLI | |
| 0.7.0 | validation and benchmarks | |
| 0.8.0 | collisions and extras | |
| 1.0.0 | docs, examples, release | |

---

## 0.1.0 — the core. two bodies that actually orbit

the boring foundation. no optimisation, no cleverness, just correct newtonian gravity.

- [x] **0.0.1** — repo, folder layout, root CMakeLists with three targets, C++20, warnings
  on, builds a hello world
- [x] **0.0.2** — `Vec3`: operators, dot, cross, length, length_squared, normalized.
  header-only, constexpr where possible. doctest arrives here, first real tests
- [x] **0.0.3** — `Body` (mass, radius, position, velocity, acceleration, id, name) and a
  `System` that owns `std::vector<Body>`
- [x] **0.0.4** — `BruteForceSolver`: pairwise newtonian gravity, O(n²), using newton's
  third law so each pair is computed once. plummer softening `r/(r²+ε²)^(3/2)` so nothing
  explodes at close range. ε configurable, and it's a fudge rather than physics
- [x] **0.0.5** — Euler integrators. wrote *two*: forward (the control group, spirals out
  20% per orbit) and semi-implicit (symplectic, stays closed). the second one exists so
  0.0.6 can tell "the method is bad" apart from "the solver is wrong"
- [x] **0.0.6** — Sun–Earth scenario in the barycentric frame, run by `orbitalis-cli`.
  found my solar mass was 0.026% high, which cost 68 minutes of year length; `Constants.hpp`
  now derives mass from IAU GM values and the computed year lands within 1.1 seconds of the
  real sidereal year

**done when:** Earth goes around the Sun once and doesn't fly off into nowhere. energy
drift is bad and I know it's bad.

**shipped 2026-08-31.** return distance 0.001319 AU after one sidereal year under
semi-implicit Euler, against 0.887 AU for forward Euler at identical cost. momentum holds
to machine precision under both. 78 tests, 269 assertions.

## 0.2.0 — I can watch it

moved ahead of the integrators on purpose. every bug from here on is visible instead of
hiding in a wall of floats.

- [x] **0.1.1** — raylib 6.0 via FetchContent, `orbitalis-viewer` opens a window. plus a
  `--version` path that skips `InitWindow` so CI can smoke-test it, and a configure-time
  assertion that `orbitalis-core` links no graphics library
- [x] **0.1.2** — render scale layer, in a new `orbitalis-render` library so the view maths
  is testable without a display. camera-relative conversion; the naive ordering has a hard
  ~9 km floor you cannot zoom past. `fit_scale` derives the world unit from the data
- [x] **0.1.3** — bodies as spheres. at true scale Earth is 0.0153 px, so radii go through
  a cube root (1471:1 becomes 11:1, strictly monotonic) with a 3-pixel floor. T toggles true
  scale; the HUD reports the exaggeration factor rather than hiding it
- [x] **0.1.4** — `OrbitCamera` + ray-sphere picking in `orbitalis-render`. following a body
  is not a camera feature: it moves `RenderFrame::focus`, which also puts the body where
  camera-relative precision is best. picking uses the *drawn* radius, so anything visible is
  clickable
- [ ] **0.1.5** — trails. ring buffer of past positions per body, drawn as a line strip
- [ ] **0.1.6** — sim loop decoupled from render loop: fixed dt accumulator,
  steps-per-frame control, pause / step-once / speed. HUD with fps, sim time, dt

**done when:** I can watch Sun–Earth, spin the camera and speed it up — and the sim gives
identical results at 30 fps and at 300 fps.

## 0.3.0 — integrators done properly

the actual meat of the numerical-methods part.

- [ ] **0.2.1** — `IIntegrator` interface, swappable at runtime, Euler becomes one impl
- [ ] **0.2.2** — **velocity Verlet**. symplectic, cheap, the workhorse for long runs
- [ ] **0.2.3** — diagnostics: kinetic + potential energy, linear and angular momentum,
  centre of mass. energy error `|E(t)−E(0)|/|E(0)|` live in the HUD
- [ ] **0.2.4** — classic **RK4**, fixed step. more accurate per step than Verlet but not
  symplectic, so energy drifts secularly
- [ ] **0.2.5** — **RKF45** adaptive: embedded 4th/5th order pair, local error estimate,
  step controller with safety factor, min/max clamps, rejection limit
- [ ] **0.2.6** — comparison harness: same scenario, every integrator, energy error vs
  time and wall-clock cost

worth remembering: adaptive stepping **breaks** the symplectic property. Verlet is for
long-term stability over millions of steps, RKF45 is for accuracy over short arcs and
close encounters. not competitors — different tools.

stretch: Yoshida 4th-order symplectic.

**done when:** I have a chart where Verlet's energy error is bounded and oscillating while
Euler and RK4 walk off in one direction.

## 0.4.0 — Barnes-Hut

O(n²) dies somewhere around a few thousand bodies.

- [ ] **0.3.1** — bounding cube over all bodies, recomputed each step
- [ ] **0.3.2** — octree build: 8 children, recursive insert, one body per leaf. needs a
  depth limit and a duplicate-position guard or coincident bodies recurse forever. flat
  node pool (`std::vector<Node>` + indices), not `unique_ptr` per node
- [ ] **0.3.3** — total mass + centre of mass per node, bottom-up
- [ ] **0.3.4** — force traversal with the θ opening criterion (`s/d < θ`, θ ≈ 0.5)
- [ ] **0.3.5** — **correctness gate**: Barnes-Hut vs brute force. as θ → 0 the relative
  error must go to ~0. if it doesn't, the tree is wrong and no benchmark matters
- [ ] **0.3.6** — benchmarks at n = 1k / 10k / 100k. confirm it bends toward n log n. also
  measure rebuild vs traversal cost
- [ ] **0.3.7** — parallelise the force loop. traversal is read-only over a shared tree so
  it parallelises almost for free. probably the AoS → SoA refactor too
- [ ] **0.3.8** — draw the octree as wireframe boxes in the viewer, toggleable

**done when:** 10k bodies run at an interactive framerate and forces match brute force
within tolerance.

## 0.5.0 — binary export

- [ ] **0.4.1** — design the format on paper first. header: magic `ORB3`, version, flags
  (endianness, f32/f64), body count, frame count, dt, t0, units. then body metadata. then
  fixed-size frames: `t` + N × (x,y,z)
- [ ] **0.4.2** — writer with buffered I/O and a **frame stride** — not every step goes to
  disk
- [ ] **0.4.3** — reader in core + `orb-inspect` to print a header and dump a frame
- [ ] **0.4.4** — python reader using `np.memmap`. fixed-size records means zero-copy,
  which is the whole reason the layout is what it is. 3D matplotlib plot to prove roundtrip
- [ ] **0.4.5** — optional f32 position mode, half the size
- [ ] **0.4.6** — export something ParaView eats directly (VTK/XDMF, CSV fallback)

**done when:** headless run → `.orb` file → python plot that matches what the viewer showed.

## 0.6.0 — orbital mechanics, scenarios, CLI

- [ ] **0.5.1** — Keplerian elements: state vector ↔ (a, e, i, Ω, ω, ν), both directions
- [ ] **0.5.2** — define bodies *by orbit* rather than raw state vectors, and read the
  elements back out live as a diagnostic
- [ ] **0.5.3** — JSON scenario files: bodies, units, integrator, dt, softening, θ,
  duration, export settings. deterministic seeds
- [ ] **0.5.4** — real CLI: `orbitalis run scenario.json --headless --out run.orb`
- [ ] **0.5.5** — built-in scenarios: real solar system (JPL Horizons), the **figure-8
  three-body** orbit, **Burrau's pythagorean 3-body**, Sun–Jupiter Trojans at L4/L5, a
  Plummer-sphere generator for big-n, and two clusters colliding
- [ ] **0.5.6** — checkpoint / restore

**done when:** I can hand someone a JSON file and they get my exact simulation.

## 0.7.0 — proving it's actually right

up to here I've been trusting my eyes. eyes are not a validation method.

- [ ] **0.6.1** — test suite in `ctest`, running in CI
- [ ] **0.6.2** — two-body vs the analytic Kepler solution, error bounded over many orbits
- [ ] **0.6.3** — Kepler's laws as assertions: equal areas in equal times, T² ∝ a³
- [ ] **0.6.4** — energy/momentum drift budget as a failing test, per integrator
- [ ] **0.6.5** — figure-8 stays a figure-8 for many periods. tiny errors destroy it
- [ ] **0.6.6** — perf regression tracking
- [ ] **0.6.7** — sanitizers, warnings-as-errors, clean static analysis

**done when:** `ctest` is green and each test maps to a physical fact I can name.

## 0.8.0 — the fun stuff

- [ ] **0.7.1** — collision detection + inelastic merging (conserve mass and momentum)
- [ ] **0.7.2** — **1PN post-newtonian correction**, reproduce Mercury's perihelion
  precession of 43 arcsec/century. hard, and allowed to fail
- [ ] **0.7.3** — individual / block timesteps
- [ ] **0.7.4** — viewer polish: velocity and force vectors, predicted paths, labels,
  scenario picker, screenshots
- [ ] **0.7.5** — barycentric / rotating reference frames (Lagrange points only make
  visual sense in a rotating frame)

everything here is cuttable except 0.7.1. picking which ones survive is itself the
milestone.

## 1.0.0 — done

- [ ] README with screenshots and a gif
- [ ] docs: the physics, the integrator comparison, the binary format spec, Barnes-Hut
- [ ] example scenarios that all run clean
- [ ] build instructions someone else can follow on a fresh machine
- [ ] tagged release with a prebuilt Windows binary
- [ ] honest "known limitations" section

**done when:** someone who isn't me can clone it, build it, and get a picture out.
