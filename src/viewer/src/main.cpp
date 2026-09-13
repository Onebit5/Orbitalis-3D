#include <orbitalis/core/Version.hpp>
#include <orbitalis/integrators/Integrator.hpp>
#include <orbitalis/physics/Diagnostics.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/render/BodyScale.hpp>
#include <orbitalis/render/OrbitCamera.hpp>
#include <orbitalis/render/Picking.hpp>
#include <orbitalis/render/PotentialSurface.hpp>
#include <orbitalis/render/RenderFrame.hpp>
#include <orbitalis/render/SimClock.hpp>
#include <orbitalis/render/Trail.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <optional>
#include <string_view>
#include <vector>

// Milestone 0.2.2: velocity Verlet, and it is now the default.
//
// The viewer holds an IIntegrator through a pointer and never learns which concrete method
// it has. I cycles through them at runtime, which makes the 0.0.5 result something you can
// watch rather than read: switch to forward Euler and the orbit visibly spirals outward
// while the symplectic methods keep it closed, at identical cost.
//
// Verlet is second order where both Eulers are first, at the same one force evaluation per
// step, so it is what a long run should be using. Cycling now walks forward-euler ->
// semi-implicit-euler -> velocity-verlet, and the HUD reports the order alongside the name.
//
// Frame delta-time still never reaches the integrator. SimClock turns real elapsed time
// into a whole number of identical fixed steps, so the same scenario gives the same answer
// at 30 fps and at 300.

namespace {

using orbitalis::BruteForceSolver;
using orbitalis::IIntegrator;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::render::BodyScale;
using orbitalis::render::cycle_selection;
using orbitalis::render::OrbitCamera;
using orbitalis::render::pick_nearest;
using orbitalis::render::PotentialSurface;
using orbitalis::render::RenderFrame;
using orbitalis::render::SimClock;
using orbitalis::render::TrailSet;
using orbitalis::render::Vec3f;
using orbitalis::render::world_radius_for_pixels;

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;

constexpr double kMinimumPixels = 3.0;
constexpr double kDragSensitivity = 0.35;
constexpr double kZoomStep = 0.88;
constexpr float kClickSlopPixels = 4.0f;

constexpr std::size_t kTrailCapacity = 900;
constexpr double kTrailSpacingUnits = 0.045;

constexpr Color kBackground{10, 12, 20, 255};
constexpr Color kText{200, 210, 230, 255};
constexpr Color kDim{90, 100, 120, 255};
constexpr Color kAccent{255, 200, 130, 255};
constexpr Color kWell{58, 74, 112, 255};

Color body_colour(orbitalis::BodyId id)
{
    static constexpr Color palette[] = {
        Color{255, 214, 120, 255}, Color{120, 175, 255, 255}, Color{200, 160, 130, 255},
        Color{160, 220, 190, 255}, Color{215, 150, 200, 255},
    };
    return palette[id % (sizeof(palette) / sizeof(palette[0]))];
}

Vector3 to_raylib(const Vec3f& v) noexcept
{
    return Vector3{v.x, v.y, v.z};
}

Vec3 to_double(const Vector3& v) noexcept
{
    return Vec3{static_cast<double>(v.x), static_cast<double>(v.y), static_cast<double>(v.z)};
}

float distance_between(const Vector3& a, const Vector3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

struct DrawnBody
{
    Vec3 centre;
    double radius;
};

std::vector<DrawnBody> lay_out(const System& system,
                               const RenderFrame& frame,
                               const BodyScale& body_scale,
                               const Camera3D& camera,
                               int viewport_height)
{
    std::vector<DrawnBody> drawn;
    drawn.reserve(system.size());

    for (const orbitalis::Body& body : system.bodies()) {
        const Vec3f position = frame.to_render(body.position);

        const double sized = body_scale.render_radius(body.radius, frame.metres_per_unit());
        const double distance = distance_between(camera.position, to_raylib(position));
        const double floor = world_radius_for_pixels(kMinimumPixels, distance, camera.fovy,
                                                     viewport_height);

        drawn.push_back(DrawnBody{to_double(to_raylib(position)), std::max(sized, floor)});
    }

    return drawn;
}

/// Draws the potential surface as a wireframe, deeper parts brighter.
void draw_well(const PotentialSurface& surface)
{
    const int n = surface.resolution();
    if (n < 2) {
        return;
    }

    auto shade = [&](const Vec3f& a, const Vec3f& b) {
        // Depth-proportional brightness, so the funnel reads even from directly above where
        // the geometry alone would be nearly invisible.
        const double depth = -0.5 * (static_cast<double>(a.y) + b.y) / surface.config().depth_units;
        const auto alpha = static_cast<unsigned char>(std::clamp(24.0 + 210.0 * depth, 0.0, 255.0));
        DrawLine3D(to_raylib(a), to_raylib(b), Color{kWell.r, kWell.g, kWell.b, alpha});
    };

    for (int j = 0; j < n; ++j) {
        for (int i = 0; i + 1 < n; ++i) {
            shade(surface.vertex(i, j), surface.vertex(i + 1, j));
        }
    }
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j + 1 < n; ++j) {
            shade(surface.vertex(i, j), surface.vertex(i, j + 1));
        }
    }
}

void draw_trail(const orbitalis::render::Trail& trail, const RenderFrame& frame, Color colour)
{
    if (trail.size() < 2) {
        return;
    }

    Vector3 previous = to_raylib(frame.to_render(trail[0]));

    for (std::size_t i = 1; i < trail.size(); ++i) {
        const Vector3 current = to_raylib(frame.to_render(trail[i]));

        const double age = static_cast<double>(i) / static_cast<double>(trail.size() - 1);
        const auto alpha = static_cast<unsigned char>(200.0 * age * age + 10.0);

        DrawLine3D(previous, current, Color{colour.r, colour.g, colour.b, alpha});
        previous = current;
    }
}

struct HudState
{
    const System* system;
    const RenderFrame* frame;
    const BodyScale* body_scale;
    const OrbitCamera* camera;
    const SimClock* clock;
    const IIntegrator* integrator;
    const orbitalis::ConservationMonitor* conservation;
    const TrailSet* trails;
    double elapsed;
    std::optional<orbitalis::BodyId> selected;
    bool centre_on_selection;
    bool show_well;
};

void draw_hud(const HudState& hud)
{
    const System& system = *hud.system;

    const int panel_height = 358 + static_cast<int>(system.size()) * 22;
    DrawRectangle(0, 0, 600, panel_height, Color{10, 12, 20, 195});
    DrawRectangle(0, GetScreenHeight() - 48, GetScreenWidth(), 48, Color{10, 12, 20, 195});

    DrawText(orbitalis::version_banner(), 24, 24, 28, kText);
    DrawText(orbitalis::milestone_name(), 24, 60, 18, kDim);

    char line[200];

    std::snprintf(line, sizeof(line), "elapsed  %.2f days   (%.3f orbits)",
                  hud.elapsed / orbitalis::kDay, hud.elapsed / orbitalis::kSiderealYear);
    DrawText(line, 24, 100, 18, kText);

    std::snprintf(line, sizeof(line), "clock    dt %.0f s   x%.4g real-time   %s%s",
                  hud.clock->timestep(), hud.clock->time_scale(),
                  hud.clock->paused() ? "PAUSED" : "running",
                  hud.clock->fell_behind() ? "   (behind)" : "");
    DrawText(line, 24, 124, 18, hud.clock->paused() ? kAccent : kText);

    std::snprintf(line, sizeof(line), "method   %s   order %d   %s",
                  hud.integrator->name(), hud.integrator->order(),
                  hud.integrator->is_symplectic() ? "symplectic" : "NOT symplectic");
    DrawText(line, 24, 148, 18, hud.integrator->is_symplectic() ? kText : kAccent);

    std::snprintf(line, sizeof(line), "camera   az %6.1f   el %+6.1f   dist %.4g units",
                  hud.camera->azimuth_degrees(), hud.camera->elevation_degrees(),
                  hud.camera->distance());
    DrawText(line, 24, 172, 18, kText);

    std::snprintf(line, sizeof(line), "detail   %.4g m per float step at this range",
                  hud.frame->resolution_at(hud.camera->distance()));
    DrawText(line, 24, 196, 18, kDim);

    std::snprintf(line, sizeof(line), "view     %s   %s   %s",
                  hud.centre_on_selection ? "centred on selection" : "star-centred",
                  hud.body_scale->true_scale() ? "TRUE SCALE" : "compressed",
                  hud.show_well ? "gravity well on" : "gravity well off");
    DrawText(line, 24, 220, 18, kDim);

    std::snprintf(line, sizeof(line), "trails   %zu / %zu points",
                  hud.trails->size() > 1 ? (*hud.trails)[1].size() : 0,
                  hud.trails->capacity_per_body());
    DrawText(line, 24, 244, 18, kDim);

    // The point of the whole milestone. Energy drift is signed, because the direction is
    // the informative part: positive means the method is pumping energy in, which for a
    // bound orbit means climbing outward. Absolute value would throw that away.
    const orbitalis::ConservationMonitor& conservation = *hud.conservation;
    const double drift = conservation.relative_energy_error();

    std::snprintf(line, sizeof(line), "energy   E %+.6e J   T %+.3e   U %+.3e",
                  conservation.latest().total, conservation.latest().kinetic,
                  conservation.latest().potential);
    DrawText(line, 24, 276, 18, kText);

    // Orange once the drift is somewhere a symplectic method at this timestep never goes.
    // Verlet sits at 2e-8 here and semi-implicit at 3e-4, so 1e-3 flags forward Euler
    // within a fraction of an orbit and never cries wolf about the other two.
    std::snprintf(line, sizeof(line), "drift    %+.4e   worst %.4e   (sampled per frame)",
                  drift, conservation.worst_relative_energy_error());
    DrawText(line, 24, 300, 18, std::abs(drift) > 1e-3 ? kAccent : kText);

    // Both scaled against the largest single body, since the totals are zero by
    // construction and an absolute figure at these masses means nothing on its own.
    //
    // Drawn as two pieces with independent colours, because these two fail separately and a
    // single colour would hide that. Linear momentum stays at roundoff under every method
    // here, since Newton's third law makes it the solver's property. Angular momentum needs
    // the integrator's update ordering to cooperate as well, and forward Euler's does not:
    // it has lost 1% of L before it has finished one orbit, while its p is still at 1e-15.
    // Colouring them together made a perfectly healthy p look guilty by association.
    const double p_drift = conservation.relative_momentum_drift();
    const double l_drift = conservation.relative_angular_momentum_drift();

    std::snprintf(line, sizeof(line), "conserved  p %.2e", p_drift);
    DrawText(line, 24, 324, 18, p_drift > 1e-9 ? kAccent : kDim);
    const int p_width = MeasureText(line, 18);

    std::snprintf(line, sizeof(line), "   L %.2e   relative", l_drift);
    DrawText(line, 24 + p_width, 324, 18, l_drift > 1e-9 ? kAccent : kDim);

    int y = 354;
    for (orbitalis::BodyId i = 0; i < system.size(); ++i) {
        const std::string_view name = system.name(i);
        std::snprintf(line, sizeof(line), "%s%-6.*s  %.4e m from barycentre",
                      hud.selected == i ? "> " : "  ", static_cast<int>(name.size()), name.data(),
                      system[i].position.length());
        DrawText(line, 24, y, 16, body_colour(i));
        y += 22;
    }

    DrawText("SPACE pause   . step   +/- speed   I integrator   click/TAB select   "
             "F centre   G well   T scale   C clear   ESC",
             24, GetScreenHeight() - 34, 15, kDim);
    DrawFPS(GetScreenWidth() - 96, 24);
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc > 1 && std::strcmp(argv[1], "--version") == 0) {
        std::printf("%s\n", orbitalis::version_banner());
        std::printf("  milestone : %s\n", orbitalis::milestone_name());
        std::printf("  target    : orbitalis-viewer\n");
        std::printf("  raylib    : %s\n", RAYLIB_VERSION);
        return 0;
    }

    System system = orbitalis::scenarios::sun_earth();

    const Vec3 barycentre = system.center_of_mass();
    RenderFrame frame{barycentre, RenderFrame::fit_scale(system.bodies(), barycentre, 6.0)};

    BodyScale body_scale;
    TrailSet trails{system.size(), kTrailCapacity};
    PotentialSurface well;

    const BruteForceSolver solver;

    // Held through the interface, so switching method at runtime is a pointer swap and the
    // rest of the loop never learns which one it has.
    //
    // Found by name rather than by index. The index was hardcoded until 0.2.2, at which
    // point inserting velocity Verlet into the registry silently changed which method the
    // viewer started with, and nothing would have told me.
    //
    // The name itself comes from core, so the choice of default sits next to the methods
    // and is covered by a test, rather than being a string in a viewer that no test links.
    const auto names = orbitalis::integrator_names();
    std::size_t integrator_index = static_cast<std::size_t>(
        std::find(names.begin(), names.end(), orbitalis::kDefaultIntegratorName) -
        names.begin());
    if (integrator_index >= names.size()) {
        integrator_index = 0;  // unreachable: a core test pins that the default resolves
    }
    auto integrator = orbitalis::make_integrator(names[integrator_index], solver);

    const double period =
        orbitalis::scenarios::circular_period(orbitalis::kSunGM + orbitalis::kEarthGM,
                                              orbitalis::kAstronomicalUnit);

    SimClock::Config clock_config;
    clock_config.timestep = period / 2000.0;
    clock_config.time_scale = period / 18.0;  // one orbit per eighteen real seconds
    SimClock clock{clock_config};

    // Baselined here, before a single step, so the reference energy is the scenario's own
    // and not whatever the first frame happened to produce.
    orbitalis::ConservationMonitor conservation;
    conservation.reset(system, solver);

    double elapsed = 0.0;

    OrbitCamera orbit{45.0, 28.0, 18.0};
    std::optional<orbitalis::BodyId> selected;

    // Selection highlights by default and does not move the view: the star stays at the
    // centre, which is what a planetary system looks like. F opts into re-centring, which
    // is worth having because putting a body at the render origin is exactly where
    // camera-relative precision is best (0.1.2).
    bool centre_on_selection = false;
    bool show_well = true;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(kDefaultWidth, kDefaultHeight, "Orbitalis-3D");

    if (!IsWindowReady()) {
        std::fprintf(stderr, "failed to open a window\n");
        return 1;
    }

    SetTargetFPS(60);

    Camera3D camera{};
    camera.up = Vector3{0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;
    camera.target = Vector3{0.0f, 0.0f, 0.0f};

    Vector2 press_position{};
    float drag_travel = 0.0f;

    while (!WindowShouldClose()) {
        // ---- simulation -------------------------------------------------------------
        //
        // GetFrameTime() is the only place real time enters, and it stops here. The
        // integrator only ever sees clock.timestep().

        const int steps = clock.advance(static_cast<double>(GetFrameTime()));

        trails.resize(system.size());
        const bool stepped = steps > 0;

        for (int i = 0; i < steps; ++i) {
            integrator->step(system, clock.timestep());
            elapsed += clock.timestep();

            // Sampled inside the step loop rather than once per frame, so the trail's
            // resolution follows fixed timesteps too. At 0.1.5 it was per frame, which made
            // trail length quietly depend on the framerate.
            trails.sample(system.bodies(), kTrailSpacingUnits * frame.metres_per_unit());
        }

        // ---- input ------------------------------------------------------------------

        if (IsKeyPressed(KEY_SPACE)) {
            clock.set_paused(!clock.paused());
        }
        if (IsKeyPressed(KEY_PERIOD)) {
            clock.request_single_step();
        }
        if (IsKeyPressed(KEY_EQUAL) || IsKeyPressed(KEY_KP_ADD)) {
            clock.scale_speed(2.0);
        }
        if (IsKeyPressed(KEY_MINUS) || IsKeyPressed(KEY_KP_SUBTRACT)) {
            clock.scale_speed(0.5);
        }
        if (IsKeyPressed(KEY_T)) {
            body_scale.set_true_scale(!body_scale.true_scale());
        }
        if (IsKeyPressed(KEY_C)) {
            trails.clear();
        }
        if (IsKeyPressed(KEY_G)) {
            show_well = !show_well;
        }
        if (IsKeyPressed(KEY_F)) {
            centre_on_selection = !centre_on_selection;
        }
        if (IsKeyPressed(KEY_TAB)) {
            selected = cycle_selection(selected, system.size());
        }

        // Swap integrator. The trail is cleared because it is a record of a trajectory this
        // method did not produce, and leaving it would draw a path that no single
        // integrator ever took.
        if (IsKeyPressed(KEY_I)) {
            integrator_index = (integrator_index + 1) % names.size();
            integrator = orbitalis::make_integrator(names[integrator_index], solver);
            trails.clear();

            // Re-baseline for the same reason the trail is cleared. The drift on screen has
            // to be the drift *this* method caused; carrying the old baseline forward would
            // charge a fresh integrator with the previous one's accumulated error, which is
            // exactly the comparison the milestone exists to make honest.
            conservation.reset(system, solver);
        }

        if (const float wheel = GetMouseWheelMove(); wheel != 0.0f) {
            orbit.zoom(std::pow(kZoomStep, static_cast<double>(wheel)));
        }

        if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
            press_position = GetMousePosition();
            drag_travel = 0.0f;
        }
        if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
            const Vector2 delta = GetMouseDelta();
            drag_travel += std::abs(delta.x) + std::abs(delta.y);
            orbit.rotate(-delta.x * kDragSensitivity, delta.y * kDragSensitivity);
        }

        // ---- frame and camera -------------------------------------------------------

        const bool following = centre_on_selection && selected.has_value();
        frame.set_focus(following ? system[*selected].position : barycentre);

        {
            OrbitCamera::Limits limits = orbit.limits();
            limits.min_distance =
                following ? std::max(1.0e-3,
                                     body_scale.render_radius(system[*selected].radius,
                                                              frame.metres_per_unit())
                                         * 1.5)
                          : 1.0e-3;
            limits.max_distance = 200.0;
            orbit.set_limits(limits);
        }

        camera.position = to_raylib(orbit.position());

        const auto drawn = lay_out(system, frame, body_scale, camera, GetScreenHeight());

        if (show_well) {
            well.update(system.bodies(), frame);
        }

        if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT) && drag_travel <= kClickSlopPixels) {
            const Ray ray = GetScreenToWorldRay(press_position, camera);

            std::vector<Vec3> centres;
            std::vector<double> radii;
            centres.reserve(drawn.size());
            radii.reserve(drawn.size());
            for (const DrawnBody& b : drawn) {
                centres.push_back(b.centre);
                radii.push_back(b.radius);
            }

            selected = pick_nearest(to_double(ray.position), to_double(ray.direction), centres,
                                    radii);
        }

        // ---- draw -------------------------------------------------------------------

        BeginDrawing();
        ClearBackground(kBackground);

        BeginMode3D(camera);

        if (show_well) {
            draw_well(well);
        }

        for (std::size_t i = 0; i < trails.size(); ++i) {
            draw_trail(trails[i], frame, body_colour(i));
        }

        for (orbitalis::BodyId i = 0; i < drawn.size(); ++i) {
            const Vector3 centre{static_cast<float>(drawn[i].centre.x),
                                 static_cast<float>(drawn[i].centre.y),
                                 static_cast<float>(drawn[i].centre.z)};

            DrawSphereEx(centre, static_cast<float>(drawn[i].radius), 16, 16, body_colour(i));

            if (selected == i) {
                DrawSphereWires(centre, static_cast<float>(drawn[i].radius * 1.7), 10, 10,
                                Color{255, 200, 130, 110});
            }
        }
        EndMode3D();

        // Once per frame, not once per step: the potential is O(n²), so sampling every
        // step would roughly double the cost of the simulation in order to watch it. The
        // consequence is that "worst" is the worst of what was looked at, which the HUD
        // says out loud rather than pretending otherwise.
        if (stepped) {
            conservation.sample(system, solver);
        }

        HudState hud{&system, &frame,   &body_scale,         &orbit,    &clock,
                     integrator.get(),    &conservation,       &trails,   elapsed,
                     selected,            centre_on_selection, show_well};
        draw_hud(hud);

        EndDrawing();
    }

    CloseWindow();
    return 0;
}
