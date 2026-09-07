#include <orbitalis/core/Version.hpp>
#include <orbitalis/integrators/Euler.hpp>
#include <orbitalis/physics/BruteForceSolver.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/render/BodyScale.hpp>
#include <orbitalis/render/OrbitCamera.hpp>
#include <orbitalis/render/Picking.hpp>
#include <orbitalis/render/RenderFrame.hpp>
#include <orbitalis/render/Trail.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string_view>
#include <vector>

// Milestone 0.1.5: trails.
//
// Trails are stored in simulation coordinates and converted at draw time, so switching the
// followed body reprojects the whole history instead of smearing it. See Trail.hpp.
//
// The stepping below is PROVISIONAL. It advances a fixed number of steps per frame, which
// ties the simulation rate to the framerate: the same run gives different answers on a
// 30 Hz machine and a 144 Hz one. 0.1.6 replaces it with a fixed-timestep accumulator, and
// frame delta-time must never reach the integrator. It is here only because a trail with
// nothing moving is not much of a trail.

namespace {

using orbitalis::BruteForceSolver;
using orbitalis::SemiImplicitEuler;
using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::render::BodyScale;
using orbitalis::render::cycle_selection;
using orbitalis::render::OrbitCamera;
using orbitalis::render::pick_nearest;
using orbitalis::render::RenderFrame;
using orbitalis::render::TrailSet;
using orbitalis::render::Vec3f;
using orbitalis::render::world_radius_for_pixels;

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;

constexpr double kMinimumPixels = 3.0;
constexpr double kDragSensitivity = 0.35;
constexpr double kZoomStep = 0.88;
constexpr float kClickSlopPixels = 4.0f;

/// About one orbit's worth at the sampling distance below.
constexpr std::size_t kTrailCapacity = 400;

/// Record a point every this many render units of travel. Distance-based rather than
/// per-frame, so the spacing is uniform whether the body is at perihelion or aphelion.
///
/// Has to sit comfortably ABOVE the distance a body covers in one frame, or the two rates
/// beat against each other: at 0.04 the Earth was moving 0.037 per frame, so roughly every
/// other frame failed the test and the spacing alternated instead of being uniform. It was
/// invisible on screen at 2-4 pixels, and it still made the trail length depend on the
/// framerate, which is the exact coupling 0.1.6 exists to remove.
constexpr double kTrailSpacingUnits = 0.1;

constexpr Color kBackground{10, 12, 20, 255};
constexpr Color kText{200, 210, 230, 255};
constexpr Color kDim{90, 100, 120, 255};
constexpr Color kAccent{255, 200, 130, 255};

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

/// Draws one trail as a line strip, fading toward the oldest end.
///
/// Every point is converted here rather than at record time, which is the whole reason
/// trails hold metres: the same history draws correctly in any reference frame.
void draw_trail(const orbitalis::render::Trail& trail, const RenderFrame& frame, Color colour)
{
    if (trail.size() < 2) {
        return;
    }

    Vector3 previous = to_raylib(frame.to_render(trail[0]));

    for (std::size_t i = 1; i < trail.size(); ++i) {
        const Vector3 current = to_raylib(frame.to_render(trail[i]));

        // Older segments are dimmer, so the direction of travel is readable without an
        // arrow. Squared so the fade is concentrated at the tail rather than washing out
        // the whole line.
        const double age = static_cast<double>(i) / static_cast<double>(trail.size() - 1);
        const auto alpha = static_cast<unsigned char>(200.0 * age * age + 10.0);

        DrawLine3D(previous, current, Color{colour.r, colour.g, colour.b, alpha});
        previous = current;
    }
}

void draw_hud(const System& system,
              const RenderFrame& frame,
              const BodyScale& body_scale,
              const OrbitCamera& camera,
              const TrailSet& trails,
              double elapsed_seconds,
              std::optional<orbitalis::BodyId> followed)
{
    const int panel_height = 238 + static_cast<int>(system.size()) * 22;
    DrawRectangle(0, 0, 580, panel_height, Color{10, 12, 20, 190});
    DrawRectangle(0, GetScreenHeight() - 48, GetScreenWidth(), 48, Color{10, 12, 20, 190});

    DrawText(orbitalis::version_banner(), 24, 24, 28, kText);
    DrawText(orbitalis::milestone_name(), 24, 60, 18, kDim);

    char line[192];

    if (followed) {
        const std::string_view name = system.name(*followed);
        std::snprintf(line, sizeof(line), "frame    following %.*s",
                      static_cast<int>(name.size()), name.data());
        DrawText(line, 24, 100, 18, kAccent);
    } else {
        DrawText("frame    system barycentre", 24, 100, 18, kText);
    }

    std::snprintf(line, sizeof(line), "elapsed  %.2f days   (%.3f orbits)",
                  elapsed_seconds / orbitalis::kDay, elapsed_seconds / orbitalis::kSiderealYear);
    DrawText(line, 24, 124, 18, kText);

    std::snprintf(line, sizeof(line), "camera   az %6.1f   el %+6.1f   dist %.4g units",
                  camera.azimuth_degrees(), camera.elevation_degrees(), camera.distance());
    DrawText(line, 24, 148, 18, kText);

    std::snprintf(line, sizeof(line), "detail   %.4g m per float step at this range",
                  frame.resolution_at(camera.distance()));
    DrawText(line, 24, 172, 18, kDim);

    std::snprintf(line, sizeof(line), "trails   %zu / %zu points   %s",
                  trails.size() > 0 ? trails[0].size() : 0, trails.capacity_per_body(),
                  body_scale.true_scale() ? "TRUE SCALE" : "compressed");
    DrawText(line, 24, 196, 18, kDim);

    int y = 234;
    for (orbitalis::BodyId i = 0; i < system.size(); ++i) {
        const std::string_view name = system.name(i);
        std::snprintf(line, sizeof(line), "%s%-6.*s  %zu trail points",
                      followed == i ? "> " : "  ", static_cast<int>(name.size()), name.data(),
                      i < trails.size() ? trails[i].size() : 0);
        DrawText(line, 24, y, 16, body_colour(i));
        y += 22;
    }

    DrawText("drag rotate   scroll zoom   click/TAB follow   T true scale   C clear trails   ESC close",
             24, GetScreenHeight() - 34, 16, kDim);
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

    // The barycentre of an isolated system does not move, so this stays valid as the bodies
    // orbit. remove_net_drift() at 0.0.6 is what guarantees that.
    const Vec3 barycentre = system.center_of_mass();
    RenderFrame frame{barycentre, RenderFrame::fit_scale(system.bodies(), barycentre, 6.0)};

    BodyScale body_scale;
    TrailSet trails{system.size(), kTrailCapacity};

    const BruteForceSolver solver;
    SemiImplicitEuler integrator{solver};

    const double period =
        orbitalis::scenarios::circular_period(orbitalis::kSunGM + orbitalis::kEarthGM,
                                              orbitalis::kAstronomicalUnit);

    // PROVISIONAL, see the note at the top of this file. Roughly one orbit every 17 seconds
    // at 60 fps, and wrong on any other framerate. 0.1.6 fixes that properly.
    const double dt = period / 2000.0;
    constexpr int kStepsPerFrame = 2;

    double elapsed = 0.0;

    // Far enough back that the whole fitted system fits vertically. fit_scale targets 6
    // units of radius, so the orbit is 12 across, and at a 45 degree vertical FOV that needs
    // 6/tan(22.5) = 14.5 units. Starting at 14 cropped the top of Earth's orbit.
    OrbitCamera orbit{45.0, 25.0, 18.0};
    std::optional<orbitalis::BodyId> followed;

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

        for (int i = 0; i < kStepsPerFrame; ++i) {
            integrator.step(system, dt);
            elapsed += dt;
        }

        trails.resize(system.size());
        trails.sample(system.bodies(), kTrailSpacingUnits * frame.metres_per_unit());

        // ---- input ------------------------------------------------------------------

        if (IsKeyPressed(KEY_T)) {
            body_scale.set_true_scale(!body_scale.true_scale());
        }
        if (IsKeyPressed(KEY_C)) {
            trails.clear();
        }
        if (IsKeyPressed(KEY_TAB)) {
            followed = cycle_selection(followed, system.size());
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

        frame.set_focus(followed ? system[*followed].position : barycentre);

        {
            OrbitCamera::Limits limits = orbit.limits();
            limits.min_distance =
                followed ? std::max(1.0e-3,
                                    body_scale.render_radius(system[*followed].radius,
                                                             frame.metres_per_unit())
                                        * 1.5)
                         : 1.0e-3;
            limits.max_distance = 200.0;
            orbit.set_limits(limits);
        }

        camera.position = to_raylib(orbit.position());

        const auto drawn = lay_out(system, frame, body_scale, camera, GetScreenHeight());

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

            followed = pick_nearest(to_double(ray.position), to_double(ray.direction), centres,
                                    radii);
        }

        // ---- draw -------------------------------------------------------------------

        BeginDrawing();
        ClearBackground(kBackground);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);

        for (std::size_t i = 0; i < trails.size(); ++i) {
            draw_trail(trails[i], frame, body_colour(i));
        }

        for (orbitalis::BodyId i = 0; i < drawn.size(); ++i) {
            const Vector3 centre{static_cast<float>(drawn[i].centre.x),
                                 static_cast<float>(drawn[i].centre.y),
                                 static_cast<float>(drawn[i].centre.z)};

            DrawSphereEx(centre, static_cast<float>(drawn[i].radius), 16, 16, body_colour(i));

            if (followed == i) {
                DrawSphereWires(centre, static_cast<float>(drawn[i].radius * 1.6), 10, 10,
                                Color{255, 200, 130, 90});
            }
        }
        EndMode3D();

        draw_hud(system, frame, body_scale, orbit, trails, elapsed, followed);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
