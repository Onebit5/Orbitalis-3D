#include <orbitalis/core/Version.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/render/BodyScale.hpp>
#include <orbitalis/render/OrbitCamera.hpp>
#include <orbitalis/render/Picking.hpp>
#include <orbitalis/render/RenderFrame.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <optional>
#include <string_view>
#include <vector>

// Milestone 0.1.4: the orbit camera.
//
// Drag to rotate, scroll to zoom, click a body to follow it, click empty space to go back
// to the system frame.
//
// Following a body is not a camera operation. It sets RenderFrame::focus to that body's
// position, which puts the body at the render origin, which is where camera-relative
// conversion (0.1.2) has the most precision. So the mode where you most want detail is
// automatically the mode where you get it.
//
// Nothing moves yet. The simulation loop is 0.1.6.

namespace {

using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::render::BodyScale;
using orbitalis::render::OrbitCamera;
using orbitalis::render::cycle_selection;
using orbitalis::render::pick_nearest;
using orbitalis::render::RenderFrame;
using orbitalis::render::Vec3f;
using orbitalis::render::world_radius_for_pixels;

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;

constexpr double kMinimumPixels = 3.0;

/// Degrees of rotation per pixel of mouse movement.
constexpr double kDragSensitivity = 0.35;

/// One scroll notch. Multiplicative, so it means the same thing at every zoom level.
constexpr double kZoomStep = 0.88;

/// A press-and-release that moved less than this counts as a click rather than a drag.
constexpr float kClickSlopPixels = 4.0f;

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

/// Everything the viewer needs to know about where each body is drawn this frame.
struct DrawnBody
{
    Vec3 centre;   // render space, as double so picking can reuse Vec3's maths
    double radius; // render units, including the minimum-pixel floor
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

void draw_hud(const System& system,
              const RenderFrame& frame,
              const BodyScale& body_scale,
              const OrbitCamera& camera,
              std::optional<orbitalis::BodyId> followed)
{
    // Backing panels. Zoom right up to a star and the whole viewport becomes bright yellow,
    // at which point pale text on it is genuinely unreadable: I misread the camera distance
    // off my own HUD while checking this step. A viewer whose readout is only legible
    // against a dark background is not much of a readout.
    const int panel_height = 214 + static_cast<int>(system.size()) * 22;
    DrawRectangle(0, 0, 560, panel_height, Color{10, 12, 20, 190});
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

    std::snprintf(line, sizeof(line), "camera   az %6.1f   el %+6.1f   dist %.4g units",
                  camera.azimuth_degrees(), camera.elevation_degrees(), camera.distance());
    DrawText(line, 24, 124, 18, kText);

    // The payoff from 0.1.2, made visible. Zooming in genuinely buys resolution, because
    // the conversion to float happens relative to the focus rather than the origin.
    std::snprintf(line, sizeof(line), "detail   %.4g m per float step at this range",
                  frame.resolution_at(camera.distance()));
    DrawText(line, 24, 148, 18, kDim);

    std::snprintf(line, sizeof(line), "sizing   %s",
                  body_scale.true_scale() ? "TRUE SCALE" : "compressed, cube root");
    DrawText(line, 24, 172, 18, body_scale.true_scale() ? kAccent : kDim);

    int y = 210;
    for (orbitalis::BodyId i = 0; i < system.size(); ++i) {
        const std::string_view name = system.name(i);
        std::snprintf(line, sizeof(line), "%s%-6.*s  drawn %.0fx too big",
                      followed == i ? "> " : "  ", static_cast<int>(name.size()), name.data(),
                      body_scale.exaggeration(system[i].radius, frame.metres_per_unit()));
        DrawText(line, 24, y, 16, body_colour(i));
        y += 22;
    }

    DrawText("drag rotate   scroll zoom   click or TAB to follow   T true scale   ESC close",
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

    const System system = orbitalis::scenarios::sun_earth();

    const Vec3 barycentre = system.center_of_mass();
    RenderFrame frame{barycentre, RenderFrame::fit_scale(system.bodies(), barycentre, 6.0)};

    BodyScale body_scale;

    OrbitCamera orbit{45.0, 25.0, 14.0};
    {
        OrbitCamera::Limits limits;
        // Deliberately tiny. Getting right down onto a body is the whole point of follow
        // mode, and camera-relative rendering is what makes it survivable: at 1e-3 units
        // from the focus a float step is about 3 mm.
        limits.min_distance = 1.0e-3;
        limits.max_distance = 200.0;
        orbit.set_limits(limits);
    }

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
        // ---- input ------------------------------------------------------------------

        if (IsKeyPressed(KEY_T)) {
            body_scale.set_true_scale(!body_scale.true_scale());
        }

        // Cycle through bodies and back to the barycentre. Clicking a three-pixel target is
        // fiddly, and every space sim worth using offers a keyboard equivalent. The state
        // machine itself lives in orbitalis-render so it can be tested; see the note on
        // cycle_selection for why.
        if (IsKeyPressed(KEY_TAB)) {
            followed = orbitalis::render::cycle_selection(followed, system.size());
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

        // ---- follow the camera ------------------------------------------------------
        //
        // Focus first, then camera, because the camera orbits whatever the focus put at the
        // origin. Doing it the other way round would lag a frame behind.

        frame.set_focus(followed ? system[*followed].position : barycentre);

        // Keep the camera outside whatever it is following. Without this you can scroll
        // straight through the surface and end up inside the sphere looking at its
        // backfaces, which is disorienting and looks like a rendering bug.
        //
        // Based on the compressed radius rather than the drawn one on purpose: the drawn
        // radius includes the minimum-pixel floor, which itself depends on camera distance,
        // and feeding that back into the distance limit would be circular.
        {
            OrbitCamera::Limits limits = orbit.limits();
            limits.min_distance =
                followed ? std::max(1.0e-3,
                                    body_scale.render_radius(system[*followed].radius,
                                                             frame.metres_per_unit())
                                        * 1.5)
                         : 1.0e-3;
            orbit.set_limits(limits);
        }

        camera.position = to_raylib(orbit.position());

        const auto drawn = lay_out(system, frame, body_scale, camera, GetScreenHeight());

        // A press-and-release that barely moved is a click, not a sloppy drag.
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

            // Picking against the *drawn* radius, so anything visible is clickable. Earth's
            // true radius here is 2.6e-4 units, which would be an impossible target.
            const auto hit = pick_nearest(to_double(ray.position), to_double(ray.direction),
                                          centres, radii);

            followed = hit;  // a miss clears it, which is how you get back to the barycentre
        }

        // ---- draw -------------------------------------------------------------------

        BeginDrawing();
        ClearBackground(kBackground);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);

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

        draw_hud(system, frame, body_scale, orbit, followed);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
