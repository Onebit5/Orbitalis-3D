#include <orbitalis/core/Version.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/render/RenderFrame.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <raylib.h>

#include <cstdio>
#include <cstring>
#include <string_view>

// Milestone 0.1.2: the render scale layer.
//
// Simulation coordinates finally reach the screen. Bodies are drawn as fixed-size dots
// here; sizing them properly (exaggerated radii, minimum pixel size) is 0.1.3, the orbit
// camera is 0.1.4, and nothing moves until the sim loop lands at 0.1.6.

namespace {

using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::render::RenderFrame;

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;

constexpr Color kBackground{10, 12, 20, 255};
constexpr Color kText{200, 210, 230, 255};
constexpr Color kDim{90, 100, 120, 255};

/// The one place a render position becomes a raylib position.
///
/// raylib's Vector3 happens to be three floats in the same order, but converting
/// explicitly rather than reinterpreting keeps orbitalis-render free of any knowledge
/// that raylib exists.
Vector3 to_raylib(const orbitalis::render::Vec3f& v) noexcept
{
    return Vector3{v.x, v.y, v.z};
}

void draw_hud(const System& system, const RenderFrame& frame)
{
    DrawText(orbitalis::version_banner(), 24, 24, 28, kText);
    DrawText(orbitalis::milestone_name(), 24, 60, 18, kDim);

    char line[160];

    std::snprintf(line, sizeof(line), "scale     %.4e m per render unit", frame.metres_per_unit());
    DrawText(line, 24, 100, 18, kText);

    std::snprintf(line, sizeof(line), "focus     %+.4e  %+.4e  %+.4e m",
                  frame.focus().x, frame.focus().y, frame.focus().z);
    DrawText(line, 24, 124, 18, kText);

    // The honest statement of how good the picture is at this zoom. It is proportional to
    // distance from the camera, which is the trade camera-relative rendering makes and the
    // right way round: distant things are small on screen.
    std::snprintf(line, sizeof(line), "resolution %.1f m at 1 unit, %.1f m at 10 units",
                  frame.resolution_at(1.0), frame.resolution_at(10.0));
    DrawText(line, 24, 148, 18, kDim);

    int y = 190;
    for (orbitalis::BodyId i = 0; i < system.size(); ++i) {
        const std::string_view name = system.name(i);
        std::snprintf(line, sizeof(line), "%-6.*s  r = %.6e m",
                      static_cast<int>(name.size()), name.data(),
                      (system[i].position - frame.focus()).length());
        DrawText(line, 24, y, 16, kDim);
        y += 22;
    }

    DrawText("nothing is moving yet: the sim loop lands at 0.1.6",
             24, GetScreenHeight() - 58, 16, kDim);
    DrawText("ESC to close", 24, GetScreenHeight() - 34, 16, kDim);

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

    // Focus on the barycentre and let the scenario choose its own scale. Deliberately not
    // hardcoding "one AU per unit": that is right for a planetary system and useless for a
    // cluster spanning parsecs, so the scale is derived from the data instead.
    const Vec3 focus = system.center_of_mass();
    const RenderFrame frame{focus, RenderFrame::fit_scale(system.bodies(), focus, 6.0)};

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(kDefaultWidth, kDefaultHeight, "Orbitalis-3D");

    if (!IsWindowReady()) {
        std::fprintf(stderr, "failed to open a window\n");
        return 1;
    }

    SetTargetFPS(60);

    // A fixed vantage point. Interactive orbiting is 0.1.4.
    Camera3D camera{};
    camera.position = Vector3{9.0f, 7.0f, 9.0f};
    camera.target = Vector3{0.0f, 0.0f, 0.0f};
    camera.up = Vector3{0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(kBackground);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);

        for (const orbitalis::Body& b : system.bodies()) {
            // Every body goes through the same transform, and this is the only conversion
            // from double to float anywhere in the project outside of it.
            DrawSphere(to_raylib(frame.to_render(b.position)), 0.08f, kText);
        }
        EndMode3D();

        draw_hud(system, frame);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
