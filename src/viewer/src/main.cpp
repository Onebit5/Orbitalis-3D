#include <orbitalis/core/Version.hpp>
#include <orbitalis/physics/Constants.hpp>
#include <orbitalis/physics/System.hpp>
#include <orbitalis/render/BodyScale.hpp>
#include <orbitalis/render/RenderFrame.hpp>
#include <orbitalis/scenarios/Builtin.hpp>

#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string_view>

// Milestone 0.1.3: bodies as spheres.
//
// At true scale Earth is 0.0153 pixels across in this view, so sizes are compressed by a
// cube root and floored at three pixels. Press T to see how much of a lie that is.
//
// The camera is still fixed (0.1.4) and nothing moves yet (0.1.6).

namespace {

using orbitalis::System;
using orbitalis::Vec3;
using orbitalis::render::BodyScale;
using orbitalis::render::RenderFrame;
using orbitalis::render::world_radius_for_pixels;

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;

/// Nothing ever disappears below this, however far away or however small.
constexpr double kMinimumPixels = 3.0;

constexpr Color kBackground{10, 12, 20, 255};
constexpr Color kText{200, 210, 230, 255};
constexpr Color kDim{90, 100, 120, 255};

/// Purely cosmetic. Bodies all drawn in one colour are genuinely hard to read, and a Body
/// carries no colour of its own, so the viewer picks one. Warm for the star, cool for the
/// rest.
Color body_colour(orbitalis::BodyId id)
{
    static constexpr Color palette[] = {
        Color{255, 214, 120, 255},  // star
        Color{120, 175, 255, 255},  // blue
        Color{200, 160, 130, 255},  // tan
        Color{160, 220, 190, 255},  // green
        Color{215, 150, 200, 255},  // violet
    };
    return palette[id % (sizeof(palette) / sizeof(palette[0]))];
}

Vector3 to_raylib(const orbitalis::render::Vec3f& v) noexcept
{
    return Vector3{v.x, v.y, v.z};
}

float distance_between(const Vector3& a, const Vector3& b) noexcept
{
    const float dx = a.x - b.x;
    const float dy = a.y - b.y;
    const float dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// Radius to actually draw a body at: the compressed size, but never below the pixel floor.
///
/// The floor is computed per body rather than once, because it depends on how far that
/// particular body is from the camera. Something twice as far away needs twice the world
/// radius to cover the same three pixels.
double drawn_radius(const orbitalis::Body& body,
                    const orbitalis::render::Vec3f& render_position,
                    const BodyScale& body_scale,
                    const RenderFrame& frame,
                    const Camera3D& camera,
                    int viewport_height)
{
    const double sized = body_scale.render_radius(body.radius, frame.metres_per_unit());

    const double distance = distance_between(camera.position, to_raylib(render_position));
    const double floor = world_radius_for_pixels(kMinimumPixels, distance,
                                                 camera.fovy, viewport_height);

    return std::max(sized, floor);
}

void draw_hud(const System& system, const RenderFrame& frame, const BodyScale& body_scale)
{
    DrawText(orbitalis::version_banner(), 24, 24, 28, kText);
    DrawText(orbitalis::milestone_name(), 24, 60, 18, kDim);

    char line[160];

    std::snprintf(line, sizeof(line), "scale   %.4e m per render unit", frame.metres_per_unit());
    DrawText(line, 24, 100, 18, kText);

    std::snprintf(line, sizeof(line), "sizing  %s",
                  body_scale.true_scale() ? "TRUE SCALE (physically honest)"
                                          : "compressed, cube root");
    DrawText(line, 24, 124, 18,
             body_scale.true_scale() ? Color{255, 180, 120, 255} : kText);

    std::snprintf(line, sizeof(line), "floor   %.0f pixels minimum", kMinimumPixels);
    DrawText(line, 24, 148, 18, kDim);

    int y = 186;
    for (orbitalis::BodyId i = 0; i < system.size(); ++i) {
        const std::string_view name = system.name(i);
        const double exaggeration =
            body_scale.exaggeration(system[i].radius, frame.metres_per_unit());

        // Saying out loud how much the picture is lying. A viewer that silently draws the
        // Earth 300 times too big is teaching something false about the solar system.
        std::snprintf(line, sizeof(line), "%-6.*s  radius %.4e m   drawn %.0fx too big",
                      static_cast<int>(name.size()), name.data(), system[i].radius,
                      exaggeration);
        DrawText(line, 24, y, 16, body_colour(i));
        y += 22;
    }

    DrawText("T  toggle true scale        ESC  close", 24, GetScreenHeight() - 34, 16, kDim);
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

    const Vec3 focus = system.center_of_mass();
    const RenderFrame frame{focus, RenderFrame::fit_scale(system.bodies(), focus, 6.0)};

    BodyScale body_scale;

    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
    InitWindow(kDefaultWidth, kDefaultHeight, "Orbitalis-3D");

    if (!IsWindowReady()) {
        std::fprintf(stderr, "failed to open a window\n");
        return 1;
    }

    SetTargetFPS(60);

    Camera3D camera{};
    camera.position = Vector3{9.0f, 7.0f, 9.0f};
    camera.target = Vector3{0.0f, 0.0f, 0.0f};
    camera.up = Vector3{0.0f, 1.0f, 0.0f};
    camera.fovy = 45.0f;
    camera.projection = CAMERA_PERSPECTIVE;

    while (!WindowShouldClose()) {
        if (IsKeyPressed(KEY_T)) {
            body_scale.set_true_scale(!body_scale.true_scale());
        }

        BeginDrawing();
        ClearBackground(kBackground);

        BeginMode3D(camera);
        DrawGrid(20, 1.0f);

        for (orbitalis::BodyId i = 0; i < system.size(); ++i) {
            const auto position = frame.to_render(system[i].position);
            const double radius = drawn_radius(system[i], position, body_scale, frame,
                                               camera, GetScreenHeight());

            // Modest tessellation: these are small on screen and there are two of them.
            // At 100k bodies (0.4.0) spheres stop being viable and this becomes point
            // sprites or instanced geometry.
            DrawSphereEx(to_raylib(position), static_cast<float>(radius), 16, 16,
                         body_colour(i));
        }
        EndMode3D();

        draw_hud(system, frame, body_scale);
        EndDrawing();
    }

    CloseWindow();
    return 0;
}
