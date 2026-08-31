#include <orbitalis/core/Version.hpp>

#include <raylib.h>

#include <cstdio>
#include <cstring>

// Milestone 0.1.1: raylib arrives, and a window opens.
//
// Nothing is drawn from the simulation yet. The render scale layer is 0.1.2, bodies are
// 0.1.3, the camera is 0.1.4, and the sim loop is not wired to the render loop until
// 0.1.6. This step exists to get the dependency in, prove the window opens and closes
// cleanly, and establish that raylib lives here and nowhere else.

namespace {

constexpr int kDefaultWidth = 1280;
constexpr int kDefaultHeight = 720;

// Dark, slightly blue. A black background makes it impossible to tell a window that failed
// to initialise from one that is working and empty.
constexpr Color kBackground{10, 12, 20, 255};
constexpr Color kText{200, 210, 230, 255};
constexpr Color kDim{90, 100, 120, 255};

void draw_placeholder()
{
    ClearBackground(kBackground);

    DrawText(orbitalis::version_banner(), 24, 24, 28, kText);
    DrawText(orbitalis::milestone_name(), 24, 60, 18, kDim);

    DrawText("no simulation attached yet", 24, 110, 20, kText);
    DrawText("0.1.2  render scale, camera-relative coordinates", 24, 142, 16, kDim);
    DrawText("0.1.3  bodies as spheres", 24, 164, 16, kDim);
    DrawText("0.1.4  orbit camera", 24, 186, 16, kDim);
    DrawText("0.1.5  trails", 24, 208, 16, kDim);
    DrawText("0.1.6  fixed-timestep sim loop", 24, 230, 16, kDim);

    DrawText("ESC to close", 24, GetScreenHeight() - 36, 16, kDim);

    DrawFPS(GetScreenWidth() - 96, 24);
}

}  // namespace

int main(int argc, char** argv)
{
    // Exits before InitWindow, so it runs with no display attached. This is the only path
    // CI can exercise, and it is enough to catch the raylib link breaking.
    if (argc > 1 && std::strcmp(argv[1], "--version") == 0) {
        std::printf("%s\n", orbitalis::version_banner());
        std::printf("  milestone : %s\n", orbitalis::milestone_name());
        std::printf("  target    : orbitalis-viewer\n");
        std::printf("  raylib    : %s\n", RAYLIB_VERSION);
        return 0;
    }

    // Flags have to be set before InitWindow; afterwards they are ignored.
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);

    InitWindow(kDefaultWidth, kDefaultHeight, "Orbitalis-3D");

    if (!IsWindowReady()) {
        std::fprintf(stderr, "failed to open a window\n");
        return 1;
    }

    // VSYNC_HINT above is a request the driver may ignore, so cap explicitly too. The
    // simulation timestep is deliberately not tied to any of this: that decoupling is the
    // whole point of 0.1.6, and frame delta-time must never reach the integrator.
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        BeginDrawing();
        draw_placeholder();
        EndDrawing();
    }

    // Pairs with InitWindow. raylib allocates GPU resources, an audio device and a GLFW
    // context behind that call, and none of it is freed at exit without this.
    CloseWindow();

    return 0;
}
