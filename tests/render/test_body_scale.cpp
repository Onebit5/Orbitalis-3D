#include <doctest/doctest.h>

#include <orbitalis/render/BodyScale.hpp>

#include <cmath>
#include <numbers>

using orbitalis::render::BodyScale;
using orbitalis::render::world_radius_for_pixels;

namespace {

constexpr double kSunRadius = 6.957e8;      // m
constexpr double kJupiterRadius = 6.9911e7;
constexpr double kEarthRadius = 6.371e6;
constexpr double kCeresRadius = 4.73e5;

/// The scale RenderFrame::fit_scale picked for Sun-Earth at six render units.
constexpr double kMetresPerUnit = 2.4933e10;

}  // namespace

// =======================================================================================
// the problem this class exists to solve
// =======================================================================================

TEST_CASE("at true scale the planets are invisible")
{
    // Not a figure of speech. This is the motivating measurement, and it is why the
    // default mode lies about sizes.
    BodyScale::Config config;
    config.true_scale = true;
    const BodyScale scale{config};

    const double earth_units = scale.render_radius(kEarthRadius, kMetresPerUnit);
    const double sun_units = scale.render_radius(kSunRadius, kMetresPerUnit);

    // At 14.53 units from the camera, 45 degree vertical FOV, 720 px tall, one render unit
    // covers 59.8 pixels.
    const double px_per_unit = 1.0 / world_radius_for_pixels(1.0, 14.53, 45.0, 720);

    CHECK(earth_units * px_per_unit == doctest::Approx(0.0153).epsilon(1e-2));
    CHECK(sun_units * px_per_unit == doctest::Approx(1.67).epsilon(1e-2));

    SUBCASE("Earth is well under a hundredth of a pixel per side")
    {
        CHECK(earth_units * px_per_unit < 0.02);
    }
}

// =======================================================================================
// the compression
// =======================================================================================

TEST_CASE("the reference radius maps to the reference size")
{
    const BodyScale scale;  // defaults: Earth's radius -> 0.08 units

    CHECK(scale.render_radius(kEarthRadius, kMetresPerUnit) == doctest::Approx(0.08));
}

TEST_CASE("compression keeps the ordering but squashes the ratios")
{
    const BodyScale scale;

    const double sun = scale.render_radius(kSunRadius, kMetresPerUnit);
    const double jupiter = scale.render_radius(kJupiterRadius, kMetresPerUnit);
    const double earth = scale.render_radius(kEarthRadius, kMetresPerUnit);
    const double ceres = scale.render_radius(kCeresRadius, kMetresPerUnit);

    SUBCASE("strictly monotonic: a bigger body is never drawn smaller")
    {
        // The property that matters most. The picture lies about ratios, but it must never
        // lie about which body is larger.
        CHECK(sun > jupiter);
        CHECK(jupiter > earth);
        CHECK(earth > ceres);
    }

    SUBCASE("a 1471:1 physical range becomes about 11:1 on screen")
    {
        CHECK(kSunRadius / kCeresRadius == doctest::Approx(1470.8).epsilon(1e-3));
        CHECK(sun / ceres == doctest::Approx(11.37).epsilon(1e-2));
    }

    SUBCASE("and 109:1 becomes about 4.8:1")
    {
        CHECK(sun / earth == doctest::Approx(4.78).epsilon(1e-2));
    }

    SUBCASE("everything lands in a drawable range")
    {
        // Small enough not to swallow a six-unit orbit, big enough to see.
        CHECK(sun < 0.5);
        CHECK(ceres > 0.01);
    }
}

TEST_CASE("an exponent of one restores true proportions")
{
    BodyScale::Config config;
    config.exponent = 1.0;
    const BodyScale scale{config};

    const double sun = scale.render_radius(kSunRadius, kMetresPerUnit);
    const double earth = scale.render_radius(kEarthRadius, kMetresPerUnit);

    CHECK(sun / earth == doctest::Approx(kSunRadius / kEarthRadius).epsilon(1e-9));
}

TEST_CASE("a smaller exponent compresses harder")
{
    auto ratio_at = [](double exponent) {
        BodyScale::Config config;
        config.exponent = exponent;
        const BodyScale scale{config};
        return scale.render_radius(kSunRadius, kMetresPerUnit)
               / scale.render_radius(kCeresRadius, kMetresPerUnit);
    };

    CHECK(ratio_at(1.0) > ratio_at(0.5));
    CHECK(ratio_at(0.5) > ratio_at(1.0 / 3.0));
    CHECK(ratio_at(1.0 / 3.0) > ratio_at(0.25));

    // sqrt of 1470.8
    CHECK(ratio_at(0.5) == doctest::Approx(38.35).epsilon(1e-3));
}

TEST_CASE("exaggerated sizing ignores the render scale")
{
    // A body should look the same whether the scenario is a planetary system or a star
    // cluster, because fit_scale already normalised the extent. Only true-scale mode cares
    // about metres_per_unit.
    const BodyScale scale;

    CHECK(scale.render_radius(kEarthRadius, 1.0e6)
          == doctest::Approx(scale.render_radius(kEarthRadius, 1.0e14)));
}

TEST_CASE("true scale does depend on the render scale")
{
    BodyScale::Config config;
    config.true_scale = true;
    const BodyScale scale{config};

    CHECK(scale.render_radius(kEarthRadius, 1.0e6) == doctest::Approx(6.371));
    CHECK(scale.render_radius(kEarthRadius, 1.0e7) == doctest::Approx(0.6371));
}

TEST_CASE("exaggeration reports how big the lie is")
{
    const BodyScale scale;

    const double earth = scale.exaggeration(kEarthRadius, kMetresPerUnit);
    const double sun = scale.exaggeration(kSunRadius, kMetresPerUnit);

    // Measured in the viewer: Earth is drawn 313x too big and the Sun only 14x. That is
    // the compression working, and it is the whole point of a power law: small bodies need
    // far more help than large ones, so they get more.
    CHECK(earth == doctest::Approx(313.0).epsilon(1e-2));
    CHECK(sun == doctest::Approx(13.7).epsilon(1e-2));
    CHECK(earth > sun);

    SUBCASE("and true scale is not a lie at all")
    {
        BodyScale::Config config;
        config.true_scale = true;
        const BodyScale honest{config};
        CHECK(honest.exaggeration(kEarthRadius, kMetresPerUnit) == doctest::Approx(1.0));
        CHECK(honest.exaggeration(kSunRadius, kMetresPerUnit) == doctest::Approx(1.0));
    }
}

// =======================================================================================
// the screen-space floor
// =======================================================================================

TEST_CASE("world_radius_for_pixels inverts the perspective projection")
{
    const double d = 10.0;
    const double fovy = 45.0;
    constexpr int height = 720;

    const double r = world_radius_for_pixels(3.0, d, fovy, height);

    // Project it back by hand and check we get three pixels out.
    const double pixels = r * height / (2.0 * d * std::tan(0.5 * fovy * std::numbers::pi / 180.0));
    CHECK(pixels == doctest::Approx(3.0).epsilon(1e-12));
}

TEST_CASE("the floor scales the way perspective does")
{
    SUBCASE("twice as far needs twice the radius")
    {
        CHECK(world_radius_for_pixels(3.0, 20.0, 45.0, 720)
              == doctest::Approx(2.0 * world_radius_for_pixels(3.0, 10.0, 45.0, 720)));
    }

    SUBCASE("twice the pixels needs twice the radius")
    {
        CHECK(world_radius_for_pixels(6.0, 10.0, 45.0, 720)
              == doctest::Approx(2.0 * world_radius_for_pixels(3.0, 10.0, 45.0, 720)));
    }

    SUBCASE("a taller viewport needs less radius for the same pixel count")
    {
        CHECK(world_radius_for_pixels(3.0, 10.0, 45.0, 1440)
              == doctest::Approx(0.5 * world_radius_for_pixels(3.0, 10.0, 45.0, 720)));
    }

    SUBCASE("a wider field of view needs more radius")
    {
        CHECK(world_radius_for_pixels(3.0, 10.0, 90.0, 720)
              > world_radius_for_pixels(3.0, 10.0, 45.0, 720));
    }
}

TEST_CASE("the floor actually rescues a true-scale Earth")
{
    // The two halves of this step working together: even in honest mode, nothing vanishes.
    BodyScale::Config config;
    config.true_scale = true;
    const BodyScale scale{config};

    const double honest = scale.render_radius(kEarthRadius, kMetresPerUnit);
    const double floor = world_radius_for_pixels(3.0, 14.53, 45.0, 720);

    CHECK(honest < floor);              // would have been invisible
    CHECK(std::max(honest, floor) == floor);
}

// =======================================================================================
// degenerate inputs
// =======================================================================================

TEST_CASE("a point mass has no size of its own")
{
    const BodyScale scale;

    // Bodies default to radius 0 and gravity never reads the field, so this is a normal
    // state rather than an error. The pixel floor is what makes them visible.
    CHECK(scale.render_radius(0.0, kMetresPerUnit) == 0.0);
    CHECK(scale.render_radius(-1.0, kMetresPerUnit) == 0.0);
}

TEST_CASE("a broken config is corrected rather than producing NaN")
{
    BodyScale::Config config;
    config.reference_radius = 0.0;
    config.reference_size = -1.0;
    config.exponent = 0.0;

    const BodyScale scale{config};

    CHECK(scale.config().reference_radius > 0.0);
    CHECK(scale.config().reference_size > 0.0);
    CHECK(scale.config().exponent > 0.0);
    CHECK(std::isfinite(scale.render_radius(kEarthRadius, kMetresPerUnit)));
}

TEST_CASE("world_radius_for_pixels rejects nonsense instead of returning infinity")
{
    CHECK(world_radius_for_pixels(0.0, 10.0, 45.0, 720) == 0.0);
    CHECK(world_radius_for_pixels(3.0, 0.0, 45.0, 720) == 0.0);
    CHECK(world_radius_for_pixels(3.0, 10.0, 45.0, 0) == 0.0);
    CHECK(world_radius_for_pixels(3.0, 10.0, 0.0, 720) == 0.0);
    CHECK(world_radius_for_pixels(3.0, 10.0, 180.0, 720) == 0.0);
}
