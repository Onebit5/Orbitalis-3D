#pragma once

#include <cmath>

namespace orbitalis {

/// A 3D vector of doubles. Position, velocity, acceleration and force all use this.
///
/// Not a template. I only ever want doubles in the simulation (a float cannot resolve
/// Earth's orbital radius to better than ~16 km, which is larger than the distance Earth
/// covers in one timestep), and a template I instantiate exactly once is a template I
/// should not have written.
///
/// Plain aggregate: no user-declared constructors, so `Vec3 v{1.0, 2.0, 3.0}` works,
/// the type stays trivially copyable, and `Vec3 v{}` zero-initialises.
struct Vec3
{
    double x{};
    double y{};
    double z{};

    // ---- element access -----------------------------------------------------------
    //
    // Indexed access exists for the Barnes-Hut octree at 0.3.2, which picks a child
    // octant by comparing each axis in turn. Written as a chain of ternaries rather than
    // the usual `(&x)[i]` pointer trick: that trick is technically undefined behaviour
    // (you may not walk a pointer from one member to the next, even in a standard-layout
    // struct) and it is not usable in a constant expression. This version is legal,
    // constexpr, and optimises to the same thing.
    //
    // No bounds checking. i must be 0, 1 or 2.

    [[nodiscard]] constexpr double& operator[](int i) noexcept
    {
        return i == 0 ? x : (i == 1 ? y : z);
    }

    [[nodiscard]] constexpr const double& operator[](int i) const noexcept
    {
        return i == 0 ? x : (i == 1 ? y : z);
    }

    // ---- compound assignment ------------------------------------------------------

    constexpr Vec3& operator+=(const Vec3& v) noexcept
    {
        x += v.x;
        y += v.y;
        z += v.z;
        return *this;
    }

    constexpr Vec3& operator-=(const Vec3& v) noexcept
    {
        x -= v.x;
        y -= v.y;
        z -= v.z;
        return *this;
    }

    constexpr Vec3& operator*=(double s) noexcept
    {
        x *= s;
        y *= s;
        z *= s;
        return *this;
    }

    /// Divides component-wise rather than multiplying by the reciprocal.
    ///
    /// Multiplying by 1/s is the classic "optimisation" here, and it is wrong for this
    /// project: it introduces a second rounding step, so v / 3.0 stops being correctly
    /// rounded. Over a long integration those extra ULPs are exactly the kind of thing I
    /// am trying to keep out of the energy drift I will be measuring at 0.2.3.
    constexpr Vec3& operator/=(double s) noexcept
    {
        x /= s;
        y /= s;
        z /= s;
        return *this;
    }

    // ---- magnitude ----------------------------------------------------------------

    /// Squared length. constexpr, and the one you want in comparisons and in the
    /// gravity kernel, because it avoids a square root.
    [[nodiscard]] constexpr double length_squared() const noexcept
    {
        return x * x + y * y + z * z;
    }

    /// Length. NOT constexpr: std::sqrt only becomes constexpr in C++26 and this project
    /// is C++20.
    [[nodiscard]] double length() const noexcept { return std::sqrt(length_squared()); }

    /// Unit vector in the same direction.
    ///
    /// Precondition: length() is not zero. A zero vector yields NaN components rather
    /// than an error. That is deliberate; branching here would put a test in the hot
    /// path for a case that should never occur, and NaN is loud enough to find with
    /// is_finite() once something has gone wrong.
    [[nodiscard]] Vec3 normalized() const noexcept
    {
        const double len = length();
        return Vec3{x / len, y / len, z / len};
    }

    /// True if every component is finite. Worth having early: once softened gravity and
    /// close encounters arrive at 0.0.4, a single NaN propagates to every body in the
    /// system within one step, because they all pull on each other. Catching it at the
    /// source beats staring at a screen full of vanished planets.
    [[nodiscard]] bool is_finite() const noexcept
    {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }
};

// ---- unary --------------------------------------------------------------------------

[[nodiscard]] constexpr Vec3 operator+(const Vec3& v) noexcept { return v; }

[[nodiscard]] constexpr Vec3 operator-(const Vec3& v) noexcept { return Vec3{-v.x, -v.y, -v.z}; }

// ---- binary -------------------------------------------------------------------------
//
// First argument by value so the compound-assignment operator does the work once.

[[nodiscard]] constexpr Vec3 operator+(Vec3 a, const Vec3& b) noexcept { return a += b; }

[[nodiscard]] constexpr Vec3 operator-(Vec3 a, const Vec3& b) noexcept { return a -= b; }

[[nodiscard]] constexpr Vec3 operator*(Vec3 v, double s) noexcept { return v *= s; }

/// Scalar on the left, so `2.0 * v` reads the way it does on paper.
[[nodiscard]] constexpr Vec3 operator*(double s, Vec3 v) noexcept { return v *= s; }

[[nodiscard]] constexpr Vec3 operator/(Vec3 v, double s) noexcept { return v /= s; }

// ---- comparison ---------------------------------------------------------------------

/// Exact component-wise equality.
///
/// This compares doubles with ==, which is almost never what you want for computed
/// results. It is here for the cases where it is meaningful: checking against a value
/// you literally assigned, and container operations. For anything that has been through
/// arithmetic, use approx_equal().
[[nodiscard]] constexpr bool operator==(const Vec3& a, const Vec3& b) noexcept
{
    return a.x == b.x && a.y == b.y && a.z == b.z;
}

[[nodiscard]] constexpr bool operator!=(const Vec3& a, const Vec3& b) noexcept
{
    return !(a == b);
}

// ---- products -----------------------------------------------------------------------

[[nodiscard]] constexpr double dot(const Vec3& a, const Vec3& b) noexcept
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Right-handed cross product. Needed for angular momentum at 0.2.3 and for the orbital
/// elements at 0.5.1, where the orbit normal is r x v.
[[nodiscard]] constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept
{
    return Vec3{
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x,
    };
}

// ---- distance -----------------------------------------------------------------------

/// Squared distance between two points. This is the one the gravity kernel actually
/// wants at 0.0.4: Newton's law needs r^2 in the denominator, so taking a square root
/// only to square it again would be wasted work on the hottest loop in the project.
[[nodiscard]] constexpr double distance_squared(const Vec3& a, const Vec3& b) noexcept
{
    return (b - a).length_squared();
}

[[nodiscard]] inline double distance(const Vec3& a, const Vec3& b) noexcept
{
    return (b - a).length();
}

// ---- tolerant comparison ------------------------------------------------------------

/// Component-wise comparison with an absolute tolerance.
///
/// Absolute rather than relative on purpose. A relative test is the better default in
/// general, but it degenerates when the expected value is zero, and vectors in this
/// project frequently have exact zero components (anything set up in a plane has z = 0).
/// Callers scale the tolerance to the magnitudes they are working with.
[[nodiscard]] inline bool approx_equal(const Vec3& a, const Vec3& b, double tol = 1e-12) noexcept
{
    return std::abs(a.x - b.x) <= tol
        && std::abs(a.y - b.y) <= tol
        && std::abs(a.z - b.z) <= tol;
}

}  // namespace orbitalis
