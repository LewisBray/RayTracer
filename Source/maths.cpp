#include "maths.h"

#include <cassert>
#include <cmath>

#include <immintrin.h>

// Floating point util routines
static float fp_sqrt(const float x) {
    const __m128 x128 = _mm_load_ps1(&x);
    const __m128 sqrt_x128 = _mm_sqrt_ps(x128);
    return _mm_cvtss_f32(sqrt_x128);
}

static float fp_min(const float x, const float y) {
    const __m128 x128 = _mm_load_ps1(&x);
    const __m128 y128 = _mm_load_ps1(&y);
    const __m128 z = _mm_min_ss(x128, y128);
    return _mm_cvtss_f32(z);
}

static float fp_max(const float x, const float y) {
    const __m128 x128 = _mm_load_ps1(&x);
    const __m128 y128 = _mm_load_ps1(&y);
    const __m128 z = _mm_max_ss(x128, y128);
    return _mm_cvtss_f32(z);
}

static float fp_abs(float x) {
    auto* const p = reinterpret_cast<unsigned char*>(&x);
    p[3] &= 0x7f;
    return x;
}

static bool are_equal(const float x, const float y) noexcept {
    return (fp_abs(y - x) < tolerance);
}

static bool less_than(const float x, const float y) noexcept {
    return (y - x > tolerance);
}

static bool greater_than(const float x, const float y) noexcept {
    return (x - y > tolerance);
}


// Angle conversion
static float to_radians(const float angle_in_degrees) noexcept {
    return angle_in_degrees / 180.0f * PI;
}


// Vector operations
static Vector operator+(const Vector& lhs, const Vector& rhs) noexcept {
    return Vector{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

static Vector operator-(const Vector& lhs, const Vector& rhs) noexcept {
    return Vector{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

static float operator*(const Vector& lhs, const Vector& rhs) noexcept {
    return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

static Vector operator*(const float scalar, const Vector& v) noexcept {    
    return Vector{scalar * v.x, scalar * v.y, scalar * v.z};
}

static Vector operator^(const Vector& lhs, const Vector& rhs) noexcept {
    return Vector {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x
    };
}

static Vector operator/(const Vector& v, const float scalar) noexcept {
    assert(scalar != 0.0f);
    const float inverse = 1.0f / scalar;
    return Vector{inverse * v.x, inverse * v.y, inverse * v.z};
}

static float magnitude(const Vector& v) {
    return fp_sqrt(v * v);
}

static Vector normalise(const Vector& v) {
    return v / magnitude(v);
}


// Matrix operations
static Vector operator*(const Matrix& m, const Vector& v) noexcept {
    return Vector {
        m.rows[0][0] * v.x + m.rows[0][1] * v.y + m.rows[0][2] * v.z + m.rows[0][3],
        m.rows[1][0] * v.x + m.rows[1][1] * v.y + m.rows[1][2] * v.z + m.rows[1][3],
        m.rows[2][0] * v.x + m.rows[2][1] * v.y + m.rows[2][2] * v.z + m.rows[2][3]
    };
}

static Matrix operator*(const Matrix& lhs, const Matrix& rhs) noexcept {
    Matrix ret;
    ret.rows[0][0] = lhs.rows[0][0] * rhs.rows[0][0] + lhs.rows[0][1] * rhs.rows[1][0] + lhs.rows[0][2] * rhs.rows[2][0];
    ret.rows[0][1] = lhs.rows[0][0] * rhs.rows[0][1] + lhs.rows[0][1] * rhs.rows[1][1] + lhs.rows[0][2] * rhs.rows[2][1];
    ret.rows[0][2] = lhs.rows[0][0] * rhs.rows[0][2] + lhs.rows[0][1] * rhs.rows[1][2] + lhs.rows[0][2] * rhs.rows[2][2];
    ret.rows[0][3] = lhs.rows[0][0] * rhs.rows[0][3] + lhs.rows[0][1] * rhs.rows[1][3] + lhs.rows[0][2] * rhs.rows[2][3] + lhs.rows[0][3];

    ret.rows[1][0] = lhs.rows[1][0] * rhs.rows[0][0] + lhs.rows[1][1] * rhs.rows[1][0] + lhs.rows[1][2] * rhs.rows[2][0];
    ret.rows[1][1] = lhs.rows[1][0] * rhs.rows[0][1] + lhs.rows[1][1] * rhs.rows[1][1] + lhs.rows[1][2] * rhs.rows[2][1];
    ret.rows[1][2] = lhs.rows[1][0] * rhs.rows[0][2] + lhs.rows[1][1] * rhs.rows[1][2] + lhs.rows[1][2] * rhs.rows[2][2];
    ret.rows[1][3] = lhs.rows[1][0] * rhs.rows[0][3] + lhs.rows[1][1] * rhs.rows[1][3] + lhs.rows[1][2] * rhs.rows[2][3] + lhs.rows[1][3];

    ret.rows[2][0] = lhs.rows[2][0] * rhs.rows[0][0] + lhs.rows[2][1] * rhs.rows[1][0] + lhs.rows[2][2] * rhs.rows[2][0];
    ret.rows[2][1] = lhs.rows[2][0] * rhs.rows[0][1] + lhs.rows[2][1] * rhs.rows[1][1] + lhs.rows[2][2] * rhs.rows[2][1];
    ret.rows[2][2] = lhs.rows[2][0] * rhs.rows[0][2] + lhs.rows[2][1] * rhs.rows[1][2] + lhs.rows[2][2] * rhs.rows[2][2];
    ret.rows[2][3] = lhs.rows[2][0] * rhs.rows[0][3] + lhs.rows[2][1] * rhs.rows[1][3] + lhs.rows[2][2] * rhs.rows[2][3] + lhs.rows[2][3];
    
    return ret;
}

static Matrix identity_matrix() noexcept {
    return Matrix {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    };
}

static Matrix scaling_matrix(const float x_scale, const float y_scale, const float z_scale) noexcept {
    return Matrix {
        x_scale,    0.0f,    0.0f,    0.0f,
        0.0f,    y_scale,    0.0f,    0.0f,
        0.0f,       0.0f, z_scale,    0.0f
    };
}

static Matrix translation_matrix(const float x_offset, const float y_offset, const float z_offset) noexcept {
    return Matrix {
        1.0f, 0.0f, 0.0f, x_offset,
        0.0f, 1.0f, 0.0f, y_offset,
        0.0f, 0.0f, 1.0f, z_offset
    };
}

static Matrix rotation_matrix(const float angle, const float axis_x, const float axis_y, const float axis_z) noexcept {
    const Vector rotation_vector = normalise(Vector{axis_x, axis_y, axis_z});
    const float sin_angle = std::sin(angle);
    const float cos_angle = std::cos(angle);

    Matrix ret;
    ret.rows[0][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.x + cos_angle;
    ret.rows[0][1] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.y - sin_angle * rotation_vector.z;
    ret.rows[0][2] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.z + sin_angle * rotation_vector.y;
    ret.rows[0][3] = 0.0f;

    ret.rows[1][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.y + sin_angle * rotation_vector.z;
    ret.rows[1][1] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.y + cos_angle;
    ret.rows[1][2] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.z - sin_angle * rotation_vector.x;
    ret.rows[1][3] = 0.0f;

    ret.rows[2][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.z - sin_angle * rotation_vector.y;
    ret.rows[2][1] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.z + sin_angle * rotation_vector.x;
    ret.rows[2][2] = (1.0f - cos_angle) * rotation_vector.z * rotation_vector.z + cos_angle;
    ret.rows[2][3] = 0.0f;

    return ret;
}


// Shape operations
static Vector unit_surface_normal(const Triangle& triangle) noexcept {
    return normalise((triangle.b - triangle.a) ^ (triangle.c - triangle.a));
}

static Vector unit_surface_normal(const Sphere& sphere, const Vector& point) noexcept {
    return normalise(point - sphere.centre);
}

static Vector transform_direction_by_transpose(const Matrix& m, const Vector& v) noexcept {
    return Vector {
        m.rows[0][0] * v.x + m.rows[1][0] * v.y + m.rows[2][0] * v.z,
        m.rows[0][1] * v.x + m.rows[1][1] * v.y + m.rows[2][1] * v.z,
        m.rows[0][2] * v.x + m.rows[1][2] * v.y + m.rows[2][2] * v.z
    };
}

static Vector unit_surface_normal(const Ellipsoid& ellipsoid, const Vector& point) noexcept {
    const Vector point_in_ellipsoid_space = ellipsoid.inverse_transform * point;
    const Vector unit_surface_normal_in_ellipsoid_space = normalise(point_in_ellipsoid_space);  // centre at origin and radius 1 in ellipsoid space
    const Vector surface_normal = transform_direction_by_transpose(ellipsoid.inverse_transform, unit_surface_normal_in_ellipsoid_space);

    return normalise(surface_normal);
}
