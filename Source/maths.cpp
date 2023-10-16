#include <algorithm>
#include <iterator>
#include <ostream>
#include <cassert>
#include <array>
#include <cmath>

#include "maths.h"

// Angle conversion
static float to_radians(const float angle_in_degrees) noexcept {
    return angle_in_degrees / 180.0f * pi;
}


// Floating point comparisons
static bool are_equal(const float x, const float y) noexcept {
    return (std::abs(y - x) < tolerance);
}

static bool less_than(const float x, const float y) noexcept {
    return (y - x > tolerance);
}

static bool greater_than(const float x, const float y) noexcept {
    return (x - y > tolerance);
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
    return std::sqrt(v * v);
}

static Vector normalise(const Vector& v) {
    return v / magnitude(v);
}


// Matrix operations
static Vector operator*(const Matrix& m, const Vector& v) noexcept {
    return Vector {
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3],
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3],
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3]
    };
}

static Matrix operator*(const Matrix& lhs, const Matrix& rhs) noexcept {
    Matrix ret;
    ret[0][0] = lhs[0][0] * rhs[0][0] + lhs[0][1] * rhs[1][0] + lhs[0][2] * rhs[2][0];
    ret[0][1] = lhs[0][0] * rhs[0][1] + lhs[0][1] * rhs[1][1] + lhs[0][2] * rhs[2][1];
    ret[0][2] = lhs[0][0] * rhs[0][2] + lhs[0][1] * rhs[1][2] + lhs[0][2] * rhs[2][2];
    ret[0][3] = lhs[0][0] * rhs[0][3] + lhs[0][1] * rhs[1][3] + lhs[0][2] * rhs[2][3] + lhs[0][3];

    ret[1][0] = lhs[1][0] * rhs[0][0] + lhs[1][1] * rhs[1][0] + lhs[1][2] * rhs[2][0];
    ret[1][1] = lhs[1][0] * rhs[0][1] + lhs[1][1] * rhs[1][1] + lhs[1][2] * rhs[2][1];
    ret[1][2] = lhs[1][0] * rhs[0][2] + lhs[1][1] * rhs[1][2] + lhs[1][2] * rhs[2][2];
    ret[1][3] = lhs[1][0] * rhs[0][3] + lhs[1][1] * rhs[1][3] + lhs[1][2] * rhs[2][3] + lhs[1][3];

    ret[2][0] = lhs[2][0] * rhs[0][0] + lhs[2][1] * rhs[1][0] + lhs[2][2] * rhs[2][0];
    ret[2][1] = lhs[2][0] * rhs[0][1] + lhs[2][1] * rhs[1][1] + lhs[2][2] * rhs[2][1];
    ret[2][2] = lhs[2][0] * rhs[0][2] + lhs[2][1] * rhs[1][2] + lhs[2][2] * rhs[2][2];
    ret[2][3] = lhs[2][0] * rhs[0][3] + lhs[2][1] * rhs[1][3] + lhs[2][2] * rhs[2][3] + lhs[2][3];
    
    return ret;
}

static Matrix identity_matrix() noexcept {
    return Matrix {
        MatrixRow{1.0f, 0.0f, 0.0f, 0.0f},
        MatrixRow{0.0f, 1.0f, 0.0f, 0.0f},
        MatrixRow{0.0f, 0.0f, 1.0f, 0.0f}
    };
}

static Matrix scaling_matrix(const float x_scale, const float y_scale, const float z_scale) noexcept {
    return Matrix {
        MatrixRow{x_scale,    0.0f,    0.0f,    0.0f},
        MatrixRow{0.0f,    y_scale,    0.0f,    0.0f},
        MatrixRow{0.0f,       0.0f, z_scale,    0.0f}
    };
}

static Matrix translation_matrix(const float x_offset, const float y_offset, const float z_offset) noexcept {
    return Matrix {
        MatrixRow{1.0f, 0.0f, 0.0f, x_offset},
        MatrixRow{0.0f, 1.0f, 0.0f, y_offset},
        MatrixRow{0.0f, 0.0f, 1.0f, z_offset}
    };
}

static Matrix rotation_matrix(const float angle, const float axis_x, const float axis_y, const float axis_z) noexcept {
    const Vector rotation_vector = normalise(Vector{axis_x, axis_y, axis_z});
    const float sin_angle = std::sin(angle);
    const float cos_angle = std::cos(angle);

    Matrix ret;
    ret[0][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.x + cos_angle;
    ret[0][1] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.y - sin_angle * rotation_vector.z;
    ret[0][2] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.z + sin_angle * rotation_vector.y;
    ret[0][3] = 0.0f;

    ret[1][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.y + sin_angle * rotation_vector.z;
    ret[1][1] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.y + cos_angle;
    ret[1][2] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.z - sin_angle * rotation_vector.x;
    ret[1][3] = 0.0f;

    ret[2][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.z - sin_angle * rotation_vector.y;
    ret[2][1] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.z + sin_angle * rotation_vector.x;
    ret[2][2] = (1.0f - cos_angle) * rotation_vector.z * rotation_vector.z + cos_angle;
    ret[2][3] = 0.0f;

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
        m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z,
        m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z,
        m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z
    };
}

static Vector unit_surface_normal(const Ellipsoid& ellipsoid, const Vector& point) noexcept {
    const Vector point_in_ellipsoid_space = ellipsoid.inverse_transform * point;
    const Vector unit_surface_normal_in_ellipsoid_space = normalise(point_in_ellipsoid_space);  // centre at origin and radius 1 in ellipsoid space
    const Vector surface_normal = transform_direction_by_transpose(ellipsoid.inverse_transform, unit_surface_normal_in_ellipsoid_space);

    return normalise(surface_normal);
}
