#include "maths.h"

#include <algorithm>
#include <iterator>
#include <ostream>
#include <cassert>
#include <array>
#include <cmath>

// Angle conversion
real to_radians(const real angle_in_degrees) noexcept {
    return angle_in_degrees / 180.0 * pi;
}


// Floating point comparisons
constexpr real tolerance = 1.0e-6;

bool are_equal(const real x, const real y) noexcept {
    return (std::abs(y - x) < tolerance);
}

bool less_than(const real x, const real y) noexcept {
    return (y - x > tolerance);
}

bool greater_than(const real x, const real y) noexcept {
    return (x - y > tolerance);
}


// Vector operations
Vector operator+(const Vector& lhs, const Vector& rhs) noexcept {
    return Vector {
        lhs.x * rhs.w + rhs.x * lhs.w,
        lhs.y * rhs.w + rhs.y * lhs.w,
        lhs.z * rhs.w + rhs.z * lhs.w,
        lhs.w * rhs.w
    };
}

Vector operator-(const Vector& lhs, const Vector& rhs) noexcept {
    return Vector {
        lhs.x * rhs.w - rhs.x * lhs.w,
        lhs.y * rhs.w - rhs.y * lhs.w,
        lhs.z * rhs.w - rhs.z * lhs.w,
        lhs.w * rhs.w
    };
}

real operator*(const Vector& lhs, const Vector& rhs) noexcept {
    assert(lhs.w != 0.0 && rhs.w != 0.0);
    return (lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z) / (lhs.w * rhs.w);
}

Vector operator*(const real scalar, const Vector& v) noexcept {
    if (scalar == 0.0) {
        return Vector{0.0, 0.0, 0.0, 1.0};
    }
    
    return Vector{v.x, v.y,v.z, v.w / scalar};
}

Vector operator^(const Vector& lhs, const Vector& rhs) noexcept {
    return Vector {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
        lhs.w * rhs.w
    };
}

Vector operator/(const Vector& v, const real scalar) noexcept {
    assert(scalar != 0.0);
    return Vector{v.x, v.y, v.z, scalar * v.w};
}

// Likely just used for debugging and can probably be removed later
std::ostream& operator<<(std::ostream& out, const Vector& v) {
    out << '(' << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ')';
    return out;
}

real magnitude(const Vector& v) {
    return std::sqrt(v * v);
}

Vector normalise(const Vector& v) {
    return v / magnitude(v);
}

Vector homogenise(const Vector& v) noexcept {
    assert(v.w != 0.0);
    return Vector{v.x / v.w, v.y / v.w, v.z / v.w, 1.0};
}


// Matrix operations
Vector operator*(const Matrix& m, const Vector& v) noexcept {
    return Vector {
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
        m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w
    };
}

Matrix operator*(const Matrix& lhs, const Matrix& rhs) noexcept {
    Matrix ret;
    for (std::array<real, 4>& row : ret) {
        std::fill(row.begin(), row.end(), 0.0);
    }

    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 4; ++j) {
            for (std::size_t k = 0; k < 4; ++k) {
                ret[i][k] += lhs[i][j] * rhs[j][k];
            }
        }
    }
    
    return ret;
}

Matrix identity_matrix() noexcept {
    return Matrix {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
}

Matrix scaling_matrix(const real x_scale, const real y_scale, const real z_scale) noexcept {
    Matrix ret;
    for (std::array<real, 4>& row : ret) {
        std::fill(row.begin(), row.end(), 0.0);
    }

    ret[0][0] = x_scale;
    ret[1][1] = y_scale;
    ret[2][2] = z_scale;
    ret[3][3] = 1.0;

    return ret;
}

Matrix translation_matrix(const real x_offset, const real y_offset, const real z_offset) noexcept {
    Matrix ret;
    for (std::array<real, 4>& row : ret) {
        std::fill(row.begin(), row.end(), 0.0);
    }
    
    ret[0][3] = x_offset;
    ret[1][3] = y_offset;
    ret[2][3] = z_offset;
    for (int i = 0; i < 4; ++i) {
        ret[i][i] = 1.0;
    }

    return ret;
}

Matrix rotation_matrix(const real angle, const real axis_x, const real axis_y, const real axis_z) noexcept {
    const Vector rotation_vector = homogenise(normalise(Vector{axis_x, axis_y, axis_z, 1.0}));
    const real sin_angle = std::sin(angle);
    const real cos_angle = std::cos(angle);

    Matrix ret;
    ret[0][0] = (1.0 - cos_angle) * rotation_vector.x * rotation_vector.x + cos_angle;
    ret[0][1] = (1.0 - cos_angle) * rotation_vector.x * rotation_vector.y - sin_angle * rotation_vector.z;
    ret[0][2] = (1.0 - cos_angle) * rotation_vector.x * rotation_vector.z + sin_angle * rotation_vector.y;
    ret[0][3] = 0.0;

    ret[1][0] = (1.0 - cos_angle) * rotation_vector.x * rotation_vector.y + sin_angle * rotation_vector.z;
    ret[1][1] = (1.0 - cos_angle) * rotation_vector.y * rotation_vector.y + cos_angle;
    ret[1][2] = (1.0 - cos_angle) * rotation_vector.y * rotation_vector.z - sin_angle * rotation_vector.x;
    ret[1][3] = 0.0;

    ret[2][0] = (1.0 - cos_angle) * rotation_vector.x * rotation_vector.z - sin_angle * rotation_vector.y;
    ret[2][1] = (1.0 - cos_angle) * rotation_vector.y * rotation_vector.z + sin_angle * rotation_vector.x;
    ret[2][2] = (1.0 - cos_angle) * rotation_vector.z * rotation_vector.z + cos_angle;
    ret[2][3] = 0.0;

    ret[3][0] = 0.0;
    ret[3][1] = 0.0;
    ret[3][2] = 0.0;
    ret[3][3] = 1.0;

    return ret;
}


// Shape operations
Vector unit_surface_normal(const Triangle& triangle) noexcept {
    return normalise((triangle.b - triangle.a) ^ (triangle.c - triangle.a));
}

Vector unit_surface_normal(const Sphere& sphere, const Vector& point) noexcept {
    return normalise(point - sphere.centre);
}

static Vector multiply_by_transpose(const Matrix& m, const Vector& v) noexcept {
    return Vector {
        m[0][0] * v.x + m[1][0] * v.y + m[2][0] * v.z + m[3][0] * v.w,
        m[0][1] * v.x + m[1][1] * v.y + m[2][1] * v.z + m[3][1] * v.w,
        m[0][2] * v.x + m[1][2] * v.y + m[2][2] * v.z + m[3][2] * v.w,
        m[0][3] * v.x + m[1][3] * v.y + m[2][3] * v.z + m[3][3] * v.w
    };
}

Vector unit_surface_normal(const Ellipsoid& ellipsoid, const Vector& point) noexcept {
    const Vector point_in_ellipsoid_space = ellipsoid.inverse_transform * point;    
    const Vector unit_surface_normal_in_ellipsoid_space = unit_surface_normal(ellipsoid.sphere, point_in_ellipsoid_space);
    
    Vector surface_normal = homogenise(unit_surface_normal_in_ellipsoid_space);
    surface_normal.w = 0.0;
    surface_normal = multiply_by_transpose(ellipsoid.inverse_transform, surface_normal);
    surface_normal.w = 1.0;

    return normalise(surface_normal);
}
