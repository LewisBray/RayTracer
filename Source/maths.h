#ifndef MATHS_H
#define MATHS_H

#include <ostream>
#include <array>

#define real double
#define stor stod

constexpr real pi = 3.14159265358979323846264;
real to_radians(real angle_in_degrees) noexcept;

bool are_equal(real x, real y) noexcept;
bool less_than(real x, real y) noexcept;
bool greater_than(real x, real y) noexcept;

struct Vector {
    real x;
    real y;
    real z;
    real w;
};

Vector operator+(const Vector& lhs, const Vector& rhs) noexcept;
Vector operator-(const Vector& lhs, const Vector& rhs) noexcept;
real operator*(const Vector& lhs, const Vector& rhs) noexcept;
Vector operator*(real scalar, const Vector& v) noexcept;
Vector operator^(const Vector& lhs, const Vector& rhs) noexcept;
Vector operator/(const Vector& v, real scalar) noexcept;
std::ostream& operator<<(std::ostream& out, const Vector& v);
real magnitude(const Vector& v);
Vector normalise(const Vector& v);
Vector homogenise(const Vector& v) noexcept;

using Matrix = std::array<std::array<real, 4>, 4>;

Vector operator*(const Matrix& m, const Vector& v) noexcept;
Matrix operator*(const Matrix& lhs, const Matrix& rhs) noexcept;
Matrix identity_matrix() noexcept;
Matrix scaling_matrix(real x_scale, real y_scale, real z_scale) noexcept;
Matrix translation_matrix(real x_offset, real y_offset, real z_offset) noexcept;
Matrix rotation_matrix(real angle, real axis_x, real axis_y, real axis_z) noexcept;

struct Triangle {
    Vector a;
    Vector b;
    Vector c;
};

struct Sphere {
    Vector centre;
    real radius;
};

struct Ellipsoid {
    Sphere sphere;
    Matrix inverse_transform;
};

Vector unit_surface_normal(const Triangle& triangle) noexcept;
Vector unit_surface_normal(const Sphere& sphere, const Vector& point) noexcept;
Vector unit_surface_normal(const Ellipsoid& ellipsoid, const Matrix& ellipsoid_transform, const Vector& point) noexcept;

#endif
