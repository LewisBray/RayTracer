#ifndef MATHS_H
#define MATHS_H

#include <ostream>
#include <array>

constexpr float pi = 3.14159265358979323846264f;
float to_radians(float angle_in_degrees) noexcept;

constexpr float tolerance = 1.0e-4f;
bool are_equal(float x, float y) noexcept;
bool less_than(float x, float y) noexcept;
bool greater_than(float x, float y) noexcept;

struct Vector {
    float x;
    float y;
    float z;
};

Vector operator+(const Vector& lhs, const Vector& rhs) noexcept;
Vector operator-(const Vector& lhs, const Vector& rhs) noexcept;
float operator*(const Vector& lhs, const Vector& rhs) noexcept;
Vector operator*(float scalar, const Vector& v) noexcept;
Vector operator^(const Vector& lhs, const Vector& rhs) noexcept;
Vector operator/(const Vector& v, float scalar) noexcept;
std::ostream& operator<<(std::ostream& out, const Vector& v);
float magnitude(const Vector& v);
Vector normalise(const Vector& v);

using Matrix = std::array<std::array<float, 4>, 3>;

Vector operator*(const Matrix& m, const Vector& v) noexcept;
Matrix operator*(const Matrix& lhs, const Matrix& rhs) noexcept;
Matrix identity_matrix() noexcept;
Matrix scaling_matrix(float x_scale, float y_scale, float z_scale) noexcept;
Matrix translation_matrix(float x_offset, float y_offset, float z_offset) noexcept;
Matrix rotation_matrix(float angle, float axis_x, float axis_y, float axis_z) noexcept;

struct Triangle {
    Vector a;
    Vector b;
    Vector c;
};

struct Sphere {
    Vector centre;
    float radius;
};

struct Ellipsoid {
    Sphere sphere;
    Matrix inverse_transform;
};

Vector unit_surface_normal(const Triangle& triangle) noexcept;
Vector unit_surface_normal(const Sphere& sphere, const Vector& point) noexcept;
Vector unit_surface_normal(const Ellipsoid& ellipsoid, const Vector& point) noexcept;

#endif
