#ifndef MATHS_H
#define MATHS_H

static float fp_sqrt(float x);
static float fp_min(float x, float y);
static float fp_max(float x, float y);
static float fp_abs(float x);

static constexpr float tolerance = 1.0e-4f;
static bool are_equal(float x, float y) noexcept;
static bool less_than(float x, float y) noexcept;
static bool greater_than(float x, float y) noexcept;

static constexpr float PI = 3.14159265358979323846264f;
static float to_radians(float angle_in_degrees) noexcept;

struct Vector {
    float x;
    float y;
    float z;
};

static Vector operator+(const Vector& lhs, const Vector& rhs) noexcept;
static Vector operator-(const Vector& lhs, const Vector& rhs) noexcept;
static float operator*(const Vector& lhs, const Vector& rhs) noexcept;
static Vector operator*(float scalar, const Vector& v) noexcept;
static Vector operator^(const Vector& lhs, const Vector& rhs) noexcept;
static Vector operator/(const Vector& v, float scalar) noexcept;
static float magnitude(const Vector& v);
static Vector normalise(const Vector& v);

struct Matrix {
    float rows[3][4];
};

static Vector operator*(const Matrix& m, const Vector& v) noexcept;
static Matrix operator*(const Matrix& lhs, const Matrix& rhs) noexcept;
static Matrix identity_matrix() noexcept;
static Matrix scaling_matrix(float x_scale, float y_scale, float z_scale) noexcept;
static Matrix translation_matrix(float x_offset, float y_offset, float z_offset) noexcept;
static Matrix rotation_matrix(float angle, float axis_x, float axis_y, float axis_z) noexcept;

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
    Matrix inverse_transform;
};

struct AxisAlignedBoundingBox {
    float min_x;
    float max_x;
    float min_y;
    float max_y;
    float min_z;
    float max_z;
};

static Vector unit_surface_normal(const Triangle& triangle) noexcept;
static Vector unit_surface_normal(const Sphere& sphere, const Vector& point) noexcept;
static Vector unit_surface_normal(const Ellipsoid& ellipsoid, const Vector& point) noexcept;

#endif
