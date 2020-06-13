#include "maths.h"

#include <algorithm>
#include <iterator>
#include <ostream>
#include <cassert>
#include <array>
#include <cmath>

// Angle conversion
real toRadians(const real angleInDegrees) noexcept
{
    return angleInDegrees / 180.0 * pi;
}


// Floating point comparisons
constexpr real tolerance = 1.0e-6;

bool areEqual(const real x, const real y) noexcept
{
    return (std::abs(y - x) < tolerance);
}

bool lessThan(const real x, const real y) noexcept
{
    return (y - x > tolerance);
}

bool greaterThan(const real x, const real y) noexcept
{
    return (x - y > tolerance);
}


// Vector operations
Vector operator+(const Vector& lhs, const Vector& rhs) noexcept
{
    return Vector {
        lhs.x * rhs.w + rhs.x * lhs.w,
        lhs.y * rhs.w + rhs.y * lhs.w,
        lhs.z * rhs.w + rhs.z * lhs.w,
        lhs.w * rhs.w
    };
}

Vector operator-(const Vector& lhs, const Vector& rhs) noexcept
{
    return Vector {
        lhs.x * rhs.w - rhs.x * lhs.w,
        lhs.y * rhs.w - rhs.y * lhs.w,
        lhs.z * rhs.w - rhs.z * lhs.w,
        lhs.w * rhs.w
    };
}

real operator*(const Vector& lhs, const Vector& rhs) noexcept
{
    return (lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z) / (lhs.w * rhs.w);
}

Vector operator*(const real scalar, const Vector& v) noexcept
{
    if (scalar == 0.0)
        return Vector{0.0, 0.0, 0.0, 1.0};
    
    return Vector{v.x, v.y,v.z, v.w / scalar};
}

Vector operator^(const Vector& lhs, const Vector& rhs) noexcept
{
    return Vector {
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x,
        lhs.w * rhs.w
    };
}

Vector operator/(const Vector& v, const real scalar) noexcept
{
    assert(scalar != 0.0);
    return Vector{v.x, v.y, v.z, scalar * v.w};
}

// Likely just used for debugging and can probably be removed later
std::ostream& operator<<(std::ostream& out, const Vector& v)
{
    out << '(' << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ')';
    return out;
}

real magnitude(const Vector& v)
{
    return std::sqrt(v * v);
}

Vector normalise(const Vector& v)
{
    return v / magnitude(v);
}

Vector homogenise(const Vector& v) noexcept
{
    assert(v.w != 0.0);
    return Vector{v.x / v.w, v.y / v.w, v.z / v.w, 1.0};
}


// Matrix operations
Vector operator*(const Matrix& m, const Vector& v) noexcept
{
    return Vector {
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3] * v.w,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3] * v.w,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3] * v.w,
        m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3] * v.w
    };
}

Matrix operator*(const Matrix& lhs, const Matrix& rhs) noexcept
{
    Matrix ret;
    for (std::array<real, 4>& row : ret)
        std::fill(row.begin(), row.end(), 0.0);

    for (std::size_t i = 0; i < 4; ++i)
        for (std::size_t j = 0; j < 4; ++j)
            for (std::size_t k = 0; k < 4; ++k)
                ret[i][k] += lhs[i][j] * rhs[j][k];
    
    return ret;
}

Matrix identityMatrix() noexcept
{
    return Matrix {
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0
    };
}

Matrix scalingMatrix(const real xScale, const real yScale, const real zScale) noexcept
{
    Matrix ret;
    for (std::array<real, 4>& row : ret)
        std::fill(row.begin(), row.end(), 0.0);

    ret[0][0] = xScale;
    ret[1][1] = yScale;
    ret[2][2] = zScale;
    ret[3][3] = 1.0;

    return ret;
}

Matrix translationMatrix(const real xOffset, const real yOffset, const real zOffset) noexcept
{
    Matrix ret;
    for (std::array<real, 4>& row : ret)
        std::fill(row.begin(), row.end(), 0.0);
    
    ret[0][3] = xOffset;
    ret[1][3] = yOffset;
    ret[2][3] = zOffset;
    for (int i = 0; i < 4; ++i)
        ret[i][i] = 1.0;

    return ret;
}

Matrix rotationMatrix(const real angle, const real axisX, const real axisY, const real axisZ) noexcept
{
    const Vector rotationVector = homogenise(normalise(Vector{axisX, axisY, axisZ, 1.0}));
    const real sinAngle = std::sin(angle);
    const real cosAngle = std::cos(angle);

    Matrix ret;
    ret[0][0] = (1.0 - cosAngle) * rotationVector.x * rotationVector.x + cosAngle;
    ret[0][1] = (1.0 - cosAngle) * rotationVector.x * rotationVector.y - sinAngle * rotationVector.z;
    ret[0][2] = (1.0 - cosAngle) * rotationVector.x * rotationVector.z + sinAngle * rotationVector.y;
    ret[0][3] = 0.0;

    ret[1][0] = (1.0 - cosAngle) * rotationVector.x * rotationVector.y + sinAngle * rotationVector.z;
    ret[1][1] = (1.0 - cosAngle) * rotationVector.y * rotationVector.y + cosAngle;
    ret[1][2] = (1.0 - cosAngle) * rotationVector.y * rotationVector.z - sinAngle * rotationVector.x;
    ret[1][3] = 0.0;

    ret[2][0] = (1.0 - cosAngle) * rotationVector.x * rotationVector.z - sinAngle * rotationVector.y;
    ret[2][1] = (1.0 - cosAngle) * rotationVector.y * rotationVector.z + sinAngle * rotationVector.x;
    ret[2][2] = (1.0 - cosAngle) * rotationVector.z * rotationVector.z + cosAngle;
    ret[2][3] = 0.0;

    ret[3][0] = 0.0;
    ret[3][1] = 0.0;
    ret[3][2] = 0.0;
    ret[3][3] = 1.0;

    return ret;
}


// Shape operations
Vector unitSurfaceNormal(const Triangle& triangle) noexcept
{
    return normalise((triangle.b - triangle.a) ^ (triangle.c - triangle.a));
}

Vector unitSurfaceNormal(const Sphere& sphere, const Vector& point) noexcept
{
    return normalise(point - sphere.centre);
}

Vector unitSurfaceNormal(const Ellipsoid& ellipsoid,
    const Matrix& ellipsoidTransform, const Vector& point) noexcept
{
    const Vector pointInEllipsoidSpace = ellipsoid.inverseTransform * point;
    
    Vector surfaceNormal = homogenise(unitSurfaceNormal(ellipsoid.sphere, pointInEllipsoidSpace));
    surfaceNormal.w = 0.0;
    surfaceNormal = ellipsoidTransform * surfaceNormal;
    surfaceNormal.w = 1.0;

    return normalise(surfaceNormal);
}
