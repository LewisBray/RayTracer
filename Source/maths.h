#ifndef MATHS_H
#define MATHS_H

#include <ostream>
#include <array>

#define real double
#define stor stod

constexpr real pi = 3.14159265358979323846264;
real toRadians(real angleInDegrees) noexcept;

bool areEqual(real x, real y) noexcept;
bool lessThan(real x, real y) noexcept;
bool greaterThan(real x, real y) noexcept;

struct Vector
{
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
Matrix identityMatrix() noexcept;
Matrix scalingMatrix(real xScale, real yScale, real zScale) noexcept;
Matrix translationMatrix(real xOffset, real yOffset, real zOffset) noexcept;
Matrix rotationMatrix(real angle, real axisX, real axisY, real axisZ) noexcept;

#endif
