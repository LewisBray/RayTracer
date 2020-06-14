#include "test_utils.h"

bool areEqual(const Vector& lhs, const Vector& rhs) noexcept
{
    return areEqual(lhs.x, rhs.x) && areEqual(lhs.y, rhs.y) &&
        areEqual(lhs.z, rhs.z) && areEqual(lhs.w, rhs.w);
}

bool areEqual(const Matrix& lhs, const Matrix& rhs) noexcept
{
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
            if (!areEqual(lhs[row][column], rhs[row][column]))
                return false;
    
    return true;
}
