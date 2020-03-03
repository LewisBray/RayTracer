#include "test_utils.h"

bool areEqual(const Vector& lhs, const Vector& rhs) noexcept
{
    return areEqual(lhs.x, rhs.x) && areEqual(lhs.y, rhs.y) &&
        areEqual(lhs.z, rhs.z) && areEqual(lhs.w, rhs.w);
}
