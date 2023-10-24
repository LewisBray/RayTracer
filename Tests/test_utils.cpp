static bool are_equal(const Vector& lhs, const Vector& rhs) noexcept {
    return are_equal(lhs.x, rhs.x) && are_equal(lhs.y, rhs.y) && are_equal(lhs.z, rhs.z);
}

static bool are_equal(const Matrix& lhs, const Matrix& rhs) noexcept {
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t column = 0; column < 4; ++column) {
            if (!are_equal(lhs.rows[row][column], rhs.rows[row][column])) {
                return false;
            }
        }
    }
    
    return true;
}
