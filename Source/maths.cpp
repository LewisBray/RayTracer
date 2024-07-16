#include "maths.h"

#include <cassert>
#include <cmath>

#include <immintrin.h>

// Floating point util routines
static int get_exponent_127(const float x) {
    auto* const px = reinterpret_cast<const unsigned char*>(&x);

    int exponent = 0;
    exponent |= (px[2] & 0x80) >> 7;
    exponent |= (px[3] & 0x7F) << 1;
    
    return exponent;
}

// calculates log2 : (0, 1] -> (-inf, 0], does not handle subnormal numbers
static float fp_log2(const float x) {
    assert(0.0f < x && x <= 1.0f);
    
    // since floating point numbers are of the form sign * mantissa * 2^exponent and we
    // are only dealing with positive numbers, we can say log2(x) = log2(mantissa) + exponent
    // where mantissa is in [1, 2) and exponent is in [-126, 127]. Therefore, we can use a
    // polynomial approximation of log2 on [1, 2] to calculate the answer
    
    // extract exponent for addition at the end
    const int exponent_127 = get_exponent_127(x);
    assert(exponent_127 != 0);  // i.e. is not subnormal
    const int exponent = exponent_127 - 127;
    assert(exponent <= 0);
    
    // pull out mantissa by taking x and setting exponent to be 0
    float mantissa = x;
    auto* const p_mantissa = reinterpret_cast<unsigned char*>(&mantissa);
    p_mantissa[2] |= 0x80;
    p_mantissa[3] = 0x3F;
    
    // minimax approximation of log2 on [1, 2], max error 1.2538734892163924e-05
    static constexpr float MINIMAX_APPROXIMATION[] = {
        -2.80036404536401f,
        5.0917108473873602f,
        -3.5507929732452235f,
        1.6311487949127739f,
        -0.41656369634969576f,
        0.044873611393688138f
    };
    
    float log2_mantissa = 0.0f;
    float multiplier = 1.0f;
    for (const float coefficient : MINIMAX_APPROXIMATION) {
        log2_mantissa += coefficient * multiplier;
        multiplier *= mantissa;
    }
    
    float result = log2_mantissa + static_cast<float>(exponent);
    result = fp_min(result, 0.0f);  // handle numbers slightly > 0 due to floating point inaccuracies
    return result;
}

// calculates exp2 : (-inf, 0]
static float fp_exp2(const float x) {
    assert(x <= 0.0f);
    
    // since we know x is non-positive, 2^x = 1/2^|x| = 1/(2^(i + f)) = 1/2^i * 1/2^f
    // where i and f are the integer and fractional part of |x|, respectively. We can
    // use the reprsentation of IEEE floating point to due 2^i as this is equivalent
    // to setting the exponential part to be i (after offsetting) and we can use a
    // polynomial approximation for exp2 on [0, 1] to finish calculating the answer
    
    // clamp x so that we can set the exponent in 2^i, floating point numbers won't
    // handle anymore than an exponent of 127 so enforce that here, if x is any smaller
    // than -127 it makes very little difference to the answer as 2^x approaches 0 quickly
    const float clamped_x = fp_max(x, -127.0f);
    const float abs_x = fp_abs(clamped_x);
    const int int_part = static_cast<int>(abs_x);
    const float frac_part = abs_x - static_cast<float>(int_part);
    
    // set exponent to be int part of x
    float exp2_inv_int_part = 0.0f;
    unsigned char* const p_exp2_inv_int_part = reinterpret_cast<unsigned char*>(&exp2_inv_int_part);
    const auto exp2_inv_int_part_exponent_127 = static_cast<unsigned char>(-int_part + 127);
    p_exp2_inv_int_part[2] = (exp2_inv_int_part_exponent_127 & 0x01) << 7;
    p_exp2_inv_int_part[3] = (exp2_inv_int_part_exponent_127 & 0xFE) >> 1;
    
    // minimax approximation of 1/(2^x) on [0, 1], max error 5.3517174626820941e-05
    static constexpr float MINIMAX_APPROXIMATION[] = {
        0.99994648282537313f,
        -0.69137342261578116f,
        0.23097554273105286f,
        -0.039602120115271706f
    };
    
    float exp2_inv_frac_part = 0.0f;
    float multiplier = 1.0f;
    for (const float coefficient : MINIMAX_APPROXIMATION) {
        exp2_inv_frac_part += coefficient * multiplier;
        multiplier *= frac_part;
    }
    
    const float result = exp2_inv_int_part * exp2_inv_frac_part;
    return result;
}

static int is_zero_mask(const int x) {
    const int x_is_negative = x >> 31;
    const int x_is_positive = -x >> 31;
    const int x_is_non_zero = x_is_positive | x_is_negative;
    return ~x_is_non_zero;
}

// pow : [0, 1] x [0, inf) -> [0, 1], pow(x, y) = exp2(y * log2(x)), pow(x, 0) = 1,
// therefore we need, log2 : [0, 1] -> (-inf, 0] and exp2 : (-inf, 0] -> [0, 1]
static float fp_pow(const float x, const float y) {
    assert(0.0f <= x && x <= 1.0f);
    assert(0.0f <= y);
    
    // we will say subnormal values are ~0 and handle accordingly
    const int exponent_127_x = get_exponent_127(x);
    const int x_is_subnormal = is_zero_mask(exponent_127_x);
    const int exponent_127_y = get_exponent_127(y);
    const int y_is_subnormal = is_zero_mask(exponent_127_y);
    
    // currently branching here but the plan is to SIMD this in the future so leaving for now
    float result = 0.0f;
    if (y_is_subnormal) {
        result = 1.0f;
    } else if (x_is_subnormal) {
        result = 0.0f;
    } else {
        const float log2_x = fp_log2(x);
        const float z = y * log2_x;
        result = fp_exp2(z);
    }
    
    assert(0.0f <= result && result <= 1.0f);
    return result;
}

static float fp_sqrt(const float x) {
    const __m128 x128 = _mm_load_ps1(&x);
    const __m128 sqrt_x128 = _mm_sqrt_ps(x128);
    return _mm_cvtss_f32(sqrt_x128);
}

static float fp_min(const float x, const float y) {
    const __m128 x128 = _mm_load_ps1(&x);
    const __m128 y128 = _mm_load_ps1(&y);
    const __m128 z = _mm_min_ss(x128, y128);
    return _mm_cvtss_f32(z);
}

static float fp_max(const float x, const float y) {
    const __m128 x128 = _mm_load_ps1(&x);
    const __m128 y128 = _mm_load_ps1(&y);
    const __m128 z = _mm_max_ss(x128, y128);
    return _mm_cvtss_f32(z);
}

static float fp_abs(float x) {
    auto* const p = reinterpret_cast<unsigned char*>(&x);
    p[3] &= 0x7f;
    return x;
}

static bool are_equal(const float x, const float y) noexcept {
    return (fp_abs(y - x) < tolerance);
}

static bool less_than(const float x, const float y) noexcept {
    return (y - x > tolerance);
}

static bool greater_than(const float x, const float y) noexcept {
    return (x - y > tolerance);
}


// Angle conversion
static float to_radians(const float angle_in_degrees) noexcept {
    return angle_in_degrees / 180.0f * PI;
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
    return fp_sqrt(v * v);
}

static Vector normalise(const Vector& v) {
    return v / magnitude(v);
}


// Matrix operations
static Vector operator*(const Matrix& m, const Vector& v) noexcept {
    return Vector {
        m.rows[0][0] * v.x + m.rows[0][1] * v.y + m.rows[0][2] * v.z + m.rows[0][3],
        m.rows[1][0] * v.x + m.rows[1][1] * v.y + m.rows[1][2] * v.z + m.rows[1][3],
        m.rows[2][0] * v.x + m.rows[2][1] * v.y + m.rows[2][2] * v.z + m.rows[2][3]
    };
}

static Matrix operator*(const Matrix& lhs, const Matrix& rhs) noexcept {
    Matrix ret;
    ret.rows[0][0] = lhs.rows[0][0] * rhs.rows[0][0] + lhs.rows[0][1] * rhs.rows[1][0] + lhs.rows[0][2] * rhs.rows[2][0];
    ret.rows[0][1] = lhs.rows[0][0] * rhs.rows[0][1] + lhs.rows[0][1] * rhs.rows[1][1] + lhs.rows[0][2] * rhs.rows[2][1];
    ret.rows[0][2] = lhs.rows[0][0] * rhs.rows[0][2] + lhs.rows[0][1] * rhs.rows[1][2] + lhs.rows[0][2] * rhs.rows[2][2];
    ret.rows[0][3] = lhs.rows[0][0] * rhs.rows[0][3] + lhs.rows[0][1] * rhs.rows[1][3] + lhs.rows[0][2] * rhs.rows[2][3] + lhs.rows[0][3];

    ret.rows[1][0] = lhs.rows[1][0] * rhs.rows[0][0] + lhs.rows[1][1] * rhs.rows[1][0] + lhs.rows[1][2] * rhs.rows[2][0];
    ret.rows[1][1] = lhs.rows[1][0] * rhs.rows[0][1] + lhs.rows[1][1] * rhs.rows[1][1] + lhs.rows[1][2] * rhs.rows[2][1];
    ret.rows[1][2] = lhs.rows[1][0] * rhs.rows[0][2] + lhs.rows[1][1] * rhs.rows[1][2] + lhs.rows[1][2] * rhs.rows[2][2];
    ret.rows[1][3] = lhs.rows[1][0] * rhs.rows[0][3] + lhs.rows[1][1] * rhs.rows[1][3] + lhs.rows[1][2] * rhs.rows[2][3] + lhs.rows[1][3];

    ret.rows[2][0] = lhs.rows[2][0] * rhs.rows[0][0] + lhs.rows[2][1] * rhs.rows[1][0] + lhs.rows[2][2] * rhs.rows[2][0];
    ret.rows[2][1] = lhs.rows[2][0] * rhs.rows[0][1] + lhs.rows[2][1] * rhs.rows[1][1] + lhs.rows[2][2] * rhs.rows[2][1];
    ret.rows[2][2] = lhs.rows[2][0] * rhs.rows[0][2] + lhs.rows[2][1] * rhs.rows[1][2] + lhs.rows[2][2] * rhs.rows[2][2];
    ret.rows[2][3] = lhs.rows[2][0] * rhs.rows[0][3] + lhs.rows[2][1] * rhs.rows[1][3] + lhs.rows[2][2] * rhs.rows[2][3] + lhs.rows[2][3];
    
    return ret;
}

static Matrix identity_matrix() noexcept {
    return Matrix {
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f
    };
}

static Matrix scaling_matrix(const float x_scale, const float y_scale, const float z_scale) noexcept {
    return Matrix {
        x_scale,    0.0f,    0.0f,    0.0f,
        0.0f,    y_scale,    0.0f,    0.0f,
        0.0f,       0.0f, z_scale,    0.0f
    };
}

static Matrix translation_matrix(const float x_offset, const float y_offset, const float z_offset) noexcept {
    return Matrix {
        1.0f, 0.0f, 0.0f, x_offset,
        0.0f, 1.0f, 0.0f, y_offset,
        0.0f, 0.0f, 1.0f, z_offset
    };
}

static Matrix rotation_matrix(const float angle, const float axis_x, const float axis_y, const float axis_z) noexcept {
    const Vector rotation_vector = normalise(Vector{axis_x, axis_y, axis_z});
    const float sin_angle = std::sin(angle);
    const float cos_angle = std::cos(angle);

    Matrix ret;
    ret.rows[0][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.x + cos_angle;
    ret.rows[0][1] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.y - sin_angle * rotation_vector.z;
    ret.rows[0][2] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.z + sin_angle * rotation_vector.y;
    ret.rows[0][3] = 0.0f;

    ret.rows[1][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.y + sin_angle * rotation_vector.z;
    ret.rows[1][1] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.y + cos_angle;
    ret.rows[1][2] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.z - sin_angle * rotation_vector.x;
    ret.rows[1][3] = 0.0f;

    ret.rows[2][0] = (1.0f - cos_angle) * rotation_vector.x * rotation_vector.z - sin_angle * rotation_vector.y;
    ret.rows[2][1] = (1.0f - cos_angle) * rotation_vector.y * rotation_vector.z + sin_angle * rotation_vector.x;
    ret.rows[2][2] = (1.0f - cos_angle) * rotation_vector.z * rotation_vector.z + cos_angle;
    ret.rows[2][3] = 0.0f;

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
        m.rows[0][0] * v.x + m.rows[1][0] * v.y + m.rows[2][0] * v.z,
        m.rows[0][1] * v.x + m.rows[1][1] * v.y + m.rows[2][1] * v.z,
        m.rows[0][2] * v.x + m.rows[1][2] * v.y + m.rows[2][2] * v.z
    };
}

static Vector unit_surface_normal(const Matrix& ellipsoid_inverse_transform, const Vector& point) noexcept {
    const Vector point_in_ellipsoid_space = ellipsoid_inverse_transform * point;
    const Vector unit_surface_normal_in_ellipsoid_space = normalise(point_in_ellipsoid_space);  // centre at origin and radius 1 in ellipsoid space
    const Vector surface_normal = transform_direction_by_transpose(ellipsoid_inverse_transform, unit_surface_normal_in_ellipsoid_space);

    return normalise(surface_normal);
}
