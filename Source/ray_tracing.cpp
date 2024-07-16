#include <cassert>
#include <limits>

#include "ray_tracing.h"
#include "maths.h"

#include <immintrin.h>

static __m256 abs(const __m256& x) {
    const __m256i maski = _mm256_set1_epi32(0x7FFFFFFF);
    const __m256 mask = _mm256_castsi256_ps(maski);
    
    const __m256 result = _mm256_and_ps(x, mask);
    return result;
}

struct Vec3AVX {
    __m256 x;
    __m256 y;
    __m256 z;
};

static Vec3AVX load(const Vector& v) {
    Vec3AVX result = {};
    result.x = _mm256_set1_ps(v.x);
    result.y = _mm256_set1_ps(v.y);
    result.z = _mm256_set1_ps(v.z);
    
    return result;
}

static Vec3AVX load(const Vec3x8& v) {
    Vec3AVX result = {};
    result.x = _mm256_loadu_ps(v.x);
    result.y = _mm256_loadu_ps(v.y);
    result.z = _mm256_loadu_ps(v.z);
    
    return result;
}

static Vec3AVX operator+(const Vec3AVX& lhs, const Vec3AVX& rhs) {
    Vec3AVX result = {};
    result.x = _mm256_add_ps(lhs.x, rhs.x);
    result.y = _mm256_add_ps(lhs.y, rhs.y);
    result.z = _mm256_add_ps(lhs.z, rhs.z);
    
    return result;
}

static Vec3AVX operator-(const Vec3AVX& lhs, const Vec3AVX& rhs) {
    Vec3AVX result = {};
    result.x = _mm256_sub_ps(lhs.x, rhs.x);
    result.y = _mm256_sub_ps(lhs.y, rhs.y);
    result.z = _mm256_sub_ps(lhs.z, rhs.z);
    
    return result;
}

static Vec3AVX operator*(const __m256& scalar, const Vec3AVX& v) {
    Vec3AVX result = {};
    result.x = _mm256_mul_ps(scalar, v.x);
    result.y = _mm256_mul_ps(scalar, v.y);
    result.z = _mm256_mul_ps(scalar, v.z);
    
    return result;
}

static __m256 operator*(const Vec3AVX& lhs, const Vec3AVX& rhs) {
    __m256 result = _mm256_mul_ps(lhs.x, rhs.x);
    result = _mm256_fmadd_ps(lhs.y, rhs.y, result);
    result = _mm256_fmadd_ps(lhs.z, rhs.z, result);

    return result;
}

static Vec3AVX operator^(const Vec3AVX& lhs, const Vec3AVX& rhs) {
    Vec3AVX result = {};
    result.x = _mm256_sub_ps(_mm256_mul_ps(lhs.y, rhs.z), _mm256_mul_ps(lhs.z, rhs.y));
    result.y = _mm256_sub_ps(_mm256_mul_ps(lhs.z, rhs.x), _mm256_mul_ps(lhs.x, rhs.z));
    result.z = _mm256_sub_ps(_mm256_mul_ps(lhs.x, rhs.y), _mm256_mul_ps(lhs.y, rhs.x));
    
    return result;
}

static __m256 magnitude(const Vec3AVX& v) {
    return _mm256_sqrt_ps(v * v);
}

struct Mat3x4AVX {
    __m256 rows[3][4];
};

// TODO: is it better to do operations with Mat3x4x8s and load avx registers when needed? maybe no machine code generated?
static Mat3x4AVX load(const Mat3x4x8& m) {
    Mat3x4AVX result = {};
    for (int row = 0; row < 3; ++row) {
        for (int column = 0; column < 4; ++column) {
            result.rows[row][column] = _mm256_loadu_ps(m.rows[row][column]);
        }
    }

    return result;
}

static Vec3AVX operator*(const Mat3x4AVX& m, const Vec3AVX& v) {
    Vec3AVX result = {};
    // result.x = m.rows[0][0] * v.x + m.rows[0][1] * v.y + m.rows[0][2] * v.z + m.rows[0][3];
    result.x = _mm256_mul_ps(m.rows[0][0], v.x);
    result.x = _mm256_fmadd_ps(m.rows[0][1], v.y, result.x);
    result.x = _mm256_fmadd_ps(m.rows[0][2], v.z, result.x);
    result.x = _mm256_add_ps(m.rows[0][3], result.x);

    // result.y = m.rows[1][0] * v.x + m.rows[1][1] * v.y + m.rows[1][2] * v.z + m.rows[1][3];
    result.y = _mm256_mul_ps(m.rows[1][0], v.x);
    result.y = _mm256_fmadd_ps(m.rows[1][1], v.y, result.y);
    result.y = _mm256_fmadd_ps(m.rows[1][2], v.z, result.y);
    result.y = _mm256_add_ps(m.rows[1][3], result.y);

    // result.z = m.rows[2][0] * v.x + m.rows[2][1] * v.y + m.rows[2][2] * v.z + m.rows[2][3];
    result.z = _mm256_mul_ps(m.rows[2][0], v.x);
    result.z = _mm256_fmadd_ps(m.rows[2][1], v.y, result.z);
    result.z = _mm256_fmadd_ps(m.rows[2][2], v.z, result.z);
    result.z = _mm256_add_ps(m.rows[2][3], result.z);

    return result;
}

// Intersection functions
static __m256 intersect(const Vec3AVX& ray_start, const Vec3AVX& ray_direction, const Triangle8& triangle8) {
    TIME_BLOCK("intersect triangle8");
    
    const Vec3AVX a = load(triangle8.a);
    const Vec3AVX a_to_b = load(triangle8.a_to_b);
    const Vec3AVX a_to_c = load(triangle8.a_to_c);
    
    const Vec3AVX plane_normal = a_to_b ^ a_to_c;
    const __m256 ray_direction_dot_plane_normal = ray_direction * plane_normal;
    
    const __m256 abs_ray_direction_dot_plane_normal = abs(ray_direction_dot_plane_normal);
    const __m256 cmp_tolerance = _mm256_set1_ps(tolerance);
    const __m256 ray_plane_parallel = _mm256_cmp_ps(abs_ray_direction_dot_plane_normal, cmp_tolerance, _CMP_LT_OS);
    
    const __m256 intersection_distance = _mm256_div_ps((a - ray_start) * plane_normal, ray_direction_dot_plane_normal);
    const __m256 intersection_distance_non_positive = _mm256_cmp_ps(intersection_distance, cmp_tolerance, _CMP_LT_OS);
    
    const Vec3AVX intersection_point = ray_start + intersection_distance * ray_direction;
    const Vec3AVX a_to_intersection = intersection_point - a;
    
    const __m256 plane_normal_magnitude_squared = plane_normal * plane_normal;
    const __m256 alpha = _mm256_div_ps((a_to_b ^ a_to_intersection) * plane_normal, plane_normal_magnitude_squared);
    const __m256 beta = _mm256_div_ps((a_to_intersection ^ a_to_c) * plane_normal, plane_normal_magnitude_squared);
    
    const __m256 zero = _mm256_setzero_ps();
    const __m256 one = _mm256_set1_ps(1.0f);
    const __m256 alpha_lt_0 = _mm256_cmp_ps(alpha, zero, _CMP_LT_OS);
    const __m256 beta_lt_0 = _mm256_cmp_ps(beta, zero, _CMP_LT_OS);
    const __m256 alpha_plus_beta_gt_1 = _mm256_cmp_ps(_mm256_add_ps(alpha, beta), one, _CMP_GT_OS);
    const __m256 not_barycentric = _mm256_or_ps(_mm256_or_ps(alpha_lt_0, beta_lt_0), alpha_plus_beta_gt_1);
    
    const __m256 infinity = _mm256_set1_ps(FLT_MAX);
    const __m256 remove_mask = _mm256_or_ps(_mm256_or_ps(ray_plane_parallel, intersection_distance_non_positive), not_barycentric);
    const __m256 result = _mm256_or_ps(_mm256_andnot_ps(remove_mask, intersection_distance), _mm256_and_ps(remove_mask, infinity));
    
    return result;
}

static __m256 intersect(const Vec3AVX& ray_start, const Vec3AVX& ray_direction, const Sphere8& sphere8) {
    TIME_BLOCK("intersect sphere8");

    const Vec3AVX centre = load(sphere8.centre);
    const __m256 radius = _mm256_loadu_ps(sphere8.radius);

    const Vec3AVX ray_start_to_sphere_centre = centre - ray_start;
    const __m256 intersections_mid_point_distance = ray_start_to_sphere_centre * ray_direction;

    const __m256 ray_start_to_sphere_centre_distance_squared = ray_start_to_sphere_centre * ray_start_to_sphere_centre;
    const __m256 intersections_mid_point_distance_squared = _mm256_mul_ps(intersections_mid_point_distance, intersections_mid_point_distance);
    const __m256 sphere_centre_to_intersections_mid_point_squared = _mm256_sub_ps(ray_start_to_sphere_centre_distance_squared, intersections_mid_point_distance_squared);
    const __m256 sphere_radius_squared = _mm256_mul_ps(radius, radius);
    const __m256 intersections_mid_point_to_intersections_distance_squared = _mm256_sub_ps(sphere_radius_squared, sphere_centre_to_intersections_mid_point_squared);

    const __m256 zero = _mm256_setzero_ps();
    const __m256 no_intersection = _mm256_cmp_ps(intersections_mid_point_to_intersections_distance_squared, zero, _CMP_LT_OS);

    const __m256 intersections_mid_point_to_intersections_distance = _mm256_sqrt_ps(intersections_mid_point_to_intersections_distance_squared);

    // results are invalid if <= 0, we want the minimum of the valid results
    const __m256 distance_0 = _mm256_sub_ps(intersections_mid_point_distance, intersections_mid_point_to_intersections_distance);
    const __m256 distance_1 = _mm256_add_ps(intersections_mid_point_distance, intersections_mid_point_to_intersections_distance);

    const __m256 cmp_tolerance = _mm256_set1_ps(tolerance);
    const __m256 distance_0_non_positive = _mm256_cmp_ps(distance_0, cmp_tolerance, _CMP_LT_OS);
    const __m256 distance_1_non_positive = _mm256_cmp_ps(distance_1, cmp_tolerance, _CMP_LT_OS);

    const __m256 distance_0_is_invalid = _mm256_or_ps(no_intersection, distance_0_non_positive);
    const __m256 distance_1_is_invalid = _mm256_or_ps(no_intersection, distance_1_non_positive);

    const __m256 infinity = _mm256_set1_ps(FLT_MAX);
    const __m256 positive_distance_0 = _mm256_or_ps(_mm256_andnot_ps(distance_0_is_invalid, distance_0), _mm256_and_ps(distance_0_is_invalid, infinity));
    const __m256 positive_distance_1 = _mm256_or_ps(_mm256_andnot_ps(distance_1_is_invalid, distance_0), _mm256_and_ps(distance_1_is_invalid, infinity));
    const __m256 result = _mm256_min_ps(positive_distance_0, positive_distance_1);

    return result;
}

static Vec3AVX transform_direction(const Mat3x4AVX& m, const Vec3AVX& v) {
    Vec3AVX result = {};

    // result.x = m.rows[0][0] * v.x + m.rows[0][1] * v.y + m.rows[0][2] * v.z,
    result.x = _mm256_mul_ps(m.rows[0][0], v.x);
    result.x = _mm256_fmadd_ps(m.rows[0][1], v.y, result.x);
    result.x = _mm256_fmadd_ps(m.rows[0][2], v.z, result.x);

    // result.y = m.rows[1][0] * v.x + m.rows[1][1] * v.y + m.rows[1][2] * v.z,
    result.y = _mm256_mul_ps(m.rows[1][0], v.x);
    result.y = _mm256_fmadd_ps(m.rows[1][1], v.y, result.y);
    result.y = _mm256_fmadd_ps(m.rows[1][2], v.z, result.y);

    // result.z = m.rows[2][0] * v.x + m.rows[2][1] * v.y + m.rows[2][2] * v.z
    result.z = _mm256_mul_ps(m.rows[2][0], v.x);
    result.z = _mm256_fmadd_ps(m.rows[2][1], v.y, result.z);
    result.z = _mm256_fmadd_ps(m.rows[2][2], v.z, result.z);
        
    return result;
}

static __m256 intersect_with_unit_sphere(const Vec3AVX& ray_start, const Vec3AVX& ray_direction) {
    TIME_BLOCK("intersect with unit sphere8");

    const __m256 minus_one = _mm256_set1_ps(-1.0f);
    const __m256 intersections_mid_point_distance = minus_one * ray_start * ray_direction;

    const __m256 ray_start_magnitude_squared = ray_start * ray_start;
    const __m256 intersections_mid_point_distance_squared = _mm256_mul_ps(intersections_mid_point_distance, intersections_mid_point_distance);
    const __m256 sphere_centre_to_intersections_mid_point_squared = _mm256_sub_ps(ray_start_magnitude_squared, intersections_mid_point_distance_squared);
    const __m256 one = _mm256_set1_ps(1.0f);
    const __m256 intersections_mid_point_to_intersections_distance_squared = _mm256_sub_ps(one, sphere_centre_to_intersections_mid_point_squared);

    const __m256 zero = _mm256_setzero_ps();
    const __m256 no_intersection = _mm256_cmp_ps(intersections_mid_point_to_intersections_distance_squared, zero, _CMP_LT_OS);

    const __m256 intersections_mid_point_to_intersections_distance = _mm256_sqrt_ps(intersections_mid_point_to_intersections_distance_squared);

    // results are invalid if <= 0, we want the minimum of the valid results
    const __m256 distance_0 = _mm256_sub_ps(intersections_mid_point_distance, intersections_mid_point_to_intersections_distance);
    const __m256 distance_1 = _mm256_add_ps(intersections_mid_point_distance, intersections_mid_point_to_intersections_distance);

    const __m256 cmp_tolerance = _mm256_set1_ps(tolerance);
    const __m256 distance_0_non_positive = _mm256_cmp_ps(distance_0, cmp_tolerance, _CMP_LT_OS);
    const __m256 distance_1_non_positive = _mm256_cmp_ps(distance_1, cmp_tolerance, _CMP_LT_OS);

    const __m256 distance_0_is_invalid = _mm256_or_ps(no_intersection, distance_0_non_positive);
    const __m256 distance_1_is_invalid = _mm256_or_ps(no_intersection, distance_1_non_positive);

    const __m256 infinity = _mm256_set1_ps(FLT_MAX);
    const __m256 positive_distance_0 = _mm256_or_ps(_mm256_andnot_ps(distance_0_is_invalid, distance_0), _mm256_and_ps(distance_0_is_invalid, infinity));
    const __m256 positive_distance_1 = _mm256_or_ps(_mm256_andnot_ps(distance_1_is_invalid, distance_1), _mm256_and_ps(distance_1_is_invalid, infinity));
    const __m256 intersection_distance = _mm256_min_ps(positive_distance_0, positive_distance_1);

    return intersection_distance;
}

static AABBIntersectionResult intersect(const Ray& ray, const AxisAlignedBoundingBox& aabb) noexcept {
    TIME_BLOCK("intersect aabb");
    
    static_assert(std::numeric_limits<float>::is_iec559);
    assert(are_equal(magnitude(ray.direction), 1.0f));  // ensures a finite intersection time

    const float inverse_x = 1.0f / ray.direction.x;
    const float min_x_plane_intersection_distance = (aabb.min_x - ray.start.x) * inverse_x;
    const float max_x_plane_intersection_distance = (aabb.max_x - ray.start.x) * inverse_x;

    const float min_x_intersection_distance = fp_min(min_x_plane_intersection_distance, max_x_plane_intersection_distance);
    const float max_x_intersection_distance = fp_max(min_x_plane_intersection_distance, max_x_plane_intersection_distance);

    const float inverse_y = 1.0f / ray.direction.y;
    const float min_y_plane_intersection_distance = (aabb.min_y - ray.start.y) * inverse_y;
    const float max_y_plane_intersection_distance = (aabb.max_y - ray.start.y) * inverse_y;

    const float min_y_intersection_distance = fp_min(min_y_plane_intersection_distance, max_y_plane_intersection_distance);
    const float max_y_intersection_distance = fp_max(min_y_plane_intersection_distance, max_y_plane_intersection_distance);

    const float inverse_z = 1.0f / ray.direction.z;
    const float min_z_plane_intersection_distance = (aabb.min_z - ray.start.z) * inverse_z;
    const float max_z_plane_intersection_distance = (aabb.max_z - ray.start.z) * inverse_z;

    const float min_z_intersection_distance = fp_min(min_z_plane_intersection_distance, max_z_plane_intersection_distance);
    const float max_z_intersection_distance = fp_max(min_z_plane_intersection_distance, max_z_plane_intersection_distance);

    AABBIntersectionResult result = {};
    result.min_distance = fp_max(min_z_intersection_distance, fp_max(min_y_intersection_distance, min_x_intersection_distance));
    result.max_distance = fp_min(max_z_intersection_distance, fp_min(max_y_intersection_distance, max_x_intersection_distance));

    return result;
}

// Colour operations
static Colour operator+(const Colour& lhs, const Colour& rhs) noexcept {
    return Colour{lhs.red + rhs.red, lhs.green + rhs.green, lhs.blue + rhs.blue};
}

static Colour operator*(const Colour& lhs, const Colour& rhs) noexcept {
    return Colour{lhs.red * rhs.red, lhs.green * rhs.green, lhs.blue * rhs.blue};
}

static Colour operator*(const float scalar, const Colour& colour) noexcept {
    return Colour{scalar * colour.red, scalar * colour.green, scalar * colour.blue};
}

static Colour& operator+=(Colour& lhs, const Colour& rhs) noexcept {
    lhs.red += rhs.red;
    lhs.green += rhs.green;
    lhs.blue += rhs.blue;

    return lhs;
}

static Colour& operator*=(Colour& lhs, const Colour& rhs) {
    lhs.red *= rhs.red;
    lhs.green *= rhs.green;
    lhs.blue *= rhs.blue;

    return lhs;
}

static float attenuation(const AttenuationParameters& attenuation_parameters, const float distance) noexcept {
    const float constant_term = attenuation_parameters.constant;
    const float linear_term = attenuation_parameters.linear * distance;
    const float quadratic_term = attenuation_parameters.quadratic * distance * distance;

    return 1.0f / (constant_term + linear_term + quadratic_term);
}

static __m256 ellipsoid_is_absent(const Vec3AVX& transformed_ray_direction) {
    const __m256 cmp_tolerance = _mm256_set1_ps(tolerance);
    const __m256 x_is_zero = _mm256_cmp_ps(transformed_ray_direction.x, cmp_tolerance, _CMP_LT_OS);
    const __m256 y_is_zero = _mm256_cmp_ps(transformed_ray_direction.y, cmp_tolerance, _CMP_LT_OS);
    const __m256 z_is_zero = _mm256_cmp_ps(transformed_ray_direction.z, cmp_tolerance, _CMP_LT_OS);
    
    const __m256 result = _mm256_and_ps(x_is_zero, _mm256_and_ps(y_is_zero, z_is_zero));
    return result;
}

// Light source path checking functions
static bool path_is_blocked(const Vector& start, const PointLightSource& light, const Scene& scene) noexcept {
    TIME_BLOCK("path is blocked (point lights)");

    const Vector start_to_light = light.position - start;
    const __m256 distance_to_light = _mm256_set1_ps(magnitude(start_to_light));
    const __m256 inverse_distance_to_light = _mm256_div_ps(_mm256_set1_ps(1.0f), distance_to_light);
    const Vec3AVX ray_start = load(start);
    const Vec3AVX ray_direction = inverse_distance_to_light * load(start_to_light);
    
    for (const Triangle8& triangle8 : scene.triangle8s) {
        const __m256 intersection_distance = intersect(ray_start, ray_direction, triangle8);
        const __m256 less_than = _mm256_cmp_ps(intersection_distance, distance_to_light, _CMP_LT_OS);
        const int triangle_blocks_path = _mm256_movemask_ps(less_than);
        if (triangle_blocks_path) {
            return true;
        }
    }

    for (const Sphere8& sphere8 : scene.sphere8s) {
        const __m256 intersection_distance = intersect(ray_start, ray_direction, sphere8);
        const __m256 less_than = _mm256_cmp_ps(intersection_distance, distance_to_light, _CMP_LT_OS);
        const int sphere_blocks_path = _mm256_movemask_ps(less_than);
        if (sphere_blocks_path) {
            return true;
        }
    }

    const __m256 infinity = _mm256_set1_ps(FLT_MAX);
    const Vec3AVX light_position = load(light.position);
    for (const Mat3x4x8& ellipsoid8_inverse_transform : scene.ellipsoid8_inverse_transforms) {
        const Mat3x4AVX ellipsoid_inverse_transform = load(ellipsoid8_inverse_transform);
        
        const Vec3AVX transformed_ray_start = ellipsoid_inverse_transform * ray_start;
        Vec3AVX transformed_ray_direction = transform_direction(ellipsoid_inverse_transform, ray_direction);
        
        const __m256 ellipsoid_absent = ellipsoid_is_absent(transformed_ray_direction);

        const __m256 transformed_ray_direction_magnitude = magnitude(transformed_ray_direction);
        const __m256 inverse_transformed_ray_direction_magnitude = _mm256_div_ps(_mm256_set1_ps(1.0f), transformed_ray_direction_magnitude);
        transformed_ray_direction = inverse_transformed_ray_direction_magnitude * transformed_ray_direction;
        
        const Vec3AVX transformed_light_position = ellipsoid_inverse_transform * light_position;
        const __m256 transformed_distance_to_light = magnitude(transformed_light_position - transformed_ray_start);
        
        __m256 transformed_intersection_distance = intersect_with_unit_sphere(transformed_ray_start, transformed_ray_direction);
        transformed_intersection_distance = _mm256_or_ps(
            _mm256_andnot_ps(ellipsoid_absent, transformed_intersection_distance),
            _mm256_and_ps(ellipsoid_absent, infinity)
        );

        const __m256 less_than = _mm256_cmp_ps(transformed_intersection_distance, transformed_distance_to_light, _CMP_LT_OS);
        const int ellipsoid_blocks_path = _mm256_movemask_ps(less_than);
        if (ellipsoid_blocks_path) {
            return true;
        }
    }

    return false;
}

static bool path_is_blocked(const Vector& start, const DirectionalLightSource& light, const Scene& scene) noexcept {
    TIME_BLOCK("path is blocked (directional light)");
    
    const Vec3AVX ray_start = load(start);
    const Vec3AVX ray_direction = load(-1.0f * light.direction);
    const __m256 infinity = _mm256_set1_ps(FLT_MAX);

    for (const Triangle8& triangle8 : scene.triangle8s) {
        const __m256 intersection_distance = intersect(ray_start, ray_direction, triangle8);
        const __m256 less_than = _mm256_cmp_ps(intersection_distance, infinity, _CMP_LT_OS);
        const int triangle_blocks_path = _mm256_movemask_ps(less_than);
        if (triangle_blocks_path) {
            return true;
        }
    }

    for (const Sphere8& sphere8 : scene.sphere8s) {
        const __m256 intersection_distance = intersect(ray_start, ray_direction, sphere8);
        const __m256 less_than = _mm256_cmp_ps(intersection_distance, infinity, _CMP_LT_OS);
        const int sphere_blocks_path = _mm256_movemask_ps(less_than);
        if (sphere_blocks_path) {
            return true;
        }
    }

    for (const Mat3x4x8& ellipsoid8_inverse_transform : scene.ellipsoid8_inverse_transforms) {
        const Mat3x4AVX ellipsoid_inverse_transform = load(ellipsoid8_inverse_transform);
        
        const Vec3AVX transformed_ray_start = ellipsoid_inverse_transform * ray_start;
        Vec3AVX transformed_ray_direction = transform_direction(ellipsoid_inverse_transform, ray_direction);
        
        const __m256 ellipsoid_absent = ellipsoid_is_absent(transformed_ray_direction);

        const __m256 transformed_ray_direction_magnitude = magnitude(transformed_ray_direction);
        const __m256 inverse_transformed_ray_direction_magnitude = _mm256_div_ps(_mm256_set1_ps(1.0f), transformed_ray_direction_magnitude);
        transformed_ray_direction = inverse_transformed_ray_direction_magnitude * transformed_ray_direction;
        
        __m256 transformed_intersection_distance = intersect_with_unit_sphere(transformed_ray_start, transformed_ray_direction);
        transformed_intersection_distance = _mm256_or_ps(
            _mm256_andnot_ps(ellipsoid_absent, transformed_intersection_distance),
            _mm256_and_ps(ellipsoid_absent, infinity)
        );

        const __m256 less_than = _mm256_cmp_ps(transformed_intersection_distance, infinity, _CMP_LT_OS);
        const int ellipsoid_blocks_path = _mm256_movemask_ps(less_than);
        if (ellipsoid_blocks_path) {
            return true;
        }
    }

    return false;
}

static Vector ray_direction_through_pixel(
    const unsigned pixel_x,
    const unsigned pixel_y,
    const float x_offset,
    const float y_offset,
    const BasisVectors& camera_basis_vectors,
    const Dimensions& half_image_dimensions_world,
    const Dimensions& half_image_dimensions_pixels
) noexcept {
    const float alpha = half_image_dimensions_world.width * (half_image_dimensions_pixels.width - (static_cast<float>(pixel_x) + x_offset)) / half_image_dimensions_pixels.width;
    const float beta = half_image_dimensions_world.height * (half_image_dimensions_pixels.height - (static_cast<float>(pixel_y) + y_offset)) / half_image_dimensions_pixels.height;

    return normalise(alpha * camera_basis_vectors.i + beta * camera_basis_vectors.j + camera_basis_vectors.k);
}

static Colour intersect(Ray ray, const Scene& scene, const int max_bounce_count) noexcept {
    static constexpr std::size_t INVALID_INDEX = static_cast<std::size_t>(-1);
    
    Colour colour{0.0f, 0.0f, 0.0f};
    Colour colour_weighting{1.0f, 1.0f, 1.0f};
    for (int bounce_index = 0; bounce_index < max_bounce_count; ++bounce_index) {
        const Vec3AVX ray_start = load(ray.start);
        const Vec3AVX ray_direction = load(ray.direction);

        // intersect triangles
        __m256i closest_intersecting_triangle_indices = _mm256_set1_epi32(-1);
        __m256 closest_triangle_intersection_distances = _mm256_set1_ps(FLT_MAX);
        for (std::size_t i = 0; i < scene.triangle8s.size(); ++i) {
            const __m256 intersection_distances = intersect(ray_start, ray_direction, scene.triangle8s[i]);
            const __m256 less_than = _mm256_cmp_ps(intersection_distances, closest_triangle_intersection_distances, _CMP_LT_OS);
            closest_triangle_intersection_distances = _mm256_or_ps(
                _mm256_and_ps(less_than, intersection_distances),
                _mm256_andnot_ps(less_than, closest_triangle_intersection_distances)
            );
            
            const __m256i triangle_indices = _mm256_set1_epi32(i);
            const __m256i less_thani = _mm256_castps_si256(less_than);
            closest_intersecting_triangle_indices = _mm256_or_si256(
                _mm256_and_si256(less_thani, triangle_indices),
                _mm256_andnot_si256(less_thani, closest_intersecting_triangle_indices)
            );
        }

        std::size_t closest_intersecting_triangle_index = INVALID_INDEX;
        float closest_triangle_intersection_distance = FLT_MAX;
        
        float triangle8_distances[8] = {};
        _mm256_storeu_ps(triangle8_distances, closest_triangle_intersection_distances);
        unsigned int triangle8_indices[8] = {};
        _mm256_storeu_si256((__m256i*)triangle8_indices, closest_intersecting_triangle_indices);  // TODO: undefined behaviour?
        
        for (int j = 0; j < 8; ++j) {
            if (triangle8_distances[j] < closest_triangle_intersection_distance) {
                closest_intersecting_triangle_index = 8 * triangle8_indices[j] + j;
                closest_triangle_intersection_distance = triangle8_distances[j];
            }
        }
        
        // intersect spheres
        __m256i closest_intersecting_sphere_indices = _mm256_set1_epi32(-1);
        __m256 closest_sphere_intersection_distances = _mm256_set1_ps(FLT_MAX);
        for (std::size_t i = 0; i < scene.sphere8s.size(); ++i) {
            const __m256 intersection_distances = intersect(ray_start, ray_direction, scene.sphere8s[i]);
            const __m256 less_than = _mm256_cmp_ps(intersection_distances, closest_sphere_intersection_distances, _CMP_LT_OS);
            closest_sphere_intersection_distances = _mm256_or_ps(
                _mm256_and_ps(less_than, intersection_distances),
                _mm256_andnot_ps(less_than, closest_sphere_intersection_distances)
            );
            
            const __m256i sphere_indices = _mm256_set1_epi32(i);
            const __m256i less_thani = _mm256_castps_si256(less_than);
            closest_intersecting_sphere_indices = _mm256_or_si256(
                _mm256_and_si256(less_thani, sphere_indices),
                _mm256_andnot_si256(less_thani, closest_intersecting_sphere_indices)
            );
        }

        std::size_t closest_intersecting_sphere_index = INVALID_INDEX;
        float closest_sphere_intersection_distance = FLT_MAX;
        
        float sphere8_distances[8] = {};
        _mm256_storeu_ps(sphere8_distances, closest_sphere_intersection_distances);
        unsigned int sphere8_indices[8] = {};
        _mm256_storeu_si256((__m256i*)sphere8_indices, closest_intersecting_sphere_indices);  // TODO: undefined behaviour?
        
        for (int j = 0; j < 8; ++j) {
            if (sphere8_distances[j] < closest_sphere_intersection_distance) {
                closest_intersecting_sphere_index = 8 * sphere8_indices[j] + j;
                closest_sphere_intersection_distance = sphere8_distances[j];
            }
        }

        // intersect ellipsoids
        __m256i closest_intersecting_ellipsoid_indices = _mm256_set1_epi32(-1);
        __m256 closest_ellipsoid_intersection_distances = _mm256_set1_ps(FLT_MAX);
        for (std::size_t i = 0; i < scene.ellipsoid8_inverse_transforms.size(); ++i) {
            const Mat3x4AVX ellipsoid_inverse_transform = load(scene.ellipsoid8_inverse_transforms[i]);
            const Mat3x4AVX ellipsoid_transform = load(scene.ellipsoid8_transforms[i]);

            const Vec3AVX transformed_ray_start = ellipsoid_inverse_transform * ray_start;
            Vec3AVX transformed_ray_direction = transform_direction(ellipsoid_inverse_transform, ray_direction);
            
            const __m256 ellipsoid_absent = ellipsoid_is_absent(transformed_ray_direction);

            const __m256 transformed_ray_direction_magnitude = magnitude(transformed_ray_direction);
            const __m256 inverse_transformed_ray_direction_magnitude = _mm256_div_ps(_mm256_set1_ps(1.0f), transformed_ray_direction_magnitude);
            transformed_ray_direction = inverse_transformed_ray_direction_magnitude * transformed_ray_direction;

            const __m256 transformed_intersection_distances = intersect_with_unit_sphere(transformed_ray_start, transformed_ray_direction);
            
            const Vec3AVX transformed_intersection_point = transformed_ray_start + transformed_intersection_distances * transformed_ray_direction;
            const Vec3AVX intersection_point = ellipsoid_transform * transformed_intersection_point;
            
            const __m256 all_intersection_distances = magnitude(intersection_point - ray_start);
            
            const __m256 infinity = _mm256_set1_ps(FLT_MAX);
            const __m256 intersection_distances = _mm256_or_ps(
                _mm256_andnot_ps(ellipsoid_absent, all_intersection_distances),
                _mm256_and_ps(ellipsoid_absent, infinity)
            );

            const __m256 less_than = _mm256_cmp_ps(intersection_distances, closest_ellipsoid_intersection_distances, _CMP_LT_OS);
            closest_ellipsoid_intersection_distances = _mm256_or_ps(
                _mm256_and_ps(less_than, intersection_distances),
                _mm256_andnot_ps(less_than, closest_ellipsoid_intersection_distances)
            );
            
            const __m256i ellipsoid_indices = _mm256_set1_epi32(i);
            const __m256i less_thani = _mm256_castps_si256(less_than);
            closest_intersecting_ellipsoid_indices = _mm256_or_si256(
                _mm256_and_si256(less_thani, ellipsoid_indices),
                _mm256_andnot_si256(less_thani, closest_intersecting_ellipsoid_indices)
            );
        }

        std::size_t closest_intersecting_ellipsoid_index = INVALID_INDEX;
        float closest_ellipsoid_intersection_distance = FLT_MAX;
        
        float ellipsoid8_distances[8] = {};
        _mm256_storeu_ps(ellipsoid8_distances, closest_ellipsoid_intersection_distances);
        unsigned int ellipsoid8_indices[8] = {};
        _mm256_storeu_si256((__m256i*)ellipsoid8_indices, closest_intersecting_ellipsoid_indices);  // TODO: undefined behaviour?
        
        for (int j = 0; j < 8; ++j) {
            if (ellipsoid8_distances[j] < closest_ellipsoid_intersection_distance) {
                closest_intersecting_ellipsoid_index = 8 * ellipsoid8_indices[j] + j;
                closest_ellipsoid_intersection_distance = ellipsoid8_distances[j];
            }
        }

        if (closest_intersecting_triangle_index != INVALID_INDEX || closest_intersecting_sphere_index != INVALID_INDEX ||closest_intersecting_ellipsoid_index != INVALID_INDEX) {
            assert(closest_triangle_intersection_distance < FLT_MAX || closest_sphere_intersection_distance < FLT_MAX || closest_ellipsoid_intersection_distance < FLT_MAX);

            Material material{};
            Vector surface_normal{};
            Vector intersection_point{};
            if (closest_triangle_intersection_distance - closest_sphere_intersection_distance <= tolerance && closest_triangle_intersection_distance - closest_ellipsoid_intersection_distance <= tolerance) {
                assert(closest_intersecting_triangle_index != INVALID_INDEX);
                const std::size_t index = closest_intersecting_triangle_index;
                
                intersection_point = ray.start + closest_triangle_intersection_distance * ray.direction;
                
                const std::size_t batch_index = index / 8;
                const std::size_t instance_index = index % 8;
                
                assert(batch_index < scene.triangle8s.size());
                
                const Triangle8& triangle8 = scene.triangle8s[batch_index];
                const Vector a{triangle8.a.x[instance_index], triangle8.a.y[instance_index], triangle8.a.z[instance_index]};
                const Vector a_to_b{triangle8.a_to_b.x[instance_index], triangle8.a_to_b.y[instance_index], triangle8.a_to_b.z[instance_index]};
                const Vector a_to_c{triangle8.a_to_c.x[instance_index], triangle8.a_to_c.y[instance_index], triangle8.a_to_c.z[instance_index]};
                const Vector b = a_to_b + a;
                const Vector c = a_to_c + a;
                const Triangle triangle{a, b, c};
                surface_normal = unit_surface_normal(triangle);
                
                material = scene.triangle_materials[index];
            } else if (closest_sphere_intersection_distance - closest_triangle_intersection_distance <= tolerance && closest_sphere_intersection_distance - closest_ellipsoid_intersection_distance <= tolerance) {
                assert(closest_intersecting_sphere_index != INVALID_INDEX);
                const std::size_t index = closest_intersecting_sphere_index;

                intersection_point = ray.start + closest_sphere_intersection_distance * ray.direction;

                const std::size_t batch_index = index / 8;
                const std::size_t instance_index = index % 8;

                assert(batch_index < scene.sphere8s.size());
                
                const Sphere8& sphere8 = scene.sphere8s[batch_index];
                const Vector centre{sphere8.centre.x[instance_index], sphere8.centre.y[instance_index], sphere8.centre.z[instance_index]};
                const float radius = sphere8.radius[instance_index];
                const Sphere sphere{centre, radius};
                surface_normal = unit_surface_normal(sphere, intersection_point);

                material = scene.sphere_materials[index];
            } else if (closest_ellipsoid_intersection_distance - closest_triangle_intersection_distance <= tolerance && closest_ellipsoid_intersection_distance - closest_sphere_intersection_distance <= tolerance) {
                assert(closest_intersecting_ellipsoid_index != INVALID_INDEX);
                const std::size_t index = closest_intersecting_ellipsoid_index;
                
                intersection_point = ray.start + closest_ellipsoid_intersection_distance * ray.direction;
                
                const std::size_t batch_index = index / 8;
                const std::size_t instance_index = index % 8;
                
                assert(batch_index < scene.ellipsoid8_inverse_transforms.size());
                
                const Mat3x4x8& ellipsoid_inverse_transform8 = scene.ellipsoid8_inverse_transforms[batch_index];
                
                Matrix ellipsoid_inverse_transform = {};
                for (int row = 0; row < 3; ++row) {
                    for (int column = 0; column < 4; ++column) {
                        ellipsoid_inverse_transform.rows[row][column] = ellipsoid_inverse_transform8.rows[row][column][instance_index];
                    }
                }
                
                surface_normal = unit_surface_normal(ellipsoid_inverse_transform, intersection_point);
                
                material = scene.ellipsoid_materials[index];
            } else {
                assert(false);  // this shouldn't be possible
            }

            colour += colour_weighting * (scene.ambient + material.emission);

            const Vector direction_to_ray_start = -1.0f * ray.direction;
            const Vector epsilon_shift = 2.0f * tolerance * surface_normal;
            const Vector point_above_surface = intersection_point + epsilon_shift;
            if (scene.has_directional_light_source) {
                const DirectionalLightSource& directional_light_source = scene.directional_light_source;
                if (!path_is_blocked(point_above_surface, directional_light_source, scene)) {
                    const Vector direction_to_light = -1.0f * directional_light_source.direction;

                    const float diffuse_intensity = fp_max(surface_normal * direction_to_light, 0.0f);
                    const Colour diffuse_contribution = diffuse_intensity * material.diffuse;

                    const Vector half_angle = normalise(direction_to_ray_start + direction_to_light);
                    const float specular_intensity = fp_pow(fp_max(surface_normal * half_angle, 0.0f), material.shininess);
                    const Colour specular_contribution = specular_intensity * material.specular;

                    const Colour directional_light_contribution = directional_light_source.colour * (diffuse_contribution + specular_contribution);
                    colour += colour_weighting * directional_light_contribution;
                }
            }

            for (const PointLightSource& light : scene.point_light_sources) {
                if (path_is_blocked(point_above_surface, light, scene)) {
                    continue;
                }

                const Vector intersection_point_to_light = light.position - intersection_point;
                const float distance_to_light = magnitude(intersection_point_to_light);
                const Vector direction_to_light = intersection_point_to_light / distance_to_light;

                const float diffuse_intensity = fp_max(surface_normal * direction_to_light, 0.0f);
                const Colour diffuse_contribution = diffuse_intensity * material.diffuse;

                const Vector half_angle = normalise(direction_to_ray_start + direction_to_light);
                const float specular_intensity = fp_pow(fp_max(surface_normal * half_angle, 0.0f), material.shininess);
                const Colour specular_contribution = specular_intensity * material.specular;

                const float light_attenuation = attenuation(scene.attenuation_parameters, distance_to_light);

                const Colour point_light_contribution = light_attenuation * light.colour * (diffuse_contribution + specular_contribution);
                colour += colour_weighting * point_light_contribution;
            }

            colour_weighting *= material.specular;
            ray.start = point_above_surface;
            ray.direction = ray.direction - 2.0f * (ray.direction * surface_normal) * surface_normal;
        } else {
            break;
        }
    }

    return colour;
}
