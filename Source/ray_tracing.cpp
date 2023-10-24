#include <cassert>
#include <limits>

#include "ray_tracing.h"
#include "maths.h"

// Intersection functions
static float intersect(const Ray& ray, const Triangle& triangle) noexcept {
    TIME_BLOCK("intersect triangle");

    assert(are_equal(magnitude(ray.direction), 1.0f));

    const Vector a_to_b = triangle.b - triangle.a;
    const Vector a_to_c = triangle.c - triangle.a;

    const Vector plane_normal = a_to_b ^ a_to_c;
    if (are_equal(ray.direction * plane_normal, 0.0f)) {  // Ray and plane are parallel
        return FLT_MAX;
    }
    
    const float intersection_distance = ((triangle.a - ray.start) * plane_normal) / (ray.direction * plane_normal);
    if (intersection_distance < tolerance) {    // i.e. <= 0 with some tolerance
        return FLT_MAX;
    }
    
    const Vector intersection_point = ray.start + intersection_distance * ray.direction;
    const Vector a_to_intersection = intersection_point - triangle.a;

    const float plane_normal_magnitude_squared = plane_normal * plane_normal;
    const float alpha = ((a_to_b ^ a_to_intersection) * plane_normal) / plane_normal_magnitude_squared;
    const float beta = ((a_to_intersection ^ a_to_c) * plane_normal) / plane_normal_magnitude_squared;

    if (alpha < 0.0f || beta < 0.0f || alpha + beta > 1.0f) {
        return FLT_MAX;
    }

    return intersection_distance;
}

struct SphereIntersectionResult {
    float min_distance;
    float max_distance;
    bool is_valid;
};

static SphereIntersectionResult intersect(const Ray& ray, const Sphere& sphere) noexcept {
    TIME_BLOCK("intersect sphere");

    assert(are_equal(magnitude(ray.direction), 1.0f));

    const Vector ray_start_to_sphere_centre = sphere.centre - ray.start;
    const float intersections_mid_point_distance = ray_start_to_sphere_centre * ray.direction;

    const float ray_start_to_sphere_centre_distance_squared = ray_start_to_sphere_centre * ray_start_to_sphere_centre;
    const float intersections_mid_point_distance_squared = intersections_mid_point_distance * intersections_mid_point_distance;
    const float sphere_centre_to_intersections_mid_point_squared = ray_start_to_sphere_centre_distance_squared - intersections_mid_point_distance_squared;
    const float sphere_radius_squared = sphere.radius * sphere.radius;
    const float intersections_mid_point_to_intersections_distance_squared = sphere_radius_squared - sphere_centre_to_intersections_mid_point_squared;
    if (intersections_mid_point_to_intersections_distance_squared < 0.0f) {
        return SphereIntersectionResult{};
    }

    const float intersections_mid_point_to_intersections_distance = fp_sqrt(intersections_mid_point_to_intersections_distance_squared);
    SphereIntersectionResult result = {};
    result.min_distance = intersections_mid_point_distance - intersections_mid_point_to_intersections_distance;
    result.max_distance = intersections_mid_point_distance + intersections_mid_point_to_intersections_distance;
    result.is_valid = true;

    return result;
}

static SphereIntersectionResult intersect_with_unit_sphere(const Ray& ray) noexcept {
    TIME_BLOCK("intersect ellipsoid");

    assert(are_equal(magnitude(ray.direction), 1.0f));

    const float intersections_mid_point_distance = -1.0f * ray.start * ray.direction;

    const float ray_start_magnitude_squared = ray.start * ray.start;
    const float intersections_mid_point_distance_squared = intersections_mid_point_distance * intersections_mid_point_distance;
    const float sphere_centre_to_intersections_mid_point_squared = ray_start_magnitude_squared - intersections_mid_point_distance_squared;
    const float intersections_mid_point_to_intersections_distance_squared = 1.0f - sphere_centre_to_intersections_mid_point_squared;
    if (intersections_mid_point_to_intersections_distance_squared < 0.0f) {
        return SphereIntersectionResult{};
    }

    const float intersections_mid_point_to_intersections_distance = fp_sqrt(intersections_mid_point_to_intersections_distance_squared);
    SphereIntersectionResult result = {};
    result.min_distance = intersections_mid_point_distance - intersections_mid_point_to_intersections_distance;
    result.max_distance = intersections_mid_point_distance + intersections_mid_point_to_intersections_distance;
    result.is_valid = true;

    return result;
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

static Vector transform_direction(const Matrix& m, const Vector& v) noexcept {
    return Vector {
        m.rows[0][0] * v.x + m.rows[0][1] * v.y + m.rows[0][2] * v.z,
        m.rows[1][0] * v.x + m.rows[1][1] * v.y + m.rows[1][2] * v.z,
        m.rows[2][0] * v.x + m.rows[2][1] * v.y + m.rows[2][2] * v.z
    };
}

// Ray helper methods
static Ray transform_ray(const Matrix& transform, Ray ray) noexcept {
    ray.start = transform * ray.start;
    ray.direction = transform_direction(transform, ray.direction);

    return Ray{ray.start, normalise(ray.direction)};
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

// Light source path checking functions
static bool path_is_blocked(const Vector& start, const PointLightSource& light, const Scene& scene) noexcept {
    TIME_BLOCK("path is blocked (point lights)");

    const Vector start_to_light = light.position - start;
    const float distance_to_light = magnitude(start_to_light);
    const Ray ray{start, start_to_light / distance_to_light};

    for (const Triangle& triangle : scene.triangles) {
        const float intersection_distance = intersect(ray, triangle);
        if (intersection_distance < distance_to_light) {
            return true;
        }
    }

    for (const Sphere& sphere : scene.spheres) {
        const SphereIntersectionResult sphere_intersection = intersect(ray, sphere);
        if (sphere_intersection.is_valid) {
            const float min_distance = sphere_intersection.min_distance;
            const float max_distance = sphere_intersection.max_distance;
            if ((greater_than(min_distance, 0.0f) && less_than(min_distance, distance_to_light))
                || (greater_than(max_distance, 0.0f) && less_than(max_distance, distance_to_light))) {
                return true;
            }
        }
    }

    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
        const SphereIntersectionResult transformed_sphere_intersection = intersect_with_unit_sphere(transformed_ray);
        if (transformed_sphere_intersection.is_valid) {
            const Vector transformed_light_position = ellipsoid.inverse_transform * light.position;
            const float transformed_distance_to_light = magnitude(transformed_light_position - transformed_ray.start);

            const float transformed_min_distance = transformed_sphere_intersection.min_distance;
            const float transformed_max_distance = transformed_sphere_intersection.max_distance;

            if ((greater_than(transformed_min_distance, 0.0f) && less_than(transformed_min_distance, transformed_distance_to_light))
                || (greater_than(transformed_max_distance, 0.0f) && less_than(transformed_max_distance, transformed_distance_to_light))) {
                return true;
            }
        }
    }

    return false;
}

static bool path_is_blocked(const Vector& start, const DirectionalLightSource& light, const Scene& scene) noexcept {
    TIME_BLOCK("path is blocked (directional light)");

    const Ray ray{start, -1.0f * light.direction};

    for (const Triangle& triangle : scene.triangles) {
        const float intersection_distance = intersect(ray, triangle);
        if (intersection_distance < FLT_MAX) {
            return true;
        }
    }

    for (const Sphere& sphere : scene.spheres) {
        const SphereIntersectionResult sphere_intersection = intersect(ray, sphere);
        if (sphere_intersection.is_valid) {
            const float min_distance = sphere_intersection.min_distance;
            const float max_distance = sphere_intersection.max_distance;
            if (greater_than(min_distance, 0.0f) || greater_than(max_distance, 0.0f)) {
                return true;
            }
        }
    }

    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
        const SphereIntersectionResult transformed_sphere_intersection = intersect_with_unit_sphere(transformed_ray);
        if (transformed_sphere_intersection.is_valid) {
            const float transformed_min_distance = transformed_sphere_intersection.min_distance;
            const float transformed_max_distance = transformed_sphere_intersection.max_distance;
            if (greater_than(transformed_min_distance, 0.0f) || greater_than(transformed_max_distance, 0.0f)) {
                return true;
            }
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
        std::size_t closest_intersecting_triangle_index = INVALID_INDEX;
        float closest_triangle_intersection_distance = FLT_MAX;
        for (std::size_t i = 0; i < scene.triangles.size(); ++i) {
            const float intersection_distance = intersect(ray, scene.triangles[i]);
            if (intersection_distance < closest_triangle_intersection_distance) {
                closest_intersecting_triangle_index = i;
                closest_triangle_intersection_distance = intersection_distance;
            }
        }

        std::size_t closest_intersecting_sphere_index = INVALID_INDEX;
        float closest_sphere_intersection_distance = FLT_MAX;
        for (std::size_t i = 0; i < scene.spheres.size(); ++i) {
            const SphereIntersectionResult sphere_intersection = intersect(ray, scene.spheres[i]);
            if (sphere_intersection.is_valid) {
                const float min_distance = sphere_intersection.min_distance;
                const float max_distance = sphere_intersection.max_distance;

                const bool min_distance_is_positive = greater_than(min_distance, 0.0f);
                const bool max_distance_is_positive = greater_than(max_distance, 0.0f);

                float closest_intersection_distance = FLT_MAX;
                if (min_distance_is_positive) {
                    closest_intersection_distance = min_distance;
                } else if (max_distance_is_positive) {
                    closest_intersection_distance = max_distance;
                }

                if (less_than(closest_intersection_distance, closest_sphere_intersection_distance)) {
                    closest_intersecting_sphere_index = i;
                    closest_sphere_intersection_distance = closest_intersection_distance;
                }
            }
        }

        std::size_t closest_intersecting_ellipsoid_index = INVALID_INDEX;
        float closest_ellipsoid_intersection_distance = FLT_MAX;
        for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
            const Ellipsoid& ellipsoid = scene.ellipsoids[i];
            const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
            const SphereIntersectionResult transformed_sphere_intersection = intersect_with_unit_sphere(transformed_ray);
            if (transformed_sphere_intersection.is_valid) {
                const float transformed_min_distance = transformed_sphere_intersection.min_distance;
                const float transformed_max_distance = transformed_sphere_intersection.max_distance;

                const bool transformed_min_distance_is_positive = greater_than(transformed_min_distance, 0.0f);
                const bool transformed_max_distance_is_positive = greater_than(transformed_max_distance, 0.0f);

                assert(transformed_min_distance <= transformed_max_distance);
                float transformed_closest_intersection_distance = FLT_MAX;
                if (transformed_min_distance_is_positive) {
                    transformed_closest_intersection_distance = transformed_min_distance;
                } else if (transformed_max_distance_is_positive) {
                    transformed_closest_intersection_distance = transformed_max_distance;
                }

                if (transformed_closest_intersection_distance < FLT_MAX) {  // TODO: can this check be removed? i.e. adding inf to float produces inf? etc...
                    const Matrix& ellipsoid_transform = scene.ellipsoid_transforms[i];
                    const Vector transformed_intersection_point = transformed_ray.start + transformed_closest_intersection_distance * transformed_ray.direction;
                    const Vector intersection_point = ellipsoid_transform * transformed_intersection_point;
                    const float intersection_distance = magnitude(intersection_point - ray.start);
                    if (less_than(intersection_distance, closest_ellipsoid_intersection_distance)) {
                        closest_intersecting_ellipsoid_index = i;
                        closest_ellipsoid_intersection_distance = intersection_distance;
                    }
                }
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
                
                assert(index < scene.triangles.size());
                const Triangle& triangle = scene.triangles[index];
                surface_normal = unit_surface_normal(triangle);
                
                material = scene.triangle_materials[index];
            } else if (closest_sphere_intersection_distance - closest_triangle_intersection_distance <= tolerance && closest_sphere_intersection_distance - closest_ellipsoid_intersection_distance <= tolerance) {
                assert(closest_intersecting_sphere_index != INVALID_INDEX);
                const std::size_t index = closest_intersecting_sphere_index;

                intersection_point = ray.start + closest_sphere_intersection_distance * ray.direction;

                assert(index < scene.spheres.size());
                const Sphere& sphere = scene.spheres[index];
                surface_normal = unit_surface_normal(sphere, intersection_point);

                material = scene.sphere_materials[index];
            } else if (closest_ellipsoid_intersection_distance - closest_triangle_intersection_distance <= tolerance && closest_ellipsoid_intersection_distance - closest_sphere_intersection_distance <= tolerance) {
                assert(closest_intersecting_ellipsoid_index != INVALID_INDEX);
                const std::size_t index = closest_intersecting_ellipsoid_index;
                
                intersection_point = ray.start + closest_ellipsoid_intersection_distance * ray.direction;
                
                assert(index < scene.ellipsoids.size());
                const Ellipsoid& ellipsoid = scene.ellipsoids[index];
                surface_normal = unit_surface_normal(ellipsoid, intersection_point);
                
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
                    const float specular_intensity = std::pow(fp_max(surface_normal * half_angle, 0.0f), material.shininess);
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
                const float specular_intensity = std::pow(fp_max(surface_normal * half_angle, 0.0f), material.shininess);
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
