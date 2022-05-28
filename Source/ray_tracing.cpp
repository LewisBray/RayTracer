#include "ray_tracing.h"
#include "maths.h"

#include <algorithm>
#include <optional>
#include <cassert>
#include <utility>
#include <limits>
#include <cmath>

// Intersection functions
std::optional<float> intersect(const Ray& ray, const Triangle& triangle) noexcept {
    const Vector a_to_b = triangle.b - triangle.a;
    const Vector a_to_c = triangle.c - triangle.a;

    const Vector plane_normal = a_to_b ^ a_to_c;
    if (are_equal(ray.direction * plane_normal, 0.0f)) {  // Ray and plane are parallel
        return std::nullopt;
    }
    
    const float intersection_distance = ((triangle.a - ray.start) * plane_normal) / (ray.direction * plane_normal);
    if (intersection_distance < tolerance) {    // i.e. <= 0 with some tolerance
        return std::nullopt;
    }
    
    const Vector intersection_point = ray.start + intersection_distance * ray.direction;
    const Vector a_to_intersection = intersection_point - triangle.a;

    const float plane_normal_magnitude_squared = plane_normal * plane_normal;
    const float alpha = ((a_to_b ^ a_to_intersection) * plane_normal) / plane_normal_magnitude_squared;
    const float beta = ((a_to_intersection ^ a_to_c) * plane_normal) / plane_normal_magnitude_squared;

    if (alpha < 0.0f || beta < 0.0f || alpha + beta > 1.0f) {
        return std::nullopt;
    }

    return intersection_distance;
}

std::optional<std::pair<float, float>> intersect(const Ray& ray, const Sphere& sphere) noexcept {
    assert(are_equal(magnitude(ray.direction), 1.0f));

    const Vector ray_start_to_sphere_centre = sphere.centre - ray.start;
    const float intersections_mid_point_distance = ray_start_to_sphere_centre * ray.direction;

    const float ray_start_to_sphere_centre_distance_squared = ray_start_to_sphere_centre * ray_start_to_sphere_centre;
    const float intersections_mid_point_distance_squared = intersections_mid_point_distance * intersections_mid_point_distance;
    const float sphere_centre_to_intersections_mid_point_squared = ray_start_to_sphere_centre_distance_squared - intersections_mid_point_distance_squared;
    const float sphere_radius_squared = sphere.radius * sphere.radius;
    if (sphere_centre_to_intersections_mid_point_squared > sphere_radius_squared) {
        return std::nullopt;
    }

    const float intersections_mid_point_to_intersections_distance = std::sqrt(sphere_radius_squared - sphere_centre_to_intersections_mid_point_squared);
    const float intersection_distance_1 = intersections_mid_point_distance - intersections_mid_point_to_intersections_distance;
    const float intersection_distance_2 = intersections_mid_point_distance + intersections_mid_point_to_intersections_distance;

    return std::pair{intersection_distance_1, intersection_distance_2};
}

static Vector transform_direction(const Matrix& m, const Vector& v) noexcept {
    return Vector {
        m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
        m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
        m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z
    };
}

// Ray helper methods
Ray transform_ray(const Matrix& transform, Ray ray) noexcept {
    ray.start = transform * ray.start;
    ray.direction = transform_direction(transform, ray.direction);

    return Ray{ray.start, normalise(ray.direction)};
}

// Colour operations
static Colour operator+(const Colour& lhs, const Colour& rhs) noexcept {
    return Colour{lhs.red + rhs.red, lhs.green + rhs.green, lhs.blue + rhs.blue};
}

static Colour& operator+=(Colour& lhs, const Colour& rhs) noexcept {
    lhs.red += rhs.red;
    lhs.green += rhs.green;
    lhs.blue += rhs.blue;

    return lhs;
}

static Colour operator*(const float scalar, const Colour& colour) noexcept {
    return Colour{scalar * colour.red, scalar * colour.green, scalar * colour.blue};
}

static Colour operator*(const Colour& lhs, const Colour& rhs) noexcept {
    return Colour{lhs.red * rhs.red, lhs.green * rhs.green, lhs.blue * rhs.blue};
}

// Attenuation functions
static float attenuation(const AttenuationParameters& attenuation_parameters, const float distance) noexcept {
    const float constant_term = attenuation_parameters.constant;
    const float linear_term = attenuation_parameters.linear * distance;
    const float quadratic_term = attenuation_parameters.quadratic * distance * distance;

    return 1.0f / (constant_term + linear_term + quadratic_term);
}

// Light source path checking functions
static bool path_is_blocked(const Vector& start, const PointLightSource& light, const Scene& scene) noexcept {
    const Vector start_to_destination = light.position - start;
    const float distance_to_destination = magnitude(start_to_destination);
    const Ray ray{start, start_to_destination / distance_to_destination};

    for (const Triangle& triangle : scene.triangles) {
        const std::optional<float> intersection_distance = intersect(ray, triangle);
        if (intersection_distance.has_value() && intersection_distance.value() < distance_to_destination) {
            return true;
        }
    }

    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
        const std::optional<std::pair<float, float>> transformed_intersection_distances = intersect(transformed_ray, ellipsoid.sphere);
        if (transformed_intersection_distances.has_value()) {
            const Vector transformed_light_position = ellipsoid.inverse_transform * light.position;
            const float transformed_distance_to_light = magnitude(transformed_light_position - transformed_ray.start);

            const float transformed_intersection_distance_1 = transformed_intersection_distances.value().first;
            const float transformed_intersection_distance_2 = transformed_intersection_distances.value().second;

            if ((greater_than(transformed_intersection_distance_1, 0.0f) && less_than(transformed_intersection_distance_1, transformed_distance_to_light))
                || (greater_than(transformed_intersection_distance_2, 0.0f) && less_than(transformed_intersection_distance_2, transformed_distance_to_light))) {
                return true;
            }
        }
    }

    return false;
}

static bool path_is_blocked(const Vector& start, const DirectionalLightSource& light, const Scene& scene) noexcept {
    const Ray ray{start, -1.0f * light.direction};

    for (const Triangle& triangle : scene.triangles) {
        const std::optional<float> intersection_distance = intersect(ray, triangle);
        if (intersection_distance.has_value()) {
            return true;
        }
    }

    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
        const std::optional<std::pair<float, float>> transformed_intersection_distances = intersect(transformed_ray, ellipsoid.sphere);
        if (transformed_intersection_distances.has_value()) {
            const float transformed_intersection_distance_1 = transformed_intersection_distances.value().first;
            const float transformed_intersection_distance_2 = transformed_intersection_distances.value().second;
            if (greater_than(transformed_intersection_distance_1, 0.0f) || greater_than(transformed_intersection_distance_2, 0.0f)) {
                return true;
            }
        }
    }

    return false;
}

// Exposed functions
Vector ray_direction_through_pixel(
    const int pixel_x,
    const int pixel_y,
    const BasisVectors& camera_basis_vectors,
    const Dimensions& half_image_dimensions_world,
    const Dimensions& half_image_dimensions_pixels
) noexcept {
    const float alpha = half_image_dimensions_world.width * (half_image_dimensions_pixels.width - (static_cast<float>(pixel_x) + 0.5f)) / half_image_dimensions_pixels.width;
    const float beta = half_image_dimensions_world.height * (half_image_dimensions_pixels.height - (static_cast<float>(pixel_y) + 0.5f)) / half_image_dimensions_pixels.height;

    return normalise(alpha * camera_basis_vectors.i + beta * camera_basis_vectors.j + camera_basis_vectors.k);
}

#include <iostream>

// TODO: Likely just used for debugging and can probably be removed later
std::ostream& operator<<(std::ostream& out, const Colour& colour) {
    out << '(' << colour.red << ", " << colour.green << ", " << colour.blue << ')';
    return out;
}

Colour intersect(const Ray& ray, const Scene& scene) noexcept {
    constexpr float infinity = std::numeric_limits<float>::infinity();

    std::optional<std::size_t> closest_intersecting_triangle_index = std::nullopt;
    float closest_triangle_intersection_distance = infinity;
    for (std::size_t i = 0; i < scene.triangles.size(); ++i) {
        const std::optional<float> intersection_distance = intersect(ray, scene.triangles[i]);
        if (intersection_distance.has_value() && intersection_distance.value() < closest_triangle_intersection_distance) {
            closest_intersecting_triangle_index = i;
            closest_triangle_intersection_distance = intersection_distance.value();
        }
    }

    std::optional<std::size_t> closest_intersecting_ellipsoid_index = std::nullopt;
    float closest_ellipsoid_intersection_distance = infinity;
    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
        const std::optional<std::pair<float, float>> transformed_intersection_distances = intersect(transformed_ray, ellipsoid.sphere);
        if (transformed_intersection_distances.has_value()) {
            const float transformed_intersection_distance_1 = transformed_intersection_distances.value().first;
            const float transformed_intersection_distance_2 = transformed_intersection_distances.value().second;

            const bool transformed_intersection_distance_1_is_positive = greater_than(transformed_intersection_distance_1, 0.0f);
            const bool transformed_intersection_distance_2_is_positive = greater_than(transformed_intersection_distance_2, 0.0f);

            assert(transformed_intersection_distance_1 <= transformed_intersection_distance_2);
            std::optional<float> transformed_closest_intersection_distance = std::nullopt;
            if (transformed_intersection_distance_1_is_positive) {
                transformed_closest_intersection_distance = transformed_intersection_distance_1;
            } else if (transformed_intersection_distance_2_is_positive) {
                transformed_closest_intersection_distance = transformed_intersection_distance_2;
            }

            if (transformed_closest_intersection_distance.has_value()) {
                const Matrix& ellipsoid_transform = scene.ellipsoid_transforms[i];
                const Vector transformed_intersection_point = transformed_ray.start + transformed_closest_intersection_distance.value() * transformed_ray.direction;
                const Vector intersection_point = ellipsoid_transform * transformed_intersection_point;
                const float intersection_distance = magnitude(intersection_point - ray.start);
                if (less_than(intersection_distance, closest_ellipsoid_intersection_distance)) {
                    closest_intersecting_ellipsoid_index = i;
                    closest_ellipsoid_intersection_distance = intersection_distance;
                }
            }
        }
    }

    Colour colour{0.0f, 0.0f, 0.0f};
    if (closest_intersecting_triangle_index.has_value() || closest_intersecting_ellipsoid_index.has_value()) {
        assert(closest_triangle_intersection_distance < infinity || closest_ellipsoid_intersection_distance < infinity);
        
        Material material{};
        Vector surface_normal{};
        Vector intersection_point{};
        if (less_than(closest_triangle_intersection_distance, closest_ellipsoid_intersection_distance)) {
            assert(closest_intersecting_triangle_index.has_value());
            const std::size_t index = closest_intersecting_triangle_index.value();
            
            intersection_point = ray.start + closest_triangle_intersection_distance * ray.direction;
            
            assert(index < scene.triangles.size());
            const Triangle& triangle = scene.triangles[index];
            surface_normal = unit_surface_normal(triangle);
            
            material = scene.triangle_materials[index];
        } else {    // ellipsoid is closer
            assert(closest_intersecting_ellipsoid_index.has_value());
            const std::size_t index = closest_intersecting_ellipsoid_index.value();
            
            intersection_point = ray.start + closest_ellipsoid_intersection_distance * ray.direction;
            
            assert(index < scene.ellipsoids.size());
            const Ellipsoid& ellipsoid = scene.ellipsoids[index];
            surface_normal = unit_surface_normal(ellipsoid, intersection_point);
            
            material = scene.ellipsoid_materials[index];
        }

        colour = scene.ambient + material.emission;

        const Vector direction_to_ray_start = -1.0f * ray.direction;
        const Vector epsilon_shift = 2.0f * tolerance * surface_normal;
        const Vector point_above_surface = intersection_point + epsilon_shift;
        if (scene.directional_light_source.has_value()) {
            const DirectionalLightSource& directional_light_source = scene.directional_light_source.value();
            if (!path_is_blocked(point_above_surface, directional_light_source, scene)) {
                const Vector direction_to_light = -1.0f * directional_light_source.direction;

                const float diffuse_intensity = std::max(surface_normal * direction_to_light, 0.0f);
                const Colour diffuse_contribution = diffuse_intensity * material.diffuse;

                const Vector half_angle = normalise(direction_to_ray_start + direction_to_light);
                const float specular_intensity = std::pow(std::max(surface_normal * half_angle, 0.0f), material.shininess);
                const Colour specular_contribution = specular_intensity * material.specular;

                const Colour directional_light_contribution = directional_light_source.colour * (diffuse_contribution + specular_contribution);
                colour += directional_light_contribution;
            }
        }

        for (const PointLightSource& light : scene.point_light_sources) {
            if (path_is_blocked(point_above_surface, light, scene)) {
                continue;
            }

            const Vector intersection_point_to_light = light.position - intersection_point;
            const float distance_to_light = magnitude(intersection_point_to_light);
            const Vector direction_to_light = intersection_point_to_light / distance_to_light;

            const float diffuse_intensity = std::max(surface_normal * direction_to_light, 0.0f);
            const Colour diffuse_contribution = diffuse_intensity * material.diffuse;

            const Vector half_angle = normalise(direction_to_ray_start + direction_to_light);
            const float specular_intensity = std::pow(std::max(surface_normal * half_angle, 0.0f), material.shininess);
            const Colour specular_contribution = specular_intensity * material.specular;

            const float light_attenuation = attenuation(scene.attenuation_parameters, distance_to_light);

            const Colour point_light_contribution = light_attenuation * light.colour * (diffuse_contribution + specular_contribution);
            colour += point_light_contribution;
        }
    }

    return colour;
}
