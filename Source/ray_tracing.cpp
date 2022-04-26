#include "ray_tracing.h"
#include "maths.h"

#include <algorithm>
#include <optional>
#include <cassert>
#include <limits>
#include <cmath>

// Intersection functions
std::optional<real> intersect(const Ray& ray, const Triangle& triangle) noexcept {
    const Vector a_to_b = triangle.b - triangle.a;
    const Vector a_to_c = triangle.c - triangle.a;

    const Vector plane_normal = a_to_b ^ a_to_c;
    if (are_equal(ray.direction * plane_normal, 0.0)) {  // Ray and plane are parallel
        return std::nullopt;
    }
    
    const real intersection_distance = ((triangle.a - ray.start) * plane_normal) / (ray.direction * plane_normal);
    if (less_than(intersection_distance, 0.0)) {
        return std::nullopt;
    }
    
    const Vector intersection_point = ray.start + intersection_distance * ray.direction;
    const Vector a_to_intersection = intersection_point - triangle.a;

    const real alpha = ((a_to_b ^ a_to_intersection) * plane_normal) / (plane_normal * plane_normal);
    const real beta = ((a_to_intersection ^ a_to_c) * plane_normal) / (plane_normal * plane_normal);

    if (less_than(alpha, 0.0) || less_than(beta, 0.0) || greater_than(alpha + beta, 1.0)) {
        return std::nullopt;
    }

    return intersection_distance;
}

std::optional<real> intersect(const Ray& ray, const Sphere& sphere) noexcept {
    const Vector sphere_centre_to_ray_start = ray.start - sphere.centre;
    const real a = ray.direction * ray.direction;
    const real b = 2 * ray.direction * sphere_centre_to_ray_start;
    const real c = sphere_centre_to_ray_start * sphere_centre_to_ray_start - sphere.radius * sphere.radius;
    
    const real discriminant = b * b - 4 * a * c;
    if (less_than(discriminant, 0.0)) {
        return std::nullopt;
    }
    
    const real intersection_distance_1 = (-b - std::sqrt(discriminant)) / 2.0 * a;
    const real intersection_distance_2 = intersection_distance_1 + std::sqrt(discriminant) / a;

    const bool intersection_distance_1_is_negative = less_than(intersection_distance_1, 0.0);
    const bool intersection_distance_2_is_negative = less_than(intersection_distance_2, 0.0);
    if (intersection_distance_1_is_negative && intersection_distance_2_is_negative) {
        return std::nullopt;
    } else if (intersection_distance_1_is_negative) {
        return intersection_distance_2;
    } else if (intersection_distance_2_is_negative) {
        return intersection_distance_1;
    } else {
        return std::min(intersection_distance_1, intersection_distance_2);
    }
}

// Ray helper methods
static Ray transform_ray(const Matrix& transform, Ray ray) noexcept {
    ray.start = transform * ray.start;

    ray.direction = homogenise(ray.direction);
    ray.direction.w = 0.0;
    ray.direction = transform * ray.direction;
    ray.direction.w = 1.0;

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

static Colour operator*(const real scalar, const Colour& colour) noexcept {
    return Colour{scalar * colour.red, scalar * colour.green, scalar * colour.blue};
}

static Colour operator*(const Colour& lhs, const Colour& rhs) noexcept {
    return Colour{lhs.red * rhs.red, lhs.green * rhs.green, lhs.blue * rhs.blue};
}

static real intensity(const Colour& colour) noexcept {
    return (colour.red + colour.green + colour.blue) / 3.0;
}

// Attenuation functions
static real attenuation(const AttenuationParameters& attenuation_parameters, const real distance) noexcept {
    const real constant_term = attenuation_parameters.constant;
    const real linear_term = attenuation_parameters.linear * distance;
    const real quadratic_term = attenuation_parameters.quadratic * distance * distance;

    return 1.0 / (constant_term + linear_term + quadratic_term);
}

// Light source path checking functions
static bool path_is_blocked(const Vector& start, const PointLightSource& light, const Scene& scene) noexcept {
    const Vector start_to_destination = light.position - start;
    const real distance_to_destination = magnitude(start_to_destination);
    const Ray ray{start, start_to_destination / distance_to_destination};

    for (const Triangle& triangle : scene.triangles) {
        const std::optional<real> intersection_distance = intersect(ray, triangle);
        if (intersection_distance.has_value() && !are_equal(intersection_distance.value(), 0.0) && less_than(intersection_distance.value(), distance_to_destination)) {
            return true;
        }
    }

    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
        const std::optional<real> transformed_intersection_distance = intersect(transformed_ray, ellipsoid.sphere);
        if (transformed_intersection_distance.has_value()) {
            const Matrix& ellipsoid_transform = scene.ellipsoid_transforms[i];
            const Vector transformed_intersection_point = transformed_ray.start + transformed_intersection_distance.value() * transformed_ray.direction;
            const Vector intersection_point = ellipsoid_transform * transformed_intersection_point;
            const real intersection_distance = magnitude(intersection_point - ray.start);
            if (!are_equal(intersection_distance, 0.0) && less_than(intersection_distance, distance_to_destination)) {
                return true;
            }
        }
    }

    return false;
}

static bool path_is_blocked(const Vector& start, const DirectionalLightSource& light, const Scene& scene) noexcept {
    const Ray ray{start, light.direction};

    for (const Triangle& triangle : scene.triangles) {
        const std::optional<real> intersection_distance = intersect(ray, triangle);
        if (intersection_distance.has_value() && !are_equal(intersection_distance.value(), 0.0)) {
            return true;
        }
    }

    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
        const std::optional<real> transformed_intersection_distance = intersect(transformed_ray, ellipsoid.sphere);
        if (transformed_intersection_distance.has_value()) {
            const Matrix& ellipsoid_transform = scene.ellipsoid_transforms[i];
            const Vector transformed_intersection_point = transformed_ray.start + transformed_intersection_distance.value() * transformed_ray.direction;
            const Vector intersection_point = ellipsoid_transform * transformed_intersection_point;
            const real intersection_distance = magnitude(intersection_point - ray.start);
            if (!are_equal(intersection_distance, 0.0)) {
                return true;
            }
        }
    }

    return false;
}

// Exposed functions
Ray ray_through_pixel(const Camera& camera, const int x, const int y, const Image& image) noexcept {
    const Vector w = normalise(camera.look_at - camera.eye);    // check this isn't 0 vector
    const Vector u = normalise(camera.up ^ w);
    const Vector v = w ^ u;

    const real half_image_world_width = std::tan(0.5 * to_radians(camera.field_of_view.x));
    const real half_image_pixel_width = 0.5 * image.width;
    const real alpha = half_image_world_width * (half_image_pixel_width - (x + 0.5)) / half_image_pixel_width;

    const real half_image_world_height = std::tan(0.5 * to_radians(camera.field_of_view.y));
    const real half_image_pixel_height = 0.5 * image.height;
    const real beta = half_image_world_height * (half_image_pixel_height - (y + 0.5)) / half_image_pixel_height;
    
    const Vector ray_direction = normalise(alpha * u + beta * v + w);
    return Ray{camera.eye, ray_direction};
}

// TODO: Likely just used for debugging and can probably be removed later
std::ostream& operator<<(std::ostream& out, const Colour& colour) {
    out << '(' << colour.red << ", " << colour.green << ", " << colour.blue << ')';
    return out;
}

Colour intersect(const Ray& ray, const Scene& scene, const Vector& camera_eye) noexcept {
    constexpr real infinity = std::numeric_limits<real>::infinity();

    std::optional<std::size_t> closest_intersecting_triangle_index = std::nullopt;
    real closest_triangle_intersection_distance = std::numeric_limits<real>::infinity();
    for (std::size_t i = 0; i < scene.triangles.size(); ++i) {
        const std::optional<real> intersection_distance = intersect(ray, scene.triangles[i]);
        if (intersection_distance.has_value() && less_than(intersection_distance.value(), closest_triangle_intersection_distance)) {
            closest_intersecting_triangle_index = i;
            closest_triangle_intersection_distance = intersection_distance.value();
        }
    }

    std::optional<std::size_t> closest_intersecting_ellipsoid_index = std::nullopt;
    real closest_ellipsoid_intersection_distance = std::numeric_limits<real>::infinity();
    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformed_ray = transform_ray(ellipsoid.inverse_transform, ray);
        const std::optional<real> transformed_intersection_distance = intersect(transformed_ray, ellipsoid.sphere);
        if (transformed_intersection_distance.has_value()) {
            const Matrix& ellipsoid_transform = scene.ellipsoid_transforms[i];
            const Vector transformed_intersection_point = transformed_ray.start + transformed_intersection_distance.value() * transformed_ray.direction;
            const Vector intersection_point = ellipsoid_transform * transformed_intersection_point;
            const real intersection_distance = magnitude(intersection_point - ray.start);
            if (less_than(intersection_distance, closest_ellipsoid_intersection_distance)) {
                closest_intersecting_ellipsoid_index = i;
                closest_ellipsoid_intersection_distance = intersection_distance;
            }
        }
    }

    Colour colour{0.0, 0.0, 0.0};
    if (closest_intersecting_triangle_index.has_value() || closest_intersecting_ellipsoid_index.has_value()) {
        assert(closest_triangle_intersection_distance < infinity || closest_ellipsoid_intersection_distance < infinity);
        
        Colour ambient{};
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
            
            ambient = scene.triangle_ambients[index];
            material = scene.triangle_materials[index];
        } else {    // ellipsoid is closer
            assert(closest_intersecting_ellipsoid_index.has_value());
            const std::size_t index = closest_intersecting_ellipsoid_index.value();
            
            intersection_point = ray.start + closest_ellipsoid_intersection_distance * ray.direction;
            
            assert(index < scene.ellipsoids.size());
            const Ellipsoid& ellipsoid = scene.ellipsoids[index];
            surface_normal = unit_surface_normal(ellipsoid, intersection_point);
            
            ambient = scene.ellipsoid_ambients[index];
            material = scene.ellipsoid_materials[index];
        }

        colour = ambient + material.emission;

        const Vector intersection_point_to_camera = camera_eye - intersection_point;
        const real distance_to_camera = magnitude(intersection_point_to_camera);
        const Vector direction_to_camera = intersection_point_to_camera / distance_to_camera;

        if (scene.directional_light_source.has_value()) {
            const DirectionalLightSource& directional_light_source = scene.directional_light_source.value();
            if (!path_is_blocked(intersection_point, directional_light_source, scene)) {
                const Vector direction_to_light = -1.0 * directional_light_source.direction;

                const real diffuse_intensity = std::max(surface_normal * direction_to_light, 0.0);
                const Colour diffuse_contribution = diffuse_intensity * material.diffuse;

                const Vector half_angle = normalise(direction_to_camera + direction_to_light);
                const real specular_intensity = std::pow(std::max(surface_normal * half_angle, 0.0), material.shininess);
                const Colour specular_contribution = specular_intensity * material.specular;

                const Colour directional_light_contribution = directional_light_source.colour * (diffuse_contribution + specular_contribution);
                colour += directional_light_contribution;
            }
        }

        for (const PointLightSource& light : scene.point_light_sources) {
            if (path_is_blocked(intersection_point, light, scene)) {
                continue;
            }

            const Vector intersection_point_to_light = light.position - intersection_point;
            const real distance_to_light = magnitude(intersection_point_to_light);
            const Vector direction_to_light = intersection_point_to_light / distance_to_light;

            const real diffuse_intensity = std::max(surface_normal * direction_to_light, 0.0);
            const Colour diffuse_contribution = diffuse_intensity * material.diffuse;

            const Vector half_angle = normalise(direction_to_camera + direction_to_light);
            const real specular_intensity = std::pow(std::max(surface_normal * half_angle, 0.0), material.shininess);
            const Colour specular_contribution = specular_intensity * material.specular;

            const real light_attenuation = attenuation(scene.attenuation_parameters, distance_to_light);

            const Colour point_light_contribution = light_attenuation * light.colour * (diffuse_contribution + specular_contribution);
            colour += point_light_contribution;
        }
    }

    return colour;
}
