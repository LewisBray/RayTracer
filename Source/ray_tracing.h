#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include "maths.h"

#include <optional>
#include <utility>
#include <vector>
#include <string>

struct FieldOfView {
    float x;
    float y;
};

struct Camera {
    Vector eye;
    Vector look_at;
    Vector up;
    FieldOfView field_of_view;
};

struct Ray {
    Vector start;
    Vector direction;
};

struct Colour {
    float red;
    float green;
    float blue;
};

struct DirectionalLightSource {
    Vector direction;
    Colour colour;
};

struct PointLightSource {
    Vector position;
    Colour colour;
};

struct Material {
    Colour diffuse;
    Colour specular;
    Colour emission;
    float shininess;
};

struct AttenuationParameters {
    float constant;
    float linear;
    float quadratic;
};

struct Scene {
    std::vector<Triangle> triangles;
    std::vector<Material> triangle_materials;

    std::vector<Ellipsoid> ellipsoids;
    std::vector<Matrix> ellipsoid_transforms;
    std::vector<Material> ellipsoid_materials;

    std::optional<DirectionalLightSource> directional_light_source;
    std::vector<PointLightSource> point_light_sources;

    Colour ambient;
    AttenuationParameters attenuation_parameters;
};

struct Image {
    int width;
    int height;
    std::uint8_t* pixels;
    std::string filename;
};

struct BasisVectors {
    Vector i;
    Vector j;
    Vector k;
};

struct Dimensions {
    float width;
    float height;
};

Vector ray_direction_through_pixel(
    int pixel_x,
    int pixel_y,
    const BasisVectors& camera_basis_vectors,
    const Dimensions& half_image_dimensions_world,
    const Dimensions& half_image_dimensions_pixels
) noexcept;

Colour intersect(const Ray& ray, const Scene& scene) noexcept;

// Exposed at the moment for testing but ultimately should be static
std::optional<float> intersect(const Ray& ray, const Triangle& triangle) noexcept;
std::optional<std::pair<float, float>> intersect(const Ray& ray, const Sphere& sphere) noexcept;

#endif
