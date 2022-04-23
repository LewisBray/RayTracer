#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include "maths.h"

#include <optional>
#include <vector>
#include <string>

struct FieldOfView {
    real x;
    real y;
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
    real red;
    real green;
    real blue;
};

struct AttenuationParameters {
    real constant;
    real linear;
    real quadratic;
};

struct DirectionalLightSource {
    Vector direction;
    Colour colour;
    AttenuationParameters attenuation_parameters;
};

struct PointLightSource {
    Vector position;
    Colour colour;
    AttenuationParameters attenuation_parameters;
};

struct Material {
    Colour diffuse;
    Colour specular;
    Colour emission;
    real shininess;
};

struct Scene {
    std::vector<Triangle> triangles;
    std::vector<Material> triangle_materials;
    std::vector<Colour> triangle_ambients;

    std::vector<Ellipsoid> ellipsoids;
    std::vector<Matrix> ellipsoid_transforms;
    std::vector<Material> ellipsoid_materials;
    std::vector<Colour> ellipsoid_ambients;

    std::optional<DirectionalLightSource> directional_light_source;
    std::vector<PointLightSource> point_light_sources;
};

struct Image {
    int width;
    int height;
    std::uint8_t* pixels;
    std::string filename;
};

Ray ray_through_pixel(const Camera& camera, int x, int y, const Image& image) noexcept;
Colour intersect(const Ray& ray, const Scene& scene, const Vector& camera_eye) noexcept;

// Exposed at the moment for testing but ultimately should be static
std::optional<real> intersect(const Ray& ray, const Triangle& triangle) noexcept;
std::optional<real> intersect(const Ray& ray, const Sphere& sphere) noexcept;

#endif
