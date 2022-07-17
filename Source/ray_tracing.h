#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include <optional>
#include <utility>
#include <vector>
#include <string>

#include "maths.h"

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

static Colour& operator+=(Colour& lhs, const Colour& rhs) noexcept;
static Colour operator*(const float scalar, const Colour& colour) noexcept;

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

// Disable padded warning as it's not relevant for this struct
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpadded"
struct Scene {
    std::vector<Triangle> triangles;
    std::vector<Material> triangle_materials;

    std::vector<Sphere> spheres;
    std::vector<Material> sphere_materials;

    std::vector<Ellipsoid> ellipsoids;
    std::vector<Matrix> ellipsoid_transforms;
    std::vector<Material> ellipsoid_materials;

    AxisAlignedBoundingBox bounding_box;

    Colour ambient;
    AttenuationParameters attenuation_parameters;
    std::vector<PointLightSource> point_light_sources;
    std::optional<DirectionalLightSource> directional_light_source;
};
#pragma clang diagnostic pop

struct Image {
    unsigned width;
    unsigned height;
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

static Vector ray_direction_through_pixel(
    unsigned pixel_x,
    unsigned pixel_y,
    float x_offset,
    float y_offset,
    const BasisVectors& camera_basis_vectors,
    const Dimensions& half_image_dimensions_world,
    const Dimensions& half_image_dimensions_pixels
) noexcept;

static Colour intersect(const Ray& ray, const Scene& scene) noexcept;

#endif
