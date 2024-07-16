#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include <vector>

#include "maths.h"

struct Vec3x8 {
    float x[8];
    float y[8];
    float z[8];
};

struct Mat3x4x8 {
    float rows[3][4][8];
};

struct Triangle8 {
    Vec3x8 a;
    Vec3x8 a_to_b;
    Vec3x8 a_to_c;
};

struct Sphere8 {
    Vec3x8 centre;
    float radius[8];
};

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

struct AABBIntersectionResult {
    float min_distance;
    float max_distance;
};

static AABBIntersectionResult intersect(const Ray& ray, const AxisAlignedBoundingBox& aabb) noexcept;

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
    std::vector<Triangle8> triangle8s;
    std::vector<Material> triangle_materials;

    std::vector<Sphere8> sphere8s;
    std::vector<Material> sphere_materials;

    std::vector<Mat3x4x8> ellipsoid8_inverse_transforms;
    std::vector<Mat3x4x8> ellipsoid8_transforms;
    std::vector<Material> ellipsoid_materials;

    AxisAlignedBoundingBox bounding_box;

    Colour ambient;
    AttenuationParameters attenuation_parameters;
    std::vector<PointLightSource> point_light_sources;
    DirectionalLightSource directional_light_source;
    bool has_directional_light_source;
};
#pragma clang diagnostic pop

struct Image {
    unsigned width;
    unsigned height;
    std::uint8_t* pixels;
    char filename[256];
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

static Colour intersect(Ray ray, const Scene& scene, int max_bounce_count) noexcept;

#endif
