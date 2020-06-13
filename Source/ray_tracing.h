#ifndef RAY_TRACING_H
#define RAY_TRACING_H

#include "maths.h"

#include <optional>
#include <vector>
#include <string>

struct FieldOfView
{
    real x;
    real y;
};

struct Camera
{
    Vector eye;
    Vector lookAt;
    Vector up;
    FieldOfView fieldOfView;
};

struct Ray
{
    Vector start;
    Vector direction;
};

struct Colour
{
    real red;
    real green;
    real blue;
};

struct AttenuationParameters
{
    real constant;
    real linear;
    real quadratic;
};

struct DirectionalLightSource
{
    Vector direction;
    Colour colour;
    AttenuationParameters attenuationParameters;
};

struct PointLightSource
{
    Vector position;
    Colour colour;
    AttenuationParameters attenuationParameters;
};

struct Material
{
    Colour diffuse;
    Colour specular;
    Colour emission;
    real shininess;
};

struct Scene
{
    std::vector<Triangle> triangles;
    std::vector<Material> triangleMaterials;
    std::vector<Colour> triangleAmbients;

    std::vector<Ellipsoid> ellipsoids;
    std::vector<Matrix> ellipsoidTransforms;
    std::vector<Material> ellipsoidMaterials;
    std::vector<Colour> ellipsoidAmbients;

    DirectionalLightSource directionalLightSource;
    std::vector<PointLightSource> pointLightSources;
};

struct Image
{
    int width;
    int height;
    std::uint8_t* pixels;
    std::string filename;
};

Ray rayThroughPixel(const Camera& camera, int x, int y, const Image& image) noexcept;
Colour intersect(const Ray& ray, const Scene& scene, const Vector& cameraEye) noexcept;

// Exposed at the moment for testing but ultimately should be static
std::optional<real> intersect(const Ray& ray, const Triangle& triangle) noexcept;
std::optional<real> intersect(const Ray& ray, const Sphere& sphere) noexcept;

#endif
