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

struct Triangle
{
    Vector a;
    Vector b;
    Vector c;
};

struct Sphere
{
    Vector centre;
    real radius;
};

struct Ellipsoid
{
    Sphere sphere;
    Matrix inverseTransform;
};

struct Scene
{
    std::vector<Triangle> triangles;
    std::vector<Colour> triangleAmbients;
    std::vector<Ellipsoid> ellipsoids;
    std::vector<Matrix> ellipsoidTransforms;
    std::vector<Colour> ellipsoidAmbients;
};

struct Image
{
    int width;
    int height;
    std::uint8_t* pixels;
    std::string filename;
};

std::optional<real> intersect(const Ray& ray, const Triangle& triangle) noexcept;
std::optional<real> intersect(const Ray& ray, const Sphere& sphere) noexcept;
Ray rayThroughPixel(const Camera& camera, const int x, const int y, const Image& image) noexcept;
Colour intersects(const Ray& ray, const Scene& scene) noexcept;

#endif
