#include "ray_tracing.h"
#include "maths.h"

#include <algorithm>
#include <optional>
#include <cassert>
#include <limits>
#include <cmath>

std::optional<real> intersect(const Ray& ray, const Triangle& triangle) noexcept
{
    const Vector aToB = triangle.b - triangle.a;
    const Vector aToC = triangle.c - triangle.a;

    const Vector planeNormal = aToB ^ aToC;
    if (areEqual(ray.direction * planeNormal, 0.0))  // Ray and plane are parallel
        return std::nullopt;
    
    const real intersectionDistance =
        ((triangle.a - ray.start) * planeNormal) / (ray.direction * planeNormal);
    if (lessThan(intersectionDistance, 0.0))
        return std::nullopt;
    
    const Vector intersectionPoint = ray.start + intersectionDistance * ray.direction;
    const Vector aToIntersection = intersectionPoint - triangle.a;
    const Vector bToIntersection = intersectionPoint - triangle.b;
    const Vector cToIntersection = intersectionPoint - triangle.c;

    const real alpha = ((aToB ^ aToIntersection) * planeNormal) / (planeNormal * planeNormal);
    const real beta = ((aToIntersection ^ aToC) * planeNormal) / (planeNormal * planeNormal);

    if (lessThan(alpha, 0.0) || lessThan(beta, 0.0) || greaterThan(alpha + beta, 1.0))
        return std::nullopt;

    return intersectionDistance;
}

std::optional<real> intersect(const Ray& ray, const Sphere& sphere) noexcept
{
    const Vector sphereCentreToRayStart = ray.start - sphere.centre;
    const real a = ray.direction * ray.direction;
    const real b = 2 * ray.direction * sphereCentreToRayStart;
    const real c = sphereCentreToRayStart * sphereCentreToRayStart - sphere.radius * sphere.radius;
    
    const real determinant = b * b - 4 * a * c;
    if (lessThan(determinant, 0.0))
        return std::nullopt;
    
    const real intersectionDistance1 = (-b - std::sqrt(determinant)) / 2.0 * a;
    const real intersectionDistance2 = intersectionDistance1 + std::sqrt(determinant) / a;

    const bool intersectionDistance1isNegative = lessThan(intersectionDistance1, 0.0);
    const bool intersectionDistance2IsNegative = lessThan(intersectionDistance2, 0.0);
    if (intersectionDistance1isNegative && intersectionDistance2IsNegative)
        return std::nullopt;
    else if (intersectionDistance1isNegative)
        return intersectionDistance2;
    else if (intersectionDistance2IsNegative)
        return intersectionDistance1;
    else
        return std::min(intersectionDistance1, intersectionDistance2);
}

Ray rayThroughPixel(const Camera& camera, const int x, const int y, const Image& image) noexcept
{
    const Vector w = normalise(camera.lookAt - camera.eye); // check this isn't 0 vector
    const Vector u = normalise(camera.up ^ w);
    const Vector v = w ^ u;

    const real halfImageWorldWidth = std::tan(0.5 * toRadians(camera.fieldOfView.x));
    const real halfImagePixelWidth = 0.5 * image.width;
    const real alpha = halfImageWorldWidth * (halfImagePixelWidth - (x + 0.5)) / halfImagePixelWidth;

    const real halfImageWorldHeight = std::tan(0.5 * toRadians(camera.fieldOfView.y));
    const real halfImagePixelHeight = 0.5 * image.height;
    const real beta = halfImageWorldHeight * (halfImagePixelHeight - (y + 0.5)) / halfImagePixelHeight;
    
    const Vector rayDirection = normalise(alpha * u + beta * v + w);
    return Ray{camera.eye, rayDirection};
}

static Ray transformRay(const Matrix& transform, Ray ray) noexcept
{
    ray.start = transform * ray.start;

    ray.direction = homogenise(ray.direction);
    ray.direction.w = 0.0;
    ray.direction = transform * ray.direction;
    ray.direction.w = 1.0;

    return Ray{ray.start, normalise(ray.direction)};
}

Colour intersects(const Ray& ray, const Scene& scene) noexcept
{
    Colour colour{0.0, 0.0, 0.0};
    real closestIntersectionDistance = std::numeric_limits<real>::infinity();
    for (std::size_t i = 0; i < scene.triangles.size(); ++i)
    {
        const std::optional<real> intersectionDistance = intersect(ray, scene.triangles[i]);
        if (intersectionDistance.has_value() && intersectionDistance.value() < closestIntersectionDistance)
        {
            colour = scene.triangleAmbients[i];
            closestIntersectionDistance = intersectionDistance.value();
        }
    }
    
    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i)
    {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Ray transformedRay = transformRay(ellipsoid.inverseTransform, ray);
        const std::optional<real> transformedIntersectionDistance = intersect(transformedRay, ellipsoid.sphere);
        if (transformedIntersectionDistance.has_value())
        {
            const Matrix& ellipsoidTransform = scene.ellipsoidTransforms[i];
            const Vector transformedIntersectionPoint = transformedRay.start + transformedIntersectionDistance.value() * transformedRay.direction;
            const Vector intersectionPoint = ellipsoidTransform * transformedIntersectionPoint;
            const real intersectionDistance = magnitude(intersectionPoint - ray.start);
            if (intersectionDistance < closestIntersectionDistance)
            {
                colour = scene.ellipsoidAmbients[i];
                closestIntersectionDistance = intersectionDistance;
            }
        }
    }
    
    return colour;
}
