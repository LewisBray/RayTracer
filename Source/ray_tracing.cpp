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

Colour operator+(const Colour& lhs, const Colour& rhs) noexcept
{
    return Colour{ lhs.red + rhs.red, lhs.green + rhs.green, lhs.blue + rhs.blue};
}

Colour& operator+=(Colour& lhs, const Colour& rhs) noexcept
{
    lhs.red += rhs.red;
    lhs.green += rhs.green;
    lhs.blue += rhs.blue;

    return lhs;
}

Colour operator*(const real scalar, const Colour& colour) noexcept
{
    return Colour{scalar * colour.red, scalar * colour.green, scalar * colour.blue};
}

Vector unitSurfaceNormal(const Triangle& triangle) noexcept
{
    return normalise((triangle.b - triangle.a) ^ (triangle.c - triangle.a));
}

Vector unitSurfaceNormal(const Sphere& sphere, const Vector& point) noexcept
{
    return normalise(point - sphere.centre);
}

Vector unitSurfaceNormal(const Ellipsoid& ellipsoid,
    const Matrix& ellipsoidTransform, const Vector& point) noexcept
{
    const Vector pointInEllipsoidSpace = ellipsoid.inverseTransform * point;
    
    Vector surfaceNormal = homogenise(unitSurfaceNormal(ellipsoid.sphere, pointInEllipsoidSpace));
    surfaceNormal.w = 0.0;
    surfaceNormal = ellipsoidTransform * surfaceNormal;
    surfaceNormal.w = 1.0;

    return normalise(surfaceNormal);
}

real intensity(const Colour& colour) noexcept
{
    return (colour.red + colour.green + colour.blue) / 3.0;
}

real attenuation(const AttenuationParameters& attenuationParameters, const real distance) noexcept
{
    const real constantTerm = attenuationParameters.constant;
    const real linearTerm = attenuationParameters.linear * distance;
    const real quadraticTerm = attenuationParameters.quadratic * distance * distance;

    return 1.0 / (constantTerm + linearTerm + quadraticTerm);
}

bool pathIsBlocked(const Vector& start, const PointLightSource& light, const Scene& scene) noexcept
{
    const Vector startToDestination = light.position - start;
    const real distanceToDestination = magnitude(startToDestination);
    const Ray ray{start, startToDestination / distanceToDestination};

    for (const Triangle& triangle : scene.triangles)
    {
        const std::optional<real> intersectionDistance = intersect(ray, triangle);
        if (intersectionDistance.has_value() && !areEqual(intersectionDistance.value(), 0.0) && intersectionDistance.value() < distanceToDestination)
            return true;
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
            if (!areEqual(intersectionDistance, 0.0) && intersectionDistance < distanceToDestination)
                return true;
        }
    }

    return false;
}

bool pathIsBlocked(const Vector& start, const DirectionalLightSource& light, const Scene& scene) noexcept
{
    const Ray ray{start, light.direction};

    for (const Triangle& triangle : scene.triangles)
    {
        const std::optional<real> intersectionDistance = intersect(ray, triangle);
        if (intersectionDistance.has_value() && !areEqual(intersectionDistance.value(), 0.0))
            return true;
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
            if (!areEqual(intersectionDistance, 0.0))
                return true;
        }
    }

    return false;
}

Colour intersect(const Ray& ray, const Scene& scene, const Vector& cameraEye) noexcept
{
    constexpr real infinity = std::numeric_limits<real>::infinity();

    std::optional<std::size_t> closestIntersectingTriangleIndex = std::nullopt;
    real closestTriangleIntersectionDistance = std::numeric_limits<real>::infinity();
    for (std::size_t i = 0; i < scene.triangles.size(); ++i)
    {
        const std::optional<real> intersectionDistance = intersect(ray, scene.triangles[i]);
        if (intersectionDistance.has_value() && intersectionDistance.value() < closestTriangleIntersectionDistance)
        {
            closestIntersectingTriangleIndex = i;
            closestTriangleIntersectionDistance = intersectionDistance.value();
        }
    }

    std::optional<std::size_t> closestIntersectingEllipsoidIndex = std::nullopt;
    real closestEllipsoidIntersectionDistance = std::numeric_limits<real>::infinity();
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
            if (intersectionDistance < closestEllipsoidIntersectionDistance)
            {
                closestIntersectingEllipsoidIndex = i;
                closestEllipsoidIntersectionDistance = intersectionDistance;
            }
        }
    }

    Colour colour{0.0, 0.0, 0.0};
    if (closestIntersectingTriangleIndex.has_value() || closestIntersectingEllipsoidIndex.has_value())
    {
        assert(closestTriangleIntersectionDistance < infinity || closestEllipsoidIntersectionDistance < infinity);
        
        Colour ambient{};
        Material material{};
        Vector surfaceNormal{};
        Vector intersectionPoint{};
        if (closestTriangleIntersectionDistance < closestEllipsoidIntersectionDistance)
        {
            assert(closestIntersectingTriangleIndex.has_value());
            const std::size_t index = closestIntersectingTriangleIndex.value();
            
            intersectionPoint = ray.start + closestTriangleIntersectionDistance * ray.direction;
            
            assert(index < scene.triangles.size());
            const Triangle& triangle = scene.triangles[index];
            surfaceNormal = unitSurfaceNormal(triangle);
            
            ambient = scene.triangleAmbients[index];
            material = scene.triangleMaterials[index];
        }
        else // ellipsoid is closer
        {
            assert(closestIntersectingEllipsoidIndex.has_value());
            const std::size_t index = closestIntersectingEllipsoidIndex.value();
            
            intersectionPoint = ray.start + closestEllipsoidIntersectionDistance * ray.direction;
            
            assert(index < scene.ellipsoids.size());
            const Ellipsoid& ellipsoid = scene.ellipsoids[index];
            const Matrix& ellipsoidTransform = scene.ellipsoidTransforms[index];
            surfaceNormal = unitSurfaceNormal(ellipsoid, ellipsoidTransform, intersectionPoint);
            
            ambient = scene.ellipsoidAmbients[index];
            material = scene.ellipsoidMaterials[index];
        }

        colour = ambient + material.emission;

        const DirectionalLightSource& directionalLight = scene.directionalLightSource;
        // TO-DO: This check is horrible, fix it at input parsing stage, maybe optional directional light source?
        if (directionalLight.direction.x != 0.0 && directionalLight.direction.y != 0.0 && directionalLight.direction.z != 0.0)
        {
            if (!pathIsBlocked(intersectionPoint, directionalLight, scene))
            {
                const Vector directionToLight = directionalLight.direction;

                const Colour diffuseContribution =
                    std::max(surfaceNormal * directionToLight, 0.0) * material.diffuse;

                const Vector halfAngle = normalise(cameraEye + directionToLight);
                const Colour specularContribution =
                    std::pow(std::max(surfaceNormal * halfAngle, 0.0), material.shininess) * material.specular;

                colour += intensity(directionalLight.colour) * (diffuseContribution + specularContribution);
            }
        }

        for (const PointLightSource& light : scene.pointLightSources)
        {
            if (!pathIsBlocked(intersectionPoint, light, scene))
            {
                const Vector intersectionPointToLight = light.position - intersectionPoint;
                const real distanceToLight = magnitude(intersectionPointToLight);
                const Vector directionToLight = intersectionPointToLight / distanceToLight;

                const Colour diffuseContribution = std::max(surfaceNormal * directionToLight, 0.0) * material.diffuse;

                const Vector halfAngle = normalise(cameraEye + directionToLight);
                const Colour specularContribution =
                    std::pow(std::max(surfaceNormal * halfAngle, 0.0), material.shininess) * material.specular;

                colour += intensity(light.colour) *
                    attenuation(light.attenuationParameters, distanceToLight) *
                    (diffuseContribution + specularContribution);
            }
        }
    }

    return colour;
}
