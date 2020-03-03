#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../Source/ray_tracing.h"
#include "test_utils.h"

TEST_CASE("intersect", "[intersect]")
{
    SECTION("triangle intersection")
    {
        const Vector start{0.0, 0.0, 5.0, 1.0};
        const Vector direction{0.0, 0.0, -1.0, 1.0};
        const Ray ray{start, direction};

        const Vector xy1{0.0, 0.5, 0.0, 1.0};
        const Vector xy2{-0.5, -0.5, 0.0, 1.0};
        const Vector xy3{0.5, -0.5, 0.0, 1.0};
        const Triangle xyTriangle{xy1, xy2, xy3};

        // Ray starts 5 units in the z-axis above the triangle in the xy-plane
        const std::optional<real> xyIntersectionDistance = intersect(ray, xyTriangle);
        REQUIRE(xyIntersectionDistance.has_value());
        REQUIRE(areEqual(xyIntersectionDistance.value(), 5.0));

        // Ray shouldn't intersect with triangle that's parallel to it
        const Vector yz1{1.0, 0.5, 0.0, 1.0};
        const Vector yz2{1.0, -0.5, -0.5, 1.0};
        const Vector yz3{1.0, -0.5, 0.5, 1.0};
        const Triangle yzTriangle{yz1, yz2, yz3};
        const std::optional<real> yzIntersectionDistance = intersect(ray, yzTriangle);
        REQUIRE(!yzIntersectionDistance.has_value());

        // Ray shouldn't intersect with parallel triangle even if in same plane
        const Vector zx1{0.0, 0.0, 0.5, 1.0};
        const Vector zx2{-0.5, 0.0, -0.5, 1.0};
        const Vector zx3{-0.5, 0.0, 0.5, 1.0};
        const Triangle zxTriangle{zx1, zx2, zx3};
        const std::optional<real> zxIntersectionDistance = intersect(ray, zxTriangle);
        REQUIRE(!zxIntersectionDistance.has_value());

        // Ray should intersect with vertex at origin
        const Vector origin{0.0, 0.0, 0.0, 1.0};
        const Vector v2{0.4, 2.3, 0.1, 2.0};
        const Vector v3{3.1, 1.1, -0.7, 3.0};
        const Triangle vertexAtOriginTriangle{origin, v2, v3};
        const std::optional<real> originIntersectionDistance = intersect(ray, vertexAtOriginTriangle);
        REQUIRE(originIntersectionDistance.has_value());
        REQUIRE(areEqual(originIntersectionDistance.value(), 5.0));

        // Ray shouldn't intersect with triangle behind it
        const Vector behindCameraShift{0.0, 0.0, 1.0, 1.0 / 10.0};
        const Triangle triangleBehindCamera{xy1 + behindCameraShift, xy2 + behindCameraShift, xy3 + behindCameraShift};
        const std::optional<real> behindCameraIntersectionDistance = intersect(ray, triangleBehindCamera);
        REQUIRE(!behindCameraIntersectionDistance.has_value());

        // Try an intersection with a weird triangle and ray
        const Vector a{2.0, 5.0, 1.0, 0.5};
        const Vector b{2.0, -1.0, -3.0, 0.25};
        const Vector c{-9.0, 21.0, 15.0, 3.0};
        const Triangle weirdTriangle{a, b, c};
        const Vector weirdStart{16.0, 48.0, 40.0, 8.0};
        const Vector weirdDirection{-0.5, 0.5, -1.0, std::sqrt(6.0) / 2.0};
        const Ray weirdRay{weirdStart, weirdDirection};
        const std::optional<real> weirdIntersectionDistance = intersect(weirdRay, weirdTriangle);
        REQUIRE(weirdIntersectionDistance.has_value());
        REQUIRE(areEqual(weirdIntersectionDistance.value(), 3.178055923));
    }

    SECTION("sphere intersection")
    {
        const Vector centre{4.0, 1.0, -8.0, 1.0};
        const real radius = 3.0;
        const Sphere sphere{centre, radius};

        // Intersecting twice with sphere should yield closest intersection distance
        const Vector twiceIntersectingStart{-5.0, -1.0, -8.0, 1.0};
        const Vector twiceIntersectingDirection{6.0, 2.0, 0.0, std::sqrt(40.0)};
        const Ray twiceIntersectingRay{twiceIntersectingStart, twiceIntersectingDirection};
        const std::optional<real> twiceIntersectingDistance = intersect(twiceIntersectingRay, sphere);
        REQUIRE(twiceIntersectingDistance.has_value());
        REQUIRE(areEqual(twiceIntersectingDistance.value(), std::sqrt(40.0)));

        // Check single intersections
        const Vector onceIntersectingStart{7.0, 1.0, -20.0, 1.0};
        const Vector onceIntersectingDirection{0.0, 0.0, 1.0, 1.0};
        const Ray onceIntersectingRay{onceIntersectingStart, onceIntersectingDirection};
        const std::optional<real> onceIntersectingDistance = intersect(onceIntersectingRay, sphere);
        REQUIRE(onceIntersectingDistance.has_value());
        REQUIRE(areEqual(onceIntersectingDistance.value(), 12.0));

        // Check intersection we expect to miss
        const Vector neverIntersectingStart{4.0, 10.0, 0.0, 1.0};
        const Vector neverIntersectingDirection{1.0, 1.0, 1.0, std::sqrt(3.0)};
        const Ray neverIntersectingRay{neverIntersectingStart, neverIntersectingDirection};
        const std::optional<real> neverIntersectingDistance = intersect(neverIntersectingRay, sphere);
        REQUIRE(!neverIntersectingDistance.has_value());

        // Check single intersection with ray origin inside the sphere
        const Vector internalStart{5.0, 2.0, -7.0, 1.0};
        const Vector internalDirection{0.0, 1.0, 0.0, 1.0};
        const Ray internalRay{internalStart, internalDirection};
        const std::optional<real> internalIntersectionDistance = intersect(internalRay, sphere);
        REQUIRE(internalIntersectionDistance.has_value());
        REQUIRE(areEqual(internalIntersectionDistance.value(), 1.645751311));
    }
}

TEST_CASE("rayThroughPixel", "[rayThroughPixel]")
{
    const Vector cameraEye{0.0, 0.0, 4.0, 1.0};
    const Vector cameraLookAt{0.0, 0.0, 0.0, 1.0};
    const Vector cameraUp{0.0, 1.0, 0.0, 1.0};
    const FieldOfView cameraFieldOfView{45.0, 45.0};
    const Camera camera{cameraEye, cameraLookAt, cameraUp, cameraFieldOfView};

    const Image image{3, 3, nullptr, std::string{}};

    const real imageDimension = 2 * (std::sqrt(2.0) - 1);   // 2 * tan(fov / 2)
    const Vector imageCentre = camera.eye + normalise(camera.lookAt - camera.eye);

    for (int x = 0; x < image.width; ++x)
    {
        for (int y = 0; y < image.height; ++y)
        {
            const Ray ray = rayThroughPixel(camera, x, y, image);
            const Vector imageCentreToPixelCentre{
                (x - 1) * imageDimension / 3.0, (1 - y) * imageDimension / 3.0, 0.0, 1.0
            };
            const Vector pixelCentre = imageCentre + imageCentreToPixelCentre;
            REQUIRE(areEqual(ray.start, Vector{0.0, 0.0, 4.0, 1.0}));
            REQUIRE(areEqual(homogenise(ray.direction), homogenise(normalise(pixelCentre - camera.eye))));
        }
    }
}

// TO-DO: Ellipsoid intersections?
// TO-DO: Scene intersections?
