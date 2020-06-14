#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../Source/input_parsing.h"
#include "test_utils.h"

static bool areEqual(const Triangle& lhs, const Triangle& rhs) noexcept
{
    return areEqual(lhs.a, rhs.a) && areEqual(lhs.b, rhs.b) && areEqual(lhs.c, rhs.c);
}

static bool areEqual(const Colour& lhs, const Colour& rhs) noexcept
{
    return areEqual(lhs.red, rhs.red) && areEqual(lhs.green, rhs.green) && areEqual(lhs.blue, rhs.blue);
}

static bool areEqual(const Material& lhs, const Material& rhs) noexcept
{
    return areEqual(lhs.diffuse, rhs.diffuse) && areEqual(lhs.specular, rhs.specular) &&
        areEqual(lhs.emission, rhs.emission) && areEqual(lhs.shininess, rhs.shininess);
}

static bool areEqual(const AttenuationParameters& lhs, const AttenuationParameters& rhs) noexcept
{
    return areEqual(lhs.constant, rhs.constant) &&
        areEqual(lhs.linear, rhs.linear) && areEqual(lhs.quadratic, rhs.quadratic);
}

static bool areEqual(const Sphere& lhs, const Sphere& rhs) noexcept
{
    return areEqual(lhs.radius, rhs.radius) && areEqual(lhs.centre, rhs.centre);
}

static bool areEqual(const Ellipsoid& lhs, const Ellipsoid& rhs) noexcept
{
    return areEqual(lhs.sphere, rhs.sphere) && areEqual(lhs.inverseTransform, rhs.inverseTransform);
}

TEST_CASE("parseInputFile", "[parseInputFile]")
{
    SECTION("scene_1")
    {
        const std::variant<FileInfo, const char*> fileParsingResult = parseInputFile("./scene1.test");
        REQUIRE(std::holds_alternative<FileInfo>(fileParsingResult));

        const FileInfo& fileInfo = std::get<FileInfo>(fileParsingResult);

        const Image& image = fileInfo.image;
        REQUIRE(image.width == 640);
        REQUIRE(image.height == 480);
        REQUIRE(image.pixels != nullptr);
        REQUIRE(image.filename == "raytrace.png");

        const Camera& camera = fileInfo.camera;
        REQUIRE(areEqual(camera.eye, Vector{-4.0, -4.0, 4.0, 1.0}));
        REQUIRE(areEqual(camera.lookAt, Vector{1.0, 0.0, 0.0, 1.0}));
        REQUIRE(areEqual(camera.up, Vector{0.0, 1.0, 0.0, 1.0}));
        REQUIRE(camera.fieldOfView.x == 40);
        REQUIRE(camera.fieldOfView.y == 30);

        const Scene& scene = fileInfo.scene;

        const std::array<Vector, 4> vertices {
            Vector{-1.0, -1.0, 0.0, 1.0},
            Vector{1.0, -1.0, 0.0, 1.0},
            Vector{1.0, 1.0, 0.0, 1.0},
            Vector{-1.0, 1.0, 0.0, 1.0}
        };

        const std::array<Triangle, 2> expectedTriangles {
            Triangle{vertices[0], vertices[1], vertices[2]},
            Triangle{vertices[0], vertices[2], vertices[3]}
        };

        const std::vector<Triangle>& triangles = scene.triangles;
        REQUIRE(triangles.size() == expectedTriangles.size());
        for (std::size_t i = 0; i < triangles.size(); ++i)
            REQUIRE(areEqual(triangles[i], expectedTriangles[i]));

        const Material expectedMaterial {
            Colour{1.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            0.0
        };

        const std::vector<Material>& triangleMaterials = scene.triangleMaterials;
        REQUIRE(triangleMaterials.size() == expectedTriangles.size());
        for (const Material& triangleMaterial : triangleMaterials)
            REQUIRE(areEqual(triangleMaterial, expectedMaterial));

        const Colour expectedAmbient{0.1, 0.1, 0.1};

        const std::vector<Colour>& triangleAmbients = scene.triangleAmbients;
        REQUIRE(triangleAmbients.size() == expectedTriangles.size());
        for (const Colour& triangleAmbient : triangleAmbients)
            REQUIRE(areEqual(triangleAmbient, expectedAmbient));

        REQUIRE(scene.ellipsoids.empty());
        REQUIRE(scene.ellipsoidTransforms.empty());
        REQUIRE(scene.ellipsoidMaterials.empty());
        REQUIRE(scene.ellipsoidAmbients.empty());

        REQUIRE(scene.directionalLightSource.has_value());
        const DirectionalLightSource& directionalLightSource = scene.directionalLightSource.value();
        REQUIRE(areEqual(directionalLightSource.direction, Vector{0.0, 0.0, 1.0, 1.0}));
        REQUIRE(areEqual(directionalLightSource.colour, Colour{0.5, 0.5, 0.5}));
        REQUIRE(areEqual(directionalLightSource.attenuationParameters, AttenuationParameters{1.0, 0.0, 0.0}));

        REQUIRE(scene.pointLightSources.size() == 1);
        const PointLightSource& pointLightSource = scene.pointLightSources.front();
        REQUIRE(areEqual(pointLightSource.position, Vector{4.0, 0.0, 4.0, 1.0}));
        REQUIRE(areEqual(pointLightSource.colour, Colour{0.5, 0.5, 0.5}));
        REQUIRE(areEqual(pointLightSource.attenuationParameters, AttenuationParameters{0.0, 0.0, 1.0}));
    }

    SECTION("scene_2")
    {
        const std::variant<FileInfo, const char*> fileParsingResult = parseInputFile("./scene2.test");
        REQUIRE(std::holds_alternative<FileInfo>(fileParsingResult));

        const FileInfo& fileInfo = std::get<FileInfo>(fileParsingResult);

        const Image& image = fileInfo.image;
        REQUIRE(image.width == 640);
        REQUIRE(image.height == 480);
        REQUIRE(image.pixels != nullptr);
        REQUIRE(image.filename == "raytrace.png");

        const Camera& camera = fileInfo.camera;
        REQUIRE(areEqual(camera.eye, Vector{-2.0, -2.0, -2.0, 1.0}));
        REQUIRE(areEqual(camera.lookAt, Vector{0.0, 0.0, 0.0, 1.0}));
        REQUIRE(areEqual(camera.up, Vector{-1.0, -1.0, 2.0, 1.0}));
        REQUIRE(camera.fieldOfView.x == 80);
        REQUIRE(camera.fieldOfView.y == 60);

        const Scene& scene = fileInfo.scene;

        const std::array<Vector, 8> vertices {
            Vector{-1.0, -1.0, -1.0, 1.0},
            Vector{1.0, -1.0, -1.0, 1.0},
            Vector{1.0, 1.0, -1.0, 1.0},
            Vector{-1.0, 1.0, -1.0, 1.0},
            Vector{-1.0, -1.0, 1.0, 1.0},
            Vector{1.0, -1.0, 1.0, 1.0},
            Vector{1.0, 1.0, 1.0, 1.0},
            Vector{-1.0, 1.0, 1.0, 1.0}
        };

        const std::array<Triangle, 12> expectedTriangles {
            Triangle{vertices[0], vertices[1], vertices[5]},
            Triangle{vertices[0], vertices[5], vertices[4]},

            Triangle{vertices[3], vertices[7], vertices[6]},
            Triangle{vertices[3], vertices[6], vertices[2]},

            Triangle{vertices[1], vertices[2], vertices[6]},
            Triangle{vertices[1], vertices[6], vertices[5]},

            Triangle{vertices[0], vertices[7], vertices[3]},
            Triangle{vertices[0], vertices[4], vertices[7]},

            Triangle{vertices[0], vertices[3], vertices[2]},
            Triangle{vertices[0], vertices[2], vertices[1]},

            Triangle{vertices[4], vertices[5], vertices[6]},
            Triangle{vertices[4], vertices[6], vertices[7]}
        };

        const std::vector<Triangle>& triangles = scene.triangles;
        REQUIRE(triangles.size() == expectedTriangles.size());
        for (std::size_t i = 0; i < triangles.size(); ++i)
            REQUIRE(areEqual(triangles[i], expectedTriangles[i]));

        const Material expectedMaterial {
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            0.0
        };

        const std::vector<Material>& triangleMaterials = scene.triangleMaterials;
        REQUIRE(triangleMaterials.size() == expectedTriangles.size());
        for (const Material& triangleMaterial : triangleMaterials)
            REQUIRE(areEqual(triangleMaterial, expectedMaterial));

        const std::array<Colour, 12> expectedTriangleAmbients {
            Colour{0.5, 0.0, 0.5},
            Colour{0.5, 0.0, 0.5},

            Colour{0.5, 1.0, 0.5},
            Colour{0.5, 1.0, 0.5},

            Colour{1.0, 0.5, 0.5},
            Colour{1.0, 0.5, 0.5},

            Colour{0.0, 0.5, 0.5},
            Colour{0.0, 0.5, 0.5},

            Colour{0.5, 0.5, 0.0},
            Colour{0.5, 0.5, 0.0},

            Colour{0.5, 0.5, 1.0},
            Colour{0.5, 0.5, 1.0}
        };

        const std::vector<Colour>& triangleAmbients = scene.triangleAmbients;
        REQUIRE(triangleAmbients.size() == expectedTriangles.size());
        for (std::size_t i = 0; i < triangleAmbients.size(); ++i)
            REQUIRE(areEqual(triangleAmbients[i], expectedTriangleAmbients[i]));

        const std::array<Ellipsoid, 21> expectedEllipsoids {
            Ellipsoid{Sphere{Vector{1.0, 0.0, 0.0, 1.0}, 0.15}, identityMatrix()},

            Ellipsoid{Sphere{Vector{-0.5, 1.0, -0.5, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{0.5, 1.0, 0.5, 1.0}, 0.15}, identityMatrix()},

            Ellipsoid{Sphere{Vector{0.0, 0.0, 1.0, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{-0.5, -0.5, 1.0, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{0.5, 0.5, 1.0, 1.0}, 0.15}, identityMatrix()},

            Ellipsoid{Sphere{Vector{-1.0, -0.5, -0.5, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{-1.0, -0.5, 0.5, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{-1.0, 0.5, 0.5, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{-1.0, 0.5, -0.5, 1.0}, 0.15}, identityMatrix()},

            Ellipsoid{Sphere{Vector{-0.5, -1.0, -0.5, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{-0.5, -1.0, 0.5, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{0.5, -1.0, 0.5, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{0.5, -1.0, -0.5, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{0.0, -1.0, 0.0, 1.0}, 0.15}, identityMatrix()},

            Ellipsoid{Sphere{Vector{-0.5, -0.5, -1.0, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{-0.5, 0.0, -1.0, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{-0.5, 0.5, -1.0, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{0.5, -0.5, -1.0, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{0.5, 0.0, -1.0, 1.0}, 0.15}, identityMatrix()},
            Ellipsoid{Sphere{Vector{0.5, 0.5, -1.0, 1.0}, 0.15}, identityMatrix()}
        };

        const std::vector<Ellipsoid>& ellipsoids = scene.ellipsoids;
        REQUIRE(ellipsoids.size() == expectedEllipsoids.size());
        for (std::size_t i = 0; i < ellipsoids.size(); ++i)
            REQUIRE(areEqual(ellipsoids[i], expectedEllipsoids[i]));

        const std::vector<Matrix>& ellipsoidTransforms = scene.ellipsoidTransforms;
        REQUIRE(ellipsoidTransforms.size() == expectedEllipsoids.size());
        for (const Matrix& ellipsoidTransform : ellipsoidTransforms)
            REQUIRE(areEqual(ellipsoidTransform, identityMatrix()));

        const std::vector<Material>& ellipsoidMaterials = scene.ellipsoidMaterials;
        for (const Material& ellipsoidMaterial : ellipsoidMaterials)
            REQUIRE(areEqual(ellipsoidMaterial, expectedMaterial));

        const Colour expectedEllipsoidAmbient{1.0, 1.0, 1.0};

        const std::vector<Colour>& ellipsoidAmbients = scene.ellipsoidAmbients;
        REQUIRE(ellipsoidAmbients.size() == expectedEllipsoids.size());
        for (const Colour& ellipsoidAmbient : ellipsoidAmbients)
            REQUIRE(areEqual(ellipsoidAmbient, expectedEllipsoidAmbient));

        REQUIRE(!scene.directionalLightSource.has_value());
        REQUIRE(scene.pointLightSources.empty());
    }

    SECTION("scene_3")
    {
        const std::variant<FileInfo, const char*> fileParsingResult = parseInputFile("./scene3.test");
        REQUIRE(std::holds_alternative<FileInfo>(fileParsingResult));

        const FileInfo& fileInfo = std::get<FileInfo>(fileParsingResult);

        const Image& image = fileInfo.image;
        REQUIRE(image.width == 640);
        REQUIRE(image.height == 480);
        REQUIRE(image.pixels != nullptr);
        REQUIRE(image.filename == "scene.png");

        const Camera& camera = fileInfo.camera;
        REQUIRE(areEqual(camera.eye, Vector{0.0, -4.0, 4.0, 1.0}));
        REQUIRE(areEqual(camera.lookAt, Vector{0.0, -1.0, 0.0, 1.0}));
        REQUIRE(areEqual(camera.up, Vector{0.0, 1.0, 1.0, 1.0}));
        REQUIRE(camera.fieldOfView.x == 60);
        REQUIRE(camera.fieldOfView.y == 45);

        const Scene& scene = fileInfo.scene;

        REQUIRE(scene.triangles.empty());
        REQUIRE(scene.triangleMaterials.empty());
        REQUIRE(scene.triangleAmbients.empty());

        const std::array<Matrix, 6> expectedEllipsoidTransforms {
            translationMatrix(0.0, 0.0, 0.5) * rotationMatrix(45.0, 0.0, 0.0, 1.0) * scalingMatrix(1.0, 0.25, 0.25),
            translationMatrix(0.0, 0.0, 0.5) * rotationMatrix(-45.0, 0.0, 0.0, 1.0) * scalingMatrix(1.0, 0.25, 0.25),
            translationMatrix(-1.5, -0.8, 0.65) * scalingMatrix(0.4, 0.4, 0.4),
            translationMatrix(1.5, -0.8, 0.65) * scalingMatrix(0.4, 0.4, 0.4),
            translationMatrix(1.5, 0.8, 0.65) * scalingMatrix(0.4, 0.4, 0.4),
            translationMatrix(-1.5, 0.8, 0.65) * scalingMatrix(0.4, 0.4, 0.4)
        };

        const std::array<Matrix, 6> expectedEllipsoidInverseTransforms {
            scalingMatrix(1.0, 4.0, 4.0) * rotationMatrix(-45.0, 0.0, 0.0, 1.0) * translationMatrix(0.0, 0.0, -0.5),
            scalingMatrix(1.0, 4.0, 4.0) * rotationMatrix(45.0, 0.0, 0.0, 1.0) * translationMatrix(0.0, 0.0, -0.5),
            scalingMatrix(2.5, 2.5, 2.5) * translationMatrix(1.5, 0.8, -0.65),
            scalingMatrix(2.5, 2.5, 2.5) * translationMatrix(-1.5, 0.8, -0.65),
            scalingMatrix(2.5, 2.5, 2.5) * translationMatrix(-1.5, -0.8, -0.65),
            scalingMatrix(2.5, 2.5, 2.5) * translationMatrix(1.5, -0.8, -0.65)
        };

        const std::array<Ellipsoid, 6> expectedEllipsoids {
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expectedEllipsoidInverseTransforms[0]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expectedEllipsoidInverseTransforms[1]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expectedEllipsoidInverseTransforms[2]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expectedEllipsoidInverseTransforms[3]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expectedEllipsoidInverseTransforms[4]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expectedEllipsoidInverseTransforms[5]}
        };

        const std::vector<Ellipsoid>& ellipsoids = scene.ellipsoids;
        REQUIRE(ellipsoids.size() == expectedEllipsoids.size());
        for (std::size_t i = 0; i < ellipsoids.size(); ++i)
            REQUIRE(areEqual(ellipsoids[i], expectedEllipsoids[i]));

        const std::vector<Matrix>& ellipsoidTransforms = scene.ellipsoidTransforms;
        REQUIRE(ellipsoidTransforms.size() == expectedEllipsoidTransforms.size());
        for (std::size_t i = 0; i < ellipsoidTransforms.size(); ++i)
            REQUIRE(areEqual(ellipsoidTransforms[i], expectedEllipsoidTransforms[i]));

        const Material expectedMaterial {
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            0.0
        };

        const std::vector<Material>& ellipsoidMaterials = scene.ellipsoidMaterials;
        for (const Material& ellipsoidMaterial : ellipsoidMaterials)
            REQUIRE(areEqual(ellipsoidMaterial, expectedMaterial));

        const std::array<Colour, 6> expectedEllipsoidAmbients{ 
            Colour{0.0, 1.0, 0.0},
            Colour{1.0, 0.0, 0.0},
            Colour{0.0, 1.0, 1.0},
            Colour{0.0, 1.0, 1.0},
            Colour{0.0, 1.0, 1.0},
            Colour{0.0, 1.0, 1.0}
        };

        const std::vector<Colour>& ellipsoidAmbients = scene.ellipsoidAmbients;
        REQUIRE(ellipsoidAmbients.size() == expectedEllipsoidAmbients.size());
        for (std::size_t i = 0; i < ellipsoidAmbients.size(); ++i)
            REQUIRE(areEqual(ellipsoidAmbients[i], expectedEllipsoidAmbients[i]));

        REQUIRE(!scene.directionalLightSource.has_value());
        REQUIRE(scene.pointLightSources.empty());
    }
}
