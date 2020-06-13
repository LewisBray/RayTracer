#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../Source/maths.h"
#include "test_utils.h"

TEST_CASE("floating_point_comparison", "[floating_point_comparison]")
{
    SECTION("areEqual")
    {
        REQUIRE(areEqual(2.0, 2.0000001));
        REQUIRE(!areEqual(2.0, 2.000001));
    }

    SECTION("lessThan")
    {
        REQUIRE(lessThan(2.0, 2.000001));
        REQUIRE(!lessThan(2.0, 2.0000001));
        REQUIRE(lessThan(-5.0, 1.0));
        REQUIRE(!lessThan(2.0, -5.0));
    }

    SECTION("greaterThan")
    {
        REQUIRE(greaterThan(2.000001, 2.0));
        REQUIRE(!greaterThan(2.0000001, 2.0));
        REQUIRE(greaterThan(1.0, -5.0));
        REQUIRE(!greaterThan(-5.0, 2.0));
    }
}


TEST_CASE("angle_conversion", "[angle_conversion]")
{
    SECTION("toRadians")
    {
        REQUIRE(areEqual(toRadians(0.0), 0.0));
        REQUIRE(areEqual(toRadians(90.0), pi / 2.0));
        REQUIRE(areEqual(toRadians(180.0), pi));
        REQUIRE(areEqual(toRadians(270.0), 3.0 * pi / 2.0));
        REQUIRE(areEqual(toRadians(360.0), 2.0 * pi));
        REQUIRE(areEqual(toRadians(-90.0), -pi / 2.0));
    }
}


static constexpr Vector i{1.0, 0.0, 0.0, 1.0};
static constexpr Vector j{0.0, 1.0, 0.0, 1.0};
static constexpr Vector k{0.0, 0.0, 1.0, 1.0};

TEST_CASE("Vector", "[Vector]")
{
    SECTION("operator+")
    {
        const Vector v1{1.0, 2.0, 3.0, 1.0};
        const Vector v2{4.0, 5.0, 6.0, 1.0};
        const Vector v3 = v1 + v2;
        REQUIRE(areEqual(v3, Vector{5.0, 7.0, 9.0, 1.0}));

        const Vector v4{2.0, 6.0, 4.0, 2.0};
        const Vector v5{18.0, 12.0, 24.0, 3.0};
        const Vector v6 = v4 + v5;
        REQUIRE(areEqual(v6, Vector{42.0, 42.0, 60.0, 6.0}));
    }

    SECTION("operator-")
    {
        const Vector v1{1.0, 2.0, 3.0, 1.0};
        const Vector v2{4.0, 5.0, 6.0, 1.0};
        const Vector v3 = v1 - v2;
        REQUIRE(areEqual(v3, Vector{-3.0, -3.0, -3.0, 1.0}));

        const Vector v4{2.0, 6.0, 4.0, 2.0};
        const Vector v5{18.0, 12.0, 24.0, 3.0};
        const Vector v6 = v4 - v5;
        REQUIRE(areEqual(v6, Vector{-30.0, -6.0, -36.0, 6.0}));
    }

    SECTION("operator*")
    {
        SECTION("dot product")
        {
            REQUIRE(areEqual(i * i, 1.0));
            REQUIRE(areEqual(j * j, 1.0));
            REQUIRE(areEqual(k * k, 1.0));
            REQUIRE(areEqual(i * j, 0.0));
            REQUIRE(areEqual(j * k, 0.0));
            REQUIRE(areEqual(k * i, 0.0));

            const Vector v1{1.0, -0.5, 3.0, 1.0};
            const Vector v2{-3.2, 2.2, 1.5, 1.0};
            const real r1 = v1 * v2;
            REQUIRE(areEqual(r1, 0.2));

            const Vector v3{2.0, 5.0, 4.0, 0.5};
            const Vector v4{0.5, 1.0, -0.5, 2.0};
            const real r2 = v3 * v4;
            REQUIRE(areEqual(r2, 4.0));        
        }

        SECTION("scaling")
        {
            const Vector v5{-13.0, 5.5, 7.2, 1.0};
            const real s1 = 2.0;
            const Vector sv1 = s1 * v5;
            REQUIRE(areEqual(sv1, Vector{-13.0, 5.5, 7.2, 0.5}));
        }
    }

    SECTION("operator^")
    {
        REQUIRE(areEqual(i ^ j, k));
        REQUIRE(areEqual(j ^ k, i));
        REQUIRE(areEqual(k ^ i, j));
        REQUIRE(areEqual(j ^ i, Vector{0.0, 0.0, -1.0, 1.0}));
        REQUIRE(areEqual(k ^ j, Vector{-1.0, 0.0, 0.0, 1.0}));
        REQUIRE(areEqual(i ^ k, Vector{0.0, -1.0, 0.0, 1.0}));

        const Vector v1{-15.0, 6.0, 3.0, 1.0};
        const Vector v2{2.5, 13.4, 6.5, 1.0};
        const Vector v3 = v1 ^ v2;
        REQUIRE(areEqual(v3, Vector{-1.2, 105.0, -216.0, 1.0}));

        const Vector v4{2.8, -1.3, 4.4, 3.0};
        const Vector v5{7.7, 4.8, -5.1, -2.0};
        const Vector v6 = v4 ^ v5;
        REQUIRE(areEqual(v6, Vector{-14.49, 48.16, 23.45, -6.0}));
    }

    SECTION("operator/")
    {
        const Vector v1{1.0, 0.0, 1.0, 1.0};
        const real d1 = std::sqrt(2.0);
        const Vector dv1 = v1 / d1;
        REQUIRE(areEqual(dv1, Vector{1.0, 0.0, 1.0, d1}));

        const Vector v2{-5.0, 15.0, 25.0, 100.0};
        const real d2 = 5.0;
        const Vector dv2 = v2 / d2;
        REQUIRE(areEqual(dv2, Vector{-5.0, 15.0, 25.0, 500.0}));
    }

    SECTION("magnitude")
    {
        const Vector v1{3.0, 4.0, 5.0, 1.0};
        const real normV1 = magnitude(v1);
        REQUIRE(areEqual(normV1, std::sqrt(50.0)));

        const Vector v2{-7.0, 3.0, -6.0, -2.0};
        const real normV2 = magnitude(v2);
        REQUIRE(areEqual(normV2, std::sqrt(23.5)));
    }

    SECTION("normalise")
    {
        const Vector v1{5.0, 0.0, 0.0, 1.0};
        const Vector normalisedV1 = normalise(v1);
        REQUIRE(areEqual(normalisedV1, Vector{5.0, 0.0, 0.0, 5.0}));

        const Vector v2{2.0, 0.0, 2.0, 2.0};
        const Vector normalisedV2 = normalise(v2);
        REQUIRE(areEqual(normalisedV2, Vector{2.0, 0.0, 2.0, 2.0 * std::sqrt(2.0)}));

        const Vector v3{0.0, 0.0, 16.0, 4.0};
        const Vector normalisedV3 = normalise(v3);
        REQUIRE(areEqual(normalisedV3, Vector{0.0, 0.0, 16.0, 16.0}));

        const Vector v4{1.0, 2.0, 3.0, 3.0};
        const Vector normalisedV4 = normalise(v4);
        REQUIRE(areEqual(normalisedV4, Vector{1.0, 2.0, 3.0, std::sqrt(14.0)}));
    }

    SECTION("homogenise")
    {
        const Vector v1{2.0, 2.0, 2.0, 2.0};
        const Vector homogenisedV1 = homogenise(v1);
        REQUIRE(areEqual(homogenisedV1, Vector{1.0, 1.0, 1.0, 1.0}));

        const Vector v2{1.0, 2.0, 3.0, 6.0};
        const Vector homogenisedV2 = homogenise(v2);
        REQUIRE(areEqual(homogenisedV2, Vector{1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 1.0}));
    }
}


static bool areEqual(const Matrix& lhs, const Matrix& rhs) noexcept
{
    for (std::size_t row = 0; row < 4; ++row)
        for (std::size_t column = 0; column < 4; ++column)
            if (!areEqual(lhs[row][column], rhs[row][column]))
                return false;
    
    return true;
}

TEST_CASE("Matrix", "[Matrix]")
{
    SECTION("scalingMatrix")
    {
        const Matrix m = scalingMatrix(1.0, 2.0, 3.0);
        const Matrix answer = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 2.0, 0.0, 0.0,
            0.0, 0.0, 3.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        };
        REQUIRE(areEqual(m, answer));
    }

    SECTION("translationMatrix")
    {
        const Matrix m = translationMatrix(4.0, 5.0, 6.0);
        const Matrix answer = {
            1.0, 0.0, 0.0, 4.0,
            0.0, 1.0, 0.0, 5.0,
            0.0, 0.0, 1.0, 6.0,
            0.0, 0.0, 0.0, 1.0
        };
        REQUIRE(areEqual(m, answer));
    }

    SECTION("rotationMatrix")
    {
        const Matrix m1 = rotationMatrix(toRadians(90.0), 5.0, 0.0, 0.0);
        const Matrix answer1 = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, -1.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        };
        REQUIRE(areEqual(m1, answer1));

        const Matrix m2 = rotationMatrix(toRadians(45.0), 1.0, 1.0, 0.0);
        const real root2Inverse = 1.0 / std::sqrt(2.0);
        const Matrix answer2 = {
            0.5 * (1.0 + root2Inverse), 0.5 * (1.0 - root2Inverse),          0.5, 0.0,
            0.5 * (1.0 - root2Inverse), 0.5 * (1.0 + root2Inverse),         -0.5, 0.0,
                                  -0.5,                        0.5, root2Inverse, 0.0,
                                   0.0,                        0.0,          0.0, 1.0
        };
        REQUIRE(areEqual(m2, answer2));
    }

    SECTION("operator*")
    {
        SECTION("Vector")
        {
            const Vector v1{1.0, 2.0, 3.0, 1.0};
            const Vector v2 = identityMatrix() * v1;
            REQUIRE(areEqual(v1, v2));

            const Vector v3 = identityMatrix() * v1;
            REQUIRE(areEqual(v1, v3));

            const Vector v4 = scalingMatrix(1.0, 1.0 / 2.0, 1.0 / 3.0) * v1;
            REQUIRE(areEqual(v4, Vector{1.0, 1.0, 1.0, 1.0}));
            
            const Matrix rotate45AboutX = rotationMatrix(pi / 4.0, 1.0, 0.0, 0.0);
            const Vector v5 = rotate45AboutX * i;
            REQUIRE(areEqual(v5, i));

            const real root2Inverse = 1.0 / std::sqrt(2.0);
            const Vector v6 = rotate45AboutX * j;
            REQUIRE(areEqual(v6, Vector{0.0, root2Inverse, root2Inverse, 1.0}));

            const Vector v7 = rotate45AboutX * k;
            REQUIRE(areEqual(v7, Vector{0.0, -root2Inverse, root2Inverse, 1.0}));
        }

        SECTION("Matrix")
        {
            const Matrix m1 = scalingMatrix(10.0, 2.0, 5.0) * scalingMatrix(0.1, 0.5, 0.2);
            const Matrix answer1 = identityMatrix();
            REQUIRE(areEqual(m1, answer1));

            const Matrix m2 = {
                1.0, 2.0, 3.0, 4.0,
                5.0, 6.0, 7.0, 8.0,
                9.0, 10.0, 11.0, 12.0,
                13.0, 14.0, 15.0, 16.0
            };
            const Matrix answer2 = {
                 90.0, 100.0, 110.0, 120.0,
                202.0, 228.0, 254.0, 280.0,
                314.0, 356.0, 398.0, 440.0,
                426.0, 484.0, 542.0, 600.0
            };
            REQUIRE(areEqual(m2 * m2, answer2));
        }
    }
}


TEST_CASE("shape_normals", "[shape_normals]")
{
    SECTION("Triangle")
    {
        const Vector a{-0.5, -0.5, 0.0, 1.0};
        const Vector b{0.5, -0.5, 0.0, 1.0};
        const Vector c{0.5, 0.5, 0.0, 1.0};

        const Triangle antiClockwiseTriangle{a, b, c};
        const Vector antiClockwiseUnitSurfaceNormal = unitSurfaceNormal(antiClockwiseTriangle);
        REQUIRE(areEqual(antiClockwiseUnitSurfaceNormal, k));

        const Triangle clockwiseTriangle{a, c, b};
        const Vector clockwiseUnitSurfaceNormal = unitSurfaceNormal(clockwiseTriangle);
        REQUIRE(areEqual(clockwiseUnitSurfaceNormal, Vector{0.0, 0.0, -1.0, 1.0}));
    }

    SECTION("Sphere")
    {
        const Vector centre{1.0, 2.0, 3.0, 1.0};
        const real radius = 3.0;
        const Sphere sphere{centre, radius};

        const Vector unitNormal1 = unitSurfaceNormal(sphere, centre + i);
        REQUIRE(areEqual(unitNormal1, i));

        const Vector unitNormal2 = unitSurfaceNormal(sphere, centre - i);
        REQUIRE(areEqual(unitNormal2, Vector{-1.0, 0.0, 0.0, 1.0}));

        const Vector unitNormal3 = unitSurfaceNormal(sphere, centre + j);
        REQUIRE(areEqual(unitNormal3, j));

        const Vector unitNormal4 = unitSurfaceNormal(sphere, centre - j);
        REQUIRE(areEqual(unitNormal4, Vector{0.0, -1.0, 0.0, 1.0}));

        const Vector unitNormal5 = unitSurfaceNormal(sphere, centre + k);
        REQUIRE(areEqual(unitNormal5, k));

        const Vector unitNormal6 = unitSurfaceNormal(sphere, centre - k);
        REQUIRE(areEqual(unitNormal6, Vector{0.0, 0.0, -1.0, 1.0}));

        const Vector unitNormal7 = unitSurfaceNormal(sphere, centre + i + j + k);
        REQUIRE(areEqual(unitNormal7, normalise(i + j + k)));
    }

    SECTION("Ellipsoid")
    {
        const Vector origin{0.0, 0.0, 0.0, 1.0};
        const real radius = 2.0;

        const real xScale = 3.0;
        const real yScale = 2.0;
        const real zScale = 1.0;
        const Matrix scale = scalingMatrix(xScale, yScale, zScale);
        const Matrix scaleInverse = scalingMatrix(1.0 / xScale, 1.0 / yScale, 1.0 / zScale);

        const real angle = 90.0;
        const Matrix rotate = rotationMatrix(0.0, 0.0, 1.0, angle);
        const Matrix rotateInverse = rotationMatrix(0.0, 0.0, 1.0, -angle);
        
        const real xShift = 4.0;
        const real yShift = 5.0;
        const real zShift = 6.0;
        const Matrix translate = translationMatrix(xShift, yShift, zShift);
        const Matrix translateInverse = translationMatrix(-xShift, -yShift, -zShift);        
        
        const Matrix transform = translate * rotate * scale;
        const Matrix transformInverse = scaleInverse * rotateInverse * translateInverse;

        const Sphere sphere{origin, radius};
        const Ellipsoid ellipsoid{sphere, transformInverse};

        const Vector unitNormal3 = unitSurfaceNormal(ellipsoid, transform, transform * i);
        REQUIRE(areEqual(unitNormal3, Vector{xScale, 0.0, 0.0, xScale}));

        const Vector unitNormal2 = unitSurfaceNormal(ellipsoid, transform, transform * j);
        REQUIRE(areEqual(unitNormal2, Vector{0.0, yScale, 0.0, yScale}));

        const Vector unitNormal1 = unitSurfaceNormal(ellipsoid, transform, transform * k);
        REQUIRE(areEqual(unitNormal1, Vector{0.0, 0.0, zScale, zScale}));
    }
}
