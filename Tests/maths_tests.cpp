#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../Source/maths.h"
#include "test_utils.h"

TEST_CASE("floating_point_comparison", "[floating_point_comparison]") {
    SECTION("are_equal") {
        REQUIRE(are_equal(2.0, 2.0000001));
        REQUIRE(!are_equal(2.0, 2.000001));
    }

    SECTION("less_than") {
        REQUIRE(less_than(2.0, 2.000001));
        REQUIRE(!less_than(2.0, 2.0000001));
        REQUIRE(less_than(-5.0, 1.0));
        REQUIRE(!less_than(2.0, -5.0));
    }

    SECTION("greater_than") {
        REQUIRE(greater_than(2.000001, 2.0));
        REQUIRE(!greater_than(2.0000001, 2.0));
        REQUIRE(greater_than(1.0, -5.0));
        REQUIRE(!greater_than(-5.0, 2.0));
    }
}


TEST_CASE("angle_conversion", "[angle_conversion]") {
    SECTION("to_radians") {
        REQUIRE(are_equal(to_radians(0.0), 0.0));
        REQUIRE(are_equal(to_radians(90.0), pi / 2.0));
        REQUIRE(are_equal(to_radians(180.0), pi));
        REQUIRE(are_equal(to_radians(270.0), 3.0 * pi / 2.0));
        REQUIRE(are_equal(to_radians(360.0), 2.0 * pi));
        REQUIRE(are_equal(to_radians(-90.0), -pi / 2.0));
    }
}


static constexpr Vector i{1.0, 0.0, 0.0, 1.0};
static constexpr Vector j{0.0, 1.0, 0.0, 1.0};
static constexpr Vector k{0.0, 0.0, 1.0, 1.0};

TEST_CASE("Vector", "[Vector]") {
    SECTION("operator+") {
        const Vector v1{1.0, 2.0, 3.0, 1.0};
        const Vector v2{4.0, 5.0, 6.0, 1.0};
        const Vector v3 = v1 + v2;
        REQUIRE(are_equal(v3, Vector{5.0, 7.0, 9.0, 1.0}));

        const Vector v4{2.0, 6.0, 4.0, 2.0};
        const Vector v5{18.0, 12.0, 24.0, 3.0};
        const Vector v6 = v4 + v5;
        REQUIRE(are_equal(v6, Vector{42.0, 42.0, 60.0, 6.0}));
    }

    SECTION("operator-") {
        const Vector v1{1.0, 2.0, 3.0, 1.0};
        const Vector v2{4.0, 5.0, 6.0, 1.0};
        const Vector v3 = v1 - v2;
        REQUIRE(are_equal(v3, Vector{-3.0, -3.0, -3.0, 1.0}));

        const Vector v4{2.0, 6.0, 4.0, 2.0};
        const Vector v5{18.0, 12.0, 24.0, 3.0};
        const Vector v6 = v4 - v5;
        REQUIRE(are_equal(v6, Vector{-30.0, -6.0, -36.0, 6.0}));
    }

    SECTION("operator*") {
        SECTION("dot_product") {
            REQUIRE(are_equal(i * i, 1.0));
            REQUIRE(are_equal(j * j, 1.0));
            REQUIRE(are_equal(k * k, 1.0));
            REQUIRE(are_equal(i * j, 0.0));
            REQUIRE(are_equal(j * k, 0.0));
            REQUIRE(are_equal(k * i, 0.0));

            const Vector v1{1.0, -0.5, 3.0, 1.0};
            const Vector v2{-3.2, 2.2, 1.5, 1.0};
            const real r1 = v1 * v2;
            REQUIRE(are_equal(r1, 0.2));

            const Vector v3{2.0, 5.0, 4.0, 0.5};
            const Vector v4{0.5, 1.0, -0.5, 2.0};
            const real r2 = v3 * v4;
            REQUIRE(are_equal(r2, 4.0));        
        }

        SECTION("scaling") {
            const Vector v5{-13.0, 5.5, 7.2, 1.0};
            const real s1 = 2.0;
            const Vector sv1 = s1 * v5;
            REQUIRE(are_equal(sv1, Vector{-13.0, 5.5, 7.2, 0.5}));
        }
    }

    SECTION("operator^") {
        REQUIRE(are_equal(i ^ j, k));
        REQUIRE(are_equal(j ^ k, i));
        REQUIRE(are_equal(k ^ i, j));
        REQUIRE(are_equal(j ^ i, Vector{0.0, 0.0, -1.0, 1.0}));
        REQUIRE(are_equal(k ^ j, Vector{-1.0, 0.0, 0.0, 1.0}));
        REQUIRE(are_equal(i ^ k, Vector{0.0, -1.0, 0.0, 1.0}));

        const Vector v1{-15.0, 6.0, 3.0, 1.0};
        const Vector v2{2.5, 13.4, 6.5, 1.0};
        const Vector v3 = v1 ^ v2;
        REQUIRE(are_equal(v3, Vector{-1.2, 105.0, -216.0, 1.0}));

        const Vector v4{2.8, -1.3, 4.4, 3.0};
        const Vector v5{7.7, 4.8, -5.1, -2.0};
        const Vector v6 = v4 ^ v5;
        REQUIRE(are_equal(v6, Vector{-14.49, 48.16, 23.45, -6.0}));
    }

    SECTION("operator/") {
        const Vector v1{1.0, 0.0, 1.0, 1.0};
        const real d1 = std::sqrt(2.0);
        const Vector dv1 = v1 / d1;
        REQUIRE(are_equal(dv1, Vector{1.0, 0.0, 1.0, d1}));

        const Vector v2{-5.0, 15.0, 25.0, 100.0};
        const real d2 = 5.0;
        const Vector dv2 = v2 / d2;
        REQUIRE(are_equal(dv2, Vector{-5.0, 15.0, 25.0, 500.0}));
    }

    SECTION("magnitude") {
        const Vector v1{3.0, 4.0, 5.0, 1.0};
        const real norm_v1 = magnitude(v1);
        REQUIRE(are_equal(norm_v1, std::sqrt(50.0)));

        const Vector v2{-7.0, 3.0, -6.0, -2.0};
        const real norm_v2 = magnitude(v2);
        REQUIRE(are_equal(norm_v2, std::sqrt(23.5)));
    }

    SECTION("normalise") {
        const Vector v1{5.0, 0.0, 0.0, 1.0};
        const Vector normalised_v1 = normalise(v1);
        REQUIRE(are_equal(normalised_v1, Vector{5.0, 0.0, 0.0, 5.0}));

        const Vector v2{2.0, 0.0, 2.0, 2.0};
        const Vector normalised_v2 = normalise(v2);
        REQUIRE(are_equal(normalised_v2, Vector{2.0, 0.0, 2.0, 2.0 * std::sqrt(2.0)}));

        const Vector v3{0.0, 0.0, 16.0, 4.0};
        const Vector normalised_v3 = normalise(v3);
        REQUIRE(are_equal(normalised_v3, Vector{0.0, 0.0, 16.0, 16.0}));

        const Vector v4{1.0, 2.0, 3.0, 3.0};
        const Vector normalised_v4 = normalise(v4);
        REQUIRE(are_equal(normalised_v4, Vector{1.0, 2.0, 3.0, std::sqrt(14.0)}));
    }

    SECTION("homogenise") {
        const Vector v1{2.0, 2.0, 2.0, 2.0};
        const Vector homogenised_v1 = homogenise(v1);
        REQUIRE(are_equal(homogenised_v1, Vector{1.0, 1.0, 1.0, 1.0}));

        const Vector v2{1.0, 2.0, 3.0, 6.0};
        const Vector homogenised_v2 = homogenise(v2);
        REQUIRE(are_equal(homogenised_v2, Vector{1.0 / 6.0, 2.0 / 6.0, 3.0 / 6.0, 1.0}));
    }
}

TEST_CASE("Matrix", "[Matrix]") {
    SECTION("scaling_matrix") {
        const Matrix m = scaling_matrix(1.0, 2.0, 3.0);
        const Matrix answer = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 2.0, 0.0, 0.0,
            0.0, 0.0, 3.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        };
        REQUIRE(are_equal(m, answer));
    }

    SECTION("translation_matrix") {
        const Matrix m = translation_matrix(4.0, 5.0, 6.0);
        const Matrix answer = {
            1.0, 0.0, 0.0, 4.0,
            0.0, 1.0, 0.0, 5.0,
            0.0, 0.0, 1.0, 6.0,
            0.0, 0.0, 0.0, 1.0
        };
        REQUIRE(are_equal(m, answer));
    }

    SECTION("rotation_matrix") {
        const Matrix m1 = rotation_matrix(to_radians(90.0), 5.0, 0.0, 0.0);
        const Matrix answer1 = {
            1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, -1.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        };
        REQUIRE(are_equal(m1, answer1));

        const Matrix m2 = rotation_matrix(to_radians(45.0), 1.0, 1.0, 0.0);
        const real root_2_inverse = 1.0 / std::sqrt(2.0);
        const Matrix answer2 = {
            0.5 * (1.0 + root_2_inverse), 0.5 * (1.0 - root_2_inverse),          0.5, 0.0,
            0.5 * (1.0 - root_2_inverse), 0.5 * (1.0 + root_2_inverse),         -0.5, 0.0,
                                  -0.5,                        0.5, root_2_inverse, 0.0,
                                   0.0,                        0.0,          0.0, 1.0
        };
        REQUIRE(are_equal(m2, answer2));
    }

    SECTION("operator*") {
        SECTION("Vector") {
            const Vector v1{1.0, 2.0, 3.0, 1.0};
            const Vector v2 = identity_matrix() * v1;
            REQUIRE(are_equal(v1, v2));

            const Vector v3 = identity_matrix() * v1;
            REQUIRE(are_equal(v1, v3));

            const Vector v4 = scaling_matrix(1.0, 1.0 / 2.0, 1.0 / 3.0) * v1;
            REQUIRE(are_equal(v4, Vector{1.0, 1.0, 1.0, 1.0}));
            
            const Matrix rotate_45_about_x = rotation_matrix(pi / 4.0, 1.0, 0.0, 0.0);
            const Vector v5 = rotate_45_about_x * i;
            REQUIRE(are_equal(v5, i));

            const real root_2_inverse = 1.0 / std::sqrt(2.0);
            const Vector v6 = rotate_45_about_x * j;
            REQUIRE(are_equal(v6, Vector{0.0, root_2_inverse, root_2_inverse, 1.0}));

            const Vector v7 = rotate_45_about_x * k;
            REQUIRE(are_equal(v7, Vector{0.0, -root_2_inverse, root_2_inverse, 1.0}));
        }

        SECTION("Matrix") {
            const Matrix m1 = scaling_matrix(10.0, 2.0, 5.0) * scaling_matrix(0.1, 0.5, 0.2);
            const Matrix answer1 = identity_matrix();
            REQUIRE(are_equal(m1, answer1));

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
            REQUIRE(are_equal(m2 * m2, answer2));
        }
    }
}


TEST_CASE("shape_normals", "[shape_normals]") {
    SECTION("Triangle") {
        const Vector a{-0.5, -0.5, 0.0, 1.0};
        const Vector b{0.5, -0.5, 0.0, 1.0};
        const Vector c{0.5, 0.5, 0.0, 1.0};

        const Triangle anti_clockwise_triangle{a, b, c};
        const Vector anti_clockwise_unit_surface_normal = unit_surface_normal(anti_clockwise_triangle);
        REQUIRE(are_equal(anti_clockwise_unit_surface_normal, k));

        const Triangle clockwise_triangle{a, c, b};
        const Vector clockwise_unit_surface_normal = unit_surface_normal(clockwise_triangle);
        REQUIRE(are_equal(clockwise_unit_surface_normal, Vector{0.0, 0.0, -1.0, 1.0}));
    }

    SECTION("Sphere") {
        const Vector centre{1.0, 2.0, 3.0, 1.0};
        const real radius = 3.0;
        const Sphere sphere{centre, radius};

        const Vector unit_normal1 = unit_surface_normal(sphere, centre + i);
        REQUIRE(are_equal(unit_normal1, i));

        const Vector unit_normal2 = unit_surface_normal(sphere, centre - i);
        REQUIRE(are_equal(unit_normal2, Vector{-1.0, 0.0, 0.0, 1.0}));

        const Vector unit_normal3 = unit_surface_normal(sphere, centre + j);
        REQUIRE(are_equal(unit_normal3, j));

        const Vector unit_normal4 = unit_surface_normal(sphere, centre - j);
        REQUIRE(are_equal(unit_normal4, Vector{0.0, -1.0, 0.0, 1.0}));

        const Vector unit_normal5 = unit_surface_normal(sphere, centre + k);
        REQUIRE(are_equal(unit_normal5, k));

        const Vector unit_normal6 = unit_surface_normal(sphere, centre - k);
        REQUIRE(are_equal(unit_normal6, Vector{0.0, 0.0, -1.0, 1.0}));

        const Vector unit_normal7 = unit_surface_normal(sphere, centre + i + j + k);
        REQUIRE(are_equal(unit_normal7, normalise(i + j + k)));
    }

    SECTION("Ellipsoid") {
        const Vector origin{0.0, 0.0, 0.0, 1.0};
        const real radius = 2.0;

        const real x_scale = 3.0;
        const real y_scale = 2.0;
        const real z_scale = 1.0;
        const Matrix scale = scaling_matrix(x_scale, y_scale, z_scale);
        const Matrix scale_inverse = scaling_matrix(1.0 / x_scale, 1.0 / y_scale, 1.0 / z_scale);

        const real angle = to_radians(90.0);
        const Matrix rotate = rotation_matrix(angle, 0.0, 0.0, 1.0);
        const Matrix rotate_inverse = rotation_matrix(-angle, 0.0, 0.0, 1.0);
        
        const real x_shift = 4.0;
        const real y_shift = 5.0;
        const real z_shift = 6.0;
        const Matrix translate = translation_matrix(x_shift, y_shift, z_shift);
        const Matrix translate_inverse = translation_matrix(-x_shift, -y_shift, -z_shift);        
        
        const Matrix transform = translate * rotate * scale;
        const Matrix transform_inverse = scale_inverse * rotate_inverse * translate_inverse;

        const Sphere sphere{origin, radius};
        const Ellipsoid ellipsoid{sphere, transform_inverse};

        const Vector unit_normal1 = unit_surface_normal(ellipsoid, transform * i);
        REQUIRE(are_equal(homogenise(unit_normal1), j));

        const Vector unit_normal2 = unit_surface_normal(ellipsoid, transform * j);
        REQUIRE(are_equal(homogenise(unit_normal2), homogenise(-1.0 * i)));

        const Vector unit_normal3 = unit_surface_normal(ellipsoid, transform * k);
        REQUIRE(are_equal(homogenise(unit_normal3), k));
    }
}
