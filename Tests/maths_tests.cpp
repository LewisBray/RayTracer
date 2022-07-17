#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../Source/maths.cpp"
#include "test_utils.cpp"

TEST_CASE("floating_point_comparison", "[floating_point_comparison]") {
    SECTION("are_equal") {
        REQUIRE(are_equal(2.0f, 2.0001f));
        REQUIRE(!are_equal(2.0f, 2.001f));
    }

    SECTION("less_than") {
        REQUIRE(less_than(2.0f, 2.001f));
        REQUIRE(!less_than(2.0f, 2.0001f));
        REQUIRE(less_than(-5.0f, 1.0f));
        REQUIRE(!less_than(2.0f, -5.0f));
    }

    SECTION("greater_than") {
        REQUIRE(greater_than(2.001f, 2.0f));
        REQUIRE(!greater_than(2.0001f, 2.0f));
        REQUIRE(greater_than(1.0f, -5.0f));
        REQUIRE(!greater_than(-5.0f, 2.0f));
    }
}


TEST_CASE("angle_conversion", "[angle_conversion]") {
    SECTION("to_radians") {
        REQUIRE(are_equal(to_radians(0.0f), 0.0f));
        REQUIRE(are_equal(to_radians(90.0f), pi / 2.0f));
        REQUIRE(are_equal(to_radians(180.0f), pi));
        REQUIRE(are_equal(to_radians(270.0f), 3.0f * pi / 2.0f));
        REQUIRE(are_equal(to_radians(360.0f), 2.0f * pi));
        REQUIRE(are_equal(to_radians(-90.0f), -pi / 2.0f));
    }
}


static constexpr Vector i{1.0f, 0.0f, 0.0f};
static constexpr Vector j{0.0f, 1.0f, 0.0f};
static constexpr Vector k{0.0f, 0.0f, 1.0f};

TEST_CASE("Vector", "[Vector]") {
    SECTION("operator+") {
        const Vector v1{1.0f, 2.0f, 3.0f};
        const Vector v2{4.0f, 5.0f, 6.0f};
        const Vector v3 = v1 + v2;
        REQUIRE(are_equal(v3, Vector{5.0f, 7.0f, 9.0f}));
    }

    SECTION("operator-") {
        const Vector v1{1.0f, 2.0f, 3.0f};
        const Vector v2{4.0f, 5.0f, 6.0f};
        const Vector v3 = v1 - v2;
        REQUIRE(are_equal(v3, Vector{-3.0f, -3.0f, -3.0f}));
    }

    SECTION("operator*") {
        SECTION("dot_product") {
            REQUIRE(are_equal(i * i, 1.0f));
            REQUIRE(are_equal(j * j, 1.0f));
            REQUIRE(are_equal(k * k, 1.0f));
            REQUIRE(are_equal(i * j, 0.0f));
            REQUIRE(are_equal(j * k, 0.0f));
            REQUIRE(are_equal(k * i, 0.0f));

            const Vector v1{1.0f, -0.5f, 3.0f};
            const Vector v2{-3.2f, 2.2f, 1.5f};
            const float r1 = v1 * v2;
            REQUIRE(are_equal(r1, 0.2f));
        }

        SECTION("scaling") {
            const Vector v5{-13.0f, 5.5f, 7.2f};
            const float s1 = 2.0f;
            const Vector sv1 = s1 * v5;
            REQUIRE(are_equal(sv1, Vector{-26.0f, 11.0f, 14.4f}));
        }
    }

    SECTION("operator^") {
        REQUIRE(are_equal(i ^ j, k));
        REQUIRE(are_equal(j ^ k, i));
        REQUIRE(are_equal(k ^ i, j));
        REQUIRE(are_equal(j ^ i, Vector{0.0f, 0.0f, -1.0f}));
        REQUIRE(are_equal(k ^ j, Vector{-1.0f, 0.0f, 0.0f}));
        REQUIRE(are_equal(i ^ k, Vector{0.0f, -1.0f, 0.0f}));

        const Vector v1{-15.0f, 6.0f, 3.0f};
        const Vector v2{2.5f, 13.4f, 6.5f};
        const Vector v3 = v1 ^ v2;
        REQUIRE(are_equal(v3, Vector{-1.2f, 105.0f, -216.0f}));
    }

    SECTION("operator/") {
        const Vector v1{1.0f, 0.0f, 1.0f};
        const float d1 = std::sqrt(2.0f);
        const Vector dv1 = v1 / d1;
        REQUIRE(are_equal(dv1, Vector{1.0f / d1, 0.0f, 1.0f / d1}));
    }

    SECTION("magnitude") {
        const Vector v1{3.0f, 4.0f, 5.0f};
        const float norm_v1 = magnitude(v1);
        REQUIRE(are_equal(norm_v1, std::sqrt(50.0f)));
    }

    SECTION("normalise") {
        const Vector v1{5.0f, 0.0f, 0.0f};
        const Vector normalised_v1 = normalise(v1);
        REQUIRE(are_equal(normalised_v1, Vector{1.0f, 0.0f, 0.0f}));

        const Vector v2{2.0f, 0.0f, 2.0f};
        const Vector normalised_v2 = normalise(v2);
        REQUIRE(are_equal(normalised_v2, Vector{1.0f / std::sqrt(2.0f), 0.0f, 1.0f / std::sqrt(2.0f)}));
    }
}

TEST_CASE("Matrix", "[Matrix]") {
    SECTION("scaling_matrix") {
        const Matrix m = scaling_matrix(1.0f, 2.0f, 3.0f);
        const Matrix answer = {
            1.0f, 0.0f, 0.0f, 0.0f,
            0.0f, 2.0f, 0.0f, 0.0f,
            0.0f, 0.0f, 3.0f, 0.0f
        };
        REQUIRE(are_equal(m, answer));
    }

    SECTION("translation_matrix") {
        const Matrix m = translation_matrix(4.0f, 5.0f, 6.0f);
        const Matrix answer = {
            1.0f, 0.0f, 0.0f, 4.0f,
            0.0f, 1.0f, 0.0f, 5.0f,
            0.0f, 0.0f, 1.0f, 6.0f
        };
        REQUIRE(are_equal(m, answer));
    }

    SECTION("rotation_matrix") {
        const Matrix m1 = rotation_matrix(to_radians(90.0f), 5.0f, 0.0f, 0.0f);
        const Matrix answer1 = {
            1.0f, 0.0f,  0.0f, 0.0f,
            0.0f, 0.0f, -1.0f, 0.0f,
            0.0f, 1.0f,  0.0f, 0.0f
        };
        REQUIRE(are_equal(m1, answer1));

        const Matrix m2 = rotation_matrix(to_radians(45.0f), 1.0f, 1.0f, 0.0f);
        const float root_2_inverse = 1.0f / std::sqrt(2.0f);
        const Matrix answer2 = {
            0.5f * (1.0f + root_2_inverse), 0.5f * (1.0f - root_2_inverse),           0.5f, 0.0f,
            0.5f * (1.0f - root_2_inverse), 0.5f * (1.0f + root_2_inverse),          -0.5f, 0.0f,
                                     -0.5f,                           0.5f, root_2_inverse, 0.0f
        };
        REQUIRE(are_equal(m2, answer2));
    }

    SECTION("operator*") {
        SECTION("Vector") {
            const Vector v1{1.0f, 2.0f, 3.0f};
            const Vector v2 = identity_matrix() * v1;
            REQUIRE(are_equal(v1, v2));

            const Vector v3 = identity_matrix() * v1;
            REQUIRE(are_equal(v1, v3));

            const Vector v4 = scaling_matrix(1.0f, 1.0f / 2.0f, 1.0f / 3.0f) * v1;
            REQUIRE(are_equal(v4, Vector{1.0f, 1.0f, 1.0f}));
            
            const Matrix rotate_45_about_x = rotation_matrix(pi / 4.0f, 1.0f, 0.0f, 0.0f);
            const Vector v5 = rotate_45_about_x * i;
            REQUIRE(are_equal(v5, i));

            const float root_2_inverse = 1.0f / std::sqrt(2.0f);
            const Vector v6 = rotate_45_about_x * j;
            REQUIRE(are_equal(v6, Vector{0.0f, root_2_inverse, root_2_inverse}));

            const Vector v7 = rotate_45_about_x * k;
            REQUIRE(are_equal(v7, Vector{0.0f, -root_2_inverse, root_2_inverse}));
        }

        SECTION("Matrix") {
            const Matrix m1 = scaling_matrix(10.0f, 2.0f, 5.0f) * scaling_matrix(0.1f, 0.5f, 0.2f);
            const Matrix answer1 = identity_matrix();
            REQUIRE(are_equal(m1, answer1));

            const Matrix m2 = {
                 1.0f,  2.0f,  3.0f,  4.0f,
                 5.0f,  6.0f,  7.0f,  8.0f,
                 9.0f, 10.0f, 11.0f, 12.0f
            };
            const Matrix answer2 = {
                 38.0f,  44.0f,  50.0f,  60.0f,
                 98.0f, 116.0f, 134.0f, 160.0f,
                158.0f, 188.0f, 218.0f, 260.0f
            };
            REQUIRE(are_equal(m2 * m2, answer2));
        }
    }
}


TEST_CASE("shape_normals", "[shape_normals]") {
    SECTION("Triangle") {
        const Vector a{-0.5f, -0.5f, 0.0f};
        const Vector b{0.5f, -0.5f, 0.0f};
        const Vector c{0.5f, 0.5f, 0.0f};

        const Triangle anti_clockwise_triangle{a, b, c};
        const Vector anti_clockwise_unit_surface_normal = unit_surface_normal(anti_clockwise_triangle);
        REQUIRE(are_equal(anti_clockwise_unit_surface_normal, k));

        const Triangle clockwise_triangle{a, c, b};
        const Vector clockwise_unit_surface_normal = unit_surface_normal(clockwise_triangle);
        REQUIRE(are_equal(clockwise_unit_surface_normal, Vector{0.0f, 0.0f, -1.0f}));
    }

    SECTION("Sphere") {
        const Vector centre{1.0f, 2.0f, 3.0f};
        const float radius = 3.0f;
        const Sphere sphere{centre, radius};

        const Vector unit_normal1 = unit_surface_normal(sphere, centre + i);
        REQUIRE(are_equal(unit_normal1, i));

        const Vector unit_normal2 = unit_surface_normal(sphere, centre - i);
        REQUIRE(are_equal(unit_normal2, Vector{-1.0f, 0.0f, 0.0f}));

        const Vector unit_normal3 = unit_surface_normal(sphere, centre + j);
        REQUIRE(are_equal(unit_normal3, j));

        const Vector unit_normal4 = unit_surface_normal(sphere, centre - j);
        REQUIRE(are_equal(unit_normal4, Vector{0.0f, -1.0f, 0.0f}));

        const Vector unit_normal5 = unit_surface_normal(sphere, centre + k);
        REQUIRE(are_equal(unit_normal5, k));

        const Vector unit_normal6 = unit_surface_normal(sphere, centre - k);
        REQUIRE(are_equal(unit_normal6, Vector{0.0f, 0.0f, -1.0f}));

        const Vector unit_normal7 = unit_surface_normal(sphere, centre + i + j + k);
        REQUIRE(are_equal(unit_normal7, normalise(i + j + k)));
    }

    SECTION("Ellipsoid") {
        const float x_scale = 6.0f;
        const float y_scale = 4.0f;
        const float z_scale = 2.0f;
        const Matrix scale = scaling_matrix(x_scale, y_scale, z_scale);
        const Matrix scale_inverse = scaling_matrix(1.0f / x_scale, 1.0f / y_scale, 1.0f / z_scale);

        const float angle = to_radians(90.0f);
        const Matrix rotate = rotation_matrix(angle, 0.0f, 0.0f, 1.0f);
        const Matrix rotate_inverse = rotation_matrix(-angle, 0.0f, 0.0f, 1.0f);
        
        const float x_shift = 4.0f;
        const float y_shift = 5.0f;
        const float z_shift = 6.0f;
        const Matrix translate = translation_matrix(x_shift, y_shift, z_shift);
        const Matrix translate_inverse = translation_matrix(-x_shift, -y_shift, -z_shift);        
        
        const Matrix transform = translate * rotate * scale;
        const Matrix transform_inverse = scale_inverse * rotate_inverse * translate_inverse;

        const Ellipsoid ellipsoid{transform_inverse};

        const Vector unit_normal1 = unit_surface_normal(ellipsoid, transform * i);
        REQUIRE(are_equal(unit_normal1, j));

        const Vector unit_normal2 = unit_surface_normal(ellipsoid, transform * j);
        REQUIRE(are_equal(unit_normal2, -1.0f * i));

        const Vector unit_normal3 = unit_surface_normal(ellipsoid, transform * k);
        REQUIRE(are_equal(unit_normal3, k));
    }
}
