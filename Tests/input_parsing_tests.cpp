#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../Source/input_parsing.h"
#include "test_utils.h"

static bool are_equal(const Triangle& lhs, const Triangle& rhs) noexcept {
    return are_equal(lhs.a, rhs.a) && are_equal(lhs.b, rhs.b) && are_equal(lhs.c, rhs.c);
}

static bool are_equal(const Colour& lhs, const Colour& rhs) noexcept {
    return are_equal(lhs.red, rhs.red) && are_equal(lhs.green, rhs.green) && are_equal(lhs.blue, rhs.blue);
}

static bool are_equal(const Material& lhs, const Material& rhs) noexcept {
    return are_equal(lhs.diffuse, rhs.diffuse) && are_equal(lhs.specular, rhs.specular) &&
        are_equal(lhs.emission, rhs.emission) && are_equal(lhs.shininess, rhs.shininess);
}

static bool are_equal(const AttenuationParameters& lhs, const AttenuationParameters& rhs) noexcept {
    return are_equal(lhs.constant, rhs.constant) && are_equal(lhs.linear, rhs.linear) && are_equal(lhs.quadratic, rhs.quadratic);
}

static bool are_equal(const Sphere& lhs, const Sphere& rhs) noexcept {
    return are_equal(lhs.radius, rhs.radius) && are_equal(lhs.centre, rhs.centre);
}

static bool are_equal(const Ellipsoid& lhs, const Ellipsoid& rhs) noexcept {
    return are_equal(lhs.sphere, rhs.sphere) && are_equal(lhs.inverse_transform, rhs.inverse_transform);
}

TEST_CASE("parse_input_file", "[parse_input_file]") {
    SECTION("scene_1") {
        const std::variant<FileInfo, const char*> file_parsing_result = parse_input_file("./scene1.test");
        REQUIRE(std::holds_alternative<FileInfo>(file_parsing_result));

        const FileInfo& file_info = std::get<FileInfo>(file_parsing_result);

        const Image& image = file_info.image;
        REQUIRE(image.width == 640);
        REQUIRE(image.height == 480);
        REQUIRE(image.pixels != nullptr);
        REQUIRE(image.filename == "raytrace.png");

        const Camera& camera = file_info.camera;
        REQUIRE(are_equal(camera.eye, Vector{-4.0, -4.0, 4.0, 1.0}));
        REQUIRE(are_equal(camera.look_at, Vector{1.0, 0.0, 0.0, 1.0}));
        REQUIRE(are_equal(camera.up, Vector{0.0, 1.0, 0.0, 1.0}));
        REQUIRE(camera.field_of_view.x == 40);
        REQUIRE(camera.field_of_view.y == 30);

        const Scene& scene = file_info.scene;

        const std::array<Vector, 4> vertices {
            Vector{-1.0, -1.0, 0.0, 1.0},
            Vector{1.0, -1.0, 0.0, 1.0},
            Vector{1.0, 1.0, 0.0, 1.0},
            Vector{-1.0, 1.0, 0.0, 1.0}
        };

        const std::array<Triangle, 2> expected_triangles {
            Triangle{vertices[0], vertices[1], vertices[2]},
            Triangle{vertices[0], vertices[2], vertices[3]}
        };

        const std::vector<Triangle>& triangles = scene.triangles;
        REQUIRE(triangles.size() == expected_triangles.size());
        for (std::size_t i = 0; i < triangles.size(); ++i)
            REQUIRE(are_equal(triangles[i], expected_triangles[i]));

        const Material expected_material {
            Colour{1.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            0.0
        };

        const std::vector<Material>& triangle_materials = scene.triangle_materials;
        REQUIRE(triangle_materials.size() == expected_triangles.size());
        for (const Material& triangle_material : triangle_materials) {
            REQUIRE(are_equal(triangle_material, expected_material));
        }

        const Colour expected_ambient{0.1, 0.1, 0.1};

        const std::vector<Colour>& triangle_ambients = scene.triangle_ambients;
        REQUIRE(triangle_ambients.size() == expected_triangles.size());
        for (const Colour& triangle_ambient : triangle_ambients) {
            REQUIRE(are_equal(triangle_ambient, expected_ambient));
        }

        REQUIRE(scene.ellipsoids.empty());
        REQUIRE(scene.ellipsoid_transforms.empty());
        REQUIRE(scene.ellipsoid_materials.empty());
        REQUIRE(scene.ellipsoid_ambients.empty());

        REQUIRE(scene.directional_light_source.has_value());
        const DirectionalLightSource& directional_light_source = scene.directional_light_source.value();
        REQUIRE(are_equal(directional_light_source.direction, Vector{0.0, 0.0, 1.0, 1.0}));
        REQUIRE(are_equal(directional_light_source.colour, Colour{0.5, 0.5, 0.5}));

        REQUIRE(scene.point_light_sources.size() == 1);
        const PointLightSource& point_light_source = scene.point_light_sources.front();
        REQUIRE(are_equal(point_light_source.position, Vector{4.0, 0.0, 4.0, 1.0}));
        REQUIRE(are_equal(point_light_source.colour, Colour{0.5, 0.5, 0.5}));
        
        REQUIRE(are_equal(scene.attenuation_parameters, AttenuationParameters{1.0, 0.0, 0.0}));
    }

    SECTION("scene_2") {
        const std::variant<FileInfo, const char*> file_parsing_result = parse_input_file("./scene2.test");
        REQUIRE(std::holds_alternative<FileInfo>(file_parsing_result));

        const FileInfo& file_info = std::get<FileInfo>(file_parsing_result);

        const Image& image = file_info.image;
        REQUIRE(image.width == 640);
        REQUIRE(image.height == 480);
        REQUIRE(image.pixels != nullptr);
        REQUIRE(image.filename == "raytrace.png");

        const Camera& camera = file_info.camera;
        REQUIRE(are_equal(camera.eye, Vector{-2.0, -2.0, -2.0, 1.0}));
        REQUIRE(are_equal(camera.look_at, Vector{0.0, 0.0, 0.0, 1.0}));
        REQUIRE(are_equal(camera.up, Vector{-1.0, -1.0, 2.0, 1.0}));
        REQUIRE(camera.field_of_view.x == 80);
        REQUIRE(camera.field_of_view.y == 60);

        const Scene& scene = file_info.scene;

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

        const std::array<Triangle, 12> expected_triangles {
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
        REQUIRE(triangles.size() == expected_triangles.size());
        for (std::size_t i = 0; i < triangles.size(); ++i) {
            REQUIRE(are_equal(triangles[i], expected_triangles[i]));
        }

        const Material expected_material {
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            0.0
        };

        const std::vector<Material>& triangle_materials = scene.triangle_materials;
        REQUIRE(triangle_materials.size() == expected_triangles.size());
        for (const Material& triangle_material : triangle_materials) {
            REQUIRE(are_equal(triangle_material, expected_material));
        }

        const std::array<Colour, 12> expected_triangle_ambients {
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

        const std::vector<Colour>& triangle_ambients = scene.triangle_ambients;
        REQUIRE(triangle_ambients.size() == expected_triangles.size());
        for (std::size_t i = 0; i < triangle_ambients.size(); ++i) {
            REQUIRE(are_equal(triangle_ambients[i], expected_triangle_ambients[i]));
        }

        const std::array<Ellipsoid, 21> expected_ellipsoids {
            Ellipsoid{Sphere{Vector{1.0, 0.0, 0.0, 1.0}, 0.15}, identity_matrix()},

            Ellipsoid{Sphere{Vector{-0.5, 1.0, -0.5, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{0.5, 1.0, 0.5, 1.0}, 0.15}, identity_matrix()},

            Ellipsoid{Sphere{Vector{0.0, 0.0, 1.0, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{-0.5, -0.5, 1.0, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{0.5, 0.5, 1.0, 1.0}, 0.15}, identity_matrix()},

            Ellipsoid{Sphere{Vector{-1.0, -0.5, -0.5, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{-1.0, -0.5, 0.5, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{-1.0, 0.5, 0.5, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{-1.0, 0.5, -0.5, 1.0}, 0.15}, identity_matrix()},

            Ellipsoid{Sphere{Vector{-0.5, -1.0, -0.5, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{-0.5, -1.0, 0.5, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{0.5, -1.0, 0.5, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{0.5, -1.0, -0.5, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{0.0, -1.0, 0.0, 1.0}, 0.15}, identity_matrix()},

            Ellipsoid{Sphere{Vector{-0.5, -0.5, -1.0, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{-0.5, 0.0, -1.0, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{-0.5, 0.5, -1.0, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{0.5, -0.5, -1.0, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{0.5, 0.0, -1.0, 1.0}, 0.15}, identity_matrix()},
            Ellipsoid{Sphere{Vector{0.5, 0.5, -1.0, 1.0}, 0.15}, identity_matrix()}
        };

        const std::vector<Ellipsoid>& ellipsoids = scene.ellipsoids;
        REQUIRE(ellipsoids.size() == expected_ellipsoids.size());
        for (std::size_t i = 0; i < ellipsoids.size(); ++i) {
            REQUIRE(are_equal(ellipsoids[i], expected_ellipsoids[i]));
        }

        const std::vector<Matrix>& ellipsoid_transforms = scene.ellipsoid_transforms;
        REQUIRE(ellipsoid_transforms.size() == expected_ellipsoids.size());
        for (const Matrix& ellipsoid_transform : ellipsoid_transforms) {
            REQUIRE(are_equal(ellipsoid_transform, identity_matrix()));
        }

        const std::vector<Material>& ellipsoid_materials = scene.ellipsoid_materials;
        for (const Material& ellipsoid_material : ellipsoid_materials) {
            REQUIRE(are_equal(ellipsoid_material, expected_material));
        }

        const Colour expected_ellipsoid_ambient{1.0, 1.0, 1.0};

        const std::vector<Colour>& ellipsoid_ambients = scene.ellipsoid_ambients;
        REQUIRE(ellipsoid_ambients.size() == expected_ellipsoids.size());
        for (const Colour& ellipsoid_ambient : ellipsoid_ambients) {
            REQUIRE(are_equal(ellipsoid_ambient, expected_ellipsoid_ambient));
        }

        REQUIRE(!scene.directional_light_source.has_value());
        REQUIRE(scene.point_light_sources.empty());
    }

    SECTION("scene_3") {
        const std::variant<FileInfo, const char*> file_parsing_result = parse_input_file("./scene3.test");
        REQUIRE(std::holds_alternative<FileInfo>(file_parsing_result));

        const FileInfo& file_info = std::get<FileInfo>(file_parsing_result);

        const Image& image = file_info.image;
        REQUIRE(image.width == 640);
        REQUIRE(image.height == 480);
        REQUIRE(image.pixels != nullptr);
        REQUIRE(image.filename == "scene.png");

        const Camera& camera = file_info.camera;
        REQUIRE(are_equal(camera.eye, Vector{0.0, -4.0, 4.0, 1.0}));
        REQUIRE(are_equal(camera.look_at, Vector{0.0, -1.0, 0.0, 1.0}));
        REQUIRE(are_equal(camera.up, Vector{0.0, 1.0, 1.0, 1.0}));
        REQUIRE(camera.field_of_view.x == 60);
        REQUIRE(camera.field_of_view.y == 45);

        const Scene& scene = file_info.scene;

        REQUIRE(scene.triangles.empty());
        REQUIRE(scene.triangle_materials.empty());
        REQUIRE(scene.triangle_ambients.empty());

        const std::array<Matrix, 6> expected_ellipsoid_transforms {
            translation_matrix(0.0, 0.0, 0.5) * rotation_matrix(45.0, 0.0, 0.0, 1.0) * scaling_matrix(1.0, 0.25, 0.25),
            translation_matrix(0.0, 0.0, 0.5) * rotation_matrix(-45.0, 0.0, 0.0, 1.0) * scaling_matrix(1.0, 0.25, 0.25),
            translation_matrix(-1.5, -0.8, 0.65) * scaling_matrix(0.4, 0.4, 0.4),
            translation_matrix(1.5, -0.8, 0.65) * scaling_matrix(0.4, 0.4, 0.4),
            translation_matrix(1.5, 0.8, 0.65) * scaling_matrix(0.4, 0.4, 0.4),
            translation_matrix(-1.5, 0.8, 0.65) * scaling_matrix(0.4, 0.4, 0.4)
        };

        const std::array<Matrix, 6> expected_ellipsoid_inverse_transforms {
            scaling_matrix(1.0, 4.0, 4.0) * rotation_matrix(-45.0, 0.0, 0.0, 1.0) * translation_matrix(0.0, 0.0, -0.5),
            scaling_matrix(1.0, 4.0, 4.0) * rotation_matrix(45.0, 0.0, 0.0, 1.0) * translation_matrix(0.0, 0.0, -0.5),
            scaling_matrix(2.5, 2.5, 2.5) * translation_matrix(1.5, 0.8, -0.65),
            scaling_matrix(2.5, 2.5, 2.5) * translation_matrix(-1.5, 0.8, -0.65),
            scaling_matrix(2.5, 2.5, 2.5) * translation_matrix(-1.5, -0.8, -0.65),
            scaling_matrix(2.5, 2.5, 2.5) * translation_matrix(1.5, -0.8, -0.65)
        };

        const std::array<Ellipsoid, 6> expected_ellipsoids {
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expected_ellipsoid_inverse_transforms[0]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expected_ellipsoid_inverse_transforms[1]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expected_ellipsoid_inverse_transforms[2]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expected_ellipsoid_inverse_transforms[3]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expected_ellipsoid_inverse_transforms[4]},
            Ellipsoid{Sphere{Vector{0.0, 0.0, 0.0, 1.0}, 1.0}, expected_ellipsoid_inverse_transforms[5]}
        };

        const std::vector<Ellipsoid>& ellipsoids = scene.ellipsoids;
        REQUIRE(ellipsoids.size() == expected_ellipsoids.size());
        for (std::size_t i = 0; i < ellipsoids.size(); ++i) {
            REQUIRE(are_equal(ellipsoids[i], expected_ellipsoids[i]));
        }

        const std::vector<Matrix>& ellipsoid_transforms = scene.ellipsoid_transforms;
        REQUIRE(ellipsoid_transforms.size() == expected_ellipsoid_transforms.size());
        for (std::size_t i = 0; i < ellipsoid_transforms.size(); ++i) {
            REQUIRE(are_equal(ellipsoid_transforms[i], expected_ellipsoid_transforms[i]));
        }

        const Material expected_material {
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            Colour{0.0, 0.0, 0.0},
            0.0
        };

        const std::vector<Material>& ellipsoid_materials = scene.ellipsoid_materials;
        for (const Material& ellipsoid_material : ellipsoid_materials)
            REQUIRE(are_equal(ellipsoid_material, expected_material));

        const std::array<Colour, 6> expected_ellipsoid_ambients{ 
            Colour{0.0, 1.0, 0.0},
            Colour{1.0, 0.0, 0.0},
            Colour{0.0, 1.0, 1.0},
            Colour{0.0, 1.0, 1.0},
            Colour{0.0, 1.0, 1.0},
            Colour{0.0, 1.0, 1.0}
        };

        const std::vector<Colour>& ellipsoid_ambients = scene.ellipsoid_ambients;
        REQUIRE(ellipsoid_ambients.size() == expected_ellipsoid_ambients.size());
        for (std::size_t i = 0; i < ellipsoid_ambients.size(); ++i) {
            REQUIRE(are_equal(ellipsoid_ambients[i], expected_ellipsoid_ambients[i]));
        }

        REQUIRE(!scene.directional_light_source.has_value());
        REQUIRE(scene.point_light_sources.empty());
    }
}
