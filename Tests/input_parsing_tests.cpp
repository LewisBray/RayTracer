#include "test_framework.h"

#include "../Source/maths.cpp"
#include "../Source/input_parsing.cpp"
#include "test_utils.cpp"

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
    return are_equal(lhs.inverse_transform, rhs.inverse_transform);
}

TEST(parse_input_file_scene_1) {
    const std::variant<FileInfo, const char*> file_parsing_result = parse_input_file("./scene1.test");
    EXPECT_TRUE(std::holds_alternative<FileInfo>(file_parsing_result));

    const FileInfo& file_info = std::get<FileInfo>(file_parsing_result);

    const Image& image = file_info.image;
    EXPECT_EQ(image.width, 640);
    EXPECT_EQ(image.height, 480);
    EXPECT_TRUE(image.pixels != nullptr);
    EXPECT_EQ(image.filename, "raytrace.png");

    const Camera& camera = file_info.camera;
    EXPECT_TRUE(are_equal(camera.eye, Vector{-4.0f, -4.0f, 4.0f}));
    EXPECT_TRUE(are_equal(camera.look_at, Vector{1.0f, 0.0f, 0.0f}));
    EXPECT_TRUE(are_equal(camera.up, Vector{0.0f, 1.0f, 0.0f}));
    EXPECT_TRUE(are_equal(camera.field_of_view.x, 40.0f));
    EXPECT_TRUE(are_equal(camera.field_of_view.y, 30.0f));

    const Scene& scene = file_info.scene;

    const std::array<Vector, 4> vertices {
        Vector{-1.0f, -1.0f, 0.0f},
        Vector{1.0f, -1.0f, 0.0f},
        Vector{1.0f, 1.0f, 0.0f},
        Vector{-1.0f, 1.0f, 0.0f}
    };

    const std::array<Triangle, 2> expected_triangles {
        Triangle{vertices[0], vertices[1], vertices[2]},
        Triangle{vertices[0], vertices[2], vertices[3]}
    };

    const std::vector<Triangle>& triangles = scene.triangles;
    EXPECT_EQ(triangles.size(), expected_triangles.size());
    for (std::size_t i = 0; i < triangles.size(); ++i) {
        EXPECT_TRUE(are_equal(triangles[i], expected_triangles[i]));
    }

    const Material expected_material {
        Colour{1.0f, 0.0f, 0.0f},
        Colour{0.0f, 0.0f, 0.0f},
        Colour{0.0f, 0.0f, 0.0f},
        0.0f
    };

    const std::vector<Material>& triangle_materials = scene.triangle_materials;
    EXPECT_EQ(triangle_materials.size(), expected_triangles.size());
    for (const Material& triangle_material : triangle_materials) {
        EXPECT_TRUE(are_equal(triangle_material, expected_material));
    }

    EXPECT_TRUE(scene.ellipsoids.empty());
    EXPECT_TRUE(scene.ellipsoid_transforms.empty());
    EXPECT_TRUE(scene.ellipsoid_materials.empty());

    EXPECT_TRUE(scene.directional_light_source.has_value());
    const DirectionalLightSource& directional_light_source = scene.directional_light_source.value();
    EXPECT_TRUE(are_equal(directional_light_source.direction, Vector{0.0f, 0.0f, 1.0f}));
    EXPECT_TRUE(are_equal(directional_light_source.colour, Colour{0.5f, 0.5f, 0.5f}));

    EXPECT_EQ(scene.point_light_sources.size(), 1);
    const PointLightSource& point_light_source = scene.point_light_sources.front();
    EXPECT_TRUE(are_equal(point_light_source.position, Vector{4.0f, 0.0f, 4.0f}));
    EXPECT_TRUE(are_equal(point_light_source.colour, Colour{0.5f, 0.5f, 0.5f}));

    EXPECT_TRUE(are_equal(scene.ambient, Colour{0.1f, 0.1f, 0.1f}));
    EXPECT_TRUE(are_equal(scene.attenuation_parameters, AttenuationParameters{1.0f, 0.0f, 0.0f}));
}

TEST(parse_input_file_scene_2) {
    const std::variant<FileInfo, const char*> file_parsing_result = parse_input_file("./scene2.test");
    EXPECT_TRUE(std::holds_alternative<FileInfo>(file_parsing_result));

    const FileInfo& file_info = std::get<FileInfo>(file_parsing_result);

    const Image& image = file_info.image;
    EXPECT_EQ(image.width, 640);
    EXPECT_EQ(image.height, 480);
    EXPECT_TRUE(image.pixels != nullptr);
    EXPECT_EQ(image.filename, "raytrace.png");

    const Camera& camera = file_info.camera;
    EXPECT_TRUE(are_equal(camera.eye, Vector{-2.0f, -2.0f, -2.0f}));
    EXPECT_TRUE(are_equal(camera.look_at, Vector{0.0f, 0.0f, 0.0f}));
    EXPECT_TRUE(are_equal(camera.up, Vector{-1.0f, -1.0f, 2.0f}));
    EXPECT_TRUE(are_equal(camera.field_of_view.x, 80.0f));
    EXPECT_TRUE(are_equal(camera.field_of_view.y, 60.0f));

    const Scene& scene = file_info.scene;

    const std::array<Vector, 8> vertices {
        Vector{-1.0f, -1.0f, -1.0f},
        Vector{1.0f, -1.0f, -1.0f},
        Vector{1.0f, 1.0f, -1.0f},
        Vector{-1.0f, 1.0f, -1.0f},
        Vector{-1.0f, -1.0f, 1.0f},
        Vector{1.0f, -1.0f, 1.0f},
        Vector{1.0f, 1.0f, 1.0f},
        Vector{-1.0f, 1.0f, 1.0f}
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
    EXPECT_EQ(triangles.size(), expected_triangles.size());
    for (std::size_t i = 0; i < triangles.size(); ++i) {
        EXPECT_TRUE(are_equal(triangles[i], expected_triangles[i]));
    }

    const Colour expected_diffuse{0.0f, 0.0f, 0.0f};
    const Colour expected_specular{0.0f, 0.0f, 0.0f};
    const float expected_shininess = 0.0f;

    const std::array<Colour, 12> expected_triangle_emissions {
        Colour{0.5f, 0.0f, 0.5f},
        Colour{0.5f, 0.0f, 0.5f},

        Colour{0.5f, 1.0f, 0.5f},
        Colour{0.5f, 1.0f, 0.5f},

        Colour{1.0f, 0.5f, 0.5f},
        Colour{1.0f, 0.5f, 0.5f},

        Colour{0.0f, 0.5f, 0.5f},
        Colour{0.0f, 0.5f, 0.5f},

        Colour{0.5f, 0.5f, 0.0f},
        Colour{0.5f, 0.5f, 0.0f},

        Colour{0.5f, 0.5f, 1.0f},
        Colour{0.5f, 0.5f, 1.0f}
    };

    const std::vector<Material>& triangle_materials = scene.triangle_materials;
    EXPECT_EQ(triangle_materials.size(), expected_triangle_emissions.size());
    for (std::size_t i = 0; i < triangle_materials.size(); ++i) {
        const Material& material = triangle_materials[i];
        EXPECT_TRUE(are_equal(material.diffuse, expected_diffuse));
        EXPECT_TRUE(are_equal(material.specular, expected_specular));
        EXPECT_TRUE(are_equal(material.emission, expected_triangle_emissions[i]));
        EXPECT_TRUE(are_equal(material.shininess, expected_shininess));
    }

    const std::array<Sphere, 21> expected_spheres {
        Sphere{Vector{1.0f, 0.0f, 0.0f}, 0.15f},

        Sphere{Vector{-0.5f, 1.0f, -0.5f}, 0.15f},
        Sphere{Vector{0.5f, 1.0f, 0.5f}, 0.15f},

        Sphere{Vector{0.0f, 0.0f, 1.0f}, 0.15f},
        Sphere{Vector{-0.5f, -0.5f, 1.0f}, 0.15f},
        Sphere{Vector{0.5f, 0.5f, 1.0f}, 0.15f},

        Sphere{Vector{-1.0f, -0.5f, -0.5f}, 0.15f},
        Sphere{Vector{-1.0f, -0.5f, 0.5f}, 0.15f},
        Sphere{Vector{-1.0f, 0.5f, 0.5f}, 0.15f},
        Sphere{Vector{-1.0f, 0.5f, -0.5f}, 0.15f},

        Sphere{Vector{-0.5f, -1.0f, -0.5f}, 0.15f},
        Sphere{Vector{-0.5f, -1.0f, 0.5f}, 0.15f},
        Sphere{Vector{0.5f, -1.0f, 0.5f}, 0.15f},
        Sphere{Vector{0.5f, -1.0f, -0.5f}, 0.15f},
        Sphere{Vector{0.0f, -1.0f, 0.0f}, 0.15f},

        Sphere{Vector{-0.5f, -0.5f, -1.0f}, 0.15f},
        Sphere{Vector{-0.5f, 0.0f, -1.0f}, 0.15f},
        Sphere{Vector{-0.5f, 0.5f, -1.0f}, 0.15f},
        Sphere{Vector{0.5f, -0.5f, -1.0f}, 0.15f},
        Sphere{Vector{0.5f, 0.0f, -1.0f}, 0.15f},
        Sphere{Vector{0.5f, 0.5f, -1.0f}, 0.15f}
    };

    const std::vector<Sphere>& spheres = scene.spheres;
    EXPECT_EQ(spheres.size(), expected_spheres.size());
    for (std::size_t i = 0; i < spheres.size(); ++i) {
        EXPECT_TRUE(are_equal(spheres[i], expected_spheres[i]));
    }

    const Material expected_material {
        Colour{0.0f, 0.0f, 0.0f},
        Colour{0.0f, 0.0f, 0.0f},
        Colour{1.0f, 1.0f, 1.0f},
        0.0f
    };

    const std::vector<Material>& sphere_materials = scene.sphere_materials;
    for (const Material& sphere_material : sphere_materials) {
        EXPECT_TRUE(are_equal(sphere_material, expected_material));
    }

    EXPECT_TRUE(!scene.directional_light_source.has_value());
    EXPECT_TRUE(scene.point_light_sources.empty());

    EXPECT_TRUE(are_equal(scene.ambient, Colour{0.0f, 0.0f, 0.0f}));
}

TEST(parse_input_file_scene_3) {
    const std::variant<FileInfo, const char*> file_parsing_result = parse_input_file("./scene3.test");
    EXPECT_TRUE(std::holds_alternative<FileInfo>(file_parsing_result));

    const FileInfo& file_info = std::get<FileInfo>(file_parsing_result);

    const Image& image = file_info.image;
    EXPECT_EQ(image.width, 640);
    EXPECT_EQ(image.height, 480);
    EXPECT_TRUE(image.pixels != nullptr);
    EXPECT_EQ(image.filename, "scene.png");

    const Camera& camera = file_info.camera;
    EXPECT_TRUE(are_equal(camera.eye, Vector{0.0f, -4.0f, 4.0f}));
    EXPECT_TRUE(are_equal(camera.look_at, Vector{0.0f, -1.0f, 0.0f}));
    EXPECT_TRUE(are_equal(camera.up, Vector{0.0f, 1.0f, 1.0f}));
    EXPECT_TRUE(are_equal(camera.field_of_view.x, 60.0f));
    EXPECT_TRUE(are_equal(camera.field_of_view.y, 45.0f));

    const Scene& scene = file_info.scene;

    EXPECT_TRUE(scene.triangles.empty());
    EXPECT_TRUE(scene.triangle_materials.empty());

    const std::array<Sphere, 4> expected_spheres = {
        Sphere{Vector{-1.5f, -0.8f, 0.65f}, 0.4f},
        Sphere{Vector{1.5f, -0.8f, 0.65f}, 0.4f},
        Sphere{Vector{1.5f, 0.8f, 0.65f}, 0.4f},
        Sphere{Vector{-1.5f, 0.8f, 0.65f}, 0.4f}
    };

    const std::vector<Sphere>& spheres = scene.spheres;
    EXPECT_EQ(spheres.size(), expected_spheres.size());
    for (std::size_t i = 0; i < spheres.size(); ++i) {
        EXPECT_TRUE(are_equal(spheres[i], expected_spheres[i]));
    }

    const Colour expected_diffuse{0.0f, 0.0f, 0.0f};
    const Colour expected_specular{0.0f, 0.0f, 0.0f};
    const float expected_shininess = 0.0f;

    const Colour expected_sphere_emission{0.0f, 1.0f, 1.0f};
    const std::vector<Material>& sphere_materials = scene.sphere_materials;
    EXPECT_EQ(sphere_materials.size(), spheres.size());
    for (const Material& material : sphere_materials) {
        EXPECT_TRUE(are_equal(material.diffuse, expected_diffuse));
        EXPECT_TRUE(are_equal(material.specular, expected_specular));
        EXPECT_TRUE(are_equal(material.emission, expected_sphere_emission));
        EXPECT_TRUE(are_equal(material.shininess, expected_shininess));
    }

    const float angle = to_radians(45.0f);
    const std::array<Ellipsoid, 2> expected_ellipsoids {
        Ellipsoid{scaling_matrix(1.0f, 4.0f, 4.0f) * rotation_matrix(-angle, 0.0f, 0.0f, 1.0f) * translation_matrix(0.0f, 0.0f, -0.5f)},
        Ellipsoid{scaling_matrix(1.0f, 4.0f, 4.0f) * rotation_matrix(angle, 0.0f, 0.0f, 1.0f) * translation_matrix(0.0f, 0.0f, -0.5f)}
    };

    const std::array<Matrix, 2> expected_ellipsoid_transforms {
        translation_matrix(0.0f, 0.0f, 0.5f) * rotation_matrix(angle, 0.0f, 0.0f, 1.0f) * scaling_matrix(1.0f, 0.25f, 0.25f),
        translation_matrix(0.0f, 0.0f, 0.5f) * rotation_matrix(-angle, 0.0f, 0.0f, 1.0f) * scaling_matrix(1.0f, 0.25f, 0.25f)
    };

    const std::vector<Ellipsoid>& ellipsoids = scene.ellipsoids;
    EXPECT_EQ(ellipsoids.size(), expected_ellipsoids.size());
    for (std::size_t i = 0; i < ellipsoids.size(); ++i) {
        EXPECT_TRUE(are_equal(ellipsoids[i], expected_ellipsoids[i]));
    }

    const std::vector<Matrix>& ellipsoid_transforms = scene.ellipsoid_transforms;
    EXPECT_EQ(ellipsoid_transforms.size(), expected_ellipsoid_transforms.size());
    for (std::size_t i = 0; i < ellipsoid_transforms.size(); ++i) {
        EXPECT_TRUE(are_equal(ellipsoid_transforms[i], expected_ellipsoid_transforms[i]));
    }

    const std::array<Colour, 2> expected_ellipsoid_emissions{ 
        Colour{0.0f, 1.0f, 0.0f},
        Colour{1.0f, 0.0f, 0.0f}
    };

    const std::vector<Material>& ellipsoid_materials = scene.ellipsoid_materials;
    EXPECT_EQ(ellipsoid_materials.size(), expected_ellipsoid_emissions.size());
    for (std::size_t i = 0; i < ellipsoid_materials.size(); ++i) {
        const Material& material = ellipsoid_materials[i];
        EXPECT_TRUE(are_equal(material.diffuse, expected_diffuse));
        EXPECT_TRUE(are_equal(material.specular, expected_specular));
        EXPECT_TRUE(are_equal(material.emission, expected_ellipsoid_emissions[i]));
        EXPECT_TRUE(are_equal(material.shininess, expected_shininess));
    }

    EXPECT_TRUE(!scene.directional_light_source.has_value());
    EXPECT_TRUE(scene.point_light_sources.empty());

    EXPECT_TRUE(are_equal(scene.ambient, Colour{0.0f, 0.0f, 0.0f}));
}
