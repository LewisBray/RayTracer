#define CATCH_CONFIG_MAIN
#include "catch.hpp"

#include "../Source/maths.cpp"
#include "../Source/ray_tracing.cpp"
#include "test_utils.cpp"

TEST_CASE("intersect", "[intersect]") {
    SECTION("triangle_intersection") {
        const Vector start{0.0f, 0.0f, 5.0f};
        const Vector direction{0.0f, 0.0f, -1.0f};
        const Ray ray{start, direction};

        const Vector xy_1{0.0f, 0.5f, 0.0f};
        const Vector xy_2{-0.5f, -0.5f, 0.0f};
        const Vector xy_3{0.5f, -0.5f, 0.0f};
        const Triangle xy_triangle{xy_1, xy_2, xy_3};

        // Ray starts 5 units in the z-axis above the triangle in the xy-plane
        const std::optional<float> xy_intersection_distance = intersect(ray, xy_triangle);
        REQUIRE(xy_intersection_distance.has_value());
        REQUIRE(are_equal(xy_intersection_distance.value(), 5.0f));

        // Ray shouldn't intersect with triangle that's parallel to it
        const Vector yz_1{1.0f, 0.5f, 0.0f};
        const Vector yz_2{1.0f, -0.5f, -0.5f};
        const Vector yz_3{1.0f, -0.5f, 0.5f};
        const Triangle yz_Triangle{yz_1, yz_2, yz_3};
        const std::optional<float> yz_intersection_distance = intersect(ray, yz_Triangle);
        REQUIRE(!yz_intersection_distance.has_value());

        // Ray shouldn't intersect with parallel triangle even if in same plane
        const Vector zx_1{0.0f, 0.0f, 0.5f};
        const Vector zx_2{-0.5f, 0.0f, -0.5f};
        const Vector zx_3{-0.5f, 0.0f, 0.5f};
        const Triangle zx_triangle{zx_1, zx_2, zx_3};
        const std::optional<float> zx_intersection_distance = intersect(ray, zx_triangle);
        REQUIRE(!zx_intersection_distance.has_value());

        // Ray should intersect with vertex at origin
        const Vector origin{0.0f, 0.0f, 0.0f};
        const Vector v2{0.4f, 2.3f, 0.1f};
        const Vector v3{3.1f, 1.1f, -0.7f};
        const Triangle vertex_at_origin_triangle{origin, v2, v3};
        const std::optional<float> origin_intersection_distance = intersect(ray, vertex_at_origin_triangle);
        REQUIRE(origin_intersection_distance.has_value());
        REQUIRE(are_equal(origin_intersection_distance.value(), 5.0f));

        // Ray shouldn't intersect with triangle behind it
        const Vector behind_camera_shift{0.0f, 0.0f, 10.0f};
        const Triangle triangle_behind_camera{xy_1 + behind_camera_shift, xy_2 + behind_camera_shift, xy_3 + behind_camera_shift};
        const std::optional<float> behind_camera_intersection_distance = intersect(ray, triangle_behind_camera);
        REQUIRE(!behind_camera_intersection_distance.has_value());

        // Try an intersection with a weird triangle and ray
        const Vector a{4.0f, 10.0f, 2.0f};
        const Vector b{8.0f, -4.0f, -12.0f};
        const Vector c{-3.0f, 7.0f, 5.0f};
        const Triangle weird_triangle{a, b, c};
        const Vector weird_start{2.0f, 6.0f, 5.0f};
        const Vector weird_direction{-1.0f / std::sqrt(6.0f), 1.0f / std::sqrt(6.0f), -2.0f / std::sqrt(6.0f)};
        const Ray weird_ray{weird_start, weird_direction};
        const std::optional<float> weird_intersection_distance = intersect(weird_ray, weird_triangle);
        REQUIRE(weird_intersection_distance.has_value());
        REQUIRE(are_equal(weird_intersection_distance.value(), 3.178055923f));
    }

    SECTION("sphere_intersection") {
        const Vector centre{4.0f, 1.0f, -8.0f};
        const float radius = 3.0f;
        const Sphere sphere{centre, radius};

        // Intersecting twice with sphere should yield closest intersection distance
        const Vector twice_intersecting_start{4.0f, 1.0f, -10.0f};
        const Vector twice_intersecting_direction{0.0f, 0.0f, 1.0f};
        const Ray twice_intersecting_ray{twice_intersecting_start, twice_intersecting_direction};
        const std::optional<std::pair<float, float>> twice_intersecting_distances = intersect(twice_intersecting_ray, sphere);
        REQUIRE(twice_intersecting_distances.has_value());
        REQUIRE(are_equal(twice_intersecting_distances.value().first, -1.0f));
        REQUIRE(are_equal(twice_intersecting_distances.value().second, 5.0f));

        // Check single intersections
        const Vector once_intersecting_start{7.0f, 1.0f, -20.0f};
        const Vector once_intersecting_direction{0.0f, 0.0f, 1.0f};
        const Ray once_intersecting_ray{once_intersecting_start, once_intersecting_direction};
        const std::optional<std::pair<float, float>> once_intersecting_distances = intersect(once_intersecting_ray, sphere);
        REQUIRE(once_intersecting_distances.has_value());
        REQUIRE(are_equal(once_intersecting_distances.value().first, once_intersecting_distances.value().second));
        REQUIRE(are_equal(once_intersecting_distances.value().first, 12.0f));

        // Check intersection we expect to miss
        const Vector never_intersecting_start{4.0f, 10.0f, 0.0f};
        const Vector never_intersecting_direction{1.0f / std::sqrt(3.0f), 1.0f / std::sqrt(3.0f), 1.0f / std::sqrt(3.0f)};
        const Ray never_intersecting_ray{never_intersecting_start, never_intersecting_direction};
        const std::optional<std::pair<float, float>> never_intersecting_distances = intersect(never_intersecting_ray, sphere);
        REQUIRE(!never_intersecting_distances.has_value());

        // Check intersection with ray origin inside the sphere
        const Vector internal_start{5.0f, 2.0f, -7.0f};
        const Vector internal_direction{0.0f, 1.0f, 0.0f};
        const Ray internal_ray{internal_start, internal_direction};
        const std::optional<std::pair<float, float>> internal_intersection_distances = intersect(internal_ray, sphere);
        REQUIRE(internal_intersection_distances.has_value());
        REQUIRE(are_equal(internal_intersection_distances.value().first, -3.645751311f));
        REQUIRE(are_equal(internal_intersection_distances.value().second, 1.645751311f));
    }

    // TODO: Triangle and sphere intersection methods should be tested indirectly through the
    // scene intersection method so the above tests need to be moved into the sub-sections
    // of this section and have a simple scene set up to mimic what's above
    SECTION("scene_intersection") {

    }
}

TEST_CASE("ray_through_pixel", "[ray_through_pixel]") {
    const Vector camera_eye{0.0f, 0.0f, 4.0f};
    const Vector camera_look_at{0.0f, 0.0f, 0.0f};
    const Vector camera_up{0.0f, 1.0f, 0.0f};
    const FieldOfView camera_field_of_view{45.0f, 45.0f};
    const Camera camera{camera_eye, camera_look_at, camera_up, camera_field_of_view};

    const Image image{3, 3, nullptr, std::string{}};

    const float image_dimension = 2.0f * (std::sqrt(2.0f) - 1.0f);   // 2 * tan(fov / 2)
    const Vector image_centre = camera.eye + normalise(camera.look_at - camera.eye);

    const Vector camera_basis_k = normalise(camera.look_at - camera.eye);
    const Vector camera_basis_i = normalise(camera.up ^ camera_basis_k);
    const Vector camera_basis_j = camera_basis_k ^ camera_basis_i;

    const BasisVectors camera_basis_vectors {
        camera_basis_i,
        camera_basis_j,
        camera_basis_k
    };

    const Dimensions half_image_dimensions_world {
        std::tan(0.5f * to_radians(camera.field_of_view.x)),
        std::tan(0.5f * to_radians(camera.field_of_view.y))
    };

    const Dimensions half_image_dimensions_pixels {
        0.5f * static_cast<float>(image.width),
        0.5f * static_cast<float>(image.height)
    };

    for (int x = 0; x < image.width; ++x) {
        for (int y = 0; y < image.height; ++y) {
            const Vector direction = ray_direction_through_pixel(x, y, 0.5f, 0.5f, camera_basis_vectors, half_image_dimensions_world, half_image_dimensions_pixels);
            const Vector image_centre_to_pixel_centre {
                (x - 1) * image_dimension / 3.0f, (1 - y) * image_dimension / 3.0f, 0.0f
            };
            const Vector pixel_centre = image_centre + image_centre_to_pixel_centre;
            REQUIRE(are_equal(direction, normalise(pixel_centre - camera.eye)));
        }
    }
}

// TODO: Ellipsoid intersections?
// TODO: Scene intersections?
