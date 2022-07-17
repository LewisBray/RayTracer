#include <iostream>
#include <sstream>
#include <variant>
#include <chrono>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "include/stb_image_write.h"
#pragma clang diagnostic pop

#include "maths.h"
#include "ray_tracing.h"
#include "input_parsing.h"

#include "maths.cpp"
#include "ray_tracing.cpp"
#include "input_parsing.cpp"

int main(const int argc, const char* const argv[]) {
    if (argc < 2) {
        std::cout << "Specify an input file." << std::endl;
        return -1;
    }
    
    const std::variant<FileInfo, const char*> file_parsing_result = parse_input_file(argv[1]);
    if (std::holds_alternative<const char*>(file_parsing_result)) {
        const char* const error_message = std::get<const char*>(file_parsing_result);
        std::cout << "Failed to parse input file.  " << error_message << std::endl;
        return -1;
    }

    const FileInfo& file_info = std::get<FileInfo>(file_parsing_result);
    std::cout << "File info:" << std::endl;
    const Image& image = file_info.image;
    std::cout
        << "Image: \n\tfilename - " << image.filename
        << "\n\twidth - " << image.width
        << "\n\theight - " << image.height
        << std::endl;
    
    const Camera& camera = file_info.camera;
    std::cout
        << "Camera: \n\teye - " << camera.eye
        << "\n\tlook at - " << camera.look_at
        << "\n\tup - " << camera.up
        << "\n\tfield of view - (" << camera.field_of_view.x << ", " << camera.field_of_view.y << ')'
        << std::endl;
    
    constexpr auto to_string = [](const Colour& colour) {
        std::stringstream ss;
        ss << '(' << colour.red << ", " << colour.green << ", " << colour.blue << ')';
        return ss.str();
    };
    
    const Scene& scene = file_info.scene;
    
    std::cout << "Triangles:\n";
    for (const Triangle& triangle : scene.triangles) {
        std::cout << '\t' << triangle.a << ", " << triangle.b << ", " << triangle.c << std::endl;
    }

    std::cout << "Spheres:\n";
    for (const Sphere& sphere : scene.spheres) {
        std::cout << "\tCentre: " << sphere.centre << " radius: " << sphere.radius << std::endl;
    }
    
    std::cout << "Ellipsoids:\n";
    for (const Matrix& ellipsoid_transform : scene.ellipsoid_transforms) {
        std::cout
            << '(' << ellipsoid_transform[0][0] << ", " << ellipsoid_transform[0][1] << ", " << ellipsoid_transform[0][2] << ", " << ellipsoid_transform[0][3] << ')' << std::endl
            << '(' << ellipsoid_transform[1][0] << ", " << ellipsoid_transform[1][1] << ", " << ellipsoid_transform[1][2] << ", " << ellipsoid_transform[1][3] << ')' << std::endl
            << '(' << ellipsoid_transform[2][0] << ", " << ellipsoid_transform[2][1] << ", " << ellipsoid_transform[2][2] << ", " << ellipsoid_transform[2][3] << ")," << std::endl;
    }

    if (scene.directional_light_source.has_value()) {
        const DirectionalLightSource& directional_light_source = scene.directional_light_source.value();
        std::cout
            << "Directional light: \n\tdirection - " << directional_light_source.direction
            << "\n\tcolour - " << to_string(directional_light_source.colour)
            << std::endl;
    }

    std::cout << "Point lights:\n";
    for (const PointLightSource& point_light_source : scene.point_light_sources) {
        std::cout << "\tPosition: " << point_light_source.position << " Colour: " << to_string(point_light_source.colour) << std::endl;
    }

    std::cout << "Ambient: (" << scene.ambient.red << ", " << scene.ambient.green << ", " << scene.ambient.blue << ')' << std::endl;

    std::cout << "Max recursion depth:\n\t" << file_info.max_recursion_depth << std::endl;

    std::cout << "bounding box: " << std::endl
              << scene.bounding_box.min_x << std::endl
              << scene.bounding_box.max_x << std::endl
              << scene.bounding_box.min_y << std::endl
              << scene.bounding_box.max_y << std::endl
              << scene.bounding_box.min_z << std::endl
              << scene.bounding_box.max_z << std::endl;

    auto start = std::chrono::steady_clock::now();

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

    constexpr int sqrt_samples_per_pixel = 1;
    constexpr float inverse_sqrt_samples_per_pixel = 1.0f / static_cast<float>(sqrt_samples_per_pixel);
    constexpr float inverse_double_sqrt_samples_per_pixel = 0.5f * inverse_sqrt_samples_per_pixel;
    
    constexpr int samples_per_pixel = sqrt_samples_per_pixel * sqrt_samples_per_pixel;
    constexpr float inverse_samples_per_pixel = 1.0f / static_cast<float>(samples_per_pixel);

    for (unsigned y = 0; y < image.height; ++y) {
        for (unsigned x = 0; x < image.width; ++x) {
            Colour colour{0.0f, 0.0f, 0.0f};
            for (int sample = 0; sample < samples_per_pixel; ++sample) {
                const float x_offset = static_cast<float>(sample % sqrt_samples_per_pixel) * inverse_sqrt_samples_per_pixel + inverse_double_sqrt_samples_per_pixel;
                const float y_offset = static_cast<float>(sample / sqrt_samples_per_pixel) * inverse_sqrt_samples_per_pixel + inverse_double_sqrt_samples_per_pixel;

                const Vector ray_direction = ray_direction_through_pixel(x, y, x_offset, y_offset, camera_basis_vectors, half_image_dimensions_world, half_image_dimensions_pixels);
                const Ray ray{camera.eye, ray_direction};
                const std::pair<float, float> bounding_box_intersection_times = intersect(ray, scene.bounding_box);
                if (bounding_box_intersection_times.second >= bounding_box_intersection_times.first) {
                    colour += intersect(ray, scene);
                }
            }

            colour = inverse_samples_per_pixel * colour;
            unsigned char* const pixel = image.pixels + 3 * x + 3 * y * image.width;
            pixel[0] = static_cast<unsigned char>(colour.red * 255.0f);
            pixel[1] = static_cast<unsigned char>(colour.green * 255.0f);
            pixel[2] = static_cast<unsigned char>(colour.blue * 255.0f);
        }
    }

    auto end = std::chrono::steady_clock::now();

    auto diff = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Ray tracing time: " << diff.count() << std::endl;

    stbi_write_png(image.filename.c_str(), static_cast<int>(image.width), static_cast<int>(image.height), 3, image.pixels, 3 * static_cast<int>(image.width));
    
    return 0;
}
