#include "input_parsing.h"
#include "ray_tracing.h"
#include "maths.h"

#include <iostream>
#include <sstream>
#include <variant>

#include <chrono>

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "include/stb_image_write.h"

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
    
    std::cout << "Ellipsoids:\n";
    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Ellipsoid& ellipsoid = scene.ellipsoids[i];
        const Sphere& sphere = ellipsoid.sphere;
        const Vector centre = scene.ellipsoid_transforms[i] * sphere.centre;
        std::cout << '\t' << centre << ", " << sphere.radius << std::endl;
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

    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            const Vector ray_direction = ray_direction_through_pixel(x, y, camera_basis_vectors, half_image_dimensions_world, half_image_dimensions_pixels);
            const Ray ray{camera.eye, ray_direction};
            const Colour colour = intersect(ray, scene);
            unsigned char* const pixel = image.pixels + 3 * x + 3 * y * image.width;
            pixel[0] = static_cast<unsigned char>(colour.red * 255.0f);
            pixel[1] = static_cast<unsigned char>(colour.green * 255.0f);
            pixel[2] = static_cast<unsigned char>(colour.blue * 255.0f);
        }
    }

    auto end = std::chrono::steady_clock::now();

    auto diff = std::chrono::duration_cast<std::chrono::seconds>(end - start);
    std::cout << "Ray tracing time: " << diff.count() << std::endl;

    stbi_write_png(image.filename.c_str(), image.width, image.height, 3, image.pixels, 3 * image.width);
    
    return 0;
}
