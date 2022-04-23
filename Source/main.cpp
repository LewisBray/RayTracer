#include "input_parsing.h"
#include "ray_tracing.h"
#include "maths.h"

#include <iostream>
#include <sstream>
#include <variant>

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
    for (std::size_t i = 0; i < scene.triangles.size(); ++i) {
        const Triangle& triangle = scene.triangles[i];
        const Colour& ambient = scene.triangle_ambients[i];
        std::cout << '\t' << triangle.a << ", " << triangle.b << ", " << triangle.c << ": " << to_string(ambient) << std::endl;
    }
    std::cout << "Ellipsoids:\n";
    for (std::size_t i = 0; i < scene.ellipsoids.size(); ++i) {
        const Sphere& sphere = scene.ellipsoids[i].sphere;
        const Colour& ambient = scene.ellipsoid_ambients[i];
        std::cout << '\t' << sphere.centre << ", " << sphere.radius << ": " << to_string(ambient) << std::endl;
    }
    
    std::cout << "Max recursion depth:\n\t" << file_info.max_recursion_depth << std::endl;

    // const Triangle triangle{{0.5, -0.5, 0.0, 1.0}, {0.0, 0.5, 0.0, 1.0}, {-0.5, -0.5, 0.0, 1.0}};
    // const Ray ray{{0.0, 0.0, 5.0, 1.0}, {0.0, 0.05, 3.0, 1.0}};
    // const auto intersection = intersect(ray, triangle);
    // if (intersection.has_value())
    //     std::cout << intersection.value() << std::endl;
    // else
    //     std::cout << "No intersection" << std::endl;

    for (int y = 0; y < image.height; ++y) {
        for (int x = 0; x < image.width; ++x) {
            const Ray ray = ray_through_pixel(camera, x, y, image);
            const Colour colour = intersect(ray, scene, camera.eye);
            unsigned char* const pixel = image.pixels + 3 * x + 3 * y * image.width;
            pixel[0] = static_cast<unsigned char>(colour.red * 255);
            pixel[1] = static_cast<unsigned char>(colour.green * 255);
            pixel[2] = static_cast<unsigned char>(colour.blue * 255);
        }
    }

    stbi_write_png(image.filename.c_str(), image.width, image.height, 3, image.pixels, 3 * image.width);
    
    return 0;
}
