// TODO:
//  - improve tolerance issues
//  - add profiling
//  - reduce reliance on c/c++ std library
//  - SIMD routines
//  - multithreading

#include <variant>
#include <cstdio>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "include/stb_image_write.h"
#pragma clang diagnostic pop

#include "maths.h"
#include "ray_tracing.h"
#include "input_parsing.h"

// TODO: has to be included before other .cpp files to work, maybe have some sort of header to fix this?
#define PROFILING
#include "profiling.cpp"

#include "maths.cpp"
#include "ray_tracing.cpp"
#include "input_parsing.cpp"

int main(const int argc, const char* const argv[]) {
    if (argc < 2) {
        std::puts("Specify an input file.");
        return -1;
    }

    profiling::initialise();
    
    const std::variant<FileInfo, const char*> file_parsing_result = parse_input_file(argv[1]);
    if (std::holds_alternative<const char*>(file_parsing_result)) {
        const char* const error_message = std::get<const char*>(file_parsing_result);
        std::printf("Failed to parse input file. %s\n", error_message);
        return -1;
    }

    const FileInfo& file_info = std::get<FileInfo>(file_parsing_result);
    const Scene& scene = file_info.scene;
    const Image& image = file_info.image;
    const Camera& camera = file_info.camera;

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

    {
        TIME_BLOCK("ray tracing");
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
                        colour += intersect(ray, scene, file_info.max_recursion_depth);
                    }
                }

                colour = inverse_samples_per_pixel * colour;
                unsigned char* const pixel = image.pixels + 3 * x + 3 * y * image.width;
                pixel[0] = static_cast<unsigned char>(colour.red * 255.0f);
                pixel[1] = static_cast<unsigned char>(colour.green * 255.0f);
                pixel[2] = static_cast<unsigned char>(colour.blue * 255.0f);
            }
        }
    }

    stbi_write_png(image.filename.c_str(), static_cast<int>(image.width), static_cast<int>(image.height), 3, image.pixels, 3 * static_cast<int>(image.width));

    profiling::print_collected_data();
    
    return 0;
}
