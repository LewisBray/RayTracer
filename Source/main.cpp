#include <thread>
#include <cstdio>

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Weverything"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "include/stb_image_write.h"
#pragma clang diagnostic pop

#include "maths.h"
#include "ray_tracing.h"
#include "input_parsing.h"
#include "render_work_queue.h"

// TODO: has to be included before other .cpp files to work, maybe have some sort of header to fix this?
// TODO: doesn't work with multithreading, maybe have separate contexts for each thread?
// #define PROFILING
#include "profiling.cpp"

#include "maths.cpp"
#include "ray_tracing.cpp"
#include "input_parsing.cpp"
#include "render_work_queue.cpp"

int main(const int argc, const char* const argv[]) {
    if (argc < 2) {
        std::puts("Specify an input file.");
        return -1;
    }

    profiling::initialise();
    
    const ParseInputFileResult file_parsing_result = parse_input_file(argv[1]);
    if (file_parsing_result.error != nullptr) {
        std::printf("Failed to parse input file. %s\n", file_parsing_result.error);
        return -1;
    }

    const FileInfo& file_info = file_parsing_result.file_info;
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

    RenderWorkQueue work_queue = {};
    work_queue.entries.resize(image.height);
    
    {
        TIME_BLOCK("ray tracing");
        for (unsigned y = 0; y < image.height; ++y) {
            RenderWorkQueue::Entry entry = {};
            entry.scene = &scene;
            entry.camera = &camera;
            entry.camera_basis_vectors = &camera_basis_vectors;
            entry.image = &image;
            entry.half_image_dimensions_world = half_image_dimensions_world;
            entry.y = y;
            entry.max_recursion_depth = file_info.max_recursion_depth;
            
            push_entry(work_queue, entry);
        }
        
        const unsigned int core_count = std::thread::hardware_concurrency();
        const unsigned int worker_core_count = core_count - 1;
        std::vector<std::thread> worker_threads;
        worker_threads.reserve(worker_core_count);
        for (unsigned int core_index = 0; core_index < worker_core_count; ++core_index) {
            worker_threads.emplace_back([&work_queue]() { thread_proc(work_queue); });
        }
        
        // have the main thread participate in doing work until all entries are done
        thread_proc(work_queue);
                
        for (unsigned int core_index = 0; core_index < worker_core_count; ++core_index) {
            worker_threads[core_index].join();
        }
    }

    stbi_write_png(image.filename, static_cast<int>(image.width), static_cast<int>(image.height), 3, image.pixels, 3 * static_cast<int>(image.width));

    profiling::print_collected_data();
    
    return 0;
}
