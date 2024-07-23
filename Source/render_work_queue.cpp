#include "render_work_queue.h"

static void push_entry(RenderWorkQueue& queue, const RenderWorkQueue::Entry& entry) {
    assert(queue.entry_count < queue.entries.capacity());
    
    queue.entries[queue.entry_count] = entry;
    
    ++queue.entry_count;
}

static bool work_in_progress(RenderWorkQueue& queue) {
    return (queue.completed_entry_count != queue.entry_count);
}

static void render_scanline(
    const Scene& scene,
    const Camera& camera,
    const BasisVectors& camera_basis_vectors,
    const Image& image,
    const Dimensions& half_image_dimensions_world,
    const unsigned y,
    const int max_recursion_depth
) {
    static constexpr int SQRT_SAMPLES_PER_PIXEL = 4;
    
    static constexpr float inverse_sqrt_samples_per_pixel = 1.0f / static_cast<float>(SQRT_SAMPLES_PER_PIXEL);
    static constexpr float inverse_double_sqrt_samples_per_pixel = 0.5f * SQRT_SAMPLES_PER_PIXEL;
    
    static constexpr int samples_per_pixel = SQRT_SAMPLES_PER_PIXEL * SQRT_SAMPLES_PER_PIXEL;
    static constexpr float inverse_samples_per_pixel = 1.0f / static_cast<float>(samples_per_pixel);

    const Dimensions half_image_dimensions_pixels {
        0.5f * static_cast<float>(image.width),
        0.5f * static_cast<float>(image.height)
    };

    for (unsigned x = 0; x < image.width; ++x) {
        Colour colour{0.0f, 0.0f, 0.0f};
        for (int sample = 0; sample < samples_per_pixel; ++sample) {
            const float x_offset = static_cast<float>(sample % SQRT_SAMPLES_PER_PIXEL) * inverse_sqrt_samples_per_pixel + inverse_double_sqrt_samples_per_pixel;
            const float y_offset = static_cast<float>(sample / SQRT_SAMPLES_PER_PIXEL) * inverse_sqrt_samples_per_pixel + inverse_double_sqrt_samples_per_pixel;

            const Vector ray_direction = ray_direction_through_pixel(x, y, x_offset, y_offset, camera_basis_vectors, half_image_dimensions_world, half_image_dimensions_pixels);
            const Ray ray{camera.eye, ray_direction};
            const AABBIntersectionResult bounding_box_intersection = intersect(ray, scene.bounding_box);
            if (bounding_box_intersection.max_distance >= bounding_box_intersection.min_distance) {
                colour += intersect(ray, scene, max_recursion_depth);
            }
        }

        colour = inverse_samples_per_pixel * colour;
        unsigned char* const pixel = image.pixels + 3 * x + 3 * y * image.width;
        pixel[0] = static_cast<unsigned char>(colour.red * 255.0f);
        pixel[1] = static_cast<unsigned char>(colour.green * 255.0f);
        pixel[2] = static_cast<unsigned char>(colour.blue * 255.0f);
    }
}

static void thread_proc(RenderWorkQueue& work_queue) {
    while (work_in_progress(work_queue)) {
        int original_entry_to_do_index = work_queue.next_entry_to_do_index.load();
        if (original_entry_to_do_index < work_queue.entry_count) {
            const bool index_incremented = work_queue.next_entry_to_do_index.compare_exchange_weak(original_entry_to_do_index, original_entry_to_do_index + 1);
            if (index_incremented) {
                const RenderWorkQueue::Entry& entry_to_do = work_queue.entries[original_entry_to_do_index];
                render_scanline(
                    *entry_to_do.scene,
                    *entry_to_do.camera,
                    *entry_to_do.camera_basis_vectors,
                    *entry_to_do.image,
                    entry_to_do.half_image_dimensions_world,
                    entry_to_do.y,
                    entry_to_do.max_recursion_depth
                );
                
                ++work_queue.completed_entry_count;
            }
        }
    }
}
