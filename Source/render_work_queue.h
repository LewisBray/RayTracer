#ifndef RENDER_WORK_QUEUE_H
#define RENDER_WORK_QUEUE_H

#include <atomic>

#include "ray_tracing.h"

struct RenderWorkQueue {
    struct Entry {
        const Scene* scene;
        const Camera* camera;
        const BasisVectors* camera_basis_vectors;
        const Image* image;
        Dimensions half_image_dimensions_world;
        unsigned y;
        int max_recursion_depth;
    };
    
    std::vector<Entry> entries;
    std::atomic<int> entry_count;
    std::atomic<int> next_entry_to_do_index;
    std::atomic<int> completed_entry_count;
};

static void push_entry(RenderWorkQueue& queue, const RenderWorkQueue::Entry& entry);
static bool work_in_progress(RenderWorkQueue& queue);

static void render_scanline(
    const Scene& scene,
    const Camera& camera,
    const BasisVectors& camera_basis_vectors,
    const Image& image,
    const Dimensions& half_image_dimensions_world,
    const unsigned y,
    const int max_recursion_depth
);

static void thread_proc(RenderWorkQueue& work_queue);

#endif
