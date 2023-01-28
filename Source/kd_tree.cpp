#include "ray_tracing.h"
#include "kd_tree.h"
#include "maths.h"

#include <algorithm>    // TODO: remove if replacing with radix sort
#include <cassert>
#include <limits>
#include <vector>

#include <iostream> // TODO: remove

static void print(const Node* const node) {
    if (node == nullptr) {
        return;
    }

    std::cout
        << node->bounding_box.min_x << ", "
        << node->bounding_box.max_x << ", "
        << node->bounding_box.min_y << ", "
        << node->bounding_box.max_y << ", "
        << node->bounding_box.min_z << ", "
        << node->bounding_box.max_z << std::endl;
    for (auto index : node->triangle_indices) {
        std::cout << index << ", ";
    }
    // std::cout << node->triangle_indices.size();
    std::cout << std::endl;

    print(node->left);
    print(node->right);
}

static float surface_area(const AxisAlignedBoundingBox& bounding_box) noexcept {
    assert(bounding_box.max_x >= bounding_box.min_x);
    assert(bounding_box.max_y >= bounding_box.min_y);
    assert(bounding_box.max_z >= bounding_box.min_z);

    const float width = bounding_box.max_x - bounding_box.min_x;
    const float height = bounding_box.max_y - bounding_box.min_y;
    const float depth = bounding_box.max_z - bounding_box.min_z;

    const float xy_face_area = width * height;
    const float xz_face_area = width * depth;
    const float yz_face_area = height * depth;

    return 2.0f * (xy_face_area + xz_face_area + yz_face_area);
}

enum class Event {
    BEGIN = 0,
    END = 1
};

struct SplitInfo {
    float position;
    Event event;
};

static bool split_info_less_than(const SplitInfo& lhs, const SplitInfo& rhs) noexcept {
    if (less_than(lhs.position, rhs.position)) {
        return true;
    } else if (are_equal(lhs.position, rhs.position)) {
        return lhs.event < rhs.event;
    } else {
        return false;
    }
}

static Node* add_node(
    const std::vector<std::uint32_t>& triangle_indices,
    const std::vector<AxisAlignedBoundingBox>& triangle_bounding_boxes,
    Node* node
) {
    assert(!triangle_indices.empty());
    assert(!triangle_bounding_boxes.empty());
    assert(triangle_indices.size() <= triangle_bounding_boxes.size());

    const AxisAlignedBoundingBox& parent_bounding_box = node->bounding_box;
    // std::cout
    //     << parent_bounding_box.min_x << ", "
    //     << parent_bounding_box.max_x << ", "
    //     << parent_bounding_box.min_y << ", "
    //     << parent_bounding_box.max_y << ", "
    //     << parent_bounding_box.min_z << ", "
    //     << parent_bounding_box.max_z << std::endl;
    // for (auto index : triangle_indices) {
    //     std::cout << index << ", ";
    // }
    // std::cout << std::endl << std::endl;

    const std::size_t events_count = 2 * triangle_indices.size();

    std::vector<SplitInfo> split_infos[3] = {};
    split_infos[0].reserve(events_count);
    split_infos[1].reserve(events_count);
    split_infos[2].reserve(events_count);
    for (const std::uint32_t triangle_index : triangle_indices) {
        const AxisAlignedBoundingBox& triangle_bounding_box = triangle_bounding_boxes[triangle_index];
        for (int axis = Axis::X; axis < 3; axis += 1) {
            const float triangle_bounding_box_min =
                (axis == Axis::X) * triangle_bounding_box.min_x +
                (axis == Axis::Y) * triangle_bounding_box.min_y +
                (axis == Axis::Z) * triangle_bounding_box.min_z;
            split_infos[axis].emplace_back(SplitInfo{triangle_bounding_box_min, Event::BEGIN});

            const float triangle_bounding_box_max =
                (axis == Axis::X) * triangle_bounding_box.max_x +
                (axis == Axis::Y) * triangle_bounding_box.max_y +
                (axis == Axis::Z) * triangle_bounding_box.max_z;
            split_infos[axis].emplace_back(SplitInfo{triangle_bounding_box_max, Event::END});
        }
    }

    for (int axis = Axis::X; axis < 3; ++axis) {
        std::sort(split_infos[axis].begin(), split_infos[axis].end(), split_info_less_than);
    }

    constexpr float infinity = std::numeric_limits<float>::infinity();

    const float parent_surface_area = surface_area(parent_bounding_box);

    const float parent_bounding_box_mins[3] = {
        parent_bounding_box.min_x,
        parent_bounding_box.min_y,
        parent_bounding_box.min_z
    };

    const float parent_bounding_box_maxs[3] = {
        parent_bounding_box.max_x,
        parent_bounding_box.max_y,
        parent_bounding_box.max_z
    };

    float minimum_split_positions[3] = {};
    float minimum_split_costs[3] = {infinity, infinity, infinity};
    for (int axis = Axis::X; axis < 3; ++axis) {
        std::size_t index = 0;
        std::size_t current_lhs_primitives_count = 0;
        std::size_t current_rhs_primitives_count = triangle_bounding_boxes.size();
        
        while (index < events_count && (split_infos[axis][index].position - parent_bounding_box_mins[axis] < tolerance) && split_infos[axis][index].event == Event::BEGIN) {
            ++current_lhs_primitives_count;
            ++index;
        }

        while (index < events_count && are_equal(split_infos[axis][index].position, parent_bounding_box_mins[axis]) && split_infos[axis][index].event == Event::END) {
            --current_rhs_primitives_count;
            ++index;
        }

        const std::size_t start_index = index;

        index = events_count;
        while (index > 1 && (split_infos[axis][index - 1].position - parent_bounding_box_maxs[axis] > -tolerance)) {
            --index;
        }

        const std::size_t end_index = index;

        std::size_t split_index = start_index;
        assert(!split_infos[axis].empty());
        while (split_index < end_index) {
            const SplitInfo& current_split_info = split_infos[axis][split_index];
            const float split_position = current_split_info.position;
            
            std::size_t begin_events_count = 0;
            while (split_index < end_index && are_equal(split_infos[axis][split_index].position, split_position) && split_infos[axis][split_index].event == Event::BEGIN) {
                ++begin_events_count;
                ++split_index;
            }

            std::size_t end_events_count = 0;
            while (split_index < end_index && are_equal(split_infos[axis][split_index].position, split_position) && split_infos[axis][split_index].event == Event::END) {
                ++end_events_count;
                ++split_index;
            }

            assert(end_events_count <= current_rhs_primitives_count);
            current_rhs_primitives_count -= end_events_count;

            AxisAlignedBoundingBox lhs_split_bounding_box = parent_bounding_box;
            lhs_split_bounding_box.max_x = (axis == Axis::X) ? split_position : lhs_split_bounding_box.max_x;
            lhs_split_bounding_box.max_y = (axis == Axis::Y) ? split_position : lhs_split_bounding_box.max_y;
            lhs_split_bounding_box.max_z = (axis == Axis::Z) ? split_position : lhs_split_bounding_box.max_z;
            const float lhs_surface_area = surface_area(lhs_split_bounding_box);

            AxisAlignedBoundingBox rhs_split_bounding_box = parent_bounding_box;
            rhs_split_bounding_box.min_x = (axis == Axis::X) ? split_position : rhs_split_bounding_box.min_x;
            rhs_split_bounding_box.min_y = (axis == Axis::Y) ? split_position : rhs_split_bounding_box.min_y;
            rhs_split_bounding_box.min_z = (axis == Axis::Z) ? split_position : rhs_split_bounding_box.min_z;
            const float rhs_surface_area = surface_area(rhs_split_bounding_box);

            const float lhs_contribution = lhs_surface_area / parent_surface_area * current_lhs_primitives_count;
            const float rhs_contribution = rhs_surface_area / parent_surface_area * current_rhs_primitives_count;
            const float split_cost = lhs_contribution + rhs_contribution;

            if (split_cost < minimum_split_costs[axis]) {
                minimum_split_positions[axis] = split_position;
                minimum_split_costs[axis] = split_cost;
            }

            current_lhs_primitives_count += begin_events_count;
        }
    }

    int axis_to_split = Axis::X;
    for (int axis = Axis::X; axis < 3; ++axis) {
        if (minimum_split_costs[axis] < minimum_split_costs[axis_to_split]) {
            axis_to_split = axis;
        }
    }

    const float split_cost = minimum_split_costs[axis_to_split];
    const std::size_t no_split_cost = triangle_indices.size();
    
    if (split_cost + 0.5f < no_split_cost) {
        const float split_position = minimum_split_positions[axis_to_split];

        std::vector<std::uint32_t> lhs_indices;
        std::vector<std::uint32_t> rhs_indices;
        for (const std::uint32_t triangle_index : triangle_indices) {
            const AxisAlignedBoundingBox& triangle_bounding_box = triangle_bounding_boxes[triangle_index];
            const float triangle_bounding_box_min =
                (axis_to_split == Axis::X) * triangle_bounding_box.min_x +
                (axis_to_split == Axis::Y) * triangle_bounding_box.min_y +
                (axis_to_split == Axis::Z) * triangle_bounding_box.min_z;

            const float triangle_bounding_box_max =
                (axis_to_split == Axis::X) * triangle_bounding_box.max_x +
                (axis_to_split == Axis::Y) * triangle_bounding_box.max_y +
                (axis_to_split == Axis::Z) * triangle_bounding_box.max_z;

            if (less_than(triangle_bounding_box_max, split_position)) {  // TODO: could this be more simple/less branchy?
                lhs_indices.push_back(triangle_index);
            } else if (are_equal(triangle_bounding_box_max, split_position)) {
                if (less_than(triangle_bounding_box_min, triangle_bounding_box_max)) {
                    lhs_indices.push_back(triangle_index);
                } else {    // planar and lies on split_position line
                    rhs_indices.push_back(triangle_index);
                }
            } else if (less_than(triangle_bounding_box_min, split_position) && less_than(split_position, triangle_bounding_box_max)) {
                lhs_indices.push_back(triangle_index);
                rhs_indices.push_back(triangle_index);
            } else if (are_equal(triangle_bounding_box_min, split_position)) {
                rhs_indices.push_back(triangle_index);
            } else if (less_than(split_position, triangle_bounding_box_min)) {
                rhs_indices.push_back(triangle_index);
            } else {
                assert(false);  // shouldn't get here???
            }
        }

        Node* lhs_node = new Node;
        lhs_node->bounding_box = parent_bounding_box;
        lhs_node->bounding_box.max_x = (axis_to_split == Axis::X) ? split_position : lhs_node->bounding_box.max_x;
        lhs_node->bounding_box.max_y = (axis_to_split == Axis::Y) ? split_position : lhs_node->bounding_box.max_y;
        lhs_node->bounding_box.max_z = (axis_to_split == Axis::Z) ? split_position : lhs_node->bounding_box.max_z;
        lhs_node->left = nullptr;
        lhs_node->right = nullptr;

        assert(!lhs_indices.empty());
        lhs_node = add_node(lhs_indices, triangle_bounding_boxes, lhs_node);
        node->left = lhs_node;

        Node* rhs_node = new Node;
        rhs_node->bounding_box = parent_bounding_box;
        rhs_node->bounding_box.min_x = (axis_to_split == Axis::X) ? split_position : rhs_node->bounding_box.min_x;
        rhs_node->bounding_box.min_y = (axis_to_split == Axis::Y) ? split_position : rhs_node->bounding_box.min_y;
        rhs_node->bounding_box.min_z = (axis_to_split == Axis::Z) ? split_position : rhs_node->bounding_box.min_z;
        rhs_node->left = nullptr;
        rhs_node->right = nullptr;

        assert(!rhs_indices.empty());
        rhs_node = add_node(rhs_indices, triangle_bounding_boxes, rhs_node);
        node->right = rhs_node;
    } else {
        node->triangle_indices = triangle_indices;
    }

    return node;
}
