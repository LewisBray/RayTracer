#ifndef KD_TREE_H
#define KD_TREE_H

#include <vector>

#include "maths.h"

namespace Axis {
    constexpr int X = 0;
    constexpr int Y = 1;
    constexpr int Z = 2;
}

struct Node {
    AxisAlignedBoundingBox bounding_box;

    std::vector<std::uint32_t> triangle_indices;
    // std::vector<std::size_t> sphere_indices;
    // std::vector<std::size_t> ellipsoid_indices;
    
    Node* left;
    Node* right;
    
    int axis;
    int padding;    // TODO: remove
};

static Node* add_node(
    const std::vector<std::uint32_t>& triangle_indices,
    const std::vector<AxisAlignedBoundingBox>& triangle_bounding_boxes,
    Node* node
);

#endif
