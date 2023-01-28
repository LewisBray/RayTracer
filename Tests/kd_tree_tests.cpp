#include "test_framework.h"

#include "../Source/maths.cpp"
#include "../Source/kd_tree.cpp"
#include "test_utils.cpp"

static bool are_equal(const AxisAlignedBoundingBox& lhs, const AxisAlignedBoundingBox& rhs) noexcept {
    return are_equal(lhs.min_x, rhs.min_x) && are_equal(lhs.max_x, rhs.max_x) &&
        are_equal(lhs.min_y, rhs.min_y) && are_equal(lhs.max_y, rhs.max_y) &&
        are_equal(lhs.min_z, rhs.min_z) && are_equal(lhs.max_z, rhs.max_z);
}

TEST(tree_square) {
    const AxisAlignedBoundingBox square{-1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox bottom_left{-1.0f, 0.0f, -1.0f, 0.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox top_left{-1.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox left{-1.0f, 0.0f, -1.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox right{0.0f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f};

    const std::vector<AxisAlignedBoundingBox> bounding_boxes{bottom_left, top_left, right};
    const std::vector<std::uint32_t> indices{0, 1, 2};

    Node* tree = new Node;
    tree->bounding_box = square;
    tree->left = nullptr;
    tree->right = nullptr;

    tree = add_node(indices, bounding_boxes, tree);

    EXPECT_TRUE(tree->triangle_indices.empty());

    const Node* const left_node = tree->left;
    EXPECT_TRUE(left_node != nullptr);
    EXPECT_TRUE(are_equal(left_node->bounding_box, left));
    EXPECT_EQ(left_node->triangle_indices.size(), 2);
    EXPECT_EQ(left_node->triangle_indices[0], 0);
    EXPECT_EQ(left_node->triangle_indices[1], 1);
    EXPECT_EQ(left_node->left, nullptr);
    EXPECT_EQ(left_node->right, nullptr);

    const Node* const right_node = tree->right;
    EXPECT_TRUE(right_node != nullptr);
    EXPECT_TRUE(are_equal(right_node->bounding_box, right));
    EXPECT_EQ(right_node->triangle_indices.size(), 1);
    EXPECT_EQ(right_node->triangle_indices.front(), 2);
    EXPECT_EQ(right_node->left, nullptr);
    EXPECT_EQ(right_node->right, nullptr);
}

TEST(tree_beyond_square) {
    const AxisAlignedBoundingBox square{-1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox to_left{-2.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_top_half{-1.0f, 1.0f, 0.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_bottom_half{-1.0f, 1.0f, -1.0f, 0.0f, 0.0f, 0.0f};

    const std::vector<AxisAlignedBoundingBox> bounding_boxes{to_left, square_bottom_half};
    const std::vector<std::uint32_t> indices{0, 1};

    Node* tree = new Node;
    tree->bounding_box = square;
    tree->left = nullptr;
    tree->right = nullptr;

    tree = add_node(indices, bounding_boxes, tree);

    EXPECT_TRUE(tree->triangle_indices.empty());

    const Node* const left_node = tree->left;
    EXPECT_TRUE(left_node != nullptr);
    EXPECT_TRUE(are_equal(left_node->bounding_box, square_bottom_half));
    EXPECT_EQ(left_node->triangle_indices.size(), 1);
    EXPECT_EQ(left_node->triangle_indices.front(), 1);
    EXPECT_EQ(left_node->left, nullptr);
    EXPECT_EQ(left_node->right, nullptr);

    const Node* const right_node = tree->right;
    EXPECT_TRUE(right_node != nullptr);
    EXPECT_TRUE(are_equal(right_node->bounding_box, square_top_half));
    EXPECT_EQ(right_node->triangle_indices.size(), 1);
    EXPECT_EQ(right_node->triangle_indices.front(), 0);
    EXPECT_EQ(right_node->left, nullptr);
    EXPECT_EQ(right_node->right, nullptr);
}

TEST(tree_multiple_beyond_square_perpendicular) {
    const AxisAlignedBoundingBox square{-3.0f, 3.0f, -3.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_top{-3.0f, 3.0f, 1.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_middle{-3.0f, 3.0f, -1.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_bottom{-3.0f, 3.0f, -3.0f, -1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_top_and_middle{-3.0f, 3.0f, -1.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox to_left{-4.0f, 2.0f, 1.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox to_left_and_right{-4.0f, 4.0f, -1.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox to_right{-2.0f, 4.0f, -3.0f, -1.0f, 0.0f, 0.0f};

    const std::vector<AxisAlignedBoundingBox> bounding_boxes{to_left, to_left_and_right, to_right};
    const std::vector<std::uint32_t> indices{0, 1, 2};

    Node* tree = new Node;
    tree->bounding_box = square;
    tree->left = nullptr;
    tree->right = nullptr;

    tree = add_node(indices, bounding_boxes, tree);

    EXPECT_TRUE(tree->triangle_indices.empty());

    const Node* const left_node = tree->left;
    EXPECT_TRUE(left_node != nullptr);
    EXPECT_TRUE(are_equal(left_node->bounding_box, square_bottom));
    EXPECT_EQ(left_node->triangle_indices.size(), 1);
    EXPECT_EQ(left_node->triangle_indices.front(), 2);
    EXPECT_EQ(left_node->left, nullptr);
    EXPECT_EQ(left_node->right, nullptr);

    const Node* const right_node = tree->right;
    EXPECT_TRUE(right_node != nullptr);
    EXPECT_TRUE(are_equal(right_node->bounding_box, square_top_and_middle));
    EXPECT_EQ(right_node->triangle_indices.size(), 2);
    EXPECT_EQ(right_node->triangle_indices[0], 0);
    EXPECT_EQ(right_node->triangle_indices[1], 1);
    EXPECT_EQ(right_node->left, nullptr);
    EXPECT_EQ(right_node->right, nullptr);
}

TEST(tree_multiple_beyond_square_parallel) {
    const AxisAlignedBoundingBox square{-3.0f, 3.0f, -3.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_left{-3.0f, -1.0f, -3.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_middle{-1.0f, 1.0f, -3.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_right{1.0f, 3.0f, -3.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox square_middle_and_right{-1.0f, 3.0f, -3.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox overlapping_left{-4.0f, -1.0f, -3.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox in_middle{-1.0f, 1.0f, -3.0f, 3.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox overlapping_right{1.0f, 4.0f, -3.0f, 3.0f, 0.0f, 0.0f};

    const std::vector<AxisAlignedBoundingBox> bounding_boxes{overlapping_left, in_middle, overlapping_right};
    const std::vector<std::uint32_t> indices{0, 1, 2};

    Node* tree = new Node;
    tree->bounding_box = square;
    tree->left = nullptr;
    tree->right = nullptr;

    tree = add_node(indices, bounding_boxes, tree);

    EXPECT_TRUE(tree->triangle_indices.empty());

    const Node* const left_node = tree->left;
    EXPECT_TRUE(left_node != nullptr);
    EXPECT_TRUE(are_equal(left_node->bounding_box, square_left));
    EXPECT_EQ(left_node->triangle_indices.size(), 1);
    EXPECT_EQ(left_node->triangle_indices.front(), 0);
    EXPECT_EQ(left_node->left, nullptr);
    EXPECT_EQ(left_node->right, nullptr);

    const Node* const right_node = tree->right;
    EXPECT_TRUE(right_node != nullptr);
    EXPECT_TRUE(are_equal(right_node->bounding_box, square_middle_and_right));
    EXPECT_EQ(right_node->triangle_indices.size(), 2);
    EXPECT_EQ(right_node->triangle_indices[0], 1);
    EXPECT_EQ(right_node->triangle_indices[1], 2);
    EXPECT_EQ(right_node->left, nullptr);
    EXPECT_EQ(right_node->right, nullptr);
}

TEST(overlapping_bounding_boxes) {
    const AxisAlignedBoundingBox square{-1.0f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox to_left{-1.0f, 0.5f, -1.0f, 1.0f, 0.0f, 0.0f};
    const AxisAlignedBoundingBox to_right{-0.5f, 1.0f, -1.0f, 1.0f, 0.0f, 0.0f};

    const std::vector<AxisAlignedBoundingBox> bounding_boxes{to_left, to_right};
    const std::vector<std::uint32_t> indices{0, 1};

    Node* tree = new Node;
    tree->bounding_box = square;
    tree->left = nullptr;
    tree->right = nullptr;

    tree = add_node(indices, bounding_boxes, tree);

    EXPECT_EQ(tree->triangle_indices.size(), 2);
    EXPECT_EQ(tree->triangle_indices[0], 0);
    EXPECT_EQ(tree->triangle_indices[1], 1);
    EXPECT_EQ(tree->left, nullptr);
    EXPECT_EQ(tree->right, nullptr);
}
