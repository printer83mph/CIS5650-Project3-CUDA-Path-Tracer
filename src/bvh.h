#pragma once

#include "sceneStructs.h"

#include <glm/glm.hpp>

#include <vector>

#define MAX_PRIMITIVES_PER_NODE

namespace BVH {

// Input for tree building
template <typename T> struct Primitive {
    AABB bounds;
    T data;
};

// Tree output
template <typename T> struct FlatNode {
    AABB bounds;
    bool is_leaf;
    T leaf_data;
    size_t child_offsets[MAX_PRIMITIVES_PER_NODE];
};

template <typename T> FlatNode<T> buildTree(const std::vector<Primitive<T>> &primitives);

} // namespace BVH