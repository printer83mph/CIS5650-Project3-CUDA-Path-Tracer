#pragma once

#include "sceneStructs.h"

#include <glm/glm.hpp>

#include <vector>

namespace BVH {

// Input for tree building
template <typename T> struct Primitive {
    AABB bounds;
    T data;
};

// Output node
struct FlatNode {
    AABB bounds;
    bool isLeaf;
    union {
        size_t childOffsets[2]; // For non-leaf nodes
        struct {                // For leaf nodes
            size_t dataStart;   // Start index for data array
            size_t dataCount;   // Number of primitives
        } leafInfo;
    };
};

// Tree output
template <typename T> struct Tree {
    std::vector<FlatNode> nodes;
    std::vector<T> leafData;
};

template <typename T> Tree<T> buildTree(const std::vector<Primitive<T>> &primitives);

} // namespace BVH