#include "bvh.h"

#include <memory>

namespace BVH {

// "private" methods
namespace {
template <typename T> struct BuildNode {
    AABB bounds;
    std::vector<std::unique_ptr<BVH::BuildNode<T>>> childNodes;
    std::vector<size_t> childIndices;
};
} // namespace

template <typename T> FlatNode<T> buildTree(const std::vector<Primitive<T>> &primitives) {
    // Step 1: create initially flat tree of BuildNodes
    std::vector<std::unique_ptr<BVH::BuildNode<T>>> buildNodes(primitives.size());

    for (int i = 0; i < primitives.size(); ++i) {
        const Primitive<T> &primitive = primitives[i];
        buildNodes.push_back(std::make_unique(BuildNode<T>{primitive.bounds, {}, {i}}));
    }

    // Step 2: recursively group BuildNodes together

    // Step 3: flatten BuildNodes into FlatNodes
}

} // namespace BVH
