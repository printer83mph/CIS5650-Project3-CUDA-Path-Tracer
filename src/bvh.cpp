#include "bvh.h"

#include <limits>
#include <memory>

namespace BVH {

// "private" methods
namespace {

template <typename T> struct BuildNode {
    AABB bounds;
    bool isLeaf;
    bool isOneOffFromLeaves;
    std::vector<std::unique_ptr<BVH::BuildNode<T>>> children;
    T data;
};

template <typename T>
std::unique_ptr<BuildNode<T>> groupBuildNodes(std::vector<std::unique_ptr<BuildNode<T>>> *nodes) {
    if (nodes->empty())
        return nullptr;

    std::unique_ptr<BuildNode<T>> newParentNode = std::make_unique<BuildNode<T>>();

    // compute bounds of all contained primitives
    newParentNode->bounds = (*nodes)[0]->bounds; // initialize dummy bounds
    for (size_t i = 1; i < nodes->size(); ++i) {
        auto &node = (*nodes)[i];
        newParentNode->bounds.min = glm::min(newParentNode->bounds.min, node->bounds.min);
        newParentNode->bounds.max = glm::max(newParentNode->bounds.max, node->bounds.max);
    }

    // If we don't need to split, then just move node ownership and return
    if (nodes->size() <= MAX_PRIMITIVES_PER_NODE) {
        newParentNode->isLeaf = false;
        newParentNode->isOneOffFromLeaves = true;

        for (std::unique_ptr<BuildNode<T>> &node : nodes) {
            newParentNode->children.push_back(std::move(node));
        }

        return newParentNode;
    }

    // create two child nodes and split children
    std::vector<std::unique_ptr<BuildNode<T>>> leftChildren, rightChildren;

    // Figure out the largest axis
    int largestAxis = 0;
    float largestAxisSize = 0;
    for (int axis = 0; axis < 3; ++axis) {
        float axisSize =
            glm::abs(newParentNode->bounds.min[axis] - newParentNode->bounds.max[axis]);
        if (axisSize > largestAxisSize) {
            largestAxis = axis;
            largestAxisSize = axisSize;
        }
    }

    if (largestAxisSize < std::numeric_limits<float>::epsilon()) {
        // Fallback if everything is in the same place and we risk infinite recursion:
        // split roughly in half
        size_t mid = nodes->size() / 2;
        for (size_t i = 0; i < mid; ++i) {
            leftChildren.push_back(std::move((*nodes)[i]));
        }
        for (size_t i = mid; i < nodes->size(); ++i) {
            rightChildren.push_back(std::move((*nodes)[i]));
        }
        nodes->clear();
    } else {
        // Normal behavior: split by midpoint of full bounds
        float splitValue =
            (newParentNode->bounds.min[largestAxis] + newParentNode->bounds.max[largestAxis]) * 0.5;

        // Move nodes to right or left vector
        for (std::unique_ptr<BuildNode<T>> &node : *nodes) {
            glm::vec3 centroid = (node->bounds.max + node->bounds.min) * 0.5f;

            if (centroid[largestAxis] < splitValue)
                leftChildren.push_back(std::move(node));
            else
                rightChildren.push_back(std::move(node));
        }
        nodes->clear();
    }

    newParentNode->isLeaf = false;
    newParentNode->isOneOffFromLeaves = false;
    newParentNode->children.push_back(groupBuildNodes(&leftChildren));
    newParentNode->children.push_back(groupBuildNodes(&rightChildren));

    return newParentNode;
};

template <typename T> Tree<T> createFlatTree(const BuildNode<T> &rootNode) {
    Tree<T> flatTree;
    std::vector<const BuildNode<T> *> nodeStack;
    nodeStack.push_back(&rootNode);

    // We basically run DFS and do some extra funky logic to track data indices
    while (!nodeStack.empty()) {
        const BuildNode<T> *currentNode = nodeStack.back();
        nodeStack.pop_back();

        FlatNode flatNode;
        flatNode.bounds = currentNode->bounds;
        flatNode.isLeaf = currentNode->isLeaf;

        if (currentNode->isLeaf) {
            // Leaf node: store data info (this should basically never happen, but just in case)
            flatNode.leafInfo.dataStart = flatTree.leafData.size();
            flatNode.leafInfo.dataCount = 1;
            flatTree.leafData.push_back(currentNode->data);
        } else if (currentNode->isOneOffFromLeaves) {
            // One off from leaves: we store all "children" in a single leaf node
            flatNode.isLeaf = true;
            flatNode.leafInfo.dataStart = flatTree.leafData.size();
            flatNode.leafInfo.dataCount = currentNode->children.size();

            for (const auto &child : currentNode->children) {
                flatTree.leafData.push_back(child->data);
            }
        } else {
            // Branch node: store ye olde child offsets
            flatNode.childOffsets[0] = flatTree.nodes.size() + nodeStack.size() + 1;
            flatNode.childOffsets[1] = flatTree.nodes.size() + nodeStack.size() + 2;

            // Add children to stack in funky reverse order so left runs first
            nodeStack.push_back(currentNode->children[1].get());
            nodeStack.push_back(currentNode->children[0].get());
        }

        flatTree.nodes.push_back(flatNode);
    }

    return flatTree;
}

} // namespace

template <typename T> std::vector<FlatNode> buildTree(const std::vector<Primitive<T>> &primitives) {
    // Step 1: create initially flat vector of BuildNodes
    std::vector<std::unique_ptr<BuildNode<T>>> buildNodes;

    for (int i = 0; i < primitives.size(); ++i) {
        const Primitive<T> &primitive = primitives[i];
        buildNodes.push_back(std::make_unique<BuildNode<T>>(
            BuildNode<T>{primitive.bounds, true, {}, static_cast<T>(i)}));
    }

    // Step 2: recursively group BuildNodes together. this will clear the `buildNodes` vector.
    std::unique_ptr<BuildNode<T>> rootBuildNode = groupBuildNodes(&buildNodes);

    // Step 3: flatten BuildNodes into FlatNodes
    return createFlatTree(*rootBuildNode);
}

} // namespace BVH
