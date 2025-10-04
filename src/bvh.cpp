#include "bvh.h"

#include <algorithm>
#include <cstdio>
#include <memory>

#define MAX_PRIMITIVES_PER_NODE 4

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
    newParentNode->bounds = (*nodes)[0]->bounds; // initialize with first node bounds
    for (size_t i = 1; i < nodes->size(); ++i) {
        auto &node = (*nodes)[i];
        newParentNode->bounds.min = glm::min(newParentNode->bounds.min, node->bounds.min);
        newParentNode->bounds.max = glm::max(newParentNode->bounds.max, node->bounds.max);
    }

    // If we don't need to split, then just move node ownership and return
    if (nodes->size() <= MAX_PRIMITIVES_PER_NODE) {
        newParentNode->isLeaf = false;
        newParentNode->isOneOffFromLeaves = true;

        for (std::unique_ptr<BuildNode<T>> &node : *nodes) {
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

    if (largestAxisSize < FLT_MIN) {
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

#define SPLIT_AT_MEDIAN_INSTEAD_OF_CENTROID 0
#if !SPLIT_AT_MEDIAN_INSTEAD_OF_CENTROID
        // Normal behavior: split by midpoint of full bounds (centroid)
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

#endif
#if SPLIT_AT_MEDIAN_INSTEAD_OF_CENTROID
        // Sort nodes by centroid along the largest axis
        std::sort(nodes->begin(), nodes->end(), [largestAxis](const auto &a, const auto &b) {
            glm::vec3 centroidA = (a->bounds.min + a->bounds.max) * 0.5f;
            glm::vec3 centroidB = (b->bounds.min + b->bounds.max) * 0.5f;
            return centroidA[largestAxis] < centroidB[largestAxis];
        });

        // Split at median
        size_t mid = nodes->size() / 2;
        for (size_t i = 0; i < mid; ++i) {
            leftChildren.push_back(std::move((*nodes)[i]));
        }
        for (size_t i = mid; i < nodes->size(); ++i) {
            rightChildren.push_back(std::move((*nodes)[i]));
        }
        nodes->clear();
#endif

        // Fallback: if split failed to partition, force a split
        if (leftChildren.empty() || rightChildren.empty()) {
            printf("got race case!\n");
            std::vector<std::unique_ptr<BuildNode<T>>> tempVec;
            if (leftChildren.empty()) {
                tempVec = std::move(rightChildren);
            } else {
                tempVec = std::move(leftChildren);
            }
            size_t mid = tempVec.size() / 2;
            leftChildren.clear();
            rightChildren.clear();

            for (size_t i = 0; i < mid; ++i) {
                leftChildren.push_back(std::move(tempVec[i]));
            }
            for (size_t i = mid; i < tempVec.size(); ++i) {
                rightChildren.push_back(std::move(tempVec[i]));
            }
        } else {
            printf("no race case!\n");
        }
    }

    newParentNode->isLeaf = false;
    newParentNode->isOneOffFromLeaves = false;
    newParentNode->children.push_back(groupBuildNodes(&leftChildren));
    newParentNode->children.push_back(groupBuildNodes(&rightChildren));

    return newParentNode;
};

template <typename T> void getFlatTotalNodeCount(const BuildNode<T> &node, int *countPtr) {
    *countPtr += 1;

    if (node.isLeaf || node.isOneOffFromLeaves) {
        return;
    }

    for (const auto &child : node.children) {
        getFlatTotalNodeCount(*child, countPtr);
    }
}

template <typename T> Tree<T> createFlatTree(const BuildNode<T> &rootNode) {
    std::printf("flattening tree...\n");
    Tree<T> flatTree;

    std::vector<const BuildNode<T> *> nodeStack;
    nodeStack.push_back(&rootNode);

    while (!nodeStack.empty()) {
        const BuildNode<T> *node = nodeStack.back();
        nodeStack.pop_back();

        FlatNode flatNode = FlatNode();
        flatNode.bounds = node->bounds;

        // Super base case (rare): we somehow got to a leaf node
        if (node->isLeaf) {
            flatNode.isLeaf = true;
            flatNode.leafInfo.dataStart = flatTree.leafData.size();
            flatNode.leafInfo.dataCount = 1;

            flatTree.leafData.push_back(node->data);
        }
        // Base case: we're one off from leaves
        else if (node->isOneOffFromLeaves) {
            flatNode.isLeaf = true;
            flatNode.leafInfo.dataStart = flatTree.leafData.size();
            flatNode.leafInfo.dataCount = node->children.size();

            for (auto &child : node->children) {
                flatTree.leafData.push_back(child->data);
            }
        }
        // Recursive case: we gotta add children
        else {
            flatNode.isLeaf = false;
            flatNode.childOffsets[0] = flatTree.nodes.size() + 1;

            // Right child will be at current position + 1 + total nodes in left subtree
            int leftNodeCount = 0;
            getFlatTotalNodeCount(*node->children[0], &leftNodeCount);
            flatNode.childOffsets[1] = flatTree.nodes.size() + 1 + leftNodeCount;

            // Push children in reverse order since stack is LIFO
            // This ensures left child is processed first
            for (int i = node->children.size() - 1; i >= 0; --i) {
                nodeStack.push_back(node->children[i].get());
            }
        }

        flatTree.nodes.push_back(flatNode);
    }

    std::printf("tree flattened!\n");
    return flatTree;
}

} // namespace

template <typename T> BVH::Tree<T> buildTree(const std::vector<Primitive<T>> &primitives) {
    // Step 1: create initially flat vector of BuildNodes
    std::vector<std::unique_ptr<BuildNode<T>>> buildNodes;

    std::printf("building bvh tree with %d nodes\n", (int)primitives.size());
    for (int i = 0; i < primitives.size(); ++i) {
        const Primitive<T> &primitive = primitives[i];
        buildNodes.push_back(std::make_unique<BuildNode<T>>(
            BuildNode<T>{primitive.bounds, true, false, {}, primitive.data}));
    }

    // Step 2: recursively group BuildNodes together. this will clear the `buildNodes` vector.
    std::unique_ptr<BuildNode<T>> rootBuildNode = groupBuildNodes(&buildNodes);

    // Step 3: flatten BuildNodes into FlatNodes
    return createFlatTree(*rootBuildNode);
}
template Tree<Geom> buildTree<Geom>(const std::vector<Primitive<Geom>> &);

} // namespace BVH
