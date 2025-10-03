#include "bvh.h"
#include "intersections.h"
#include "sceneStructs.h"
#include <limits>

__host__ __device__ float pickGeometryIntersectionTest(Geom geom, Ray r,
                                                       glm::vec3 &intersectionPoint,
                                                       glm::vec3 &normal, bool &outside) {
    if (geom.type == CUBE) {
        return boxIntersectionTest(geom, r, intersectionPoint, normal, outside);
    } else if (geom.type == SPHERE) {
        return sphereIntersectionTest(geom, r, intersectionPoint, normal, outside);
    } else if (geom.type == MESH) {
        // Naive implementation: iterate through triangles, run tri intersection on each
        // TODO
        return boxIntersectionTest(geom, r, intersectionPoint, normal, outside);
    }
}

__host__ __device__ float AABBRayIntersectionTest(Ray ray, AABB aabb) {
    float t1 = (aabb.min.x - ray.origin.x) / ray.direction.x;
    float t2 = (aabb.max.x - ray.origin.x) / ray.direction.x;
    float t3 = (aabb.min.y - ray.origin.y) / ray.direction.y;
    float t4 = (aabb.max.y - ray.origin.y) / ray.direction.y;
    float t5 = (aabb.min.z - ray.origin.z) / ray.direction.z;
    float t6 = (aabb.max.z - ray.origin.z) / ray.direction.z;

    float tmin = glm::max(glm::max(glm::min(t1, t2), glm::min(t3, t4)), glm::min(t5, t6));
    float tmax = glm::min(glm::min(glm::max(t1, t2), glm::max(t3, t4)), glm::max(t5, t6));

    // if tmax < 0, ray (line) is intersecting AABB, but whole AABB is behing us
    if (tmax < 0)
        return -1;

    // if tmin > tmax, ray doesn't intersect AABB
    if (tmin > tmax)
        return -1;

    if (tmin < 0.f)
        return tmax;

    return tmin;
}

__host__ __device__ float boxIntersectionTest(
    Geom box,
    Ray r,
    glm::vec3 &intersectionPoint,
    glm::vec3 &normal,
    bool &outside)
{
    Ray q;
    q.origin    =                multiplyMV(box.inverseTransform, glm::vec4(r.origin   , 1.0f));
    q.direction = glm::normalize(multiplyMV(box.inverseTransform, glm::vec4(r.direction, 0.0f)));

    float tmin = -1e38f;
    float tmax = 1e38f;
    glm::vec3 tmin_n;
    glm::vec3 tmax_n;
    for (int xyz = 0; xyz < 3; ++xyz)
    {
        float qdxyz = q.direction[xyz];
        /*if (glm::abs(qdxyz) > 0.00001f)*/
        {
            float t1 = (-0.5f - q.origin[xyz]) / qdxyz;
            float t2 = (+0.5f - q.origin[xyz]) / qdxyz;
            float ta = glm::min(t1, t2);
            float tb = glm::max(t1, t2);
            glm::vec3 n;
            n[xyz] = t2 < t1 ? +1 : -1;
            if (ta > 0 && ta > tmin)
            {
                tmin = ta;
                tmin_n = n;
            }
            if (tb < tmax)
            {
                tmax = tb;
                tmax_n = n;
            }
        }
    }

    if (tmax >= tmin && tmax > 0)
    {
        outside = true;
        if (tmin <= 0)
        {
            tmin = tmax;
            tmin_n = tmax_n;
            outside = false;
        }
        intersectionPoint = multiplyMV(box.transform, glm::vec4(getPointOnRay(q, tmin), 1.0f));
        normal = glm::normalize(multiplyMV(box.invTranspose, glm::vec4(tmin_n, 0.0f)));
        return glm::length(r.origin - intersectionPoint);
    }

    return -1;
}

__host__ __device__ float sphereIntersectionTest(
    Geom sphere,
    Ray r,
    glm::vec3 &intersectionPoint,
    glm::vec3 &normal,
    bool &outside)
{
    float radius = .5;

    glm::vec3 ro = multiplyMV(sphere.inverseTransform, glm::vec4(r.origin, 1.0f));
    glm::vec3 rd = glm::normalize(multiplyMV(sphere.inverseTransform, glm::vec4(r.direction, 0.0f)));

    Ray rt;
    rt.origin = ro;
    rt.direction = rd;

    float vDotDirection = glm::dot(rt.origin, rt.direction);
    float radicand = vDotDirection * vDotDirection - (glm::dot(rt.origin, rt.origin) - powf(radius, 2));
    if (radicand < 0)
    {
        return -1;
    }

    float squareRoot = sqrt(radicand);
    float firstTerm = -vDotDirection;
    float t1 = firstTerm + squareRoot;
    float t2 = firstTerm - squareRoot;

    float t = 0;
    if (t1 < 0 && t2 < 0)
    {
        return -1;
    }
    else if (t1 > 0 && t2 > 0)
    {
        t = min(t1, t2);
        outside = true;
    }
    else
    {
        t = max(t1, t2);
        outside = false;
    }

    glm::vec3 objspaceIntersection = getPointOnRay(rt, t);

    intersectionPoint = multiplyMV(sphere.transform, glm::vec4(objspaceIntersection, 1.f));
    normal = glm::normalize(multiplyMV(sphere.invTranspose, glm::vec4(objspaceIntersection, 0.f)));
    if (!outside)
    {
        normal = -normal;
    }

    return glm::length(r.origin - intersectionPoint);
}

/**
 * @param nodes  BVH tree root node pointer, with remainder of nodes continuing after
 * @param geoms  Geometry array to which BVH nodes point
 */
__host__ __device__ float BVHGeomIntersectionTest(BVH::FlatNode *nodes, Geom *geoms, Ray r,
                                                  glm::vec3 &intersectionPoint, glm::vec3 &normal,
                                                  bool &outside, int *hitGeomIndex) {
    const int MAX_STACK = 2048;
    int nodeStack[MAX_STACK];

    int stackPtr = 0;
    nodeStack[stackPtr++] = 0; // Start with root node

    float closestT = FLT_MAX;
    bool hitFound = false;

    while (stackPtr > 0) {
        int nodeIndex = nodeStack[--stackPtr];
        BVH::FlatNode &node = nodes[nodeIndex];

        // Test ray against node's bounding box
        float t = AABBRayIntersectionTest(r, node.bounds);
        if (t < 0 || t >= closestT) {
            continue; // No intersection or farther than current closest
        }

        if (node.isLeaf) {
            // Test against all primitives in this leaf
            for (size_t i = node.leafInfo.dataStart;
                 i < node.leafInfo.dataStart + node.leafInfo.dataCount; ++i) {
                glm::vec3 tempPoint, tempNormal;
                bool tempOutside;
                float tempT = -1;
                Geom &geom = geoms[i];

                tempT = pickGeometryIntersectionTest(geom, r, tempPoint, tempNormal, tempOutside);

                if (tempT > 0 && tempT < closestT) {
                    closestT = tempT;
                    intersectionPoint = tempPoint;
                    normal = tempNormal;
                    outside = tempOutside;
                    *hitGeomIndex = i;
                    hitFound = true;
                }
            }
        } else {
            // Internal node - add children to stack and continue on our merry way
            if (stackPtr + 2 <= MAX_STACK) {
                nodeStack[stackPtr++] = node.childOffsets[0];
                nodeStack[stackPtr++] = node.childOffsets[1];
            }
        }
    }

    return hitFound ? closestT : -1;
}

__host__ __device__ AABB getBoxBounds(Geom box) {
    // Get the 8 corners of the unit cube
    glm::vec3 corners[8] = {glm::vec3(-0.5f, -0.5f, -0.5f), glm::vec3(+0.5f, -0.5f, -0.5f),
                            glm::vec3(-0.5f, +0.5f, -0.5f), glm::vec3(+0.5f, +0.5f, -0.5f),
                            glm::vec3(-0.5f, -0.5f, +0.5f), glm::vec3(+0.5f, -0.5f, +0.5f),
                            glm::vec3(-0.5f, +0.5f, +0.5f), glm::vec3(+0.5f, +0.5f, +0.5f)};

    // Transform all corners to world space
    AABB bounds;
    bounds.min = glm::vec3(std::numeric_limits<float>::max());
    bounds.max = glm::vec3(-std::numeric_limits<float>::max());

    for (int i = 0; i < 8; ++i) {
        glm::vec3 worldCorner = multiplyMV(box.transform, glm::vec4(corners[i], 1.0f));
        bounds.min = glm::min(bounds.min, worldCorner);
        bounds.max = glm::max(bounds.max, worldCorner);
    }

    return bounds;
}

__host__ __device__ AABB getMeshBounds(const Geom &meshGeom, const Mesh &mesh) {
    // Get the 8 corners of the mesh bounds
    glm::vec3 corners[8];
    int i = 0;
    glm::vec3 mmin = mesh.bounds.min;
    glm::vec3 mmax = mesh.bounds.max;
    for (int x = 0; x < 2; x += 1) {
        int xv = (x ? mmin : mmax).x;
        for (int y = 0; y < 2; y += 1) {
            int yv = (y ? mmin : mmax).y;
            for (int z = 0; z < 2; z += 1) {
                int zv = (z ? mmin : mmax).z;
                corners[i++] = glm::vec3(xv, yv, zv);
            }
        }
    }

    // Transform all corners to world space
    AABB bounds;
    bounds.min = glm::vec3(std::numeric_limits<float>::max());
    bounds.max = glm::vec3(-std::numeric_limits<float>::max());

    for (int i = 0; i < 8; ++i) {
        glm::vec3 worldCorner = multiplyMV(meshGeom.transform, glm::vec4(corners[i], 1.0f));
        bounds.min = glm::min(bounds.min, worldCorner);
        bounds.max = glm::max(bounds.max, worldCorner);
    }

    return bounds;
}

__host__ __device__ AABB getPickGeomBounds(Geom geom, Mesh *mesh) {
    if (geom.type == SPHERE)
        return getBoxBounds(geom);
    if (geom.type == CUBE)
        return getBoxBounds(geom);
    else if (geom.type == MESH)
        return getMeshBounds(geom, *mesh);
    else
        return AABB{glm::vec3(0), glm::vec3(0)};
}