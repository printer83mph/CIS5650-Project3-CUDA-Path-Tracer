#include "bvh.h"
#include "glm/detail/type_vec.hpp"
#include "glm/gtc/constants.hpp"
#include "pathtrace.h"

#include <cstdio>
#include <cuda.h>
#include <thrust/execution_policy.h>
#include <thrust/partition.h>
#include <thrust/random.h>
#include <vector>

#include "interactions.h"
#include "intersections.h"
#include "scene.h"
#include "sceneStructs.h"
#include "utilities.h"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#define ERRORCHECK 1

#define FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define checkCUDAError(msg) checkCUDAErrorFn(msg, FILENAME, __LINE__)
void checkCUDAErrorFn(const char* msg, const char* file, int line)
{
#if ERRORCHECK
    cudaDeviceSynchronize();
    cudaError_t err = cudaGetLastError();
    if (cudaSuccess == err)
    {
        return;
    }

    fprintf(stderr, "CUDA error");
    if (file)
    {
        fprintf(stderr, " (%s:%d)", file, line);
    }
    fprintf(stderr, ": %s: %s\n", msg, cudaGetErrorString(err));
#ifdef _WIN32
    getchar();
#endif // _WIN32
    exit(EXIT_FAILURE);
#endif // ERRORCHECK
}

__host__ __device__
thrust::default_random_engine makeSeededRandomEngine(int iter, int index, int depth)
{
    int h = utilhash((1 << 31) | (depth << 22) | iter) ^ utilhash(index);
    return thrust::default_random_engine(h);
}

//Kernel that writes the image to the OpenGL PBO directly.
__global__ void sendImageToPBO(uchar4* pbo, glm::ivec2 resolution, int iter, glm::vec3* image)
{
    int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    int y = (blockIdx.y * blockDim.y) + threadIdx.y;

    if (x < resolution.x && y < resolution.y)
    {
        int index = x + (y * resolution.x);
        glm::vec3 pix = image[index];

        glm::ivec3 color;
        color.x = glm::clamp((int)(pix.x / iter * 255.0), 0, 255);
        color.y = glm::clamp((int)(pix.y / iter * 255.0), 0, 255);
        color.z = glm::clamp((int)(pix.z / iter * 255.0), 0, 255);

        // Each thread writes one pixel location in the texture (textel)
        pbo[index].w = 0;
        pbo[index].x = color.x;
        pbo[index].y = color.y;
        pbo[index].z = color.z;
    }
}

static Scene* hst_scene = NULL;
static GuiDataContainer* guiData = NULL;
static glm::vec3* dev_image = NULL;
static Geom* dev_geoms = NULL;
static BVH::FlatNode *dev_BVHTree = NULL;
static Triangle *dev_meshTriangles = NULL;
static GpuMesh *dev_meshes = NULL;
static Material* dev_materials = NULL;
static PathSegment* dev_paths = NULL;
static ShadeableIntersection* dev_intersections = NULL;
static int *dev_intersectionMaterialStartIndices;
// $DEV_MEMORY: static variables for device memory, any extra info you need, etc

void InitDataContainer(GuiDataContainer* imGuiData)
{
    guiData = imGuiData;
}

void pathtraceInit(Scene* scene)
{
    hst_scene = scene;

    const Camera& cam = hst_scene->state.camera;
    const int pixelcount = cam.resolution.x * cam.resolution.y;

    cudaMalloc(&dev_image, pixelcount * sizeof(glm::vec3));
    cudaMemset(dev_image, 0, pixelcount * sizeof(glm::vec3));

    cudaMalloc(&dev_paths, pixelcount * sizeof(PathSegment));

    cudaMalloc(&dev_geoms, scene->geoms.size() * sizeof(Geom));
    cudaMemcpy(dev_geoms, scene->geoms.data(), scene->geoms.size() * sizeof(Geom), cudaMemcpyHostToDevice);

    // TODO: maybe add BVH flag
    cudaMalloc(&dev_BVHTree, scene->bvhNodes.size() * sizeof(BVH::FlatNode));
    cudaMemcpy(dev_BVHTree, scene->bvhNodes.data(), scene->bvhNodes.size() * sizeof(BVH::FlatNode),
               cudaMemcpyHostToDevice);

    // std::cout << "loaded " << scene->bvhNodes.size() << " bvh nodes" << std::endl;

    std::vector<Triangle> tris;
    std::vector<GpuMesh> meshes;
    // printf("hello!");
    for (Mesh &mesh : scene->meshes) {
        // Insert all ye mesh data into device array
        meshes.push_back(GpuMesh{mesh.bounds, tris.size(), mesh.triangles.size()});
        // Insert all triangles into device array
        tris.reserve(tris.size() + mesh.triangles.size());
        tris.insert(tris.end(), mesh.triangles.begin(), mesh.triangles.end());
    }

    cudaMalloc(&dev_meshTriangles, tris.size() * sizeof(Triangle));
    cudaMemcpy(dev_meshTriangles, tris.data(), tris.size() * sizeof(Triangle),
               cudaMemcpyHostToDevice);

    cudaMalloc(&dev_meshes, meshes.size() * sizeof(GpuMesh));
    cudaMemcpy(dev_meshes, meshes.data(), meshes.size() * sizeof(GpuMesh), cudaMemcpyHostToDevice);

    cudaMalloc(&dev_materials, scene->materials.size() * sizeof(Material));
    cudaMemcpy(dev_materials, scene->materials.data(), scene->materials.size() * sizeof(Material), cudaMemcpyHostToDevice);

    cudaMalloc(&dev_intersections, pixelcount * sizeof(ShadeableIntersection));
    cudaMemset(dev_intersections, 0, pixelcount * sizeof(ShadeableIntersection));

    cudaMalloc(&dev_intersectionMaterialStartIndices, scene->materials.size() * sizeof(int));

    // $DEV_MEMORY: initialize any extra device memeory you need

    checkCUDAError("pathtraceInit");
}

void pathtraceFree()
{
    cudaFree(dev_image);  // no-op if dev_image is null
    cudaFree(dev_paths);
    cudaFree(dev_geoms);
    cudaFree(dev_BVHTree);
    cudaFree(dev_meshTriangles);
    cudaFree(dev_meshes);
    cudaFree(dev_materials);
    cudaFree(dev_intersections);
    cudaFree(dev_intersectionMaterialStartIndices);
    // $DEV_MEMORY: clean up any extra device memory you created

    checkCUDAError("pathtraceFree");
}

/**
* Generate PathSegments with rays from the camera through the screen into the
* scene, which is the first bounce of rays.
*
* Antialiasing - add rays for sub-pixel sampling
* motion blur - jitter rays "in time"
* lens effect - jitter ray origin positions based on a lens
*/
__global__ void generateRayFromCamera(Camera cam, int iter, int traceDepth, PathSegment* pathSegments)
{
    int x = (blockIdx.x * blockDim.x) + threadIdx.x;
    int y = (blockIdx.y * blockDim.y) + threadIdx.y;

    if (x < cam.resolution.x && y < cam.resolution.y) {
        int index = x + (y * cam.resolution.x);
        PathSegment& segment = pathSegments[index];

        segment.ray.origin = cam.position;
        segment.color = glm::vec3(1.0f, 1.0f, 1.0f);

        thrust::default_random_engine rng =
            makeSeededRandomEngine(iter, y * cam.resolution.x + x, 0);
        thrust::uniform_real_distribution<float> u01(0.f, 1.f);

        segment.ray.direction = glm::normalize(
            cam.view -
            cam.right * cam.pixelLength.x *
                ((float)x - (float)cam.resolution.x * 0.5f + u01(rng)) -
            cam.up * cam.pixelLength.y *
                ((float)y - (float)cam.resolution.y * 0.5f + u01(rng)));

        float forwardComponent = glm::dot(segment.ray.direction, cam.view);
        glm::vec3 focalPoint =
            segment.ray.origin + segment.ray.direction * cam.focalDistance / forwardComponent;

        float angle = u01(rng) * glm::two_pi<float>();
        float radius = sqrt(u01(rng));
        glm::vec3 randomAperturePoint =
            cam.position +
            (cam.up * sin(angle) + cam.right * cos(angle)) * radius * cam.apertureRadius;

        segment.ray.origin = randomAperturePoint;
        segment.ray.direction = glm::normalize(focalPoint - randomAperturePoint);

        segment.pixelIndex = index;
        segment.remainingBounces = traceDepth;
    }
}

__global__ void computeIntersections(int depth, int num_paths, PathSegment *pathSegments,
                                     Geom *geoms, BVH::FlatNode *bvhTree, Triangle *meshTriangles,
                                     GpuMesh *meshes, int geoms_size,
                                     ShadeableIntersection *intersections) {
    int path_index = blockIdx.x * blockDim.x + threadIdx.x;

    if (path_index < num_paths)
    {
        PathSegment pathSegment = pathSegments[path_index];

        float t;
        glm::vec3 intersect_point;
        glm::vec3 normal;
        float t_min = FLT_MAX;
        int hit_geom_index = -1;
        bool outside = true;

        glm::vec3 tmp_intersect;
        glm::vec3 tmp_normal;

        // Run fancy BVH intersection test
        t_min = BVHGeomIntersectionTest(bvhTree, geoms, meshTriangles, meshes, pathSegment.ray,
                                        intersect_point, normal, outside, &hit_geom_index);

#if 0 // old naive parse through global geoms
        for (int i = 0; i < geoms_size; i++)
        {
            Geom& geom = geoms[i];

            t = pickGeometryIntersectionTest(geom, pathSegment.ray, tmp_intersect, tmp_normal,
                                             outside);

            // Compute the minimum t from the intersection tests to determine what
            // scene geometry object was hit first.
            if (t > 0.0f && t_min > t)
            {
                t_min = t;
                hit_geom_index = i;
                intersect_point = tmp_intersect;
                normal = tmp_normal;
            }
        }
#endif

        if (hit_geom_index == -1) {
            intersections[path_index].t = -1.0f;
        } else {
            // The ray hits something
            intersections[path_index].t = t_min;
            intersections[path_index].materialId = geoms[hit_geom_index].materialid;
            intersections[path_index].surfaceNormal = normal;
        }
    }
}

__device__ inline float cosineHemispherePdf(float cosTheta) {
    return cosTheta * glm::one_over_pi<float>();
}

__global__ void shadeIdealDiffuseMaterial(int iter, int num_paths,
                                          ShadeableIntersection *shadeableIntersections,
                                          PathSegment *pathSegments, Material *materials) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx > num_paths)
        return;

    ShadeableIntersection intersection = shadeableIntersections[idx];
    if (intersection.t > 0.0f) // if the intersection exists...
    {
        // Set up the RNG
        thrust::default_random_engine rng = makeSeededRandomEngine(iter, idx, 0);
        thrust::uniform_real_distribution<float> u01(0, 1);

        Material material = materials[intersection.materialId];
        glm::vec3 materialColor = material.color;

        if (material.emittance > 0.0f) {
            // If the material indicates that the object was a light, "light" the ray
            pathSegments[idx].color *= (materialColor * material.emittance);
            pathSegments[idx].remainingBounces = 0;
        } else {
            // If our ray hit something, it should bounce off!
            PathSegment &segment = pathSegments[idx];

            // Check if this is the last bounce
            // TODO: there must be a better way of going about this with less divergence
            if (segment.remainingBounces <= 1) {
                // we failed, men: no contribution
                segment.color = glm::vec3(0.0f);
                segment.remainingBounces = 0;
            } else {
                // Apply color from material etc
                glm::vec3 brdf = materialColor * glm::one_over_pi<float>();
                segment.color *= brdf * glm::pi<float>();

                // Set up new ray babey
                scatterRay(segment,
                           segment.ray.origin + segment.ray.direction * (intersection.t - 0.001f),
                           intersection.surfaceNormal, material, rng);

                segment.remainingBounces--;
            }
        }
    } else {
        // We have reached the uncaring void

        // If there was no intersection, color the ray black.
        // Lots of renderers use 4 channel color, RGBA, where A = alpha, often
        // used for opacity, in which case they can indicate "no opacity".
        // This can be useful for post-processing and image compositing.
        pathSegments[idx].color = glm::vec3(0.0f);
        pathSegments[idx].remainingBounces = 0;
    }
}

__global__ void shadeAmbientOcclusionPass(int iter, int num_paths,
                                          ShadeableIntersection *shadeableIntersections,
                                          PathSegment *pathSegments, Material *materials,
                                          int bounceDepth) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx > num_paths)
        return;

    ShadeableIntersection intersection = shadeableIntersections[idx];
    if (intersection.t > 0.0f) // if the intersection exists...
    {
        // Set up the RNG
        thrust::default_random_engine rng = makeSeededRandomEngine(iter, idx, 0);
        thrust::uniform_real_distribution<float> u01(0, 1);

        Material material = materials[intersection.materialId];
        glm::vec3 materialColor = material.color;

        // If our ray hit something, it should bounce off!
        PathSegment &segment = pathSegments[idx];

        // Check if this is the last bounce
        // TODO: there must be a better way of going about this with less divergence
        if (segment.remainingBounces <= 1) {
            // we failed, men: no contribution
            segment.color = glm::vec3(0.0f);
            segment.remainingBounces = 0;
        } else {
            // Apply plain white color
            glm::vec3 brdf = glm::vec3(1.f) * glm::one_over_pi<float>();
            segment.color *= brdf * glm::pi<float>();

            // Set up new ray babey
            glm::vec3 intersectionPoint =
                segment.ray.origin + segment.ray.direction * intersection.t;
            glm::vec3 offsetPoint = intersectionPoint + intersection.surfaceNormal * 0.01f;

            scatterRay(segment,
                       segment.ray.origin + segment.ray.direction * (intersection.t - 0.001f),
                       intersection.surfaceNormal, material, rng);
            segment.remainingBounces--;
        }

    } else {
        // We have reached the uncaring void

        // multiplier should be 0 when on first ray, otherwise 1
        float multiplier = min(abs((float)bounceDepth - pathSegments[idx].remainingBounces), 1.f);
        pathSegments[idx].color *= multiplier;
        // if (bounceDepth == pathSegments[idx].remainingBounces) {
        //     pathSegments[idx].color = glm::zero<glm::vec3>();
        // }

        // If there was no intersection, color the ray black.
        // Lots of renderers use 4 channel color, RGBA, where A = alpha, often
        // used for opacity, in which case they can indicate "no opacity".
        // This can be useful for post-processing and image compositing.
        pathSegments[idx].remainingBounces = 0;
    }
}

__global__ void shadeNormalPass(int iter, int num_paths,
                                ShadeableIntersection *shadeableIntersections,
                                PathSegment *pathSegments, Material *materials) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx > num_paths)
        return;

    ShadeableIntersection intersection = shadeableIntersections[idx];
    if (intersection.t > 0.0f) // if the intersection exists...
    {
        pathSegments[idx].color = (intersection.surfaceNormal + glm::vec3(1.f, 1.f, 1.f)) * 0.5f;
        pathSegments[idx].remainingBounces = 0;
    }

    else {
        pathSegments[idx].color = glm::vec3(0.0f);
        pathSegments[idx].remainingBounces = 0;
    }
}

__global__ void shadeDepthPass(int iter, int num_paths,
                               ShadeableIntersection *shadeableIntersections,
                               PathSegment *pathSegments, Material *materials, float inv_depthEnd) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx > num_paths)
        return;

    ShadeableIntersection intersection = shadeableIntersections[idx];
    if (intersection.t > 0.0f) // if the intersection exists...
    {
        pathSegments[idx].color = glm::vec3(glm::max(0.f, 1 - intersection.t * inv_depthEnd));
        pathSegments[idx].remainingBounces = 0;
    }

    else {
        pathSegments[idx].color = glm::vec3(0.0f);
        pathSegments[idx].remainingBounces = 0;
    }
}

// Add the current iteration's output to the overall image
__global__ void finalGather(int nPaths, glm::vec3* image, PathSegment* iterationPaths)
{
    int index = (blockIdx.x * blockDim.x) + threadIdx.x;

    if (index < nPaths)
    {
        PathSegment iterationPath = iterationPaths[index];
        image[iterationPath.pixelIndex] += iterationPath.color;
    }
}

struct has_no_remaining_bounces {
    __host__ __device__ bool operator()(const PathSegment &segment) {
        return segment.remainingBounces <= 0;
    }
};
struct has_remaining_bounces {
    __host__ __device__ bool operator()(const PathSegment &segment) {
        return segment.remainingBounces > 0;
    }
};

struct material_comparator {
    __host__ __device__ bool operator()(const ShadeableIntersection &a,
                                        const ShadeableIntersection &b) {
        // Sort intersections with t < 0 to the start
        if (a.t < 0 && b.t >= 0)
            return true;
        if (a.t >= 0 && b.t < 0)
            return false;

        // For intersections with the same t sign, sort by material ID
        return a.materialId < b.materialId;
    }
};

PathTrace::Options PathTrace::defaultOptions() {
    PathTrace::Options opts;

    opts.renderMode = RenderMode::DEFAULT;
    opts.depthPassMaxDistance = 30.f;

    opts.contiguousMaterials = false;
    opts.renderKernelPerMaterial = false;
    opts.minPathCountForSorting = 256;

    return opts;
}

/**
 * Set each `odata[i]` to the start index of material `i` in `idata`.
 *
 * This method assumes we have all the t < 0 rays sorted into the beginning of `idata`.
 */
__global__ void getMaterialStartIndices(int n, int *odata, const ShadeableIntersection *idata) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx == 0 || idx > n)
        return;

    const ShadeableIntersection &isect = idata[idx];

    int isNewMaterial = isect.t > 0 && isect.materialId != idata[idx - 1].materialId;
    if (isNewMaterial) {
        odata[isect.materialId] = idx;
    }
}

/**
 * Wrapper for the __global__ call that sets up the kernel calls and does a ton
 * of memory management
 */
void pathtrace(uchar4 *pbo, int frame, int iter, const PathTrace::Options &options) {
    const int traceDepth = hst_scene->state.traceDepth;
    const Camera& cam = hst_scene->state.camera;
    const int pixelcount = cam.resolution.x * cam.resolution.y;

    // 2D block for generating ray from camera
    const dim3 blockSize2d(8, 8);
    const dim3 blocksPerGrid2d(
        (cam.resolution.x + blockSize2d.x - 1) / blockSize2d.x,
        (cam.resolution.y + blockSize2d.y - 1) / blockSize2d.y);

    // 1D block for path tracing
    const int blockSize1d = 128;

    // --- Initial Ray Generation ---

    generateRayFromCamera<<<blocksPerGrid2d, blockSize2d>>>(cam, iter, traceDepth, dev_paths);
    checkCUDAError("generate camera ray");

    int depth = 0;
    PathSegment* dev_path_end = dev_paths + pixelcount;
    int num_paths = dev_path_end - dev_paths;

    // --- PathSegment Tracing Stage ---
    // Shoot ray into scene, bounce between objects, push shading chunks

    bool iterationComplete = false;
    while (!iterationComplete)
    {
        // clean shading chunks
        cudaMemset(dev_intersections, 0, pixelcount * sizeof(ShadeableIntersection));

        // tracing
        dim3 numblocksPathSegmentTracing = (num_paths + blockSize1d - 1) / blockSize1d;
        computeIntersections<<<numblocksPathSegmentTracing, blockSize1d>>>(
            depth, num_paths, dev_paths, dev_geoms, dev_BVHTree, dev_meshTriangles, dev_meshes,
            hst_scene->geoms.size(), dev_intersections);
        checkCUDAError("trace one bounce");
        cudaDeviceSynchronize();
        depth++;

        if (options.renderMode == PathTrace::RenderMode::DEFAULT) {
            bool sortedMaterials = false;

            // Only sort materials if enabled AND we have more paths
            if (options.contiguousMaterials && num_paths >= options.minPathCountForSorting) {
                // Sort dev_intersections and dev_paths by material ID
                thrust::sort_by_key(thrust::device, dev_intersections,
                                    dev_intersections + num_paths, dev_paths,
                                    material_comparator());
                sortedMaterials = true;
            }

            if (sortedMaterials && options.renderKernelPerMaterial) {
                // Find material start indices after sorting
                getMaterialStartIndices<<<numblocksPathSegmentTracing, blockSize1d>>>(
                    num_paths, dev_intersectionMaterialStartIndices, dev_intersections);
                checkCUDAError("get material start indices");
                cudaDeviceSynchronize();
                // The first [0, dev_intersectionMaterialStartIndices[0]) items of
                // dev_intersections have t < 0, meaning they didn't intersect with anything.

                // Launch shading kernel for each material contiguously
                int *hst_materialStartIndices = new int[hst_scene->materials.size()]();
                cudaMemcpy(hst_materialStartIndices, dev_intersectionMaterialStartIndices,
                           hst_scene->materials.size() * sizeof(int), cudaMemcpyDeviceToHost);

                // Process intersections with t < 0 (no intersection)
                int numNoIntersection = hst_materialStartIndices[0];
                if (numNoIntersection > 0) {
                    dim3 numBlocksNoIntersection =
                        (numNoIntersection + blockSize1d - 1) / blockSize1d;
                    shadeIdealDiffuseMaterial<<<numBlocksNoIntersection, blockSize1d>>>(
                        iter, numNoIntersection, dev_intersections, dev_paths, dev_materials);
                    checkCUDAError("shading non-intersected rays");
                }

                // Process each material type
                for (int matId = 0; matId < hst_scene->materials.size(); ++matId) {
                    int startIdx = hst_materialStartIndices[matId];
                    int endIdx = (matId == hst_scene->materials.size() - 1)
                                     ? num_paths
                                     : hst_materialStartIndices[matId + 1];
                    int numPathsForMaterial = endIdx - startIdx;

                    if (numPathsForMaterial > 0) {
                        dim3 numBlocksForMaterial =
                            (numPathsForMaterial + blockSize1d - 1) / blockSize1d;
                        shadeIdealDiffuseMaterial<<<numBlocksForMaterial, blockSize1d>>>(
                            iter, numPathsForMaterial, dev_intersections + startIdx,
                            dev_paths + startIdx, dev_materials);
                    }
                }
                checkCUDAError("shading materials by id");
                cudaDeviceSynchronize();

                delete[] hst_materialStartIndices;

            } else {
                shadeIdealDiffuseMaterial<<<numblocksPathSegmentTracing, blockSize1d>>>(
                    iter, num_paths, dev_intersections, dev_paths, dev_materials);
                checkCUDAError("shading diffuse material");
                cudaDeviceSynchronize();
            }
        } else if (options.renderMode == PathTrace::RenderMode::AMBIENT_OCCLUSION) {
            shadeAmbientOcclusionPass<<<numblocksPathSegmentTracing, blockSize1d>>>(
                iter, num_paths, dev_intersections, dev_paths, dev_materials, traceDepth);
            checkCUDAError("shading AO pass");
            cudaDeviceSynchronize();
        } else if (options.renderMode == PathTrace::RenderMode::NORMAL) {
            shadeNormalPass<<<numblocksPathSegmentTracing, blockSize1d>>>(
                iter, num_paths, dev_intersections, dev_paths, dev_materials);
            checkCUDAError("shading normal pass");
            cudaDeviceSynchronize();
        } else if (options.renderMode == PathTrace::RenderMode::DEPTH) {
            shadeDepthPass<<<numblocksPathSegmentTracing, blockSize1d>>>(
                iter, num_paths, dev_intersections, dev_paths, dev_materials,
                1.f / options.depthPassMaxDistance);
            checkCUDAError("shading depth pass");
            cudaDeviceSynchronize();
        }

        PathSegment *new_dev_paths_end = thrust::partition(
            thrust::device, dev_paths, dev_paths + num_paths, has_remaining_bounces());

        num_paths = new_dev_paths_end - dev_paths;

        if (num_paths == 0 || depth > traceDepth) {
            iterationComplete = true;
        }

        if (guiData != NULL)
        {
            guiData->TracedDepth = depth;
        }
    }

    // Assemble this iteration and apply it to the image
    dim3 numBlocksPixels = (pixelcount + blockSize1d - 1) / blockSize1d;
    finalGather<<<numBlocksPixels, blockSize1d>>>(pixelcount, dev_image, dev_paths);

    ///////////////////////////////////////////////////////////////////////////

    // Send results to OpenGL buffer for rendering
    sendImageToPBO<<<blocksPerGrid2d, blockSize2d>>>(pbo, cam.resolution, iter, dev_image);

    // Retrieve image from GPU
    cudaMemcpy(hst_scene->state.image.data(), dev_image,
        pixelcount * sizeof(glm::vec3), cudaMemcpyDeviceToHost);

    checkCUDAError("pathtrace");
}
