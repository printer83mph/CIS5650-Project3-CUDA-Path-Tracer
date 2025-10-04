#include "scene.h"

#include "bvh.h"
#include "intersections.h"
#include "sceneStructs.h"
#include "utilities.h"

#include <algorithm>
#include <glm/detail/type_vec.hpp>
#include <glm/gtc/matrix_inverse.hpp>
#include <glm/gtx/string_cast.hpp>

#include <json.hpp>
#include <tiny_obj_loader.h>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>

using namespace std;
using json = nlohmann::json;

Scene::Scene(string filename)
{
    cout << "Reading scene from " << filename << " ..." << endl;
    cout << " " << endl;
    auto ext = filename.substr(filename.find_last_of('.'));
    if (ext == ".json")
    {
        loadFromJSON(filename);
        buildBVH();
        return;
    }
    else
    {
        cout << "Couldn't read from " << filename << endl;
        exit(-1);
    }
}

void Scene::loadFromJSON(const std::string& jsonName)
{
    std::ifstream f(jsonName);
    json data = json::parse(f);

    // Used to do relative pathing for ye obj files
    std::filesystem::path jsonFolder = std::filesystem::path(jsonName).parent_path();

    const auto& materialsData = data["Materials"];
    std::unordered_map<std::string, uint32_t> MatNameToID;
    for (const auto& item : materialsData.items())
    {
        const auto& name = item.key();
        const auto& p = item.value();
        Material newMaterial{};
        // TODO: handle materials loading differently
        if (p["TYPE"] == "Diffuse")
        {
            const auto& col = p["RGB"];
            newMaterial.color = glm::vec3(col[0], col[1], col[2]);
        }
        else if (p["TYPE"] == "Emitting")
        {
            const auto& col = p["RGB"];
            newMaterial.color = glm::vec3(col[0], col[1], col[2]);
            newMaterial.emittance = p["EMITTANCE"];
        }
        else if (p["TYPE"] == "Specular")
        {
            const auto& col = p["RGB"];
            newMaterial.color = glm::vec3(col[0], col[1], col[2]);
        }
        MatNameToID[name] = materials.size();
        materials.emplace_back(newMaterial);
    }

    const auto &meshesData = data["Meshes"];
    std::unordered_map<std::string, uint32_t> MeshNameToID;
    for (const auto &item : meshesData.items()) {
        const auto &name = item.key();
        const auto &p = item.value();

        MeshNameToID[name] = meshes.size();
        this->meshes.emplace_back(Mesh{});
        importObj(jsonFolder / std::filesystem::path(p), this->meshes.back());
    }

    const auto& objectsData = data["Objects"];
    for (const auto& p : objectsData)
    {
        const auto& type = p["TYPE"];
        Geom newGeom;
        if (type == "cube") {
            newGeom.type = CUBE;
        } else if (type == "sphere") {
            newGeom.type = SPHERE;
        } else if (type == "mesh") {
            newGeom.type = MESH;
            newGeom.meshId = MeshNameToID[p["MESH"]];
        }
        newGeom.materialid = MatNameToID[p["MATERIAL"]];
        const auto& trans = p["TRANS"];
        const auto& rotat = p["ROTAT"];
        const auto& scale = p["SCALE"];
        newGeom.translation = glm::vec3(trans[0], trans[1], trans[2]);
        newGeom.rotation = glm::vec3(rotat[0], rotat[1], rotat[2]);
        newGeom.scale = glm::vec3(scale[0], scale[1], scale[2]);
        newGeom.transform = utilityCore::buildTransformationMatrix(
            newGeom.translation, newGeom.rotation, newGeom.scale);
        newGeom.inverseTransform = glm::inverse(newGeom.transform);
        newGeom.invTranspose = glm::inverseTranspose(newGeom.transform);

        this->geoms.push_back(newGeom);
    }
    const auto& cameraData = data["Camera"];
    Camera& camera = state.camera;
    RenderState& state = this->state;
    camera.resolution.x = cameraData["RES"][0];
    camera.resolution.y = cameraData["RES"][1];
    float fovy = cameraData["FOVY"];
    state.iterations = cameraData["ITERATIONS"];
    state.traceDepth = cameraData["DEPTH"];
    state.imageName = cameraData["FILE"];
    const auto& pos = cameraData["EYE"];
    const auto& lookat = cameraData["LOOKAT"];
    const auto& up = cameraData["UP"];

    // Physical camera options
    camera.apertureRadius = cameraData.contains("APERTURE_RADIUS")
                                ? cameraData["APERTURE_RADIUS"].get<float>()
                                : 0.018f;
    camera.focalDistance =
        cameraData.contains("FOCAL_DISTANCE") ? cameraData["FOCAL_DISTANCE"].get<float>() : 5.f;

    camera.position = glm::vec3(pos[0], pos[1], pos[2]);
    camera.lookAt = glm::vec3(lookat[0], lookat[1], lookat[2]);
    camera.up = glm::vec3(up[0], up[1], up[2]);

    //calculate fov based on resolution
    float yscaled = tan(fovy * (PI / 180));
    float xscaled = (yscaled * camera.resolution.x) / camera.resolution.y;
    float fovx = (atan(xscaled) * 180) / PI;
    camera.fov = glm::vec2(fovx, fovy);

    camera.right = glm::normalize(glm::cross(camera.view, camera.up));
    camera.pixelLength = glm::vec2(2 * xscaled / (float)camera.resolution.x,
        2 * yscaled / (float)camera.resolution.y);

    camera.view = glm::normalize(camera.lookAt - camera.position);

    //set up render camera stuff
    int arraylen = camera.resolution.x * camera.resolution.y;
    state.image.resize(arraylen);
    std::fill(state.image.begin(), state.image.end(), glm::vec3());
}

void Scene::buildBVH() {

    // Fill primitives vector with geometry + bounds
    std::vector<BVH::Primitive<Geom>> primitives;
    std::printf("building primitives from scene\n");
    for (auto &geom : this->geoms) {

#define ENABLE_TRI_NODES true
#if ENABLE_TRI_NODES
        if (geom.type == MESH) {
            std::printf("building mesh as triangle primitives\n");
            loadTrisAsPrimitives(geom, this->meshes[geom.meshId], &primitives);
            continue;
        }
#endif

        AABB bounds =
            getPickGeomBounds(geom, geom.type == MESH ? &this->meshes[geom.meshId] : nullptr);
        std::printf("building normal geometry with bounds min: (%.2f, %.2f, %.2f) max: (%.2f, "
                    "%.2f, %.2f)\n",
                    bounds.min.x, bounds.min.y, bounds.min.z, bounds.max.x, bounds.max.y,
                    bounds.max.z);

        primitives.push_back(BVH::Primitive<Geom>{bounds, geom});
    }

    BVH::Tree<Geom> tree = BVH::buildTree<Geom>(primitives);

    // Copy over tree data
    this->bvhNodes = tree.nodes;
    this->geoms = tree.leafData;
}

// Instead of making "mesh" nodes, we just make a shitton of "triangle" nodes
void Scene::loadTrisAsPrimitives(const Geom &geom, const Mesh &mesh,
                                 std::vector<BVH::Primitive<Geom>> *primitives) {

    primitives->reserve(primitives->size() + mesh.triangles.size());
    for (int i = 0; i < mesh.triangles.size(); ++i) {

        // if (i > 40)
        //     break;

        const Triangle &tri = mesh.triangles[i];

        Geom triGeom = Geom{};
        triGeom.type = TRIANGLE;
        triGeom.materialid = geom.materialid;
        triGeom.transform = glm::mat4(1.f);
        triGeom.invTranspose = glm::mat4(1.f);
        triGeom.inverseTransform = glm::mat4(1.f);

        // TODO: remove
        // triGeom.triData = tri;

        AABB bounds = AABB{glm::vec3(FLT_MAX), glm::vec3(FLT_MIN)};
        for (int vertIndex = 0; vertIndex < 3; ++vertIndex) {
            triGeom.triData.vertices[vertIndex] =
                multiplyMV(geom.transform, glm::vec4(tri.vertices[vertIndex], 1.f));
            triGeom.triData.normals[vertIndex] =
                multiplyMV(geom.invTranspose, glm::vec4(tri.normals[vertIndex], 0.f));

            bounds.min = glm::min(bounds.min, triGeom.triData.vertices[vertIndex]);
            bounds.max = glm::max(bounds.max, triGeom.triData.vertices[vertIndex]);
        }

        primitives->emplace_back(BVH::Primitive<Geom>{bounds, triGeom});
    }
}

void importObj(const std::string &filename, Mesh &mesh) {
    tinyobj::ObjReaderConfig reader_config;
    reader_config.triangulate = true;
    reader_config.triangulation_method = "simple";

    tinyobj::ObjReader reader;

    std::cout << "Loading OBJ file: " << filename << std::endl;
    if (!reader.ParseFromFile(filename, reader_config)) {
        if (!reader.Error().empty()) {
            std::cerr << "TinyObjReader: " << reader.Error();
        }
        exit(1);
    }

    if (!reader.Warning().empty()) {
        std::cout << "TinyObjReader: " << reader.Warning();
    }

    auto &attrib = reader.GetAttrib();
    auto &shapes = reader.GetShapes();

    if (shapes.size() > 0) {
        mesh.bounds.min = glm::vec3(FLT_MAX);
        mesh.bounds.max = glm::vec3(FLT_MIN);
    } else {
        mesh.bounds.min = glm::zero<glm::vec3>();
        mesh.bounds.max = glm::zero<glm::vec3>();
    }

    for (const auto &shape : shapes) {
        size_t index_offset = 0;

        std::cout << "loading shape with " << shape.mesh.num_face_vertices.size() << " faces"
                  << std::endl;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
            size_t fv = size_t(shape.mesh.num_face_vertices[f]);
            // std::cout << "processing face with " << fv << " vertices" << std::endl;

            auto &triangle = mesh.triangles.emplace_back(Triangle{});
            for (size_t v = 0; v < fv; ++v) {
                tinyobj::index_t idx = shape.mesh.indices[index_offset + v];
                for (int axis = 0; axis < 3; ++axis) {
                    float value = attrib.vertices[3 * size_t(idx.vertex_index) + axis];

                    triangle.vertices[v][axis] = value;

                    // Enable if we precompute mesh bounds
                    mesh.bounds.min[axis] = glm::min(mesh.bounds.min[axis], value);
                    mesh.bounds.max[axis] = glm::max(mesh.bounds.max[axis], value);
                }

                // Check if `normal_index` is zero or positive. negative = no normal data
                if (idx.normal_index >= 0) {
                    for (int axis = 0; axis < 3; ++axis)
                        triangle.normals[v][axis] =
                            attrib.normals[3 * size_t(idx.normal_index) + axis];
                }
            }

            index_offset += fv;
        }
    }
}
