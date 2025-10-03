#pragma once

#include "bvh.h"
#include "sceneStructs.h"
#include <vector>

class Scene
{
private:
  void loadFromJSON(const std::string &jsonName);
  void buildBVH();

public:
  Scene(std::string filename);

  std::vector<Geom> geoms;
  std::vector<BVH::FlatNode> bvhNodes;
  std::vector<Material> materials;
  std::vector<Mesh> meshes;
  RenderState state;
};

void importObj(const std::string &filename, Mesh &mesh);
