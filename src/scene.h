#pragma once

#include "sceneStructs.h"
#include <vector>

void importObj(const std::string &filename, Mesh &mesh);

class Scene
{
private:
  void loadFromJSON(const std::string &jsonName);

public:
  Scene(std::string filename);

  std::vector<Geom> geoms;
  std::vector<Material> materials;
  std::vector<Mesh> meshes;
  RenderState state;
};
