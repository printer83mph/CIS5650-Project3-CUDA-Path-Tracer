#pragma once

#include "scene.h"
#include "utilities.h"

namespace PathTrace {

enum RenderMode {
    DEFAULT = 0,
    AMBIENT_OCCLUSION = 1,
    NORMAL = 2,
    DEPTH = 3,
};

struct Options {
    // View
    RenderMode renderMode;
    float depthPassMaxDistance;

    // Optimizations
    bool contiguousMaterials;
    bool renderKernelPerMaterial;
    int minPathCountForSorting;
};
Options defaultOptions();

} // namespace PathTrace

void InitDataContainer(GuiDataContainer* guiData);
void pathtraceInit(Scene *scene);
void pathtraceFree();
void pathtrace(uchar4 *pbo, int frame, int iteration, const PathTrace::Options &options);
