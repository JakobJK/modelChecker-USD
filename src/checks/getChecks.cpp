#include "getChecks.h"

std::vector<std::string> getChecks() {
    return {"trailingNumbers",
            "duplicatedNames",
            "shaders",
            "unfrozenTransforms",
            "overlappingUVBetweenMeshes",
            "selfPenetratingMesh",
            "selfPenetratingUVs",
            "uncenteredPivots",
            "parentGeometry",
            "emptyGroups",
            "triangles",
            "ngons",
            "openEdges",
            "poles",
            "hardEdges",
            "lamina",
            "zeroAreaFaces",
            "zeroLengthEdges",
            "noneManifoldEdges",
            "concave",
            "missingUVs",
            "uvRange",
            "crossBorder",
            "onBorder",
            "decimateMesh"};
}
