#ifndef OVERLAPPING_MESHES_H
#define OVERLAPPING_MESHES_H

#include "../Mesh.h"
#include <memory>

void overlappingMeshes(const std::vector<std::unique_ptr<Mesh>> &meshes);

#endif
