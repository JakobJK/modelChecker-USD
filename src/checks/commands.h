#ifndef COMMANDS_H
#define COMMANDS_H

#include "../Mesh.h"
#include <memory>
#include <nlohmann/json.hpp>
#include <pxr/usd/usd/prim.h>
#include <vector>

void callCommandByName(const std::string &name,
                       const std::vector<pxr::UsdPrim> &prims,
                       const std::vector<std::unique_ptr<Mesh>> &meshes,
                       nlohmann::json &data);

#endif
