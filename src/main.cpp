#include "Mesh.h"
#include "checks/commands.h"
#include "checks/getChecks.h"
#include "pxr/usd/usd/prim.h"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <nlohmann/json.hpp>
#include <pxr/base/tf/token.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <stack>
#include <vector>
#include "Formatter.h"

using json = nlohmann::json;
namespace fs = std::filesystem;

pxr::UsdPrimSiblingRange getRootNodes(pxr::UsdStageRefPtr stage) {
    pxr::UsdPrim rootPrim = stage->GetPseudoRoot();
    return rootPrim.GetChildren();
}

struct ContextData {
    std::string filepath;
    std::string rootNode;
    int polygonCount;
    int vertexCount;
    int edgeCount;
    int triangleCount;
    json to_json() const {
        return json{{"filepath", filepath},       {"rootNode", rootNode},           {"polygonCount", polygonCount},
                    {"vertexCount", vertexCount}, {"triangleCount", triangleCount}, {"edgeCount", edgeCount}};
    }
};

json initializeContext(const std::vector<std::unique_ptr<Mesh>> &meshes, std::string rootNodeName,
                       std::string filepath) {
    int polyCount = 0;
    int vertexCount = 0;
    int edgeCount = 0;
    int triangleCount = 0;

    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        polyCount += mesh_ptr->getPolygonCount();
        edgeCount += mesh_ptr->getEdgeCount();
        vertexCount += mesh_ptr->getVertexCount();
        auto polygons = mesh_ptr->getPolygons();
        for (const Polygon &polygon : polygons) {
            triangleCount += polygon.getTriangleCount();
        }
    }

    ContextData context{filepath, rootNodeName, polyCount, vertexCount, edgeCount, triangleCount};

    return context.to_json();
};

json runContext(pxr::UsdPrim rootNode, std::string filepath) {
    std::stack<pxr::UsdPrim> primStack;
    std::vector<pxr::UsdPrim> prims;

    primStack.push(rootNode);
    std::vector<std::unique_ptr<Mesh>> meshes;

    while (!primStack.empty()) {
        pxr::UsdPrim currentPrim = primStack.top();
        primStack.pop();

        if (currentPrim.IsA<pxr::UsdGeomGprim>()) {
            auto currentMesh = std::make_unique<Mesh>(currentPrim);
            meshes.push_back(std::move(currentMesh));
        }

        prims.push_back(currentPrim);

        for (const pxr::UsdPrim &child : currentPrim.GetChildren()) {
            primStack.push(child);
        }
    }

    json data = initializeContext(meshes, rootNode.GetName().GetString(), filepath);
    json errors;

    std::vector<std::string> checks = getChecks();
    for (const std::string &check : checks) {
        callCommandByName(check, prims, meshes, errors);
    }
    data["errors"] = errors;
    return data;
}

pxr::UsdStageRefPtr getStage(std::string filepath) { return pxr::UsdStage::Open(filepath); }

void writeJson(nlohmann::json data, std::string filepath) {
    fs::path inputPath = filepath;
    fs::path outputPath = inputPath.parent_path() / (inputPath.stem().string() + ".json");

    std::ofstream outputFile(outputPath);
    if (!outputFile.is_open()) {
        std::cerr << "Failed top open output file for writing." << std::endl;
        return;
    }
    outputFile << data.dump(4);
    outputFile.close();
};

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_usd_file>" << std::endl;
        return -1;
    }

    auto start = std::chrono::high_resolution_clock::now();
    std::string filepath = argv[1];

    fs::path absolutePath = fs::absolute(filepath);
    pxr::UsdStageRefPtr stage = getStage(absolutePath);
    std::vector<json> results;
    if (!stage) {
        std::cerr << "Failed to open USD file." << std::endl;
        return -1;
    }

    pxr::UsdPrimSiblingRange rootNodes = getRootNodes(stage);
    std::string absFilepath = absolutePath.string();

    for (auto const &rootNode : rootNodes) {
        auto context = runContext(rootNode, absFilepath);
        results.push_back(context);
    }

    format(results);
    auto stop = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::duration<double>>(stop - start);

    writeJson(results, filepath);
    std::cout << "Time taken by function: " << duration.count() << " seconds" << std::endl;

    return 0;
}
