#include "commands.h"
#include "../Mesh.h"
#include "../checks/overlappingMeshes.h"
#include "../components/Vertex.h"
#include "../rasterizer/Buffer.h"
#include "pxr/usd/usd/prim.h"
#include "pxr/usd/usd/stage.h"
#include "pxr/usd/usdGeom/mesh.h"
#include "pxr/usd/usdGeom/subset.h"
#include "pxr/usd/usdGeom/xformable.h"
#include "pxr/usd/usdShade/materialBindingAPI.h"
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_simplification/edge_collapse.h>
#include <CGAL/Surface_mesh_simplification/Policies/Edge_collapse/Edge_count_stop_predicate.h>
#include <CGAL/IO/OBJ.h>
#include <cstddef>
#include <glm/glm.hpp>
#include <iostream>
#include <iterator>
#include <memory>
#include <nlohmann/json.hpp>
#include <pxr/usd/usdGeom/gprim.h>
#include <unordered_map>
#include <vector>


typedef CGAL::Simple_cartesian<double> Kernel;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_3 Point_3;
typedef CGAL::Surface_mesh<Point_3> cgal_mesh;
namespace PMP = CGAL::Polygon_mesh_processing;
namespace SMS = CGAL::Surface_mesh_simplification;

using json = nlohmann::json;


struct Vec3Hash {
    std::size_t operator()(const glm::vec3& v) const {
        std::hash<float> hasher;
        return hasher(v.x) ^ hasher(v.y) ^ hasher(v.z);
    }
};

void trailingNumbers(const std::vector<pxr::UsdPrim> &prims,
                     const std::vector<std::unique_ptr<Mesh>> &meshes,
                     json &data) {
    std::vector<std::string> trailingNumbers;
    for (const pxr::UsdPrim &prim : prims) {
        std::string currentPrimName = prim.GetName().GetString();
        if (!currentPrimName.empty() && isdigit(currentPrimName.back())) {
            trailingNumbers.push_back(currentPrimName);
        }
    }
    data["trailingNumbers"] = trailingNumbers;
}

void duplicatedNames(const std::vector<pxr::UsdPrim> &prims,
                     const std::vector<std::unique_ptr<Mesh>> &meshes,
                     json &data) {
    std::vector<std::string> duplicatedNames;
    std::unordered_map<std::string, std::vector<std::string>> nameToPaths;

    for (const pxr::UsdPrim &prim : prims) {
        std::string name = prim.GetName().GetString();
        std::string fullPath = prim.GetPath().GetString();
        nameToPaths[name].push_back(fullPath);
    }

    for (const auto &entry : nameToPaths) {
        if (entry.second.size() > 1) {
            for (const std::string &path : entry.second) {
                duplicatedNames.push_back(path);
            }
        }
    }
    data["duplicatedNames"] = duplicatedNames;
}

void shaders(const std::vector<pxr::UsdPrim> &prims,
             const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<std::string> shaders;
    for (const auto &mesh : meshes) {
        auto prim = mesh->getPrim();
        pxr::UsdShadeMaterialBindingAPI bindingAPI(prim);
        pxr::UsdShadeMaterial material = bindingAPI.ComputeBoundMaterial();
        if (!material) {
            continue;
        }
        pxr::UsdPrim materialPrim = material.GetPrim();
        std::string materialName = materialPrim.GetName().GetString();
        if (materialName != "initialShadingGroup") {
            shaders.push_back(mesh->getName());
        }
    }
    data["shaders"] = shaders;
}

void unfrozenTransforms(const std::vector<pxr::UsdPrim> &prims,
                        const std::vector<std::unique_ptr<Mesh>> &meshes,
                        json &data) {
    std::vector<std::string> unfrozenTransforms;
    for (const pxr::UsdPrim &prim : prims) {
        if (pxr::UsdGeomXformable xform = pxr::UsdGeomXformable(prim)) {
            pxr::GfMatrix4d localTransform;
            bool resetsXformStack;
            if (xform.GetLocalTransformation(&localTransform, &resetsXformStack,
                                             pxr::UsdTimeCode::Default())) {
                pxr::GfVec3d translation = localTransform.ExtractTranslation();
                if (translation != pxr::GfVec3d(0, 0, 0)) {
                    unfrozenTransforms.push_back(prim.GetPath().GetString());
                }
            }
        }
    }
    data["unfrozenTransforms"] = unfrozenTransforms;
}

void geomSubsets(const std::vector<pxr::UsdPrim> &prims,
                 const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::unordered_map<std::string, std::vector<std::string>> geomSubsets;

    for (const auto &mesh_ptr : meshes) {
        const auto children = mesh_ptr->getPrim().GetChildren();
        std::string meshPath = mesh_ptr->getPrim().GetPath().GetString();
        for (const auto &child : children) {
            if (child.IsA<pxr::UsdGeomSubset>()) {
                if (geomSubsets.find(meshPath) == geomSubsets.end()) {
                    geomSubsets[meshPath] = std::vector<std::string>();
                }
                geomSubsets[meshPath].push_back(child.GetName().GetString());
            }
        }
    }
    data["geomSubsets"] = geomSubsets;
}

void overlappingUVBetweenMeshes(
    const std::vector<pxr::UsdPrim> &prims,
    const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::unordered_map<std::string, std::vector<int>>
        overlappingUVBetweenMeshes;
    overlappingMeshes(meshes);
    data["overlappingUVBetweenMeshes"] = overlappingUVBetweenMeshes;
}

void selfPenetratingMesh(const std::vector<pxr::UsdPrim> &prims,
                         const std::vector<std::unique_ptr<Mesh>> &meshes,
                         json &data) {
    std::unordered_map<std::string, std::set<int>> selfPenetratingMesh;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        cgal_mesh c_mesh;
        const auto &all_vertices = mesh_ptr->getVertices();
        const auto &all_polygons = mesh_ptr->getPolygons();

        std::unordered_map<cgal_mesh::Face_index, const Polygon *>
            face_to_polygon_map;

        for (const auto &vertex : all_vertices) {
            const auto pos = vertex.getPosition();
            auto vertex_descriptor =
                c_mesh.add_vertex(Point_3(pos.x, pos.y, pos.z));
        }

        for (const auto &polygon : all_polygons) {
            const auto triangles_uint = polygon.getTriangles(true);

            std::vector<int> triangles(triangles_uint.begin(),
                                       triangles_uint.end());
            for (size_t i = 0; i < triangles.size(); i += 3) {
                auto const a = polygon.getGlobalVertexId(triangles[i]);
                auto const b = polygon.getGlobalVertexId(triangles[i + 1]);
                auto const c = polygon.getGlobalVertexId(triangles[i + 2]);

                std::vector<cgal_mesh::Vertex_index> face_vertices = {
                    cgal_mesh::Vertex_index(a), cgal_mesh::Vertex_index(b),
                    cgal_mesh::Vertex_index(c)};
                auto face_descriptor = c_mesh.add_face(face_vertices);
                face_to_polygon_map[face_descriptor] = &polygon;
            }
        }
        std::vector<std::pair<cgal_mesh::Face_index, cgal_mesh::Face_index>>
            intersected_faces;
        if (CGAL::is_valid(c_mesh)) {
            bool has_self_intersections = PMP::does_self_intersect(
                c_mesh, PMP::parameters::vertex_point_map(
                            get(CGAL::vertex_point, c_mesh)));
            if (has_self_intersections) {
                const auto meshPath = mesh_ptr->getPrim().GetPath().GetString();
                selfPenetratingMesh[meshPath] = {};
                PMP::self_intersections(
                    c_mesh, std::back_inserter(intersected_faces),
                    PMP::parameters::face_index_map(
                        get(CGAL::face_index, c_mesh))
                        .vertex_point_map(get(CGAL::vertex_point, c_mesh)));

                for (const auto &intersection : intersected_faces) {
                    auto p1 = face_to_polygon_map[intersection.first];
                    auto p2 = face_to_polygon_map[intersection.second];
                    selfPenetratingMesh[meshPath].insert(p1->getId());
                    selfPenetratingMesh[meshPath].insert(p2->getId());
                }
            }
        }
    }
    data["selfPenetratingMesh"] = selfPenetratingMesh;
}

void parentGeometry(const std::vector<pxr::UsdPrim> &prims,
                    const std::vector<std::unique_ptr<Mesh>> &meshes,
                    json &data) {
    std::vector<std::string> parentGeometry;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        pxr::UsdPrim curPrim = mesh_ptr->getPrim();
        const auto children = curPrim.GetChildren();
        if (!children.empty()) {
            for (auto const &child : children) {
                if (pxr::UsdGeomXformable(child)) {
                    const auto mesh_path =
                        mesh_ptr->getPrim().GetPath().GetString();
                    parentGeometry.push_back(mesh_path);
                    break;
                }
            }
        }
    }
    data["parentGeometry"] = parentGeometry;
}

void emptyGroups(const std::vector<pxr::UsdPrim> &prims,
                 const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<std::string> emptyGroups;
    for (const pxr::UsdPrim &prim : prims) {
        if (prim.GetTypeName().GetString() == "xform") {
            auto children = prim.GetChildren();
            if (children.empty()) {
                emptyGroups.push_back(prim.GetName().GetString());
            }
        }
    }
    data["emptyGroups"] = emptyGroups;
}

void triangles(const std::vector<pxr::UsdPrim> &prims,
               const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<int> triangles;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        const std::vector<Polygon> &polygons = mesh_ptr->getPolygons();
        for (size_t i = 0; i < polygons.size(); ++i) {
            if (polygons[i].getVertexCount() == 3) {
                triangles.push_back(i);
            }
        }
        std::string meshPath = mesh_ptr->getPrim().GetPath().GetText();
    }
    data["triangles"] = triangles;
}

void ngons(const std::vector<pxr::UsdPrim> &prims,
           const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<int> ngons;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        const std::vector<Polygon> &polygons = mesh_ptr->getPolygons();
        for (size_t i = 0; i < polygons.size(); ++i) {
            if (polygons[i].getVertexCount() > 4) {
                ngons.push_back(i);
            }
        }
    }
    data["ngons"] = ngons;
}

void openEdges(const std::vector<pxr::UsdPrim> &prims,
               const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<int> openEdges;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        const auto edges = mesh_ptr->getEdges();
        for (size_t i = 0; i < edges.size(); ++i) {
            if (edges[i].getPolygonCount() == 1) {
                openEdges.push_back(i);
            }
        }
    }
    data["openEdges"] = openEdges;
}

void poles(const std::vector<pxr::UsdPrim> &prims,
           const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<int> poles;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        const std::vector<Vertex> &vertices = mesh_ptr->getVertices();
        for (size_t i = 0; i < vertices.size(); ++i) {
            if (vertices[i].getConnectedEdgesCount() > 5) {
                poles.push_back(i);
            }
        }
    }
    data["poles"] = poles;
}

void hardEdges(const std::vector<pxr::UsdPrim> &prims,
               const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<std::string> hardEdges;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        const auto prim = mesh_ptr->getPrim();
        const auto meshPrim = pxr::UsdGeomMesh(prim);
        pxr::TfToken subdivisionScheme;
        meshPrim.GetSubdivisionSchemeAttr().Get(&subdivisionScheme);
        std::string subdivisionSchemeStr = subdivisionScheme.GetString();
        pxr::UsdAttribute normalsAttribute = meshPrim.GetNormalsAttr();
        bool isAuthored = normalsAttribute.IsAuthored();
        if (isAuthored && subdivisionScheme == "catmullClark") {
            hardEdges.push_back(prim.GetPath().GetString());
        }
    }
    data["hardEdges"] = hardEdges;
}

void lamina(const std::vector<pxr::UsdPrim> &prims,
            const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {}

void zeroAreaFaces(const std::vector<pxr::UsdPrim> &prims,
                   const std::vector<std::unique_ptr<Mesh>> &meshes,
                   json &data) {
    std::vector<int> zeroAreaFaces;
    float tolerance = 0.000000003;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        auto polygons = mesh_ptr->getPolygons();
        for (size_t i = 0; i < polygons.size(); ++i) {
            if (polygons[i].getArea() <= tolerance) {
                zeroAreaFaces.push_back(i);
            }
        }
    }
    data["zeroAreaFaces"] = zeroAreaFaces;
}

void zeroLengthEdges(const std::vector<pxr::UsdPrim> &prims,
                     const std::vector<std::unique_ptr<Mesh>> &meshes,
                     json &data) {
    std::vector<int> zeroLengthEdges;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        float tolerance = 0.000003;
        const std::vector<Edge> edges = mesh_ptr->getEdges();
        for (size_t i = 0; i < edges.size(); ++i) {
            if (edges[i].getLength() <= tolerance) {
                zeroLengthEdges.push_back(i);
            }
        }
    }
    data["zeroLengthEdges"] = zeroLengthEdges;
}

void noneManifoldEdges(const std::vector<pxr::UsdPrim> &prims,
                       const std::vector<std::unique_ptr<Mesh>> &meshes,
                       json &data) {
    std::vector<int> noneManifoldEdges;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        const std::vector<Edge> &edges = mesh_ptr->getEdges();
        for (size_t i = 0; i < edges.size(); ++i) {
            if (edges[i].getPolygonCount() >= 3) {
                noneManifoldEdges.push_back(i);
            }
        }
    }
    data["noneManifoldEdges"] = noneManifoldEdges;
}

void concave(const std::vector<pxr::UsdPrim> &prims,
             const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<std::string> concave;
    for (const auto &mesh_ptr : meshes) {
        const auto polygons = mesh_ptr->getPolygons();
        for (const auto &polygon : polygons) {
            if (polygon.isConcave()) {
                concave.push_back("lol");
            }
        }
    }
    data["concave"] = concave;
}

void missingUVs(const std::vector<pxr::UsdPrim> &prims,
                const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<std::string> missingUVs;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        if (mesh_ptr->getUVs().empty()) {
            missingUVs.push_back(mesh_ptr->getName());
        }
    }
    data["missingUVs"] = missingUVs;
}

void uvRange(const std::vector<pxr::UsdPrim> &prims,
             const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::map<std::string, std::vector<int>> uvRange;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        std::vector<UV> uvs = mesh_ptr->getUVs();
        std::string meshname = mesh_ptr->getName();
        for (size_t i = 0; i < uvs.size(); ++i) {
            const UV &uv = uvs[i];
            std::pair<float, float> curUV = uv.getPos();
            float u = curUV.first;
            float v = curUV.second;
            if (u < 0 || u >= 10 || v < 0) {
                uvRange[meshname].push_back(i);
            }
        }
    }
    data["uvRange"] = uvRange;
}

void crossBorder(const std::vector<pxr::UsdPrim> &prims,
                 const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<int> crossBorder;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        std::vector<Polygon> polygons = mesh_ptr->getPolygons();
        for (size_t i = 0; i < polygons.size(); i++) {
            std::vector<UV> curUVs = polygons[i].getUVs();
            if (curUVs.empty())
                continue;
            std::pair<float, float> firstUV = curUVs[0].getPos();
            int uRef = int(firstUV.first);
            int vRef = int(firstUV.second);
            for (size_t j = 1; j < curUVs.size(); j++) {
                std::pair<float, float> curUV = curUVs[j].getPos();
                int curU = int(curUV.first);
                int curV = int(curUV.second);

                if (curU != uRef || curV != vRef) {
                    crossBorder.push_back(i);
                    break;
                }
            }
        }
    }
    data["crossBorder"] = crossBorder;
}

void onBorder(const std::vector<pxr::UsdPrim> &prims,
              const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    std::vector<int> onBorder;
    for (const std::unique_ptr<Mesh> &mesh_ptr : meshes) {
        std::vector<UV> UVs = mesh_ptr->getUVs();
        std::string meshName = mesh_ptr->getName();
        for (size_t i = 0; i < UVs.size(); ++i) {
        }
    }
}

void decimateMesh(const std::vector<pxr::UsdPrim> &prims, const std::vector<std::unique_ptr<Mesh>> &meshes, json &data) {
    for (const auto& mesh_ptr : meshes) {
        cgal_mesh sm;

        const auto &all_vertices = mesh_ptr->getVertices();
        const auto &all_polygons = mesh_ptr->getPolygons();

        std::vector<cgal_mesh::Vertex_index> vertex_indices;
        for (const auto &vertex : all_vertices) {
            const auto pos = vertex.getPosition();
            auto vertex_index = sm.add_vertex(Point_3(pos.x, pos.y, pos.z));
            vertex_indices.push_back(vertex_index);
        }

for (const auto &polygon : all_polygons) {
            const auto triangles = polygon.getTriangles(true);

            for (size_t i = 0; i < triangles.size(); i += 3) {
                int a = polygon.getGlobalVertexId(triangles[i]);
                int b = polygon.getGlobalVertexId(triangles[i + 1]);
                int c = polygon.getGlobalVertexId(triangles[i + 2]);

                if (a < vertex_indices.size() && b < vertex_indices.size() && c < vertex_indices.size()) {
                    std::vector<cgal_mesh::Vertex_index> face_vertices = {
                        vertex_indices[a], vertex_indices[b], vertex_indices[c]
                    };

                    auto face_descriptor = sm.add_face(face_vertices);
                    if (face_descriptor == sm.null_face()) {
                        std::cerr << "Failed to add face." << std::endl;
                    }
                } else {
                    std::cerr << "Vertex index out of bounds." << std::endl;
                }
            }
        }



        if (!CGAL::is_valid(sm)) {
            std::cerr << "Mesh is not valid after initial construction." << std::endl;
            continue;
        }

        int target_faces = sm.number_of_faces() / 10; // Reduce by 90%

        CGAL::Surface_mesh_simplification::Edge_count_stop_predicate<cgal_mesh> stop(target_faces);

        int r = SMS::edge_collapse(sm, stop);
        std::cout << "\nFinished!\n" << r << " edges removed.\n"
                  << sm.number_of_edges() << " final edges.\n";

        if (!CGAL::IO::write_OBJ(mesh_ptr->getName() + ".obj", sm)) {
            std::cerr << "Error writing OBJ file.\n";
        }
    }
}




using FunctionPointer = void (*)(const std::vector<pxr::UsdPrim> &,
                                 const std::vector<std::unique_ptr<Mesh>> &,
                                 json &data);

struct command {
    FunctionPointer func;
    std::string label;
    std::string category;
};
std::map<std::string, command> functionMap = {
//    {"trailingNumbers", {&trailingNumbers, "Trailing Numbers", "General"}},
//   {"duplicatedNames", {&duplicatedNames, "Duplicated Names", "General"}},
//    {"shaders", {&shaders, "Shaders", "General"}},
//    {"unfrozenTransforms", {&unfrozenTransforms, "Unfrozen Transforms", "General"}},
//    {"parentGeometry", {&parentGeometry, "Parent Geometry", "General"}},
//    {"emptyGroups", {&emptyGroups, "Empty Groups", "General"}},
//    {"triangles", {&triangles, "Triangles", "Topology"}},
//    {"ngons", {&ngons, "Ngons", "Topology"}}
//    {"openEdges", {&openEdges, "Open Edges", "Topology"}},
//    {"poles", {&poles, "Poles", "Topology"}},
//    {"overlappingUVBetweenMeshes",
//     {&overlappingUVBetweenMeshes, "overlappingUVBetweenMeshes", "UVs"}},
//    {"selfPenetratingMesh",   {&selfPenetratingMesh, "Self Penetrating Mesh", "Topology"}},
//    {"hardEdges", {&hardEdges, "Hard Edges", "Topology"}},
 //   {"lamina", {&lamina, "Lamina", "Topology"}},
  //  {"zeroAreaFaces", {&zeroAreaFaces, "Zero Area Faces", "Topology"}},
  //  {"zeroLengthEdges", {&zeroLengthEdges, "Zero Length Edges", "Topology"}},
   // {"noneManifoldEdges",
    // {&noneManifoldEdges, "None Manifold Edges", "Topology"}},
    //{"concave", {&concave, "Concave", "Topology"}},
//    {"missingUVs", {&missingUVs, "Missing UVs", "UVs"}},
//    {"uvRange", {&uvRange, "UV Range", "UVs"}},
//    {"crossBorder", {&crossBorder, "Cross Border", "UVs"}},
//    {"onBorder", {&onBorder, "On Border", "UVs"}},
//    {"geomSubsets", {&geomSubsets, "Geometry Subsets", "General"}}
        {"decimateMesh", {&decimateMesh, "Decimate Meshes", "Topology"}}
};

void callCommandByName(const std::string &funcName,
                       const std::vector<pxr::UsdPrim> &prims,
                       const std::vector<std::unique_ptr<Mesh>> &meshes,
                       json &data) {
    auto it = functionMap.find(funcName);
    if (it != functionMap.end()) {
        it->second.func(prims, meshes, data);
    } else {
        std::cerr << "Function named " << funcName << " not found!"
                  << std::endl;
    }
}
