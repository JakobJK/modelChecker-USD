#include "Mesh.h"
#include "components/Edge.h"
#include "components/Polygon.h"
#include "components/Vertex.h"
#include "pxr/usd/usdGeom/primvarsAPI.h"
#include <iostream>
#include <pxr/usd/usdGeom/mesh.h>

Mesh::Mesh(const pxr::UsdPrim &prim) : prim(prim) {
  pxr::UsdGeomMesh mesh(prim);
  if (mesh) {
    pxr::VtArray<int> faceVertexCounts;
    mesh.GetFaceVertexCountsAttr().Get(&faceVertexCounts);
    pxr::VtArray<int> faceVertexIndices;
    mesh.GetFaceVertexIndicesAttr().Get(&faceVertexIndices);
    const pxr::UsdGeomPrimvarsAPI api = pxr::UsdGeomPrimvarsAPI(mesh.GetPrim());
    const pxr::UsdGeomPrimvar uvPrimvar = api.GetPrimvar(pxr::TfToken("st"));

    pxr::VtArray<pxr::GfVec2f> uvValues;
    pxr::VtIntArray uvIndices;
    uvPrimvar.GetIndices(&uvIndices);

    int offset = 0;
    for (size_t i = 0; i < faceVertexCounts.size(); ++i) {
      int numVertsInThisFace = faceVertexCounts[i];

      std::vector<int> faceVertices(faceVertexIndices.begin() + offset,
                                    faceVertexIndices.begin() + offset +
                                        numVertsInThisFace);
      std::vector<int> faceUVs(uvIndices.begin() + offset,
                               uvIndices.begin() + offset + numVertsInThisFace);

      polygons.push_back(Polygon(faceVertices, faceUVs, *this, i));

      offset += numVertsInThisFace;
    }
    pxr::VtArray<pxr::GfVec3f> pointPositions;
    if (mesh.GetPointsAttr().Get(&pointPositions)) {
      for (const pxr::GfVec3f &point : pointPositions) {
        Vertex vertex(point[0], point[1], point[2], *this);
        vertices.push_back(vertex);
      }
    }

    if (uvPrimvar) {
      pxr::VtArray<pxr::GfVec2f> uvValues;
      uvPrimvar.Get(&uvValues);

      for (const pxr::GfVec2f &value : uvValues) {
        UV uv(value[0], value[1]);
        uvs.push_back(uv);
      }
    }

    std::set<std::pair<int, int>> edgeSet;

    for (const Polygon &polygon : polygons) {
      const std::vector<int> &faceVertices = polygon.vertexIndices;

      for (size_t i = 0; i < faceVertices.size(); ++i) {
        int vertA = faceVertices[i];
        int vertB = faceVertices[(i + 1) % faceVertices.size()];

        if (vertA > vertB) {
          std::swap(vertA, vertB);
        }

        if (edgeSet.find({vertA, vertB}) == edgeSet.end()) {
          Edge newEdge(vertA, vertB, *this);
          edges.push_back(newEdge);
          edgeSet.insert({vertA, vertB});
        }
      }
    }

    for (size_t i = 0; i < polygons.size(); i++) {
      for (int index : polygons[i].vertexIndices) {
        vertices[index].connectedPolygonsIndices.push_back(i);
      }
    }

    for (size_t i = 0; i < edges.size(); ++i) {
      int idxA = edges[i].vertIdxA;
      int idxB = edges[i].vertIdxB;
      vertices[idxA].connectedEdgesIndices.push_back(i);
      vertices[idxB].connectedEdgesIndices.push_back(i);
    }
    for (Edge &edge : edges) {
      Vertex &vertexA = vertices[edge.vertIdxA];
      Vertex &vertexB = vertices[edge.vertIdxB];

      const std::vector<int> &polygonsA = vertexA.connectedPolygonsIndices;
      const std::vector<int> &polygonsB = vertexB.connectedPolygonsIndices;

      std::set<int> uniquePolygons;
      for (int polygonIndex : polygonsA) {
        uniquePolygons.insert(polygonIndex);
      }

      for (int polygonIndex : polygonsB) {
        if (uniquePolygons.find(polygonIndex) != uniquePolygons.end()) {
          edge.connectedPolygonsIndices.push_back(polygonIndex);
        }
      }
    }
  }
}

int Mesh::getPolygonCount() const { return polygons.size(); }
int Mesh::getEdgeCount() const { return edges.size(); }
int Mesh::getVertexCount() const { return vertices.size(); }
int Mesh::getUVCount() const { return uvs.size(); }

const std::vector<Vertex> &Mesh::getVertices() const { return vertices; }
const std::vector<Edge> &Mesh::getEdges() const { return edges; }
const std::vector<Polygon> &Mesh::getPolygons() const { return polygons; }
const std::vector<UV> &Mesh::getUVs() const { return uvs; }
const std::string &Mesh::getName() const { return prim.GetName().GetString(); }
const pxr::UsdPrim &Mesh::getPrim() const { return prim; }
