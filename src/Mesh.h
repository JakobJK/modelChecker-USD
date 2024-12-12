#ifndef MESH_H
#define MESH_H

// Standard Library
#include <vector>

// Open_USD Classes
#include <pxr/usd/usdGeom/gprim.h>

// modelChecker Classes
#include "components/Edge.h"
#include "components/Polygon.h"
#include "components/UV.h"
#include "components/Vertex.h"

class Mesh {
public:
  Mesh(const pxr::UsdPrim &prim);

  int getPolygonCount() const;
  int getEdgeCount() const;
  int getVertexCount() const;
  int getUVCount() const;

  const std::vector<Vertex> &getVertices() const;
  const std::vector<Edge> &getEdges() const;
  const std::vector<Polygon> &getPolygons() const;
  const std::vector<UV> &getUVs() const; 

  const std::string &getName() const;
  const pxr::UsdPrim &getPrim() const;

private:
  pxr::UsdPrim prim;
  std::vector<Vertex> vertices;
  std::vector<Edge> edges;
  std::vector<Polygon> polygons;
  std::vector<UV> uvs;
};
#endif
