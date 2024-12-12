#pragma once

#include <vector>
#include "Vertex.h"
class Mesh;
class Polygon;

class Edge {

public:
  Edge(int vertIdxA, int vertIdxB, Mesh &mesh);
  std::vector<Polygon> getPolygons() const;
  std::vector<Vertex> getVertices() const;
  int getPolygonCount() const;
  float getLength() const;
private:
  friend class Mesh;
  int vertIdxA;
  int vertIdxB;
  std::vector<int> connectedPolygonsIndices;
  Mesh *mesh;
};
