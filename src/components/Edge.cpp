#include "Edge.h"
#include "../Mesh.h"
#include "Polygon.h"
#include "Vertex.h"
#include <glm/glm.hpp>
#include <iostream>
#include <vector>

Edge::Edge(int vertIdxA, int vertIdxB, Mesh &mesh)
    : vertIdxA(vertIdxA), vertIdxB(vertIdxB), mesh(&mesh) {}

int Edge::getPolygonCount() const { return connectedPolygonsIndices.size(); }

float Edge::getLength() const {
  const std::vector<Vertex> &vertsList = mesh->getVertices();
  Vertex vertA = vertsList[vertIdxA];
  Vertex vertB = vertsList[vertIdxB];
  auto p1 = vertA.getPosition();
  auto p2 = vertB.getPosition();
  
  auto pos1 = glm::vec3(p1[0], p1[1], p1[2]);
  auto pos2 = glm::vec3(p2[0], p2[1], p2[2]);
  float result = glm::distance(pos1, pos2);
  return result;
}

std::vector<Polygon> Edge::getPolygons() const {
  std::vector<Polygon> allPolygons = mesh->getPolygons();
  std::vector<Polygon> resultPolygons;

  for (int idx : connectedPolygonsIndices) {
    if (idx >= 0 && idx < allPolygons.size()) {
      resultPolygons.push_back(allPolygons[idx]);
    }
  }
  return resultPolygons;
}
