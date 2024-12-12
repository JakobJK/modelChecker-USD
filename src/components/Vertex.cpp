#include "Vertex.h"
#include "../Mesh.h"
#include "Edge.h"
#include "Polygon.h"
#include <glm/glm.hpp>
#include <iostream>

Vertex::Vertex(float x, float y, float z, Mesh &mesh)
    : x(x), y(y), z(z), mesh(&mesh) {}

std::vector<Edge> Vertex::getConnectedEdges() const {
  std::vector<Edge> connectedEdges;
  const std::vector<Edge> &allEdges = mesh->getEdges();
  for (int index : connectedEdgesIndices) {
    if (index >= 0 && index < allEdges.size()) {
      connectedEdges.push_back(allEdges[index]);
    }
  }
  return connectedEdges;
}

std::vector<Polygon> Vertex::getConnectedPolygons() const {
    std::vector<Polygon> connectedPolygons;
    const std::vector<Polygon> &allPolygons = mesh->getPolygons();
    for (int index : connectedPolygonsIndices){
        if (index >= 0 && index < allPolygons.size()){
            connectedPolygons.push_back(allPolygons[index]);
        }
    }
    return connectedPolygons;
}



int Vertex::getConnectedEdgesCount() const {
  return connectedEdgesIndices.size();
}

int Vertex::getConnectedPolygonsCount() const {
    return connectedPolygonsIndices.size();
}


Mesh &Vertex::getMesh() const { return *mesh; }

glm::vec3 Vertex::getPosition() const { return glm::vec3(x, y, z); }
