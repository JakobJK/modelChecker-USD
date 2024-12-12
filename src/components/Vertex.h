#ifndef VERTEX_H
#define VERTEX_H

#include <vector>
#include <glm/glm.hpp>

class Mesh;
class UV; 
class Edge;
class Polygon;
class Vertex {
public:
  Vertex(float x, float y, float z, Mesh &mesh);
  std::vector<Edge> getConnectedEdges() const; 
  std::vector<Polygon> getConnectedPolygons() const;
  glm::vec3 getPosition() const;
  int getConnectedEdgesCount() const;
  int getConnectedPolygonsCount() const;
  Mesh& getMesh() const;


private:
  friend class Mesh;
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  std::vector<int> connectedEdgesIndices;
  std::vector<int> connectedPolygonsIndices;
  std::vector<int> UVIndices;
  Mesh *mesh;
};

#endif
