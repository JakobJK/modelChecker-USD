#ifndef POLYGON_H
#define POLYGON_H

#include <vector>
#include <unordered_map>
#include <cstdint>

class Mesh;
class UV;
class Polygon {
  private:
    friend class Mesh;
    int id;
    std::vector<int> vertexIndices;
    std::vector<int> uvIndices;
    Mesh *mesh;
    std::unordered_map<int, int> uvVertexPairs;
  public:
    int getGlobalVertexId(int id) const; 
    int getGlobalUVId(int id) const; 
    std::vector<uint32_t> getTriangles(bool get3dTriangles) const; 
    Polygon(const std::vector<int> &vertexIndices,
            const std::vector<int> &uvIndices, Mesh &mesh, int id);
    std::vector<UV> getUVs() const;
    int getVertexCount() const;
    int getTriangleCount() const;
    bool isConcave() const;
    float getArea() const;
    float getUVArea() const;
    int getId() const;
    const std::vector<int>& getUVIndices() const;
};

#endif
