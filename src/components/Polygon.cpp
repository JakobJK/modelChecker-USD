#include "Polygon.h"
#include "../Mesh.h"
#include <cstdint>
#include <earcut.hpp>
#include <glm/glm.hpp>
#include <iostream>
#include <vector>

Polygon::Polygon(const std::vector<int> &vertexIndices,
                 const std::vector<int> &uvIndices, Mesh &mesh, int id)
    : mesh(&mesh), vertexIndices(vertexIndices), uvIndices(uvIndices), id(id) {
    for (const auto &pair : uvVertexPairs) {
        uvVertexPairs[pair.first] = pair.second;
    }

}

int Polygon::getTriangleCount() const {
    std::vector<UV> uvs = getUVs();
    using Point = std::array<double, 2>;
    std::vector<std::vector<Point>> polygon(1);

    for (const auto &uv : uvs) {
        auto curUV = uv.getPos();
        polygon[0].push_back({curUV.first, curUV.second});
    }
    auto indices = mapbox::earcut<uint32_t>(polygon);
    return int(indices.size() / 3);
}

std::vector<uint32_t> Polygon::getTriangles(bool get3dTriangles) const {
    std::vector<UV> uvs = getUVs();
    using Point = std::array<double, 2>;
    std::vector<std::vector<Point>> polygon(1);

    for (const auto &uv : uvs) {
        auto curUV = uv.getPos();
        polygon[0].push_back({curUV.first, curUV.second});
    }
    auto indices = mapbox::earcut<uint32_t>(polygon);

    if (get3dTriangles) {
        for (auto &index : indices) {
            auto it = uvVertexPairs.find(index);
            if (it != uvVertexPairs.end()) {
                index = it->second;
            }
        }
    }
    return indices;
}

int Polygon::getVertexCount() const { return vertexIndices.size(); }
std::vector<UV> Polygon::getUVs() const {
    std::vector<UV> result;
    const std::vector<UV> &allUVs = mesh->getUVs();
    for (int index : uvIndices) {
        result.push_back(allUVs[index]);
    }
    return result;
}

float Polygon::getArea() const {
    auto indices = getTriangles(true);
    const auto &allVertices = mesh->getVertices();
    std::vector<Vertex> vertices;
    for (uint32_t index : indices) {
        vertices.push_back(allVertices[index]);
    }
    float area = 0.0f;

    for (size_t i = 0; i < vertices.size(); i += 3) {
        glm::vec3 p1 = vertices[i].getPosition();
        glm::vec3 p2 = vertices[i + 1].getPosition();
        glm::vec3 p3 = vertices[i + 2].getPosition();
        glm::vec3 edge1 = p2 - p1;
        glm::vec3 edge2 = p3 - p1;
        area += glm::length(glm::cross(edge1, edge2)) / 2.0f;
    }

    return area;
}

float Polygon::getUVArea() const {
    std::vector<UV> uvs = getUVs();
    auto indices = getTriangles(false);
    double area = 0.0;
    for (size_t i = 0; i < indices.size(); i += 3) {
        const auto &p1 = uvs[indices[i]].getPos();
        const auto &p2 = uvs[indices[i + 1]].getPos();
        const auto &p3 = uvs[indices[i + 2]].getPos();

        const auto p1u = p1.first;
        const auto p1v = p1.second;

        const auto p2u = p2.first;
        const auto p2v = p2.second;

        const auto p3u = p3.first;
        const auto p3v = p3.second;

        double a = glm::length(glm::vec2(p2u - p1u, p2v - p1v));
        double b = glm::length(glm::vec2(p3u - p2u, p3v - p2v));
        double c = glm::length(glm::vec2(p1u - p3u, p1v - p3v));
        double s = 0.5 * (a + b + c);
        area += std::sqrt(s * (s - a) * (s - b) * (s - c));
    }

    return static_cast<float>(area);
}

bool Polygon::isConcave() const {
    const auto &vertices = mesh->getVertices();
    size_t numVertices = getVertexCount();

    if (numVertices < 4)
        return false;

    glm::vec3 firstNormal;
    bool isFirstNormal = true;

    for (size_t i = 0; i < numVertices; i++) {
        size_t nextIndex1 = (i + 1) % numVertices;
        size_t nextIndex2 = (i + 2) % numVertices;

        glm::vec3 current = vertices[vertexIndices[i]].getPosition();
        glm::vec3 next1 = vertices[vertexIndices[nextIndex1]].getPosition();
        glm::vec3 next2 = vertices[vertexIndices[nextIndex2]].getPosition();

        glm::vec3 edge1 = next1 - current;
        glm::vec3 edge2 = next2 - next1;
        glm::vec3 normal = glm::cross(edge1, edge2);

        if (isFirstNormal) {
            firstNormal = normal;
            isFirstNormal = false;
        } else {
            if (glm::dot(firstNormal, normal) < 0) {
                return true;
            }
        }
    }

    return false;
}

const std::vector<int>& Polygon::getUVIndices() const {
    return vertexIndices;
}

int Polygon::getGlobalVertexId(int id) const {
    if (id >= vertexIndices.size()) {
        return -1;
    }
    return vertexIndices[id];
}

int Polygon::getGlobalUVId(int id) const {
    if (id >= uvIndices.size()){
        return -1;
    }
    return uvIndices[id];
}



int Polygon::getId() const { return id; }
