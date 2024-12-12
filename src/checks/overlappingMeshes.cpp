#include "overlappingMeshes.h"
#include "../Mesh.h"
#include "../components/Polygon.h"
#include "../components/UV.h"
#include <cmath>
#include <fstream>
#include <iostream>

void draw_dot(int x, int y, std::vector<int> &buffer,int &bufferSize, int polygonId) {
    if (x >= 0 && x < bufferSize && y >= 0 && y < bufferSize) {
        int idx = bufferSize * y + x;
        buffer[idx] += 1;
    }
}

void drawline(int y, int xStart, int xEnd, std::vector<int> &buffer, int &bufferSize,
              int polygonId) {
    for (int x = xStart + 1; x <= xEnd; ++x) {
        draw_dot(x, y, buffer, bufferSize, polygonId);
    }
}

void paintTriangle(const std::vector<glm::vec2> &UVs, std::vector<int> &buffer, int &bufferSize,
                   int polygonId) {
    auto [a, b, c] = std::tie(UVs[0], UVs[1], UVs[2]);
    float invTSlope1 = 0.0f, invTSlope2 = 0.0f;
    float invBSlope1 = 0.0f, invBSlope2 = 0.0f;

    if (a.y != b.y) {
        invTSlope1 = (b.x - a.x) / std::abs(b.y - a.y);
    }

    if (a.y != c.y) {
        invTSlope2 = (c.x - a.x) / std::abs(c.y - a.y);
        invBSlope2 = invTSlope2;
    }

    if (c.y != b.y) {
        invBSlope1 = (c.x - b.x) / std::abs(c.y - b.y);
    }

    const auto size = bufferSize;

    if (a.y != b.y) {
        for (int y = static_cast<int>(std::ceil(a.y * size));
             y < static_cast<int>(std::ceil(b.y * size)); ++y) {
            float xStartF = b.x * size + (y - b.y * size) * invTSlope1;
            float xEndF = a.x * size + (y - a.y * size) * invTSlope2;
            if (xStartF > xEndF) {
                std::swap(xStartF, xEndF);
            }
            int xStart = static_cast<int>(std::ceil(xStartF));
            int xEnd = static_cast<int>(std::floor(xEndF));
            drawline(y, xStart, xEnd, buffer, bufferSize, polygonId);
        }
    }

    if (b.y != c.y) {
        for (int y = static_cast<int>(std::ceil(b.y * size));
             y < static_cast<int>(std::ceil(c.y * size)); ++y) {
            float xStartF = b.x * size + (y - b.y * size) * invBSlope1;
            float xEndF = a.x * size + (y - a.y * size) * invBSlope2;
            if (xStartF > xEndF) {
                std::swap(xStartF, xEndF);
            }

            int xStart = static_cast<int>(std::ceil(xStartF));
            int xEnd = static_cast<int>(std::ceil(xEndF));
            drawline(y, xStart, xEnd, buffer, bufferSize, polygonId);
        }
    }
}


void outputFile(const std::vector<int> &buffer, int &bufferSize){
    std::ofstream myfile;
    myfile.open("image.ppm");

    if (!myfile.is_open()) {
        std::cerr << "Error: Could not open the file for writing." << std::endl;
        return;
    }

    myfile << "P3\n" << bufferSize << ' ' << bufferSize << "\n255\n";
    for (int i = 0; i < bufferSize; ++i) {
        for (int j = 0; j < bufferSize; ++j) {
            int cur = buffer[i * bufferSize + j];

            if (cur == -1) {

                int ir = 0;
                int ig = 0;
                int ib = 0;
                myfile << ir << ' ' << ig << ' ' << ib << '\n';

            } else if (cur == 0) {

                int ir = 180;
                int ig = 200;
                int ib = 30;
                myfile << ir << ' ' << ig << ' ' << ib << '\n';
            } else {

                int ir = 0; // 0, this should be fine ?? lolol
                int ig = 180;
                int ib = 180;
                myfile << ir << ' ' << ig << ' ' << ib << '\n';
            }
        }
    }
    myfile.close();
};

void overlappingMeshes(const std::vector<std::unique_ptr<Mesh>> &meshes) {

    int bufferSize = 4096;
    std::vector<int> buffer(bufferSize * bufferSize, -1);
    for (const auto &mesh_ptr : meshes) {
        const auto polygons = mesh_ptr->getPolygons();
        const auto uvs = mesh_ptr->getUVs();
        for (const auto &polygon : polygons) {
            const auto triangles = polygon.getTriangles(false);
            for (size_t i = 0; i < triangles.size(); i += 3) {
                const auto aIdx = polygon.getGlobalUVId(triangles[i]);
                const auto bIdx = polygon.getGlobalUVId(triangles[i + 1]);
                const auto cIdx = polygon.getGlobalUVId(triangles[i + 2]);

                auto a_pair = uvs[aIdx].getPos();
                auto b_pair = uvs[bIdx].getPos();
                auto c_pair = uvs[cIdx].getPos();

                glm::vec2 a{a_pair.first, a_pair.second};
                glm::vec2 b{b_pair.first, b_pair.second};
                glm::vec2 c{c_pair.first, c_pair.second};

                std::vector<glm::vec2> UVs = {a, b, c};
                std::sort(UVs.begin(), UVs.end(),
                          [](const glm::vec2 &lhs, const glm::vec2 &rhs) {
                              return lhs.y < rhs.y;
                          });
                paintTriangle(UVs, buffer, bufferSize, polygon.getId());
            }
        }
    }
    outputFile(buffer, bufferSize);
}
