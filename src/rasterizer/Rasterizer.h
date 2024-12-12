#ifndef RASTERIZER_H
#define RASTERIZER_H

#include <vector>
#include <set>
#include <string>
#include <glm/glm.hpp>
#include "../components/Polygon.h"
#include "../components/Polygon.h"
#include "Buffer.h"

class Rasterizer {
public:
    Rasterizer(int bufferSize);
    void drawTriangle(glm::vec3 triangle, Buffer &buffer);
    void drawLine(glm::vec2 point_a, glm::vec2 point_b, Buffer &buffer);
    void drawDot(glm::vec2 a, Buffer &buffer);
    
};

#endif

