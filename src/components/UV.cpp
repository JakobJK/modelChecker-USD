#include "UV.h"
#include <utility>

UV::UV(float u, float v) : u(u), v(v) {}

std::pair<float, float> UV::getPos() const {
        return {u, v};
} 


int UV::getUdim() const {
    int u_tile = static_cast<int>(u);
    int v_tile = static_cast<int>(v);
    
    if (u_tile < 0 || u_tile > 9 || v_tile < 0) {
        return -1; 
    }

    int udim = 1000 + (u_tile + 1) + ((v_tile + 1) * 10);
    return udim;
}
