#ifndef UV_H
#define UV_H

#include <string>
#include <utility>
class UV {
        public:
                UV(float u, float v);
                int getUdim() const;
                std::pair<float, float> getPos() const;
        private:
                float u;
                float v;
};
#endif
