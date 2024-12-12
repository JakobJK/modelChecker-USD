#ifndef BUFFER_H
#define BUFFER_H

#include <vector>
#include <set>


class Buffer {
    private:
        std::vector<std::vector<std::set<int>>> store;
        int size;
    public:
        Buffer (int size);
        int getSize() const;
};

#endif
