//
// Created by 闻永言 on 2021/7/11.
//

#ifndef RECONSTRUCTOR_TRIANGLE_HPP
#define RECONSTRUCTOR_TRIANGLE_HPP

#include <vector>

namespace sfs {
    class Triangle {
    public:
        uint32_t vertexIndex[3];

        Triangle(uint32_t v0, uint32_t v1, uint32_t v2) {
            vertexIndex[0] = v0;
            vertexIndex[1] = v1;
            vertexIndex[2] = v2;
        };
    };
}

#endif
