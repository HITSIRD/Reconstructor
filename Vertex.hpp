//
// Created by 闻永言 on 2021/8/15.
//

#ifndef RECONSTRUCTOR_VERTEX_HPP
#define RECONSTRUCTOR_VERTEX_HPP

#include "Type.hpp"
#include <utility>

namespace sfs {
    class Vertex {
    public:
        float4 position; // coordinate in world space

        /**
         *
         */
        Vertex() {
            position << 0, 0, 0, 1.f;
        }

        /**
         *
         * @param v world space coordinate
         */
        Vertex(float4 v): position(std::move(v)) {}
    };
}

#endif //RECONSTRUCTOR_VERTEX_HPP
