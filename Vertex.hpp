//
// Created by 闻永言 on 2021/8/15.
//

#ifndef RENDERER_VERTEX_HPP
#define RENDERER_VERTEX_HPP

#include "Type.hpp"
#include <utility>

class Vertex
{
public:
    float4 world; // coordinate in world space

    /**
     *
     */
    Vertex()
    {
        world << 0, 0, 0, 1.0f;
    }

    /**
     *
     * @param v world space coordinate
     */
    Vertex(float4 v):world(std::move(v)){}
};

#endif //RENDERER_VERTEX_HPP
