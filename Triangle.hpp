//
// Created by 闻永言 on 2021/7/11.
//

#ifndef RENDERER_TRIANGLE_HPP
#define RENDERER_TRIANGLE_HPP

#include <vector>

using namespace std;

class Triangle
{
public:
    // Index of vertices
    uint32_t vertex_0;
    uint32_t vertex_1;
    uint32_t vertex_2;

    Triangle(uint32_t v_0, uint32_t v_1, uint32_t v_2)
    {
        vertex_0 = v_0;
        vertex_1 = v_1;
        vertex_2 = v_2;
    };
};

#endif
