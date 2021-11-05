//
// Created by 闻永言 on 2021/8/18.
//

#ifndef RENDERER_TYPE_HPP
#define RENDERER_TYPE_HPP

#include "Eigen/Core"

typedef Eigen::Vector2f float2;
typedef Eigen::Vector2i int2;
typedef Eigen::Vector3f float3;
typedef Eigen::Vector3i int3;
typedef Eigen::Vector4f float4;
typedef Eigen::Vector4i int4;

typedef Eigen::Vector4f float4;
typedef Eigen::Vector3f float3;

typedef Eigen::Matrix4f float4x4;
typedef Eigen::Matrix3f float3x3;

typedef uint32_t index_t;
typedef int64_t error_t;

namespace SfS
{
    enum DirectionType
    {
        FRONT = 0b00000001,
        BACK = 0b00000010,
        LEFT = 0b00000100,
        RIGHT = 0b00001000,
        TOP = 0b00010000,
        BOTTOM = 0b00100000
    };

    enum DirectionMask
    {
        MASK_FRONT = 0b00111110,
        MASK_BACK = 0b00111101,
        MASK_LEFT = 0b00111011,
        MASK_RIGHT = 0b00110111,
        MASK_TOP = 0b00101111,
        MASK_BOTTOM = 0b00011111,
        BOUND = 0b00111111
    };
}

#endif //RENDERER_TYPE_HPP
