//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTER_VOXEL_HPP
#define RECONSTRUCTER_VOXEL_HPP

#include <cstdint>

class Voxel
{
public:
    bool occupied;
    bool test;
    uint8_t bound; // 0x00000000: 0x00 0(BOTTOM) 0(TOP) 0(RIGHT) 0(LEFT) 0(BACK) 0(FRONT), 1 means neighbor is occupied

    Voxel():occupied(true), test(false), bound(0){}

    ~Voxel() = default;
};

#endif //RECONSTRUCTER_VOXEL_HPP
