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
    uint8_t bound; // 0x00000000: 0x00 0(F) 0(B) 0(L) 0(R) 0(T) 0(B), 1 means neighbor is occupied

    Voxel():occupied(true), test(false), bound(0){}

    ~Voxel() = default;
};

#endif //RECONSTRUCTER_VOXEL_HPP
