//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTER_IMAGE_HPP
#define RECONSTRUCTER_IMAGE_HPP

#include <cstdint>
#include "Type.hpp"

class Image
{
public:
    int x; // width of image
    int y; // height of image
    int num_camera; // number of images shot by each camera
    unsigned char *data;
    int8_t *coefficients;
    bool *contours; // reference images contours, true means a point in contour, index is same as refer_image data

    /**
     *
     * @param _x width
     * @param _y height
     * @param channel
     */
    Image(int _x, int _y, int _num_camera);

    ~Image();

    /**
     *
     * @param offset image order
     * @param d
     */
    void set_data(int offset, const unsigned char *d) const;

    Image operator-(Image &img);
};

#endif //RECONSTRUCTER_IMAGE_HPP
