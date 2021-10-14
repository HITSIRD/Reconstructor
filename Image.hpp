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
    index_t x; // width of image
    index_t y; // height of image
    index_t num_camera; // number of images shot by each camera
    unsigned char *data;
    int8_t *coefficients;
    bool *contours; // reference images contours, true means a point in contour, index is same as refer_image data

    /**
     *
     * @param _x width
     * @param _y height
     * @param channel
     */
    Image(index_t _x, index_t _y, index_t _num_camera);

    ~Image();

    /**
     *
     * @param offset image order
     * @param d
     */
    void set_data(index_t offset, const unsigned char *d) const;

    Image operator-(Image &img);
};

#endif //RECONSTRUCTER_IMAGE_HPP
