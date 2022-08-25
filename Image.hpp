//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTOR_IMAGE_HPP
#define RECONSTRUCTOR_IMAGE_HPP

#include <cstdint>
#include "Type.hpp"

namespace sfs {
    class Image {
    public:
        int x; // width of image
        int y; // height of image
        int numCameras; // number of images shot by each camera
        unsigned char *data;
        int8_t *coefficients;
        bool *contours; // reference images contours, true means a point in contour, index is same as refer_image data

        /**
         *
         * @param _x
         * @param _y
         * @param _numCameras
         */
        Image(int _x, int _y, int _numCameras);

        ~Image();

        /**
         *
         * @param offset image order
         * @param d
         */
        void setData(int offset, const unsigned char *d) const;
    };
}

#endif //RECONSTRUCTOR_IMAGE_HPP
