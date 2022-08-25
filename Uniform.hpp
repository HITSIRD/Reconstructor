//
// Created by 闻永言 on 2022/1/4.
//

#ifndef RECONSTRUCTOR_UNIFORM_HPP
#define RECONSTRUCTOR_UNIFORM_HPP

#include "Type.hpp"
#include "Camera.hpp"
#include "Image.hpp"

namespace sfs {
    class Uniform {
    public:
        std::vector<Camera *> *cameras;
        Image *referImage;
        index_t numPixels;

        Uniform();

        /**
         *
         */
        Uniform(index_t _numPixels);

        ~Uniform();

        /**
         *
         * @param camerasPointer
         */
        void setCameras(std::vector<Camera *> *camerasPointer);

        /**
         *
         * @param refer
         */
        void setReferImage(Image *refer);
    };
}

#endif //RECONSTRUCTOR_UNIFORM_HPP
