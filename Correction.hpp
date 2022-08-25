//
// Created by 闻永言 on 2021/10/7.
//

#ifndef RECONSTRUCTOR_CORRECTION_HPP
#define RECONSTRUCTOR_CORRECTION_HPP

#include "Model.hpp"

namespace sfs {
    enum CORRECTION_MODE {
        MATRIX, EULER_ANGLE
    };

    class Correction {
    public:
        /**
         * Camera calibration correction using LM algorithm.
         * @param uniform camera parameters to correct
         * @param matches3D2D
         * @param matches2D2D
         * @param vertices vertices set in the model, to record the world coordination by the vertex array index
         * @param mode MATRIX: to correction rotate matrix directly, EULER_ANGLE: to correction the rotate matrix after
         * converting to euler angle, then convert back to matrix form, using ZYX order.
         */
        static void correction(
                Uniform *uniform, const std::vector<std::pair<index_t, index_t>> &matches3D2D,
                const std::vector<std::pair<index_t, index_t>> &matches2D2D, Vertex *vertices,
                sfs::CORRECTION_MODE mode = sfs::EULER_ANGLE);

    private:
        /**
         *
         * @param uniform
         * @param matches3D2D
         * @param matches2D2D
         * @param vertices
         */
        static void correctionMatrix(
                Uniform *uniform, const std::vector<std::pair<index_t, index_t>> &matches3D2D,
                const std::vector<std::pair<index_t, index_t>> &matches2D2D, Vertex *vertices);

        /**
         *
         * @param uniform
         * @param matches3D2D
         * @param matches2D2D
         * @param vertices
         */
        static void correctionEuler(
                Uniform *uniform, const std::vector<std::pair<index_t, index_t>> &matches3D2D,
                const std::vector<std::pair<index_t, index_t>> &matches2D2D, Vertex *vertices);
    };
}

#endif //RECONSTRUCTOR_CORRECTION_HPP
