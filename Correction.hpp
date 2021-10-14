//
// Created by 闻永言 on 2021/10/7.
//

#ifndef RECONSTRUCTOR_CORRECTION_HPP
#define RECONSTRUCTOR_CORRECTION_HPP

#include "Model.hpp"

namespace SfS
{
    enum CORRECTION_MODE
    {
        MATRIX, EULER_ANGLE
    };
}

class Correction
{
public:
    /**
     * Camera calibration correction using LM algorithm.
     * @param uniform camera parameters to correct
     * @param matches_3d_2d
     * @param matches_2d_2d
     * @param vertices vertices set in the model, to record the world coordination by the vertex array index
     * @param mode MATRIX: to correction rotate matrix directly, EULER_ANGLE: to correction the rotate matrix after
     * converting to euler angle, then convert back to matrix form, using ZYX order.
     */
    static void correction(
            Uniform *uniform, const std::vector<Pair<index_t, index_t>> &matches_3d_2d,
            const std::vector<Pair<index_t, index_t>> &matches_2d_2d, Vertex *vertices,
            SfS::CORRECTION_MODE mode = SfS::EULER_ANGLE);

private:
    /**
     *
     * @param uniform
     * @param matches_3d_2d
     * @param matches_2d_2d
     * @param vertices
     */
    static void correction_matrix(
            Uniform *uniform, const std::vector<Pair<index_t, index_t>> &matches_3d_2d,
            const std::vector<Pair<index_t, index_t>> &matches_2d_2d, Vertex *vertices);

    /**
     *
     * @param uniform
     * @param matches_3d_2d
     * @param matches_2d_2d
     * @param vertices
     */
    static void correction_euler(
            Uniform *uniform, const std::vector<Pair<index_t, index_t>> &matches_3d_2d,
            const std::vector<Pair<index_t, index_t>> &matches_2d_2d, Vertex *vertices);
};

#endif //RECONSTRUCTOR_CORRECTION_HPP
