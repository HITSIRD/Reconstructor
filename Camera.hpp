//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTER_CAMERA_HPP
#define RECONSTRUCTER_CAMERA_HPP

#include "Type.hpp"

namespace sfs {
    enum SYSTEM {
        RIGHT_HAND, LEFT_HAND
    };

    class Camera {
    public:
        // camera inner parameters
        int x; // pixel number in x
        int y; // pixel number in y

        float4 position; // camera coordinate in world space
        float4 focal;
        float4 up;
        float FovH; // field of view
        float FovV; // field of view

        float n; // near is_clip plane
        float f; // far is_clip plane

        float4x4 matrixView; // world space to camera space matrix
        float4x4 matrixPerspective; // perspective projection transformation matrix
        float4x4 matrixViewport; // normal device space to screen space matrix

        float4x4 matrixWorldToScreen; // matrixView * matrixPerspective * matrixView
        float4x4 matrixVP; // matrixPerspective * matrixView
        float4x4 matrixPV; // matrixViewport * matrixPerspective

        Camera();

        /**
         * destructor
         */
        ~Camera();

        /**
         * Set camera viewport parameters.
         * @param _x window width
         * @param _y window height
         * @param ccdSizeX ccd size in x
         * @param ccdSizeY ccd size in y
         * @param _focal focal of camera
         */
        void setViewport(int _x, int _y, float ccdSizeX, float ccdSizeY, float _focal);

        /**
         * Set camera viewport parameters
         * @param _x window width
         * @param _y window height
         * @param Fov FovV, FovH will be automatically set in window scale
         */
        void setViewport(int _x, int _y, float Fov);

        /**
         * Set view space transformation matrix.
         * @param cameraPos camera center coordination
         * @param focalPos focal center coordination
         * @param _up look up vector
         * @param system right system (0), left system (1)
         */
        void setLookAt(float4 cameraPos, float4 focalPos, float4 _up, sfs::SYSTEM system = sfs::RIGHT_HAND);

        /**
         *
         * @param cameraPos
         * @param h
         * @param p
         * @param b
         * @param system right system (0), left system (1)
         */
        void setLookAt(float4 cameraPos, float h, float p, float b, sfs::SYSTEM system = sfs::RIGHT_HAND);

        /**
         *
         * @param R
         * @param t
         */
        void setLookAt(const float3x3 &R, const float3 &t);

        /**
         *
         * @param direction
         * @param angle
         */
        void rotate(float4 direction, float angle);

        /**
         *
         * @param R_mu
         * @param R_sigma
         * @param t_mu
         * @param t_sigma
         */
        void addOuterNoise(float R_mu, float R_sigma, float t_mu, float t_sigma);

    private:
//        /**
//        * Calculate rotation matrix of H, P, B.
//        * @param H
//        * @param P
//        * @param B
//        */
//        void calculate_HPB(float3x3 &H, float3x3 &P, float3x3 &B) const;
//
//        /**
//         * Calculate rotation matrix l_R3.
//         */
//        float3x3 calculate_R();
    };
}

#endif //RECONSTRUCTER_CAMERA_HPP
