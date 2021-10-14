//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTER_CAMERA_HPP
#define RECONSTRUCTER_CAMERA_HPP

#include "Type.hpp"

namespace SfS
{
    enum SYSTEM
    {
        RIGHT_HAND,
        LEFT_HAND
    };
}

class Camera
{
public:
    // camera inner parameters
    int x; // pixel number in x
    int y; // pixel number in y

    float4 camera_center; // camera coordinate in world space
    float4 focal_center;
    float4 up;
    float FovH; // field of view
    float FovV; // field of view

    float n; // near is_clip plane
    float f; // far is_clip plane

    float4x4 M_view; // world space to camera space matrix
    float4x4 M_per; // perspective projection transformation matrix
    float4x4 M_viewport; // normal device space to screen space matrix

    float4x4 P; // M_viewport * M_per * M_view
    float4x4 VP; // M_per * M_view
    float4x4 PV; // M_viewport * M_per;

    /**
     *
     */
    Camera(){}

    /**
     * destructor
     */
    ~Camera();

    /**
     * Set camera viewport parameters.
     * @param _x window width
     * @param _y window height
     * @param ccd_size_x ccd size in x
     * @param ccd_size_y ccd size in y
     * @param focal focal of camera
     */
    void set_viewport(int _x, int _y, float ccd_size_x, float ccd_size_y, float focal);

    /**
     * Set camera viewport parameters
     * @param _x window width
     * @param _y window height
     * @param Fov FovV, FovH will be automatically set in window scale
     */
    void set_viewport(int _x, int _y, float Fov);

    /**
     * Set view space transformation matrix.
     * @param _camera_center camera center coordination
     * @param _focal_center focal center coordination
     * @param up look up vector
     * @param system right system (0), left system (1)
     */
    void set_look_at(float4 _camera_center, float4 _focal_center, float4 up, SfS::SYSTEM system = SfS::RIGHT_HAND);

    /**
     *
     * @param _camera_center
     * @param h
     * @param p
     * @param b
     * @param system right system (0), left system (1)
     */
    void set_look_at(float4 _camera_center, float h, float p, float b, SfS::SYSTEM system = SfS::RIGHT_HAND);

    /**
     *
     * @param R
     * @param t
     */
    void set_look_at(const float3x3 &R, const float3 &t);

    /**
     *
     * @param direction
     * @param angle
     */
    void rotate(float4 direction, float angle);

    /**
     * Convert the c4d left-handed world coordinate to camera right-handed coordinate.
     * Update the parameters.
     */
    void update();

    /**
     *
     * @param R_mu
     * @param R_sigma
     * @param t_mu
     * @param t_sigma
     */
    void add_outer_noise(float R_mu, float R_sigma, float t_mu, float t_sigma);
private:
    /**
    * Calculate rotation matrix of H, P, B.
    * @param H
    * @param P
    * @param B
    */
    void calculate_HPB(float3x3 &H, float3x3 &P, float3x3 &B) const;

    /**
     * Calculate rotation matrix l_R3.
     */
    float3x3 calculate_R();
};

#endif //RECONSTRUCTER_CAMERA_HPP
