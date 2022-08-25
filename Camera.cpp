//
// Created by 闻永言 on 2021/9/22.
//

#include "Camera.hpp"
#include "Util.hpp"
#include "Eigen/Geometry"

using namespace sfs;

Camera::Camera() = default;

Camera::~Camera() = default;

void Camera::setViewport(int _x, int _y, float ccdSizeX, float ccdSizeY, float _focal) {
    x = _x;
    y = _y;
    FovH = 2.0f * atanf(ccdSizeX / (2.0f * _focal));
    FovV = 2.0f * atanf(ccdSizeY / (2.0f * _focal));

    n = _focal / 1000.0f; // mm to m
    f = 10000.0f; // far is_clip plane
    float r = f * tanf(FovH * 0.5f);
    float l = -r;
    float t = f * tanf(FovV * 0.5f);
    float b = -t;

    matrixPerspective << 2.0f * f / (r - l), 0, 0, 0, 0, 2.0f * f / (t - b), 0, 0, 0, 0, f / (f - n), f * n /
            (n - f), 0, 0, 1.0f, 0;

    matrixViewport << (float)x * 0.5f, 0, 0, (float)x * 0.5f, 0, (float)y * 0.5f, 0, (float)y *
            0.5f, 0, 0, 1.0f, 0, 0, 0, 0, 1.0f;
}

void Camera::setViewport(int _x, int _y, float Fov) {
    x = _x;
    y = _y;
    FovH = Fov;
    FovV = 2.0f * atanf(tanf(deg2rad(Fov) * 0.5f) * (float)y / (float)x);

    n = tanf(Fov * 0.5f) / 1000.0f; // mm to m
    f = 10000.0f; // far is_clip plane
    float r = f * tanf(FovH * 0.5f);
    float l = -r;
    float t = f * tanf(FovV * 0.5f);
    float b = -t;

    matrixPerspective << 2.0f * f / (r - l), 0, 0, 0, 0, 2.0f * f / (t - b), 0, 0, 0, 0, f / (f - n), f * n /
            (n - f), 0, 0, 1.0f, 0;

    matrixViewport << (float)x * 0.5f, 0, 0, (float)x * 0.5f, 0, (float)y * 0.5f, 0, (float)y *
            0.5f, 0, 0, 1.0f, 0, 0, 0, 0, 1.0f;
}

void Camera::setLookAt(float4 cameraPos, float4 focalPos, float4 _up, sfs::SYSTEM mode) {
    if (mode == sfs::RIGHT_HAND) {
        position = std::move(cameraPos);
        focal = std::move(focalPos);
        up = std::move(_up);
        float4 Z = (focal - position).normalized();
        float4 X = Z.cross3(up).normalized();
        float4 Y = Z.cross3(X).normalized();
        float4 temp(0, 0, 0, 0);
        matrixView << X, Y, Z, temp;
        matrixView.transposeInPlace();
        float4 translation(-X.dot(position), -Y.dot(position), -Z.dot(position), 1.0f);
        matrixView.col(3) = translation;

        matrixVP = matrixPerspective * matrixView;
        matrixWorldToScreen = matrixViewport * matrixVP;
        matrixPV = matrixViewport * matrixPerspective;
    } else if (mode == sfs::LEFT_HAND) {
        cameraPos.y() = -cameraPos.y();
        focalPos.y() = -focalPos.y();
        _up.y() = -_up.y();
        setLookAt(cameraPos, focalPos, _up, sfs::RIGHT_HAND);
    }
}

void Camera::setLookAt(const float3x3 &R, const float3 &t) {
    Eigen::RowVector4f temp(0, 0, 0, 1.0f);
    matrixView << R, t, temp;
    matrixVP = matrixPerspective * matrixView;
    matrixWorldToScreen = matrixViewport * matrixVP;
    matrixPV = matrixViewport * matrixPerspective;
}

void Camera::addOuterNoise(float R_mu, float R_sigma, float t_mu, float t_sigma) {
//    matrixView(0, 0) += gaussian_rand(R_mu, R_sigma);
//    matrixView(0, 1) += gaussian_rand(R_mu, R_sigma);
//    matrixView(0, 2) += gaussian_rand(R_mu, R_sigma);
//    matrixView(1, 0) += gaussian_rand(R_mu, R_sigma);
//    matrixView(1, 1) += gaussian_rand(R_mu, R_sigma);
//    matrixView(1, 2) += gaussian_rand(R_mu, R_sigma);
//    matrixView(2, 0) += gaussian_rand(R_mu, R_sigma);
//    matrixView(2, 1) += gaussian_rand(R_mu, R_sigma);
//    matrixView(2, 2) += gaussian_rand(R_mu, R_sigma);
    matrixView(0, 3) += gaussian_rand(t_mu, t_sigma);
    matrixView(1, 3) += gaussian_rand(t_mu, t_sigma);
    matrixView(2, 3) += gaussian_rand(t_mu, t_sigma);

    float3x3 R;
    R << matrixView(0, 0), matrixView(0, 1), matrixView(0, 2), matrixView(1, 0), matrixView(1, 1), matrixView(1,
            2), matrixView(2, 0), matrixView(2, 1), matrixView(2, 2);
    float3 euler = R.eulerAngles(2, 1, 0);
    euler[0] += gaussian_rand(R_mu, R_sigma);
    euler[1] += gaussian_rand(R_mu, R_sigma);
    euler[2] += gaussian_rand(R_mu, R_sigma);
    R = Eigen::AngleAxisf(euler[0], float3::UnitZ()) * Eigen::AngleAxisf(euler[1], float3::UnitY()) *
            Eigen::AngleAxisf(euler[2], float3::UnitX());
    matrixView(0, 0) = R(0, 0);
    matrixView(0, 1) = R(0, 1);
    matrixView(0, 2) = R(0, 2);
    matrixView(1, 0) = R(1, 0);
    matrixView(1, 1) = R(1, 1);
    matrixView(1, 2) = R(1, 2);
    matrixView(2, 0) = R(2, 0);
    matrixView(2, 1) = R(2, 1);
    matrixView(2, 2) = R(2, 2);

    matrixVP = matrixPerspective * matrixView;
    matrixWorldToScreen = matrixViewport * matrixVP;
}

//void Camera::calculate_HPB(float3x3 &H, float3x3 &P, float3x3 &B) const
//{
//    //    float h_rad = -deg2rad(pitch);
//    //    float p_rad = -deg2rad(yaw);
//    //    float b_rad = -deg2rad(roll);
//    //
//    //    H << cos(h_rad), 0, sin(h_rad), 0, 1, 0, -sin(h_rad), 0, cos(h_rad);
//    //    P << 1, 0, 0, 0, cos(p_rad), -sin(p_rad), 0, sin(p_rad), cos(p_rad);
//    //    B << cos(b_rad), -sin(b_rad), 0, sin(b_rad), cos(b_rad), 0, 0, 0, 1;
//}
//
//float3x3 Camera::calculate_R()
//{
//    //    mat3 H, P, B, S_y;
//    //    calculate_HPB(H, P, B);
//    //    S_y << 1, 0, 0, 0, -1, 0, 0, 0, 1;
//    //    R3 = S_y * H * P * B * S_y;
//    //    R3.transposeInPlace();
//    //    Eigen::RowVector4f term_0(0, 0, 0, 1);
//    //    vec3 term_1(0, 0, 0);
//}
