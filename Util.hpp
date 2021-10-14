//
// Created by 闻永言 on 2021/9/17.
//

#ifndef RENDERER_UTIL_HPP
#define RENDERER_UTIL_HPP

#include <random>

#define PI 3.1415926536f

/**
 *
 * @param deg
 * @return
 */
inline float deg2rad(float deg)
{
    return deg * PI / 180.0f;
}

/**
 *
 * @param rad
 * @return
 */
inline float rad2deg(float rad)
{
    return rad * 180.0f / PI;
}

/**
 *
 * @tparam T
 * @param v0
 * @param v1
 * @param v2
 * @param u
 * @param v
 * @param w
 * @return
 */
template<typename T> inline T lerp(const T &v0, const T &v1, const T &v2, float u, float v, float w)
{
    return u * v0 + v * v1 + w * v2;
}

/**
 *
 * @param x1
 * @param y1
 * @param x0
 * @param y0
 * @return
 */
inline float distance2d(index_t x1, index_t y1, index_t x0, index_t y0)
{
    float dx = (float)x1 - (float)x0;
    float dy = (float)y1 - (float)y0;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * Judge if a point is in a triangle.
 * @param AB
 * @param BC
 * @param CA
 * @return
 */
inline bool is_in_triangle(float AB, float BC, float CA)
{
    float S = 1.0f / (AB + BC + CA);
    return BC * S < 1.0f && CA * S < 1.0f && AB * S < 1.0f;
}

/**
 * Get a random number obeys the Gauss distribution.
 * @param mu
 * @param sigma
 * @return
 */
static float gaussian_rand(float mu, float sigma)
{
    std::default_random_engine e;
    std::random_device rd;
    std::normal_distribution<> gaussian(mu, sigma);
    e.seed(rd());
    return gaussian(e);
}

#endif //RENDERER_UTIL_HPP
