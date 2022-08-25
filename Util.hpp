//
// Created by 闻永言 on 2021/9/17.
//

#ifndef RECONSTRUCTOR_UTIL_HPP
#define RECONSTRUCTOR_UTIL_HPP

#include <random>

namespace sfs {
    /**
     * utility constants
     */
    static const float Pi = 3.1415926535897932f;
    static const float Pi2 = 6.2831853071795865f; // 2 * Pi
    static const float InvPi = 0.3183098861837907f; // 1 / Pi
    static const float Inv2Pi = 0.6366197723675813f; // 2 / Pi
    static const float Inv4Pi = 1.2732395447351627f; // 4 / Pi
    static const float PiOver2 = 1.5707963267948966f; // Pi / 2
    static const float PiOver4 = 0.7853981633974483f; // Pi / 4
    static const float Sqrt2 = 1.4142135623730950f; // Pi / 4
    static const float Log2 = 0.6931471805599453f; // log(2)
    static const float InvLog2 = 1.4426950408889634f; // 1 / log(2)
    static const float Inv255 = 0.0039215686274510f; // 1 / 255.0
    static const float Inv65535 = 0.0000152590218967f; // 1 / 65535.0

    static const float4 ZeroFloat4(0, 0, 0, 0);
    static const float4
            RandomFloat4(0.5305025836169190f, 0.4564595701410036f, -0.7142910258448331f, 0); // random float4 vector


    /**
     *
     * @param deg
     * @return
     */
    inline float deg2rad(float deg) {
        return deg * Pi / 180.f;
    }

    /**
     *
     * @param rad
     * @return
     */
    inline float rad2deg(float rad) {
        return rad * 180.f / Pi;
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
    template<typename T> inline T lerp(const T &v0, const T &v1, const T &v2, float u, float v, float w) {
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
    inline float distance2d(index_t x1, index_t y1, index_t x0, index_t y0) {
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
    inline bool is_in_triangle(float AB, float BC, float CA) {
        float S = 1.0f / (AB + BC + CA);
        return BC * S < 1.0f && CA * S < 1.0f && AB * S < 1.0f;
    }

    /**
     * Get a random number obeys the Gauss distribution.
     * @param mu
     * @param sigma
     * @return
     */
    static float gaussian_rand(float mu, float sigma) {
        std::default_random_engine e;
        std::random_device rd;
        std::normal_distribution<> gaussian(mu, sigma);
        e.seed(rd());
        return gaussian(e);
    }

    /**
     *
     * @tparam T
     * @param arg
     * @return
     */
    template<class T> inline std::string toString(const T &arg) {
        std::ostringstream s;
        s << arg;
        return s.str();
    }
}

#endif //RECONSTRUCTOR_UTIL_HPP
