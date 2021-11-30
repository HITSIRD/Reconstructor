//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTER_IODATA_HPP
#define RECONSTRUCTER_IODATA_HPP

#include <string>
#include "Camera.hpp"
#include "SfIS.hpp"

class iodata
{
public:
    /**
     *
     * @param file_name
     * @param camera
     * @param refer_image
     * @param bounding_box
     * @param voxel_size
     * @oaram iteration
     */
    static void read_config(
            const std::string &file_name, std::vector<Camera *> &camera, Image *refer_image, BoundingBox &bounding_box,
            float &voxel_size, int &iteration);

    /**
     *
     * @param voxel_config model configuration
     * @param config reference image parameters
     * @param sfis
     */
    static void read_config(const std::string &voxel_config, const std::string &config, SfIS &sfis);

    /**
     *
     * @param directory working directory
     * @param model
     */
    static void write_ply(const std::string &directory, Model *model);

    /**
     *
     * @param directory working directory
     * @param image
     */
    static void write_projection(const std::string &directory, const Image &image);

    /**
     *
     * @param directory working directory
     * @param image
     */
    static void write_test(const std::string &directory, const Image &image);

    /**
     *
     * @param directory working directory
     * @param image
     */
    static void write_sie(const std::string &directory, const Image &image, const Image &refer);

    /**
     *
     * @param directory working directory
     * @param image
     */
    static void write_refer_contours(const std::string &directory, const Image &image);

    /**
     *
     * @param directory working directory
     * @param image
     */
    static void write_recon_contours(const std::string &directory, const Image &image);
};

template<class T> inline string to_string(T arg)
{
    std::ostringstream s;
    s << arg;
    return s.str();
}

#endif //RECONSTRUCTER_IODATA_HPP
