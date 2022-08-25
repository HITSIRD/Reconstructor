//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTOR_IODATA_HPP
#define RECONSTRUCTOR_IODATA_HPP

#include <string>
#include "Camera.hpp"
#include "SfIS.hpp"

namespace sfs {
    class iodata {
    public:
        /**
         *
         * @param fileName
         * @param camera
         * @param referImage
         * @param boundingBox
         * @param voxelSize
         * @oaram iteration
         */
        static void readConfig(
                const std::string &fileName, std::vector<Camera *> &camera, Image *referImage, BoundingBox &boundingBox,
                float &voxelSize, int &iteration);

        /**
         *
         * @param voxelConfig model configuration
         * @param config reference image parameters
         * @param sfis
         */
        static void readConfig(const std::string &voxelConfig, const std::string &config, SfIS &sfis);

        /**
         *
         * @param directory working directory
         * @param model
         */
        static void writePly(const std::string &directory, Model *model);

        /**
         *
         * @param directory working directory
         * @param image
         */
        static void writeProjection(const std::string &directory, const Image &image);

        /**
         *
         * @param directory working directory
         * @param image
         */
        static void writeTest(const std::string &directory, const Image &image);

        /**
         *
         * @param directory working directory
         * @param image
         */
        static void writeSie(const std::string &directory, const Image &image, const Image &refer);

        /**
         *
         * @param directory working directory
         * @param image
         */
        static void writeReferContours(const std::string &directory, const Image &image);

        /**
         *
         * @param directory working directory
         * @param image
         */
        static void writeReconContours(const std::string &directory, const Image &image);
    };
}

#endif //RECONSTRUCTOR_IODATA_HPP
