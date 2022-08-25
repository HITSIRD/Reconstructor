//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTOR_SFIS_HPP
#define RECONSTRUCTOR_SFIS_HPP

#include <vector>
#include "Model.hpp"
#include "Camera.hpp"

namespace sfs {
    enum MODE {
        NO_CORRECTION = 0, CORRECTION = 1
    };

    typedef struct BoundingBox {
        float3 minPoint;
        float3 maxPoint;
        float3 boundingBoxSize;
    } BoundingBox;

    class SfIS {
    public:
        std::vector<Model *> models;
        std::vector<Camera *> cameras;

        MODE mode;
        int iteration;

        Image *referImage;

        BoundingBox boundingBox;
        float expectedVoxelSize;
        bool selfAdaptive;

        /**
         *
         * @param voxelConfig model configuration
         * @param config reference image parameters
         */
        SfIS(const std::string &voxelConfig, const std::string &config);

        ~SfIS();

        /**
         *
         * @param configFile
         */
        void readConfig(const std::string &configFile);

        /**
         * Reconstruction in selected rendering mode (with/without correction).
         * @param renderMode
         */
        void render(sfs::MODE renderMode);

        /**
         *
         */
        void calculateReferContours();

        /**
         *
         * @param model
         * @param voxelSize
         * @return
         */
        Model *splitFromModel(Model *model, float voxelSize);

    private:
        /**
         *
         */
        void initializeSelfAdaptive();

        /**
         *
         */
        void renderNormal();

        /**
         *
         */
        void renderCorrection();

        /**
         *
         */
        void renderHierarchy();
    };
}

#endif //RECONSTRUCTOR_SFIS_HPP
