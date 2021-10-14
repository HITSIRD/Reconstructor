//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTER_SFIS_HPP
#define RECONSTRUCTER_SFIS_HPP

#include <vector>
#include "Model.hpp"
#include "Camera.hpp"

namespace SfS
{
    enum MODE
    {
        NO_CORRECTION = 0, CORRECTION = 1
    };
}

typedef struct BoundingBox
{
    float3 min_point;
    float3 max_point;
    float3 bounding_box_size;
} BoundingBox;

class SfIS
{
public:
    std::vector<Model *> models;
    std::vector<Camera *> cameras;

    int iteration;

    Image *refer_image;

    BoundingBox bounding_box;
    float initial_voxel_size;

    /**
     *
     * @param voxel_config model configuration
     * @param config reference image parameters
     */
    SfIS(const std::string &voxel_config, const std::string &config);

    ~SfIS();

    /**
     *
     * @param config_file
     */
    void read_config(const std::string &config_file);

    /**
     *
     */
    void render(SfS::MODE mode);

    /**
     *
     */
    void calculate_refer_contours();

private:
    /**
     *
     */
    void render_normal();

    /**
     *
     */
    void render_correct();

    /**
     *
     * @param model
     */
    void generate_child_grid(Model *model);

    /**
     *
     * @param parent
     * @param child
     * @param distance
     */
    void propagate_label_to_child(Model *parent, Model *child, float distance);
};

#endif //RECONSTRUCTER_SFIS_HPP
