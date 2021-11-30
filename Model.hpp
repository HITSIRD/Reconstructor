//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTER_MODEL_HPP
#define RECONSTRUCTER_MODEL_HPP

#include "Voxel.hpp"
#include "Vertex.hpp"
#include "Triangle.hpp"
#include "Image.hpp"
#include "Camera.hpp"
#include <vector>

namespace SfS
{
    enum ProjectionMode
    {
        NORMAL, FAST
    };
}

template<typename T1, typename T2> struct Pair
{
    T1 first;
    T2 second;

    Pair(T1 _first, T2 _second):first(_first), second(_second){}
};

class Uniform
{
public:
    std::vector<Camera *> *cameras;
    Image *refer_image;
    index_t num_pixel;

    Uniform();

    /**
     *
     */
    Uniform(index_t _num_pixel);

    ~Uniform();

    /**
     *
     * @param cameras_pointer
     */
    void set_cameras(std::vector<Camera *> *cameras_pointer);

    /**
     *
     * @param refer
     */
    void set_refer_image(Image *refer);
};

/**
 * Store back projection pixels index of each voxel.
 */
class TermsArray
{
public:
    const index_t term_buffer_size = 1 << 28; // 256 MB
    uint64_t capacity;
    uint64_t num_terms;
    index_t num_index; // number of index in indices, num_index = num_voxel + 1
    uint64_t *indices; // to slice terms of each voxel
    std::vector<index_t *> terms; // back projection pixels index of each voxel

    // temporal variations
    index_t buffer_index;
    index_t term_index;

    TermsArray(){};

    TermsArray(index_t num_voxel);

    ~TermsArray();

    /**
     * If a term buffer is full, get a new buffer.
     */
    void increase_buffer();

    /**
     * Reset indices and temporal variations indices.
     */
    void reset();
};

class Model
{
public:
    index_t x; // voxel number in x axis
    index_t y; // voxel number in y axis
    index_t z; // voxel number in z axis
    index_t vx; // vertex number in x axis, vx = x + 1
    index_t vy; // vertex number in y axis, vy = y + 1
    index_t vz; // vertex number in z axis, vz = z + 1

    index_t num_voxel; // max voxel number 2^32 := 4.29e9
    float voxel_size;
    float voxel_radius;
    error_t sfis_error;
//    error_t false_positive;
//    error_t false_negative;

    Voxel *voxels; // voxel configuration in grid
    index_t num_vertices;
    Vertex *vertices; // store the vertices in the grid

    //    Image *reconstruct_image;
    index_t *occupied_config; // voxel number in every reconstruction pixel

    TermsArray *terms_array;
    Uniform *uniform;

    std::vector<Pair<index_t, index_t>> matches_3d_2d; // Pair<3d point index, reconstruction 2d point index>
    std::vector<Pair<index_t, index_t>> matches_2d_2d; // Pair<reconstruction 2d point index, reference 2d point index>

    Model(){};

    /**
     *
     * @param _x
     * @param _y
     * @param _z
     * @param origin origin point coordinate
     * @param voxel_size
     */
    Model(index_t _x, index_t _y, index_t _z, float3 origin, float voxel_size);

    ~Model();

    /**
     *
     */
    void initialize();

    /**
     *
     * @param uniform
     */
    void set_uniform(Uniform *uniform);

    /**
     * Reset all information, make all voxel be occupied and clear back projection information.
     */
    void reset();

    /**
     *
     */
    void local_min_search();

    /**
     *
     * @param _vertices
     * @param _triangles
     */
    void convert_polygon_mesh(std::vector<Vertex> &_vertices, std::vector<Triangle> &_triangles) const;

    /**
     *
     */
    void calculate_error();

    /**
     *
     */
    void back_projection(SfS::ProjectionMode mode);

    /**
     *
     */
    void visual_hull() const;

    /**
     *
     */
    void write_projection(const std::string &directory) const;

    /**
     *
     */
    void write_sie(const std::string &directory) const;

    //     /**
    //      *
    //      */
    //     void write_contour_2d_points() const;

    /**
     * Propagate occupied configuration labels to child model divided by parent model.
     */
    void propagate_labels(Model *child) const;

    /**
     *
     */
    void correspond();

    /**
     *
     */
    void cut() const;
private:
    /**
     *
     */
    void culling();

    /**
     *
     */
    void back_projection_normal();

    /**
     *
     */
    void back_projection_fast() const;

    /**
     *
     * @param index
     */
    void update_bound(index_t index) const;

    /**
     *
     * @param index
     */
    void update_bound_with_neighbors(index_t index) const;

    /**
     *
     * @param index
     * @param direction
     * @return
     */
    bool is_bound(index_t index, SfS::DirectionType direction) const;

    /**
     *
     * @param index_x
     * @param index_y
     * @param index_z
     * @param direction
     * @return
     */
    bool is_bound(index_t index_x, index_t index_y, index_t index_z, SfS::DirectionType direction) const;

    /**
     *
     * @param v_0
     * @param v_1
     * @param v_2
     * @param occupied
     */
    void draw_triangle(const Vertex &v_0, const Vertex &v_1, const Vertex &v_2, bool occupied) const;

    /**
     *
     * @param c center world coordinate
     * @param occupied
     */
    void draw_circle(const float4 &c, bool occupied) const;

    /**
     *
     * @param voxel_index
     */
    error_t calculate_partial(index_t voxel_index) const;
};

#endif //RECONSTRUCTER_MODEL_HPP
