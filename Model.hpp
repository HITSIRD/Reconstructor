//
// Created by 闻永言 on 2021/9/22.
//

#ifndef RECONSTRUCTOR_MODEL_HPP
#define RECONSTRUCTOR_MODEL_HPP

#include "Voxel.hpp"
#include "Vertex.hpp"
#include "Triangle.hpp"
#include "Image.hpp"
#include "Camera.hpp"
#include "TermsArray.hpp"
#include "Uniform.hpp"
#include <unordered_map>

namespace sfs {
    enum ProjectionMode {
        NORMAL, FAST
    };

    class Model {
    public:
        int3 voxelSize;
        int3 vertexSize; // voxelSize + 1

        index_t numVoxels; // max voxel number 2^32 := 4.29e9
        float voxelInnerSize;
        float voxelRadius;
        error_t sfisError;
//    error_t false_positive;
//    error_t false_negative;

        Voxel *voxels; // voxel configuration in grid
        index_t numVertices;
        Vertex *vertices; // store the vertices in the grid

        index_t *occupiedConfig; // voxel number in every reconstruction pixel

        TermsArray *termsArray;
        Uniform *uniform;

//        std::vector<std::pair<index_t, index_t>> matches3D2D; // Pair<3d point index, reconstruction 2d point index>
//        std::vector<std::pair<index_t, index_t>>
//                matches2D2D; // Pair<reconstruction 2d point index, reference 2d point index>

        std::vector<std::vector<std::pair<float4, float2>>> matches3D2D;
        std::vector<std::vector<std::pair<float2, float2>>> matches2D2D;

        Model() {};

        /**
         *
         * @param _x
         * @param _y
         * @param _z
         * @param origin origin point coordinate
         * @param _voxelInnerSize size of single voxel
         */
        Model(index_t _x, index_t _y, index_t _z, float3 origin, float _voxelInnerSize);

        ~Model();

        /**
         *
         */
        void initialize();

        /**
         *
         * @param uniform
         */
        void setUniform(Uniform *uniform);

        /**
         * Reset all information, make all voxel be occupied and clear back projection information.
         */
        void reset();

        /**
         *
         */
        void localMinSearch();

        /**
         *
         * @param _vertices
         * @param _triangles
         */
        void convertPolygonMesh(std::vector<Vertex> &_vertices, std::vector<Triangle> &_triangles);

        /**
         *
         */
        void updateSfISError();

        /**
         *
         * @param mode
         */
        void backProjection(sfs::ProjectionMode mode);

        /**
         *
         */
        void visualHull() const;

        /**
         *
         * @param directory
         */
        void writeProjection(const std::string &directory) const;

        /**
         *
         * @param directory
         */
        void writeSIE(const std::string &directory) const;

        //     /**
        //      *
        //      */
        //     void write_contour_2d_points() const;

        /**
         * Propagate occupied configuration labels to child model divided by parent model.
         */
        void propagateLabels(Model *child) const;

        /**
         *
         */
        void correspond();

        /**
         *
         * @param directory
         */
        void getContourPoints(const std::string &directory);

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
        void backProjectionNormal();

        /**
         *
         */
        void backProjectionFast() const;

        /**
         *
         * @param index
         */
        void updateBoundInformation(index_t index) const;

        /**
         *
         * @param index
         */
        void updateBoundWithNeighbors(index_t index) const;

        /**
         *
         * @param index
         * @param direction
         * @return
         */
        bool isBound(index_t index, sfs::DirectionType direction) const;

        /**
         *
         * @param indexX
         * @param indexY
         * @param indexZ
         * @param direction
         * @return
         */
        bool isBound(index_t indexX, index_t indexY, index_t indexZ, sfs::DirectionType direction) const;

        /**
         *
         * @param v0
         * @param v1
         * @param v2
         * @param occupied
         */
        void drawTriangle(const Vertex &v0, const Vertex &v1, const Vertex &v2, bool occupied) const;

        /**
         *
         * @param c center world coordinate
         * @param occupied
         */
        void drawCircle(const float4 &c, bool occupied) const;

        /**
         *
         * @param voxelIndex
         */
        error_t getPartial(index_t voxelIndex) const;
    };
}

#endif //RECONSTRUCTOR_MODEL_HPP
