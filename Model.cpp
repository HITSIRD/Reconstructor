//
// Created by 闻永言 on 2021/9/22.
//

#include <iostream>
#include "Model.hpp"
#include "Util.hpp"
#include "iodata.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include <utility>
#include <vector>

using namespace std;
using namespace sfs;

int num_triangle = 0;

Model::Model(index_t _x, index_t _y, index_t _z, float3 origin, float _voxelInnerSize): uniform(nullptr),
        voxelInnerSize(_voxelInnerSize) {
    voxelSize << _x, _y, _z;
    vertexSize << _x + 1, _y + 1, _z + 1;
    numVoxels = voxelSize.x() * voxelSize.y() * voxelSize.z();
    numVertices = vertexSize.x() * vertexSize.y() * vertexSize.z();
    voxelRadius = voxelInnerSize * 0.5f;
    voxels = new Voxel[numVoxels];
    sfisError = 0;
    //    false_positive = 0;
    //    false_negative = 0;

    occupiedConfig = nullptr;
    vertices = new Vertex[numVertices];
    termsArray = new TermsArray(numVoxels);
    index_t vertex_index = 0;

    // calculate vertex coordinate
    for (index_t index_z = 0; index_z < vertexSize.z(); index_z++) {
        for (index_t index_x = 0; index_x < vertexSize.x(); index_x++) {
            for (index_t index_y = 0; index_y < vertexSize.y(); index_y++, vertex_index++) {
                vertices[vertex_index].position << origin.x() + (float)index_x * voxelInnerSize, origin.y() +
                        (float)index_y * voxelInnerSize, origin.z() + (float)index_z * voxelInnerSize, 1.0f;
            }
        }
    }
}

Model::~Model() {
    delete[] voxels;
    delete[] vertices;
    delete[] occupiedConfig;
    delete termsArray;
}

void Model::initialize() {
    occupiedConfig = new index_t[uniform->numPixels];

    // update voxel bound attribute
    for (index_t i = 0; i < numVoxels; i++) {
        updateBoundInformation(i);
    }
}

void Model::setUniform(Uniform *u) {
    uniform = u;
}

void Model::reset() {
    for (index_t i = 0; i < numVoxels; i++) {
        voxels[i].occupied = true;
        voxels[i].test = false;
        voxels[i].bound = 0;
    }

    termsArray->reset();
    for (index_t i = 0; i < uniform->numPixels; i++) {
        occupiedConfig[i] = 0;
    }

    matches3D2D.clear();
    matches2D2D.clear();
}

void Model::backProjection(sfs::ProjectionMode mode) {
    cout << "back projection..." << endl;
    switch (mode) {
        case NORMAL:
            backProjectionNormal();
            break;
        case FAST:
            backProjectionFast();
            break;
    }
}

void Model::backProjectionNormal() {
    for (index_t i = 0; i < uniform->numPixels; i++) {
        occupiedConfig[i] = 0;
    }
    termsArray->reset();
    index_t voxel_index = 0;

    // rasterize every plane of voxel
    for (int index_z = 0; index_z < voxelSize.z(); index_z++) {
        if (index_z % 10 == 0) {
            cout << "count: " << voxel_index << ", term array size: " << termsArray->numTerms << endl;
        }

        for (int index_x = 0; index_x < voxelSize.x(); index_x++) {
            for (int index_y = 0; index_y < voxelSize.y(); index_y++, voxel_index++) {
                index_t index0 = (index_z * vertexSize.x() + index_x) * vertexSize.y() + index_y;
                index_t index2 = index0 + vertexSize.y();
                index_t index4 = index0 + vertexSize.x() * vertexSize.y();
                index_t index6 = index4 + vertexSize.y();
                const Vertex &v0 = vertices[index0];
                const Vertex &v1 = vertices[index0 + 1];
                const Vertex &v2 = vertices[index2];
                const Vertex &v3 = vertices[index2 + 1];
                const Vertex &v4 = vertices[index4];
                const Vertex &v5 = vertices[index4 + 1];
                const Vertex &v6 = vertices[index6];
                const Vertex &v7 = vertices[index6 + 1];

                bool occupied = voxels[voxel_index].occupied;

                // top
                drawTriangle(v4, v6, v5, occupied);
                drawTriangle(v5, v6, v7, occupied);
                // bottom
                drawTriangle(v0, v1, v2, occupied);
                drawTriangle(v2, v1, v3, occupied);
                // front
                drawTriangle(v6, v2, v7, occupied);
                drawTriangle(v7, v2, v3, occupied);
                // back
                drawTriangle(v4, v5, v0, occupied);
                drawTriangle(v5, v1, v0, occupied);
                // left
                drawTriangle(v4, v0, v6, occupied);
                drawTriangle(v6, v0, v2, occupied);
                // right
                drawTriangle(v7, v3, v5, occupied);
                drawTriangle(v5, v3, v1, occupied);

                termsArray->indices[voxel_index + 1] = termsArray->numTerms; // record index
            }
        }
    }

    cout << "term array size: " << termsArray->numTerms << endl;
}

void Model::backProjectionFast() const {
    for (index_t i = 0; i < uniform->numPixels; i++) {
        occupiedConfig[i] = 0;
    }
    termsArray->reset();
    index_t voxel_index = 0;

    for (int index_z = 0; index_z < voxelSize.z(); index_z++) {
        if (index_z % 10 == 0) {
            cout << "count: " << voxel_index << ", term array size: " << termsArray->numTerms << endl;
        }

        for (int index_x = 0; index_x < voxelSize.x(); index_x++) {
            for (int index_y = 0; index_y < voxelSize.y(); index_y++, voxel_index++) {
                index_t index = (index_z * vertexSize.x() + index_x) * vertexSize.y() + index_y;
                drawCircle(vertices[index].position + float4(voxelRadius, voxelRadius, voxelRadius, 0),
                        voxels[voxel_index].occupied);
                termsArray->indices[voxel_index + 1] = termsArray->numTerms; // record index
            }
        }
    }

    cout << "term array size: " << termsArray->numTerms << endl;
}

void Model::localMinSearch() {
    cout << "local minimum search..." << endl;
    //    culling();
    //    calculate_error();
    //    cout << "sfis error: " << sfis_error / 127 << endl;

    int label_changed = 1;
    int num_to_test = 0;
    int num_occupied = 0;
    int turn = 1;

    //    for (index_t i = 0; i < num_voxel; i++)
    //    {
    //        update_bound(i);
    //    }
    for (index_t i = 0; i < numVoxels; i++) {
        if (voxels[i].bound ^ BOUND) {
            if (voxels[i].bound != 0) {
                voxels[i].test = true;
            } else // single voxel which without neighbors, make it not be tested
            {
                if (!voxels[i].occupied) {
                    voxels[i].test = false;
                } else {
                    voxels[i].test = true;
                }
            }
        } else {
            voxels[i].test = false;
        }
    }

    for (index_t i = 0; i < numVoxels; i++) {
        if (voxels[i].test) {
            num_to_test++;
        }
        if (voxels[i].occupied) {
            num_occupied++;
        }
    }
    cout << "number to test is " << num_to_test << endl;
    cout << "number of occupied is " << num_occupied << endl;

    while (true) {
        cout << "turn " << turn << ": number to test is " << num_to_test << ", " << "number of occupied is "
                << num_occupied << ", ";
        label_changed = 0;
        turn++;

        int count = 0;
        for (index_t i = 0; i < numVoxels; i++) {
            if (voxels[i].test) {
                error_t partial = getPartial(i);
                bool condition_0 = partial < 0 && voxels[i].occupied;
                bool condition_1 = partial >= 0 && (!voxels[i].occupied);
                if (condition_0 || condition_1) {
                    uint64_t start_index = termsArray->indices[i];
                    uint64_t end_index = termsArray->indices[i + 1];
                    index_t buffer_index = start_index / termsArray->termBufferSize;
                    index_t term_index = start_index % termsArray->termBufferSize;

                    if (voxels[i].occupied) {
                        voxels[i].occupied = false;
                        num_occupied--;
                        for (; start_index != end_index; start_index++, term_index++) {
                            if (term_index == termsArray->termBufferSize) {
                                term_index = 0;
                                buffer_index++;
                            }
                            index_t index = termsArray->terms[buffer_index][term_index];
                            occupiedConfig[index]--;
                        }
                    } else {
                        voxels[i].occupied = true;
                        num_occupied++;
                        for (; start_index != end_index; start_index++, term_index++) {
                            if (term_index == termsArray->termBufferSize) {
                                term_index = 0;
                                buffer_index++;
                            }
                            index_t index = termsArray->terms[buffer_index][term_index];
                            occupiedConfig[index]++;
                        }
                    }

                    label_changed++;
                    sfisError -= abs(partial);
                }
            }
        }

        //        if (last_turn_error <= sfis_error)
        //        {
        //            cout << "label changed: " << label_changed << ", current sfis error: " << sfis_error / 127 << endl;
        //            break;
        //        }
        //        last_turn_error = sfis_error;
        if (label_changed == 0) {
            cout << "label changed: " << label_changed << ", current sfis error: " << sfisError / 127 << endl;
            break;
        }

        for (index_t i = 0; i < numVoxels; i++) {
            updateBoundInformation(i);
        }
        for (index_t i = 0; i < numVoxels; i++) {
            if (voxels[i].bound ^ BOUND) {
                if (voxels[i].bound != 0) {
                    if (!voxels[i].test) {
                        num_to_test++;
                    }
                    voxels[i].test = true;
                } else // single voxel which is without neighbors, make it not be tested
                {
                    if (!voxels[i].occupied) {
                        if (voxels[i].test) {
                            num_to_test--;
                        }
                        voxels[i].test = false;
                    } else // should test if is occupied
                    {
                        if (!voxels[i].test) {
                            num_to_test++;
                        }
                        voxels[i].test = true;
                    }
                }
            } else {
                if (voxels[i].test) {
                    num_to_test--;
                }
                voxels[i].test = false;
            }
        }
        cout << "label changed: " << label_changed << ", current sfis error: " << sfisError / 127 << endl;
    }

    //    cout << "fill..." << endl;
    //    for (index_t t = 0; t < num_voxel; t++)
    //    {
    //        if (calculate_partial(t) == 0 && !voxels[t].occupied)
    //        {
    //            voxels[t].occupied = true;
    //            num_occupied++;
    //        }
    //    }
    //    cout << "occupied: " << num_occupied << endl;
    //
    //    turn = 0;
    //    int change = 1;
    //    for (index_t i = 0; i < num_voxel; i++)
    //    {
    //        update_bound(i);
    //    }
    //    while (change != 0)
    //    {
    //        change = 0;
    //        turn++;
    //
    //        for (index_t t = 0; t < num_voxel; t++)
    //        {
    //            if (!voxels[t].occupied && voxels[t].bound > 0)
    //            {
    //                int b = 0;
    //                b += (voxels[t].bound & FRONT) > 0;
    //                b += (voxels[t].bound & BACK) > 0;
    //                b += (voxels[t].bound & LEFT) > 0;
    //                b += (voxels[t].bound & RIGHT) > 0;
    //                b += (voxels[t].bound & TOP) > 0;
    //                b += (voxels[t].bound & BOTTOM) > 0;
    //                if (b >= 5)
    //                {
    //                    voxels[t].occupied = true;
    //                    num_occupied++;
    //                    change++;
    //                }
    //            }
    //        }
    //
    //        cout << "turn " << turn << ": " << change << ", occupied: " << num_occupied << endl;
    //
    //        // update
    //        for (index_t i = 0; i < num_voxel; i++)
    //        {
    //            update_bound(i);
    //        }
    //    }
}

void Model::visualHull() const {
    cout << "visual hull..." << endl;

    for (index_t i = 0; i < numVoxels; i++) {
        bool some_positive = false;
        bool some_negative = false;
        uint64_t start_index = termsArray->indices[i];
        uint64_t end_index = termsArray->indices[i + 1];
        index_t buffer_index = start_index / termsArray->termBufferSize;
        index_t term_index = start_index % termsArray->termBufferSize;

        for (; start_index != end_index; start_index++, term_index++) {
            if (term_index == termsArray->termBufferSize) {
                term_index = 0;
                buffer_index++;
            }
            index_t index = termsArray->terms[buffer_index][term_index];

            if (uniform->referImage->coefficients[index] > 0) {
                some_positive = true;
            } else {
                some_negative = true;
            }

            if (some_positive && some_negative) {
                voxels[i].test = true;
                break;
            }
        }

        // voxel is always out of silhouette
        if (!some_positive) {
            voxels[i].occupied = false;

            // update terms
            start_index = termsArray->indices[i];
            end_index = termsArray->indices[i + 1];
            buffer_index = start_index / termsArray->termBufferSize;
            term_index = start_index % termsArray->termBufferSize;

            for (uint64_t j = start_index; j < end_index; j++, term_index++) {
                if (term_index == termsArray->termBufferSize) {
                    term_index = 0;
                    buffer_index++;
                }
                occupiedConfig[termsArray->terms[buffer_index][term_index]]--;
            }
        }
    }

    // update
    //    for (index_t i = 0; i < num_voxel; i++)
    //    {
    //        update_bound(i);
    //    }
    //    for (index_t i = 0; i < num_voxel; i++)
    //    {
    //        if (voxels[i].bound ^ BOUND)
    //        {
    //            if (voxels[i].bound != 0)
    //            {
    //                voxels[i].test = true;
    //            } else // single voxel which without neighbors, make it not be tested
    //            {
    //                if (!voxels[i].occupied)
    //                {
    //                    voxels[i].test = false;
    //                } else
    //                {
    //                    voxels[i].test = true;
    //                }
    //            }
    //        } else
    //        {
    //            voxels[i].test = false;
    //        }
    //    }
}

void Model::culling() {
    cout << "culling..." << endl;

    index_t image_size = uniform->referImage->x * uniform->referImage->y;
    for (index_t i = 0; i < numVoxels; i++) {
        if (voxels[i].occupied && voxels[i].test) {
            bool all_negative = true;
            uint64_t start_index = termsArray->indices[i];
            uint64_t end_index = termsArray->indices[i + 1];
            index_t buffer_index = start_index / termsArray->termBufferSize;
            index_t term_index = start_index % termsArray->termBufferSize;

            if (term_index == termsArray->termBufferSize) {
                term_index = 0;
                buffer_index++;
            }
            index_t previous_index = termsArray->terms[buffer_index][term_index];
            index_t previous_image_index = previous_index / image_size;

            for (; start_index != end_index; start_index++, term_index++) {
                if (term_index == termsArray->termBufferSize) {
                    term_index = 0;
                    buffer_index++;
                }
                index_t index = termsArray->terms[buffer_index][term_index];
                index_t image_index = index / image_size;

                if (previous_image_index == image_index) {
                    if (uniform->referImage->coefficients[index] > 0) {
                        all_negative = false;
                    }
                } else {
                    previous_image_index = image_index;
                    if (all_negative) {
                        break;
                    } else {
                        all_negative = true;
                    }

                    if (uniform->referImage->coefficients[index] > 0) {
                        all_negative = false;
                    }
                }
            }

            if (all_negative) // voxel is out of silhouette in an image
            {
                //                voxels[i].test = false;
                voxels[i].occupied = false;

                // update terms
                start_index = termsArray->indices[i];
                end_index = termsArray->indices[i + 1];
                buffer_index = start_index / termsArray->termBufferSize;
                term_index = start_index % termsArray->termBufferSize;

                for (uint64_t j = start_index; j < end_index; j++, term_index++) {
                    if (term_index == termsArray->termBufferSize) {
                        term_index = 0;
                        buffer_index++;
                    }
                    occupiedConfig[termsArray->terms[buffer_index][term_index]]--;
                }
            }
        }
    }

    for (index_t i = 0; i < numVoxels; i++) {
        updateBoundInformation(i);
    }
    for (index_t i = 0; i < numVoxels; i++) {
        if (voxels[i].bound ^ BOUND) {
            if (voxels[i].bound != 0) {
                voxels[i].test = true;
            } else // single voxel which is without neighbors, make it not be tested
            {
                if (!voxels[i].occupied) {
                    voxels[i].test = false;
                } else // should test if is occupied
                {
                    voxels[i].test = true;
                }
            }
        } else {
            voxels[i].test = false;
        }
    }
}

void Model::cut() const {
    for (index_t i = 0; i < numVoxels; i++) {
        if (voxels[i].occupied && (voxels[i].bound ^ BOUND)) {
            voxels[i].occupied = false;
        }
    }
}

void Model::convertPolygonMesh(vector<Vertex> &_vertices, vector<Triangle> &_triangles) {
    int numHull = 0;
    auto *outputIndex = new index_t[numVertices];
    index_t max = 1 << 31;
    for (index_t i = 0; i < numVertices; i++) {
        outputIndex[i] = max; // means empty
    }
    index_t voxelIndex = 0;
    index_t currentIndex = 0;

    // update bound information
    for (index_t i = 0; i < numVoxels; i++) {
        updateBoundInformation(i);
    }

    for (index_t index_z = 0; index_z < voxelSize.z(); index_z++) {
        for (index_t index_x = 0; index_x < voxelSize.x(); index_x++) {
            for (index_t index_y = 0; index_y < voxelSize.y(); index_y++, voxelIndex++) {
                if (voxels[voxelIndex].occupied && voxels[voxelIndex].bound < BOUND) {
                    numHull++;
                    index_t v0 = (index_z * vertexSize.x() + index_x) * vertexSize.y() + index_y;
                    index_t v1 = v0 + 1;
                    index_t v2 = v0 + vertexSize.y();
                    index_t v3 = v2 + 1;
                    index_t v4 = v0 + vertexSize.x() * vertexSize.y();
                    index_t v5 = v4 + 1;
                    index_t v6 = v4 + vertexSize.y();
                    index_t v7 = v6 + 1;

                    if (!(voxels[voxelIndex].bound & FRONT)) {
                        if (outputIndex[v2] == max) {
                            outputIndex[v2] = currentIndex;
                            _vertices.push_back(vertices[v2]);
                            currentIndex++;
                        }
                        if (outputIndex[v3] == max) {
                            outputIndex[v3] = currentIndex;
                            _vertices.push_back(vertices[v3]);
                            currentIndex++;
                        }
                        if (outputIndex[v6] == max) {
                            outputIndex[v6] = currentIndex;
                            _vertices.push_back(vertices[v6]);
                            currentIndex++;
                        }
                        if (outputIndex[v7] == max) {
                            outputIndex[v7] = currentIndex;
                            _vertices.push_back(vertices[v7]);
                            currentIndex++;
                        }
                        _triangles.emplace_back(outputIndex[v6], outputIndex[v2], outputIndex[v7]);
                        _triangles.emplace_back(outputIndex[v7], outputIndex[v2], outputIndex[v3]);
                    }
                    if (!(voxels[voxelIndex].bound & BACK)) {
                        if (outputIndex[v0] == max) {
                            outputIndex[v0] = currentIndex;
                            _vertices.push_back(vertices[v0]);
                            currentIndex++;
                        }
                        if (outputIndex[v1] == max) {
                            outputIndex[v1] = currentIndex;
                            _vertices.push_back(vertices[v1]);
                            currentIndex++;
                        }
                        if (outputIndex[v4] == max) {
                            outputIndex[v4] = currentIndex;
                            _vertices.push_back(vertices[v4]);
                            currentIndex++;
                        }
                        if (outputIndex[v5] == max) {
                            outputIndex[v5] = currentIndex;
                            _vertices.push_back(vertices[v5]);
                            currentIndex++;
                        }
                        _triangles.emplace_back(outputIndex[v4], outputIndex[v5], outputIndex[v0]);
                        _triangles.emplace_back(outputIndex[v5], outputIndex[v1], outputIndex[v0]);
                    }
                    if (!(voxels[voxelIndex].bound & LEFT)) {
                        if (outputIndex[v0] == max) {
                            outputIndex[v0] = currentIndex;
                            _vertices.push_back(vertices[v0]);
                            currentIndex++;
                        }
                        if (outputIndex[v2] == max) {
                            outputIndex[v2] = currentIndex;
                            _vertices.push_back(vertices[v2]);
                            currentIndex++;
                        }
                        if (outputIndex[v4] == max) {
                            outputIndex[v4] = currentIndex;
                            _vertices.push_back(vertices[v4]);
                            currentIndex++;
                        }
                        if (outputIndex[v6] == max) {
                            outputIndex[v6] = currentIndex;
                            _vertices.push_back(vertices[v6]);
                            currentIndex++;
                        }
                        _triangles.emplace_back(outputIndex[v4], outputIndex[v0], outputIndex[v6]);
                        _triangles.emplace_back(outputIndex[v6], outputIndex[v0], outputIndex[v2]);
                    }
                    if (!(voxels[voxelIndex].bound & RIGHT)) {
                        if (outputIndex[v1] == max) {
                            outputIndex[v1] = currentIndex;
                            _vertices.push_back(vertices[v1]);
                            currentIndex++;
                        }
                        if (outputIndex[v3] == max) {
                            outputIndex[v3] = currentIndex;
                            _vertices.push_back(vertices[v3]);
                            currentIndex++;
                        }
                        if (outputIndex[v5] == max) {
                            outputIndex[v5] = currentIndex;
                            _vertices.push_back(vertices[v5]);
                            currentIndex++;
                        }
                        if (outputIndex[v7] == max) {
                            outputIndex[v7] = currentIndex;
                            _vertices.push_back(vertices[v7]);
                            currentIndex++;
                        }
                        _triangles.emplace_back(outputIndex[v7], outputIndex[v3], outputIndex[v5]);
                        _triangles.emplace_back(outputIndex[v5], outputIndex[v3], outputIndex[v1]);
                    }
                    if (!(voxels[voxelIndex].bound & TOP)) {
                        if (outputIndex[v4] == max) {
                            outputIndex[v4] = currentIndex;
                            _vertices.push_back(vertices[v4]);
                            currentIndex++;
                        }
                        if (outputIndex[v5] == max) {
                            outputIndex[v5] = currentIndex;
                            _vertices.push_back(vertices[v5]);
                            currentIndex++;
                        }
                        if (outputIndex[v6] == max) {
                            outputIndex[v6] = currentIndex;
                            _vertices.push_back(vertices[v6]);
                            currentIndex++;
                        }
                        if (outputIndex[v7] == max) {
                            outputIndex[v7] = currentIndex;
                            _vertices.push_back(vertices[v7]);
                            currentIndex++;
                        }
                        _triangles.emplace_back(outputIndex[v4], outputIndex[v6], outputIndex[v5]);
                        _triangles.emplace_back(outputIndex[v5], outputIndex[v6], outputIndex[v7]);
                    }
                    if (!(voxels[voxelIndex].bound & BOTTOM)) {
                        if (outputIndex[v0] == max) {
                            outputIndex[v0] = currentIndex;
                            _vertices.push_back(vertices[v0]);
                            currentIndex++;
                        }
                        if (outputIndex[v1] == max) {
                            outputIndex[v1] = currentIndex;
                            _vertices.push_back(vertices[v1]);
                            currentIndex++;
                        }
                        if (outputIndex[v2] == max) {
                            outputIndex[v2] = currentIndex;
                            _vertices.push_back(vertices[v2]);
                            currentIndex++;
                        }
                        if (outputIndex[v3] == max) {
                            outputIndex[v3] = currentIndex;
                            _vertices.push_back(vertices[v3]);
                            currentIndex++;
                        }
                        _triangles.emplace_back(outputIndex[v0], outputIndex[v1], outputIndex[v2]);
                        _triangles.emplace_back(outputIndex[v2], outputIndex[v1], outputIndex[v3]);
                    }
                }
            }
        }
    }

    delete[] outputIndex;
    cout << "hull voxel number: " << numHull << endl;
}

void Model::updateBoundInformation(index_t index) const {
    index_t indexXY = index % (voxelSize.x() * voxelSize.y());
    index_t indexZ = index / (voxelSize.x() * voxelSize.y());
    if (indexXY / voxelSize.y() == voxelSize.x() - 1) {
        voxels[index].bound &= MASK_FRONT;
    } else {
        if (voxels[index + voxelSize.y()].occupied) {
            voxels[index].bound |= FRONT;
        } else {
            voxels[index].bound &= MASK_FRONT;
        }
    }
    if (indexXY / voxelSize.y() == 0) {
        voxels[index].bound &= MASK_BACK;
    } else {
        if (voxels[index - voxelSize.y()].occupied) {
            voxels[index].bound |= BACK;
        } else {
            voxels[index].bound &= MASK_BACK;
        }
    }
    if (indexXY % voxelSize.y() == 0) {
        voxels[index].bound &= MASK_LEFT;
    } else {
        if (voxels[index - 1].occupied) {
            voxels[index].bound |= LEFT;
        } else {
            voxels[index].bound &= MASK_LEFT;
        }
    }
    if (indexXY % voxelSize.y() == voxelSize.y() - 1) {
        voxels[index].bound &= MASK_RIGHT;
    } else {
        if (voxels[index + 1].occupied) {
            voxels[index].bound |= RIGHT;
        } else {
            voxels[index].bound &= MASK_RIGHT;
        }
    }
    if (indexZ == voxelSize.z() - 1) {
        voxels[index].bound &= MASK_TOP;
    } else {
        if (voxels[index + voxelSize.x() * voxelSize.y()].occupied) {
            voxels[index].bound |= TOP;
        } else {
            voxels[index].bound &= MASK_TOP;
        }
    }
    if (indexZ == 0) {
        voxels[index].bound &= MASK_BOTTOM;
    } else {
        if (voxels[index - voxelSize.x() * voxelSize.y()].occupied) {
            voxels[index].bound |= BOTTOM;
        } else {
            voxels[index].bound &= MASK_BOTTOM;
        }
    }
}

bool Model::isBound(index_t index, DirectionType direction) const {
    index_t index_xy = index % (voxelSize.x() * voxelSize.y());
    index_t index_z = index / (voxelSize.x() * voxelSize.y());
    switch (direction) {
        case FRONT:
            return (index_xy / voxelSize.y() == voxelSize.x() - 1) || (!voxels[index + voxelSize.y()].occupied);
        case BACK:
            return (index_xy / voxelSize.y() == 0) || (!voxels[index - voxelSize.y()].occupied);
        case LEFT:
            return (index_xy % voxelSize.y() == 0) || (!voxels[index - 1].occupied);
        case RIGHT:
            return (index_xy % voxelSize.y() == voxelSize.y() - 1) || (!voxels[index + 1].occupied);
        case TOP:
            return (index_z == voxelSize.z() - 1) || (!voxels[index + voxelSize.x() * voxelSize.y()].occupied);
        case BOTTOM:
            return (index_z == 0) || (!voxels[index - voxelSize.x() * voxelSize.y()].occupied);
    }
}

bool Model::isBound(index_t indexX, index_t indexY, index_t indexZ, DirectionType direction) const {
    index_t index = (indexZ * voxelSize.x() + indexX) * voxelSize.y() + indexY;
    switch (direction) {
        case FRONT:
            return (indexX == voxelSize.x() - 1) || (!voxels[index + voxelSize.y()].occupied);
        case BACK:
            return (indexX == 0) || (!voxels[index - voxelSize.y()].occupied);
        case LEFT:
            return (indexY == 0) || (!voxels[index - 1].occupied);
        case RIGHT:
            return (indexY == voxelSize.y() - 1) || (!voxels[index + 1].occupied);
        case TOP:
            return (indexZ == voxelSize.z() - 1) || (!voxels[index + voxelSize.x() * voxelSize.y()].occupied);
        case BOTTOM:
            return (indexZ == 0) || (!voxels[index - voxelSize.x() * voxelSize.y()].occupied);
    }
}

void Model::drawTriangle(const Vertex &v0, const Vertex &v1, const Vertex &v2, bool occupied) const {
    int camera_order = 0;
    num_triangle++;

    for (const auto camera: *uniform->cameras) {
        index_t offset = camera_order * camera->x * camera->y;
        camera_order++;

        float4 screen_0 = camera->matrixWorldToScreen * v0.position;
        float4 screen_1 = camera->matrixWorldToScreen * v1.position;
        float4 screen_2 = camera->matrixWorldToScreen * v2.position;
        screen_0 = screen_0 / screen_0.w();
        screen_1 = screen_1 / screen_1.w();
        screen_2 = screen_2 / screen_2.w();

        // back face culling
        float AB_x = screen_1.x() - screen_0.x();
        float AB_y = screen_1.y() - screen_0.y();
        float AC_x = screen_2.x() - screen_0.x();
        float AC_y = screen_2.y() - screen_0.y();
        if (AB_x * AC_y - AB_y * AC_x > 0) {
            continue;
        }

        // real bounding box
        int min_x = max((int)min(screen_0.x(), min(screen_1.x(), screen_2.x())), 0);
        int min_y = max((int)min(screen_0.y(), min(screen_1.y(), screen_2.y())), 0);
        int max_x = min((int)max(screen_0.x(), max(screen_1.x(), screen_2.x())) + 1, camera->x - 1);
        int max_y = min((int)max(screen_0.y(), max(screen_1.y(), screen_2.y())) + 1, camera->y - 1);

        // rasterization
        for (int i = min_y; i <= max_y; i++) {
            for (int j = min_x; j <= max_x; j++) {
                float center_x = float(j) + 0.5f;
                float center_y = float(i) + 0.5f;
                float v0x = screen_0.x() - center_x;
                float v0y = screen_0.y() - center_y;
                float v1x = screen_1.x() - center_x;
                float v1y = screen_1.y() - center_y;
                float v2x = screen_2.x() - center_x;
                float v2y = screen_2.y() - center_y;
                float AB = v1x * v0y - v1y * v0x;
                float BC = v2x * v1y - v2y * v1x;
                float CA = v0x * v2y - v0y * v2x;
                if (AB > 0 && BC > 0 && CA > 0) {
                    if (is_in_triangle(AB, BC, CA)) {
                        index_t term_index = offset + i * camera->x + j;
                        if (occupied) {
                            occupiedConfig[term_index]++;
                        }
                        if (termsArray->capacity == termsArray->numTerms) {
                            termsArray->increaseBuffer();
                        }

                        termsArray->terms[termsArray->bufferIndex][termsArray->termIndex] = term_index;
                        termsArray->numTerms++;
                        termsArray->termIndex++;
                    }
                }
            }
        }
    }
}

void Model::drawCircle(const float4 &c, bool occupied) const {
    index_t offset = 0;
    num_triangle++;

    for (const auto &camera: *uniform->cameras) {
        float4 edge = camera->matrixView * c;
        edge.x() += voxelRadius;
        float4 center = camera->matrixWorldToScreen * c;
        center = center / center.w();
        edge = camera->matrixPV * edge;
        edge = edge / edge.w();
        float r = edge.x() - center.x();
        //        r = r < 0.5f ? 0.5f : r;
        float r2 = r * r;

        // real bounding box
        int min_x = max((int)(center.x() - r), 0);
        int min_y = max((int)(center.y() - r), 0);
        int max_x = min((int)(center.x() + r) + 1, camera->x - 1);
        int max_y = min((int)(center.y() + r) + 1, camera->y - 1);

        float x0 = float(min_x) + 0.5f;
        float y0 = float(min_y) + 0.5f;

        // rasterization
        float p_y = y0;
        index_t term_index = offset + min_y * camera->x;
        for (int i = min_y; i <= max_y; i++) {
            float p_x = x0;
            index_t index = term_index + min_x;

            for (int j = min_x; j <= max_x; j++) {
                float dx = center.x() - p_x;
                float dy = center.y() - p_y;

                if (dx * dx + dy * dy < r2) {
                    if (occupied) {
                        occupiedConfig[index]++;
                    }
                    if (termsArray->capacity == termsArray->numTerms) {
                        termsArray->increaseBuffer();
                    }

                    termsArray->terms[termsArray->bufferIndex][termsArray->termIndex] = index;
                    termsArray->numTerms++;
                    termsArray->termIndex++;
                }
                p_x += 1.0f;
                index++;
            }
            p_y += 1.0f;
            term_index += camera->x;
        }
        offset += camera->x * camera->y;
    }
}

void Model::updateSfISError() {
    sfisError = 0;

    for (index_t i = 0; i < uniform->numPixels; i++) {
        sfisError += ((255 - uniform->referImage->data[i]) / 2);
        if (occupiedConfig[i] == 0) {
            sfisError += uniform->referImage->coefficients[i];
        }
    }
}

error_t Model::getPartial(index_t voxelIndex) const {
    uint64_t start_index = termsArray->indices[voxelIndex];
    uint64_t end_index = termsArray->indices[voxelIndex + 1];
    index_t buffer_index = start_index / termsArray->termBufferSize;
    index_t term_index = start_index % termsArray->termBufferSize;

    error_t partial = 0;

    if (voxels[voxelIndex].occupied) {
        for (; start_index != end_index; start_index++, term_index++) {
            if (term_index == termsArray->termBufferSize) {
                term_index = 0;
                buffer_index++;
            }
            index_t index = termsArray->terms[buffer_index][term_index];

            if (occupiedConfig[index] == 1) {
                partial += uniform->referImage->coefficients[index];
            }
        }
    } else {
        for (; start_index != end_index; start_index++, term_index++) {
            if (term_index == termsArray->termBufferSize) {
                term_index = 0;
                buffer_index++;
            }
            index_t index = termsArray->terms[buffer_index][term_index];

            if (occupiedConfig[index] == 0) {
                partial += uniform->referImage->coefficients[index];
            }
        }
    }

    return partial;
}

void Model::writeProjection(const string &directory) const {
    Image image = Image(uniform->referImage->x, uniform->referImage->y, uniform->referImage->numCameras);

    //    for (index_t i = 0; i < uniform->num_pixel; i++)
    //    {
    //        image.data[i] = occupied_config[i] > 0 ? 255 : 0;
    //    }
    for (index_t i = 0; i < numVoxels; i++) {
        if (voxels[i].occupied) {
            uint64_t start_index = termsArray->indices[i];
            uint64_t end_index = termsArray->indices[i + 1];
            index_t buffer_index = start_index / termsArray->termBufferSize;
            index_t term_index = start_index % termsArray->termBufferSize;
            for (; start_index != end_index; start_index++, term_index++) {
                if (term_index == termsArray->termBufferSize) {
                    term_index = 0;
                    buffer_index++;
                }
                index_t index = termsArray->terms[buffer_index][term_index];
                image.data[index] = 255;
            }
        }
    }
    iodata::writeProjection(directory, image);
}

void Model::writeSIE(const string &directory) const {
    Image image = Image(uniform->referImage->x, uniform->referImage->y, uniform->referImage->numCameras);

    for (index_t i = 0; i < uniform->numPixels; i++) {
        image.data[i] = occupiedConfig[i] > 0 ? 255 : 0;
    }
    iodata::writeSie(directory, image, *uniform->referImage);
}

void Model::propagateLabels(Model *child) const {
    cout << "propagate labels..." << endl;
    assert(child->voxelSize.x() / voxelSize.x() == 2 && child->voxelSize.y() / voxelSize.y() == 2 &&
            child->voxelSize.z() / voxelSize.z() == 2);
    index_t voxel_index = 0;
    bool padding_x = child->voxelSize.x() % voxelSize.x() == 0;
    bool padding_y = child->voxelSize.y() % voxelSize.y() == 0;
    bool padding_z = child->voxelSize.z() % voxelSize.z() == 0;

    for (int i = 0; i < child->numVoxels; i++) {
        child->voxels[i].occupied = false;
    }

    for (int index_z = 0; index_z < child->voxelSize.z(); index_z++) {
        index_t parent_z = index_z / 2;
        bool edge_z = (index_z != child->voxelSize.z() - 1) || padding_z;
        for (int index_x = 0; index_x < child->voxelSize.x(); index_x++) {
            index_t parent_x = index_x / 2;
            bool edge_x = (index_x != child->voxelSize.x() - 1) || padding_x;
            for (int index_y = 0; index_y < child->voxelSize.y(); index_y++, voxel_index++) {
                bool edge_y = (index_y != child->voxelSize.y() - 1) || padding_y;
                if (edge_z && edge_x && edge_y) {
                    index_t parent_y = index_y / 2;
                    index_t parent_index = voxelSize.y() * (parent_z * voxelSize.x() + parent_x) + parent_y;
                    child->voxels[voxel_index].occupied = voxels[parent_index].occupied;
                }
            }
        }
    }
}

void Model::correspond() {
//    vector<index_t> all_points; // all 2d points
//    vector<index_t> refer_2d_points;
//    //    vector<index_t> recon_2d_points; // contour 2d points
//
//    for (auto matches: matches3D2D)
//    {
//        all_points.push_back(matches.second);
//    }
//    set<index_t> all_points_set(all_points.begin(), all_points.end()); // to remove same index
//    all_points.assign(all_points_set.begin(), all_points_set.end());
//
//    vector<vector<float2>> recon_points(uniform->referImage->numCameras); // to process 2D points in each image
//    index_t image_x = uniform->referImage->x;
//    index_t image_y = uniform->referImage->y;
//    index_t image_size = image_x * image_y;
//    for (index_t i = 0; i < uniform->referImage->numCameras; i++)
//    {
//        index_t start = i * image_size;
//        index_t end = start + image_size;
//        for (auto p: all_points)
//        {
//            if (p >= start && p < end)
//            {
//                index_t point_index = p - start;
//                float point_x = (float)(point_index % image_x) + 0.5f;
//                float point_y = (float)(point_index / image_x) + 0.5f;
//                recon_points[i].emplace_back(float2(point_x, point_y));
//            }
//        }
//    }
//
//    // calculate convex hull in each image
//    for (index_t i = 0; i < uniform->referImage->numCameras; i++)
//    {
//        vector<cv::Point2f> cv_points;
//        vector<cv::Point2f> hull;
//        for (const auto &p: recon_points[i])
//        {
//            cv_points.emplace_back(p.x(), p.y());
//        }
//        cv::convexHull(cv_points, hull);
//
//        recon_points[i].clear();
//        for (const auto &p: hull)
//        {
//            recon_points[i].emplace_back(float2(p.x, p.y));
//        }
//    }
//
//    // processing reference image contours
//    for (index_t index = 0; index < uniform->numPixels; index++)
//    {
//        if (uniform->referImage->contours[index])
//        {
//            refer_2d_points.push_back(index);
//        }
//    }
//
//    vector<vector<float2>> refer_points(uniform->referImage->numCameras);
//    for (index_t i = 0; i < uniform->referImage->numCameras; i++)
//    {
//        index_t start = i * image_size;
//        index_t end = start + image_size;
//        for (auto p: refer_2d_points)
//        {
//            if (p >= start && p < end)
//            {
//                index_t point_index = p - start;
//                float point_x = (float)(point_index % image_x) + 0.5f;
//                float point_y = (float)(point_index / image_x) + 0.5f;
//                refer_points[i].emplace_back(float2(point_x, point_y));
//            }
//        }
//    }
//
//    // correspond 2D-2D points
//    float max_distance = sqrtf((float)(image_x * image_x + image_y * image_y));
//    for (index_t i = 0; i < uniform->referImage->numCameras; i++)
//    {
//        index_t offset = i * image_size;
//        for (const auto &p1: recon_points[i])
//        {
//            float distance = max_distance;
//            float2 near;
//            for (const auto &p2: refer_points[i])
//            {
//                float current_distance = (p2 - p1).lpNorm<2>();
//                if (current_distance < distance)
//                {
//                    distance = current_distance;
//                    near = p2;
//                    if (distance < 0.5) // distance is shorter than a pixel
//                    {
//                        break;
//                    }
//                }
//            }
//            matches2D2D.emplace_back(offset + (index_t)p1.y() * image_x + (index_t)p1.x(),
//                    offset + (index_t)near.y() * image_x + (index_t)near.x());
//        }
//    }
//
//    // update 2D-2D matches, remove pair that 2D point is not on the contour
//    bool *contours = new bool[uniform->numPixels];
//    for (auto &index: matches2D2D)
//    {
//        contours[index.first] = true;
//    }
//    vector<std::pair<index_t, index_t>> term = matches3D2D; // copy
//    matches3D2D.clear();
//    for (const auto &p: term)
//    {
//        if (contours[p.second])
//        {
//            matches3D2D.push_back(p);
//        }
//    }
//
//    delete[] contours;
//    cout << "3D-2D matches pair: " << matches3D2D.size() << endl;
//    cout << "2D-2D matches pair: " << matches2D2D.size() << endl;

    //    Image image = Image(uniform->refer_image->x, uniform->refer_image->y, uniform->refer_image->num_camera);
    //
    //    for (index = 0; index < recon_2d_points.size(); index++)
    //    {
    //        image.contours[recon_2d_points[index]] = true;
    //    }
    //    iodata::write_recon_contours(image);

    //        for (const auto &p: matches_2d_2d)
    //        {
    //            index_t recon_index = p.first % image_size;
    //            index_t recon_x = recon_index % image_x;
    //            index_t recon_y = recon_index / image_x;
    //
    //            index_t refer_index = p.second % image_size;
    //            index_t refer_x = refer_index % image_x;
    //            index_t refer_y = refer_index / image_x;
    //            cout << "recon [" << recon_x << ", " << recon_y << "], refer [" << refer_x << ", " << refer_y << "]" << endl;
    //        }
}

void Model::getContourPoints(const string &directory) {
    cout << "calculate reconstruction contour..." << endl;
    matches3D2D.clear();

    // update bound information
    for (index_t i = 0; i < numVoxels; i++) {
        updateBoundInformation(i);
    }

    auto *vertexBuffer = new char[numVertices]; // 0: out of contour, 1: on the contour, 2: in the contour
    for (const auto &camera: *uniform->cameras) {
        index_t voxelIndex = 0;
        vector<pair<float4, float2>> matches;

        for (index_t i = 0; i < numVertices; i++) {
            vertexBuffer[i] = 0; // means not in contour
        }

        for (index_t index_z = 0; index_z < voxelSize.z(); index_z++) {
            for (index_t index_x = 0; index_x < voxelSize.x(); index_x++) {
                for (index_t index_y = 0; index_y < voxelSize.y(); index_y++, voxelIndex++) {
                    if (voxels[voxelIndex].occupied && voxels[voxelIndex].bound < BOUND) {
                        index_t v0 = (index_z * vertexSize.x() + index_x) * vertexSize.y() + index_y;
                        index_t v1 = v0 + 1;
                        index_t v2 = v0 + vertexSize.y();
                        index_t v3 = v2 + 1;
                        index_t v4 = v0 + vertexSize.x() * vertexSize.y();
                        index_t v5 = v4 + 1;
                        index_t v6 = v4 + vertexSize.y();
                        index_t v7 = v6 + 1;

                        index_t vertexIndices[8] = {v0, v1, v2, v3, v4, v5, v6, v7};
                        float4 vertexPositions[8];
                        vector<cv::Point2f> cvPoints;
                        vector<cv::Point2f> hull;
                        int index = 0;

                        for (const auto p: vertexIndices) {
                            vertexPositions[index] = camera->matrixWorldToScreen * vertices[p].position;
                            vertexPositions[index] /= vertexPositions[index].w();
                            cvPoints.emplace_back(vertexPositions[index].x(), vertexPositions[index].y());
                            index++;
                        }
                        cv::convexHull(cvPoints, hull);

                        for (const auto &p: hull) {
                            for (int i = 0; i < 8; i++) {
                                if (norm(cvPoints[i] - p) < 0.0001) // on the contour
                                {
                                    if (vertexBuffer[vertexIndices[i]] == 0) {
                                        vertexBuffer[vertexIndices[i]] = 1;
                                        matches.emplace_back(vertices[vertexIndices[i]].position,
                                                float2(vertexPositions[i].x(), vertexPositions[i].y()));
                                    }
                                } else if (vertexBuffer[vertexIndices[i]] == 0 || vertexBuffer[vertexIndices[i]] == 1) {
                                    vertexBuffer[vertexIndices[i]] = 2;
                                }
                            }
                        }
                    }
                }
            }
        }
        matches3D2D.push_back(matches);
    }
    delete[] vertexBuffer;

    Image image = Image(uniform->referImage->x, uniform->referImage->y, uniform->referImage->numCameras);
    index_t imageSize = uniform->referImage->x * uniform->referImage->y;

    for (int index = 0; index < uniform->referImage->numCameras; index++) {
        index_t offset = index * imageSize;
        for (const auto &p: matches3D2D[index]) {
            int x = (int)p.second.x();
            int y = (int)p.second.y();
            if (x >= 0 && x < image.x && y >= 0 && y < image.y) {
                image.data[offset + y * image.x + x] = 255;
            }
        }
    }
    iodata::writeReconContours(directory, image);
}
