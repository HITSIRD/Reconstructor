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

using namespace std;
using namespace SfS;

int num_triangle = 0;

Uniform::Uniform():cameras(nullptr){}

Uniform::Uniform(index_t _num_pixel):num_pixel(_num_pixel){}

Uniform::~Uniform() = default;

void Uniform::set_cameras(std::vector<Camera *> *cameras_pointer)
{
    cameras = cameras_pointer;
}

void Uniform::set_refer_image(Image *refer)
{
    refer_image = refer;
}

TermsArray::TermsArray(index_t num_voxel)
{
    capacity = 0;
    num_terms = 0;
    buffer_index = 0;
    term_index = 0;
    num_index = num_voxel + 1; // +1 to store first voxel start index
    indices = new uint64_t[num_index];
}

TermsArray::~TermsArray()
{
    delete[] indices;
    for (auto buffer: terms)
    {
        delete[] buffer;
    }
}

void TermsArray::increase_buffer()
{
    terms.push_back(new index_t[term_buffer_size]);
    capacity += term_buffer_size;
    if (terms.size() != 1) // index start at 0
    {
        buffer_index++;
    }
    term_index = 0;
}

void TermsArray::reset()
{
    for (index_t i = 0; i < num_index; i++)
    {
        indices[i] = 0;
    }
    for (auto buffer: terms)
    {
        delete[] buffer;
    }
    terms.clear();

    capacity = 0;
    num_terms = 0;
    buffer_index = 0;
    term_index = 0;
}

Model::Model(index_t _x, index_t _y, index_t _z, float3 origin, float _voxel_size):x(_x), y(_y), z(_z), vx(_x + 1),
                                                                                   vy(_y + 1), vz(_z + 1),
                                                                                   uniform(nullptr)
{
    num_voxel = x * y * z;
    voxel_size = _voxel_size;
    voxel_radius = voxel_size * 0.5f;
    voxels = new Voxel[num_voxel];
    sfis_error = 0;
    //    false_positive = 0;
    //    false_negative = 0;

    num_vertices = vx * vy * vz;
    vertices = new Vertex[num_vertices];
    terms_array = new TermsArray(num_voxel);
    index_t vertex_index = 0;

    // calculate vertex coordinate
    for (index_t index_z = 0; index_z < vz; index_z++)
    {
        for (index_t index_x = 0; index_x < vx; index_x++)
        {
            for (index_t index_y = 0; index_y < vy; index_y++, vertex_index++)
            {
                vertices[vertex_index].world << origin.x() + (float)index_x * voxel_size, origin.y() +
                                                                                          (float)index_y * voxel_size,
                        origin.z() + (float)index_z * voxel_size, 1.0f;
            }
        }
    }
}

Model::~Model()
{
    delete[] voxels;
    delete[] vertices;
    delete[] occupied_config;
    delete terms_array;
}

void Model::initialize()
{
    occupied_config = new index_t[uniform->num_pixel];

    // update voxel bound attribute
    for (index_t i = 0; i < num_voxel; i++)
    {
        update_bound(i);
    }
}

void Model::set_uniform(Uniform *u)
{
    uniform = u;
}

void Model::reset()
{
    for (index_t i = 0; i < num_voxel; i++)
    {
        voxels[i].occupied = true;
        voxels[i].test = false;
        voxels[i].bound = 0;
    }
    //    sfis_error = 0;
    //    false_positive = 0;
    //    false_negative = 0;

    terms_array->reset();
    for (index_t i = 0; i < uniform->num_pixel; i++)
    {
        occupied_config[i] = 0;
    }

    matches_3d_2d.clear();
    matches_2d_2d.clear();
}

void Model::back_projection(SfS::ProjectionMode mode)
{
    cout << "back projection..." << endl;
    switch (mode)
    {
        case NORMAL:
            back_projection_normal();
            break;
        case FAST:
            back_projection_fast();
            break;
    }
}

void Model::back_projection_normal()
{
    for (index_t i = 0; i < uniform->num_pixel; i++)
    {
        occupied_config[i] = 0;
    }
    terms_array->reset();
    index_t voxel_index = 0;

    // rasterize every plane of voxel
    for (int index_z = 0; index_z < z; index_z++)
    {
        if (index_z % 10 == 0)
        {
            cout << "count: " << voxel_index << ", term array size: " << terms_array->num_terms << endl;
        }

        for (int index_x = 0; index_x < x; index_x++)
        {
            for (int index_y = 0; index_y < y; index_y++, voxel_index++)
            {
                index_t index_0 = (index_z * vx + index_x) * vy + index_y;
                index_t index_2 = index_0 + vy;
                index_t index_4 = index_0 + vx * vy;
                index_t index_6 = index_4 + vy;
                const Vertex &v_0 = vertices[index_0];
                const Vertex &v_1 = vertices[index_0 + 1];
                const Vertex &v_2 = vertices[index_2];
                const Vertex &v_3 = vertices[index_2 + 1];
                const Vertex &v_4 = vertices[index_4];
                const Vertex &v_5 = vertices[index_4 + 1];
                const Vertex &v_6 = vertices[index_6];
                const Vertex &v_7 = vertices[index_6 + 1];

                bool occupied = voxels[voxel_index].occupied;

                // top
                draw_triangle(v_4, v_6, v_5, occupied);
                draw_triangle(v_5, v_6, v_7, occupied);
                // bottom
                draw_triangle(v_0, v_1, v_2, occupied);
                draw_triangle(v_2, v_1, v_3, occupied);
                // front
                draw_triangle(v_6, v_2, v_7, occupied);
                draw_triangle(v_7, v_2, v_3, occupied);
                // back
                draw_triangle(v_4, v_5, v_0, occupied);
                draw_triangle(v_5, v_1, v_0, occupied);
                // left
                draw_triangle(v_4, v_0, v_6, occupied);
                draw_triangle(v_6, v_0, v_2, occupied);
                // right
                draw_triangle(v_7, v_3, v_5, occupied);
                draw_triangle(v_5, v_3, v_1, occupied);

                terms_array->indices[voxel_index + 1] = terms_array->num_terms; // record index
            }
        }
    }

    cout << "term array size: " << terms_array->num_terms << endl;
}

void Model::back_projection_fast() const
{
    for (index_t i = 0; i < uniform->num_pixel; i++)
    {
        occupied_config[i] = 0;
    }
    terms_array->reset();
    index_t voxel_index = 0;

    for (int index_z = 0; index_z < z; index_z++)
    {
        if (index_z % 10 == 0)
        {
            cout << "count: " << voxel_index << ", term array size: " << terms_array->num_terms << endl;
        }

        for (int index_x = 0; index_x < x; index_x++)
        {
            for (int index_y = 0; index_y < y; index_y++, voxel_index++)
            {
                index_t index = (index_z * vx + index_x) * vy + index_y;
                draw_circle(
                        vertices[index].world + float4(voxel_radius, voxel_radius, voxel_radius, 0),
                        voxels[voxel_index].occupied);
                terms_array->indices[voxel_index + 1] = terms_array->num_terms; // record index
            }
        }
    }

    cout << "term array size: " << terms_array->num_terms << endl;
}

void Model::local_min_search()
{
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
    for (index_t i = 0; i < num_voxel; i++)
    {
        if (voxels[i].bound ^ BOUND)
        {
            if (voxels[i].bound != 0)
            {
                voxels[i].test = true;
            } else // single voxel which without neighbors, make it not be tested
            {
                if (!voxels[i].occupied)
                {
                    voxels[i].test = false;
                } else
                {
                    voxels[i].test = true;
                }
            }
        } else
        {
            voxels[i].test = false;
        }
    }

    for (index_t i = 0; i < num_voxel; i++)
    {
        if (voxels[i].test)
        {
            num_to_test++;
        }
        if (voxels[i].occupied)
        {
            num_occupied++;
        }
    }
    cout << "number to test is " << num_to_test << endl;
    cout << "number of occupied is " << num_occupied << endl;

    while (true)
    {
        cout << "turn " << turn << ": number to test is " << num_to_test << ", " << "number of occupied is "
             << num_occupied << ", ";
        label_changed = 0;
        turn++;

        int count = 0;
        for (index_t i = 0; i < num_voxel; i++)
        {
            if (voxels[i].test)
            {
                error_t partial = calculate_partial(i);
                bool condition_0 = partial < 0 && voxels[i].occupied;
                bool condition_1 = partial >= 0 && (!voxels[i].occupied);
                if (condition_0 || condition_1)
                {
                    uint64_t start_index = terms_array->indices[i];
                    uint64_t end_index = terms_array->indices[i + 1];
                    index_t buffer_index = start_index / terms_array->term_buffer_size;
                    index_t term_index = start_index % terms_array->term_buffer_size;

                    if (voxels[i].occupied)
                    {
                        voxels[i].occupied = false;
                        num_occupied--;
                        for (; start_index != end_index; start_index++, term_index++)
                        {
                            if (term_index == terms_array->term_buffer_size)
                            {
                                term_index = 0;
                                buffer_index++;
                            }
                            index_t index = terms_array->terms[buffer_index][term_index];
                            occupied_config[index]--;
                        }
                    } else
                    {
                        voxels[i].occupied = true;
                        num_occupied++;
                        for (; start_index != end_index; start_index++, term_index++)
                        {
                            if (term_index == terms_array->term_buffer_size)
                            {
                                term_index = 0;
                                buffer_index++;
                            }
                            index_t index = terms_array->terms[buffer_index][term_index];
                            occupied_config[index]++;
                        }
                    }

                    label_changed++;
                    sfis_error -= abs(partial);
                }
            }
        }

        //        if (last_turn_error <= sfis_error)
        //        {
        //            cout << "label changed: " << label_changed << ", current sfis error: " << sfis_error / 127 << endl;
        //            break;
        //        }
        //        last_turn_error = sfis_error;
        if (label_changed == 0)
        {
            cout << "label changed: " << label_changed << ", current sfis error: " << sfis_error / 127 << endl;
            break;
        }

        for (index_t i = 0; i < num_voxel; i++)
        {
            update_bound(i);
        }
        for (index_t i = 0; i < num_voxel; i++)
        {
            if (voxels[i].bound ^ BOUND)
            {
                if (voxels[i].bound != 0)
                {
                    if (!voxels[i].test)
                    {
                        num_to_test++;
                    }
                    voxels[i].test = true;
                } else // single voxel which is without neighbors, make it not be tested
                {
                    if (!voxels[i].occupied)
                    {
                        if (voxels[i].test)
                        {
                            num_to_test--;
                        }
                        voxels[i].test = false;
                    } else // should test if is occupied
                    {
                        if (!voxels[i].test)
                        {
                            num_to_test++;
                        }
                        voxels[i].test = true;
                    }
                }
            } else
            {
                if (voxels[i].test)
                {
                    num_to_test--;
                }
                voxels[i].test = false;
            }
        }
        cout << "label changed: " << label_changed << ", current sfis error: " << sfis_error / 127 << endl;
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

void Model::visual_hull() const
{
    cout << "visual hull..." << endl;

    for (index_t i = 0; i < num_voxel; i++)
    {
        bool some_positive = false;
        bool some_negative = false;
        uint64_t start_index = terms_array->indices[i];
        uint64_t end_index = terms_array->indices[i + 1];
        index_t buffer_index = start_index / terms_array->term_buffer_size;
        index_t term_index = start_index % terms_array->term_buffer_size;

        for (; start_index != end_index; start_index++, term_index++)
        {
            if (term_index == terms_array->term_buffer_size)
            {
                term_index = 0;
                buffer_index++;
            }
            index_t index = terms_array->terms[buffer_index][term_index];

            if (uniform->refer_image->coefficients[index] > 0)
            {
                some_positive = true;
            } else
            {
                some_negative = true;
            }

            if (some_positive && some_negative)
            {
                voxels[i].test = true;
                break;
            }
        }

        if (!some_positive) // voxel is always out of silhouette
        {
            voxels[i].occupied = false;

            // update terms
            start_index = terms_array->indices[i];
            end_index = terms_array->indices[i + 1];
            buffer_index = start_index / terms_array->term_buffer_size;
            term_index = start_index % terms_array->term_buffer_size;

            for (uint64_t j = start_index; j < end_index; j++, term_index++)
            {
                if (term_index == terms_array->term_buffer_size)
                {
                    term_index = 0;
                    buffer_index++;
                }
                occupied_config[terms_array->terms[buffer_index][term_index]]--;
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

void Model::culling()
{
    cout << "culling..." << endl;

    index_t image_size = uniform->refer_image->x * uniform->refer_image->y;
    for (index_t i = 0; i < num_voxel; i++)
    {
        if (voxels[i].occupied && voxels[i].test)
        {
            bool all_negative = true;
            uint64_t start_index = terms_array->indices[i];
            uint64_t end_index = terms_array->indices[i + 1];
            index_t buffer_index = start_index / terms_array->term_buffer_size;
            index_t term_index = start_index % terms_array->term_buffer_size;

            if (term_index == terms_array->term_buffer_size)
            {
                term_index = 0;
                buffer_index++;
            }
            index_t previous_index = terms_array->terms[buffer_index][term_index];
            index_t previous_image_index = previous_index / image_size;

            for (; start_index != end_index; start_index++, term_index++)
            {
                if (term_index == terms_array->term_buffer_size)
                {
                    term_index = 0;
                    buffer_index++;
                }
                index_t index = terms_array->terms[buffer_index][term_index];
                index_t image_index = index / image_size;

                if (previous_image_index == image_index)
                {
                    if (uniform->refer_image->coefficients[index] > 0)
                    {
                        all_negative = false;
                    }
                } else
                {
                    previous_image_index = image_index;
                    if (all_negative)
                    {
                        break;
                    } else
                    {
                        all_negative = true;
                    }

                    if (uniform->refer_image->coefficients[index] > 0)
                    {
                        all_negative = false;
                    }
                }
            }

            if (all_negative) // voxel is out of silhouette in an image
            {
                //                voxels[i].test = false;
                voxels[i].occupied = false;

                // update terms
                start_index = terms_array->indices[i];
                end_index = terms_array->indices[i + 1];
                buffer_index = start_index / terms_array->term_buffer_size;
                term_index = start_index % terms_array->term_buffer_size;

                for (uint64_t j = start_index; j < end_index; j++, term_index++)
                {
                    if (term_index == terms_array->term_buffer_size)
                    {
                        term_index = 0;
                        buffer_index++;
                    }
                    occupied_config[terms_array->terms[buffer_index][term_index]]--;
                }
            }
        }
    }

    for (index_t i = 0; i < num_voxel; i++)
    {
        update_bound(i);
    }
    for (index_t i = 0; i < num_voxel; i++)
    {
        if (voxels[i].bound ^ BOUND)
        {
            if (voxels[i].bound != 0)
            {
                voxels[i].test = true;
            } else // single voxel which is without neighbors, make it not be tested
            {
                if (!voxels[i].occupied)
                {
                    voxels[i].test = false;
                } else // should test if is occupied
                {
                    voxels[i].test = true;
                }
            }
        } else
        {
            voxels[i].test = false;
        }
    }
}

void Model::cut() const
{
    for (index_t i = 0; i < num_voxel; i++)
    {
        if(voxels[i].occupied && (voxels[i].bound ^ BOUND))
        {
            voxels[i].occupied = false;
        }
    }
}

void Model::convert_polygon_mesh(vector<Vertex> &_vertices, vector<Triangle> &_triangles) const
{
    int num_hull = 0;
    auto *output_index = new index_t[num_vertices];
    index_t max = 1 << 31;
    for (index_t i = 0; i < num_vertices; i++)
    {
        output_index[i] = max; // means empty
    }
    index_t voxel_index = 0;
    index_t current_index = 0;
    vector<index_t> vertex_index;

    // update bound information
    for (index_t i = 0; i < num_voxel; i++)
    {
        update_bound(i);
    }

    for (index_t index_z = 0; index_z < z; index_z++)
    {
        for (index_t index_x = 0; index_x < x; index_x++)
        {
            for (index_t index_y = 0; index_y < y; index_y++, voxel_index++)
            {
                if (voxels[voxel_index].occupied && voxels[voxel_index].bound < BOUND)
                {
                    num_hull++;
                    index_t v_0 = (index_z * vx + index_x) * vy + index_y;
                    index_t v_1 = v_0 + 1;
                    index_t v_2 = v_0 + vy;
                    index_t v_3 = v_2 + 1;
                    index_t v_4 = v_0 + vx * vy;
                    index_t v_5 = v_4 + 1;
                    index_t v_6 = v_4 + vy;
                    index_t v_7 = v_6 + 1;

                    if (!(voxels[voxel_index].bound & FRONT))
                    {
                        if (output_index[v_2] == max)
                        {
                            output_index[v_2] = current_index;
                            _vertices.push_back(vertices[v_2]);
                            vertex_index.push_back(v_2);
                            current_index++;
                        }
                        if (output_index[v_3] == max)
                        {
                            output_index[v_3] = current_index;
                            _vertices.push_back(vertices[v_3]);
                            vertex_index.push_back(v_3);
                            current_index++;
                        }
                        if (output_index[v_6] == max)
                        {
                            output_index[v_6] = current_index;
                            _vertices.push_back(vertices[v_6]);
                            vertex_index.push_back(v_6);
                            current_index++;
                        }
                        if (output_index[v_7] == max)
                        {
                            output_index[v_7] = current_index;
                            _vertices.push_back(vertices[v_7]);
                            vertex_index.push_back(v_7);
                            current_index++;
                        }
                        _triangles.emplace_back(output_index[v_6], output_index[v_2], output_index[v_7]);
                        _triangles.emplace_back(output_index[v_7], output_index[v_2], output_index[v_3]);
                    }
                    if (!(voxels[voxel_index].bound & BACK))
                    {
                        if (output_index[v_0] == max)
                        {
                            output_index[v_0] = current_index;
                            _vertices.push_back(vertices[v_0]);
                            vertex_index.push_back(v_0);
                            current_index++;
                        }
                        if (output_index[v_1] == max)
                        {
                            output_index[v_1] = current_index;
                            _vertices.push_back(vertices[v_1]);
                            vertex_index.push_back(v_1);
                            current_index++;
                        }
                        if (output_index[v_4] == max)
                        {
                            output_index[v_4] = current_index;
                            _vertices.push_back(vertices[v_4]);
                            vertex_index.push_back(v_4);
                            current_index++;
                        }
                        if (output_index[v_5] == max)
                        {
                            output_index[v_5] = current_index;
                            _vertices.push_back(vertices[v_5]);
                            vertex_index.push_back(v_5);
                            current_index++;
                        }
                        _triangles.emplace_back(output_index[v_4], output_index[v_5], output_index[v_0]);
                        _triangles.emplace_back(output_index[v_5], output_index[v_1], output_index[v_0]);
                    }
                    if (!(voxels[voxel_index].bound & LEFT))
                    {
                        if (output_index[v_0] == max)
                        {
                            output_index[v_0] = current_index;
                            _vertices.push_back(vertices[v_0]);
                            vertex_index.push_back(v_0);
                            current_index++;
                        }
                        if (output_index[v_2] == max)
                        {
                            output_index[v_2] = current_index;
                            _vertices.push_back(vertices[v_2]);
                            vertex_index.push_back(v_2);
                            current_index++;
                        }
                        if (output_index[v_4] == max)
                        {
                            output_index[v_4] = current_index;
                            _vertices.push_back(vertices[v_4]);
                            vertex_index.push_back(v_4);
                            current_index++;
                        }
                        if (output_index[v_6] == max)
                        {
                            output_index[v_6] = current_index;
                            _vertices.push_back(vertices[v_6]);
                            vertex_index.push_back(v_6);
                            current_index++;
                        }
                        _triangles.emplace_back(output_index[v_4], output_index[v_0], output_index[v_6]);
                        _triangles.emplace_back(output_index[v_6], output_index[v_0], output_index[v_2]);
                    }
                    if (!(voxels[voxel_index].bound & RIGHT))
                    {
                        if (output_index[v_1] == max)
                        {
                            output_index[v_1] = current_index;
                            _vertices.push_back(vertices[v_1]);
                            vertex_index.push_back(v_1);
                            current_index++;
                        }
                        if (output_index[v_3] == max)
                        {
                            output_index[v_3] = current_index;
                            _vertices.push_back(vertices[v_3]);
                            vertex_index.push_back(v_3);
                            current_index++;
                        }
                        if (output_index[v_5] == max)
                        {
                            output_index[v_5] = current_index;
                            _vertices.push_back(vertices[v_5]);
                            vertex_index.push_back(v_5);
                            current_index++;
                        }
                        if (output_index[v_7] == max)
                        {
                            output_index[v_7] = current_index;
                            _vertices.push_back(vertices[v_7]);
                            vertex_index.push_back(v_7);
                            current_index++;
                        }
                        _triangles.emplace_back(output_index[v_7], output_index[v_3], output_index[v_5]);
                        _triangles.emplace_back(output_index[v_5], output_index[v_3], output_index[v_1]);
                    }
                    if (!(voxels[voxel_index].bound & TOP))
                    {
                        if (output_index[v_4] == max)
                        {
                            output_index[v_4] = current_index;
                            _vertices.push_back(vertices[v_4]);
                            vertex_index.push_back(v_4);
                            current_index++;
                        }
                        if (output_index[v_5] == max)
                        {
                            output_index[v_5] = current_index;
                            _vertices.push_back(vertices[v_5]);
                            vertex_index.push_back(v_5);
                            current_index++;
                        }
                        if (output_index[v_6] == max)
                        {
                            output_index[v_6] = current_index;
                            _vertices.push_back(vertices[v_6]);
                            vertex_index.push_back(v_6);
                            current_index++;
                        }
                        if (output_index[v_7] == max)
                        {
                            output_index[v_7] = current_index;
                            _vertices.push_back(vertices[v_7]);
                            vertex_index.push_back(v_7);
                            current_index++;
                        }
                        _triangles.emplace_back(output_index[v_4], output_index[v_6], output_index[v_5]);
                        _triangles.emplace_back(output_index[v_5], output_index[v_6], output_index[v_7]);
                    }
                    if (!(voxels[voxel_index].bound & BOTTOM))
                    {
                        if (output_index[v_0] == max)
                        {
                            output_index[v_0] = current_index;
                            _vertices.push_back(vertices[v_0]);
                            vertex_index.push_back(v_0);
                            current_index++;
                        }
                        if (output_index[v_1] == max)
                        {
                            output_index[v_1] = current_index;
                            _vertices.push_back(vertices[v_1]);
                            vertex_index.push_back(v_1);
                            current_index++;
                        }
                        if (output_index[v_2] == max)
                        {
                            output_index[v_2] = current_index;
                            _vertices.push_back(vertices[v_2]);
                            vertex_index.push_back(v_2);
                            current_index++;
                        }
                        if (output_index[v_3] == max)
                        {
                            output_index[v_3] = current_index;
                            _vertices.push_back(vertices[v_3]);
                            vertex_index.push_back(v_3);
                            current_index++;
                        }
                        _triangles.emplace_back(output_index[v_0], output_index[v_1], output_index[v_2]);
                        _triangles.emplace_back(output_index[v_2], output_index[v_1], output_index[v_3]);
                    }
                }
            }
        }
    }

    delete[] output_index;
    cout << "hull voxel number: " << num_hull << endl;

    //    int camera_order = 0;
    //    //    Image image = Image(uniform->refer_image->x, uniform->refer_image->y, uniform->refer_image->num_camera);
    //
    //    for (const auto camera: *uniform->cameras)
    //    {
    //        index_t offset = camera_order * camera->x * camera->y;
    //        camera_order++;
    //
    //        int index = 0;
    //        for (const auto &vertex: _vertices)
    //        {
    //            float4 screen = camera->P * vertex.world;
    //            screen = screen / screen.w();
    //            int pixel_x = (int)round(screen.x());
    //            int pixel_y = (int)round(screen.y());
    //
    //            if (pixel_x < 0 || pixel_x >= uniform->refer_image->x || pixel_y < 0 || pixel_y >= uniform->refer_image->y)
    //            {
    //                continue;
    //            }
    //
    //            //            image.data[offset + pixel_y * uniform ->refer_image->x + pixel_x] = 255;
    //            matches_3d_2d.emplace_back(vertex_index[index], offset + pixel_y * uniform->refer_image->x + pixel_x);
    //            index++;
    //        }
    //    }

    //    iodata::write_contour_2d_points(image);
}

void Model::update_bound(index_t index) const
{
    index_t index_xy = index % (x * y);
    index_t index_z = index / (x * y);
    if (index_xy / y == x - 1)
    {
        voxels[index].bound &= MASK_FRONT;
    } else
    {
        if (voxels[index + y].occupied)
        {
            voxels[index].bound |= FRONT;
        } else
        {
            voxels[index].bound &= MASK_FRONT;
        }
    }
    if (index_xy / y == 0)
    {
        voxels[index].bound &= MASK_BACK;
    } else
    {
        if (voxels[index - y].occupied)
        {
            voxels[index].bound |= BACK;
        } else
        {
            voxels[index].bound &= MASK_BACK;
        }
    }
    if (index_xy % y == 0)
    {
        voxels[index].bound &= MASK_LEFT;
    } else
    {
        if (voxels[index - 1].occupied)
        {
            voxels[index].bound |= LEFT;
        } else
        {
            voxels[index].bound &= MASK_LEFT;
        }
    }
    if (index_xy % y == y - 1)
    {
        voxels[index].bound &= MASK_RIGHT;
    } else
    {
        if (voxels[index + 1].occupied)
        {
            voxels[index].bound |= RIGHT;
        } else
        {
            voxels[index].bound &= MASK_RIGHT;
        }
    }
    if (index_z == z - 1)
    {
        voxels[index].bound &= MASK_TOP;
    } else
    {
        if (voxels[index + x * y].occupied)
        {
            voxels[index].bound |= TOP;
        } else
        {
            voxels[index].bound &= MASK_TOP;
        }
    }
    if (index_z == 0)
    {
        voxels[index].bound &= MASK_BOTTOM;
    } else
    {
        if (voxels[index - x * y].occupied)
        {
            voxels[index].bound |= BOTTOM;
        } else
        {
            voxels[index].bound &= MASK_BOTTOM;
        }
    }
}

bool Model::is_bound(index_t index, DirectionType direction) const
{
    index_t index_xy = index % (x * y);
    index_t index_z = index / (x * y);
    switch (direction)
    {
        case FRONT:
            return (index_xy / y == x - 1) || (!voxels[index + y].occupied);
        case BACK:
            return (index_xy / y == 0) || (!voxels[index - y].occupied);
        case LEFT:
            return (index_xy % y == 0) || (!voxels[index - 1].occupied);
        case RIGHT:
            return (index_xy % y == y - 1) || (!voxels[index + 1].occupied);
        case TOP:
            return (index_z == z - 1) || (!voxels[index + x * y].occupied);
        case BOTTOM:
            return (index_z == 0) || (!voxels[index - x * y].occupied);
    }
}

bool Model::is_bound(index_t index_x, index_t index_y, index_t index_z, DirectionType direction) const
{
    index_t index = (index_z * x + index_x) * y + index_y;
    switch (direction)
    {
        case FRONT:
            return (index_x == x - 1) || (!voxels[index + y].occupied);
        case BACK:
            return (index_x == 0) || (!voxels[index - y].occupied);
        case LEFT:
            return (index_y == 0) || (!voxels[index - 1].occupied);
        case RIGHT:
            return (index_y == y - 1) || (!voxels[index + 1].occupied);
        case TOP:
            return (index_z == z - 1) || (!voxels[index + x * y].occupied);
        case BOTTOM:
            return (index_z == 0) || (!voxels[index - x * y].occupied);
    }
}

void Model::draw_triangle(const Vertex &v_0, const Vertex &v_1, const Vertex &v_2, bool occupied) const
{
    int camera_order = 0;
    num_triangle++;

    for (const auto &camera: *uniform->cameras)
    {
        index_t offset = camera_order * camera->x * camera->y;
        camera_order++;

        float4 screen_0 = camera->P * v_0.world;
        float4 screen_1 = camera->P * v_1.world;
        float4 screen_2 = camera->P * v_2.world;
        screen_0 = screen_0 / screen_0.w();
        screen_1 = screen_1 / screen_1.w();
        screen_2 = screen_2 / screen_2.w();

        // back face culling
        float AB_x = screen_1.x() - screen_0.x();
        float AB_y = screen_1.y() - screen_0.y();
        float AC_x = screen_2.x() - screen_0.x();
        float AC_y = screen_2.y() - screen_0.y();
        if (AB_x * AC_y - AB_y * AC_x > 0)
        {
            continue;
        }

        // real bounding box
        int min_x = max((int)min(screen_0.x(), min(screen_1.x(), screen_2.x())), 0);
        int min_y = max((int)min(screen_0.y(), min(screen_1.y(), screen_2.y())), 0);
        int max_x = min((int)max(screen_0.x(), max(screen_1.x(), screen_2.x())) + 1, camera->x - 1);
        int max_y = min((int)max(screen_0.y(), max(screen_1.y(), screen_2.y())) + 1, camera->y - 1);

        // rasterization
        for (int i = min_y; i <= max_y; i++)
        {
            for (int j = min_x; j <= max_x; j++)
            {
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
                if (AB > 0 && BC > 0 && CA > 0)
                {
                    if (is_in_triangle(AB, BC, CA))
                    {
                        index_t term_index = offset + i * camera->x + j;
                        if (occupied)
                        {
                            occupied_config[term_index]++;
                        }
                        if (terms_array->capacity == terms_array->num_terms)
                        {
                            terms_array->increase_buffer();
                        }

                        terms_array->terms[terms_array->buffer_index][terms_array->term_index] = term_index;
                        terms_array->num_terms++;
                        terms_array->term_index++;
                    }
                }
            }
        }
    }
}

void Model::draw_circle(const float4 &c, bool occupied) const
{
    index_t offset = 0;
    num_triangle++;

    for (const auto &camera: *uniform->cameras)
    {
        float4 edge = camera->M_view * c;
        edge.x() += voxel_radius;
        float4 center = camera->P * c;
        center = center / center.w();
        edge = camera->PV * edge;
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
        for (int i = min_y; i <= max_y; i++)
        {
            float p_x = x0;
            index_t index = term_index + min_x;

            for (int j = min_x; j <= max_x; j++)
            {
                float dx = center.x() - p_x;
                float dy = center.y() - p_y;

                if (dx * dx + dy * dy < r2)
                {
                    if (occupied)
                    {
                        occupied_config[index]++;
                    }
                    if (terms_array->capacity == terms_array->num_terms)
                    {
                        terms_array->increase_buffer();
                    }

                    terms_array->terms[terms_array->buffer_index][terms_array->term_index] = index;
                    terms_array->num_terms++;
                    terms_array->term_index++;
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

void Model::calculate_error()
{
    sfis_error = 0;

    for (index_t i = 0; i < uniform->num_pixel; i++)
    {
        sfis_error += ((255 - uniform->refer_image->data[i]) / 2);
        if (occupied_config[i] == 0)
        {
            sfis_error += uniform->refer_image->coefficients[i];
        }
    }
}

error_t Model::calculate_partial(index_t voxel_index) const
{
    uint64_t start_index = terms_array->indices[voxel_index];
    uint64_t end_index = terms_array->indices[voxel_index + 1];
    index_t buffer_index = start_index / terms_array->term_buffer_size;
    index_t term_index = start_index % terms_array->term_buffer_size;

    error_t partial = 0;

    if (voxels[voxel_index].occupied)
    {
        for (; start_index != end_index; start_index++, term_index++)
        {
            if (term_index == terms_array->term_buffer_size)
            {
                term_index = 0;
                buffer_index++;
            }
            index_t index = terms_array->terms[buffer_index][term_index];

            if (occupied_config[index] == 1)
            {
                partial += uniform->refer_image->coefficients[index];
            }
        }
    } else
    {
        for (; start_index != end_index; start_index++, term_index++)
        {
            if (term_index == terms_array->term_buffer_size)
            {
                term_index = 0;
                buffer_index++;
            }
            index_t index = terms_array->terms[buffer_index][term_index];

            if (occupied_config[index] == 0)
            {
                partial += uniform->refer_image->coefficients[index];
            }
        }
    }

    return partial;
}

void Model::write_projection(const string &directory) const
{
    Image image = Image(uniform->refer_image->x, uniform->refer_image->y, uniform->refer_image->num_camera);

    //    for (index_t i = 0; i < uniform->num_pixel; i++)
    //    {
    //        image.data[i] = occupied_config[i] > 0 ? 255 : 0;
    //    }
    for (index_t i = 0; i < num_voxel; i++)
    {
        if (voxels[i].occupied)
        {
            uint64_t start_index = terms_array->indices[i];
            uint64_t end_index = terms_array->indices[i + 1];
            index_t buffer_index = start_index / terms_array->term_buffer_size;
            index_t term_index = start_index % terms_array->term_buffer_size;
            for (; start_index != end_index; start_index++, term_index++)
            {
                if (term_index == terms_array->term_buffer_size)
                {
                    term_index = 0;
                    buffer_index++;
                }
                index_t index = terms_array->terms[buffer_index][term_index];
                image.data[index] = 255;
            }
        }
    }
    iodata::write_projection(directory, image);
}

void Model::write_sie(const string &directory) const
{
    Image image = Image(uniform->refer_image->x, uniform->refer_image->y, uniform->refer_image->num_camera);

    for (index_t i = 0; i < uniform->num_pixel; i++)
    {
        image.data[i] = occupied_config[i] > 0 ? 255 : 0;
    }
    iodata::write_sie(directory, image, *uniform->refer_image);
}

void Model::propagate_labels(Model *child) const
{
    cout << "propagate labels..." << endl;
    assert(child->x / x == 2 && child->y / y == 2 && child->z / z == 2);
    index_t voxel_index = 0;
    bool padding_x = child->x % x == 0;
    bool padding_y = child->y % y == 0;
    bool padding_z = child->z % z == 0;

    for (int i = 0; i < child->num_voxel; i++)
    {
        child->voxels[i].occupied = false;
    }

    for (int index_z = 0; index_z < child->z; index_z++)
    {
        index_t parent_z = index_z / 2;
        bool edge_z = (index_z != child->z - 1) || padding_z;
        for (int index_x = 0; index_x < child->x; index_x++)
        {
            index_t parent_x = index_x / 2;
            bool edge_x = (index_x != child->x - 1) || padding_x;
            for (int index_y = 0; index_y < child->y; index_y++, voxel_index++)
            {
                bool edge_y = (index_y != child->y - 1) || padding_y;
                if (edge_z && edge_x && edge_y)
                {
                    index_t parent_y = index_y / 2;
                    index_t parent_index = y * (parent_z * x + parent_x) + parent_y;
                    child->voxels[voxel_index].occupied = voxels[parent_index].occupied;
                }
            }
        }
    }
}

void Model::correspond()
{
    vector<index_t> all_points; // all 2d points
    vector<index_t> refer_2d_points;
    //    vector<index_t> recon_2d_points; // contour 2d points

    for (auto matches: matches_3d_2d)
    {
        all_points.push_back(matches.second);
    }
    set<index_t> all_points_set(all_points.begin(), all_points.end()); // to remove same index
    all_points.assign(all_points_set.begin(), all_points_set.end());

    vector<vector<float2>> recon_points(uniform->refer_image->num_camera); // to process 2D points in each image
    index_t image_x = uniform->refer_image->x;
    index_t image_y = uniform->refer_image->y;
    index_t image_size = image_x * image_y;
    for (index_t i = 0; i < uniform->refer_image->num_camera; i++)
    {
        index_t start = i * image_size;
        index_t end = start + image_size;
        for (auto p: all_points)
        {
            if (p >= start && p < end)
            {
                index_t point_index = p - start;
                float point_x = (float)(point_index % image_x) + 0.5f;
                float point_y = (float)(point_index / image_x) + 0.5f;
                recon_points[i].emplace_back(float2(point_x, point_y));
            }
        }
    }

    // calculate convex hull in each image
    for (index_t i = 0; i < uniform->refer_image->num_camera; i++)
    {
        vector<cv::Point2f> cv_points;
        vector<cv::Point2f> hull;
        for (const auto &p: recon_points[i])
        {
            cv_points.emplace_back(p.x(), p.y());
        }
        cv::convexHull(cv_points, hull);

        recon_points[i].clear();
        for (const auto &p: hull)
        {
            recon_points[i].emplace_back(float2(p.x, p.y));
        }
    }

    // processing reference image contours
    for (index_t index = 0; index < uniform->num_pixel; index++)
    {
        if (uniform->refer_image->contours[index])
        {
            refer_2d_points.push_back(index);
        }
    }

    vector<vector<float2>> refer_points(uniform->refer_image->num_camera);
    for (index_t i = 0; i < uniform->refer_image->num_camera; i++)
    {
        index_t start = i * image_size;
        index_t end = start + image_size;
        for (auto p: refer_2d_points)
        {
            if (p >= start && p < end)
            {
                index_t point_index = p - start;
                float point_x = (float)(point_index % image_x) + 0.5f;
                float point_y = (float)(point_index / image_x) + 0.5f;
                refer_points[i].emplace_back(float2(point_x, point_y));
            }
        }
    }

    // correspond 2D-2D points
    float max_distance = sqrtf((float)(image_x * image_x + image_y * image_y));
    for (index_t i = 0; i < uniform->refer_image->num_camera; i++)
    {
        index_t offset = i * image_size;
        for (const auto &p1: recon_points[i])
        {
            float distance = max_distance;
            float2 near;
            for (const auto &p2: refer_points[i])
            {
                float current_distance = (p2 - p1).lpNorm<2>();
                if (current_distance < distance)
                {
                    distance = current_distance;
                    near = p2;
                    if (distance < 0.5) // distance is shorter than a pixel
                    {
                        break;
                    }
                }
            }
            matches_2d_2d.emplace_back(
                    offset + (index_t)p1.y() * image_x + (index_t)p1.x(),
                    offset + (index_t)near.y() * image_x + (index_t)near.x());
        }
    }

    // update 2D-2D matches, remove pair that 2D point is not on the contour
    bool *contours = new bool[uniform->num_pixel];
    for (auto &index: matches_2d_2d)
    {
        contours[index.first] = true;
    }
    vector<Pair<index_t, index_t>> term = matches_3d_2d; // copy
    matches_3d_2d.clear();
    for (const auto &p: term)
    {
        if (contours[p.second])
        {
            matches_3d_2d.push_back(p);
        }
    }

    delete[] contours;
    cout << "3D-2D matches pair: " << matches_3d_2d.size() << endl;
    cout << "2D-2D matches pair: " << matches_2d_2d.size() << endl;

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
