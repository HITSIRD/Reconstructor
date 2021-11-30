//
// Created by 闻永言 on 2021/9/22.
//

#include <iostream>
#include "SfIS.hpp"
#include "iodata.hpp"
#include "Correction.hpp"
#include "sys/time.h"
#include "opencv2/imgproc.hpp"

using namespace std;

SfIS::SfIS(const string &voxel_config, const string &config)
{
    iodata::read_config(voxel_config, config, *this);
}

SfIS::~SfIS()
{
//    for (auto model: models)
//    {
//        delete model;
//    }
    for (auto camera: cameras)
    {
        delete camera;
    }
}

void SfIS::read_config(const string &config_file)
{
    iodata::read_config(config_file, cameras, refer_image, bounding_box, expected_voxel_size, iteration);
}

void SfIS::render(SfS::MODE mode)
{
    switch (mode)
    {
        case SfS::NO_CORRECTION:
            cout << "mode: NO_CORRECTION" << endl;
            //            render_normal();
            render_hierarchy();
            break;
        case SfS::CORRECTION:
            cout << "mode: CORRECTION" << endl;
            render_correct();
            break;
    }
}

void SfIS::render_normal()
{
    struct timeval start, end, t0, t1;
    double start_time, end_time, cost;

    gettimeofday(&start, NULL);
    Model *model = new Model((index_t)(bounding_box.bounding_box_size.x() / expected_voxel_size),
                             (index_t)(bounding_box.bounding_box_size.y() / expected_voxel_size),
                             (index_t)(bounding_box.bounding_box_size.z() / expected_voxel_size),
                             bounding_box.min_point, expected_voxel_size);
    models.push_back(model);

    // clear and create working directory
    string directory = "Data/Model_S" + to_string<float>(model->voxel_size) + "_I" + to_string<int>(iteration);
    string equation = "\"";
    string command = "rm -r " + equation + directory + equation;
    system(command.c_str());
    command = "mkdir -p " + equation + directory + equation;
    system(command.c_str());

    Uniform *uniform = new Uniform(refer_image->num_camera * refer_image->x * refer_image->y);
    uniform->set_cameras(&cameras);
    uniform->set_refer_image(refer_image);
    model->set_uniform(uniform);
    model->initialize();

    cout << "model size[x, y, z]: [" << model->x << ", " << model->y << ", " << model->z << "]" << endl;
    cout << "current voxel size: " << expected_voxel_size << endl;
    cout << "voxel number: " << model->num_voxel << endl;
    cout << "pixel number: " << uniform->num_pixel << endl;
    cout << "bounding box: [" << bounding_box.min_point.x() << ", " << bounding_box.min_point.y() << ", "
         << bounding_box.min_point.z() << "], [";
    cout << bounding_box.max_point.x() << ", " << bounding_box.max_point.y() << ", " << bounding_box.max_point.x()
         << "]" << endl;
    cout << "iteration: " << iteration << endl;
    cout << "expected voxel size: " << expected_voxel_size << endl;

    gettimeofday(&t0, NULL);
    model->back_projection(SfS::FAST);
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "back projection cost time: " << cost << " s" << endl;

    model->visual_hull();
    model->calculate_error();
    cout << "sfis error: " << model->sfis_error / 127 << endl;

    gettimeofday(&t0, NULL);
    model->local_min_search();
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "local minimum search cost time: " << cost << " s" << endl;

    cout << "writing ply file..." << endl;
    gettimeofday(&t0, NULL);
    iodata::write_ply(directory, model);
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "write model cost time: " << cost << " s" << endl;

    cout << "writing sie comparison..." << endl;
    model->write_sie(directory);
    //    model->write_projection(directory);

    gettimeofday(&end, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(end.tv_sec - start.tv_sec) + double(end.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "total cost time: " << cost << " s" << endl;

    //    iodata::write_test(directory, *refer_image);
}

void SfIS::render_correct()
{
    struct timeval start, end, t0, t1;
    double start_time, end_time, cost;

    gettimeofday(&start, NULL);
    Model *model = new Model((index_t)(bounding_box.bounding_box_size.x() / expected_voxel_size),
                             (index_t)(bounding_box.bounding_box_size.y() / expected_voxel_size),
                             (index_t)(bounding_box.bounding_box_size.z() / expected_voxel_size),
                             bounding_box.min_point, expected_voxel_size);
    models.push_back(model);

    // clear and create working directory
    string directory = "Data/Model_S" + to_string<float>(model->voxel_size) + "_I" + to_string<int>(iteration);
    string equation = "\"";
    string command = "rm -r " + equation + directory + equation;
    system(command.c_str());
    command = "mkdir -p " + equation + directory + equation;
    system(command.c_str());

    // add noise
    cout << "add noise..." << endl;
    for (auto &camera: cameras)
    {
        camera->add_outer_noise(0, 0.02f, 0, 0.2f);
    }
    Uniform *uniform = new Uniform(refer_image->num_camera * refer_image->x * refer_image->y);
    uniform->set_cameras(&cameras);
    uniform->set_refer_image(refer_image);
    model->set_uniform(uniform);
    model->initialize();
    calculate_refer_contours();

    cout << "model size[x, y, z]: [" << model->x << ", " << model->y << ", " << model->z << "]" << endl;
    cout << "current voxel size: " << expected_voxel_size << endl;
    cout << "voxel number: " << model->num_voxel << endl;
    cout << "pixel number: " << uniform->num_pixel << endl;
    cout << "bounding box: [" << bounding_box.min_point.x() << ", " << bounding_box.min_point.y() << ", "
         << bounding_box.min_point.z() << "], [";
    cout << bounding_box.max_point.x() << ", " << bounding_box.max_point.y() << ", " << bounding_box.max_point.x()
         << "]" << endl;
    cout << "iteration: " << iteration << endl;
    cout << "expected voxel size: " << expected_voxel_size << endl;
    error_t last_turn_error = 1 << 30;
    int turn = 1;

    // reconstruction and parameters collaborative optimization
    while (true)
    {
        cout << "----------------------------------------------------" << endl;
        cout << "reconstruction turn " << turn << endl;
        if (turn > 1)
        {
            last_turn_error = model->sfis_error; // store last turn error after 1 turn
            cout << "last turn error: " << last_turn_error / 127 << endl;
        }
        turn++;

        //        for (auto &camera: cameras)
        //        {
        //            cout << camera->M_view << endl;
        //        }

        gettimeofday(&t0, NULL);
        model->back_projection(SfS::FAST);
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "back projection cost time: " << cost << " s" << endl;

        //    model->write_projection(directory);

        model->visual_hull();
        model->calculate_error();
        cout << "sfis error: " << model->sfis_error / 127 << endl;

        gettimeofday(&t0, NULL);
        model->local_min_search();
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "local minimum search cost time: " << cost << " s" << endl;

        if (model->sfis_error >= last_turn_error)
        {
            cout << "correction finished in " << turn << " turns" << endl;
            cout << "final error: " << last_turn_error / 127 << endl;
            break;
        }

        cout << "writing ply file..." << endl;
        gettimeofday(&t0, NULL);
        iodata::write_ply("model.ply", model);
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "write model cost time: " << cost << " s" << endl;

        cout << "writing sie comprision..." << endl;
        model->write_sie(directory);

        cout << "correspond..." << endl;
        gettimeofday(&t0, NULL);
        model->correspond();
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "correspond cost time: " << cost << " s" << endl;

        gettimeofday(&end, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(end.tv_sec - start.tv_sec) + double(end.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "total cost time: " << cost << " s" << endl;

        //    iodata::write_test(directory, *refer_image);
        Correction::correction(uniform, model->matches_3d_2d, model->matches_2d_2d, model->vertices, SfS::MATRIX);
        model->reset();
    }
}

void SfIS::render_hierarchy()
{
    struct timeval start, end, t0, t1, total_start, total_end;
    double start_time, end_time, cost;

    gettimeofday(&total_start, NULL);
    gettimeofday(&start, NULL);

    float voxel_size = (float)(1 << iteration) * expected_voxel_size;
    Model *model = new Model((index_t)(bounding_box.bounding_box_size.x() / voxel_size),
                             (index_t)(bounding_box.bounding_box_size.y() / voxel_size),
                             (index_t)(bounding_box.bounding_box_size.z() / voxel_size), bounding_box.min_point,
                             voxel_size);
    models.push_back(model);

    // clear and create working directory
    string directory = "Data/Model_S" + to_string<float>(expected_voxel_size) + "_I" + to_string<int>(iteration);
    string equation = "\"";
    string command = "rm -r " + equation + directory + equation;
    system(command.c_str());
    command = "mkdir -p " + equation + directory + equation;
    system(command.c_str());

    Uniform *uniform = new Uniform(refer_image->num_camera * refer_image->x * refer_image->y);
    uniform->set_cameras(&cameras);
    uniform->set_refer_image(refer_image);
    model->set_uniform(uniform);
    model->initialize();

    cout << "model size[x, y, z]: [" << model->x << ", " << model->y << ", " << model->z << "]" << endl;
    cout << "current voxel size: " << voxel_size << endl;
    cout << "voxel number: " << model->num_voxel << endl;
    cout << "pixel number: " << uniform->num_pixel << endl;
    cout << "bounding box: [" << bounding_box.min_point.x() << ", " << bounding_box.min_point.y() << ", "
         << bounding_box.min_point.z() << "], [";
    cout << bounding_box.max_point.x() << ", " << bounding_box.max_point.y() << ", " << bounding_box.max_point.x()
         << "]" << endl;
    cout << "iteration: " << iteration << endl;
    cout << "expected voxel size: " << expected_voxel_size / (float)iteration << endl;

    gettimeofday(&t0, NULL);
    model->back_projection(SfS::FAST);
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "back projection cost time: " << cost << " s" << endl;

    model->visual_hull();
    model->calculate_error();
    cout << "sfis error: " << model->sfis_error / 127 << endl;

    gettimeofday(&t0, NULL);
    model->local_min_search();
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "local minimum search cost time: " << cost << " s" << endl;

    model->terms_array->reset();

    gettimeofday(&end, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(end.tv_sec - start.tv_sec) + double(end.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "total cost time: " << cost << " s" << endl;

    int iter = 0;
    while (iter < iteration)
    {
        iter++;
        cout << "split iteration " << iter << endl;

        gettimeofday(&start, NULL);

        voxel_size = (float)(1 << (iteration - iter)) * expected_voxel_size;
        models[iter - 1]->cut(); // cut
        Model *split_model = split_from_model(models[iter - 1], voxel_size);
        delete models[iter - 1];
        models.push_back(split_model);

        split_model->set_uniform(uniform);
        split_model->initialize();

        cout << "model size[x, y, z]: [" << split_model->x << ", " << split_model->y << ", " << split_model->z << "]"
             << endl;
        cout << "current voxel size: " << voxel_size << endl;
        cout << "voxel number: " << split_model->num_voxel << endl;
        cout << "expected voxel size: " << expected_voxel_size << endl;

        gettimeofday(&t0, NULL);
        split_model->back_projection(SfS::FAST);
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "back projection cost time: " << cost << " s" << endl;

        split_model->calculate_error();
        cout << "sfis error: " << split_model->sfis_error / 127 << endl;

        gettimeofday(&t0, NULL);
        split_model->local_min_search();
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "local minimum search cost time: " << cost << " s" << endl;

        gettimeofday(&end, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(end.tv_sec - start.tv_sec) + double(end.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "iter " << iter << " cost time: " << cost << " s" << endl;
    }
    cout << "writing ply file..." << endl;
    gettimeofday(&t0, NULL);
    iodata::write_ply(directory, models[iteration]);
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "write model cost time: " << cost << " s" << endl;

    gettimeofday(&total_end, NULL);
    start_time = total_start.tv_usec / 1000000.0;
    end_time = double(total_end.tv_sec - total_start.tv_sec) + double(total_end.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "total cost time: " << cost << " s" << endl;

    cout << "writing sie comparison..." << endl;
    models[iteration]->write_sie(directory);
    //    model->write_projection(directory);
}

void SfIS::calculate_refer_contours()
{
    // get reference
    int camera_order = 0;
    int contours_count = 0;

    for (const auto &camera: cameras)
    {
        index_t offset = camera_order * camera->x * camera->y;
        camera_order++;

        cv::Mat src = cv::Mat(camera->y, camera->x, CV_8U);
        for (index_t i = 0; i < camera->x * camera->y; i++)
        {
            src.data[i] = refer_image->data[offset + i] > 0 ? 255 : 0;
        }
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierachy;
        cv::findContours(src, contours, hierachy, cv::RETR_EXTERNAL, 1, cv::Point(0, 0));
        if (!contours.empty())
        {
            contours_count++;
            for (auto &point: contours[0])
            {
                refer_image->contours[offset + point.y * camera->x + point.x] = true;
            }
        }
    }

    //    cout << "contours count: " << contours_count << endl;
    //    iodata::write_refer_contours(*refer_image);
}

Model *SfIS::split_from_model(Model *model, float voxel_size)
{
    Model *child = new Model((index_t)(bounding_box.bounding_box_size.x() / voxel_size),
                             (index_t)(bounding_box.bounding_box_size.y() / voxel_size),
                             (index_t)(bounding_box.bounding_box_size.z() / voxel_size), bounding_box.min_point,
                             voxel_size);

    model->propagate_labels(child);
    return child;
}
