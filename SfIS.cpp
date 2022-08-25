//
// Created by 闻永言 on 2021/9/22.
//

#include <iostream>
#include "SfIS.hpp"
#include "iodata.hpp"
#include "Correction.hpp"
#include "sys/time.h"
#include "opencv2/imgproc.hpp"
#include "Eigen/Dense"
#include "Util.hpp"

using namespace std;
using namespace sfs;

SfIS::SfIS(const string &voxelConfig, const string &config): mode(NO_CORRECTION), selfAdaptive(false) {
    iodata::readConfig(voxelConfig, config, *this);
}

SfIS::~SfIS() {
    for (auto model: models) {
        delete model;
    }
    for (auto camera: cameras) {
        delete camera;
    }
}

void SfIS::readConfig(const string &configFile) {
    iodata::readConfig(configFile, cameras, referImage, boundingBox, expectedVoxelSize, iteration);
}

void SfIS::render(sfs::MODE renderMode) {
    mode = renderMode;
    renderHierarchy();

//    switch (mode)
//    {
//        case SfS::NO_CORRECTION:
//            cout << "mode: NO_CORRECTION" << endl;
//            //            render_normal();
//            renderHierarchy();
//            break;
//        case SfS::CORRECTION:
//            cout << "mode: CORRECTION" << endl;
//            renderCorrection();
//            break;
//    }
}

void SfIS::renderNormal() {
    struct timeval start, end, t0, t1;
    double start_time, end_time, cost;

    gettimeofday(&start, NULL);
    Model *model = new Model((index_t)(boundingBox.boundingBoxSize.x() / expectedVoxelSize),
            (index_t)(boundingBox.boundingBoxSize.y() / expectedVoxelSize),
            (index_t)(boundingBox.boundingBoxSize.z() / expectedVoxelSize), boundingBox.minPoint, expectedVoxelSize);
    models.push_back(model);

    // clear and create working directory
    string directory = "Data/Model_S" + toString<float>(model->voxelInnerSize) + "_I" + toString<int>(iteration);
    string equation = "\"";
    string command = "rm -r " + equation + directory + equation;
    system(command.c_str());
    command = "mkdir -p " + equation + directory + equation;
    system(command.c_str());

    Uniform *uniform = new Uniform(referImage->numCameras * referImage->x * referImage->y);
    uniform->setCameras(&cameras);
    uniform->setReferImage(referImage);
    model->setUniform(uniform);
    model->initialize();

    cout << "model size[x, y, z]: [" << model->voxelSize.x() << ", " << model->voxelSize.y() << ", "
            << model->voxelSize.z() << "]" << endl;
    cout << "current voxel size: " << expectedVoxelSize << endl;
    cout << "voxel number: " << model->numVoxels << endl;
    cout << "pixel number: " << uniform->numPixels << endl;
    cout << "bounding box: [" << boundingBox.minPoint.x() << ", " << boundingBox.minPoint.y() << ", "
            << boundingBox.minPoint.z() << "], [";
    cout << boundingBox.maxPoint.x() << ", " << boundingBox.maxPoint.y() << ", " << boundingBox.maxPoint.x() << "]"
            << endl;
    cout << "iteration: " << iteration << endl;
    cout << "expected voxel size: " << expectedVoxelSize << endl;

    gettimeofday(&t0, NULL);
    model->backProjection(sfs::FAST);
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "back projection cost time: " << cost << " s" << endl;

    model->visualHull();
    model->updateSfISError();
    cout << "sfis error: " << model->sfisError / 127 << endl;

    gettimeofday(&t0, NULL);
    model->localMinSearch();
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "local minimum search cost time: " << cost << " s" << endl;

    cout << "writing ply file..." << endl;
    gettimeofday(&t0, NULL);
    iodata::writePly(directory, model);
    gettimeofday(&t1, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "write model cost time: " << cost << " s" << endl;

    cout << "writing sie comparison..." << endl;
    model->writeSIE(directory);
    //    model->write_projection(directory);

    gettimeofday(&end, NULL);
    start_time = t0.tv_usec / 1000000.0;
    end_time = double(end.tv_sec - start.tv_sec) + double(end.tv_usec) / 1000000.0;
    cost = end_time - start_time;
    cout << "total cost time: " << cost << " s" << endl;

    //    iodata::write_test(directory, *refer_image);
}

void SfIS::renderCorrection() {
    struct timeval start, end, t0, t1;
    double start_time, end_time, cost;

    gettimeofday(&start, NULL);
    Model *model = new Model((index_t)(boundingBox.boundingBoxSize.x() / expectedVoxelSize),
            (index_t)(boundingBox.boundingBoxSize.y() / expectedVoxelSize),
            (index_t)(boundingBox.boundingBoxSize.z() / expectedVoxelSize), boundingBox.minPoint, expectedVoxelSize);
    models.push_back(model);

    // clear and create working directory
    string directory = "Data/Model_S" + toString<float>(model->voxelInnerSize) + "_I" + toString<int>(iteration);
    string equation = "\"";
    string command = "rm -r " + equation + directory + equation;
    system(command.c_str());
    command = "mkdir -p " + equation + directory + equation;
    system(command.c_str());

    // add noise
    cout << "add noise..." << endl;
    for (auto &camera: cameras) {
        camera->addOuterNoise(0, 0.02f, 0, 0.2f);
    }
    Uniform *uniform = new Uniform(referImage->numCameras * referImage->x * referImage->y);
    uniform->setCameras(&cameras);
    uniform->setReferImage(referImage);
    model->setUniform(uniform);
    model->initialize();
    calculateReferContours();

    cout << "model size[x, y, z]: [" << model->voxelSize.x() << ", " << model->voxelSize.y() << ", "
            << model->voxelSize.z() << "]" << endl;
    cout << "current voxel size: " << expectedVoxelSize << endl;
    cout << "voxel number: " << model->numVoxels << endl;
    cout << "pixel number: " << uniform->numPixels << endl;
    cout << "bounding box: [" << boundingBox.minPoint.x() << ", " << boundingBox.minPoint.y() << ", "
            << boundingBox.minPoint.z() << "], [";
    cout << boundingBox.maxPoint.x() << ", " << boundingBox.maxPoint.y() << ", " << boundingBox.maxPoint.x() << "]"
            << endl;
    cout << "iteration: " << iteration << endl;
    cout << "expected voxel size: " << expectedVoxelSize << endl;
    error_t last_turn_error = 1 << 30;
    int turn = 1;

    // reconstruction and parameters collaborative optimization
    while (true) {
        cout << "----------------------------------------------------" << endl;
        cout << "reconstruction turn " << turn << endl;
        if (turn > 1) {
            last_turn_error = model->sfisError; // store last turn error after 1 turn
            cout << "last turn error: " << last_turn_error / 127 << endl;
        }
        turn++;

        //        for (auto &camera: cameras)
        //        {
        //            cout << camera->M_view << endl;
        //        }

        gettimeofday(&t0, NULL);
        model->backProjection(sfs::FAST);
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "back projection cost time: " << cost << " s" << endl;

        //    model->write_projection(directory);

        model->visualHull();
        model->updateSfISError();
        cout << "sfis error: " << model->sfisError / 127 << endl;

        gettimeofday(&t0, NULL);
        model->localMinSearch();
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "local minimum search cost time: " << cost << " s" << endl;

        if (model->sfisError >= last_turn_error) {
            cout << "correction finished in " << turn << " turns" << endl;
            cout << "final error: " << last_turn_error / 127 << endl;
            break;
        }

        cout << "writing ply file..." << endl;
        gettimeofday(&t0, NULL);
        iodata::writePly("model.ply", model);
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "write model cost time: " << cost << " s" << endl;

        cout << "writing sie comprision..." << endl;
        model->writeSIE(directory);

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
//        Correction::correction(uniform, model->matches3D2D, model->matches2D2D, model->vertices, SfS::EULER_ANGLE);
        model->reset();
    }
}

void SfIS::renderHierarchy() {
    struct timeval start, end, t0, t1, total_start, total_end;
    double start_time, end_time, cost;

//    cout << "add noise..." << endl;
//    cout << cameras[0]->matrixView;
//    for (auto &camera: cameras)
//    {
//        camera->addOuterNoise(0, 0.01f, 0, 10.f);
//    }

    gettimeofday(&total_start, NULL);
    gettimeofday(&start, NULL);

    if (selfAdaptive) {
        initializeSelfAdaptive();
    }

    // clear and create working directory
    string directory = "Data/Model_S" + toString<float>(expectedVoxelSize) + "_I" + toString<int>(iteration);
    string equation = "\"";
    string command = "rm -r " + equation + directory + equation;
    system(command.c_str());
    command = "mkdir -p " + equation + directory + equation;
    system(command.c_str());

    Uniform *uniform = new Uniform(referImage->numCameras * referImage->x * referImage->y);
    uniform->setCameras(&cameras);
    uniform->setReferImage(referImage);
    if (mode == CORRECTION) {
        calculateReferContours();
    }

    int turn = 1;
    error_t last_turn_error = 10000000000;

    // reconstruction and parameters collaborative optimization
    while (true) {
        cout << "----------------------------------------------------" << endl;
        cout << "reconstruction turn " << turn << endl;
        if (turn > 1) {
            last_turn_error = models[iteration]->sfisError; // store last turn error after 1 turn
            cout << "last turn error: " << last_turn_error / 127 << endl;
            delete models[iteration];
            models[iteration] = nullptr;
        }

        models.clear();
        float voxel_size = (float)(1 << iteration) * expectedVoxelSize;
        Model *model = new Model((index_t)(boundingBox.boundingBoxSize.x() / voxel_size),
                (index_t)(boundingBox.boundingBoxSize.y() / voxel_size),
                (index_t)(boundingBox.boundingBoxSize.z() / voxel_size), boundingBox.minPoint, voxel_size);
        models.push_back(model);
        model->setUniform(uniform);
        model->initialize();

        cout << "model size[x, y, z]: [" << model->voxelSize.x() << ", " << model->voxelSize.y() << ", "
                << model->voxelSize.z() << "]" << endl;
        cout << "current voxel size: " << voxel_size << endl;
        cout << "voxel number: " << model->numVoxels << endl;
        cout << "pixel number: " << uniform->numPixels << endl;
        cout << "bounding box: [" << boundingBox.minPoint.x() << ", " << boundingBox.minPoint.y() << ", "
                << boundingBox.minPoint.z() << "], [";
        cout << boundingBox.maxPoint.x() << ", " << boundingBox.maxPoint.y() << ", " << boundingBox.maxPoint.x() << "]"
                << endl;
        cout << "iteration: " << iteration << endl;
        cout << "expected voxel size: " << expectedVoxelSize << endl;

        gettimeofday(&t0, NULL);
        model->backProjection(sfs::FAST);
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "back projection cost time: " << cost << " s" << endl;

        model->visualHull();
        model->updateSfISError();
        cout << "sfis error: " << model->sfisError / 127 << endl;

        gettimeofday(&t0, NULL);
        model->localMinSearch();
        gettimeofday(&t1, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "local minimum search cost time: " << cost << " s" << endl;

        model->termsArray->reset();

        gettimeofday(&end, NULL);
        start_time = t0.tv_usec / 1000000.0;
        end_time = double(end.tv_sec - start.tv_sec) + double(end.tv_usec) / 1000000.0;
        cost = end_time - start_time;
        cout << "total cost time: " << cost << " s" << endl;

        int iter = 0;
        while (iter < iteration) {
            iter++;
            cout << "--------------------------------------------------------------" << endl;
            cout << "split iteration " << iter << endl;

            gettimeofday(&start, NULL);

            voxel_size = (float)(1 << (iteration - iter)) * expectedVoxelSize;
            models[iter - 1]->cut(); // cut
            Model *split_model = splitFromModel(models[iter - 1], voxel_size);
            delete models[iter - 1];
            models[iter - 1] = nullptr;
            models.push_back(split_model);

            split_model->setUniform(uniform);
            split_model->initialize();

            cout << "model size[x, y, z]: [" << split_model->voxelSize.x() << ", " << split_model->voxelSize.y() << ", "
                    << split_model->voxelSize.z() << "]" << endl;
            cout << "current voxel size: " << voxel_size << endl;
            cout << "voxel number: " << split_model->numVoxels << endl;
            cout << "expected voxel size: " << expectedVoxelSize << endl;

            gettimeofday(&t0, NULL);
            split_model->backProjection(sfs::FAST);
            gettimeofday(&t1, NULL);
            start_time = t0.tv_usec / 1000000.0;
            end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
            cost = end_time - start_time;
            cout << "back projection cost time: " << cost << " s" << endl;

            split_model->updateSfISError();
            cout << "sfis error: " << split_model->sfisError / 127 << endl;

            gettimeofday(&t0, NULL);
            split_model->localMinSearch();
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

        if (mode == CORRECTION && models[iteration]->sfisError >= last_turn_error) {
            cout << "correction finished in " << turn << " turns" << endl;
            cout << "final error: " << last_turn_error / 127 << endl;
            break;
        }

        cout << "writing ply file..." << endl;
        gettimeofday(&t0, NULL);
        iodata::writePly(directory, models[iteration]);
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
        models[iteration]->writeSIE(directory);
        models[iteration]->getContourPoints(directory);
        //    model->write_projection(directory);

        if (mode == CORRECTION) {
            cout << "correspond..." << endl;
            gettimeofday(&t0, NULL);
            models[iteration]->correspond();
            gettimeofday(&t1, NULL);
            start_time = t0.tv_usec / 1000000.0;
            end_time = double(t1.tv_sec - t0.tv_sec) + double(t1.tv_usec) / 1000000.0;
            cost = end_time - start_time;
            cout << "correspond cost time: " << cost << " s" << endl;

//            Correction::correction(uniform, models[iteration]->matches3D2D, models[iteration]->matches2D2D, models[iteration]->vertices, SfS::EULER_ANGLE);
            turn++;
            continue;
        } else {
            break;
        }
    }
}

void SfIS::initializeSelfAdaptive() {
    cout << "initialize self adaptive iteration params..." << endl;
    assert(!cameras.empty());
    float pixel_res = rad2deg(3600.0f * cameras[0]->FovH / (float)cameras[0]->x); // convert to arcsec
    cout << "pixel resolution: " << pixel_res << " arcsec" << endl;

    float3 center = (boundingBox.minPoint + boundingBox.maxPoint) * 0.5f;
    float4 box_center;
    box_center << center, 1.0f;
    float avg_size = boundingBox.boundingBoxSize.lpNorm<2>();
    float max_distance = 0;

    float4 tmp(0, 0, 0, 1.0f);
    for (auto camera: cameras) {
        float dis = (camera->matrixView.inverse() * tmp - box_center).lpNorm<2>();
        if (max_distance < dis) {
            max_distance = dis;
        }
    }
    float estimate_min_res = rad2deg(3600.0f * expectedVoxelSize / (max_distance + avg_size * 0.5f));
    cout << "estimate min resolution: " << estimate_min_res << " arcsec" << endl;

    cout << "itr: " << logf(20.0f * pixel_res / estimate_min_res) / logf(2.0f) << endl;
    int itr = (int)(logf(20.0f * pixel_res / estimate_min_res) / logf(2.0f));
    iteration = itr < 0 ? 0 : itr;
    cout << "self adaptive iteration:" << iteration << endl;
}

void SfIS::calculateReferContours() {
    // get reference
    int camera_order = 0;
    int contours_count = 0;

    for (const auto &camera: cameras) {
        index_t offset = camera_order * camera->x * camera->y;
        camera_order++;

        cv::Mat src = cv::Mat(camera->y, camera->x, CV_8U);
        for (index_t i = 0; i < camera->x * camera->y; i++) {
            src.data[i] = referImage->data[offset + i] > 0 ? 255 : 0;
        }
        vector<vector<cv::Point>> contours;
        vector<cv::Vec4i> hierachy;
        cv::findContours(src, contours, hierachy, cv::RETR_EXTERNAL, 1, cv::Point(0, 0));
        if (!contours.empty()) {
            contours_count++;
            for (auto &point: contours[0]) {
                referImage->contours[offset + point.y * camera->x + point.x] = true;
            }
        }
    }

    //    cout << "contours count: " << contours_count << endl;
    //    iodata::write_refer_contours(*refer_image);
}

Model *SfIS::splitFromModel(Model *model, float voxelSize) {
    Model *child = new Model((index_t)(boundingBox.boundingBoxSize.x() / voxelSize),
            (index_t)(boundingBox.boundingBoxSize.y() / voxelSize),
            (index_t)(boundingBox.boundingBoxSize.z() / voxelSize), boundingBox.minPoint, voxelSize);

    model->propagateLabels(child);
    return child;
}
