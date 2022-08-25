//
// Created by 闻永言 on 2021/9/22.
//

#include "iodata.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;
using namespace sfs;

void iodata::readConfig(
        const string &fileName, std::vector<Camera *> &camera, Image *referImage, BoundingBox &boundingBox,
        float &voxelSize, int &iteration) {
    cout << "configuration: " << fileName << endl;
    ifstream in;
    int num_camera;
    float x, y, z;
    int pixel_x, pixel_y;
    float ccd_size_x, ccd_size_y, focal;
    string image_file;
    float camera_center_x, camera_center_y, camera_center_z;
    float focal_center_x, focal_center_y, focal_center_z;
    float up_x, up_y, up_z;

    string term;

    in.open(fileName.c_str());
    if (!in.is_open()) {
        cerr << "FAIL TO OPEN FILE" << endl;
        throw exception();
    }

    in >> num_camera;
    cout << "camera number: " << num_camera << endl;

    // load bounding box
    in >> x >> y >> z;
    boundingBox.minPoint << x, y, z;
    in >> x >> y >> z;
    boundingBox.maxPoint << x, y, z;
    boundingBox.boundingBoxSize = boundingBox.maxPoint - boundingBox.minPoint;

    in >> term;
    if (term == "VOXEL_SIZE") {
        in >> voxelSize;
    } else {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    in >> term;
    if (term == "ITERATION") {
        in >> iteration;
    } else {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    // load camera inner parameters
    in >> pixel_x >> pixel_y >> ccd_size_x >> ccd_size_y >> focal;

    referImage = new Image(pixel_x, pixel_y, num_camera);

    for (int i = 0; i < num_camera; i++) {
        in >> image_file;
        cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        cout << "image " << i << ": " << image_file << endl;
        referImage->setData(i, image.data);

        in >> camera_center_x >> camera_center_y >> camera_center_z;
        in >> focal_center_x >> focal_center_y >> focal_center_z;
        in >> up_x >> up_y >> up_z;
        cout << camera_center_x << " " << camera_center_y << " " << camera_center_z << endl;
        cout << focal_center_x << " " << focal_center_y << " " << focal_center_z << endl;
        cout << up_x << " " << up_y << " " << up_z << endl;

        auto c = new Camera();
        float4 camera_center(camera_center_x / 100.0f, camera_center_y / 100.0f, camera_center_z / 100.0f, 1.0f);
        float4 focal_center(focal_center_x / 100.0f, focal_center_y / 100.0f, focal_center_z / 100.0f, 1.0f);
        float4 up(up_x, up_y, up_z, 0);
        c->setViewport(pixel_x, pixel_y, ccd_size_x, ccd_size_y, focal);
        c->setLookAt(camera_center, focal_center, up);
        camera.push_back(c);
    }
    in.close();

    cout << "image size: " << pixel_x << "x" << pixel_y << endl;
}

void iodata::readConfig(const string &voxelConfig, const string &config, SfIS &sfis) {
    cout << "voxel configuration: " << voxelConfig << endl;
    cout << "image configuration: " << config << endl;
    ifstream in;
    int num_camera;
    float x, y, z;
    int pixel_x, pixel_y;
    float ccd_size_x, ccd_size_y, focal;
    string image_file;
    float view_00, view_01, view_02, view_10, view_11, view_12, view_20, view_21, view_22, view_03, view_13, view_23;
    string term;

    // load voxel configuration
    in.open(voxelConfig.c_str());
    if (!in.is_open()) {
        cerr << "FAIL TO OPEN FILE" << endl;
        throw exception();
    }
    // load bounding box
    in >> x >> y >> z;
    sfis.boundingBox.minPoint << x, y, z;
    in >> x >> y >> z;
    sfis.boundingBox.maxPoint << x, y, z;
    sfis.boundingBox.boundingBoxSize = sfis.boundingBox.maxPoint - sfis.boundingBox.minPoint;

    in >> term;
    if (term == "VOXEL_SIZE") {
        in >> sfis.expectedVoxelSize;
    } else {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    in >> term;
    if (term == "ITERATION") {
        in >> sfis.iteration;
    } else {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    in >> term;
    if (term == "SAI") {
        in >> term;
        if (term == "ON") {
            sfis.selfAdaptive = true;
        } else if (term == "OFF") {
            sfis.selfAdaptive = false;
        } else {
            cerr << "FILE FORMAT ERROR" << endl;
            throw exception();
        }
    } else {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    in.close();

    // load image configuration
    in.open(config.c_str());
    if (!in.is_open()) {
        cerr << "FAIL TO OPEN FILE" << endl;
        throw exception();
    }

    in >> num_camera;
    cout << "camera number: " << num_camera << endl;

    // load camera inner parameters
    in >> pixel_x >> pixel_y >> ccd_size_x >> ccd_size_y >> focal;

    sfis.referImage = new Image(pixel_x, pixel_y, num_camera);

    for (int i = 0; i < num_camera; i++) {
        in >> image_file;
        cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        cv::threshold(image, image, 1, 255, cv::THRESH_BINARY);
        cout << "image " << i << ": " << image_file << endl;
        sfis.referImage->setData(i, image.data);

        in >> view_00 >> view_01 >> view_02;
        in >> view_10 >> view_11 >> view_12;
        in >> view_20 >> view_21 >> view_22;
        in >> view_03 >> view_13 >> view_23;

        float3x3 R;
        R << view_00, view_01, view_02, view_10, view_11, view_12, view_20, view_21, view_22;
        float3 t(view_03, view_13, view_23);

        auto c = new Camera();
        c->setViewport(pixel_x, pixel_y, ccd_size_x, ccd_size_y, focal);
        c->setLookAt(R, t);
        sfis.cameras.push_back(c);
    }
    in.close();

    cout << "image size: " << pixel_x << "x" << pixel_y << endl;
}

void iodata::writePly(const string &directory, Model *model) {
    //    string command = "touch " + directory + "/model.ply";
    //    system(command.c_str());

    string outfile = directory + "/model.ply";
    cout << "writing to " << outfile << endl;

    vector<Vertex> vertices;
    vector<Triangle> triangles;
    model->convertPolygonMesh(vertices, triangles);
    std::ofstream out;
    out.open(outfile.c_str());
    if (!out.is_open()) {
        cerr << "FAIL TO OPEN FILE" << endl;
        throw exception();
    }

    out << "ply" << endl;
    out << "format ascii 1.0" << endl;
    out << "element vertex " << vertices.size() << endl;
    out << "property float x" << endl;
    out << "property float y" << endl;
    out << "property float z" << endl;
    out << "element face " << triangles.size() << endl;
    out << "property list uchar int vertex_indices" << endl;
    out << "end_header" << endl;

    for (const auto &vertex: vertices) {
        out << vertex.position.x() << " " << vertex.position.y() << " " << vertex.position.z() << endl;
    }

    for (const auto &triangle: triangles) {
        out << "3 " << triangle.vertexIndex[0] << " " << triangle.vertexIndex[1] << " " << triangle.vertexIndex[2]
                << endl;
    }

    out.close();
}

void iodata::writeProjection(const string &directory, const Image &image) {
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/Projection" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.numCameras; i++) {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/Projection/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8U);

        index_t size = image.x * image.y;
        index_t offset = i * size;
        for (index_t j = 0; j < size; j++) {
            img.data[j] = image.data[offset + j];
        }
        cv::imwrite(file_name, img);
    }
}

void iodata::writeTest(const string &directory, const Image &image) {
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/Test" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.numCameras; i++) {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/Test/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8U);

        index_t size = image.x * image.y;
        index_t offset = i * size;
        for (index_t j = 0; j < size; j++) {
            img.data[j] = image.data[offset + j];
        }
        cv::imwrite(file_name, img);
    }
}

void iodata::writeSie(const string &directory, const Image &image, const Image &refer) {
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/sie" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    cv::Mat img = cv::Mat(image.y, image.x, CV_8UC3);
    int size = image.x * image.y;

    for (int i = 0; i < image.numCameras; i++) {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/sie/img_" + number + ".png";

        int index = i * size;
        auto *p = img.data;
        for (int j = 0; j < size; j++, index++) {
            if (image.data[index] > 0 && refer.coefficients[index] > 0) {
                *p = 255; // B
                p++;
                *p = 255; // G
                p++;
                *p = 255; // R
                p++;
            } else if (image.data[index] == 0 && refer.coefficients[index] < 0) {
                *p = 0; // B
                p++;
                *p = 0; // G
                p++;
                *p = 0; // R
                p++;
            } else if (image.data[index] > 0 && refer.coefficients[index] < 0) {
                *p = 0; // B
                p++;
                *p = 0; // G
                p++;
                *p = 255; // R
                p++;
            } else if (image.data[index] == 0 && refer.coefficients[index] > 0) {
                *p = 255; // B
                p++;
                *p = 0; // G
                p++;
                *p = 0; // R
                p++;
            }
        }
        cv::imwrite(file_name, img);
    }
}

void iodata::writeReferContours(const string &directory, const Image &image) {
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/Reference contours" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.numCameras; i++) {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/Reference contours/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8U);

        index_t size = image.x * image.y;
        index_t offset = i * size;
        for (index_t j = 0; j < size; j++) {
            img.data[j] = image.data[offset + j];
        }
        cv::imwrite(file_name, img);
    }
}

void iodata::writeReconContours(const string &directory, const Image &image) {
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/Reconstruction contours" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.numCameras; i++) {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/Reconstruction contours/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8U);

        index_t size = image.x * image.y;
        index_t offset = i * size;
        for (index_t j = 0; j < size; j++) {
            img.data[j] = image.data[offset + j];
        }
        cv::imwrite(file_name, img);
    }
}
