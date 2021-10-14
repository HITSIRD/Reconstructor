//
// Created by 闻永言 on 2021/9/22.
//

#include "iodata.hpp"
#include <fstream>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace std;

void iodata::read_config(
        const string &file_name, std::vector<Camera *> &camera, Image *refer_image, BoundingBox &bounding_box,
        float &voxel_size, int &iteration)
{
    cout << "configuration: " << file_name << endl;
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

    in.open(file_name.c_str());
    if (!in.is_open())
    {
        cerr << "FAIL TO OPEN FILE" << endl;
        throw exception();
    }

    in >> num_camera;
    cout << "camera number: " << num_camera << endl;

    // load bounding box
    in >> x >> y >> z;
    bounding_box.min_point << x, y, z;
    in >> x >> y >> z;
    bounding_box.max_point << x, y, z;
    bounding_box.bounding_box_size = bounding_box.max_point - bounding_box.min_point;

    in >> term;
    if (term == "VOXEL_SIZE")
    {
        in >> voxel_size;
    } else
    {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    in >> term;
    if (term == "ITERATION")
    {
        in >> iteration;
    } else
    {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    // load camera inner parameters
    in >> pixel_x >> pixel_y >> ccd_size_x >> ccd_size_y >> focal;

    refer_image = new Image(pixel_x, pixel_y, num_camera);

    for (int i = 0; i < num_camera; i++)
    {
        in >> image_file;
        cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        cout << "image " << i << ": " << image_file << endl;
        refer_image->set_data(i, image.data);

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
        c->set_viewport(pixel_x, pixel_y, ccd_size_x, ccd_size_y, focal);
        c->set_look_at(camera_center, focal_center, up);
        camera.push_back(c);
    }
    in.close();

    cout << "image size: " << pixel_x << "x" << pixel_y << endl;
}

void iodata::read_config(const string &voxel_config, const string &config, SfIS &sfis)
{
    cout << "voxel configuration: " << voxel_config << endl;
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
    in.open(voxel_config.c_str());
    if (!in.is_open())
    {
        cerr << "FAIL TO OPEN FILE" << endl;
        throw exception();
    }
    // load bounding box
    in >> x >> y >> z;
    sfis.bounding_box.min_point << x, y, z;
    in >> x >> y >> z;
    sfis.bounding_box.max_point << x, y, z;
    sfis.bounding_box.bounding_box_size = sfis.bounding_box.max_point - sfis.bounding_box.min_point;

    in >> term;
    if (term == "VOXEL_SIZE")
    {
        in >> sfis.initial_voxel_size;
    } else
    {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    in >> term;
    if (term == "ITERATION")
    {
        in >> sfis.iteration;
    } else
    {
        cerr << "FILE FORMAT ERROR" << endl;
        throw exception();
    }

    in.close();

    // load image configuration
    in.open(config.c_str());
    if (!in.is_open())
    {
        cerr << "FAIL TO OPEN FILE" << endl;
        throw exception();
    }

    in >> num_camera;
    cout << "camera number: " << num_camera << endl;

    // load camera inner parameters
    in >> pixel_x >> pixel_y >> ccd_size_x >> ccd_size_y >> focal;

    sfis.refer_image = new Image(pixel_x, pixel_y, num_camera);

    for (int i = 0; i < num_camera; i++)
    {
        in >> image_file;
        cv::Mat image = cv::imread(image_file, cv::IMREAD_GRAYSCALE);
        cv::threshold(image, image, 1, 255, cv::THRESH_BINARY);
        cout << "image " << i << ": " << image_file << endl;
        sfis.refer_image->set_data(i, image.data);

        in >> view_00 >> view_01 >> view_02;
        in >> view_10 >> view_11 >> view_12;
        in >> view_20 >> view_21 >> view_22;
        in >> view_03 >> view_13 >> view_23;

        float3x3 R;
        R << view_00, view_01, view_02, view_10, view_11, view_12, view_20, view_21, view_22;
        float3 t(view_03, view_13, view_23);

        auto c = new Camera();
        c->set_viewport(pixel_x, pixel_y, ccd_size_x, ccd_size_y, focal);
        c->set_look_at(R, t);
        sfis.cameras.push_back(c);
    }
    in.close();

    cout << "image size: " << pixel_x << "x" << pixel_y << endl;
}

void iodata::write_ply(const string &directory, Model *model)
{
//    string command = "touch " + directory + "/model.ply";
//    system(command.c_str());

    string outfile = directory + "/model.ply";
    cout << "writing to " << outfile << endl;

    vector<Vertex> vertices;
    vector<Triangle> triangles;
    model->convert_polygon_mesh(vertices, triangles);
    std::ofstream out;
    out.open(outfile.c_str());
    if (!out.is_open())
    {
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

    for (const auto &vertex: vertices)
    {
        out << vertex.world.x() << " " << vertex.world.y() << " " << vertex.world.z() << endl;
    }

    for (const auto &triangle: triangles)
    {
        out << "3 " << triangle.vertex_0 << " " << triangle.vertex_1 << " " << triangle.vertex_2 << endl;
    }

    out.close();
}

void iodata::write_projection(const string &directory, const Image &image)
{
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/Projection" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.num_camera; i++)
    {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/Projection/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8U);

        index_t size = image.x * image.y;
        index_t offset = i * size;
        for (index_t j = 0; j < size; j++)
        {
            img.data[j] = image.data[offset + j];
        }
        cv::imwrite(file_name, img);
    }
}

void iodata::write_test(const string &directory, const Image &image)
{
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/Test" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.num_camera; i++)
    {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/Test/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8U);

        index_t size = image.x * image.y;
        index_t offset = i * size;
        for (index_t j = 0; j < size; j++)
        {
            img.data[j] = image.data[offset + j];
        }
        cv::imwrite(file_name, img);
    }
}

void iodata::write_sie(const string &directory, Image &image, Image &refer)
{
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/sie" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.num_camera; i++)
    {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/sie/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8UC3);

        int size = image.x * image.y;
        int offset = i * size;

        auto *p = (uint8_t *)img.data;

        int index = offset;
        for (int j = 0; j < size; j++, index++)
        {
            if (image.data[index] > 0 && refer.coefficients[index] > 0)
            {
                *p = 255; // B
                p++;
                *p = 255; // G
                p++;
                *p = 255; // R
                p++;
            } else if (image.data[index] == 0 && refer.coefficients[index] < 0)
            {
                *p = 0; // B
                p++;
                *p = 0; // G
                p++;
                *p = 0; // R
                p++;
            } else if (image.data[index] > 0 && refer.coefficients[index] < 0)
            {
                *p = 0; // B
                p++;
                *p = 0; // G
                p++;
                *p = 255; // R
                p++;
            } else if (image.data[index] == 0 && refer.coefficients[index] > 0)
            {
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

void iodata::write_refer_contours(const string &directory, const Image &image)
{
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/Reference contours" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.num_camera; i++)
    {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/Reference contours/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8U);

        index_t size = image.x * image.y;
        index_t offset = i * size;
        for (index_t j = 0; j < size; j++)
        {
            img.data[j] = image.data[offset + j];
        }
        cv::imwrite(file_name, img);
    }
}

void iodata::write_recon_contours(const string &directory, const Image &image)
{
    string equation = "\"";
    string command = "mkdir -p " + equation + directory + "/Reconstruction contours" + equation;
    system(command.c_str());
    stringstream ss;
    string number;
    for (int i = 0; i < image.num_camera; i++)
    {
        ss.clear();
        ss << setw(4) << setfill('0') << i;
        ss >> number;
        string file_name = directory + "/Reconstruction contours/img_" + number + ".png";
        cv::Mat img = cv::Mat(image.y, image.x, CV_8U);

        index_t size = image.x * image.y;
        index_t offset = i * size;
        for (index_t j = 0; j < size; j++)
        {
            img.data[j] = image.data[offset + j];
        }
        cv::imwrite(file_name, img);
    }
}
