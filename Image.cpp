//
// Created by 闻永言 on 2021/9/22.
//

#include "Image.hpp"
#include <cassert>

Image::Image(int _x, int _y, int _num_camera):x(_x), y(_y), num_camera(_num_camera)
{
    int size = x * y * num_camera;
    data = new unsigned char[size];
    coefficients = new int8_t[size];
    contours = new bool[size];
    for (int i = 0; i < size; i++)
    {
        contours[i] = false;
    }
}

Image::~Image()
{
    delete[] data;
    delete[] coefficients;
    delete[] contours;
}

void Image::set_data(int offset, const unsigned char *d) const
{
    int base = offset * x * y;
    for (int i = base, j = 0; i < base + x * y; i++, j++)
    {
        data[i] = d[j];
        int prob_occ = (int)d[j] / 2;
        int prob_empty = (255 - (int)d[j]) / 2;
        coefficients[i] = prob_occ - prob_empty;
    }
}

Image Image::operator-(Image &img)
{
    assert(x == img.x && y == img.y && num_camera == img.num_camera);

}