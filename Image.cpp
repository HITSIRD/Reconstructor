//
// Created by 闻永言 on 2021/9/22.
//

#include "Image.hpp"

using namespace sfs;

Image::Image(int _x, int _y, int _numCameras): x(_x), y(_y), numCameras(_numCameras) {
    int size = x * y * numCameras;
    data = new unsigned char[size];
    coefficients = new int8_t[size];
    contours = new bool[size];
    for (int i = 0; i < size; i++) {
        data[i] = 0;
        contours[i] = false;
    }
}

Image::~Image() {
    delete[] data;
    delete[] coefficients;
    delete[] contours;
}

void Image::setData(int offset, const unsigned char *d) const {
    int base = offset * x * y;
    for (int i = base, j = 0; i < base + x * y; i++, j++) {
        data[i] = d[j];
        int prob_occ = (int)d[j] / 2;
        int prob_empty = (255 - (int)d[j]) / 2;
        coefficients[i] = prob_occ - prob_empty;
    }
}
