//
// Created by 闻永言 on 2022/1/4.
//

#include "Uniform.hpp"

using namespace sfs;
using namespace std;

Uniform::Uniform(): cameras(nullptr) {}

Uniform::Uniform(index_t _numPixels): numPixels(_numPixels) {}

Uniform::~Uniform() = default;

void Uniform::setCameras(vector<sfs::Camera *> *camerasPointer) {
    cameras = camerasPointer;
}

void Uniform::setReferImage(sfs::Image *refer) {
    referImage = refer;
}
