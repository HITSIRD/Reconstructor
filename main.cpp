#include <iostream>
#include "SfIS.hpp"

using namespace std;

int main(int argc, char *argv[])
{
    if (argc > 3)
    {
        string voxel_config = argv[1]; // model file name
        string config = argv[2]; // model file name
        SfS::MODE mode = (SfS::MODE)atoi(argv[3]); // calibration correction mode

        auto *sfs = new SfIS(voxel_config, config);
        sfs->render(mode);
        delete sfs;
        return 0;
    }
}
