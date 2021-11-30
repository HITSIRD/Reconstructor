cmake_minimum_required(VERSION 3.20)
project(Reconstructor)

set(CMAKE_CXX_STANDARD 20)

SET(CMAKE_CXX_COMPILER /opt/homebrew/Cellar/llvm/13.0.0_1/bin/clang-13)
#SET(CMAKE_CXX_COMPILER /opt/homebrew/Cellar/gcc/11.2.0/bin/aarch64-apple-darwin20-g++-11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(/opt/homebrew/include/eigen3)

#find_library(ceres REQUIRED)
include_directories(/opt/homebrew/Cellar/ceres-solver/2.0.0_4/include)
include_directories(/opt/homebrew/Cellar/glog/0.5.0/include)
include_directories(/opt/homebrew/Cellar/gflags/2.2.2/include)
#include_directories(include)

aux_source_directory(. SRC)

#set(OpenMP_CXX_FLAGS -fopenmp)
#set(OpenMP_CXX_LIB_NAMES libomp)
#
#find_package(OpenMP REQUIRED)
#if (OPENMP_FOUND)
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -O3")
#    link_directories(/opt/homebrew/Cellar/libomp/13.0.0/lib)
#endif ()

add_executable(Reconstructor ${SRC})

target_link_libraries(Reconstructor /opt/homebrew/Cellar/llvm/13.0.0_1/lib/libc++.dylib)
target_link_libraries(Reconstructor ${OpenCV_LIBRARIES})
target_link_libraries(Reconstructor /opt/homebrew/Cellar/ceres-solver/2.0.0_4/lib/libceres.dylib)
target_link_libraries(Reconstructor /opt/homebrew/Cellar/gflags/2.2.2/lib/libgflags.dylib)
target_link_libraries(Reconstructor /opt/homebrew/Cellar/glog/0.5.0/lib/libglog.dylib)
