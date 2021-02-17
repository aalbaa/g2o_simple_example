cmake_minimum_required(VERSION 3.10)
project( g2o_tutorial)
# Specify g2o's root directory (where it was cloned)
set(G2O_ROOT /home/aalbaali/Documents/Code_base/External/cxx/g2o)
list( APPEND CMAKE_MODULE_PATH ${G2O_ROOT}/cmake_modules )

# This flag is needed sometimes (without it, the executable doesn't run as expected)
# set(BUILD_SHARED_LIBS ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(G2O REQUIRED)

# Specify C++ version (minimum 14 required)
set (CMAKE_CXX_STANDARD 17)
# Include the g2o directory
include_directories(${G2O_ROOT})

# Build library
add_library(tutorial_slam2d_library ${G2O_LIB_TYPE}
  se2.h edge_se2.cpp    vertex_se2.cpp
  edge_se2.h vertex_se2.h
  parameter_se2_offset.h parameter_se2_offset.cpp
  types_tutorial_slam2d.h types_tutorial_slam2d.cpp
  g2o_tutorial_slam2d_api.h
)

# Link g2o libraries (I didn't link all of them)
target_link_libraries(tutorial_slam2d_library     
  g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
  Eigen3::Eigen
)

add_executable(tutorial_slam2d
  tutorial_slam2d.cpp
)

# Link the local library to the executable
target_link_libraries(tutorial_slam2d tutorial_slam2d_library)
