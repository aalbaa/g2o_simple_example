cmake_minimum_required(VERSION 3.10)

set(G2O_DIR /home/aalbaali/Documents/Code_base/External/cxx/g2o)
list( APPEND CMAKE_MODULE_PATH ${G2O_DIR}/cmake_modules )
set (G2O_ROOT  G2O_DIR)

set(BUILD_SHARED_LIBS ON)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(G2O REQUIRED)

# # add_definitions( -D_USE_MATH_DEFINES)

# # Specify C++ version
set (CMAKE_CXX_STANDARD 14)

include_directories(${G2O_DIR})
# Add library
add_library(tutorial_slam2d_library ${G2O_LIB_TYPE}
  edge_se2_pointxy.cpp  se2.h                vertex_point_xy.h
  edge_se2.cpp    edge_se2_pointxy.h    vertex_se2.cpp
  edge_se2.h      vertex_point_xy.cpp  vertex_se2.h
  parameter_se2_offset.h parameter_se2_offset.cpp
  types_tutorial_slam2d.h types_tutorial_slam2d.cpp
  g2o_tutorial_slam2d_api.h
)

# # set_target_properties(tutorial_slam2d_library PROPERTIES OUTPUT_NAME ${LIB_PREFIX}tutorial_slam2d)

# target_link_libraries(tutorial_slam2d_library g2o::core g2o::solver_eigen Eigen3::solver_eigen)
target_link_libraries(tutorial_slam2d_library     
  g2o_core g2o_types_slam3d g2o_solver_csparse g2o_stuff g2o_csparse_extension g2o_types_sba
  Eigen3::Eigen
)

add_executable(tutorial_slam2d
  tutorial_slam2d.cpp
)

# set_target_properties(tutorial_slam2d PROPERTIES OUTPUT_NAME tutorial_slam2d${EXE_POSTFIX})

target_link_libraries(tutorial_slam2d tutorial_slam2d_library)
