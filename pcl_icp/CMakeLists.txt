cmake_minimum_required(VERSION 3.5.1)
project(pcl_icp)

set(CMAKE_CXX_STANDARD 11)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(pcl_icp icp1.cpp pairwise.cpp icp2.cpp icp3.cpp)
target_link_libraries(pcl_icp ${PCL_LIBRARIES})