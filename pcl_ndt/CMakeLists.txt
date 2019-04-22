cmake_minimum_required(VERSION 3.13)
project(pcl_ndt)

set(CMAKE_CXX_STANDARD 11)
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_ndt pcl_ndt1.cpp)
target_link_libraries(pcl_ndt ${PCL_LIBRARIES})