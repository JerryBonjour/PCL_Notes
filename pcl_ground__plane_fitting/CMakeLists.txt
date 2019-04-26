cmake_minimum_required(VERSION 2.8.3)
project(pcl_ground__plane_fitting)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(PCL 1.7 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pcl_ground__plane_fitting
#  CATKIN_DEPENDS pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(${PROJECT_NAME}_node src/plane_fitting_core.cpp src/plane_fitting_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
