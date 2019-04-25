cmake_minimum_required(VERSION 2.8.3)
project(pcl_ground_clip2)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pcl_ros roscpp rospy sensor_msgs
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

#导入pcl
link_directories(${PCL_LIBRARY_DIRS})
add_executable(${PROJECT_NAME}_node src/pcl_clip_node.cpp src/pcl_clip_core.cpp)
#链接pcl库
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )