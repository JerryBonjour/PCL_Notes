cmake_minimum_required(VERSION 2.8.3)
project(pcl_test)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        pcl_ros
        roscpp
        sensor_msgs
        )

#添加三个包到CATKIN_DEPENDS
catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

#导入pcl库
link_directories(${PCL_LIBRARY_DIRS})
add_executable(${PROJECT_NAME}_node src/pcl_test_node.cpp src/pcl_test_core.cpp)
#链接pcl库
target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        )
