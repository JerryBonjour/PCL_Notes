cmake_minimum_required(VERSION 3.5.1)
project(pcl_test)

set(CMAKE_CXX_STANDARD 11)

#手动添加pcl包
find_package(PCL 1.7 REQUIRED)


#利用CMake的宏完成对PCL的头文件路径和链接路径变量的配置和添加，如果缺少下面几行，生成文件的过程中就会提示
#找不到相关的头文件，在配置CMake时，当找到了安装的PCL，下面相关的包含的头文件，链接库，路径变量就会自动设置
#                    PCL_FOUND:如果找到了就会被设置为1 ，否则就不设置
#                    PCL_INCLUDE_DIRS:被设置为PCL安装的头文件和依赖头文件的目录
#                    PCL_LIBRARIES:被设置成所建立和安装的PCL库头文件
#                    PCL_LIBRARIES_DIRS:被设置成PCL库和第三方依赖的头文件所在的目录
#                    PCL_VERSION:所找到的PCL的版本
#                    PCL_COMPONENTS:列出所有可用的组件
#                    PCL_DEFINITIONS:列出所需要的预处理器定义和编译器标志


#添加并链接lib
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#CMake从单个源文件pcl_test建立一个可执行文件
add_executable(pcl_test main.cpp)

#编译器知道我们现在访问所用的方法，还需要让链接器知道所链接的库，PCL找到库文件由
#PCL_COMMON_LIBRARIES变量指示，通过target_link_libraries这个宏来出发链接操作
target_link_libraries(pcl_test ${PCL_LIBRARIES})
#install(TARGETS pcl_test RUNTIME DESTINATION bin)
