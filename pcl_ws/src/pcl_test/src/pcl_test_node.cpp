//
// Created by catalina on 19-4-24.
//
/*
 * ros实现点云降采样并输出
 *
 * */

#include "../include/pcl_test_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_test");

    ros::NodeHandle nh;

    PclTestCore core(nh);
    return 0;
}
