//
// Created by catalina on 19-4-26.
//

#include "../include/plane_fitting_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ground_plane_fitting");

    ros::NodeHandle nh("~");

    PlaneGroundFilter core(nh);
    return 0;
}