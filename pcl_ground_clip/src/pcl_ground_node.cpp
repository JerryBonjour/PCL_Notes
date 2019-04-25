//
// Created by catalina on 19-4-25.
//

#include "../include/pcl_ground_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ground");

    ros::NodeHandle nh;

    PclGroundCore core(nh);
    // core.Spin();
    return 0;
}