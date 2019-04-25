/*
 * 点云过滤
 *
 * */
#include "../include/pcl_clip_core.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcl_ground_clip2");

    ros::NodeHandle nh;
    PclClipCore core(nh);
    return 0;
}