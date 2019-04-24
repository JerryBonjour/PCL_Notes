//
// Created by catalina on 19-4-24.
//

#include "../include/pcl_filter_core.h"

PclFilterCore::~PclFilterCore() {}
void PclFilterCore::spin() {}

//回调函数
void PclFilterCore::point_cb(const sensor_msgs::PointCloud2ConstPtr in_cloud) {

    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr (new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud, *current_pc_ptr);
    pcl::VoxelGrid<pcl::PointXYZI> vg;

    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);
    vg.filter(*filtered_pc_ptr);

//    转换成PointCloud2并发布
    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg (*filtered_pc_ptr, pub_pc);

    pub_pc.header = in_cloud->header;//时间戳保持一致

    pub_filter_points_.publish(pub_pc);

}


PclFilterCore::PclFilterCore(ros::NodeHandle &nh) {

    sub_point_cloud_ = nh.subscribe("/vlp16_0/velodyne_points", 10, &PclFilterCore::point_cb, this);
    pub_filter_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_velodyne_points", 10);
    ros::spin();
}
