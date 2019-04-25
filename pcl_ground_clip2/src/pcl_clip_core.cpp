//
// Created by catalina on 19-4-24.
//
/**
 * RANSAC方法切割地面
 */
#include "../include/pcl_clip_core.h"

PclClipCore::~PclClipCore() {}
void PclClipCore::spin() {}

//回调函数
void PclClipCore::point_cb(const sensor_msgs::PointCloud2ConstPtr input) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*input,*cloud);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // pcl::ModelCoefficients coefficients;   //申明模型的参数
    // pcl::PointIndices inliers;             //申明存储模型的内点的索引
    // 创建一个分割方法
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 这一句可以选择最优化参数的因子
    seg.setOptimizeCoefficients (true);

    seg.setModelType (pcl::SACMODEL_PLANE);   //平面模型
    seg.setMethodType (pcl::SAC_RANSAC);    //分割平面模型所使用的分割方法RANSAC
    seg.setDistanceThreshold (0.2);        //设置最小的阀值距离

    seg.setInputCloud (cloud);   //设置输入的点云
    seg.segment (*inliers,*coefficients);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model");
    }

    // 提取地面点 -> ros发布出去
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloud_filtered);

    //再rviz上显示所以要转换回PointCloud2
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_filtered,output);
    output.header = input->header;//时间戳保持一致
    pub_filter_points_.publish(output);
}


PclClipCore::PclClipCore(ros::NodeHandle &nh) {

    sub_point_cloud_ = nh.subscribe("/velodyne_points", 10, &PclClipCore::point_cb, this);
    pub_filter_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_velodyne_points", 10);
    ros::spin();
}
