#ifndef SRC_PLANE_FITTING_CORE_H
#define SRC_PLANE_FITTING_CORE_H

#pragma once

#include <ros/ros.h>

// For disable PCL complile lib, to use PointXYZIR
#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

// using eigen lib
#include <Eigen/Dense>

#include <sensor_msgs/PointCloud2.h>

namespace velodyne_pointcloud
{
    struct PointXYZIR
    {
        PCL_ADD_POINT4D;                // quad-word XYZ
        float intensity;                // laser intensity reading
        uint16_t ring;                  // laser ring number
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
    } EIGEN_ALIGN16;

};
//增加新的PointT类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

namespace plane_ground_filter
{
    struct PointXYZIRL
    {
        PCL_ADD_POINT4D;                // quad-word XYZ
        float intensity;                // laser intensity reading
        uint16_t ring;                  // laser ring number
        uint16_t label;                 // point label
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
    } EIGEN_ALIGN16;

};

#define SLRPointXYZIRL plane_ground_filter::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>

//增加新的PointT类型
POINT_CLOUD_REGISTER_POINT_STRUCT(plane_ground_filter::PointXYZIRL, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(uint16_t, label, label))

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

class PlaneGroundFilter {
private:
    ros::Subscriber sub_point_cloud_;
    ros::Publisher pub_ground_, pub_no_ground_, pub_all_points_;
    std::string point_topic_;

    int sensor_model_;
    double sensor_height_, clip_height_, min_distance_, max_distance_;//传感器高度、要裁剪的高度范围、距离远近
    int num_seq_ = 1;
    int num_iter_, num_lpr_;
    double th_seeds_, th_dist_;

    float d_, th_dist_d_;
    MatrixXf normal_;

    pcl::PointCloud<VPoint>::Ptr g_seeds_pc;
    pcl::PointCloud<VPoint>::Ptr g_ground_pc;
    pcl::PointCloud<VPoint>::Ptr g_not_ground_pc;
    pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc;

// 平面模型估计
    void estimate_plane_(void);
//    提取种子点
    void extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted);
//    点云处理过程合计
    void post_process(const pcl::PointCloud<VPoint>::Ptr in, pcl::PointCloud<VPoint>::Ptr out);
// 回调函数
    void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);
//
    void clip_above(const pcl::PointCloud<VPoint>::Ptr in,const pcl::PointCloud<VPoint>::Ptr out);
//    移除近距离，远距离噪点
    void remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in,const pcl::PointCloud<VPoint>::Ptr out);
public:
    PlaneGroundFilter(ros::NodeHandle &nh);
    ~PlaneGroundFilter();
    void Spin();
};



#endif //SRC_PLANE_FITTING_CORE_H
