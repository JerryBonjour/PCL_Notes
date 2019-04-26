//
// Created by catalina on 19-4-25.
//

#ifndef PCL_GROUND_CLIP_PCL_GROUND_CORE_H
#define PCL_GROUND_CLIP_PCL_GROUND_CORE_H


#pragma once

#include <ros/ros.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/PointCloud2.h>

#define CLIP_HEIGHT 0.2 //截取掉高于雷达自身0.2米的点
#define MIN_DISTANCE 2.4
#define RADIAL_DIVIDER_ANGLE 0.18 //360度均分成2000条射线
#define SENSOR_HEIGHT 1.78 //雷达的高度

#define concentric_divider_distance_ 0.01 //0.1 meters default
#define min_height_threshold_ 0.05 //最小高度阀值
#define local_max_slope_ 8   //相邻两点的坡度阀值rad  max slope of the ground between points, degree
#define general_max_slope_ 5 //整个地面的坡度阀值 max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2

class PclGroundCore
{

private:
    ros::Subscriber sub_point_cloud_;

    ros::Publisher pub_ground_, pub_no_ground_;

//  方便半径和夹角表示，使用以下结构代替pcl::PointCloudXYZI
    struct PointXYZIRTColor
    {
        pcl::PointXYZI point;

        float radius; // 点到雷达的水平距离 cylindric coords on XY Plane
        float theta;  //相对于x方向的夹角 angle deg on XY plane

        size_t radial_div;     //角度微分 index of the radial divsion to which this point belongs to
        size_t concentric_div; //距离微分 index of the concentric division to which this points belongs to

        size_t original_index; //index of this point in the source pointcloud
    };
    typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

    size_t radial_dividers_num_; //射线数量
    size_t concentric_dividers_num_;

    void point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud);

//  点云裁剪
    void clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);
//  过滤近点的点云
    void remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

    void XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                          PointCloudXYZIRTColor &out_organized_points,
                          std::vector<pcl::PointIndices> &out_radial_divided_indices,
                          std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);
//  判断是否为地面点
    void classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                     pcl::PointIndices &out_ground_indices,
                     pcl::PointIndices &out_no_ground_indices);
//  重新发布话题
    void publish_cloud(const ros::Publisher &in_publisher,
                       const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                       const std_msgs::Header &in_header);

public:
    PclGroundCore(ros::NodeHandle &nh);
    ~PclGroundCore();
    void Spin();
};

#endif //PCL_GROUND_CLIP_PCL_GROUND_CORE_H
