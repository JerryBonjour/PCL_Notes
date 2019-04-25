//
// Created by catalina on 19-4-24.
//

//#ifndef PCL_FILTER_PCL_FILTER_CORE_H
//#define PCL_FILTER_PCL_FILTER_CORE_H
#pragma once
#include <iostream>

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/extract_indices.h>


class PclClipCore {
    private:
        ros::Subscriber sub_point_cloud_;
        ros::Publisher pub_filter_points_;
        
        void point_cb (const sensor_msgs::PointCloud2ConstPtr input);
    public:
        PclClipCore(ros::NodeHandle &nh);
        ~PclClipCore();
        void spin();
};


//#endif //PCL_FILTER_PCL_FILTER_CORE_H
