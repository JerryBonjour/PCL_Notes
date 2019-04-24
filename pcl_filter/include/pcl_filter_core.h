//
// Created by catalina on 19-4-24.
//

//#ifndef PCL_FILTER_PCL_FILTER_CORE_H
//#define PCL_FILTER_PCL_FILTER_CORE_H
#pragma once

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>

class PclFilterCore {
    private:
        ros::Subscriber sub_point_cloud_;
        ros::Publisher pub_filter_points_;
        
        void point_cb (const sensor_msgs::PointCloud2ConstPtr in_cloud);
    public:
        PclFilterCore(ros::NodeHandle &nh);
        ~PclFilterCore();
        void spin();
};


//#endif //PCL_FILTER_PCL_FILTER_CORE_H
