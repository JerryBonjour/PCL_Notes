#include "../include/pcl_test_core.h"



void PclTestCore::point_cb(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){
//    定义两个点云指针
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    /*pcl::PointCloud<T>是最基本的一种数据结构，它表示一块点云数据（点的集合），我们可以指定点的数据结构，
     * 在上述实例中，采用了pcl::PointXYZI这种类型的点。pcl::PointXYZI结构体使用(x, y, z, intensity)这四个数值来描述一个三维度空间点。
     * */


//    通常使用sensor_msgs/PointCloud2.h 做为点云数据的消息格式，
//    可使用pcl::fromROSMsg和pcl::toROSMsg将sensor_msgs::PointCloud2与pcl::PointCloud<T>进行转换。
    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

//    Voxel Grid Filter对原始点云进行降采样
    pcl::VoxelGrid<pcl::PointXYZI> vg;

    vg.setInputCloud(current_pc_ptr);
    vg.setLeafSize(0.2f, 0.2f, 0.2f);//使用0.2的立方体进行分割，小立方体的形心来表示此立方体的所有点，保留这些点作为降采样的输出
    vg.filter(*filtered_pc_ptr);

    sensor_msgs::PointCloud2 pub_pc;
    pcl::toROSMsg(*filtered_pc_ptr, pub_pc);

    pub_pc.header = in_cloud_ptr->header;

    pub_filtered_points_.publish(pub_pc);
}


PclTestCore::PclTestCore(ros::NodeHandle &nh){

//    订阅点云话题，降采样后发布到filtered_points
    sub_point_cloud_ = nh.subscribe("/velodyne_points",10, &PclTestCore::point_cb, this);

    pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 10);

    ros::spin();
}

PclTestCore::~PclTestCore(){}

void PclTestCore::Spin(){

}
