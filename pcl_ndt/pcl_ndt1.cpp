/**
 * 正态分布点云配准
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/ndt.h> //正态分布配准标准类头文件
#include <pcl/filters/approximate_voxel_grid.h>//滤波类头文件
#include <pcl/visualization/pcl_visualizer.h> //
#include <boost/thread/thread.hpp>


int main() {

    //加载目标点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);//共享指针
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan1.pcd", *target_cloud) == -1)
    {
        PCL_ERROR("could not read file room_scan1.pcd");
        return -1;
    }

    std::cout << "loaded " << target_cloud->size() << " data points from room_scan1.pcd"<<endl;

//    加载源点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("room_scan2.pcd", *input_cloud) == -1)
    {
        PCL_ERROR("could not read file room_scan2.pcd");
        return -1;
    }

    std::cout << "loaded " << input_cloud->size() << " data points from room_scan2.pcd"<<endl;

//    后续配准是完成对源点云到目标点云的参考坐标系的变换矩阵的估计
/*
 * 过滤输入点云到原始尺寸的10%提高匹配速度
 * 在NDT算法中，目标点云对应的体素网格数据结构的的统计计算不使用单个点，而是使用包含在每个体素网格中的点的统计数据
 * */
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize(0.2, 0.2, 0.2);
    approximate_voxel_filter.setInputCloud(input_cloud);
    approximate_voxel_filter.filter(*filter_cloud);

    std::cout << "filter cloud contains  " << filter_cloud->size() << " data points from room_scan2.pcd"<<endl;

// 初始化NDT对象及参数
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;
    ndt.setTransformationEpsilon(0.01);//为终止条件设置最小转换差异
    ndt.setStepSize(0.1);//为more-thuente线搜索设置最大步长
    ndt.setResolution(1.0);//NDT网格结构的分辨率

    ndt.setMaximumIterations(35);//设置匹配的最大迭代次数
    ndt.setInputSource(filter_cloud);
    ndt.setInputTarget(target_cloud);

//    使用机器人测距法得到的初始变换矩阵的结果
    Eigen::AngleAxisf init_rotation (0.6931, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation (1.79387, 0.720047, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();

//    计算需要的刚体变换以便将输入的源点云匹配到目标点云上去
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align(*output_cloud, init_guess);//output_cloud不能作为最终点云变换，因为源点云进行了滤波处理

    std::cout << "Normal Distributions Transform has converged:  " << ndt.hasConverged() << " score: "<< ndt.getFitnessScore()<< std::endl;

//    使用创建的变换对为过滤的输入点云进行变换
    pcl::transformPointCloud(*input_cloud, *output_cloud, ndt.getFinalTransformation());
//    保存转换后的源点云作为最终的变换输出
    pcl::io::savePCDFileASCII("room_scan_output.pcd", *output_cloud);

//    初始化点云可视化对象
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer_final->setBackgroundColor(0, 0, 0);
//    对点云目标进行着色 red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_color(target_cloud, 255, 0, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(target_cloud, target_color, "target_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target_cloud");

//    对转换后的源点云进行着色 green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> output_color(output_cloud, 0, 255, 0);
    viewer_final->addPointCloud<pcl::PointXYZ>(output_cloud, output_color, "output_cloud");
    viewer_final->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "output_cloud");

//启动可视化
    viewer_final->addCoordinateSystem(1.0, "global");//显示xyz指示轴
    viewer_final->initCameraParameters();//显示摄像头初始化参数

    while(!viewer_final->wasStopped())
    {
        viewer_final->spinOnce(100);
        boost::this_thread::sleep(boost::posix_time::microseconds(10000));
    }

    return 0;
}