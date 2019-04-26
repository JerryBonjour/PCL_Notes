#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
/**
 * 1.两幅图像通过icp求变换矩阵
 *
 */

/*int main() {

    //创建pcl::PointCloud<pcl::PointXYZ>共享指针并初始化
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

//    随机填充点云
    cloud_in->width = 5;
    cloud_in->height = 1;
    cloud_in->is_dense = false;//无序
    cloud_in->points.resize(cloud_in->width * cloud_in->height);
    for (size_t i = 0; i < cloud_in->points.size(); ++i)
    {
        cloud_in->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
        cloud_in->points[i].z = 1024 * rand() /(RAND_MAX + 1.0f);
    }
//打印点云总数及坐标
    std::cout << "Saved: " << cloud_in->size() << " data points to input:" << std::endl;
    for (size_t i = 0;  i < cloud_in->points.size(); i++) {

        std::cout << "   " << cloud_in->points[i].x << " " << cloud_in->points[i].y << " " << cloud_in->points[i].z <<std::endl;
    }

    *cloud_out = * cloud_in;
    std::cout << "size: " << cloud_out->points.size() << std::endl;

//  实现简单的点云刚体变换构造目标点云 x平移0.7
    for (size_t i = 0; i < cloud_in->points.size(); ++i) {
        cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;

    }
    std::cout << "Tansformed: " << cloud_in->points.size() << " data points: " << std::endl;

    for (size_t i = 0;  i < cloud_out->points.size(); i++) {

        std::cerr << "    " << cloud_out->points[i].x << " " << cloud_out->points[i].y << " " << cloud_out->points[i].z <<std::endl;
    }

//  点云配准
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in); //输入源点云
    icp.setInputTarget(cloud_out);//匹配目标点云
    pcl::PointCloud<pcl::PointXYZ> Final; //存储经过配准变换的点云
    icp.align(Final);//对齐

    std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
    std::cout << "Matrix:\n" << icp.getFinalTransformation() << std::endl;//打印转换关系矩阵

    return 0;
}*/
