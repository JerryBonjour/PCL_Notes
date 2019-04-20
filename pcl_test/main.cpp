#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>

using  namespace std;

int main() {
    std::cout << "Hello, World!" << std::endl;

    /**
     * 1.向pcd文件中写入点云数据
     *  熟悉pointcloud的数据结构
     */
   /* pcl::PointCloud<pcl::PointXYZ> cloud;
    //创建点云并设置参数
    cloud.width = 50;
    cloud.height = 10;
    cloud.is_dense = false;//不是稠密型
    cloud.points.resize(cloud.width * cloud.height);//点云总数大小

    //随机数填充点云
    for (size_t i = 0; i < cloud.points.size(); i++)
    {
        cloud.points[i].x = 1024 * random() / (RAND_MAX + 1.0f);
        cloud.points[i].y = 1024 * random() / (RAND_MAX + 1.0f);
        cloud.points[i].z = 1024 * random()/ (RAND_MAX + 1.0f);
    }

    //存储点云数据
    pcl::io::savePCDFileASCII("pcd_test.pcd", cloud);
    //打印点云数据
    std::cerr << "PointCloud:" << cloud.points.size() << "保存到pcd_test" << std::endl;

    for (size_t i = 0;  i < cloud.points.size(); i++) {

        std::cerr << "(" << cloud.points[i].x << "," << cloud.points[i].y << "," << cloud.points[i].z << ")" <<std::endl;

    }

    return 0;*/

    /**
     * 2.点云显示
     *
     * */
      /* pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
       uint8_t r(255), g(15), b(15);

       for (float z(-1.0); z <= 1.0; z+=0.05)
       {
           for (float angle(0.0); angle <= 360.0; angle += 5.0) {
               pcl::PointXYZRGB point;
               point.x = 0.5 * cosf(pcl::deg2rad(angle));
               point.y = sinf(pcl::deg2rad(angle));
               point.z = z;

               uint32_t rgb = (static_cast<uint32_t >(r) << 16 | static_cast<uint32_t >(g) << 8 | static_cast<uint32_t>(b));

               point.rgb = *reinterpret_cast<float *>(&rgb);
               point_cloud_ptr->points.push_back(point);
           }

           if(z < 0.0)
           {
               r -= 12;
               g +=12;
           } else
           {
               g -= 12;
               b += 12;
           }
       }

       point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
       point_cloud_ptr->height = 1;

       pcl::io::savePCDFileASCII("pcd_test2.pcd", *point_cloud_ptr);

       pcl::visualization::CloudViewer viewer("cloud_viewer");
       viewer.showCloud(point_cloud_ptr);
       while (!viewer.wasStopped())
       {}
       return 0;*/

    /**
     * 3.从PCD文件中读取点云
     */

    typedef pcl::PointXYZRGB PointT;
    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    string dir ="/home/catalina/PCL_Notes/pcl_test/cmake-build-debug/";//pcd文件路径
    string fileName="pcd_test2.pcd";

    if(pcl::io::loadPCDFile<PointT> ((dir+fileName), *cloud) == -1)
    {
        PCL_ERROR("无法读取PCD文件");
        return -1;
    }
    printf("加载PCD文件失败\n", cloud->width * cloud->height);

    pcl::visualization::PCLVisualizer viewer("点云可视化");
    viewer.setCameraPosition(0, 0, -3.0, 0, -1, 0);
    viewer.addCoordinateSystem(0.3);

    viewer.addPointCloud(cloud);
    while (!viewer.wasStopped())
    viewer.spinOnce(100);
    return 0;
}