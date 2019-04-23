//
// Created by catalina on 19-4-22.
//
/**
 * 2.icp对点云数据配准计算
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h> //pcl控制台解析
#include <boost/thread/thread.hpp>

/**
 * 按下空格则执行icp计算
 * @param event
 * @param nothing
 */
bool next_iteration = false;//键盘交互函数
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void * nothing)
{
    if(event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

int main() {

    //这里需要读取具体的点云文件
    //创建pcl::PointCloud<pcl::PointXYZ>共享指针并初始化
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr Final (new pcl::PointCloud<pcl::PointXYZ>);////存储经过配准变换的点云

//    读取点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit.pcd", *cloud_in) == -1)
    {
        PCL_ERROR("could not read file rabbit.pcd");
        return -1;
    }
    std::cout << "loaded " << cloud_in->size() << " data points from rabbit.pcd"<<endl;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>("rabbit_t.pcd", *cloud_target) == -1)
    {
        PCL_ERROR("could not read file rabbit_t.pcd");
        return -1;
    }

    std::cout << "loaded " << cloud_target->size() << " data points from rabbit_t.pcd"<<endl;


    //  点云配准
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputCloud(cloud_in); //输入源点云
    icp.setInputTarget(cloud_target);//匹配目标点云
    pcl::PointCloud<pcl::PointXYZ> Final2;
//    icp.align(Final2);//对齐

//    可视化窗口设置
    boost::shared_ptr<pcl::visualization::PCLVisualizer> view(new pcl::visualization::PCLVisualizer("icp test"));//定义两个窗口共享指针
    int v1, v2;//v1显示初始位置, v2展示配准过程

    //四个窗口参数分别对应x_min y_min x_max y_max
    view->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    view->createViewPort(0.5, 0.0, 1.0, 1.0, v2);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sources_cloud_color(cloud_in, 250, 0, 0);//设置源点云颜色为红色
    view->addPointCloud(cloud_in, sources_cloud_color,"source_cloud_v1", v1);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> target_cloud_color (cloud_target, 0, 250, 0);//目标点云绿色
    view->addPointCloud(cloud_target, target_cloud_color, "target_cloud_v1", v1);//将点云添加到v1窗口

    view->setBackgroundColor(0.0, 0.05, 0.05, v1);
    view->setBackgroundColor(0.05, 0.05, 0.05, v2);
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source_cloud_v1");//显示点的大小
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_v1");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> aligend_cloud_color(Final, 255, 255, 255);
    view->addPointCloud(Final, aligend_cloud_color, "aligend_cloud_color_v2", v2);//设置配准结果为白色
    view->addPointCloud(cloud_target, target_cloud_color,"target_cloud_color_v2", v2);

    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "aligend_cloud_color_v2");//显示点的大小
    view->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target_cloud_color_v2");

    view->registerKeyboardCallback(&keyboardEvent, (void* )NULL);//设置键盘回调函数
    int iterations = 0;
    while (! view->wasStopped())
    {
        view->spinOnce();//运行视图
        if(next_iteration)
        {
            icp.align(*Final);
            //converged=1则本次配准成功，后者输出可变换矩阵
            std::cout << " has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;
            std::cout << "Matrix:\n" << icp.getFinalTransformation() << std::endl;//打印转换关系矩阵
            std::cout << "iteration = " << ++iterations;
            if (iterations == 1000)
            {
                return 0;
            }

            view->updatePointCloud(Final, aligend_cloud_color, "aligend_cloud_color_v2");
        }
        next_iteration = false;
    }
}

