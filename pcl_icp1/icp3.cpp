//
// Created by catalina on 19-4-23.
//

/**
 * 编译完成后执行./pcl_icp1 bunny.ply
 * 3.加载点云，进行刚体变换，之后使用icp算法将变换后的点云与原来的点云对齐
 */
#include <iostream>
#include <string>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>

//定义点云格式
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

//打印旋转和平移矩阵
void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
    printf("Rotation matrix:\n");
    printf("    |  %6.3f %6.3f %6.3f %6.3f | \n", matrix(0, 0), matrix(0, 1), matrix(0, 2));
    printf("R = |  %6.3f %6.3f %6.3f %6.3f | \n", matrix(1, 0), matrix(1, 1), matrix(1, 2));
    printf("    |  %6.3f %6.3f %6.3f %6.3f | \n", matrix(2, 0), matrix(2, 1), matrix(2, 2));
    printf("Translation vector :\n");
    printf("t = < %6.3f, %6.3f, %6.3f > \n\n", matrix(0, 3), matrix(1, 3),matrix(2, 3));
}

//使用空格增加迭代次数并更新显示
void keyboardEvent(const pcl::visualization::KeyboardEvent &event, void * nothing)
{
    if(event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}


int main(int argc, char* argv[])
{
    PointCloudT::Ptr cloud_in (new PointCloudT);//源点云
    PointCloudT::Ptr cloud_tr (new PointCloudT);//转换后点云
    PointCloudT::Ptr cloud_icp (new PointCloudT);//ICP输出点云

//    检验输入命令的合法性
    if (argc < 2)
    {
        printf("Usage : \n");
        printf("\t\t%s bunny.ply number_of_ICP_iterations\n", argv[0]);
        PCL_ERROR("Provide one ply file.\n");
        return -1;
    }

    int iterations = 1;
    if(argc > 2)
    {
//        如果两条命令以上说明用户将迭代次数作为传递参数
        if(iterations < 1)
        {
            PCL_ERROR("Number of initial iterations must be >= 1\n");
            return -1;
        }
    }

    pcl::console::TicToc time;
    time.tic();

    if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0)
    {
        PCL_ERROR("Error loading cloud %s. \n", argv[1]);
        return -1;
    }
    std::cout << "\n Loaded file " << argv[1] << " (" << cloud_in->size() << " points) in " << time.toc() << " ms\n" << std::endl;

//    定义旋转矩阵和平移变量为4*4的矩阵
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();//初始化

//    旋转矩阵定义
    double theta = M_PI / 8; //旋转角度用弧度表示
    transformation_matrix (0, 0) = cos (theta);
    transformation_matrix (0, 1) = -sin(theta);
    transformation_matrix (1, 0) = sin(theta);
    transformation_matrix (1, 1) = cos(theta);

//    Z轴的平移变量
    transformation_matrix (2, 3) = 0.4;

//    打印转换矩阵
    std::cout << "Applying this rigid transformation to : cloud_in -> cloud_icp "<< std::endl;
    print4x4Matrix(transformation_matrix);

//    点云转换
    pcl::transformPointCloud (*cloud_in, *cloud_icp, transformation_matrix);
    *cloud_tr = *cloud_icp;

//    迭代最近点法
    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_icp);//源点云
    icp.setInputTarget(cloud_in);//目标点云
    icp.align(*cloud_icp);//匹配后的源点云  output

    icp.setMaximumIterations(1);
    std::cout << "Applied " << iterations << " ICP iterations in " << time.toc() << " ms" << std::endl;

    if(icp.hasConverged()) //输出变换矩阵的适合性评估
    {
        std::cout << "\n ICP has converged, score is " << icp.getFitnessScore() << std::endl;
        std::cout << "\n ICP transformation " << iterations << " : cloud_icp -> cloud_in " << std::endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        print4x4Matrix(transformation_matrix);
    } else{
        PCL_ERROR("\n ICP has not converged.");
        return -1;
    }

//    可视化结果
    pcl::visualization::PCLVisualizer viewer ("ICP Compute");
    int v1(0), v2(1);
    viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
    viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

//    定义显示的颜色信息
    float bckgr_gray_level = 0.0;
    float txt_gray_lv1 = 1.0 - bckgr_gray_level;

//    设置源点云为白色，转换后的点云为绿色, icp配准后的点云为红色
    pcl::visualization::PointCloudColorHandlerCustom<PointT>  cloud_in_color_h (cloud_in, (int)255 * txt_gray_lv1, (int) 255 * txt_gray_lv1, txt_gray_lv1);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v1", v1);
    viewer.addPointCloud(cloud_in, cloud_in_color_h, "cloud_in_v2", v2);

//    转换后点云显示为绿色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h (cloud_tr, 20, 180, 20);
    viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

//    ICP配准后点云为红色
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_icp, 180, 20, 20);
    viewer.addPointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2", v2);

//    在界面窗口添加文字描述
    viewer.addText("White: Original point cloud\n Green:Matrix transformed point cloud", 10, 15, 16,txt_gray_lv1, txt_gray_lv1, txt_gray_lv1, "icp_info_1", v1);
    viewer.addText("White: Original point cloud\n Red:ICP aligned point cloud", 10, 15, 16,txt_gray_lv1, txt_gray_lv1, txt_gray_lv1, "icp_info_2", v2);

//输入迭代次数
    std::stringstream ss;
    ss << iterations;
    std::string iterations_cnt = "ICP iterations = " + ss.str();
    viewer.addText(iterations_cnt, 10, 60, 16,txt_gray_lv1, txt_gray_lv1, txt_gray_lv1, "iterations_cnt", v2);

//    设置背景颜色
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
    viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);
//    设置相机的坐标和方向
    viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
    viewer.setSize(1280, 1024);//窗口大小设置

//    注册按键回调函数
    viewer.registerKeyboardCallback(&keyboardEvent, (void*) NULL);
//    显示
    while(!viewer.wasStopped())
    {
        viewer.spinOnce();

        if(next_iteration)
        {
//            迭代计算
            time.tic();
            icp.align(*cloud_icp);
            std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;
            if(icp.hasConverged())
            {
                printf("\033[11A");
                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
                std::cout << "\n ICP Transformation " << ++iterations << ": cloud_icp -> cloud_in " << std::endl;
                transformation_matrix *= icp.getFinalTransformation().cast<double>();
                print4x4Matrix(transformation_matrix);

                ss.str("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = "+ss.str();
                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lv1, txt_gray_lv1, txt_gray_lv1, "iterations_cnt");
                viewer.updatePointCloud(cloud_icp, cloud_icp_color_h, "cloud_icp_v2");
            }else{
                PCL_ERROR("\nICP has not converged.\n");
                return -1;
            }
        }
        next_iteration = false;
    }
    return 0;
}
























