//
// Created by catalina on 19-4-22.
//

#include <boost/make_shared.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h> //体素网格化滤波类
#include <pcl/filters/filter.h>
#include <pcl/features/normal_3d.h>//法线特征
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h> //非线性ICP相关头文件
#include <pcl/registration/transforms.h>//变换矩阵类
#include <pcl/visualization/pcl_visualizer.h>//可视化

using pcl::visualization::PointCloudColorHandlerGenericField;
using pcl::visualization::PointCloudColorHandlerCustom;

//定义
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
typedef pcl::PointNormal PointNormalT;
typedef pcl::PointCloud<PointNormalT> PointCloudWithNormals;

//申请全局可视化对象变量, 定义左右视点分别显示配准前后的结果点云
pcl::visualization::PCLVisualizer *p;
int vp_1, vp_2; //定义左右视点的ID

//管理点云对象
struct PCD
{
    PointCloud::Ptr cloud;//点云共享指针
    std::string f_name; //文件名称

    PCD() : cloud(new PointCloud){};
};

struct PCDComparator  //文件比较处理
{
    bool operator() (const PCD& p1, const PCD& p2)
    {
        return (p1.f_name < p2.f_name);
    }
};


//以x,y,z,curvature定义一个新的点的表示
class MyPointRepresemtation : public pcl::PointRepresentation <PointNormalT>
{
    using  pcl::PointRepresentation<PointNormalT>::nr_dimensions_;

public:
    MyPointRepresemtation()
    {
        nr_dimensions_ = 4;//定义点的维度
    }

//    重载将点转化为四维数组
    virtual  void copyToFloatArray(const PointNormalT &p, float * out) const
    {
        out[0] = p.x;
        out[1] = p.y;
        out[2] = p.z;
        out[3] = p.curvature;
    }
};


//显示配准前的源点云和目标点云
void showCloudsLeft(const PointCloud::Ptr cloud_target, const PointCloud::Ptr cloud_source)
{
    p->removePointCloud("vp1_target");
    p->removePointCloud("vp2_source");

    PointCloudColorHandlerCustom<PointT> tgt_h (cloud_target, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> src_h (cloud_source, 255, 0, 0);
    p->addPointCloud(cloud_target, tgt_h, "vp1_target", vp_1);
    p->addPointCloud(cloud_source, src_h, "vp1_source", vp_1);

    PCL_INFO("Press q to begin the registration:\n");
    p->spin();
}

//显示配准后的源点云和目标点云
void showCloudsRight(const PointCloudWithNormals::Ptr cloud_target, const PointCloudWithNormals::Ptr cloud_source)
{
    p->removePointCloud("source");
    p->removePointCloud("target");

    PointCloudColorHandlerGenericField<PointNormalT> tgt_color_handler (cloud_target, "curvature");
    if(!tgt_color_handler.isCapable())
        PCL_WARN("can not create curature color handler");
    PointCloudColorHandlerGenericField<PointNormalT> src_color_handler (cloud_source, "curvature");
    if(!src_color_handler.isCapable())
        PCL_WARN("can not create curature color handler");

    p->addPointCloud(cloud_target, tgt_color_handler, "target", vp_2);
    p->addPointCloud(cloud_source, src_color_handler, "source", vp_2);

    p->spin();
}

/**
 * ]
 *
 * @param argc
 * @param argv  从mainc传过来的参数,
 * @param models 点云数据集
 */

void loadData(int argc, char **argv, std::vector<PCD, Eigen::aligned_allocator<PCD>> &models)
{
    std::string extension (".pcd");
//    第一个参数是命令本身，从第二个参数开始解析
    for (int i = 1; i < argc; i++) {
        std::string fname = std::string(argv[i]);
//        pcd文件名至少是5个字符
        if(fname.size() <= extension.size())
            continue;
        std::transform(fname.begin(), fname.end(), fname.begin(), (int(*) (int))tolower);
//        检查参数是否为pcd后缀文件
        if(fname.compare(fname.size() - extension.size(), extension.size(), extension) == 0)
        {
//            加载点云并保存在总体的点云列表中
            PCD m;
            m.f_name = argv[i];
            pcl::io::loadPCDFile(argv[i], *m.cloud);

//            移除点云中的无效点
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*m.cloud, *m.cloud, indices);

            models.push_back(m);
        }

    }
}

/**
 * 实现配准计算
 * @param cloud_src
 * @param cloud_tgt
 * @param output 输出配准后的点云
 * @param final_transform  变换矩阵
 * @param downsample  是否需要降采样
 */

void pairAlign(const PointCloud::Ptr cloud_src, const PointCloud::Ptr cloud_tgt, PointCloud::Ptr output,
        Eigen::Matrix4f &final_transform, bool downsample = false)
{
    PointCloud::Ptr src(new PointCloud);//存储滤波后的源点云
    PointCloud::Ptr tgt(new PointCloud);
    pcl::VoxelGrid<PointT> grid;
//   是否需要进行降采样
    if(downsample)
    {
        grid.setLeafSize(0.05, 0.05, 0.05);//设置滤波时的体素大小
        grid.setInputCloud(cloud_src);
        grid.filter(*src);

        grid.setInputCloud(cloud_tgt);
        grid.filter(*src);
    } else{
        src = cloud_src;
        tgt = cloud_tgt;
    }

//    计算表面的法向量和曲率
    PointCloudWithNormals::Ptr points_with_normals_src (new PointCloudWithNormals);
    PointCloudWithNormals::Ptr points_with_normals_tgt (new PointCloudWithNormals);

//    点云法线估计向量
    pcl::NormalEstimation<PointT, PointNormalT> norm_est;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    norm_est.setSearchMethod(tree);
    norm_est.setKSearch(30);

    norm_est.setInputCloud(src);
    norm_est.compute(*points_with_normals_src);
    pcl::copyPointCloud(*src, *points_with_normals_src);

    norm_est.setInputCloud(tgt);
    norm_est.compute(*points_with_normals_tgt);
    pcl::copyPointCloud(*tgt, *points_with_normals_tgt);

    MyPointRepresemtation point_represemtation;
    float alpha[4] = {1.0, 1.0, 1.0, 1.0};//设置x,y,z,curature的权重均衡
    point_represemtation.setRescaleValues(alpha);

//    配准
    pcl::IterativeClosestPointNonLinear<PointNormalT, PointNormalT> reg;//配准对象
    reg.setTransformationEpsilon(1e-6);//收敛判断条件,越小精度越大收敛越慢
    reg.setMaxCorrespondenceDistance(0.1);//匹配点对的距离大于10cm的不考虑
    reg.setPointRepresentation(boost::make_shared<const MyPointRepresemtation>(point_represemtation));//设置点表示

    reg.setInputSource(points_with_normals_src);//设置源点云
    reg.setInputTarget(points_with_normals_tgt);

//    设置迭代信息以及可视化迭代过程
    Eigen::Matrix4f Ti = Eigen::Matrix4f::Identity(), prev, targetToSource;
    PointCloudWithNormals::Ptr reg_result = points_with_normals_src;
    reg.setMaximumIterations(2);//每迭代两次就认为收敛，并停止内部迭代

    for (int i = 0; i < 30; ++i) {
        PCL_INFO("Iteration Nr. %d. \n");

//        存储点云并可视化
    points_with_normals_src = reg_result;

//    估计
    reg.setInputSource(points_with_normals_src);
    reg.align(*reg_result);

    Ti = reg.getFinalTransformation() *Ti;//计算每一次迭代的转换矩阵

    if(fabs((reg.getLastIncrementalTransformation()-prev).sum()) < reg.getTransformationEpsilon())
        reg.setMaxCorrespondenceDistance(reg.getMaxCorrespondenceDistance() - 0.001);

    prev = reg.getLastIncrementalTransformation();
    showCloudsRight(points_with_normals_tgt, points_with_normals_src);//可视化当前状态
    }

//    获取源点云到目标点云的转换矩阵
    targetToSource = Ti.inverse();

    pcl::transformPointCloud(*cloud_tgt, *output, targetToSource);
    p->removePointCloud("source");
    p->removePointCloud("target");

    PointCloudColorHandlerCustom<PointT> cloud_tgt_h (output, 0, 255, 0);
    PointCloudColorHandlerCustom<PointT> cloud_src_h (cloud_src, 255, 0, 0);
    p->addPointCloud(output,cloud_tgt_h, "target", vp_2);
    p->addPointCloud(cloud_src, cloud_src_h, "source", vp_2);

    PCL_INFO("Press q to continue the registeration. \n");
    p->spin();

    p->removePointCloud("source");
    p->removePointCloud("target");
    *output += *cloud_src;//叠加到转换后的点云上
    final_transform  = targetToSource;
}


int main(int argc, char** argv)
{
//    存储管理所有打开的点云
    std::vector<PCD, Eigen::aligned_allocator<PCD>> data;
    loadData(argc, argv, data);

//    检查输入
    if(data.empty())
    {
        PCL_ERROR("Syntax is: %s <source.pcd> <target.pcd> [*]", argv);
        PCL_ERROR("[*] - multiple files can be added.The registration results of (i, i+1) will be registered against(i+2), etc");
        return -1;
    } else
    PCL_INFO("loaded %d datasets.", (int) data.size());

//    创建PCL可视化对象
    p = new pcl::visualization::PCLVisualizer(argc, argv, "Pairwise Incremental Registeration example");
    p->createViewPort(0.0, 0, 0.5, 1.0, vp_1);//左半窗口
    p->createViewPort(0.5, 0, 1.0, 1.0, vp_2);

    PointCloud::Ptr result (new PointCloud), target, source;
    Eigen::Matrix4f GlobleTransform = Eigen::Matrix4f::Identity(), pairTansform;
//    循环处理所有的点云
    for(size_t i =1; i < data.size(); ++i)
    {
        source = data[i-1].cloud;//连续配准
        target = data[i].cloud;//相邻两组点云

        showCloudsLeft(source, target);

        PointCloud::Ptr temp(new PointCloud);//返回配准后两组点云在第一组点云坐标下的点云
        PCL_INFO("Aligning %s (%d) with %s (%d).\n", data[i-1].f_name.c_str(), source->points.size(), data[i].f_name.c_str(), target->points.size());

//        目标点到source的变换矩阵
        pairAlign(source, target, temp, pairTansform, true);

        pcl::transformPointCloud(*temp, *result, GlobleTransform);//把当前两两配准之后的点云temp转换到全局坐标系下，返回result
        GlobleTransform = GlobleTransform * pairTansform;//用当前两组点云的变换更新全局变换

        std::stringstream ss;
        ss << i << ".pcd";
        pcl::io::savePCDFile(ss.str(), *result, true);

    }

}




















