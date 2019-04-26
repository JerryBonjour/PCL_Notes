#include "../include/plane_fitting_core.h"

bool point_cmp(VPoint a, VPoint b)
{
    return a.z < b.z;
}

PlaneGroundFilter::PlaneGroundFilter(ros::NodeHandle &nh)
{
//读取参数配置 roslaunch在启动的时候已经加载完配置好的参数，这里直接读取
    std::string input_topic;
    nh.getParam("input_topic", input_topic);

    sub_point_cloud_ = nh.subscribe(input_topic, 10, &PlaneGroundFilter::point_cb, this);

// 获取pub
    std::string no_ground_topic, ground_topic, all_points_topic;

    nh.getParam("no_ground_point_topic", no_ground_topic);
    nh.getParam("ground_point_topic", ground_topic);
    nh.getParam("all_points_topic", all_points_topic);

    nh.getParam("clip_height", clip_height_);
    ROS_INFO("clip_height: %f", clip_height_);
    nh.getParam("sensor_height", sensor_height_);
    ROS_INFO("sensor_height: %f", sensor_height_);
    nh.getParam("min_distance", min_distance_);
    ROS_INFO("min_distance: %f", min_distance_);
    nh.getParam("max_distance", max_distance_);
    ROS_INFO("max_distance: %f", max_distance_);

    nh.getParam("sensor_model", sensor_model_);
    ROS_INFO("sensor_model: %d", sensor_model_);
    nh.getParam("num_iter", num_iter_);
    ROS_INFO("num_iter: %d", num_iter_);
    nh.getParam("num_lpr", num_lpr_);
    ROS_INFO("num_lpr: %d", num_lpr_);
    nh.getParam("th_seeds", th_seeds_);
    ROS_INFO("th_seeds: %f", th_seeds_);
    nh.getParam("th_dist", th_dist_);
    ROS_INFO("th_dist: %f", th_dist_);
// 发布分割后的点云
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>(ground_topic, 10);
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>(no_ground_topic, 10);
    pub_all_points_ = nh.advertise<sensor_msgs::PointCloud2>(all_points_topic, 10);

    g_seeds_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);//种子集
    g_ground_pc = pcl::PointCloud<VPoint>::Ptr (new pcl::PointCloud<VPoint>);//地面点集
    g_not_ground_pc = pcl::PointCloud<VPoint>::Ptr (new pcl::PointCloud<VPoint>);//非地面点集
    g_all_pc = pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>); //所有点集

    ros::spin();
}

// 回调函数 点云处理过程
void PlaneGroundFilter::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud)
{
    // PointCloud2 转 PointCloud<VPoint>
    pcl::PointCloud<VPoint> laserCloudIn;
    pcl::fromROSMsg(*in_cloud, laserCloudIn);
    pcl::PointCloud<VPoint> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud, laserCloudIn_org);

//  分割地面点
    SLRPointXYZIRL point;
    for(size_t i = 0; i < laserCloudIn.points.size(); i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.ring = laserCloudIn.points[i].ring;
        point.label = 0u; //0表示未聚类
        g_all_pc->points.push_back(point);//获取所有的点
    }

// 按z轴方向距离远近对点进行排序
    sort(laserCloudIn.points.begin(), laserCloudIn.points.end(), point_cmp);

// 去掉噪点 消除近点反射
    pcl::PointCloud<VPoint>::iterator it = laserCloudIn.points.begin();
    for(int i = 0; i < laserCloudIn.points.size(); i++)
    {
        if(laserCloudIn.points[i].z < -1.5 * sensor_height_)
        {
            i++;
        }else
        {
            break;
        }

    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);

//  提取初始种子点
    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;
//  地面平面拟合主循环
    for(int i = 0; i < num_iter_; i++)
    {
//        平面模型估计
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        // pointcloud to matrix
        MatrixXf points(laserCloudIn_org.points.size(), 3);
        int j = 0;
        for(auto p : laserCloudIn_org.points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }
        // 点到平面的正交投影距离
        VectorXf result = points * normal_;
        // 距离阀值过滤
        for(int r = 0; r < result.rows(); r++)
        {
            if(result[r] < th_dist_d_)
            {//地面
                g_all_pc->points[r].label = 1u;
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
            else
            {//非地面，未聚类
                 g_all_pc->points[r].label = 0u;
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }


    }

    pcl::PointCloud<VPoint>::Ptr final_no_ground(new pcl::PointCloud<VPoint>);
    // 进一步处理非地面点
    post_process(g_not_ground_pc, final_no_ground);

    ROS_INFO_STREAM("origin: "<<g_not_ground_pc->points.size()<<" post_process: "<<final_no_ground->points.size());

    // 发布切割好的地面点几集
    sensor_msgs::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud->header.stamp;
    ground_msg.header.frame_id = in_cloud->header.frame_id;
    pub_ground_.publish(ground_msg);

    // publish not ground points
    sensor_msgs::PointCloud2 groundless_msg;
    pcl::toROSMsg(*final_no_ground, groundless_msg);
    groundless_msg.header.stamp = in_cloud->header.stamp;
    groundless_msg.header.frame_id = in_cloud->header.frame_id;
    pub_no_ground_.publish(groundless_msg);

    // publish all points
    sensor_msgs::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud->header.stamp;
    all_points_msg.header.frame_id = in_cloud->header.frame_id;
    pub_all_points_.publish(all_points_msg);
    g_all_pc->clear();
}

/*
    @brief Extract initial seeds of the given pointcloud sorted segment.
    This function filter ground seeds points accoring to heigt.
    This function will set the `g_ground_pc` to `g_seed_pc`.
    @param p_sorted: sorted pointcloud
    @param ::num_lpr_: num of LPR points
    @param ::th_seeds_: threshold distance of seeds
    @param ::
*/
void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted)
{
    // LPR is the mean of low point representative
    double sum = 0;
    int cnt = 0;
    // Calculate the mean height value.
    for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0; // in case divide by 0
    g_seeds_pc->clear();
    // iterate pointcloud, filter those height is less than lpr.height+th_seeds_
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
    // return seeds points
}


void PlaneGroundFilter::post_process(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::PointCloud<VPoint>::Ptr cliped_pc_ptr(new pcl::PointCloud<VPoint>);
    clip_above(in, cliped_pc_ptr);

    pcl::PointCloud<VPoint>::Ptr remove_close(new pcl::PointCloud<VPoint>);
    remove_close_far_pt(cliped_pc_ptr, out);
}

/**
 * 裁剪掉过近的点云
 * @param in
 * @param out
 */
void PlaneGroundFilter::remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in,const pcl::PointCloud<VPoint>::Ptr out){
    pcl::ExtractIndices<VPoint> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for(size_t i = 0; i < in->points.size(); i++)
    {
       double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

       if((distance < min_distance_) || (distance > max_distance_))
       {
           indices.indices.push_back(i);
       }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);
    cliper.filter(*out);
}

/**
 * 根据高度裁剪掉过高的点云
 * @param in
 * @param out
 */
void PlaneGroundFilter::clip_above(const pcl::PointCloud<VPoint>::Ptr in,
                                   const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;

#pragma omp for
    for(size_t i = 0; i < in->points.size(); i++)
    {
        if(in->points[i].z > clip_height_)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);
    cliper.filter(*out);
}
/**
 * 平面模型估计  用简单的线性模型估计 ax+by+cz+d = 0
 * 点云中的点到这个平面的正交投影距离小于阀值th_dist_d_,则属于地面点
 */
void PlaneGroundFilter::estimate_plane_()
{
    Eigen::Matrix3f cov; //初始点集的协方差矩阵，描述种子点集的散布情况
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);
    // SVD分解
    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    normal_ = (svd.matrixU().col(2));  //points*normal_ 即为点到平面的正交投影距离

    Eigen::Vector3f seeds_mean = pc_mean.head<3>();
    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    th_dist_d_ = th_dist_ - d_; //点到平面的距离阀值
}

PlaneGroundFilter::~PlaneGroundFilter(){}

void PlaneGroundFilter::Spin(){}

