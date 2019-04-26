#include "../include/pcl_ground_core.h"

PclGroundCore::PclGroundCore(ros::NodeHandle &nh)
{
    sub_point_cloud_ = nh.subscribe("/velodyne_points", 10, &PclGroundCore::point_cb, this);

//发布到新的topic上
    pub_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_ground", 10);//分割出来的地面
    pub_no_ground_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points_no_ground", 10);//分割出来的非地面

    ros::spin();
}

/**
 * 回调函数,切割出地面后重新发布
 * @param in_cloud
 */
void PclGroundCore::point_cb(const sensor_msgs::PointCloud2ConstPtr &in_cloud_ptr)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr cliped_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);
//  裁剪
    clip_above(CLIP_HEIGHT, current_pc_ptr, cliped_pc_ptr);
//    过滤近距离
    pcl::PointCloud<pcl::PointXYZI>::Ptr remove_close(new pcl::PointCloud<pcl::PointXYZI>);
    remove_close_pt(MIN_DISTANCE, cliped_pc_ptr, remove_close);

//   重新组织点云
    PointCloudXYZIRTColor organized_points;
    std::vector<pcl::PointIndices> radial_division_indices; //射线索引
    std::vector<pcl::PointIndices> closest_indices;
    std::vector<PointCloudXYZIRTColor> radial_ordered_clouds; //排好序的点云索引
// 2000条射线
    radial_dividers_num_ = ceil(360 / RADIAL_DIVIDER_ANGLE);

    XYZI_to_RTZColor(remove_close, organized_points,
                     radial_division_indices, radial_ordered_clouds);

    pcl::PointIndices ground_indices, no_ground_indices;
//  地面点分类
    classify_pc(radial_ordered_clouds, ground_indices, no_ground_indices);

    pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>);

    pcl::ExtractIndices<pcl::PointXYZI> extract_ground;
    extract_ground.setInputCloud(remove_close);
    extract_ground.setIndices(boost::make_shared<pcl::PointIndices>(ground_indices));

    extract_ground.setNegative(false); //true removes the indices, false leaves only the indices
    extract_ground.filter(*ground_cloud_ptr);

    extract_ground.setNegative(true); //true removes the indices, false leaves only the indices
    extract_ground.filter(*no_ground_cloud_ptr);

    //////pub for debug
    // sensor_msgs::PointCloud2 pub_pc;
    // pcl::toROSMsg(*remove_close, pub_pc);

    // pub_pc.header = in_cloud_ptr->header;

    // pub_ground_.publish(pub_pc);

    publish_cloud(pub_ground_, ground_cloud_ptr, in_cloud_ptr->header);
    publish_cloud(pub_no_ground_, no_ground_cloud_ptr, in_cloud_ptr->header);
}



PclGroundCore::~PclGroundCore() {}

void PclGroundCore::Spin()
{
}

/**
 * 点云裁剪，过高的区域忽略不计，裁剪掉1.28米以上的高度
 * @param clip_height
 * @param in
 * @param out
 */
void PclGroundCore::clip_above(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                             const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;

//    使用OpenMP的并行化语法，加速for循环
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}
/**
 * 过滤近距离的点云，消除雷达反射的影响
 * @param min_distance
 * @param in
 * @param out
 */
void PclGroundCore::remove_close_pt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in,
                                  const pcl::PointCloud<pcl::PointXYZI>::Ptr out)
{
    pcl::ExtractIndices<pcl::PointXYZI> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
#pragma omp for
    for (size_t i = 0; i < in->points.size(); i++)
    {
//        计算点到雷达发射器的距离
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if (distance < min_distance)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(boost::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true); //ture to remove the indices
    cliper.filter(*out);
}

/*!
 * 重新组织计算后的点云
 * @param[in] in_cloud Input Point Cloud to be organized in radial segments
 * @param[out] out_organized_points Custom Point Cloud filled with XYZRTZColor data
 * @param[out] out_radial_divided_indices Indices of the points in the original cloud for each radial segment
 * @param[out] out_radial_ordered_clouds Vector of Points Clouds, each element will contain the points ordered
 */
void PclGroundCore::XYZI_to_RTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                                   PointCloudXYZIRTColor &out_organized_points,
                                   std::vector<pcl::PointIndices> &out_radial_divided_indices,
                                   std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds)
{
    out_organized_points.resize(in_cloud->points.size());
    out_radial_divided_indices.clear();
    out_radial_divided_indices.resize(radial_dividers_num_);
    out_radial_ordered_clouds.resize(radial_dividers_num_);

    for (size_t i = 0; i < in_cloud->points.size(); i++)
    {
        PointXYZIRTColor new_point;
        auto radius = (float)sqrt(
                in_cloud->points[i].x * in_cloud->points[i].x + in_cloud->points[i].y * in_cloud->points[i].y);
        auto theta = (float)atan2(in_cloud->points[i].y, in_cloud->points[i].x) * 180 / M_PI;
        if (theta < 0)
        {
            theta += 360;
        }
        //角度的微分
        auto radial_div = (size_t)floor(theta / RADIAL_DIVIDER_ANGLE);
        //半径的微分
        auto concentric_div = (size_t)floor(fabs(radius / concentric_divider_distance_));

        new_point.point = in_cloud->points[i];
        new_point.radius = radius;
        new_point.theta = theta;
        new_point.radial_div = radial_div;
        new_point.concentric_div = concentric_div;
        new_point.original_index = i;

        out_organized_points[i] = new_point;

        //radial divisions更加角度的微分组织射线
        out_radial_divided_indices[radial_div].indices.push_back(i);

        out_radial_ordered_clouds[radial_div].push_back(new_point);

    } //end for

    //将同一根射线上的点按照半径（距离）排序
#pragma omp for
    for (size_t i = 0; i < radial_dividers_num_; i++)
    {
        std::sort(out_radial_ordered_clouds[i].begin(), out_radial_ordered_clouds[i].end(),
                  [](const PointXYZIRTColor &a, const PointXYZIRTColor &b) { return a.radius < b.radius; });
    }
}

/*!
 * 分割地面和非地面
 * @param in_radial_ordered_clouds Vector of an Ordered PointsCloud ordered by radial distance from the origin
 * @param out_ground_indices Returns the indices of the points classified as ground in the original PointCloud
 * @param out_no_ground_indices Returns the indices of the points classified as not ground in the original PointCloud
 */
void PclGroundCore::classify_pc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                              pcl::PointIndices &out_ground_indices,
                              pcl::PointIndices &out_no_ground_indices)
{
    out_ground_indices.indices.clear();
    out_no_ground_indices.indices.clear();

#pragma omp for
    for (size_t i = 0; i < in_radial_ordered_clouds.size(); i++) //sweep through each radial division 遍历每一根射线
    {
        float prev_radius = 0.f;
        float prev_height = -SENSOR_HEIGHT; //水平地面
        bool prev_ground = false;
        bool current_ground = false;
        for (size_t j = 0; j < in_radial_ordered_clouds[i].size(); j++) //loop through each point in the radial div
        {
            float points_distance = in_radial_ordered_clouds[i][j].radius - prev_radius;
            float height_threshold = tan(DEG2RAD(local_max_slope_)) * points_distance; //local_max_slope_同一条射线上相邻两点的坡度阀值
            float current_height = in_radial_ordered_clouds[i][j].point.z; //general_max_slope_整个地面的坡度阀值
            float general_height_threshold = tan(DEG2RAD(general_max_slope_)) * in_radial_ordered_clouds[i][j].radius;//高度阀值

            //for points which are very close causing the height threshold to be tiny, set a minimum value
            if (points_distance > concentric_divider_distance_ && height_threshold < min_height_threshold_)
            {
                height_threshold = min_height_threshold_;
            }

            //check current point height against the LOCAL threshold (previous point)
            if (current_height <= (prev_height + height_threshold) && current_height >= (prev_height - height_threshold))
            {
                //Check again using general geometry (radius from origin) if previous points wasn't ground
                if (!prev_ground)
                {
//                    当前点的高度是否在地面加减高度阀值范围内 判断当前点是否为地面
                    if (current_height <= (-SENSOR_HEIGHT + general_height_threshold) && current_height >= (-SENSOR_HEIGHT - general_height_threshold))
                    {
                        current_ground = true;
                    }
                    else
                    {
                        current_ground = false;
                    }
                }
                else
                {
                    current_ground = true;
                }
            }
            else
            {
                //check if previous point is too far from previous one, if so classify again
                if (points_distance > reclass_distance_threshold_ &&
                    (current_height <= (-SENSOR_HEIGHT + height_threshold) && current_height >= (-SENSOR_HEIGHT - height_threshold)))
                {
                    current_ground = true;
                }
                else
                {
                    current_ground = false;
                }
            }

            if (current_ground)
            {
                out_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = true;
            }
            else
            {
                out_no_ground_indices.indices.push_back(in_radial_ordered_clouds[i][j].original_index);
                prev_ground = false;
            }

            prev_radius = in_radial_ordered_clouds[i][j].radius;
            prev_height = in_radial_ordered_clouds[i][j].point.z;
        }
    }
}

/**
 *
 * @param in_publisher
 * @param in_cloud_to_publish_ptr
 * @param in_header
 */
void PclGroundCore::publish_cloud(const ros::Publisher &in_publisher,
                                const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud_to_publish_ptr,
                                const std_msgs::Header &in_header)
{
    sensor_msgs::PointCloud2 cloud_msg;
//    转换成PointCloud2的格式发布
    pcl::toROSMsg(*in_cloud_to_publish_ptr, cloud_msg);
    cloud_msg.header = in_header;//时间戳一致
    in_publisher.publish(cloud_msg);
}

